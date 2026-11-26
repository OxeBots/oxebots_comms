#include "oxebots_comms/Nrf24.hpp"
#include "oxebots_comms/kick_scaling.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

const std::string C_RED    = "\033[91m";
const std::string C_GREEN  = "\033[92m";
const std::string C_YELLOW = "\033[93m";
const std::string C_BLUE   = "\033[94m";
const std::string C_CYAN   = "\033[96m";
const std::string C_BOLD   = "\033[1m";
const std::string C_RESET  = "\033[0m";

constexpr int TX_TIMER_PERIOD_MS        = 10;    // ~100 Hz TX loop
constexpr int TELEMETRY_TIMEOUT_MS      = 20;    // ESP32 Base RTT is ~2-3 ms
constexpr int CLI_TELEMETRY_TIMEOUT_MS  = 80;
constexpr int OFFLINE_PROBE_INTERVAL_MS = 1000;  // 1 Hz hot-plug probe for offline robots

constexpr std::array<uint8_t, 4> DEFAULT_ADDR_SUFFIX = {0xC6, 0x9A, 0xD3, 0xE5};
constexpr uint8_t DEFAULT_BASE_ADDR_LSB = 0xB4;

std::string g_override_port = "";
int g_override_baud = -1;
int g_override_channel = -1;
int g_override_rate = -1;
int g_override_pa = -1;
bool g_skip_config = false;

inline uint32_t now_ms_u32()
{
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count() &
        0xFFFFFFFFULL);
}

inline double safe_finite(double v, double fallback = 0.0)
{
    return std::isfinite(v) ? v : fallback;
}

inline int16_t clamp_i16(double v)
{
    const double clean = safe_finite(v, 0.0);
    const double clamped = std::clamp(
        clean,
        static_cast<double>(std::numeric_limits<int16_t>::min()),
        static_cast<double>(std::numeric_limits<int16_t>::max()));
    return static_cast<int16_t>(std::llround(clamped));
}

const char* rate_to_str(uint8_t rate)
{
    switch (rate) {
        case BS_RATE_1MBPS:
            return "1 Mbps";
        case BS_RATE_2MBPS:
            return "2 Mbps";
        case BS_RATE_250KBPS:
            return "250 Kbps";
        default:
            return "?";
    }
}

const char* pa_to_str(uint8_t pa)
{
    switch (pa) {
        case 0:
            return "PA_MIN (-18 dBm)";
        case 1:
            return "PA_LOW (-12 dBm)";
        case 2:
            return "PA_HIGH (-6 dBm)";
        case 3:
            return "PA_MAX (0 dBm)";
        default:
            return "?";
    }
}

uint8_t parse_robot_id_token(std::string token)
{
    std::transform(token.begin(), token.end(), token.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (token == "broadcast" || token == "all" || token == "bcast") {
        return ROBOT_ID_BROADCAST;
    }
    int id = std::stoi(token);
    if (id == ROBOT_ID_BROADCAST) return ROBOT_ID_BROADCAST;
    return static_cast<uint8_t>(std::clamp(id, 0, static_cast<int>(ROBOT_ID_MAX)));
}

bool send_robot_command(
    oxebots_comms::BaseStationClient& client,
    const struct RobotCommand& cmd,
    struct BaseRfTxStatus* out_status = nullptr)
{
    uint8_t tx_buf[BYTES_LENGTH_ROBOT_COMMAND] = {0};
    struct RobotCommand copy = cmd;
    EncodeRobotCommand(&copy, tx_buf);
    const uint8_t flags = (cmd.header.robot_id == ROBOT_ID_BROADCAST) ? BS_TX_FLAG_NO_ACK : 0;
    return client.send_rf_packet(
        cmd.header.robot_id, tx_buf, BYTES_LENGTH_ROBOT_COMMAND, flags, out_status, 25);
}

bool send_robot_config(
    oxebots_comms::BaseStationClient& client,
    uint8_t robot_id,
    uint8_t config_flags,
    uint16_t param = 0,
    struct BaseRfTxStatus* out_status = nullptr)
{
    struct RobotConfig cfg = {};
    cfg.header.msg_type = MSG_TYPE_CONFIG;
    cfg.header.robot_id = robot_id;
    cfg.header.timestamp = now_ms_u32();
    cfg.config_flags = config_flags;
    cfg.param = param;

    uint8_t buf[BYTES_LENGTH_ROBOT_CONFIG] = {0};
    EncodeRobotConfig(&cfg, buf);
    const uint8_t flags = (robot_id == ROBOT_ID_BROADCAST) ? BS_TX_FLAG_NO_ACK : 0;
    return client.send_rf_packet(robot_id, buf, BYTES_LENGTH_ROBOT_CONFIG, flags, out_status, 30);
}

bool send_telemetry_request_and_wait(
    oxebots_comms::BaseStationClient& client,
    uint8_t robot_id,
    struct RobotTelemetry* out_tel,
    double* out_rtt_ms,
    struct BaseRfTxStatus* out_tx_status = nullptr,
    int timeout_ms = TELEMETRY_TIMEOUT_MS)
{
    if (robot_id == ROBOT_ID_BROADCAST) return false;

    struct RobotTelemetryRequest req = {};
    req.header.msg_type = MSG_TYPE_TELEMETRY_REQ;
    req.header.robot_id = robot_id;
    req.header.timestamp = now_ms_u32();

    uint8_t tx_buf[BYTES_LENGTH_ROBOT_TELEMETRY_REQUEST] = {0};
    EncodeRobotTelemetryRequest(&req, tx_buf);

    client.clear_rx_queue();
    const auto t_start = std::chrono::steady_clock::now();

    struct BaseRfTxStatus tx_st = {};
    const bool sent = client.send_rf_packet(
        robot_id,
        tx_buf,
        BYTES_LENGTH_ROBOT_TELEMETRY_REQUEST,
        BS_TX_FLAG_EXPECT_REPLY,
        &tx_st,
        25);
    if (out_tx_status != nullptr) *out_tx_status = tx_st;
    if (!sent || tx_st.status != BS_TX_STATUS_OK) {
        return false;
    }

    const auto deadline = t_start + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        const int rem_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now())
                .count());
        if (rem_ms <= 0) break;

        struct BaseRfRx rx = {};
        if (!client.wait_rf_rx(&rx, rem_ms)) {
            break;
        }
        if (rx.payload_len < BYTES_LENGTH_ROBOT_TELEMETRY) {
            continue;
        }
        if ((rx.payload[0] & 0x0FU) != MSG_TYPE_TELEMETRY) {
            continue;
        }
        struct RobotTelemetry tel = {};
        DecodeRobotTelemetry(&tel, rx.payload);
        if (tel.header.robot_id == robot_id) {
            const auto t_end = std::chrono::steady_clock::now();
            if (out_tel != nullptr) *out_tel = tel;
            if (out_rtt_ms != nullptr) {
                *out_rtt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            }
            return true;
        }
    }
    return false;
}

void print_base_info(const struct BaseInfo& info, const std::string& port, int baud)
{
    std::cout << "\n"
              << C_BOLD << std::string(68, '=') << "\n"
              << " ESP32 BASE STATION (nRF24L01+ | " << port << " @ " << baud << " baud)\n"
              << std::string(68, '=') << C_RESET << "\n"
              << "  Protocolo       : v" << static_cast<int>(info.protocol_version) << "\n"
              << "  Rádio           : "
              << (info.radio_ready ? (C_GREEN + "PRONTO" + C_RESET) : (C_RED + "ERRO" + C_RESET))
              << "\n"
              << "  Canal RF        : " << static_cast<int>(info.channel) << " ("
              << (2400 + static_cast<int>(info.channel)) << " MHz)\n"
              << "  Taxa / Potência : " << rate_to_str(info.data_rate) << " | "
              << pa_to_str(info.pa_level) << "\n"
              << "  Retransmissão   : ARD=" << static_cast<int>(info.retransmit_delay) << " ("
              << ((static_cast<int>(info.retransmit_delay) + 1) * 250)
              << " us) | ARC=" << static_cast<int>(info.retransmit_count) << "\n"
              << "  Contadores      : TX OK=" << info.tx_ok_count
              << " | TX Fail=" << info.tx_fail_count << " | RX=" << info.rx_count
              << " | Watchdog Reinits=" << info.radio_reinit_count << "\n"
              << "  Endereço Base   : LSB=0x" << std::hex << std::uppercase << std::setw(2)
              << std::setfill('0') << static_cast<int>(info.base_addr_lsb) << " + {";
    for (int i = 0; i < 4; ++i) {
        std::cout << "0x" << std::setw(2) << static_cast<int>(info.addr_suffix[i])
                  << (i < 3 ? "," : "");
    }
    std::cout << "}" << std::dec << std::setfill(' ') << "\n"
              << std::string(68, '=') << "\n"
              << std::endl;
}

void print_scan_result(const struct BaseScanResult& scan)
{
    std::cout << "\n"
              << C_BOLD << "=== SCAN DE ESPECTRO RF (RPD >= -64 dBm) ===" << C_RESET << "\n"
              << "  Faixa escaneada : canais " << static_cast<int>(scan.start_channel) << ".."
              << (static_cast<int>(scan.start_channel) + static_cast<int>(scan.count) - 1) << "\n"
              << "  Melhor canal    : " << C_GREEN << static_cast<int>(scan.best_channel) << " ("
              << (2400 + static_cast<int>(scan.best_channel)) << " MHz)" << C_RESET << "\n\n";

    for (uint8_t i = 0; i < scan.count; ++i) {
        const int ch = static_cast<int>(scan.start_channel) + i;
        const int pct = scan.busy_pct[i];
        if (pct == 0 && ch != scan.best_channel && (ch % 10 != 0)) {
            continue;
        }
        const int bars = pct / 5;
        const std::string& color = (pct >= 50) ? C_RED : (pct >= 15 ? C_YELLOW : C_GREEN);
        std::cout << "  ch " << std::setw(3) << ch << " (" << (2400 + ch) << " MHz) ["
                  << std::setw(3) << pct << "%] " << color << std::string(bars, '#') << C_RESET
                  << ((ch == scan.best_channel) ? (" " + C_CYAN + "<-- RECOMENDADO" + C_RESET) : "")
                  << "\n";
    }
    std::cout << std::endl;
}

bool switch_channel_coordinated(
    oxebots_comms::BaseStationClient& client, uint8_t new_channel, bool notify_robots)
{
    if (new_channel > 125) {
        std::cout << C_RED << "Canal inválido (use 0..125)." << C_RESET << std::endl;
        return false;
    }

    struct BaseInfo info = {};
    if (!client.get_info(&info)) {
        std::cout << C_RED << "Não foi possível ler info atual da Base Station." << C_RESET
                  << std::endl;
        return false;
    }

    if (notify_robots) {
        std::cout << C_CYAN << "Notificando robôs para migrar do canal "
                  << static_cast<int>(info.channel) << " para o canal "
                  << static_cast<int>(new_channel) << "..." << C_RESET << std::endl;
        for (uint8_t id = 0; id <= ROBOT_ID_MAX; ++id) {
            struct BaseRfTxStatus st = {};
            if (send_robot_config(client, id, CONFIG_FLAG_SET_CHANNEL, new_channel, &st) &&
                st.status == BS_TX_STATUS_OK) {
                std::cout << C_GREEN << "  ✓ Robô " << static_cast<int>(id)
                          << " confirmou troca (ACK em " << st.tx_time_us << " us)" << C_RESET
                          << "\n";
            }
        }
        // Extra broadcast copies for any robot not yet assigned/answering unicast.
        for (int i = 0; i < 3; ++i) {
            send_robot_config(client, ROBOT_ID_BROADCAST, CONFIG_FLAG_SET_CHANNEL, new_channel);
        }
    }

    struct BaseRadioConfig cfg = {};
    cfg.channel = new_channel;
    cfg.data_rate = info.data_rate;
    cfg.pa_level = info.pa_level;
    cfg.retransmit_delay = info.retransmit_delay;
    cfg.retransmit_count = info.retransmit_count;
    cfg.persist_nvs = 1;
    std::memcpy(cfg.addr_suffix, info.addr_suffix, 4);
    cfg.base_addr_lsb = info.base_addr_lsb;

    struct BaseInfo updated = {};
    if (!client.set_config(cfg, &updated)) {
        std::cout << C_RED << "Falha ao aplicar novo canal na Base Station." << C_RESET
                  << std::endl;
        return false;
    }
    std::cout << C_GREEN << "✓ Base Station migrada para o canal "
              << static_cast<int>(updated.channel) << " (" << (2400 + updated.channel)
              << " MHz, salvo em NVS)." << C_RESET << std::endl;
    return true;
}

std::vector<uint8_t> parse_benchmark_targets(const std::string& spec)
{
    std::vector<uint8_t> ids;
    if (spec.empty()) return {0, 1, 2};

    std::string s = spec;
    if (s.rfind("id=", 0) == 0) {
        s = s.substr(3);
    }
    if (s.find(',') != std::string::npos) {
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, ',')) {
            if (!item.empty()) {
                uint8_t id = parse_robot_id_token(item);
                if (id <= ROBOT_ID_MAX && std::find(ids.begin(), ids.end(), id) == ids.end()) {
                    ids.push_back(id);
                }
            }
        }
        return ids;
    }

    int count = std::stoi(s);
    if (count <= 0) count = 1;
    if (count > static_cast<int>(ROBOT_ID_MAX) + 1) count = ROBOT_ID_MAX + 1;
    for (int i = 0; i < count; ++i) {
        ids.push_back(static_cast<uint8_t>(i));
    }
    return ids;
}

struct BenchStats
{
    int cmd_sent = 0;
    int cmd_ack_ok = 0;
    int cmd_retries_sum = 0;
    double tx_us_sum = 0.0;
    int tel_req_sent = 0;
    int tel_received = 0;
    int consecutive_losses = 0;
    bool unreachable = false;
    std::chrono::steady_clock::time_point last_tel_attempt{};
    std::vector<double> rtts_ms;
    uint8_t last_battery = 0;
    double last_kicker_v = 0.0;
};

void run_benchmark(
    oxebots_comms::BaseStationClient& client,
    const std::string& target_spec,
    double duration_s,
    int tel_interval)
{
    std::vector<uint8_t> robot_ids = parse_benchmark_targets(target_spec);
    if (robot_ids.empty()) robot_ids = {0};

    std::set<uint8_t> online;
    std::cout << C_CYAN << "Sondando robôs online (IDs 0.." << static_cast<int>(ROBOT_ID_MAX)
              << ")..." << C_RESET << std::flush;
    for (uint8_t id = 0; id <= ROBOT_ID_MAX; ++id) {
        struct RobotTelemetry tel = {};
        double rtt = 0.0;
        if (send_telemetry_request_and_wait(client, id, &tel, &rtt, nullptr, 30)) {
            online.insert(id);
        }
    }
    if (!online.empty()) {
        std::cout << " " << C_GREEN << "✓ Online: [";
        bool first = true;
        for (uint8_t id : online) {
            if (!first) std::cout << ", ";
            std::cout << static_cast<int>(id);
            first = false;
        }
        std::cout << "]" << C_RESET << "\n";
    } else {
        std::cout << " " << C_YELLOW << "(nenhum respondeu agora)" << C_RESET << "\n";
    }

    const auto start = std::chrono::steady_clock::now();
    std::map<uint8_t, BenchStats> stats;
    for (uint8_t rid : robot_ids) {
        BenchStats st{};
        st.unreachable = (online.count(rid) == 0);
        st.last_tel_attempt = start;
        stats[rid] = st;
    }

    std::cout << "\n"
              << C_BOLD << std::string(90, '=') << "\n"
              << " BENCHMARK ESP32 BASE STATION (Duração: " << duration_s
              << "s | Telemetria a cada " << tel_interval << " cmds)\n"
              << std::string(90, '=') << C_RESET << "\n\n";

    int cycles = 0;
    auto last_print = start;
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_s = std::chrono::duration<double>(now - start).count();
        if (elapsed_s >= duration_s) break;

        cycles++;
        for (size_t idx = 0; idx < robot_ids.size(); ++idx) {
            const uint8_t rid = robot_ids[idx];
            auto& st = stats[rid];

            struct RobotCommand cmd = {};
            cmd.header.msg_type = MSG_TYPE_COMMAND;
            cmd.header.robot_id = rid;
            cmd.header.timestamp = now_ms_u32();

            struct BaseRfTxStatus tx_st = {};
            if (send_robot_command(client, cmd, &tx_st)) {
                st.cmd_sent++;
                if (tx_st.status == BS_TX_STATUS_OK) {
                    st.cmd_ack_ok++;
                    st.cmd_retries_sum += tx_st.retries;
                    st.tx_us_sum += tx_st.tx_time_us;
                }
            }

            if (tel_interval <= 0) continue;

            const auto step_now = std::chrono::steady_clock::now();
            bool do_tel = false;
            if (!st.unreachable) {
                do_tel = ((st.cmd_sent + static_cast<int>(idx)) % tel_interval == 0);
            } else {
                const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    step_now - st.last_tel_attempt)
                                    .count();
                do_tel = (ms >= OFFLINE_PROBE_INTERVAL_MS);
            }
            if (!do_tel) continue;

            st.last_tel_attempt = step_now;
            struct RobotTelemetry tel = {};
            double rtt_ms = 0.0;
            st.tel_req_sent++;
            if (send_telemetry_request_and_wait(
                    client, rid, &tel, &rtt_ms, nullptr, TELEMETRY_TIMEOUT_MS)) {
                st.tel_received++;
                st.consecutive_losses = 0;
                st.unreachable = false;
                st.rtts_ms.push_back(rtt_ms);
                st.last_battery = tel.battery_percentage;
                st.last_kicker_v = tel.kicker_voltage / 100.0;
            } else {
                st.consecutive_losses++;
                if (st.consecutive_losses >= 2) st.unreachable = true;
            }
        }

        const auto now_p = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now_p - last_print).count() >= 0.5) {
            last_print = now_p;
            const double dt = std::chrono::duration<double>(now_p - start).count();
            int total_cmds = 0;
            for (const auto& kv : stats) total_cmds += kv.second.cmd_sent;
            std::cout << "\r[" << std::fixed << std::setprecision(1) << dt << "s] Cmds: "
                      << std::setw(6) << (total_cmds / std::max(dt, 0.001)) << " Hz ("
                      << std::setw(5)
                      << (total_cmds / std::max(dt, 0.001) / robot_ids.size()) << " Hz/robô) |";
            for (uint8_t rid : robot_ids) {
                const auto& st = stats[rid];
                const double ack_pct =
                    st.cmd_sent > 0 ? (100.0 * st.cmd_ack_ok / st.cmd_sent) : 0.0;
                std::cout << " R" << static_cast<int>(rid) << ":" << std::setprecision(0) << ack_pct
                          << "%ACK";
            }
            std::cout << "   " << std::flush;
        }
    }

    const double total_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    int total_cmds = 0;
    for (const auto& kv : stats) total_cmds += kv.second.cmd_sent;

    std::cout << "\n\n"
              << C_BOLD << std::string(96, '=') << "\n"
              << " RELATÓRIO FINAL (" << std::fixed << std::setprecision(2) << total_s
              << "s | Ciclos: " << cycles << " | Total: " << (total_cmds / std::max(total_s, 0.001))
              << " Hz)\n"
              << std::string(96, '=') << C_RESET << "\n"
              << " Robô |  Cmds | TX ACK% | Retries | TX Avg  | Tel RX/Req | RTT Médio | Bateria\n"
              << std::string(96, '-') << "\n";

    for (uint8_t rid : robot_ids) {
        const auto& st = stats[rid];
        const double ack_pct = st.cmd_sent > 0 ? (100.0 * st.cmd_ack_ok / st.cmd_sent) : 0.0;
        const double avg_ret =
            st.cmd_ack_ok > 0 ? (static_cast<double>(st.cmd_retries_sum) / st.cmd_ack_ok) : 0.0;
        const double avg_tx_us = st.cmd_ack_ok > 0 ? (st.tx_us_sum / st.cmd_ack_ok) : 0.0;
        double avg_rtt = 0.0;
        if (!st.rtts_ms.empty()) {
            for (double v : st.rtts_ms) avg_rtt += v;
            avg_rtt /= st.rtts_ms.size();
        }
        std::cout << "  " << std::setw(3) << static_cast<int>(rid) << " | " << std::setw(5)
                  << st.cmd_sent << " | " << std::setw(6) << std::setprecision(1) << ack_pct
                  << "% | " << std::setw(7) << std::setprecision(2) << avg_ret << " | "
                  << std::setw(5) << std::setprecision(0) << avg_tx_us << "us | " << std::setw(4)
                  << st.tel_received << "/" << std::setw(4) << st.tel_req_sent << "  | ";
        if (!st.rtts_ms.empty()) {
            std::cout << std::setw(7) << std::setprecision(2) << avg_rtt << "ms | "
                      << static_cast<int>(st.last_battery) << "% (" << std::setprecision(1)
                      << st.last_kicker_v << "V)\n";
        } else {
            std::cout << "    N/A   | OFFLINE\n";
        }
    }
    std::cout << std::string(96, '=') << "\n" << std::endl;
}

void print_cli_help()
{
    std::cout << "\n"
              << C_BOLD << "=== COMANDOS DO MODO INTERATIVO (ESP32 BASE STATION) ===" << C_RESET
              << "\n"
              << "  {x:<mm>, y:<mm>, vx:<mm/s>, vy:<mm/s>, w:<0.01rad/s>, kick:<0-255>, id:<id>}\n"
              << "  tel <robot_id>                                     -> Solicita telemetria unicast\n"
              << "  bench [robos|id=0|0,1,2] [duracao_s] [tel_int]     -> Benchmark TX ACK% + Telemetria RTT\n"
              << "  info                                               -> Exibe estado/contadores da Base Station\n"
              << "  scan [start_ch end_ch samples]                     -> Varredura RPD de espectro (0..125)\n"
              << "  channel <0..125> [base_only]                       -> Migra robôs + Base para novo canal (NVS)\n"
              << "  auto_channel                                       -> Escaneia e migra para o melhor canal\n"
              << "  set_id <target_id|broadcast> <new_id>              -> Altera ID do robô (e atualiza endereço RF)\n"
              << "  calibrate [<robot_id>|broadcast]                   -> Calibração de rodas/PID\n"
              << "  calibrate_mag [<robot_id>|broadcast]               -> Calibração do magnetômetro\n"
              << "  reset_nvs [<robot_id>|broadcast]                   -> Reseta ID e canal do robô\n"
              << "  factory_reset [<robot_id>|broadcast]               -> Apaga toda NVS e reinicia robô\n"
              << "  help / q / quit / exit\n"
              << std::endl;
}

void run_interactive_cli(oxebots_comms::BaseStationClient& client)
{
    std::cout << C_BOLD << "\n=== TERMINAL INTERATIVO ESP32 BASE STATION ===" << C_RESET << "\n";
    print_cli_help();

    std::string line;
    while (true) {
        std::cout << C_BOLD << "Cmd > " << C_RESET << std::flush;
        if (!std::getline(std::cin, line)) break;

        size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) continue;
        std::string raw = line.substr(first, line.find_last_not_of(" \t\r\n") - first + 1);
        if (raw == "q" || raw == "quit" || raw == "exit") break;
        if (raw == "help" || raw == "?") {
            print_cli_help();
            continue;
        }

        std::vector<std::string> parts;
        {
            std::stringstream ss(raw);
            std::string tok;
            while (ss >> tok) parts.push_back(tok);
        }
        if (parts.empty()) continue;

        try {
            if (parts[0] == "info") {
                struct BaseInfo info = {};
                if (client.get_info(&info)) {
                    print_base_info(info, client.port_name(), client.baud_rate());
                } else {
                    std::cout << C_RED << "Sem resposta da Base Station." << C_RESET << std::endl;
                }
                continue;
            }
            if (parts[0] == "scan") {
                uint8_t start_ch = (parts.size() > 1) ? static_cast<uint8_t>(std::stoi(parts[1])) : 0;
                uint8_t end_ch = (parts.size() > 2) ? static_cast<uint8_t>(std::stoi(parts[2])) : 125;
                uint8_t samples = (parts.size() > 3) ? static_cast<uint8_t>(std::stoi(parts[3])) : 32;
                std::cout << C_CYAN << "Escaneando canais " << static_cast<int>(start_ch) << ".."
                          << static_cast<int>(end_ch) << " (" << static_cast<int>(samples)
                          << " amostras/canal)..." << C_RESET << std::endl;
                struct BaseScanResult scan = {};
                if (client.scan_spectrum(start_ch, end_ch, samples, &scan)) {
                    print_scan_result(scan);
                } else {
                    std::cout << C_RED << "Falha/timeout no scan de espectro." << C_RESET
                              << std::endl;
                }
                continue;
            }
            if (parts[0] == "channel") {
                if (parts.size() < 2) {
                    std::cout << C_YELLOW << "Uso: channel <0..125> [base_only]" << C_RESET
                              << std::endl;
                    continue;
                }
                const uint8_t ch = static_cast<uint8_t>(std::stoi(parts[1]));
                const bool notify = !(parts.size() > 2 && parts[2] == "base_only");
                switch_channel_coordinated(client, ch, notify);
                continue;
            }
            if (parts[0] == "auto_channel") {
                struct BaseScanResult scan = {};
                if (client.scan_spectrum(0, 125, 32, &scan) && scan.count > 0) {
                    print_scan_result(scan);
                    switch_channel_coordinated(client, scan.best_channel, true);
                } else {
                    std::cout << C_RED << "Scan falhou." << C_RESET << std::endl;
                }
                continue;
            }
            if (parts[0] == "set_id") {
                if (parts.size() < 3) {
                    std::cout << C_YELLOW << "Uso: set_id <target_id|broadcast> <new_id>"
                              << C_RESET << std::endl;
                    continue;
                }
                const uint8_t target = parse_robot_id_token(parts[1]);
                const uint16_t new_id = static_cast<uint16_t>(std::stoi(parts[2]));
                struct BaseRfTxStatus st = {};
                send_robot_config(client, target, CONFIG_FLAG_SET_ID, new_id, &st);
                std::cout << C_CYAN << "[SET_ID] target=" << static_cast<int>(target)
                          << " new_id=" << new_id << " status=" << static_cast<int>(st.status)
                          << C_RESET << std::endl;
                continue;
            }
            if (parts[0] == "calibrate" || parts[0] == "calibrate_mag" ||
                parts[0] == "reset_nvs" || parts[0] == "factory_reset") {
                uint8_t rid = (parts.size() > 1) ? parse_robot_id_token(parts[1]) : 0;
                uint8_t flag = CONFIG_FLAG_RUN_CALIBRATION;
                if (parts[0] == "calibrate_mag") flag = CONFIG_FLAG_CALIBRATE_MAG;
                else if (parts[0] == "reset_nvs") flag = CONFIG_FLAG_RESET_NVS;
                else if (parts[0] == "factory_reset") flag = CONFIG_FLAG_FACTORY_RESET;
                struct BaseRfTxStatus st = {};
                send_robot_config(client, rid, flag, 0, &st);
                std::cout << C_CYAN << "[CONFIG] robot=" << static_cast<int>(rid) << " flag=0x"
                          << std::hex << static_cast<int>(flag) << std::dec
                          << " status=" << static_cast<int>(st.status) << C_RESET << std::endl;
                continue;
            }
            if (parts[0] == "tel") {
                uint8_t rid = (parts.size() > 1) ? parse_robot_id_token(parts[1]) : 0;
                struct RobotTelemetry tel = {};
                double rtt_ms = 0.0;
                struct BaseRfTxStatus tx_st = {};
                if (send_telemetry_request_and_wait(
                        client, rid, &tel, &rtt_ms, &tx_st, CLI_TELEMETRY_TIMEOUT_MS)) {
                    std::cout << C_GREEN << "✓ [TEL] Robô " << static_cast<int>(tel.header.robot_id)
                              << " | RTT=" << std::fixed << std::setprecision(2) << rtt_ms
                              << "ms (retries=" << static_cast<int>(tx_st.retries)
                              << ") | Batt=" << static_cast<int>(tel.battery_percentage)
                              << "% | Kicker=" << (tel.kicker_voltage / 100.0)
                              << "V | Err=0x" << std::hex << tel.error_flags << std::dec
                              << C_RESET << std::endl;
                } else {
                    std::cout << C_RED << "✗ [TIMEOUT] Sem resposta do Robô "
                              << static_cast<int>(rid)
                              << " (tx_status=" << static_cast<int>(tx_st.status) << ")" << C_RESET
                              << std::endl;
                }
                continue;
            }
            if (parts[0] == "bench" || parts[0] == "benchmark") {
                std::string spec = (parts.size() > 1) ? parts[1] : "3";
                double dur = (parts.size() > 2) ? std::stod(parts[2]) : 10.0;
                int tel_int = (parts.size() > 3) ? std::stoi(parts[3]) : 5;
                run_benchmark(client, spec, dur, tel_int);
                continue;
            }

            // Motion command: {vx:300, id:0}
            if (raw.find(':') == std::string::npos) {
                std::cout << C_YELLOW << "Comando desconhecido: '" << raw
                          << "' (digite 'help' para ver os comandos)." << C_RESET << std::endl;
                continue;
            }
            int16_t x = 0, y = 0, angle = 0, vx = 0, vy = 0, w = 0;
            uint8_t kick = 0, robot_id = 0;
            std::string clean = raw;
            for (char ch : std::string("{}[]")) {
                clean.erase(std::remove(clean.begin(), clean.end(), ch), clean.end());
            }
            std::stringstream ss(clean);
            std::string pair_str;
            while (std::getline(ss, pair_str, ',')) {
                size_t colon = pair_str.find(':');
                if (colon == std::string::npos) continue;
                std::string k = pair_str.substr(0, colon);
                std::string v = pair_str.substr(colon + 1);
                auto trim = [](std::string& s) {
                    const char* ws = " \t\r\n\"'";
                    size_t b = s.find_first_not_of(ws);
                    if (b == std::string::npos) {
                        s.clear();
                        return;
                    }
                    s = s.substr(b, s.find_last_not_of(ws) - b + 1);
                };
                trim(k);
                trim(v);
                std::transform(k.begin(), k.end(), k.begin(), [](unsigned char c) {
                    return static_cast<char>(std::tolower(c));
                });
                if (k == "x") x = static_cast<int16_t>(std::stoi(v));
                else if (k == "y") y = static_cast<int16_t>(std::stoi(v));
                else if (k == "angle") angle = static_cast<int16_t>(std::stoi(v));
                else if (k == "vx") vx = static_cast<int16_t>(std::stoi(v));
                else if (k == "vy") vy = static_cast<int16_t>(std::stoi(v));
                else if (k == "w") w = static_cast<int16_t>(std::stoi(v));
                else if (k == "kick") kick = static_cast<uint8_t>(std::stoi(v));
                else if (k == "id") robot_id = parse_robot_id_token(v);
            }

            struct RobotCommand cmd = {};
            cmd.header.msg_type = MSG_TYPE_COMMAND;
            cmd.header.robot_id = robot_id;
            cmd.header.timestamp = now_ms_u32();
            cmd.target_pose.x = x;
            cmd.target_pose.y = y;
            cmd.target_pose.angle = angle;
            cmd.target_pose.x_v = vx;
            cmd.target_pose.y_v = vy;
            cmd.target_pose.angular_vel = w;
            cmd.kick_velocity = kick;

            struct BaseRfTxStatus st = {};
            const bool ok = send_robot_command(client, cmd, &st);
            std::cout << (ok && (st.status == BS_TX_STATUS_OK || st.status == BS_TX_STATUS_NO_ACK_SENT)
                              ? C_GREEN
                              : C_YELLOW)
                      << "→ [CMD] robot=" << static_cast<int>(robot_id) << " vx=" << vx
                      << " vy=" << vy << " w=" << w << " kick=" << static_cast<int>(kick)
                      << " | status=" << static_cast<int>(st.status)
                      << " retries=" << static_cast<int>(st.retries)
                      << " tx=" << st.tx_time_us << "us" << C_RESET << std::endl;
        } catch (const std::exception& e) {
            std::cout << C_RED << "Erro: " << e.what() << C_RESET << std::endl;
        }
    }
}

}  // namespace

Nrf24HardwareBridge::Nrf24HardwareBridge(bool enable_ros_loop)
    : Node("nrf24_hardware_bridge")
{
    this->declare_parameter("serial_port", std::string("auto"));
    this->declare_parameter("serial_baud", 921600);
    this->declare_parameter("radio_channel", 76);
    this->declare_parameter("radio_rate", static_cast<int>(BS_RATE_2MBPS));
    this->declare_parameter("radio_pa_level", 3);
    this->declare_parameter("radio_retransmit_delay", 1);
    this->declare_parameter("radio_retransmit_count", 3);
    this->declare_parameter("auto_select_channel", false);
    this->declare_parameter("command_mode", std::string("velocity"));
    this->declare_parameter("telemetry_request_interval", 5);
    this->declare_parameter("max_kick_speed", oxebots_comms::kDefaultMaxKickSpeedMps);

    const std::string port_spec = g_override_port.empty()
                                      ? this->get_parameter("serial_port").as_string()
                                      : g_override_port;
    const int baud =
        (g_override_baud > 0) ? g_override_baud : this->get_parameter("serial_baud").as_int();
    const std::string command_mode = this->get_parameter("command_mode").as_string();
    telemetry_request_interval_ = this->get_parameter("telemetry_request_interval").as_int();
    max_kick_speed_ = oxebots_comms::sanitize_max_kick_speed(
        this->get_parameter("max_kick_speed").as_double());

    base_client_.set_log_callback([this](uint8_t level, const std::string& text) {
        if (level <= 1) {
            RCLCPP_ERROR(this->get_logger(), "[BASE] %s", text.c_str());
        } else if (level == 2) {
            RCLCPP_WARN(this->get_logger(), "[BASE] %s", text.c_str());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "[BASE] %s", text.c_str());
        }
    });

    if (!base_client_.connect(port_spec, baud)) {
        RCLCPP_ERROR(
            this->get_logger(),
            "ESP32 Base Station não encontrada (port='%s', baud=%d): %s",
            port_spec.c_str(),
            baud,
            base_client_.last_error().c_str());
        throw std::runtime_error("ESP32 Base Station ausente (" + base_client_.last_error() + ")");
    }

    RCLCPP_INFO(
        this->get_logger(),
        "%sESP32 Base Station conectada em %s @ %d baud.%s",
        C_GREEN.c_str(),
        base_client_.port_name().c_str(),
        base_client_.baud_rate(),
        C_RESET.c_str());

    if (!g_skip_config) {
        configure_base_station();
    }

    if (enable_ros_loop) {
        telemetry_pub_ =
            this->create_publisher<oxebots_interfaces::msg::RobotTelemetry>("/robot_telemetry", 10);

        game_data_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
            "game_data",
            rclcpp::SensorDataQoS(),
            std::bind(&Nrf24HardwareBridge::game_data_callback, this, std::placeholders::_1));

        calibrate_robot_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/nrf24/calibrate_robot",
            10,
            std::bind(&Nrf24HardwareBridge::calibrate_robot_callback, this, std::placeholders::_1));

        if (command_mode == "point") {
            trajectory_setpoint_sub_ =
                this->create_subscription<oxebots_interfaces::msg::RobotTrajectorySetpoint>(
                    "/robot_trajectory_setpoints",
                    rclcpp::SensorDataQoS(),
                    std::bind(
                        &Nrf24HardwareBridge::send_trajectory_setpoint,
                        this,
                        std::placeholders::_1));
        } else {
            command_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotCmd>(
                "/robot_commands",
                rclcpp::SensorDataQoS(),
                std::bind(&Nrf24HardwareBridge::send_command, this, std::placeholders::_1));
        }

        tx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(TX_TIMER_PERIOD_MS),
            std::bind(&Nrf24HardwareBridge::tx_timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "%sPonte ROS 2 -> ESP32 Base Station pronta! Modo: '%s' | Telemetria a cada %d cmds%s",
            C_GREEN.c_str(),
            command_mode.c_str(),
            telemetry_request_interval_,
            C_RESET.c_str());
    }
}

Nrf24HardwareBridge::~Nrf24HardwareBridge()
{
    base_client_.disconnect();
}

void Nrf24HardwareBridge::configure_base_station()
{
    const int ch = (g_override_channel >= 0)
                       ? g_override_channel
                       : this->get_parameter("radio_channel").as_int();
    const int rate =
        (g_override_rate >= 0) ? g_override_rate : this->get_parameter("radio_rate").as_int();
    const int pa =
        (g_override_pa >= 0) ? g_override_pa : this->get_parameter("radio_pa_level").as_int();
    const int ard = this->get_parameter("radio_retransmit_delay").as_int();
    const int arc = this->get_parameter("radio_retransmit_count").as_int();
    const bool auto_select = this->get_parameter("auto_select_channel").as_bool();

    struct BaseRadioConfig cfg = {};
    cfg.channel = static_cast<uint8_t>(std::clamp(ch, 0, 125));
    cfg.data_rate = static_cast<uint8_t>(std::clamp(rate, 0, 2));
    cfg.pa_level = static_cast<uint8_t>(std::clamp(pa, 0, 3));
    cfg.retransmit_delay = static_cast<uint8_t>(std::clamp(ard, 0, 15));
    cfg.retransmit_count = static_cast<uint8_t>(std::clamp(arc, 0, 15));
    cfg.persist_nvs = 0;
    std::memcpy(cfg.addr_suffix, DEFAULT_ADDR_SUFFIX.data(), 4);
    cfg.base_addr_lsb = DEFAULT_BASE_ADDR_LSB;

    struct BaseInfo info = {};
    if (base_client_.set_config(cfg, &info)) {
        RCLCPP_INFO(
            this->get_logger(),
            "Base Station configurada (ch=%u, %s, %s, ARD=%u, ARC=%u, radio_ready=%u).",
            info.channel,
            rate_to_str(info.data_rate),
            pa_to_str(info.pa_level),
            info.retransmit_delay,
            info.retransmit_count,
            info.radio_ready);
    } else {
        RCLCPP_WARN(this->get_logger(), "Falha ao aplicar configuração inicial na Base Station.");
    }

    if (auto_select) {
        struct BaseScanResult scan = {};
        if (base_client_.scan_spectrum(0, 125, 24, &scan) && scan.count > 0 &&
            scan.best_channel != info.channel) {
            RCLCPP_INFO(
                this->get_logger(),
                "Auto-canal selecionou canal %u (%u MHz); migrando robôs + Base...",
                scan.best_channel,
                2400U + scan.best_channel);
            switch_channel_coordinated(base_client_, scan.best_channel, true);
        }
    }
}

void Nrf24HardwareBridge::game_data_callback(
    const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
    for (const auto& robot : msg->robots.allies) {
        if (std::isfinite(robot.orientation)) {
            robot_orientations_[robot.id] = robot.orientation;
        }
    }
}

void Nrf24HardwareBridge::calibrate_robot_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (!msg) return;
    pending_calibrations_.push_back(msg->data);
    RCLCPP_INFO(
        this->get_logger(),
        "[CALIB] Requisição de calibração enfileirada para robô %u.",
        static_cast<unsigned>(msg->data));
}

struct Pose Nrf24HardwareBridge::kinematic_pose_to_bitproto(
    const oxebots_interfaces::msg::RobotKinematicPose& pose)
{
    struct Pose p = {};
    p.x = clamp_i16(pose.x);
    p.y = clamp_i16(pose.y);
    p.angle = clamp_i16(safe_finite(pose.angle) * (180.0 / M_PI) * 100.0);
    p.x_v = clamp_i16(pose.x_v);
    p.y_v = clamp_i16(pose.y_v);
    p.angular_vel = clamp_i16(safe_finite(pose.angular_vel) * 100.0);
    return p;
}

void Nrf24HardwareBridge::send_command(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg)
{
    const auto now = std::chrono::steady_clock::now();
    for (const auto& robot_ros : msg->robots) {
        const double vx_global = safe_finite(robot_ros.x_velocity);
        const double vy_global = safe_finite(robot_ros.y_velocity);
        const double w = safe_finite(robot_ros.angular_velocity);
        const double kick = safe_finite(robot_ros.kick_speed);

        const double theta = robot_orientations_[robot_ros.id];
        const double v_forward = vx_global * std::cos(theta) + vy_global * std::sin(theta);
        const double v_left = -vx_global * std::sin(theta) + vy_global * std::cos(theta);

        struct RobotCommand cmd = {};
        cmd.header.msg_type = MSG_TYPE_COMMAND;
        cmd.header.robot_id = robot_ros.id;
        cmd.header.timestamp =
            static_cast<uint32_t>((this->now().nanoseconds() / 1000000ULL) & 0xFFFFFFFFULL);
        cmd.target_pose.x_v = clamp_i16(v_forward * 1000.0);
        cmd.target_pose.y_v = clamp_i16(v_left * 1000.0);
        cmd.target_pose.angular_vel = -clamp_i16(w * 100.0);
        cmd.kick_velocity = oxebots_comms::kick_speed_to_u8(kick, max_kick_speed_);

        auto& slot = pending_commands_[robot_ros.id];
        slot.cmd = cmd;
        slot.updated_at = now;
        slot.pending = true;
    }
}

void Nrf24HardwareBridge::send_trajectory_setpoint(
    const oxebots_interfaces::msg::RobotTrajectorySetpoint::SharedPtr msg)
{
    const auto now = std::chrono::steady_clock::now();
    for (const auto& robot_ros : msg->robots) {
        struct RobotCommand cmd = {};
        cmd.header.msg_type = MSG_TYPE_COMMAND;
        cmd.header.robot_id = robot_ros.id;
        cmd.header.timestamp =
            static_cast<uint32_t>((this->now().nanoseconds() / 1000000ULL) & 0xFFFFFFFFULL);
        cmd.target_pose = kinematic_pose_to_bitproto(robot_ros.target);
        cmd.vision_pose = kinematic_pose_to_bitproto(robot_ros.vision);
        cmd.kick_velocity = oxebots_comms::kick_speed_to_u8(robot_ros.kick_speed, max_kick_speed_);

        auto& slot = pending_commands_[robot_ros.id];
        slot.cmd = cmd;
        slot.updated_at = now;
        slot.pending = true;
    }
}

void Nrf24HardwareBridge::tx_timer_callback()
{
    if (!base_client_.is_connected()) return;

    if (!pending_calibrations_.empty()) {
        auto ids = std::move(pending_calibrations_);
        pending_calibrations_.clear();
        for (uint8_t rid : ids) {
            struct BaseRfTxStatus st = {};
            if (send_robot_config(base_client_, rid, CONFIG_FLAG_RUN_CALIBRATION, 0, &st)) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[CALIB] CONFIG_FLAG_RUN_CALIBRATION enviado ao robô %u (status=%u).",
                    rid,
                    st.status);
            }
        }
    }

    if (pending_commands_.empty()) return;

    const auto now = std::chrono::steady_clock::now();
    std::vector<uint8_t> sent_ids;

    for (auto& [robot_id, entry] : pending_commands_) {
        if (!entry.pending) continue;
        entry.pending = false;

        entry.cmd.header.timestamp =
            static_cast<uint32_t>((this->now().nanoseconds() / 1000000ULL) & 0xFFFFFFFFULL);

        struct BaseRfTxStatus tx_st = {};
        if (transmit_command(entry.cmd, &tx_st)) {
            sent_ids.push_back(robot_id);
            auto& link = link_states_[robot_id];
            if (tx_st.status == BS_TX_STATUS_OK || tx_st.status == BS_TX_STATUS_NO_ACK_SENT) {
                link.consecutive_tx_fails = 0;
            } else {
                link.consecutive_tx_fails++;
            }
        }
    }

    if (telemetry_request_interval_ <= 0) return;

    for (uint8_t robot_id : sent_ids) {
        if (robot_id == ROBOT_ID_BROADCAST) continue;

        auto [it, inserted] = link_states_.try_emplace(robot_id);
        auto& st = it->second;
        if (inserted) {
            st.cmd_since_last_tel = robot_id % telemetry_request_interval_;
        }

        st.cmd_since_last_tel++;
        if (st.cmd_since_last_tel < telemetry_request_interval_) continue;
        st.cmd_since_last_tel = 0;

        if (!st.is_online) {
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                now - st.last_tel_attempt)
                                .count();
            if (ms < OFFLINE_PROBE_INTERVAL_MS) continue;
        }

        st.last_tel_attempt = now;
        if (request_and_receive_telemetry(robot_id)) {
            if (!st.is_online) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "%s[RADIO] Robô %d voltou a responder telemetria!%s",
                    C_GREEN.c_str(),
                    robot_id,
                    C_RESET.c_str());
            }
            st.is_online = true;
            st.consecutive_tel_losses = 0;
        } else {
            st.consecutive_tel_losses++;
            if (st.consecutive_tel_losses >= 2 && st.is_online) {
                st.is_online = false;
                RCLCPP_WARN(
                    this->get_logger(),
                    "%s[RADIO] Robô %d sem resposta de telemetria (sondagem a 1 Hz).%s",
                    C_YELLOW.c_str(),
                    robot_id,
                    C_RESET.c_str());
            }
        }
    }
}

bool Nrf24HardwareBridge::transmit_command(
    const struct RobotCommand& cmd, struct BaseRfTxStatus* out_status)
{
    return send_robot_command(base_client_, cmd, out_status);
}

bool Nrf24HardwareBridge::request_and_receive_telemetry(uint8_t robot_id)
{
    struct RobotTelemetry tel = {};
    double rtt_ms = 0.0;
    if (send_telemetry_request_and_wait(
            base_client_, robot_id, &tel, &rtt_ms, nullptr, TELEMETRY_TIMEOUT_MS)) {
        publish_telemetry(tel);
        return true;
    }
    return false;
}

void Nrf24HardwareBridge::publish_telemetry(const struct RobotTelemetry& telemetry_bp)
{
    oxebots_interfaces::msg::RobotTelemetry msg;
    msg.timestamp = telemetry_bp.header.timestamp;
    msg.robot_id = telemetry_bp.header.robot_id;
    msg.x = static_cast<float>(telemetry_bp.robot_pose.x);
    msg.y = static_cast<float>(telemetry_bp.robot_pose.y);
    msg.orientation = static_cast<float>(telemetry_bp.robot_pose.angle / 100.0);
    msg.vx = static_cast<float>(telemetry_bp.robot_pose.x_v);
    msg.vy = static_cast<float>(telemetry_bp.robot_pose.y_v);
    msg.w = static_cast<float>(telemetry_bp.robot_pose.angular_vel / 100.0);
    msg.battery_percentage = telemetry_bp.battery_percentage;
    msg.kicker_voltage = static_cast<float>(telemetry_bp.kicker_voltage / 100.0);
    msg.error_flags = telemetry_bp.error_flags;
    telemetry_pub_->publish(msg);
}

int main(int argc, char* argv[])
{
    const std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);

    bool cli_mode = false;
    bool benchmark_mode = false;
    bool scan_mode = false;
    std::string bench_spec = "3";
    double bench_duration = 10.0;
    int bench_tel_interval = 5;

    bool oneshot_config = false;
    uint8_t oneshot_id = 0;
    uint8_t oneshot_flags = 0;
    uint16_t oneshot_param = 0;

    try {
        for (size_t i = 1; i < args.size(); ++i) {
            const std::string& arg = args[i];
            if (arg == "--cli" || arg == "--config") {
                cli_mode = true;
            } else if (arg == "--scan") {
                scan_mode = true;
            } else if (arg == "--benchmark" || arg == "--bench") {
                benchmark_mode = true;
                if (i + 1 < args.size() && !args[i + 1].empty() && args[i + 1][0] != '-') {
                    bench_spec = args[++i];
                }
                if (i + 1 < args.size() && !args[i + 1].empty() && args[i + 1][0] != '-') {
                    bench_duration = std::stod(args[++i]);
                }
                if (i + 1 < args.size() && !args[i + 1].empty() && args[i + 1][0] != '-') {
                    bench_tel_interval = std::stoi(args[++i]);
                }
            } else if ((arg == "--port" || arg == "-p") && i + 1 < args.size()) {
                g_override_port = args[++i];
            } else if ((arg == "--baud" || arg == "-b") && i + 1 < args.size()) {
                g_override_baud = std::stoi(args[++i]);
            } else if ((arg == "--channel" || arg == "-c") && i + 1 < args.size()) {
                g_override_channel = std::stoi(args[++i]);
            } else if ((arg == "--rate" || arg == "-r") && i + 1 < args.size()) {
                g_override_rate = std::stoi(args[++i]);
            } else if (arg == "--pa" && i + 1 < args.size()) {
                g_override_pa = std::stoi(args[++i]);
            } else if (arg == "--skip-config") {
                g_skip_config = true;
            } else if (arg == "--set-id" && i + 2 < args.size()) {
                oneshot_config = true;
                oneshot_id = parse_robot_id_token(args[++i]);
                oneshot_flags = CONFIG_FLAG_SET_ID;
                oneshot_param = static_cast<uint16_t>(std::stoi(args[++i]));
            } else if (arg == "--set-channel" && i + 2 < args.size()) {
                oneshot_config = true;
                oneshot_id = parse_robot_id_token(args[++i]);
                oneshot_flags = CONFIG_FLAG_SET_CHANNEL;
                oneshot_param = static_cast<uint16_t>(std::stoi(args[++i]));
            } else if (arg == "--calibrate" && i + 1 < args.size()) {
                oneshot_config = true;
                oneshot_id = parse_robot_id_token(args[++i]);
                oneshot_flags = CONFIG_FLAG_RUN_CALIBRATION;
            } else if (arg == "--calibrate-mag" && i + 1 < args.size()) {
                oneshot_config = true;
                oneshot_id = parse_robot_id_token(args[++i]);
                oneshot_flags = CONFIG_FLAG_CALIBRATE_MAG;
            } else if (arg == "--reset-nvs" && i + 1 < args.size()) {
                oneshot_config = true;
                oneshot_id = parse_robot_id_token(args[++i]);
                oneshot_flags = CONFIG_FLAG_RESET_NVS;
            } else if (arg == "--factory-reset" && i + 1 < args.size()) {
                oneshot_config = true;
                oneshot_id = parse_robot_id_token(args[++i]);
                oneshot_flags = CONFIG_FLAG_FACTORY_RESET;
            } else if (arg == "--help" || arg == "-h") {
                print_cli_help();
                rclcpp::shutdown();
                return 0;
            }
        }

        const bool ros_loop = (!cli_mode && !benchmark_mode && !oneshot_config && !scan_mode);
        auto node = std::make_shared<Nrf24HardwareBridge>(ros_loop);

        if (oneshot_config) {
            struct BaseRfTxStatus st = {};
            send_robot_config(node->base_client(), oneshot_id, oneshot_flags, oneshot_param, &st);
            std::cout << "Config enviado: status=" << static_cast<int>(st.status)
                      << " retries=" << static_cast<int>(st.retries) << std::endl;
        } else if (scan_mode) {
            struct BaseScanResult scan = {};
            if (node->base_client().scan_spectrum(0, 125, 32, &scan)) {
                print_scan_result(scan);
            }
        } else if (benchmark_mode) {
            run_benchmark(node->base_client(), bench_spec, bench_duration, bench_tel_interval);
        } else if (cli_mode) {
            run_interactive_cli(node->base_client());
        } else {
            rclcpp::spin(node);
        }
    } catch (const std::exception& e) {
        std::cerr << C_RED << "Erro fatal: " << e.what() << C_RESET << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
