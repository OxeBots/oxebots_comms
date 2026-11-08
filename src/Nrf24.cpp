#include "oxebots_comms/Nrf24.hpp"

#include <fcntl.h>
#include <libserialport.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace
{
// ============================================================================
// 1. CONSTANTES DE CORES ANSI E TEMPORIZAÇÃO DE RÁDIO / UART
// ============================================================================

const std::string C_RED    = "\033[91m";
const std::string C_GREEN  = "\033[92m";
const std::string C_YELLOW = "\033[93m";
const std::string C_BLUE   = "\033[94m";
const std::string C_CYAN   = "\033[96m";
const std::string C_BOLD   = "\033[1m";
const std::string C_RESET  = "\033[0m";

// Nota de Arquitetura sobre o Dongle USB nRF24L01+ (CH340 + MCU STC15F104W @ 115200 baud):
// O microcontrolador STC15 do adaptador USB implementa UART e SPI por software (bit-banging).
// Ao receber um pacote pela serial (ex: 30 bytes de RobotCommand = ~2.6 ms), o STC15 aguarda
// ~4 ms de silêncio na linha UART para detectar fim de quadro, chaveia o nRF24L01+ para PTX,
// transmite via RF e, ao concluir, envia 6 bytes de status (02 00 00 00 00 00) de volta pela
// UART (~12.5 ms no total).
// Se o PC enviar o próximo pacote antes de o STC15 emitir esses 6 bytes de status, o STC15
// está ocupado no SPI/RF e perde bytes da UART, corrompendo o tamanho do pacote!
constexpr int FAST_GUARD_US             = 15000;  // 15ms: timeout máx aguardando ACK de 6B do dongle
constexpr int SAFE_ACK_GUARD_US         = 22000;  // 22ms: modo conservador (--safe_ack)
constexpr int DONGLE_SETTLE_US          = 1500;   // 1.5ms: folga após os 6B do dongle
constexpr int TELEMETRY_TIMEOUT_MS      = 35;     // Timeout aguardando RobotTelemetry (22B) após Req
constexpr int CLI_TELEMETRY_TIMEOUT_MS  = 150;    // Timeout para comando manual `tel <id>` no CLI
constexpr int OFFLINE_PROBE_INTERVAL_MS = 1000;   // Robô offline é sondado a 1 Hz (hot-plug)
constexpr int TX_TIMER_PERIOD_MS        = 15;     // Período do timer de transmissão ROS (~66 Hz)

// Flags globais configuráveis via argumentos de linha de comando
bool g_safe_ack_mode           = false;
bool g_skip_at_config          = false;
double g_override_freq         = -1.0;
int g_override_rate            = -1;
std::string g_override_tx_addr = "";
std::string g_override_rx_addr = "";
int g_active_serial_fd         = -1;

// ============================================================================
// 2. FUNÇÕES UTILITÁRIAS DE CONVERSÃO E SATURAÇÃO
// ============================================================================

inline int get_active_guard_us()
{
    return g_safe_ack_mode ? SAFE_ACK_GUARD_US : FAST_GUARD_US;
}

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

inline uint8_t clamp_u8(double v)
{
    const double clean = safe_finite(v, 0.0);
    const double clamped = std::clamp(clean, 0.0, 255.0);
    return static_cast<uint8_t>(std::llround(clamped));
}

// ============================================================================
// 3. FUNÇÕES DE TRANSMISSÃO E RECEPÇÃO DE PACOTES BITPROTO NA SERIAL
// ============================================================================

/**
 * @brief Codifica e envia um pacote RobotCommand (30 bytes, fire-and-forget) pela serial,
 *        sincronizando o término da transmissão RF através dos 6 bytes de status do dongle.
 */
bool send_robot_command_packet(int fd, const struct RobotCommand& cmd, int guard_us)
{
    if (fd < 0) return false;

    // IMPORTANTE: As funções Encode* geradas pelo Bitproto em C utilizam OR bit-a-bit (|=).
    // O buffer DEVE ser sempre inicializado com zeros ({0}), caso contrário lixo da stack
    // corrompe os bits do cabeçalho (msg_type, robot_id) e do payload!
    uint8_t tx_buf[BYTES_LENGTH_ROBOT_COMMAND] = {0};
    struct RobotCommand cmd_copy = cmd;
    EncodeRobotCommand(&cmd_copy, tx_buf);

    tcflush(fd, TCIFLUSH);
    auto t_start = std::chrono::steady_clock::now();

    ssize_t written = write(fd, tx_buf, BYTES_LENGTH_ROBOT_COMMAND);
    if (written != BYTES_LENGTH_ROBOT_COMMAND) {
        return false;
    }
    tcdrain(fd);

    // Aguarda os 6 bytes de confirmação de fim de TX do dongle (02 00 00 00 00 00)
    const int max_wait_ms = std::max(18, guard_us / 1000);
    int status_bytes_rcvd = 0;
    while (status_bytes_rcvd < 6) {
        int elapsed_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t_start)
                .count());
        int rem_ms = max_wait_ms - elapsed_ms;
        if (rem_ms <= 0) break;

        struct pollfd pfd = {fd, POLLIN, 0};
        int ret = poll(&pfd, 1, rem_ms);
        if (ret > 0 && (pfd.revents & POLLIN)) {
            uint8_t dump[32];
            ssize_t n = read(fd, dump, sizeof(dump));
            if (n > 0) {
                status_bytes_rcvd += static_cast<int>(n);
            }
        } else if (ret <= 0) {
            break;
        }
    }

    if (DONGLE_SETTLE_US > 0) {
        usleep(DONGLE_SETTLE_US);
    }
    tcflush(fd, TCIFLUSH);
    return true;
}

/**
 * @brief Codifica e envia um RobotTelemetryRequest (5 bytes) para `robot_id` e aguarda
 *        o pacote RobotTelemetry (22 bytes) correspondente dentro de `timeout_ms`.
 */
bool send_telemetry_request_and_wait(
    int fd,
    uint8_t robot_id,
    struct RobotTelemetry* out_tel,
    double* out_rtt_ms,
    int timeout_ms = TELEMETRY_TIMEOUT_MS)
{
    if (fd < 0 || robot_id == ROBOT_ID_BROADCAST) return false;

    struct RobotTelemetryRequest req = {};
    req.header.msg_type = MSG_TYPE_TELEMETRY_REQ;
    req.header.robot_id = robot_id;
    req.header.timestamp = now_ms_u32();

    // IMPORTANTE: Buffer zerado obrigatório para Encode* do Bitproto
    uint8_t tx_buf[BYTES_LENGTH_ROBOT_TELEMETRY_REQUEST] = {0};
    EncodeRobotTelemetryRequest(&req, tx_buf);

    tcflush(fd, TCIFLUSH);
    auto t_start = std::chrono::steady_clock::now();

    ssize_t written = write(fd, tx_buf, BYTES_LENGTH_ROBOT_TELEMETRY_REQUEST);
    if (written != BYTES_LENGTH_ROBOT_TELEMETRY_REQUEST) {
        return false;
    }
    tcdrain(fd);

    std::vector<uint8_t> rx_buffer;
    rx_buffer.reserve(64);
    const uint8_t dongle_status[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};

    while (true) {
        auto now = std::chrono::steady_clock::now();
        int elapsed_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now - t_start).count());
        int remaining_ms = timeout_ms - elapsed_ms;
        if (remaining_ms <= 0) {
            break;
        }

        struct pollfd pfd = {fd, POLLIN, 0};
        int ret = poll(&pfd, 1, remaining_ms);

        if (ret > 0 && (pfd.revents & POLLIN)) {
            uint8_t temp_buf[64];
            ssize_t n = read(fd, temp_buf, sizeof(temp_buf));
            if (n > 0) {
                rx_buffer.insert(rx_buffer.end(), temp_buf, temp_buf + n);

                // Remove sequências de 6 bytes de status de TX do dongle (02 00 00 00 00 00)
                while (rx_buffer.size() >= 6 &&
                       std::equal(
                           std::begin(dongle_status),
                           std::end(dongle_status),
                           rx_buffer.begin())) {
                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 6);
                }

                // Procura por um quadro válido de RobotTelemetry (22 bytes)
                while (rx_buffer.size() >= BYTES_LENGTH_ROBOT_TELEMETRY) {
                    uint8_t raw_msg_type = rx_buffer[0] & 0x0F;
                    if (raw_msg_type != MSG_TYPE_TELEMETRY) {
                        rx_buffer.erase(rx_buffer.begin());
                        continue;
                    }

                    // IMPORTANTE: Struct zerada obrigatória para Decode* do Bitproto
                    struct RobotTelemetry tel = {};
                    DecodeRobotTelemetry(&tel, rx_buffer.data());
                    rx_buffer.erase(
                        rx_buffer.begin(), rx_buffer.begin() + BYTES_LENGTH_ROBOT_TELEMETRY);

                    if (tel.header.robot_id == robot_id) {
                        auto t_end = std::chrono::steady_clock::now();
                        double rtt =
                            std::chrono::duration<double, std::milli>(t_end - t_start).count();
                        if (out_tel) *out_tel = tel;
                        if (out_rtt_ms) *out_rtt_ms = rtt;
                        if (DONGLE_SETTLE_US > 0) {
                            usleep(DONGLE_SETTLE_US);
                        }
                        tcflush(fd, TCIFLUSH);
                        return true;
                    }
                }
            }
        } else if (ret <= 0) {
            break;
        }
    }

    tcflush(fd, TCIFLUSH);
    return false;
}

/**
 * @brief Codifica e envia um pacote RobotConfig (8 bytes) para configurar ID, calibração ou NVS.
 */
bool send_robot_config_packet(int fd, uint8_t robot_id, uint8_t config_flags, uint16_t param = 0)
{
    struct RobotConfig cfg = {};
    cfg.header.msg_type = MSG_TYPE_CONFIG;
    cfg.header.robot_id = robot_id;
    cfg.header.timestamp = now_ms_u32();
    cfg.config_flags = config_flags;
    cfg.param = param;

    uint8_t buf[BYTES_LENGTH_ROBOT_CONFIG] = {0};
    EncodeRobotConfig(&cfg, buf);

    tcflush(fd, TCIFLUSH);
    ssize_t written = write(fd, buf, BYTES_LENGTH_ROBOT_CONFIG);
    tcdrain(fd);
    usleep(25000);
    tcflush(fd, TCIFLUSH);

    std::string id_str =
        (robot_id == ROBOT_ID_BROADCAST) ? "BROADCAST(15)" : std::to_string(robot_id);
    std::cout << C_CYAN << "[CONFIG ENVIADO] robot_id=" << id_str << " | flags=0x" << std::hex
              << static_cast<int>(config_flags) << std::dec << " | param=" << param << C_RESET
              << std::endl;
    return (written == BYTES_LENGTH_ROBOT_CONFIG);
}

// ============================================================================
// 4. COMANDOS AT DO DONGLE USB E TRADUÇÃO GB2312 -> INGLÊS / FORMATAÇÃO
// ============================================================================

/**
 * @brief Envia um comando AT ao dongle USB nRF24L01+ e traduz todas as strings em chinês
 *        (codificação GB2312 do firmware original do STC15) para inglês legível.
 */
std::string send_at_and_translate(int fd, const std::string& cmd, int wait_ms = 500)
{
    tcflush(fd, TCIOFLUSH);
    std::string full_cmd = cmd + "\r\n";
    ssize_t w = write(fd, full_cmd.c_str(), full_cmd.size());
    (void)w;
    tcdrain(fd);

    // Aguarda processamento inicial do comando AT pelo MCU STC15 do dongle
    usleep(80000);

    std::vector<uint8_t> resp;
    auto start = std::chrono::steady_clock::now();
    while (true) {
        auto now = std::chrono::steady_clock::now();
        int elapsed = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
        if (elapsed >= wait_ms) break;

        struct pollfd pfd = {fd, POLLIN, 0};
        int ret = poll(&pfd, 1, 50);
        if (ret > 0 && (pfd.revents & POLLIN)) {
            uint8_t buf[128];
            ssize_t n = read(fd, buf, sizeof(buf));
            if (n > 0) {
                resp.insert(resp.end(), buf, buf + n);
            }
        } else if (ret == 0 && !resp.empty()) {
            break;
        }
    }

    if (resp.empty()) return "";

    std::string raw_str(resp.begin(), resp.end());

    // Dicionário completo de sequências GB2312 emitidas pelo dongle nRF24L01+ (ordenadas da
    // mais específica/longa para a mais curta para evitar substituições parciais incorretas):
    const std::vector<std::pair<std::string, std::string>> dict = {
        // Frases de confirmação de configuração (AT+BAUD, AT+RATE, AT+FREQ, AT+RXA/TXA, AT+CRC)
        {"\xCD\xA8\xD1\xB6\xB2\xA8\xCC\xD8\xC2\xCA\xC9\xE8\xD6\xC3\xB3\xC9\xB9\xA6",
         "Baud rate setting successful"},
        {"\xCD\xA8\xD1\xB6\xC6\xB5\xC2\xCA\xC9\xE8\xD6\xC3\xB3\xC9\xB9\xA6",
         "Communication frequency setting successful"},
        {"\xB4\xAB\xCA\xE4\xCB\xD9\xC2\xCA\xC9\xE8\xD6\xC3\xB3\xC9\xB9\xA6",
         "Data rate setting successful"},
        {"\x43\x52\x43\xD0\xA3\xD1\xE9\xC4\xA3\xCA\xBD\xC9\xE8\xD6\xC3\xB3\xC9\xB9\xA6",
         "CRC check mode setting successful"},
        {"\xD0\xA3\xD1\xE9\xC4\xA3\xCA\xBD\xC9\xE8\xD6\xC3\xB3\xC9\xB9\xA6",
         "Check mode setting successful"},
        {"\xB5\xD8\xD6\xB7\xC9\xE8\xD6\xC3\xB3\xC9\xB9\xA6", "Address setting successful"},
        // Campos do relatório AT?
        {"\xCF\xB5\xCD\xB3\xD0\xC5\xCF\xA2", "System Information"},
        {"\xB1\xBE\xB5\xD8\xBD\xD3\xCA\xD5\xB5\xD8\xD6\xB7\x30",
         "Local Receive Address (Pipe 0)"},
        {"\xB1\xBE\xB5\xD8\xBD\xD3\xCA\xD5\xB5\xD8\xD6\xB7", "Local Receive Address (RX)"},
        {"\xC4\xBF\xB1\xEA\xB5\xD8\xD6\xB7", "Target Address (TX)"},
        {"\xBF\xD5\xD6\xD0\xB4\xAB\xCA\xE4\xCB\xD9\xC2\xCA", "Air Data Rate"},
        {"\xB5\xCD\xD4\xEB\xC9\xF9\xB7\xC5\xB4\xF3\xD4\xF6\xD2\xE6", "LNA Gain"},
        {"\xCD\xA8\xD1\xB6\xC6\xB5\xC2\xCA", "Communication Frequency"},
        {"\xD0\xA3\xD1\xE9\xC4\xA3\xCA\xBD", "Check Mode"},
        {"\xB7\xA2\xC9\xE4\xB9\xA6\xC2\xCA", "Transmit Power"},
        {"\xB4\xAB\xCA\xE4\xCB\xD9\xC2\xCA", "Data Rate"},
        {"\xB2\xA8\xCC\xD8\xC2\xCA", "Baud Rate"},
        // Valores e sufixos
        {"\x43\x52\x43\xD0\xA3\xD1\xE9", "CRC Check"},
        {"\xC9\xE8\xD6\xC3\xB3\xC9\xB9\xA6", "Setting successful"},
        {"\xB3\xC9\xB9\xA6", "successful"},
        {"\xBF\xAA\xC6\xF4", "Enabled"},
        {"\xB9\xD8\xB1\xD5", "Disabled"},
        {"\xD0\xA3\xD1\xE9", " Check"},
        {"\xCE\xBB", "-bit "},
        // Pontuação full-width chinesa (GB2312):
        // '：' (\xA3\xBA) seguido de espaços (como em '目标地址：     0x45...') ou sozinho
        {"\xA3\xBA     ", ": "},
        {"\xA3\xBA", ": "},
        // '！！' (\xA3\xA1\xA3\xA1) e '！' (\xA3\xA1)
        {"\xA3\xA1\xA3\xA1", "!"},
        {"\xA3\xA1", "!"},
    };

    for (const auto& kv : dict) {
        size_t pos = 0;
        while ((pos = raw_str.find(kv.first, pos)) != std::string::npos) {
            raw_str.replace(pos, kv.first.length(), kv.second);
            pos += kv.second.length();
        }
    }
    return raw_str;
}

/**
 * @brief Converte uma lista de bytes hexadecimais (ex: "0x45,0x53,0x50,0x33,0x32")
 *        em sua representação ASCII legível (ex: "ESP32"), caso todos sejam imprimíveis.
 */
std::string decode_hex_addr_to_ascii(const std::string& hex_csv)
{
    std::stringstream ss(hex_csv);
    std::string token;
    std::string ascii;
    while (std::getline(ss, token, ',')) {
        size_t first = token.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        token = token.substr(first);
        try {
            int val = std::stoi(token, nullptr, 16);
            if (val >= 32 && val <= 126) {
                ascii.push_back(static_cast<char>(val));
            } else {
                return "";
            }
        } catch (...) {
            return "";
        }
    }
    return ascii.length() == 5 ? (" (\"" + ascii + "\")") : "";
}

/**
 * @brief Consulta `AT?` no dongle USB e exibe uma tabela formatada e alinhada no terminal.
 */
void print_dongle_system_info(int fd)
{
    std::string info = send_at_and_translate(fd, "AT?", 600);
    if (info.empty()) {
        std::cout << C_RED << "Sem resposta do dongle USB ao comando AT?." << C_RESET << std::endl;
        return;
    }

    std::cout << "\n"
              << C_BOLD << std::string(64, '=') << "\n"
              << " INFORMAÇÕES DO MÓDULO USB NRF24L01+ (AT?)\n"
              << std::string(64, '=') << C_RESET << "\n";

    std::stringstream ss(info);
    std::string line;
    while (std::getline(ss, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        size_t first = line.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        line = line.substr(first);

        if (line == "OK") {
            std::cout << "  " << std::left << std::setw(30) << "Status"
                      << ": " << C_GREEN << "OK" << C_RESET << "\n";
            continue;
        }
        if (line.rfind("System Information", 0) == 0) {
            continue;
        }

        size_t colon = line.find(':');
        if (colon != std::string::npos) {
            std::string key = line.substr(0, colon);
            std::string val = line.substr(colon + 1);
            size_t kw = key.find_last_not_of(" \t");
            if (kw != std::string::npos) key = key.substr(0, kw + 1);
            size_t vw = val.find_first_not_of(" \t");
            val = (vw != std::string::npos) ? val.substr(vw) : "";

            // Se for um endereço hexadecimal (0x..), anexa também a string ASCII equivalente
            if (val.find("0x") != std::string::npos) {
                val += decode_hex_addr_to_ascii(val);
            }

            std::cout << "  " << C_CYAN << std::left << std::setw(30) << key << C_RESET << ": "
                      << C_GREEN << val << C_RESET << "\n";
        } else {
            std::cout << "  " << line << "\n";
        }
    }
    std::cout << C_BOLD << std::string(64, '=') << C_RESET << std::endl;
}

uint8_t parse_robot_id_token(const std::string& token)
{
    if (token == "broadcast" || token == "BROADCAST" || token == "all" || token == "15") {
        return ROBOT_ID_BROADCAST;
    }
    return static_cast<uint8_t>(std::stoi(token));
}

// ============================================================================
// 5. BENCHMARK MULTI-ROBÔ COM RECONEXÃO AUTOMÁTICA (HOT-PLUG 1 Hz)
// ============================================================================

/**
 * @brief Executa o benchmark de comunicação nRF24L01+ (equivalente C++ ao nrf24_benchmark.py).
 *
 * Comportamento de Sondagem e Hot-Plug (Reconexão Automática):
 * - Antes de iniciar o loop, sonda os robôs para identificar quais já estão ligados.
 * - Robôs que iniciam desligados (ou que perdem 5 telemetrias seguidas durante o teste)
 *   são marcados como `OFFLINE` (`unreachable = true`), evitando travar o loop por 35ms
 *   a cada `tel_interval` comandos.
 * - Entretanto, todo robô `OFFLINE` continua recebendo `RobotCommand` normalmente E é
 *   sondado com um `RobotTelemetryRequest` a cada 1 segundo (1 Hz).
 * - Assim que o usuário liga um robô no meio do benchmark, a sondagem de 1 Hz recebe a
 *   resposta, marca o robô como `ONLINE` automaticamente e passa a requisitar telemetria
 *   normalmente a cada `tel_interval` comandos!
 */
void run_benchmark(
    int fd,
    const std::string& target_spec = "3",
    double duration_s = 10.0,
    int tel_interval = 5,
    int guard_us = FAST_GUARD_US)
{
    struct BenchStats
    {
        int cmd_sent = 0;
        int tel_req_sent = 0;
        int tel_received = 0;
        int tel_lost = 0;
        int consecutive_losses = 0;
        bool unreachable = false;
        bool ever_online = false;
        std::chrono::steady_clock::time_point last_tel_attempt{};
        std::vector<double> rtts_ms;
        int last_batt = -1;
        double last_kicker_v = 0.0;
    };

    std::vector<uint8_t> robot_ids;
    std::set<uint8_t> initially_online;
    tcflush(fd, TCIOFLUSH);

    // 1. Determina a lista de robôs do benchmark e realiza sondagem inicial
    if (target_spec.find(',') != std::string::npos || target_spec.rfind("id=", 0) == 0) {
        std::string clean_spec =
            (target_spec.rfind("id=", 0) == 0) ? target_spec.substr(3) : target_spec;
        std::stringstream ss(clean_spec);
        std::string item;
        while (std::getline(ss, item, ',')) {
            if (!item.empty()) {
                robot_ids.push_back(static_cast<uint8_t>(std::stoi(item)));
            }
        }
        std::cout << C_CYAN << "Sondando robôs selecionados..." << C_RESET << std::flush;
        for (uint8_t probe_id : robot_ids) {
            struct RobotTelemetry tel = {};
            double rtt = 0.0;
            if (send_telemetry_request_and_wait(fd, probe_id, &tel, &rtt, 45)) {
                initially_online.insert(probe_id);
            }
        }
    } else {
        int num_robots = std::max(1, std::stoi(target_spec));
        std::cout << C_CYAN << "Sondando robôs ligados (IDs 0 a 5)..." << C_RESET << std::flush;
        for (uint8_t probe_id = 0; probe_id <= 5; ++probe_id) {
            struct RobotTelemetry tel = {};
            double rtt = 0.0;
            if (send_telemetry_request_and_wait(fd, probe_id, &tel, &rtt, 45)) {
                initially_online.insert(probe_id);
                if (static_cast<int>(robot_ids.size()) < num_robots) {
                    robot_ids.push_back(probe_id);
                }
            }
        }
        for (uint8_t cand = 0; static_cast<int>(robot_ids.size()) < num_robots && cand <= 10;
             ++cand) {
            if (std::find(robot_ids.begin(), robot_ids.end(), cand) == robot_ids.end()) {
                robot_ids.push_back(cand);
            }
        }
        std::sort(robot_ids.begin(), robot_ids.end());
    }

    if (!initially_online.empty()) {
        std::cout << " " << C_GREEN << "✓ Detectados online: [";
        size_t idx = 0;
        for (uint8_t oid : initially_online) {
            std::cout << static_cast<int>(oid) << (++idx < initially_online.size() ? ", " : "");
        }
        std::cout << "]" << C_RESET << "\n";
    } else {
        std::cout << " " << C_YELLOW
                  << "(nenhum respondeu agora — sondagem automática de 1 Hz ativa!)" << C_RESET
                  << "\n";
    }

    if (robot_ids.empty()) {
        robot_ids.push_back(0);
    }

    const int num_robots = static_cast<int>(robot_ids.size());
    const auto start_bench = std::chrono::steady_clock::now();
    std::map<uint8_t, BenchStats> stats;
    std::ostringstream ids_oss;

    for (size_t i = 0; i < robot_ids.size(); ++i) {
        uint8_t rid = robot_ids[i];
        BenchStats st{};
        const bool is_on = (initially_online.count(rid) > 0);
        st.ever_online = is_on;
        st.unreachable = !is_on;  // Se começou desligado, já inicia em modo sondagem 1 Hz
        st.last_tel_attempt = start_bench;
        stats[rid] = st;
        ids_oss << static_cast<int>(rid) << (i + 1 < robot_ids.size() ? ", " : "");
    }

    std::cout << "\n"
              << C_BOLD << std::string(80, '=') << "\n"
              << " INICIANDO BENCHMARK NRF24 C++ (Robôs: [" << ids_oss.str()
              << "] | Duração: " << duration_s << "s)\n"
              << " Protocolo: RobotCommand (30B s/ ACK) + RobotTelemetryRequest (5B a cada "
              << tel_interval << " cmds)\n"
              << " Sincronismo TX: ACK 6B do Dongle (máx " << (guard_us / 1000.0)
              << "ms) | Sondagem Offline (Hot-Plug): 1 Hz\n"
              << std::string(80, '=') << C_RESET << "\n\n";

    tcflush(fd, TCIOFLUSH);
    auto last_print = start_bench;
    int cycle_count = 0;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed_s = std::chrono::duration<double>(now - start_bench).count();
        if (elapsed_s >= duration_s) break;

        cycle_count++;
        for (size_t idx = 0; idx < robot_ids.size(); ++idx) {
            uint8_t rid = robot_ids[idx];
            auto& st = stats[rid];

            // 1. Envia RobotCommand (30B, fire-and-forget, sincronizado com status 6B do dongle)
            struct RobotCommand cmd = {};
            cmd.header.msg_type = MSG_TYPE_COMMAND;
            cmd.header.robot_id = rid;
            cmd.header.timestamp = now_ms_u32();
            cmd.target_pose.x = static_cast<int16_t>(cycle_count % 1000);
            send_robot_command_packet(fd, cmd, guard_us);
            st.cmd_sent++;

            // 2. Verifica se deve enviar RobotTelemetryRequest (5B) para este robô:
            //    - Se estiver ONLINE: a cada `tel_interval` comandos (escalonado por `idx`)
            //    - Se estiver OFFLINE: sonda a cada 1000 ms (1 Hz) para detectar hot-plug!
            if (tel_interval <= 0) {
                continue;
            }

            const auto step_now = std::chrono::steady_clock::now();
            bool should_request_tel = false;

            if (!st.unreachable) {
                should_request_tel = ((st.cmd_sent + static_cast<int>(idx)) % tel_interval == 0);
            } else {
                auto ms_since_probe = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          step_now - st.last_tel_attempt)
                                          .count();
                if (ms_since_probe >= OFFLINE_PROBE_INTERVAL_MS) {
                    should_request_tel = true;
                }
            }

            if (!should_request_tel) {
                continue;
            }

            st.last_tel_attempt = step_now;
            const bool was_unreachable = st.unreachable;

            struct RobotTelemetry tel = {};
            double rtt_ms = 0.0;
            if (send_telemetry_request_and_wait(fd, rid, &tel, &rtt_ms, TELEMETRY_TIMEOUT_MS)) {
                st.tel_req_sent++;
                st.tel_received++;
                st.consecutive_losses = 0;
                st.unreachable = false;
                st.ever_online = true;
                st.rtts_ms.push_back(rtt_ms);
                st.last_batt = tel.battery_percentage;
                st.last_kicker_v = tel.kicker_voltage / 100.0;

                if (was_unreachable) {
                    std::cout << "\n"
                              << C_GREEN << "✓ [HOT-PLUG] Robô " << static_cast<int>(rid)
                              << " detectado ONLINE! Telemetria normal ativada (a cada "
                              << tel_interval << " cmds)." << C_RESET << "\n";
                }
            } else {
                // Só contabiliza perda na estatística se o robô estava no modo ONLINE normal.
                // Sondagens de 1 Hz enquanto o robô está desligado não inflam a taxa de perda!
                if (!was_unreachable) {
                    st.tel_req_sent++;
                    st.tel_lost++;
                    st.consecutive_losses++;
                    if (st.consecutive_losses >= 5) {
                        st.unreachable = true;
                        std::cout << "\n"
                                  << C_YELLOW << "[AVISO] Robô " << static_cast<int>(rid)
                                  << " sem resposta após 5 tentativas — entrando em sondagem 1 Hz."
                                  << C_RESET << "\n";
                    }
                }
            }
        }

        auto after_cycle = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(after_cycle - last_print).count() >= 0.25) {
            double total_elapsed = std::chrono::duration<double>(after_cycle - start_bench).count();
            int total_cmds = 0;
            for (const auto& kv : stats) total_cmds += kv.second.cmd_sent;
            double cmd_hz = (total_elapsed > 0.0) ? (total_cmds / total_elapsed) : 0.0;
            double per_robot_hz = (num_robots > 0) ? (cmd_hz / num_robots) : 0.0;

            std::cout << "\r" << C_CYAN << "[" << std::fixed << std::setprecision(1)
                      << total_elapsed << "s] Cmds: " << std::setw(6) << cmd_hz << " Hz ("
                      << std::setw(5) << per_robot_hz << " Hz/robô)" << C_RESET << " | ";
            for (uint8_t rid : robot_ids) {
                const auto& st = stats[rid];
                if (st.unreachable) {
                    std::cout << "R" << static_cast<int>(rid) << ":" << C_RED << "OFFLINE(1Hz) "
                              << C_RESET;
                } else {
                    double loss_pct =
                        (st.tel_req_sent > 0) ? (100.0 * st.tel_lost / st.tel_req_sent) : 0.0;
                    const std::string& c =
                        (loss_pct < 15.0) ? C_GREEN : ((loss_pct < 50.0) ? C_YELLOW : C_RED);
                    std::cout << "R" << static_cast<int>(rid) << ":" << c << std::setw(4)
                              << std::setprecision(0) << loss_pct << "% loss" << C_RESET << " ";
                }
            }
            std::cout << std::flush;
            last_print = after_cycle;
        }
    }

    double total_time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_bench).count();
    int total_cmd_sent = 0;
    int total_tel_req = 0;
    int total_tel_rx = 0;
    for (const auto& kv : stats) {
        total_cmd_sent += kv.second.cmd_sent;
        total_tel_req += kv.second.tel_req_sent;
        total_tel_rx += kv.second.tel_received;
    }

    double avg_cmd_freq = (total_time > 0.0) ? (total_cmd_sent / total_time) : 0.0;
    double freq_per_robot = (num_robots > 0) ? (avg_cmd_freq / num_robots) : 0.0;

    std::cout << "\n\n"
              << C_BOLD << std::string(86, '=') << "\n"
              << " RELATÓRIO FINAL DE BENCHMARK (" << std::fixed << std::setprecision(2)
              << total_time << " segundos)\n"
              << std::string(86, '=') << C_RESET << "\n"
              << "  Ciclos completos (Round-Robin) : " << cycle_count << "\n"
              << "  Total de Comandos Enviados     : " << total_cmd_sent << "\n"
              << "  Total de Telemetrias (RX/Req)  : " << total_tel_rx << " / " << total_tel_req
              << "\n"
              << "  Frequência Total de Comandos   : " << C_BOLD << avg_cmd_freq << " Hz" << C_RESET
              << " (pacotes/s)\n"
              << "  Frequência Efetiva por Robô    : " << C_BOLD << freq_per_robot << " Hz"
              << C_RESET << "\n"
              << std::string(86, '-') << "\n"
              << "  Robô |   Cmds | Tel Req | Tel RX | Perda Tel | RTT Médio | RTT Min/Max   | Bateria\n"
              << std::string(86, '-') << "\n";

    for (uint8_t rid : robot_ids) {
        const auto& st = stats[rid];
        double loss_pct = (st.tel_req_sent > 0) ? (100.0 * st.tel_lost / st.tel_req_sent) : 0.0;
        const std::string& color =
            (st.tel_req_sent == 0)
                ? C_YELLOW
                : ((loss_pct <= 10.0) ? C_GREEN : ((loss_pct <= 35.0) ? C_YELLOW : C_RED));

        std::string rtt_avg_str = "   N/A  ";
        std::string rtt_minmax_str = "     N/A     ";
        std::string batt_str = (!st.ever_online) ? "OFFLINE" : "N/A";

        if (!st.rtts_ms.empty()) {
            double sum = 0.0;
            double rmin = st.rtts_ms[0], rmax = st.rtts_ms[0];
            for (double v : st.rtts_ms) {
                sum += v;
                rmin = std::min(rmin, v);
                rmax = std::max(rmax, v);
            }
            std::ostringstream oss_avg, oss_mm, oss_b;
            oss_avg << std::fixed << std::setprecision(2) << std::setw(6)
                    << (sum / st.rtts_ms.size()) << "ms";
            oss_mm << std::fixed << std::setprecision(1) << std::setw(4) << rmin << "/"
                   << std::setw(4) << rmax << "ms";
            oss_b << st.last_batt << "% (" << std::fixed << std::setprecision(1) << st.last_kicker_v
                  << "V)";
            rtt_avg_str = oss_avg.str();
            rtt_minmax_str = oss_mm.str();
            batt_str = oss_b.str();
        }

        std::cout << "   " << std::setw(2) << static_cast<int>(rid) << "  | " << std::setw(6)
                  << st.cmd_sent << " | " << std::setw(7) << st.tel_req_sent << " | "
                  << std::setw(6) << st.tel_received << " | " << color << std::setw(8) << std::fixed
                  << std::setprecision(1) << loss_pct << "%" << C_RESET << " | " << rtt_avg_str
                  << "  | " << rtt_minmax_str << " | " << batt_str << "\n";
    }
    std::cout << std::string(86, '=') << "\n";
}

// ============================================================================
// 6. INTERFACE DE LINHA DE COMANDO INTERATIVA (CLI)
// ============================================================================

void print_cli_help()
{
    std::cout << "\n"
              << C_BOLD << "=== COMANDOS DO MODO INTERATIVO (CLI) ===" << C_RESET << "\n"
              << "  {x:<mm>, y:<mm>, vx:<mm/s>, vy:<mm/s>, w:<0.01rad/s>, kick:<0-255>, id:<id>} -> Envia RobotCommand (30B, sem resposta)\n"
              << "  tel <robot_id>                                         -> Envia RobotTelemetryRequest (5B) e aguarda Telemetria\n"
              << "  bench [robos|id=1|0,1] [duracao_s] [intervalo_tel]     -> Executa benchmark (ex: bench 1 10 5 ou bench 0,1 10 5)\n"
              << "  set_id <target_id|broadcast> <new_id>                  -> Altera o ID do robô na NVS\n"
              << "  calibrate [<robot_id>|broadcast]                       -> Calibração completa (Rodas + IMU + PID)\n"
              << "  calibrate_mag [<robot_id>|broadcast]                   -> Calibração do magnetômetro\n"
              << "  reset_nvs [<robot_id>|broadcast]                       -> Reseta o ID do robô para 0\n"
              << "  factory_reset [<robot_id>|broadcast]                   -> Apaga toda a NVS e reinicia a ESP32\n"
              << "  at <COMANDO_AT>                                        -> Envia comando AT direto ao dongle (ex: at AT?)\n"
              << "  info                                                   -> Exibe informações do dongle USB (AT?) formatadas\n"
              << "  help                                                   -> Mostra esta ajuda\n"
              << "  q / quit / exit                                        -> Sai do programa\n";
}

void run_interactive_cli(int fd)
{
    std::cout << "\n" << C_BOLD << "=== TERMINAL INTERATIVO NRF24 C++ ===" << C_RESET << std::endl;
    print_cli_help();

    std::string line;
    while (rclcpp::ok()) {
        std::cout << "\n" << C_BLUE << "Cmd > " << C_RESET << std::flush;
        if (!std::getline(std::cin, line)) break;

        size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) continue;
        size_t last = line.find_last_not_of(" \t\r\n");
        std::string raw = line.substr(first, last - first + 1);

        if (raw == "q" || raw == "quit" || raw == "exit") break;
        if (raw == "help") {
            print_cli_help();
            continue;
        }
        if (raw == "info") {
            print_dongle_system_info(fd);
            continue;
        }
        if (raw.rfind("at ", 0) == 0 || raw.rfind("AT", 0) == 0) {
            std::string at_cmd = (raw.rfind("at ", 0) == 0) ? raw.substr(3) : raw;
            if (at_cmd == "AT?") {
                print_dongle_system_info(fd);
            } else {
                std::string resp = send_at_and_translate(fd, at_cmd, 600);
                std::cout << C_GREEN << (resp.empty() ? "Sem resposta." : resp) << C_RESET
                          << std::endl;
            }
            continue;
        }

        std::istringstream iss(raw);
        std::vector<std::string> parts;
        std::string tok;
        while (iss >> tok) parts.push_back(tok);
        if (parts.empty()) continue;

        try {
            if (parts[0] == "set_id") {
                if (parts.size() < 3) {
                    std::cout << C_YELLOW << "Uso: set_id <target_id|broadcast> <new_id>" << C_RESET
                              << std::endl;
                    continue;
                }
                uint8_t target = parse_robot_id_token(parts[1]);
                uint16_t new_id = static_cast<uint16_t>(std::stoi(parts[2]));
                send_robot_config_packet(fd, target, CONFIG_FLAG_SET_ID, new_id);
                continue;
            }
            if (parts[0] == "calibrate_mag") {
                uint8_t rid = (parts.size() > 1) ? parse_robot_id_token(parts[1]) : 0;
                send_robot_config_packet(fd, rid, CONFIG_FLAG_CALIBRATE_MAG, 0);
                continue;
            }
            if (parts[0] == "calibrate") {
                uint8_t rid = (parts.size() > 1) ? parse_robot_id_token(parts[1]) : 0;
                send_robot_config_packet(fd, rid, CONFIG_FLAG_RUN_CALIBRATION, 0);
                continue;
            }
            if (parts[0] == "reset_nvs") {
                uint8_t rid = (parts.size() > 1) ? parse_robot_id_token(parts[1]) : 0;
                send_robot_config_packet(fd, rid, CONFIG_FLAG_RESET_NVS, 0);
                continue;
            }
            if (parts[0] == "factory_reset") {
                uint8_t rid = (parts.size() > 1) ? parse_robot_id_token(parts[1]) : 0;
                send_robot_config_packet(fd, rid, CONFIG_FLAG_FACTORY_RESET, 0);
                continue;
            }
            if (parts[0] == "tel") {
                uint8_t rid = (parts.size() > 1) ? parse_robot_id_token(parts[1]) : 0;
                struct RobotTelemetry tel = {};
                double rtt_ms = 0.0;
                std::cout << C_CYAN << "→ [TEL_REQ] robot=" << static_cast<int>(rid) << C_RESET
                          << std::flush;
                if (send_telemetry_request_and_wait(
                        fd, rid, &tel, &rtt_ms, CLI_TELEMETRY_TIMEOUT_MS)) {
                    std::cout << "\r" << C_GREEN << "✓ [RX] Robô "
                              << static_cast<int>(tel.header.robot_id) << " | RTT=" << std::fixed
                              << std::setprecision(2) << rtt_ms
                              << "ms | Batt=" << static_cast<int>(tel.battery_percentage)
                              << "% | Kicker=" << (tel.kicker_voltage / 100.0) << "V" << C_RESET
                              << "          \n";
                } else {
                    std::cout << "\n"
                              << C_RED << "✗ [TIMEOUT] Sem resposta de telemetria do Robô "
                              << static_cast<int>(rid) << C_RESET << std::endl;
                }
                continue;
            }
            if (parts[0] == "bench" || parts[0] == "benchmark") {
                std::string target_spec = (parts.size() > 1) ? parts[1] : "3";
                double dur_s = (parts.size() > 2) ? std::stod(parts[2]) : 10.0;
                int tel_int = (parts.size() > 3) ? std::stoi(parts[3]) : 5;
                run_benchmark(fd, target_spec, dur_s, tel_int, get_active_guard_us());
                continue;
            }

            // Comando de movimento: {x:0, y:0, vx:500, vy:0, w:0, kick:0, id:1}
            int16_t x = 0, y = 0, angle = 0, vx = 0, vy = 0, w = 0;
            uint8_t kick = 0, robot_id = 0;

            std::string clean = raw;
            clean.erase(std::remove(clean.begin(), clean.end(), '{'), clean.end());
            clean.erase(std::remove(clean.begin(), clean.end(), '}'), clean.end());
            clean.erase(std::remove(clean.begin(), clean.end(), '['), clean.end());
            clean.erase(std::remove(clean.begin(), clean.end(), ']'), clean.end());

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
                    size_t e = s.find_last_not_of(ws);
                    s = s.substr(b, e - b + 1);
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

            struct RobotCommand cmd_bp = {};
            cmd_bp.header.msg_type = MSG_TYPE_COMMAND;
            cmd_bp.header.robot_id = robot_id;
            cmd_bp.header.timestamp = now_ms_u32();
            cmd_bp.target_pose.x = x;
            cmd_bp.target_pose.y = y;
            cmd_bp.target_pose.angle = angle;
            cmd_bp.target_pose.x_v = vx;
            cmd_bp.target_pose.y_v = vy;
            cmd_bp.target_pose.angular_vel = w;
            cmd_bp.kick_velocity = kick;

            // Envia apenas RobotCommand (30B, fire-and-forget — não pede telemetria)
            send_robot_command_packet(fd, cmd_bp, get_active_guard_us());

            std::string id_label =
                (robot_id == ROBOT_ID_BROADCAST) ? "BROADCAST" : std::to_string(robot_id);
            std::cout << C_CYAN << "→ [CMD ENVIADO] robot=" << id_label
                      << " ts=" << cmd_bp.header.timestamp << " x=" << x << " y=" << y
                      << " vx=" << vx << " vy=" << vy << " w=" << w
                      << " kick=" << static_cast<int>(kick) << C_RESET << std::endl;
        } catch (const std::exception& e) {
            std::cout << C_RED << "Erro ao interpretar comando: " << e.what() << C_RESET
                      << std::endl;
        }
    }
}

}  // namespace

// ============================================================================
// 7. IMPLEMENTAÇÃO DA CLASSE ROS 2 Nrf24HardwareBridge
// ============================================================================

std::string Find_Nrf24_port(uint16_t port_pid, uint16_t port_vid)
{
    struct sp_port** port_list;
    enum sp_return result = sp_list_ports(&port_list);

    if (result != SP_OK) {
        std::cerr << "Error - Could not list the serial ports" << std::endl;
        return "";
    }

    std::string port_name_found = "";
    for (int i = 0; port_list[i] != NULL; i++) {
        struct sp_port* port = port_list[i];
        int vid, pid;
        if (sp_get_port_usb_vid_pid(port, &vid, &pid) == SP_OK) {
            if (vid == port_vid && pid == port_pid) {
                port_name_found = sp_get_port_name(port);
                break;
            }
        }
    }

    sp_free_port_list(port_list);
    return port_name_found;
}

Nrf24HardwareBridge::Nrf24HardwareBridge(bool enable_ros_loop) : Node("nrf24_hardware_bridge")
{
    // Declaração de parâmetros ROS 2 (carregados via match_config.yaml)
    this->declare_parameter("radio_rx_address", "ADMIN");
    this->declare_parameter("radio_tx_address", "ESP32");
    this->declare_parameter("radio_frequency", 2.476);
    this->declare_parameter("radio_rate", 3);
    this->declare_parameter("command_mode", std::string("velocity"));
    this->declare_parameter("telemetry_request_interval", 5);

    std::string command_mode = this->get_parameter("command_mode").as_string();
    telemetry_request_interval_ = this->get_parameter("telemetry_request_interval").as_int();

    // Busca automática da porta USB do adaptador CH340 (VID 1a86, PID 7523)
    const uint16_t port_vid = 0x1A86, port_pid = 0x7523;
    std::string port_name = Find_Nrf24_port(port_pid, port_vid);
    if (port_name.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Dispositivo nRF24L01 não encontrado nas portas USB.");
        throw std::runtime_error("Hardware do nRF24 ausente");
    }

    RCLCPP_INFO(this->get_logger(), "Dispositivo encontrado na porta: %s", port_name.c_str());

    if (!open_serial_port(port_name)) {
        RCLCPP_ERROR(this->get_logger(), "Erro ao abrir porta serial: %s", port_name.c_str());
        throw std::runtime_error("Falha na porta serial");
    }

    g_active_serial_fd = serial_fd_;

    if (!g_skip_at_config) {
        configure_nrf24();
    } else {
        RCLCPP_INFO(this->get_logger(), "Pulando configuração AT inicial (--skip-config).");
        tcflush(serial_fd_, TCIOFLUSH);
    }

    if (enable_ros_loop) {
        telemetry_pub_ =
            this->create_publisher<oxebots_interfaces::msg::RobotTelemetry>("/robot_telemetry", 10);

        // Assina game_data para obter a orientação atual dos robôs (conversão Global -> Local)
        game_data_sub_ = this->create_subscription<oxebots_interfaces::msg::GameData>(
            "game_data",
            rclcpp::SensorDataQoS(),
            std::bind(&Nrf24HardwareBridge::game_data_callback, this, std::placeholders::_1));

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

        // Timer dedicado para desacoplar a escrita serial dos callbacks de subscrição ROS
        tx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(TX_TIMER_PERIOD_MS),
            std::bind(&Nrf24HardwareBridge::tx_timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "%sPonte ROS 2 -> NRF24L01 pronta! Modo: '%s' | Telemetria a cada %d cmds | Sincronismo: ACK 6B do Dongle%s",
            C_GREEN.c_str(),
            command_mode.c_str(),
            telemetry_request_interval_,
            C_RESET.c_str());
    }
}

Nrf24HardwareBridge::~Nrf24HardwareBridge()
{
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        RCLCPP_INFO(this->get_logger(), "Porta serial fechada com segurança.");
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

struct Pose Nrf24HardwareBridge::kinematic_pose_to_bitproto(
    const oxebots_interfaces::msg::RobotKinematicPose& pose)
{
    struct Pose pose_bitproto = {};
    pose_bitproto.x = clamp_i16(pose.x);
    pose_bitproto.y = clamp_i16(pose.y);
    pose_bitproto.angle = clamp_i16(safe_finite(pose.angle) * (180.0 / M_PI) * 100.0);
    pose_bitproto.x_v = clamp_i16(pose.x_v);
    pose_bitproto.y_v = clamp_i16(pose.y_v);
    pose_bitproto.angular_vel = clamp_i16(safe_finite(pose.angular_vel) * 100.0);
    return pose_bitproto;
}

void Nrf24HardwareBridge::send_command(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg)
{
    const auto now = std::chrono::steady_clock::now();

    for (const auto& robot_ros : msg->robots) {
        if (!std::isfinite(robot_ros.x_velocity) || !std::isfinite(robot_ros.y_velocity) ||
            !std::isfinite(robot_ros.angular_velocity) || !std::isfinite(robot_ros.kick_speed)) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Robô %u: comando não-finito (vx=%.2f vy=%.2f w=%.2f kick=%.2f), zerando campos inválidos.",
                robot_ros.id,
                robot_ros.x_velocity,
                robot_ros.y_velocity,
                robot_ros.angular_velocity,
                robot_ros.kick_speed);
        }

        const double vx_global = safe_finite(robot_ros.x_velocity);
        const double vy_global = safe_finite(robot_ros.y_velocity);
        const double w = safe_finite(robot_ros.angular_velocity);
        const double kick = safe_finite(robot_ros.kick_speed);

        // Rotaciona velocidade do referencial Global (visão) para o referencial Local do robô
        const double theta = robot_orientations_[robot_ros.id];
        const double v_forward = vx_global * std::cos(theta) + vy_global * std::sin(theta);
        const double v_left = -vx_global * std::sin(theta) + vy_global * std::cos(theta);

        struct RobotCommand cmd_bitproto = {};
        cmd_bitproto.header.msg_type = MSG_TYPE_COMMAND;
        cmd_bitproto.header.robot_id = robot_ros.id;
        cmd_bitproto.header.timestamp =
            static_cast<uint32_t>((this->now().nanoseconds() / 1000000ULL) & 0xFFFFFFFFULL);

        cmd_bitproto.target_pose.x_v = clamp_i16(v_forward * 1000.0);
        cmd_bitproto.target_pose.y_v = clamp_i16(v_left * 1000.0);
        cmd_bitproto.target_pose.angular_vel = clamp_i16(w * 100.0);
        cmd_bitproto.kick_velocity = clamp_u8(kick);

        auto& slot = pending_commands_[robot_ros.id];
        slot.cmd = cmd_bitproto;
        slot.updated_at = now;
        slot.pending = true;
    }
}

void Nrf24HardwareBridge::send_trajectory_setpoint(
    const oxebots_interfaces::msg::RobotTrajectorySetpoint::SharedPtr msg)
{
    const auto now = std::chrono::steady_clock::now();

    for (const auto& robot_ros : msg->robots) {
        struct RobotCommand cmd_bitproto = {};
        cmd_bitproto.header.msg_type = MSG_TYPE_COMMAND;
        cmd_bitproto.header.robot_id = robot_ros.id;
        cmd_bitproto.header.timestamp =
            static_cast<uint32_t>((this->now().nanoseconds() / 1000000ULL) & 0xFFFFFFFFULL);

        cmd_bitproto.target_pose = kinematic_pose_to_bitproto(robot_ros.target);
        cmd_bitproto.vision_pose = kinematic_pose_to_bitproto(robot_ros.vision);
        cmd_bitproto.kick_velocity = clamp_u8(robot_ros.kick_speed);

        auto& slot = pending_commands_[robot_ros.id];
        slot.cmd = cmd_bitproto;
        slot.updated_at = now;
        slot.pending = true;
    }
}

void Nrf24HardwareBridge::tx_timer_callback()
{
    if (serial_fd_ < 0 || pending_commands_.empty()) return;

    const auto now = std::chrono::steady_clock::now();

    for (auto& [robot_id, entry] : pending_commands_) {
        if (!entry.pending) continue;
        entry.pending = false;

        // Atualiza o timestamp exatamente no instante do envio pelo rádio
        entry.cmd.header.timestamp =
            static_cast<uint32_t>((this->now().nanoseconds() / 1000000ULL) & 0xFFFFFFFFULL);

        // 1. Transmite o RobotCommand (30B, sincronizado pelo ACK 6B do dongle USB)
        if (!transmit_command(entry.cmd)) {
            continue;
        }

        // 2. Verifica se está na hora de solicitar telemetria para este robô
        if (telemetry_request_interval_ <= 0 || robot_id == ROBOT_ID_BROADCAST) {
            continue;
        }

        auto [it, inserted] = telemetry_states_.try_emplace(robot_id);
        auto& tel_state = it->second;
        if (inserted) {
            // Escalona o contador inicial pelo ID do robô para que dois robôs não peçam
            // telemetria no mesmo ciclo de timer
            tel_state.cmd_since_last_tel = robot_id % telemetry_request_interval_;
        }

        tel_state.cmd_since_last_tel++;
        if (tel_state.cmd_since_last_tel < telemetry_request_interval_) {
            continue;
        }
        tel_state.cmd_since_last_tel = 0;

        // Se o robô não respondeu às últimas requisições de telemetria, sonda apenas a 1 Hz
        // para não desperdiçar 35ms de janela de rádio a cada N comandos
        if (!tel_state.is_online) {
            auto ms_since_last_tel = std::chrono::duration_cast<std::chrono::milliseconds>(
                                         now - tel_state.last_tel_attempt)
                                         .count();
            if (ms_since_last_tel < OFFLINE_PROBE_INTERVAL_MS) {
                continue;
            }
        }

        tel_state.last_tel_attempt = now;
        if (request_and_receive_telemetry(robot_id)) {
            if (!tel_state.is_online) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "%s[RADIO] Robô %d voltou a responder telemetria!%s",
                    C_GREEN.c_str(),
                    robot_id,
                    C_RESET.c_str());
            }
            tel_state.is_online = true;
            tel_state.consecutive_losses = 0;
        } else {
            tel_state.consecutive_losses++;
            if (tel_state.consecutive_losses >= 2 && tel_state.is_online) {
                tel_state.is_online = false;
                RCLCPP_WARN(
                    this->get_logger(),
                    "%s[RADIO] Robô %d sem resposta de telemetria (comandos continuam normal; telemetria sondada a 1 Hz).%s",
                    C_YELLOW.c_str(),
                    robot_id,
                    C_RESET.c_str());
            }
        }
    }
}

bool Nrf24HardwareBridge::transmit_command(const struct RobotCommand& cmd)
{
    return send_robot_command_packet(serial_fd_, cmd, get_active_guard_us());
}

bool Nrf24HardwareBridge::request_and_receive_telemetry(uint8_t robot_id)
{
    struct RobotTelemetry telemetry = {};
    double rtt_ms = 0.0;

    if (send_telemetry_request_and_wait(
            serial_fd_, robot_id, &telemetry, &rtt_ms, TELEMETRY_TIMEOUT_MS)) {
        publish_telemetry(telemetry);
        RCLCPP_DEBUG(
            this->get_logger(),
            "✓ [TEL] Robô %d | RTT: %.1fms | Bateria: %d%% | Kicker: %.2fV",
            telemetry.header.robot_id,
            rtt_ms,
            telemetry.battery_percentage,
            telemetry.kicker_voltage / 100.0);
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

bool Nrf24HardwareBridge::open_serial_port(const std::string& port_name)
{
    serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        return false;
    }

    fcntl(serial_fd_, F_SETFL, 0);

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        return false;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        return false;
    }

    struct serial_struct ser_info;
    if (ioctl(serial_fd_, TIOCGSERIAL, &ser_info) == 0) {
        ser_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(serial_fd_, TIOCSSERIAL, &ser_info);
    }

    tcflush(serial_fd_, TCIOFLUSH);
    return true;
}

void Nrf24HardwareBridge::configure_nrf24()
{
    RCLCPP_INFO(this->get_logger(), "Configurando módulo USB nRF24L01 via comandos AT...");

    std::string rx_addr = g_override_rx_addr.empty()
                              ? this->get_parameter("radio_rx_address").as_string()
                              : g_override_rx_addr;
    std::string tx_addr = g_override_tx_addr.empty()
                              ? this->get_parameter("radio_tx_address").as_string()
                              : g_override_tx_addr;
    double freq = (g_override_freq > 0.0) ? g_override_freq
                                          : this->get_parameter("radio_frequency").as_double();
    int rate =
        (g_override_rate > 0) ? g_override_rate : this->get_parameter("radio_rate").as_int();

    if (rx_addr.length() != 5 || tx_addr.length() != 5) {
        RCLCPP_ERROR(
            this->get_logger(), "Endereços RX e TX devem conter exatamente 5 caracteres.");
        return;
    }

    auto format_address = [](const std::string& addr) {
        std::ostringstream oss;
        for (size_t i = 0; i < addr.length(); ++i) {
            oss << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(static_cast<uint8_t>(addr[i]));
            if (i < addr.length() - 1) oss << ",";
        }
        return oss.str();
    };

    char freq_cmd[32];
    std::snprintf(freq_cmd, sizeof(freq_cmd), "AT+FREQ=%.3f", freq);
    std::string rxa_cmd = "AT+RXA=" + format_address(rx_addr);
    std::string txa_cmd = "AT+TXA=" + format_address(tx_addr);
    std::string rate_cmd = "AT+RATE=" + std::to_string(rate);

    send_at_and_translate(serial_fd_, "AT+BAUD=7", 400);
    usleep(100000);
    send_at_and_translate(serial_fd_, rxa_cmd, 500);
    usleep(100000);
    send_at_and_translate(serial_fd_, txa_cmd, 500);
    usleep(100000);
    send_at_and_translate(serial_fd_, freq_cmd, 400);
    usleep(100000);
    send_at_and_translate(serial_fd_, rate_cmd, 400);

    // Aguarda estabilização do MCU STC15 do dongle antes de iniciar envio de pacotes binários
    usleep(250000);
    tcflush(serial_fd_, TCIOFLUSH);
    RCLCPP_INFO(
        this->get_logger(),
        "Configuração AT concluída (Freq: %.3f GHz | Rate: %d | RX: %s | TX: %s).",
        freq,
        rate,
        rx_addr.c_str(),
        tx_addr.c_str());
}

// ============================================================================
// 8. PONTO DE ENTRADA (MODO ROS 2, CLI INTERATIVO, BENCHMARK E ONE-SHOT)
// ============================================================================

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    bool cli_mode = false;
    bool benchmark_mode = false;
    std::string bench_target_spec = "3";
    double bench_duration = 10.0;
    int bench_tel_interval = 5;

    bool one_shot_config = false;
    uint8_t oneshot_robot_id = 0;
    uint8_t oneshot_flags = 0;
    uint16_t oneshot_param = 0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--safe_ack" || arg == "--safe-ack") {
            g_safe_ack_mode = true;
        } else if (arg == "--cli" || arg == "--config") {
            cli_mode = true;
        } else if (arg == "--benchmark" || arg == "--bench") {
            benchmark_mode = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                bench_target_spec = argv[++i];
            }
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                bench_duration = std::stod(argv[++i]);
            }
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                bench_tel_interval = std::stoi(argv[++i]);
            }
        } else if (arg == "--skip-config") {
            g_skip_at_config = true;
        } else if ((arg == "--freq" || arg == "-f") && i + 1 < argc) {
            g_override_freq = std::stod(argv[++i]);
        } else if ((arg == "--rate" || arg == "-r") && i + 1 < argc) {
            g_override_rate = std::stoi(argv[++i]);
        } else if ((arg == "--address" || arg == "-a") && i + 1 < argc) {
            g_override_tx_addr = argv[++i];
        } else if (arg == "--set-id" && i + 2 < argc) {
            one_shot_config = true;
            oneshot_robot_id = parse_robot_id_token(argv[++i]);
            oneshot_flags = CONFIG_FLAG_SET_ID;
            oneshot_param = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--calibrate" && i + 1 < argc) {
            one_shot_config = true;
            oneshot_robot_id = parse_robot_id_token(argv[++i]);
            oneshot_flags = CONFIG_FLAG_RUN_CALIBRATION;
        } else if (arg == "--calibrate-mag" && i + 1 < argc) {
            one_shot_config = true;
            oneshot_robot_id = parse_robot_id_token(argv[++i]);
            oneshot_flags = CONFIG_FLAG_CALIBRATE_MAG;
        } else if (arg == "--reset-nvs" && i + 1 < argc) {
            one_shot_config = true;
            oneshot_robot_id = parse_robot_id_token(argv[++i]);
            oneshot_flags = CONFIG_FLAG_RESET_NVS;
        } else if (arg == "--factory-reset" && i + 1 < argc) {
            one_shot_config = true;
            oneshot_robot_id = parse_robot_id_token(argv[++i]);
            oneshot_flags = CONFIG_FLAG_FACTORY_RESET;
        } else if (arg == "--help" || arg == "-h") {
            print_cli_help();
            std::cout
                << "\nFlags de linha de comando para ros2 run oxebots_comms nrf24_bridge_node:\n"
                << "  --cli / --config                            Abre o terminal interativo CLI\n"
                << "  --benchmark [robos] [duracao_s] [tel_int]   Executa o benchmark de rádio (ex: --benchmark 3 10 5)\n"
                << "  --safe_ack / --safe-ack                     Usa guarda de 22ms entre pacotes\n"
                << "  --set-id <id|broadcast> <new_id>            Altera o ID do robô e encerra\n"
                << "  --calibrate <id|broadcast>                  Inicia calibração completa e encerra\n"
                << "  --calibrate-mag <id|broadcast>              Inicia calibração do magnetômetro e encerra\n"
                << "  --reset-nvs <id|broadcast>                  Reseta NVS do robô e encerra\n"
                << "  --factory-reset <id|broadcast>              Apaga toda a NVS do robô e encerra\n"
                << "  --freq <GHz>                                Define a frequência (padrão: 2.476)\n"
                << "  --rate <1|2|3>                              Define a taxa de dados (1=250k, 2=1M, 3=2M)\n"
                << "  --address <5_chars>                         Define o TX address (padrão: ESP32)\n"
                << "  --skip-config                               Pula o envio de comandos AT no início\n";
            rclcpp::shutdown();
            return 0;
        }
    }

    try {
        const bool standalone_mode = (one_shot_config || benchmark_mode || cli_mode);
        auto node = std::make_shared<Nrf24HardwareBridge>(!standalone_mode);

        if (standalone_mode) {
            if (g_active_serial_fd >= 0) {
                if (one_shot_config) {
                    send_robot_config_packet(
                        g_active_serial_fd, oneshot_robot_id, oneshot_flags, oneshot_param);
                }
                if (benchmark_mode) {
                    run_benchmark(
                        g_active_serial_fd,
                        bench_target_spec,
                        bench_duration,
                        bench_tel_interval,
                        get_active_guard_us());
                }
                if (cli_mode) {
                    run_interactive_cli(g_active_serial_fd);
                }
            }
        } else {
            rclcpp::spin(node);
        }
    } catch (const std::exception& e) {
        std::cerr << "Fatal Error : " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
