#include "oxebots_comms/Nrf24.hpp"
#include <rclcpp/logging.hpp>
#include <libserialport.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <sys/ioctl.h>
#include <linux/serial.h>

namespace {
    // Cores ANSI pro terminal
    const std::string C_RED     = "\033[91m";
    const std::string C_GREEN   = "\033[92m";
    const std::string C_YELLOW  = "\033[93m";
    const std::string C_BLUE    = "\033[94m";
    const std::string C_CYAN    = "\033[96m";
    const std::string C_BOLD    = "\033[1m";
    const std::string C_RESET   = "\033[0m";

    // estado do robô para ajuste de banda 
    struct RobotRadioState {
        int consecutive_losses = 0;
        std::chrono::steady_clock::time_point last_try_time = std::chrono::steady_clock::now();
        bool is_online = true;
    };

    std::unordered_map<uint8_t, RobotRadioState> robot_states_;
    std::chrono::steady_clock::time_point last_pub_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_warn_time_ = std::chrono::steady_clock::now();

    double measured_ros_freq_hz_ = 10.0;
    double avg_robot_rtt_ms_ = 21.0; // atualiza de acordo com a resposta dos robos
    int dynamic_timeout_ms_ = 24;

    // cte de tempo e proteção
    constexpr int MIN_SAFE_TIMEOUT_MS = 22;         // min pra cobrir o latency_timer USB
    constexpr int MAX_SAFE_TIMEOUT_MS = 35;         // folga max em freq baixas
    constexpr int FAST_GUARD_US = 4000;             // 4ms: min pro Dongle não concatenar pacotes UART
    constexpr int SAFE_ACK_GUARD_US = 20000;        // 20ms: modo --safe_ack (espera o vTaskDelay(1) do ESP32)
    constexpr int TIMEOUT_RECOVERY_US = 15000;      // 15ms: delay caso o pacote estoure MAX_RT sem ACK
    constexpr int OFFLINE_PROBE_INTERVAL_MS = 1000; // robôs offline são sondados a 1 Hz

    // flags do CLI
    bool g_safe_ack_mode = false;
    bool g_skip_at_config = false;
    double g_override_freq = -1.0;
    int g_override_rate = -1;
    std::string g_override_tx_addr = "";
    std::string g_override_rx_addr = "";
    int g_active_serial_fd = -1; // guarda ref do fd aberto pelo nó para uso no modo CLI

    // Retorna o tempo de guarda ativo (4ms no modo rápido, 20ms com --safe_ack)
    inline int get_active_guard_us() {
        return g_safe_ack_mode ? SAFE_ACK_GUARD_US : FAST_GUARD_US;
    }

    // Envia um pacote RobotConfig (8 bytes) via descritor de ficheiro serial
    bool send_robot_config_packet(int fd, uint8_t robot_id, uint8_t config_flags, uint16_t param = 0) {
        struct RobotConfig cfg = {};
        cfg.header.msg_type = MSG_TYPE_CONFIG;
        cfg.header.robot_id = robot_id;
        cfg.header.timestamp = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count() & 0xFFFFFFFF);
        cfg.config_flags = config_flags;
        cfg.param = param;

        uint8_t buf[BYTES_LENGTH_ROBOT_CONFIG];
        EncodeRobotConfig(&cfg, buf);

        tcflush(fd, TCIFLUSH);
        ssize_t written = write(fd, buf, BYTES_LENGTH_ROBOT_CONFIG);
        tcdrain(fd);
        usleep(25000);
        tcflush(fd, TCIFLUSH);

        std::string id_str = (robot_id == ROBOT_ID_BROADCAST) ? "BROADCAST(15)" : std::to_string(robot_id);
        std::cout << C_CYAN << "[CONFIG ENVIADO] robot_id=" << id_str
                  << " | flags=0x" << std::hex << (int)config_flags << std::dec
                  << " | param=" << param << C_RESET << std::endl;
        return (written == BYTES_LENGTH_ROBOT_CONFIG);
    }

    // envia AT e traduz a resposta  (map do GB2312 -> Inglês)
    std::string send_at_and_translate(int fd, const std::string& cmd, int wait_ms = 500) {
        tcflush(fd, TCIOFLUSH);
        std::string full_cmd = cmd + "\r\n";
        ssize_t w = write(fd, full_cmd.c_str(), full_cmd.size());
        (void)w;
        tcdrain(fd);

        std::vector<uint8_t> resp;
        auto start = std::chrono::steady_clock::now();
        struct pollfd fds[1] = {{fd, POLLIN, 0}};

        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(wait_ms)) {
            int ret = poll(fds, 1, 40);
            if (ret > 0 && (fds[0].revents & POLLIN)) {
                uint8_t buf[256];
                int n = read(fd, buf, sizeof(buf));
                if (n > 0) resp.insert(resp.end(), buf, buf + n);
            } else if (ret == 0 && !resp.empty()) {
                break;
            }
        }

        struct GbMap { std::vector<uint8_t> seq; const char* eng; };
        const std::vector<GbMap> dict = {
            {{0xCF, 0xB5, 0xCD, 0xB3, 0xD0, 0xC5, 0xCF, 0xA2}, "System Information"},
            {{0xCD, 0xA8, 0xD1, 0xB6, 0xB2, 0xA8, 0xCC, 0xD8, 0xC2, 0xCA, 0xC9, 0xE8, 0xD6, 0xC3, 0xB3, 0xC9, 0xB9, 0xA6}, "Baud rate setting successful"},
            {{0xB4, 0xAB, 0xCA, 0xE4, 0xCB, 0xD9, 0xC2, 0xCA, 0xC9, 0xE8, 0xD6, 0xC3, 0xB3, 0xC9, 0xB9, 0xA6}, "Data rate setting successful"},
            {{0xB5, 0xD8, 0xD6, 0xB7, 0xC9, 0xE8, 0xD6, 0xC3, 0xB3, 0xC9, 0xB9, 0xA6}, "Address setting successful"},
            {{0xB5, 0xCD, 0xD4, 0xEB, 0xC9, 0xF9, 0xB7, 0xC5, 0xB4, 0xF3, 0xD4, 0xF6, 0xD2, 0xE6}, "LNA Gain"},
            {{0xBF, 0xD5, 0xD6, 0xD0, 0xB4, 0xAB, 0xCA, 0xE4, 0xCB, 0xD9, 0xC2, 0xCA}, "Air Data Rate"},
            {{0xB1, 0xBE, 0xB5, 0xD8, 0xBD, 0xD3, 0xCA, 0xD5, 0xB5, 0xD8, 0xD6, 0xB7}, "Local Receive Address"},
            {{0xC9, 0xE8, 0xD6, 0xC3, 0xB3, 0xC9, 0xB9, 0xA6}, "Setting successful"},
            {{0xC4, 0xBF, 0xB1, 0xEA, 0xB5, 0xD8, 0xD6, 0xB7}, "Target Address"},
            {{0xCD, 0xA8, 0xD1, 0xB6, 0xC6, 0xB5, 0xC2, 0xCA}, "Communication Frequency"},
            {{0xD0, 0xA3, 0xD1, 0xE9, 0xC4, 0xA3, 0xCA, 0xBD}, "Check Mode"},
            {{0xB7, 0xA2, 0xC9, 0xE4, 0xB9, 0xA6, 0xC2, 0xCA}, "Transmit Power"},
            {{0xB4, 0xAB, 0xCA, 0xE4, 0xCB, 0xD9, 0xC2, 0xCA}, "Data Rate"},
            {{0xB2, 0xA8, 0xCC, 0xD8, 0xC2, 0xCA}, "Baud Rate"},
            {{0xD0, 0xA3, 0xD1, 0xE9}, " Check"},
            {{0xBF, 0xAA, 0xC6, 0xF4}, "Enabled"},
            {{0xB9, 0xD8, 0xB1, 0xD5}, "Disabled"},
            {{0xB3, 0xC9, 0xB9, 0xA6}, "successful"},
            {{0xCE, 0xBB}, " bits "}
        };

        std::string out;
        size_t i = 0;
        while (i < resp.size()) {
            bool matched = false;
            for (const auto& item : dict) {
                if (i + item.seq.size() <= resp.size() &&
                    std::equal(item.seq.begin(), item.seq.end(), resp.begin() + i)) {
                    out += item.eng;
                    i += item.seq.size();
                    matched = true;
                    break;
                }
            }
            if (!matched) {
                if (resp[i] >= 32 || resp[i] == '\r' || resp[i] == '\n' || resp[i] == '\t') {
                    out += static_cast<char>(resp[i]);
                }
                i++;
            }
        }
        return out;
    }

    uint8_t parse_robot_id_token(const std::string& token) {
        if (token == "broadcast" || token == "BROADCAST" || token == "15") {
            return ROBOT_ID_BROADCAST;
        }
        return static_cast<uint8_t>(std::stoi(token));
    }
}

std::string Find_Nrf24_port(uint16_t port_pid, uint16_t port_vid){
    int port_vid_search, port_pid_search;
    std::string port_name_string = "null";

    std::cout << "Searching for NRF24 on USB ports " << std::endl;
    struct sp_port **port_list;
    if (sp_list_ports(&port_list) != SP_OK) return "";

    for (int i = 0; port_list[i] != NULL; i++) {
        if (sp_get_port_usb_vid_pid(port_list[i], &port_vid_search, &port_pid_search) == SP_OK) {
            if (port_pid_search == port_pid && port_vid_search == port_vid) {
                port_name_string = sp_get_port_name(port_list[i]);
                std::cout << "Found NRF24 on port: " << port_name_string << std::endl;
                break;
            }
        }
    }
    sp_free_port_list(port_list);
    return port_name_string;
}

Nrf24HardwareBridge::Nrf24HardwareBridge() : Node("nrf24_hardware_bridge")
{
    this->declare_parameter("radio_rx_address", "ADMIN");
    this->declare_parameter("radio_tx_address", "ESP32");
    this->declare_parameter("radio_frequency", 2.476);
    this->declare_parameter("radio_rate", 3);

    if (!g_override_rx_addr.empty()) this->set_parameter(rclcpp::Parameter("radio_rx_address", g_override_rx_addr));
    if (!g_override_tx_addr.empty()) this->set_parameter(rclcpp::Parameter("radio_tx_address", g_override_tx_addr));
    if (g_override_freq > 0.0)       this->set_parameter(rclcpp::Parameter("radio_frequency", g_override_freq));
    if (g_override_rate > 0)         this->set_parameter(rclcpp::Parameter("radio_rate", g_override_rate));

    telemetry_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotTelemetry>("/robot_telemetry", 10);
    command_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotCmd>(
        "/robot_commands",
        rclcpp::SensorDataQoS(),
        std::bind(&Nrf24HardwareBridge::send_command, this, std::placeholders::_1)
    );

    const uint16_t port_vid = 0x1A86, port_pid = 0x7523;
    std::string port_name_string = Find_Nrf24_port(port_pid, port_vid);

    if (port_name_string == "null" || port_name_string.empty()) {
        RCLCPP_ERROR(this->get_logger(), "NRF24 not found on USB ports");
        throw std::runtime_error("NRF24 not found");
    }

    if (!open_serial_port(port_name_string)) {
        throw std::runtime_error("Failed to open and configure serial port");
    }

    g_active_serial_fd = serial_fd_;

    if (!g_skip_at_config) {
        this->configure_nrf24();
    } else {
        RCLCPP_WARN(this->get_logger(), "Skipping AT radio config (--skip-config active).");
    }

    if (g_safe_ack_mode) {
        RCLCPP_WARN(this->get_logger(), "[MODO SAFE_ACK ATIVO] Intervalo mínimo de 20ms entre mensagens garantido!");
    } else {
        RCLCPP_INFO(this->get_logger(), "[MODO FAST ATIVO] Delay mínimo (4ms). Use --safe_ack se testar com apenas 1 robô.");
    }
}

Nrf24HardwareBridge::~Nrf24HardwareBridge() {
    if (serial_fd_ >= 0) close(serial_fd_);
}

bool Nrf24HardwareBridge::open_serial_port(const std::string& port_name) {
    serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", port_name.c_str());
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) return false;

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    cfmakeraw(&tty);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;

    struct serial_struct ser_info;
    if (ioctl(serial_fd_, TIOCGSERIAL, &ser_info) == 0) {
        ser_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(serial_fd_, TIOCSSERIAL, &ser_info);
    }

    return true;
}

void Nrf24HardwareBridge::send_command(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg) {
    if (msg->robots.empty()) return;

    auto now = std::chrono::steady_clock::now();
    double dt_sec = std::chrono::duration<double>(now - last_pub_time_).count();
    last_pub_time_ = now;

    // mede a frq de chegada no topico
    if (dt_sec > 0.001 && dt_sec < 2.0) {
        double instant_freq = 1.0 / dt_sec;
        measured_ros_freq_hz_ = 0.2 * instant_freq + 0.8 * measured_ros_freq_hz_;
    }

    // conta os robos online
    int online_count = 0;
    for (const auto & r : msg->robots) {
        if (robot_states_[r.id].is_online) online_count++;
    }
    int active_slots = std::max(1, online_count);

    // calc tempo min radio
    double guard_ms = get_active_guard_us() / 1000.0;
    double slot_time_ms = avg_robot_rtt_ms_ + guard_ms;
    double max_radio_freq_hz = 1000.0 / (active_slots * slot_time_ms);

    double cycle_budget_ms = 1000.0 / measured_ros_freq_hz_;
    double ideal_timeout_ms = (cycle_budget_ms / active_slots) - guard_ms;

    if (measured_ros_freq_hz_ > max_radio_freq_hz * 1.15) {
        dynamic_timeout_ms_ = MIN_SAFE_TIMEOUT_MS;
        if (std::chrono::duration<double>(now - last_warn_time_).count() > 2.0) {
            RCLCPP_WARN(this->get_logger(),
                "[TETO DO RADIO] Tópico a %.1f Hz (%d robôs online) excede o teto físico (~%.1f Hz | %.1fms/robô). "
                "Limitando ao teto máximo seguro (timeout=%dms)!",
                measured_ros_freq_hz_, online_count, max_radio_freq_hz, slot_time_ms, dynamic_timeout_ms_);
            last_warn_time_ = now;
        }
    } else {
        dynamic_timeout_ms_ = std::clamp(static_cast<int>(ideal_timeout_ms), MIN_SAFE_TIMEOUT_MS, MAX_SAFE_TIMEOUT_MS);
    }

    // manda pros robis online e sonda no máximo 1 robô off por ciclo (a 1 Hz)
    bool probed_offline_in_this_cycle = false;

    for (const auto & robot_ros : msg->robots) {
        auto& st = robot_states_[robot_ros.id];
        auto now_loop = std::chrono::steady_clock::now();
        auto since_last_try = std::chrono::duration_cast<std::chrono::milliseconds>(now_loop - st.last_try_time).count();

        if (!st.is_online) {
            if (probed_offline_in_this_cycle || since_last_try < OFFLINE_PROBE_INTERVAL_MS) {
                continue;
            }
            probed_offline_in_this_cycle = true;
        }

        struct RobotCommand cmd_bitproto = {};
        cmd_bitproto.header.msg_type = MSG_TYPE_COMMAND;
        cmd_bitproto.header.robot_id = robot_ros.id;
        cmd_bitproto.header.timestamp = (this->now().nanoseconds() / 1000000) & 0xFFFFFFFF;
        cmd_bitproto.target_pose.x_v = static_cast<int16_t>(robot_ros.x_velocity * 1000.0);
        cmd_bitproto.target_pose.y_v = static_cast<int16_t>(robot_ros.y_velocity * 1000.0);
        cmd_bitproto.target_pose.angular_vel = static_cast<int16_t>(robot_ros.angular_velocity * 100.0);
        cmd_bitproto.kick_velocity = robot_ros.kick_speed;

        uint8_t buffer[BYTES_LENGTH_ROBOT_COMMAND];
        EncodeRobotCommand(&cmd_bitproto, buffer);

        st.last_try_time = std::chrono::steady_clock::now();
        bool ok = send_and_wait_ack(buffer, robot_ros.id);

        if (ok) {
            if (!st.is_online) {
                RCLCPP_INFO(this->get_logger(), "[ONLINE] Robô %d voltou a responder! Restaurando frequência máxima.", robot_ros.id);
            }
            st.consecutive_losses = 0;
            st.is_online = true;
        } else {
            st.consecutive_losses++;
            // se falhar 2 vezes no modo rápido (ou 1 vez no modo safe_ack) vira off
            int loss_threshold = g_safe_ack_mode ? 1 : 2;
            if (st.is_online && st.consecutive_losses >= loss_threshold) {
                st.is_online = false;
                RCLCPP_WARN(this->get_logger(),
                    "[BACKOFF] Robô %d sem resposta. Reduzindo para sondagem a 1 Hz e priorizando os robôs ativos!",
                    robot_ros.id);
            }
        }
    }
}

bool Nrf24HardwareBridge::send_and_wait_ack(const uint8_t* tx_buffer, uint8_t robot_id) {
    tcflush(serial_fd_, TCIFLUSH);

    if (write(serial_fd_, tx_buffer, BYTES_LENGTH_ROBOT_COMMAND) != BYTES_LENGTH_ROBOT_COMMAND) {
        RCLCPP_ERROR(this->get_logger(), "Erro ao escrever na porta série.");
        return false;
    }
    tcdrain(serial_fd_);

    auto tx_start_time = std::chrono::steady_clock::now();
    std::vector<uint8_t> rx_accum;
    const uint8_t trash_return[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};

    struct pollfd fds[1];
    fds[0].fd = serial_fd_;
    fds[0].events = POLLIN;

    bool dongle_ack = false;
    bool telemetry_ok = false;

    while (std::chrono::steady_clock::now() - tx_start_time < std::chrono::milliseconds(dynamic_timeout_ms_)) {
        int ret = poll(fds, 1, 1);

        if (ret > 0 && (fds[0].revents & POLLIN)) {
            uint8_t buf[64];
            int n = read(serial_fd_, buf, sizeof(buf));
            if (n > 0) {
                rx_accum.insert(rx_accum.end(), buf, buf + n);
            }

            auto it = std::search(rx_accum.begin(), rx_accum.end(), trash_return, trash_return + 6);
            if (it != rx_accum.end()) {
                dongle_ack = true;
                rx_accum.erase(it, it + 6);
            }

            // Verifica telemetria (22 bytes)
            while (rx_accum.size() >= BYTES_LENGTH_ROBOT_TELEMETRY) {
                uint8_t raw_msg_type = rx_accum[0] & 0x0F;
                uint8_t raw_robot_id = (rx_accum[0] >> 4) & 0x0F;

                if (raw_msg_type == MSG_TYPE_TELEMETRY && raw_robot_id <= ROBOT_ID_MAX) {
                    struct RobotTelemetry telemetry_bp;
                    DecodeRobotTelemetry(&telemetry_bp, rx_accum.data());
                    publish_telemetry(telemetry_bp);

                    auto rtt = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - tx_start_time).count();

                    avg_robot_rtt_ms_ = 0.2 * static_cast<double>(rtt) + 0.8 * avg_robot_rtt_ms_;

                    RCLCPP_INFO(this->get_logger(), "[OK] ACK Robô %d (%ldms | Freq: %.1fHz) | Batt: %d%% | Kick: %.2fV",
                                raw_robot_id, rtt, measured_ros_freq_hz_,
                                telemetry_bp.battery_percentage, telemetry_bp.kicker_voltage / 100.0f);

                    rx_accum.erase(rx_accum.begin(), rx_accum.begin() + BYTES_LENGTH_ROBOT_TELEMETRY);
                    if (raw_robot_id == robot_id) {
                        telemetry_ok = true;
                        break;
                    }
                } else {
                    rx_accum.erase(rx_accum.begin());
                }
            }

            if (telemetry_ok) break;
        }
    }

    // Aplicação do delay entre mensagens:
    // - Modo normal: espera apenas 4ms (FAST_GUARD_US) para não concatenar pacotes na UART.
    // - Modo --safe_ack: espera 20ms (SAFE_ACK_GUARD_US) para garantir que o ESP32 saiu do vTaskDelay(1).
    if (telemetry_ok || dongle_ack) {
        usleep(get_active_guard_us());
    } else {
        // Se nem o dongle deu ACK (MAX_RT), espera o tempo de recuperação antes de limpar a serial
        usleep( std::max(get_active_guard_us(), TIMEOUT_RECOVERY_US) );
        tcflush(serial_fd_, TCIFLUSH);
    }

    if (!telemetry_ok) {
        RCLCPP_WARN(this->get_logger(), "[TIMEOUT] Sem telemetria do Robô %d em %dms", robot_id, dynamic_timeout_ms_);
        return false;
    }

    return true;
}

void Nrf24HardwareBridge::publish_telemetry(const struct RobotTelemetry& telemetry_bp)
{
    auto msg = oxebots_interfaces::msg::RobotTelemetry();
    msg.robot_id = telemetry_bp.header.robot_id;
    msg.timestamp = telemetry_bp.header.timestamp;
    msg.x = telemetry_bp.robot_pose.x;
    msg.y = telemetry_bp.robot_pose.y;
    msg.orientation = telemetry_bp.robot_pose.angle / 100.0f;
    msg.battery_percentage = telemetry_bp.battery_percentage;
    msg.kicker_voltage = telemetry_bp.kicker_voltage / 100.0f;
    msg.error_flags = telemetry_bp.error_flags;

    telemetry_pub_->publish(msg);
}

void Nrf24HardwareBridge::configure_nrf24() {
    RCLCPP_INFO(this->get_logger(), "Starting AT radio config...");

    std::string rx_param = this->get_parameter("radio_rx_address").as_string();
    std::string tx_param = this->get_parameter("radio_tx_address").as_string();
    double freq_param = this->get_parameter("radio_frequency").as_double();
    int rate_param = this->get_parameter("radio_rate").as_int();

    auto format_address = [](const std::string& addr) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "0x%02X,0x%02X,0x%02X,0x%02X,0x%02X",
                 addr[0], addr[1], addr[2], addr[3], addr[4]);
        return std::string(buffer);
    };

    int f_int = static_cast<int>(freq_param);
    int f_frac = static_cast<int>(std::round((freq_param - f_int) * 1000.0));
    char freq_buf[32];
    snprintf(freq_buf, sizeof(freq_buf), "AT+FREQ=%d.%03d\r\n", f_int, f_frac);

    std::vector<std::string> at_commands = {
        "AT+RXA=" + format_address(rx_param) + "\r\n",
        "AT+TXA=" + format_address(tx_param) + "\r\n",
        std::string(freq_buf),
        "AT+RATE=" + std::to_string(rate_param) + "\r\n"
    };

    const std::vector<uint8_t> success_pattern = {0xB3, 0xC9, 0xB9, 0xA6};

    usleep(50000);
    tcflush(serial_fd_, TCIOFLUSH);

    for (const std::string& cmd : at_commands) {
        ssize_t written = write(serial_fd_, cmd.c_str(), cmd.length());
        (void)written;
        tcdrain(serial_fd_);

        int timeout_ms = (cmd.find("RXA") != std::string::npos || cmd.find("TXA") != std::string::npos) ? 3000 : 500;
        auto start_time = std::chrono::steady_clock::now();
        std::vector<uint8_t> resp;
        bool success = false;

        struct pollfd fds[1];
        fds[0].fd = serial_fd_;
        fds[0].events = POLLIN;

        while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(timeout_ms)) {
            int ret = poll(fds, 1, 30);
            if (ret > 0 && (fds[0].revents & POLLIN)) {
                uint8_t buf[256];
                int n = read(serial_fd_, buf, sizeof(buf));
                if (n > 0) {
                    resp.insert(resp.end(), buf, buf + n);
                    if (std::search(resp.begin(), resp.end(), success_pattern.begin(), success_pattern.end()) != resp.end()) {
                        success = true;
                    }
                }
            } else if (ret == 0 && success) {
                break;
            }
        }

        std::string cmd_name = cmd.substr(0, cmd.find('='));
        if (success) {
            RCLCPP_INFO(this->get_logger(), "%s configured successfully.", cmd_name.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Warning: %s didn't return success", cmd_name.c_str());
        }
        tcflush(serial_fd_, TCIOFLUSH);
        usleep(20000);
    }
    RCLCPP_INFO(this->get_logger(), "Radio configured successfully!");
}

// pra vc que tá vendo o código, esse menu foi feito em cima do script em python Nrf24.py
// e em cime do bitproto (oxebots_interface)
// falta implementar um histórico de comandos

namespace {
    void print_cli_help() {
        std::cout << "\n" << C_BOLD << "=== OXEBOTS C++ RADIO CLI & CONFIGURATOR ===" << C_RESET << "\n"
                  << "Comandos no terminal interativo:\n"
                  << "  " << C_CYAN << "{x:<mm>, y:<mm>, vx:<mm/s>, vy:<mm/s>, w:<0.01rad/s>, kick:<0-255>, id:<id>}" << C_RESET << " — envia RobotCommand\n"
                  << "  " << C_GREEN << "set_id <target_id|broadcast> <new_id>" << C_RESET << "       — altera o ID do robô na NVS\n"
                  << "  " << C_GREEN << "calibrate [<robot_id>|broadcast]" << C_RESET << "            — calibra encoders + IMU + PID\n"
                  << "  " << C_GREEN << "calibrate_mag [<robot_id>|broadcast]" << C_RESET << "        — calibra o magnetómetro\n"
                  << "  " << C_GREEN << "reset_nvs [<robot_id>|broadcast]" << C_RESET << "            — reseta configs da NVS para padrão\n"
                  << "  " << C_GREEN << "factory_reset [<robot_id>|broadcast]" << C_RESET << "        — apaga toda a NVS e reinicia\n"
                  << "  " << C_YELLOW << "freq <GHz>" << C_RESET << "                                  — muda frequência do dongle (ex: freq 2.476)\n"
                  << "  " << C_YELLOW << "rate <1|2|3>" << C_RESET << "                                — muda taxa de ar (1=250K, 2=1M, 3=2Mbps)\n"
                  << "  " << C_YELLOW << "info / at?" << C_RESET << "                                  — exibe configurações AT atuais do dongle\n"
                  << "  " << C_BOLD << "help" << C_RESET << "                                        — mostra esta ajuda\n"
                  << "  " << C_BOLD << "q / quit" << C_RESET << "                                    — sai do modo CLI\n";
    }

    void run_interactive_cli(int fd) {
        print_cli_help();
        std::string line;

        while (rclcpp::ok()) {
            std::cout << "\n" << C_BLUE << "Cmd > " << C_RESET << std::flush;
            if (!std::getline(std::cin, line)) break;

            line.erase(0, line.find_first_not_of(" \t\r\n"));
            if (line.empty()) continue;
            line.erase(line.find_last_not_of(" \t\r\n") + 1);

            std::stringstream ss(line);
            std::string cmd;
            ss >> cmd;

            if (cmd == "q" || cmd == "quit" || cmd == "exit") break;
            if (cmd == "help") { print_cli_help(); continue; }

            if (cmd == "info" || cmd == "at?" || cmd == "AT?") {
                std::cout << C_GREEN << send_at_and_translate(fd, "AT?", 600) << C_RESET << std::endl;
                continue;
            }

            if (cmd == "freq") {
                double f = 2.476;
                if (!(ss >> f) || f < 2.400 || f > 2.525) {
                    std::cout << C_RED << "Uso: freq <2.400 a 2.525>" << C_RESET << std::endl;
                    continue;
                }
                char fbuf[32];
                int f_int = static_cast<int>(f);
                int f_frac = static_cast<int>(std::round((f - f_int) * 1000.0));
                snprintf(fbuf, sizeof(fbuf), "AT+FREQ=%d.%03d", f_int, f_frac);
                std::cout << C_GREEN << send_at_and_translate(fd, fbuf, 500) << C_RESET << std::endl;
                continue;
            }

            if (cmd == "rate") {
                int r = 3;
                if (!(ss >> r) || r < 1 || r > 3) {
                    std::cout << C_RED << "Uso: rate <1=250Kbps | 2=1Mbps | 3=2Mbps>" << C_RESET << std::endl;
                    continue;
                }
                std::cout << C_GREEN << send_at_and_translate(fd, "AT+RATE=" + std::to_string(r), 500) << C_RESET << std::endl;
                continue;
            }

            if (cmd == "set_id") {
                std::string target_tok; int new_id = 0;
                if (!(ss >> target_tok >> new_id)) {
                    std::cout << C_RED << "Uso: set_id <target_id|broadcast> <new_id>" << C_RESET << std::endl;
                    continue;
                }
                send_robot_config_packet(fd, parse_robot_id_token(target_tok), CONFIG_FLAG_SET_ID, static_cast<uint16_t>(new_id));
                continue;
            }

            if (cmd == "calibrate_mag") {
                std::string tok = "0"; ss >> tok;
                send_robot_config_packet(fd, parse_robot_id_token(tok), CONFIG_FLAG_CALIBRATE_MAG);
                continue;
            }

            if (cmd == "calibrate") {
                std::string tok = "0"; ss >> tok;
                send_robot_config_packet(fd, parse_robot_id_token(tok), CONFIG_FLAG_RUN_CALIBRATION);
                continue;
            }

            if (cmd == "reset_nvs") {
                std::string tok = "0"; ss >> tok;
                send_robot_config_packet(fd, parse_robot_id_token(tok), CONFIG_FLAG_RESET_NVS);
                continue;
            }

            if (cmd == "factory_reset") {
                std::string tok = "0"; ss >> tok;
                send_robot_config_packet(fd, parse_robot_id_token(tok), CONFIG_FLAG_FACTORY_RESET);
                continue;
            }

            // Comando de movimento: {vx:500, vy:0, w:0, kick:0, id:0}
            int16_t x = 0, y = 0, vx = 0, vy = 0, w = 0;
            uint8_t kick = 0, robot_id = 0;

            std::string cleaned = line;
            cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '{'), cleaned.end());
            cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '}'), cleaned.end());

            std::stringstream pss(cleaned);
            std::string pair;
            while (std::getline(pss, pair, ',')) {
                auto colon = pair.find(':');
                if (colon != std::string::npos) {
                    std::string k = pair.substr(0, colon);
                    std::string v = pair.substr(colon + 1);
                    k.erase(0, k.find_first_not_of(" \t\"'"));
                    k.erase(k.find_last_not_of(" \t\"'") + 1);
                    v.erase(0, v.find_first_not_of(" \t\"'"));
                    v.erase(v.find_last_not_of(" \t\"'") + 1);
                    try {
                        if (k == "x") x = static_cast<int16_t>(std::stoi(v));
                        else if (k == "y") y = static_cast<int16_t>(std::stoi(v));
                        else if (k == "vx") vx = static_cast<int16_t>(std::stoi(v));
                        else if (k == "vy") vy = static_cast<int16_t>(std::stoi(v));
                        else if (k == "w") w = static_cast<int16_t>(std::stoi(v));
                        else if (k == "kick") kick = static_cast<uint8_t>(std::stoi(v));
                        else if (k == "id") robot_id = parse_robot_id_token(v);
                    } catch (...) {}
                }
            }

            struct RobotCommand cmd_bp = {};
            cmd_bp.header.msg_type = MSG_TYPE_COMMAND;
            cmd_bp.header.robot_id = robot_id;
            cmd_bp.header.timestamp = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count() & 0xFFFFFFFF);
            cmd_bp.target_pose.x = x;
            cmd_bp.target_pose.y = y;
            cmd_bp.target_pose.x_v = vx;
            cmd_bp.target_pose.y_v = vy;
            cmd_bp.target_pose.angular_vel = w;
            cmd_bp.kick_velocity = kick;

            uint8_t tx_buf[BYTES_LENGTH_ROBOT_COMMAND];
            EncodeRobotCommand(&cmd_bp, tx_buf);

            tcflush(fd, TCIFLUSH);
            ssize_t w_bytes = write(fd, tx_buf, BYTES_LENGTH_ROBOT_COMMAND);
            (void)w_bytes;
            tcdrain(fd);

            std::cout << C_CYAN << "→ [CMD] id=" << (int)robot_id << " vx=" << vx << " vy=" << vy
                      << " w=" << w << " kick=" << (int)kick << C_RESET << std::endl;

            if (robot_id != ROBOT_ID_BROADCAST) {
                auto t0 = std::chrono::steady_clock::now();
                std::vector<uint8_t> rx_buf;
                const uint8_t trash[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
                struct pollfd pfds[1] = {{fd, POLLIN, 0}};
                bool got = false;

                while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(200)) {
                    if (poll(pfds, 1, 2) > 0 && (pfds[0].revents & POLLIN)) {
                        uint8_t tmp[64];
                        int n = read(fd, tmp, sizeof(tmp));
                        if (n > 0) rx_buf.insert(rx_buf.end(), tmp, tmp + n);

                        auto it = std::search(rx_buf.begin(), rx_buf.end(), trash, trash + 6);
                        if (it != rx_buf.end()) rx_buf.erase(it, it + 6);

                        while (rx_buf.size() >= BYTES_LENGTH_ROBOT_TELEMETRY) {
                            if ((rx_buf[0] & 0x0F) == MSG_TYPE_TELEMETRY) {
                                struct RobotTelemetry tel;
                                DecodeRobotTelemetry(&tel, rx_buf.data());
                                auto rtt = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t0).count();
                                std::cout << C_GREEN << "✓ [RX] Robô " << (int)tel.header.robot_id
                                          << " (" << rtt << "ms) | Batt=" << (int)tel.battery_percentage
                                          << "% | Kicker=" << (tel.kicker_voltage / 100.0f) << "V"
                                          << " | Err=0x" << std::hex << tel.error_flags << std::dec << C_RESET << std::endl;
                                got = true;
                                break;
                            } else {
                                rx_buf.erase(rx_buf.begin());
                            }
                        }
                        if (got) break;
                    }
                }
                if (!got) std::cout << C_YELLOW << "✗ [TIMEOUT] Sem resposta em 200ms" << C_RESET << std::endl;
            }
        }
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    bool cli_mode = false;
    bool one_shot_config = false;
    uint8_t oneshot_robot_id = 0;
    uint8_t oneshot_flags = 0;
    uint16_t oneshot_param = 0;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--safe_ack" || arg == "--safe-ack") {
            g_safe_ack_mode = true;
        } else if (arg == "--cli" || arg == "--config") {
            cli_mode = true;
        } else if (arg == "--skip-config") {
            g_skip_at_config = true;
        } else if ((arg == "--freq" || arg == "-f") && i + 1 < argc) {
            g_override_freq = std::stod(argv[++i]);
        } else if ((arg == "--rate" || arg == "-r") && i + 1 < argc) {
            g_override_rate = std::stoi(argv[++i]);
        } else if ((arg == "--address" || arg == "-a") && i + 1 < argc) {
            g_override_tx_addr = argv[++i];
        } else if (arg == "--rx-address" && i + 1 < argc) {
            g_override_rx_addr = argv[++i];
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
            std::cout << "\nFlags de linha de comando para ros2 run oxebots_comms nrf24_bridge_node:\n"
                      << "  --safe_ack / --safe-ack              Garante espera de 20ms entre mensagens (teste com 1 robô)\n"
                      << "  --cli / --config                     Abre o terminal interativo CLI\n"
                      << "  --set-id <id|broadcast> <new_id>     Altera o ID do robô e encerra\n"
                      << "  --calibrate <id|broadcast>           Inicia calibração completa e encerra\n"
                      << "  --calibrate-mag <id|broadcast>       Inicia calibração do magnetómetro e encerra\n"
                      << "  --reset-nvs <id|broadcast>           Reseta NVS do robô e encerra\n"
                      << "  --factory-reset <id|broadcast>       Apaga toda a NVS do robô e encerra\n"
                      << "  --freq <GHz>                         Define a frequência (ex: 2.476)\n"
                      << "  --rate <1|2|3>                       Define a taxa de dados (1=250k, 2=1M, 3=2M)\n"
                      << "  --address <5_chars>                  Define o TX address (padrão: ESP32)\n"
                      << "  --skip-config                        Pula o envio de comandos AT no início\n";
            rclcpp::shutdown();
            return 0;
        }
    }

    try {
        auto node = std::make_shared<Nrf24HardwareBridge>();

        if (one_shot_config || cli_mode) {
            if (g_active_serial_fd >= 0) {
                if (one_shot_config) {
                    send_robot_config_packet(g_active_serial_fd, oneshot_robot_id, oneshot_flags, oneshot_param);
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