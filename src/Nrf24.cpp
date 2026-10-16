#include "oxebots_comms/Nrf24.hpp"
#include <rclcpp/logging.hpp>
#include <libserialport.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <chrono>

std::string Find_Nrf24_port(uint16_t port_pid, uint16_t port_vid){
    int port_vid_search, port_pid_search;
    char *port_name_pointer;
    std::string port_name_string = "null";

    std::cout << "Searching for NRF24 on USB ports " << std::endl;
    struct sp_port **port_list;
    if (sp_list_ports(&port_list) != SP_OK) return "";

    for (int i = 0; port_list[i] != NULL; i++) {
        if (sp_get_port_usb_vid_pid(port_list[i], &port_vid_search, &port_pid_search) == SP_OK) {
            if (port_pid_search == port_pid  && port_vid_search == port_vid ) {
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

    this->configure_nrf24();
    RCLCPP_INFO(this->get_logger(), "Nrf24 bridge node started and ready!");
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

    // Configuração Posix pura (115200 8N1)
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 data bits
    tty.c_cflag &= ~CRTSCTS;// No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

    // CRÍTICO: Modo Raw absoluto. Impede a tradução de 0x0A para 0x0D 0x0A.
    cfmakeraw(&tty);
    tty.c_cc[VMIN]  = 0; // Usaremos poll() para gerir timeouts
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;
    return true;
}

void Nrf24HardwareBridge::send_command(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg) {
    // A execução aqui é bloqueante e síncrona.
    // Como usamos SensorDataQoS, o ROS simplesmente descarta mensagens
    // intermédias se a estratégia publicar mais rápido do que o rádio consegue enviar.
    for (const auto & robot_ros : msg->robots) {
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

        send_and_wait_ack(buffer, robot_ros.id);
    }
}

bool Nrf24HardwareBridge::send_and_wait_ack(const uint8_t* tx_buffer, uint8_t robot_id) {
    // 1. Limpa o buffer de receção antes de enviar (evita desfasamento de pacotes antigos)
    tcflush(serial_fd_, TCIFLUSH);

    // 2. Envia os 30 bytes
    if (write(serial_fd_, tx_buffer, BYTES_LENGTH_ROBOT_COMMAND) != BYTES_LENGTH_ROBOT_COMMAND) {
        RCLCPP_ERROR(this->get_logger(), "Erro ao escrever na porta série.");
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "[TX] Comando enviado (Robô %d)", robot_id);

    // 3. Espera síncrona pela resposta (Timeout de 20ms)
    auto start_time = std::chrono::steady_clock::now();
    std::vector<uint8_t> rx_accum;
    const uint8_t trash_return[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};

    struct pollfd fds[1];
    fds[0].fd = serial_fd_;
    fds[0].events = POLLIN;

    while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(30)) {
        int ret = poll(fds, 1, 2); // poll de 2ms

        if (ret > 0 && (fds[0].revents & POLLIN)) {
            uint8_t buf[64];
            int n = read(serial_fd_, buf, sizeof(buf));
            if (n > 0) {
                rx_accum.insert(rx_accum.end(), buf, buf + n);
            }

            // Remove o lixo do dongle imediatamente
            while (rx_accum.size() >= 6 && std::equal(trash_return, trash_return + 6, rx_accum.begin())) {
                rx_accum.erase(rx_accum.begin(), rx_accum.begin() + 6);
            }

            // Se recebemos a telemetria completa
            if (rx_accum.size() >= BYTES_LENGTH_ROBOT_TELEMETRY) {
                uint8_t raw_msg_type = rx_accum[0] & 0x0F;
                uint8_t raw_robot_id = (rx_accum[0] >> 4) & 0x0F;

                if (raw_msg_type == MSG_TYPE_TELEMETRY && raw_robot_id == robot_id) {
                    struct RobotTelemetry telemetry_bp;
                    DecodeRobotTelemetry(&telemetry_bp, rx_accum.data());
                    publish_telemetry(telemetry_bp);

                    RCLCPP_INFO(this->get_logger(), "[OK] ACK Robô %d | Batt: %d%%", raw_robot_id, telemetry_bp.battery_percentage);
                    return true; // Sucesso, abandona o loop
                } else {
                    // Dados dessincronizados, avança 1 byte para realinhar
                    rx_accum.erase(rx_accum.begin());
                }
            }
        }
    }

    RCLCPP_WARN(this->get_logger(), "[TIMEOUT] Sem resposta do Robô %d em 20ms.", robot_id);
    return false;
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
    tcflush(serial_fd_, TCIOFLUSH);

    for (const std::string& cmd : at_commands) {
        write(serial_fd_, cmd.c_str(), cmd.length());

        int timeout_ms = (cmd.find("RXA") != std::string::npos || cmd.find("TXA") != std::string::npos) ? 3000 : 500;
        auto start_time = std::chrono::steady_clock::now();
        std::vector<uint8_t> resp;
        bool success = false;

        struct pollfd fds[1];
        fds[0].fd = serial_fd_;
        fds[0].events = POLLIN;

        while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(timeout_ms)) {
            if (poll(fds, 1, 10) > 0 && (fds[0].revents & POLLIN)) {
                uint8_t buf[256];
                int n = read(serial_fd_, buf, sizeof(buf));
                if (n > 0) {
                    resp.insert(resp.end(), buf, buf + n);
                    if (std::search(resp.begin(), resp.end(), success_pattern.begin(), success_pattern.end()) != resp.end()) {
                        success = true;
                        break;
                    }
                }
            }
        }

        std::string cmd_name = cmd.substr(0, cmd.find('='));
        if (success) {
            RCLCPP_INFO(this->get_logger(), "%s configured successfully.", cmd_name.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Warning: %s didn't return success", cmd_name.c_str());
        }
        tcflush(serial_fd_, TCIFLUSH);
    }
    RCLCPP_INFO(this->get_logger(), "Radio configured successfully!");
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<Nrf24HardwareBridge>());
    } catch (const std::exception& e) {
        std::cerr << "Fatal Error : " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}