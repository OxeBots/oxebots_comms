#include "oxebots_comms/Nrf24.hpp"

std::string Find_Nrf24_port(uint16_t port_pid, uint16_t port_vid){
    // create the vars
    int port_vid_search, port_pid_search;
    char *port_name_pointer;
    std::string port_name_string = "null";

    std::cout << "Searching for NRF24 on USB ports " << std::endl;

    // create an sp_port type list to get all the ports on the deivce
    struct sp_port **port_list;
    enum sp_return result = sp_list_ports(&port_list);

    // error protection
    if (result != SP_OK)
    {
        std::cout << "Search failed, couldnt open ports" << std::endl;
        return "";
    }

    // search for the Nrf24 signature (pid and vid) on the ports list
    for (int i = 0; port_list[i] != NULL; i++)
    {
        enum sp_return status = sp_get_port_usb_vid_pid(port_list[i], &port_vid_search, &port_pid_search);

        if (status == SP_OK)
        {
            if (port_pid_search == port_pid  && port_vid_search == port_vid )
            {
                port_name_pointer = sp_get_port_name(port_list[i]);
                port_name_string = port_name_pointer;
                std::cout << "Found NRF24 on port: " << port_name_string << std::endl;
                break;
            }
        }
    }

    // release the pointers for the port list
    sp_free_port_list(port_list);

    return port_name_string;
}

Nrf24HardwareBridge::Nrf24HardwareBridge() : Node("nrf24_hardware_bridge")
{
    // declarete the AT config parameters and the default value
    this->declare_parameter("radio_rx_address", "ADMIN");
    this->declare_parameter("radio_tx_address", "ESP32");
    this->declare_parameter("radio_frequency", 2.476);
    this->declare_parameter("radio_rate", 3);

    // set the publisher to the telemetry return
    telemetry_pub_ = this->create_publisher<oxebots_interfaces::msg::RobotTelemetry>("/robot_telemetry", 10);

    // set the subscriber to the robots commands
    command_sub_ = this->create_subscription<oxebots_interfaces::msg::RobotCmd>(
        "/robot_commands",
        rclcpp::SensorDataQoS(),
        std::bind(&Nrf24HardwareBridge::send_command, this, std::placeholders::_1)
    );

    // Starts the Nrf24 search
    const uint16_t port_vid = 0x1A86, port_pid = 0x7523;
    std::string port_name_string = Find_Nrf24_port(port_pid, port_vid);

    // Fallback to search failure
    if (port_name_string == "null" || port_name_string.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "NRF24 not found on USB ports");
        throw std::runtime_error("NRF24 not found");
    }

    RCLCPP_INFO(this->get_logger(), "Connecting Nrf24 on USB port: %s", port_name_string.c_str());

    // Starting serial communication
    try {
        // creating the smart pointer (unique ptr) to the serial port object (asio)
        serial_ = std::make_unique<boost::asio::serial_port>(io_context_, port_name_string);

        // set the serial configs to match the firmware
        serial_->set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        RCLCPP_INFO(this->get_logger(), "Nrf24 bridge node started!");
    } catch (boost::system::system_error& error) {
        RCLCPP_ERROR(this->get_logger(), "Error while opening Nrf24: %s", error.what());
        throw std::runtime_error("Fail on Boost Asio");
    }

    // Setup the NRF24 hardware via AT commands
    this->configure_nrf24();

    // Start the usb listen and open the io_context
    this->start_receive();

    // thead inicialization
    io_thread_ = std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Starting Asio IO Thread");
        try {
            io_context_.run();
            RCLCPP_WARN(this->get_logger(), "Asio IO Thread stoped (no work on queue).");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Asio IO Thread error: %s", e.what());
        }
    });
}

Nrf24HardwareBridge::~Nrf24HardwareBridge() {
    // class destructor
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

void Nrf24HardwareBridge::send_command(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg){
    for (const auto & robot_ros : msg->robots)
    {
        struct RobotCommand cmd_bitproto = {};

        cmd_bitproto.header.msg_type = MSG_TYPE_COMMAND;
        cmd_bitproto.header.robot_id = robot_ros.id;

        auto now = std::chrono::system_clock::now();
        cmd_bitproto.header.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                            now.time_since_epoch()).count() & 0xFFFFFFFF;

        cmd_bitproto.target_pose.x_v = static_cast<int16_t>(robot_ros.x_velocity * 1000.0);
        cmd_bitproto.target_pose.y_v = static_cast<int16_t>(robot_ros.y_velocity * 1000.0);
        cmd_bitproto.target_pose.angular_vel = static_cast<int16_t>(robot_ros.angular_velocity * 100.0);

        cmd_bitproto.kick_velocity = robot_ros.kick_speed;

        uint8_t buffer[BYTES_LENGTH_ROBOT_COMMAND];
        EncodeRobotCommand(&cmd_bitproto, buffer);

        RCLCPP_INFO(this->get_logger(), "Sending an robot command");

        // Creates an secure copy to work
        std::vector<uint8_t> data_to_send(buffer, buffer + BYTES_LENGTH_ROBOT_COMMAND);

        // Puts the write work on the io_context queue
        boost::asio::post(io_context_, [this, data_to_send]() {
            try {
                boost::asio::write(*serial_, boost::asio::buffer(data_to_send));
                // forces the flush
                ::tcdrain(serial_->native_handle());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error while sending an robot async comand: %s", e.what());
            }
        });
    }
}

void Nrf24HardwareBridge::start_receive() {
    // Start the asynchronous receiving process and call handle_receive on receive
    serial_->async_read_some(
    boost::asio::buffer(recive_buffer_, sizeof(recive_buffer_)),
    std::bind(&Nrf24HardwareBridge::handle_receive, this,
              std::placeholders::_1, std::placeholders::_2)
    );
}

void Nrf24HardwareBridge::handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    RCLCPP_INFO(this->get_logger(), "Nrf24 received %ld bytes", bytes_transferred);

    const size_t TELEMETRY_SIZE = 22;

    if (!error) {
        // Transform an static c array into a c++ vector
        std::vector<uint8_t> raw_data(recive_buffer_, recive_buffer_ + bytes_transferred);

        // return sequency that represents tx complete
        const uint8_t trash_return[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
        const size_t TRASH_SIZE = 6;

        // puts the recive data in an process buff
        persistent_buffer_.insert(persistent_buffer_.end(),
                          recive_buffer_, recive_buffer_ + bytes_transferred);

        while (!persistent_buffer_.empty()) {
            // checks for the trash and remove it
            if (persistent_buffer_.size() >= TRASH_SIZE && std::equal(persistent_buffer_.begin(), persistent_buffer_.begin() + TRASH_SIZE, trash_return)){
                RCLCPP_INFO(this->get_logger(), "\033[94mDongle NRF24 confirmed shipping (TX Complete)!\033[0m");
                persistent_buffer_.erase(persistent_buffer_.begin(), persistent_buffer_.begin() + TRASH_SIZE);
                continue;
            }

            // cheks if the message its complete
            if (persistent_buffer_.size() < TELEMETRY_SIZE) {
                break;
            }

            // Pick the lasts 4 bits (msg_tipe) from the byte 0 (header)
            uint8_t raw_msg_type = persistent_buffer_[0] & 0x0F;

            if (raw_msg_type == MSG_TYPE_TELEMETRY) {
                uint8_t raw_robot_id = (persistent_buffer_[0] >> 4) & 0x0F;

                if (raw_robot_id <= 11) {
                    struct RobotTelemetry telemetry_bitproto;
                    DecodeRobotTelemetry(&telemetry_bitproto, persistent_buffer_.data());

                    RCLCPP_INFO(this->get_logger(), "Received telemetry (Robot %d)", telemetry_bitproto.header.robot_id);

                    // call the ros publisher process
                    this->publish_telemetry(telemetry_bitproto);

                    // Delete the package from the buff
                    persistent_buffer_.erase(persistent_buffer_.begin(), persistent_buffer_.begin() + TELEMETRY_SIZE);
                } else {
                    // trash handdler
                    persistent_buffer_.erase(persistent_buffer_.begin());
                }
            }
            else {
                // data out of sync, try again
                persistent_buffer_.erase(persistent_buffer_.begin());
            }
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Error while reading USB (NRF24) %s", error.message().c_str());
        return;
    }

    // re-make the program listen to the telemetry data
    this->start_receive();
}

void Nrf24HardwareBridge::publish_telemetry(const struct RobotTelemetry& telemetry_bp)
{
    // Creante an instance of the message
    auto msg = oxebots_interfaces::msg::RobotTelemetry();

    // Put the data
    msg.robot_id = telemetry_bp.header.robot_id;
    msg.timestamp = telemetry_bp.header.timestamp;

    msg.x = telemetry_bp.robot_pose.x;
    msg.y = telemetry_bp.robot_pose.y;
    msg.orientation = telemetry_bp.robot_pose.angle / 100.0f;

    msg.battery_percentage = telemetry_bp.battery_percentage;
    msg.kicker_voltage = telemetry_bp.kicker_voltage / 100.0f;
    msg.error_flags = telemetry_bp.error_flags;

    /* Debug log
    RCLCPP_INFO(this->get_logger(), "Robot %d | Battery: %d%% | Kicker: %.2fV | Error: %d",
                msg.robot_id, msg.battery_percentage, msg.kicker_voltage, msg.error_flags);
    */

    // Publish the message
    telemetry_pub_->publish(msg);
}

void Nrf24HardwareBridge::configure_nrf24() {
    RCLCPP_INFO(this->get_logger(), "Starting AT radio config");

    // Pick the parameters
    std::string rx_param = this->get_parameter("radio_rx_address").as_string();
    std::string tx_param = this->get_parameter("radio_tx_address").as_string();
    double freq_param = this->get_parameter("radio_frequency").as_double();
    int rate_param = this->get_parameter("radio_rate").as_int();

    // labda func to convert the strings to hex
    auto format_address = [](const std::string& addr) {
        char buffer[64];
        // the adress has to be 5 char
        snprintf(buffer, sizeof(buffer), "0x%02X,0x%02X,0x%02X,0x%02X,0x%02X",
                 addr[0], addr[1], addr[2], addr[3], addr[4]);
        return std::string(buffer);
    };

    // format the frequency
    char freq_buf[32];
    int f_int = static_cast<int>(freq_param); // Pick the integer part
    int f_frac = static_cast<int>(((freq_param - f_int) * 1000.0) + 0.5); // Picks the float part and round
    snprintf(freq_buf, sizeof(freq_buf), "AT+FREQ=%d.%03d\r\n", f_int, f_frac);

    // gather everything
    std::vector<std::string> at_commands = {
        "AT+RXA=" + format_address(rx_param) + "\r\n",
        "AT+TXA=" + format_address(tx_param) + "\r\n",
        std::string(freq_buf),
        "AT+RATE=" + std::to_string(rate_param) + "\r\n"
    };

    boost::system::error_code error;

    // "Success" pattern in GB2312 (成功)
    const std::vector<uint8_t> success_pattern = {0xB3, 0xC9, 0xB9, 0xA6};

    // Make shure the _serial its clean before starting
    ::tcflush(serial_->native_handle(), TCIOFLUSH);

    for (const std::string& at_command : at_commands) {

        // Sends AT Commands
        boost::asio::write(*serial_, boost::asio::buffer(at_command), error);

        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Error while sending AT command %s: %s",
                         at_command.c_str(), error.message().c_str());
            throw std::runtime_error("Error during Handshake radio process (Nrf24)");
        }

        // Some time that it takes to configure
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Start seek for the sucess pattern
        auto start_time = std::chrono::steady_clock::now();
        std::vector<uint8_t> response_data;
        bool cmd_success = false;

        // Waits up to 500ms for the full response to accumulate
        while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(500)) {
            int bytes_available = 0;

            if (ioctl(serial_->native_handle(), FIONREAD, &bytes_available) == 0 && bytes_available > 0) {
                char buf[256];
                size_t to_read = std::min(static_cast<size_t>(bytes_available), sizeof(buf));
                size_t bytes_read = serial_->read_some(boost::asio::buffer(buf, to_read), error);

                if (!error && bytes_read > 0) {
                    response_data.insert(response_data.end(), buf, buf + bytes_read);

                    auto it = std::search(response_data.begin(), response_data.end(),
                                          success_pattern.begin(), success_pattern.end());

                    if (it != response_data.end()) {
                        cmd_success = true;
                        break;
                    }
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        std::string cmd_name = at_command.substr(0, at_command.find('='));
        if (cmd_success) {
            RCLCPP_INFO(this->get_logger(), "%s configured", cmd_name.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Warning: %s dont return success (recived %zu bytes (trash)).",
                        cmd_name.c_str(), response_data.size());
        }

        // Clear the buffer before continue
        ::tcflush(serial_->native_handle(), TCIFLUSH);
    }

    // Clean the enviroment
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ::tcflush(serial_->native_handle(), TCIOFLUSH);
    persistent_buffer_.clear();

    RCLCPP_INFO(this->get_logger(), "Radio configured");
}

int main(int argc, char ** argv) {
    // Start the ROS
    rclcpp::init(argc, argv);

    try {
        // Try to create ROS node
        auto node = std::make_shared<Nrf24HardwareBridge>();

        // starts
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        // Error handle
        std::cerr << "Fatal Error : " << e.what() << std::endl;
    }

    // Release memory
    rclcpp::shutdown();
    return 0;
}