#include "oxebots_comms/base_station_client.hpp"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <libserialport.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <set>
#include <sstream>

namespace oxebots_comms
{
namespace
{

speed_t baud_to_speed(int baud)
{
    switch (baud) {
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 921600:
            return B921600;
        default:
            return B921600;
    }
}

bool is_usb_serial_path(const std::string& path)
{
    return path.find("ttyUSB") != std::string::npos || path.find("ttyACM") != std::string::npos;
}

}  // namespace

BaseStationClient::~BaseStationClient()
{
    disconnect();
}

std::vector<std::string> BaseStationClient::list_candidate_ports()
{
    std::set<std::string> unique_ports;
    struct sp_port** port_list = nullptr;
    if (sp_list_ports(&port_list) == SP_OK && port_list != nullptr) {
        for (int i = 0; port_list[i] != nullptr; ++i) {
            const char* name = sp_get_port_name(port_list[i]);
            if (name != nullptr && is_usb_serial_path(name)) {
                unique_ports.insert(name);
            }
        }
        sp_free_port_list(port_list);
    }

    // Fallback direct /dev scan catches hot-plugged USB serial nodes inside Docker containers
    // even when /sys/class/tty is not refreshed by libserialport.
    if (DIR* dir = opendir("/dev")) {
        while (struct dirent* ent = readdir(dir)) {
            const std::string name(ent->d_name);
            if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0) {
                unique_ports.insert("/dev/" + name);
            }
        }
        closedir(dir);
    }

    return std::vector<std::string>(unique_ports.begin(), unique_ports.end());
}

bool BaseStationClient::probe_port(
    const std::string& port, int baud, struct BaseInfo* out_info, int timeout_ms)
{
    BaseStationClient probe;
    if (!probe.open_port(port, baud)) {
        return false;
    }
    return probe.ping(out_info, timeout_ms);
}

bool BaseStationClient::connect(const std::string& port_spec, int baud)
{
    disconnect();
    baud_rate_ = baud;
    last_error_.clear();

    if (!port_spec.empty() && port_spec != "auto") {
        if (!open_port(port_spec, baud)) {
            return false;
        }
        struct BaseInfo info = {};
        if (!ping(&info, 800)) {
            last_error_ = port_spec + ": sem resposta ao PING (timeout 800 ms)";
            disconnect();
            return false;
        }
        return true;
    }

    const auto candidates = list_candidate_ports();
    if (candidates.empty()) {
        last_error_ = "nenhuma porta /dev/ttyUSB* ou /dev/ttyACM* visivel no container";
        return false;
    }

    std::ostringstream diag;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const auto& port = candidates[i];
        if (i > 0) diag << "; ";
        if (!open_port(port, baud)) {
            diag << last_error_;
            continue;
        }
        struct BaseInfo info = {};
        if (ping(&info, 650)) {
            last_error_.clear();
            return true;
        }
        diag << port << ": sem resposta ao PING";
        disconnect();
    }
    last_error_ = diag.str();
    return false;
}

void BaseStationClient::disconnect()
{
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
    port_name_.clear();
    decoder_.reset();
    info_queue_.clear();
    tx_status_queue_.clear();
    rf_rx_queue_.clear();
    scan_queue_.clear();
}

bool BaseStationClient::open_port(const std::string& port, int baud)
{
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        const int err = errno;
        last_error_ = port + ": falha ao abrir (" + std::strerror(err) + ")";
        return false;
    }

    // Exclusive lock prevents stomping on a port already owned by Robot Bench.
    if (ioctl(fd_, TIOCEXCL) != 0) {
        const int err = errno;
        last_error_ = port + ": porta ocupada/TIOCEXCL (" + std::strerror(err) + ")";
        close(fd_);
        fd_ = -1;
        return false;
    }

    // Keep DTR/RTS deasserted so opening the port does not reset the ESP32 DevKit.
    int modem_bits = 0;
    if (ioctl(fd_, TIOCMGET, &modem_bits) == 0) {
        modem_bits &= ~(TIOCM_DTR | TIOCM_RTS);
        ioctl(fd_, TIOCMSET, &modem_bits);
    }

    struct termios tty = {};
    if (tcgetattr(fd_, &tty) != 0) {
        const int err = errno;
        last_error_ = port + ": tcgetattr falhou (" + std::strerror(err) + ")";
        close(fd_);
        fd_ = -1;
        return false;
    }

    const speed_t spd = baud_to_speed(baud);
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~HUPCL;  // Avoid DTR/RTS drop on close resetting the ESP32 DevKit
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close(fd_);
        fd_ = -1;
        return false;
    }

    struct serial_struct ser_info = {};
    if (ioctl(fd_, TIOCGSERIAL, &ser_info) == 0) {
        ser_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(fd_, TIOCSSERIAL, &ser_info);
    }

    tcflush(fd_, TCIOFLUSH);
    decoder_.reset();
    port_name_ = port;
    baud_rate_ = baud;
    return true;
}

bool BaseStationClient::write_frame(uint8_t msg_type, const uint8_t* payload, std::size_t len)
{
    if (fd_ < 0) return false;
    const std::vector<uint8_t> frame = encode_base_frame(msg_type, payload, len);
    if (frame.empty()) return false;

    std::size_t total = 0;
    while (total < frame.size()) {
        const ssize_t n = write(fd_, frame.data() + total, frame.size() - total);
        if (n > 0) {
            total += static_cast<std::size_t>(n);
            continue;
        }
        struct pollfd pfd = {fd_, POLLOUT, 0};
        if (poll(&pfd, 1, 20) <= 0) {
            return false;
        }
    }
    return true;
}

void BaseStationClient::dispatch_frame(const DecodedBaseFrame& frame)
{
    switch (frame.msg_type) {
        case BS_MSG_INFO:
            if (frame.payload.size() == BYTES_LENGTH_BASE_INFO) {
                struct BaseInfo info = {};
                DecodeBaseInfo(&info, const_cast<uint8_t*>(frame.payload.data()));
                info_queue_.push_back(info);
            }
            break;

        case BS_MSG_RF_TX_STATUS:
            if (frame.payload.size() == BYTES_LENGTH_BASE_RF_TX_STATUS) {
                struct BaseRfTxStatus st = {};
                DecodeBaseRfTxStatus(&st, const_cast<uint8_t*>(frame.payload.data()));
                tx_status_queue_.push_back(st);
            }
            break;

        case BS_MSG_RF_RX:
            if (frame.payload.size() == BYTES_LENGTH_BASE_RF_RX) {
                struct BaseRfRx rx = {};
                DecodeBaseRfRx(&rx, const_cast<uint8_t*>(frame.payload.data()));
                rf_rx_queue_.push_back(rx);
                if (rf_rx_queue_.size() > 64) {
                    rf_rx_queue_.pop_front();
                }
            }
            break;

        case BS_MSG_SCAN_RESULT:
            if (frame.payload.size() == BYTES_LENGTH_BASE_SCAN_RESULT) {
                struct BaseScanResult res = {};
                DecodeBaseScanResult(&res, const_cast<uint8_t*>(frame.payload.data()));
                scan_queue_.push_back(res);
            }
            break;

        case BS_MSG_LOG:
            if (frame.payload.size() == BYTES_LENGTH_BASE_LOG && log_cb_) {
                struct BaseLog log = {};
                DecodeBaseLog(&log, const_cast<uint8_t*>(frame.payload.data()));
                const std::size_t n = std::min<std::size_t>(log.text_len, 96);
                log_cb_(log.level, std::string(reinterpret_cast<const char*>(log.text), n));
            }
            break;

        default:
            break;
    }
}

bool BaseStationClient::poll_once(int timeout_ms)
{
    if (fd_ < 0) return false;
    struct pollfd pfd = {fd_, POLLIN, 0};
    const int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0 || !(pfd.revents & POLLIN)) {
        return false;
    }
    uint8_t buf[512];
    const ssize_t n = read(fd_, buf, sizeof(buf));
    if (n <= 0) {
        return false;
    }
    decoder_.feed(buf, static_cast<std::size_t>(n), [this](const DecodedBaseFrame& frame) {
        dispatch_frame(frame);
    });
    return true;
}

bool BaseStationClient::ping(struct BaseInfo* out_info, int timeout_ms)
{
    info_queue_.clear();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    auto next_send = std::chrono::steady_clock::time_point{};

    while (std::chrono::steady_clock::now() < deadline) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_send) {
            struct BasePing msg = {};
            msg.seq = next_ping_seq_++;
            uint8_t raw[BYTES_LENGTH_BASE_PING] = {};
            EncodeBasePing(&msg, raw);
            if (!write_frame(BS_MSG_PING, raw, sizeof(raw))) {
                return false;
            }
            next_send = now + std::chrono::milliseconds(150);
        }
        if (!info_queue_.empty()) {
            const struct BaseInfo info = info_queue_.front();
            info_queue_.pop_front();
            if (info.protocol_version == BS_PROTO_VERSION) {
                if (out_info != nullptr) *out_info = info;
                return true;
            }
        }
        const int rem = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now())
                .count());
        if (rem <= 0) break;
        const int wait_ms = std::min(rem, 50);
        poll_once(wait_ms);
    }
    return false;
}

bool BaseStationClient::get_info(struct BaseInfo* out_info, int timeout_ms)
{
    info_queue_.clear();
    struct BaseEmpty msg = {};
    msg.reserved = 0;
    uint8_t raw[BYTES_LENGTH_BASE_EMPTY] = {};
    EncodeBaseEmpty(&msg, raw);
    if (!write_frame(BS_MSG_GET_INFO, raw, sizeof(raw))) {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (!info_queue_.empty()) {
            if (out_info != nullptr) *out_info = info_queue_.front();
            info_queue_.pop_front();
            return true;
        }
        const int rem = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now())
                .count());
        if (rem <= 0) break;
        poll_once(rem);
    }
    return false;
}

bool BaseStationClient::set_config(
    const struct BaseRadioConfig& cfg, struct BaseInfo* out_info, int timeout_ms)
{
    info_queue_.clear();
    struct BaseRadioConfig copy = cfg;
    uint8_t raw[BYTES_LENGTH_BASE_RADIO_CONFIG] = {};
    EncodeBaseRadioConfig(&copy, raw);
    if (!write_frame(BS_MSG_SET_CONFIG, raw, sizeof(raw))) {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (!info_queue_.empty()) {
            if (out_info != nullptr) *out_info = info_queue_.front();
            info_queue_.pop_front();
            return true;
        }
        const int rem = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now())
                .count());
        if (rem <= 0) break;
        poll_once(rem);
    }
    return false;
}

bool BaseStationClient::scan_spectrum(
    uint8_t start_channel,
    uint8_t end_channel,
    uint8_t samples_per_channel,
    struct BaseScanResult* out_result,
    int timeout_ms)
{
    scan_queue_.clear();
    struct BaseScanRequest req = {};
    req.start_channel = start_channel;
    req.end_channel = end_channel;
    req.samples_per_channel = samples_per_channel;
    uint8_t raw[BYTES_LENGTH_BASE_SCAN_REQUEST] = {};
    EncodeBaseScanRequest(&req, raw);
    if (!write_frame(BS_MSG_SCAN_REQ, raw, sizeof(raw))) {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (!scan_queue_.empty()) {
            if (out_result != nullptr) *out_result = scan_queue_.front();
            scan_queue_.pop_front();
            return true;
        }
        const int rem = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now())
                .count());
        if (rem <= 0) break;
        poll_once(rem);
    }
    return false;
}

bool BaseStationClient::send_rf_packet(
    uint8_t target_robot_id,
    const uint8_t* payload,
    uint8_t len,
    uint8_t tx_flags,
    struct BaseRfTxStatus* out_status,
    int timeout_ms)
{
    if (payload == nullptr || len == 0 || len > 31) return false;

    const uint16_t seq = next_seq_++;
    struct BaseRfTx req = {};
    req.seq = seq;
    req.target_robot_id = target_robot_id;
    req.tx_flags = tx_flags;
    req.payload_len = len;
    std::memcpy(req.payload, payload, len);

    uint8_t raw[BYTES_LENGTH_BASE_RF_TX] = {};
    EncodeBaseRfTx(&req, raw);
    if (!write_frame(BS_MSG_RF_TX, raw, sizeof(raw))) {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        while (!tx_status_queue_.empty()) {
            const struct BaseRfTxStatus st = tx_status_queue_.front();
            tx_status_queue_.pop_front();
            if (st.seq == seq) {
                if (out_status != nullptr) *out_status = st;
                return true;
            }
        }
        const int rem = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now())
                .count());
        if (rem <= 0) break;
        poll_once(rem);
    }
    return false;
}

bool BaseStationClient::pop_rf_rx(struct BaseRfRx* out_rx)
{
    if (rf_rx_queue_.empty()) return false;
    if (out_rx != nullptr) *out_rx = rf_rx_queue_.front();
    rf_rx_queue_.pop_front();
    return true;
}

bool BaseStationClient::wait_rf_rx(struct BaseRfRx* out_rx, int timeout_ms)
{
    if (pop_rf_rx(out_rx)) return true;

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pop_rf_rx(out_rx)) return true;
        const int rem = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now())
                .count());
        if (rem <= 0) break;
        poll_once(rem);
    }
    return pop_rf_rx(out_rx);
}

void BaseStationClient::clear_rx_queue()
{
    rf_rx_queue_.clear();
}

}  // namespace oxebots_comms
