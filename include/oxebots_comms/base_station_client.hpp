#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "oxebots_comms/base_station_codec.hpp"
#include "oxebots_interfaces/base_station_protocol_bp.h"

namespace oxebots_comms
{

class BaseStationClient
{
public:
    using LogCallback = std::function<void(uint8_t level, const std::string& text)>;

    BaseStationClient() = default;
    ~BaseStationClient();

    BaseStationClient(const BaseStationClient&) = delete;
    BaseStationClient& operator=(const BaseStationClient&) = delete;

    // Discover and connect to an ESP32 Base Station across /dev/ttyUSB* and /dev/ttyACM*
    // (or open a specific port when port_spec != "auto" and non-empty).
    bool connect(const std::string& port_spec = "auto", int baud = 921600);
    void disconnect();

    bool is_connected() const { return fd_ >= 0; }
    const std::string& port_name() const { return port_name_; }
    int baud_rate() const { return baud_rate_; }
    const std::string& last_error() const { return last_error_; }

    void set_log_callback(LogCallback cb) { log_cb_ = std::move(cb); }

    bool ping(struct BaseInfo* out_info, int timeout_ms = 250);
    bool get_info(struct BaseInfo* out_info, int timeout_ms = 250);
    bool set_config(
        const struct BaseRadioConfig& cfg, struct BaseInfo* out_info = nullptr, int timeout_ms = 500);
    bool scan_spectrum(
        uint8_t start_channel,
        uint8_t end_channel,
        uint8_t samples_per_channel,
        struct BaseScanResult* out_result,
        int timeout_ms = 5000);

    // Send one RF frame via Base Station and wait for hardware TX status (BS_MSG_RF_TX_STATUS).
    bool send_rf_packet(
        uint8_t target_robot_id,
        const uint8_t* payload,
        uint8_t len,
        uint8_t tx_flags,
        struct BaseRfTxStatus* out_status = nullptr,
        int timeout_ms = 50);

    // Pop an already-queued uplink RF packet, or poll the serial port until one arrives.
    bool pop_rf_rx(struct BaseRfRx* out_rx);
    bool wait_rf_rx(struct BaseRfRx* out_rx, int timeout_ms);
    void clear_rx_queue();

    static std::vector<std::string> list_candidate_ports();
    static bool probe_port(
        const std::string& port, int baud, struct BaseInfo* out_info, int timeout_ms = 250);

private:
    bool open_port(const std::string& port, int baud);
    bool write_frame(uint8_t msg_type, const uint8_t* payload, std::size_t len);
    bool poll_once(int timeout_ms);
    void dispatch_frame(const DecodedBaseFrame& frame);

    int fd_ = -1;
    std::string port_name_;
    int baud_rate_ = 921600;
    std::string last_error_;
    uint16_t next_seq_ = 1;
    uint32_t next_ping_seq_ = 1;

    BaseFrameDecoder decoder_;
    std::deque<struct BaseInfo> info_queue_;
    std::deque<struct BaseRfTxStatus> tx_status_queue_;
    std::deque<struct BaseRfRx> rf_rx_queue_;
    std::deque<struct BaseScanResult> scan_queue_;
    LogCallback log_cb_;
};

}  // namespace oxebots_comms
