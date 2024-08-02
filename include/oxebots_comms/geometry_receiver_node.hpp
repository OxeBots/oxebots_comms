#include <vector>
#include <thread>
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>

#include <oxebots_interfaces/msg/ssl_geometry_data.hpp>
#include <oxebots_interfaces/msg/ssl_field_size.hpp>
#include <oxebots_interfaces/msg/ssl_camera_calibration.hpp>
#include <oxebots_interfaces/msg/ssl_geometry_models.hpp>
#include <oxebots_interfaces/msg/ssl_ball_model_straight_two_phase.hpp>
#include <oxebots_interfaces/msg/ssl_ball_model_chip_fixed_loss.hpp>

#include "oxebots_interfaces/messages_robocup_ssl_wrapper.pb.h"
#include "oxebots_interfaces/messages_robocup_ssl_geometry.pb.h"

class GeometryReceiverNode : public rclcpp::Node {
public:
    GeometryReceiverNode();
    ~GeometryReceiverNode();

private:
    rclcpp::Publisher<oxebots_interfaces::msg::SSLGeometryData>::SharedPtr geometry_publisher;

    std::thread udp_thread;
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;

    void GetGeometryData();

    void publishGeometryData(
        oxebots_interfaces::msg::SSLFieldSize field_size,
        std::vector<oxebots_interfaces::msg::SSLCameraCalibration> camera_calibrations,
        oxebots_interfaces::msg::SSLGeometryModels geometry_models);

    void convertFieldSize(const SSL_GeometryData& geometry, oxebots_interfaces::msg::SSLFieldSize& field_size);
    void convertCameraCalibrations(const SSL_GeometryData& geometry, std::vector<oxebots_interfaces::msg::SSLCameraCalibration>& camera_calibrations);
    void convertGeometryModels(const SSL_GeometryData& geometry, oxebots_interfaces::msg::SSLGeometryModels& geometry_models);
};

