#include <vector>
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include "oxebots_comms/udp_driver.hpp"

#include <oxebots_interfaces/msg/ssl_geometry_data.hpp>
#include <oxebots_interfaces/msg/ssl_field_size.hpp>
#include <oxebots_interfaces/msg/ssl_camera_calibration.hpp>
#include <oxebots_interfaces/msg/ssl_geometry_models.hpp>
#include <oxebots_interfaces/msg/ssl_ball_model_straight_two_phase.hpp>
#include <oxebots_interfaces/msg/ssl_ball_model_chip_fixed_loss.hpp>

#include "oxebots_interfaces/messages_robocup_ssl_wrapper.pb.h"
#include "oxebots_interfaces/messages_robocup_ssl_geometry.pb.h"

class GeometryReceiverNode : public rclcpp::Node,
                        public UdpDriver<SSL_WrapperPacket> {
public:
    GeometryReceiverNode(boost::asio::io_context & io_context);
    ~GeometryReceiverNode();

protected:
    void on_receive(const SSL_WrapperPacket & packet) override;

private:
    rclcpp::Publisher<oxebots_interfaces::msg::SSLGeometryData>::SharedPtr geometry_publisher;

    boost::asio::io_context & io_context;
    std::thread io_thread;

    void publishGeometryData(
        oxebots_interfaces::msg::SSLFieldSize field_size,
        std::vector<oxebots_interfaces::msg::SSLCameraCalibration> camera_calibrations,
        oxebots_interfaces::msg::SSLGeometryModels geometry_models);

    void convertFieldSize(const SSL_GeometryData& geometry, oxebots_interfaces::msg::SSLFieldSize& field_size);
    void convertCameraCalibrations(const SSL_GeometryData& geometry, std::vector<oxebots_interfaces::msg::SSLCameraCalibration>& camera_calibrations);
    void convertGeometryModels(const SSL_GeometryData& geometry, oxebots_interfaces::msg::SSLGeometryModels& geometry_models);
};
