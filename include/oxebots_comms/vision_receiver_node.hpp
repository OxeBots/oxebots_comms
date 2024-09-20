#include <vector>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <rclcpp/rclcpp.hpp>
#include "oxebots_comms/udp_driver.hpp"

#include <oxebots_interfaces/msg/ssl_geometry_data.hpp>
#include <oxebots_interfaces/msg/ssl_field_size.hpp>
#include <oxebots_interfaces/msg/ssl_camera_calibration.hpp>
#include <oxebots_interfaces/msg/ssl_geometry_models.hpp>
#include <oxebots_interfaces/msg/ssl_ball_model_straight_two_phase.hpp>
#include <oxebots_interfaces/msg/ssl_ball_model_chip_fixed_loss.hpp>
#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/robot_game_data.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"

#include "oxebots_interfaces/messages_robocup_ssl_wrapper.pb.h"
#include "oxebots_interfaces/messages_robocup_ssl_detection.pb.h"
#include "oxebots_interfaces/messages_robocup_ssl_geometry.pb.h"

class VisionReceiverNode : public rclcpp::Node,
                         public UdpDriver<SSL_WrapperPacket>
{
   private:
    rclcpp::Publisher<oxebots_interfaces::msg::RobotPosition>::SharedPtr
    robot_publisher;

    rclcpp::Publisher<oxebots_interfaces::msg::BallPosition>::SharedPtr
    ball_publisher;

    rclcpp::Publisher<oxebots_interfaces::msg::SSLGeometryData>::SharedPtr 
    geometry_publisher;

    boost::asio::io_context & io_context;
    std::thread io_thread;

    bool is_yellow_team;

   public:
    VisionReceiverNode(boost::asio::io_context & io_context);
    ~VisionReceiverNode();

   protected:
    void on_receive(const SSL_WrapperPacket & packet) override;

   private:
    void processDetectionData(const SSL_DetectionFrame & detection);
    void processGeometryData(const SSL_GeometryData & geometry);

    void convertFieldSize(const SSL_GeometryData& geometry, oxebots_interfaces::msg::SSLFieldSize& field_size);
    
    void convertCameraCalibrations(const SSL_GeometryData& geometry, 
        std::vector<oxebots_interfaces::msg::SSLCameraCalibration>& camera_calibrations);
    
    void convertGeometryModels(const SSL_GeometryData& geometry, 
        oxebots_interfaces::msg::SSLGeometryModels& geometry_models);

    void PublishRobotData(
      std::vector<oxebots_interfaces::msg::RobotGameData> allies,
      std::vector<oxebots_interfaces::msg::RobotGameData> enemies);

    void publishGeometryData(
        oxebots_interfaces::msg::SSLFieldSize field_size,
        std::vector<oxebots_interfaces::msg::SSLCameraCalibration> camera_calibrations,
        oxebots_interfaces::msg::SSLGeometryModels geometry_models);

    void PublishBallData(oxebots_interfaces::msg::BallPosition ball_data);
};