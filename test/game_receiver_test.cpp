#include "oxebots_comms/game_receiver.hpp"

#include <gtest/gtest.h>

#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include "oxebots_comms/game_receiver.hpp"
#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/robot_game_data.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"

class GameReceiverTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        node_ = std::make_shared<GameReceiver>();
        RCLCPP_INFO(rclcpp::get_logger("test_logger"), "Node created.");
    }

    std::shared_ptr<GameReceiver> node_;
};

TEST_F(GameReceiverTest, TestUdpReceivePublish)
{
    //rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    bool message_robot_received = false;
    bool message_ball_received = false;

    SSL_WrapperPacket * data_comp = new SSL_WrapperPacket();
    SSL_DetectionFrame * detection_frame_data = new SSL_DetectionFrame();
    //Mocking necessary values...
    detection_frame_data->set_frame_number(1);
    detection_frame_data->set_t_sent(1);
    detection_frame_data->set_t_capture(1);
    detection_frame_data->set_camera_id(1);

    auto detection_ball_data = detection_frame_data->add_balls();
    detection_ball_data->set_x(1);
    detection_ball_data->set_y(2);
    detection_ball_data->set_z(3);
    detection_ball_data->set_confidence(1);
    detection_ball_data->set_pixel_x(1);
    detection_ball_data->set_pixel_y(2);

    for (int i = 0; i < 3; i++)
    {
        auto detection_robot_blue = detection_frame_data->add_robots_blue();
        auto detection_robot_yellow =
          detection_frame_data->add_robots_yellow();

        detection_robot_blue->set_y(i);
        detection_robot_blue->set_x(i);
        detection_robot_blue->set_orientation(i);
        detection_robot_blue->set_confidence(1);
        detection_robot_blue->set_pixel_y(i);
        detection_robot_blue->set_pixel_x(i);

        detection_robot_yellow->set_y(i);
        detection_robot_yellow->set_x(i);
        detection_robot_yellow->set_orientation(i);
        detection_robot_yellow->set_confidence(1);
        detection_robot_yellow->set_pixel_y(i);
        detection_robot_yellow->set_pixel_x(i);
    }

    data_comp->set_allocated_detection(detection_frame_data);
    std::string serialized_data_comp = data_comp->SerializeAsString();

    ASSERT_NE(serialized_data_comp, "");
    RCLCPP_INFO(rclcpp::get_logger("test_logger"),
                 "Serialized data is not empty.");

    // Subscription for robot data
    oxebots_interfaces::msg::RobotPosition received_robot_data;
    auto robot_subscription =
      node_->create_subscription<oxebots_interfaces::msg::RobotPosition>(
        "robot_data", 10,
        [&message_robot_received, &received_robot_data](
          const oxebots_interfaces::msg::RobotPosition::SharedPtr msg) {
            received_robot_data.allies = msg->allies;
            received_robot_data.enemies = msg->enemies;
            message_robot_received = true;
            RCLCPP_INFO(rclcpp::get_logger("test_logger"),
                         "Robot data received.");
        });

    // Subscription for ball data
    oxebots_interfaces::msg::BallPosition received_ball_data;
    auto ball_subscription =
      node_->create_subscription<oxebots_interfaces::msg::BallPosition>(
        "ball_data", 10,
        [&message_ball_received, &received_ball_data](
          oxebots_interfaces::msg::BallPosition::SharedPtr msg) {
            received_ball_data.x = msg->x;
            received_ball_data.y = msg->y;
            received_ball_data.z = msg->z;
            message_ball_received = true;
            RCLCPP_INFO(rclcpp::get_logger("test_logger"),
                         "Ball data received.");
        });

    // Simulate UDP message sending
    RCLCPP_INFO(rclcpp::get_logger("test_logger"), "Sending a message...");
    boost::asio::ip::udp::endpoint remote_endpoint(
      boost::asio::ip::address::from_string("224.5.23.2"), 10020);
    boost::asio::io_service io_service;
    udp::socket test_socket(io_service, udp::endpoint(udp::v4(), 0));

    test_socket.send_to(boost::asio::buffer(serialized_data_comp),
                        remote_endpoint);
    RCLCPP_INFO(rclcpp::get_logger("test_logger"), "UDP message sent.");

    // Wait for message reception
    auto start = std::chrono::steady_clock::now();
    while (
      !message_robot_received && !message_ball_received &&
      (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)))
    {
        RCLCPP_INFO(rclcpp::get_logger("test_logger"),
                     "Spinning executor...");
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    EXPECT_TRUE(message_robot_received);
    RCLCPP_INFO(rclcpp::get_logger("test_logger"), "Robot data expected.");

    EXPECT_TRUE(message_ball_received);
    RCLCPP_INFO(rclcpp::get_logger("test_logger"), "Ball data expected.");
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
