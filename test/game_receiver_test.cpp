#include <gtest/gtest.h>

#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <rclcpp/rclcpp.hpp>

#include "oxebots_comms/game_receiver_node.hpp"
#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/robot_game_data.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"
#include "oxebots_interfaces/ssl_vision_detection.pb.h"
#include "oxebots_interfaces/ssl_vision_wrapper.pb.h"
#include "oxebots_comms/game_receiver.hpp"

class GameReceiverTest : public ::testing::Test
{
   protected:
    void SetUp() override { 
      boost::asio::io_context io_context;
      node_ = std::make_shared<GameReceiver>(io_context); 
    }

    std::shared_ptr<GameReceiver> node_;
};

TEST_F(GameReceiverTest, TestUdpReceivePublish)
{
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    bool message_robot_received = false;
    bool message_ball_received = false;

    SSL_WrapperPacket * data_comp = new SSL_WrapperPacket();
    SSL_DetectionFrame * detection_frame_data = new SSL_DetectionFrame();

    auto detection_ball_data = detection_frame_data->add_balls();
    detection_ball_data->set_x(1);
    detection_ball_data->set_y(2);
    detection_ball_data->set_z(3);

    for (int i = 0; i < 3; i++)
    {
        auto detection_robot_blue = detection_frame_data->add_robots_blue();
        auto detection_robot_yellow =
          detection_frame_data->add_robots_yellow();

        detection_robot_blue->set_y(i);
        detection_robot_blue->set_x(i);
        detection_robot_blue->set_orientation(i);

        detection_robot_yellow->set_y(i);
        detection_robot_yellow->set_x(i);
        detection_robot_yellow->set_orientation(i);
    }

    data_comp->set_allocated_detection(detection_frame_data);
    std::string serialized_data_comp = data_comp->SerializeAsString();

    ASSERT_NE(serialized_data_comp, "");

    oxebots_interfaces::msg::RobotPosition received_robot_data;
    auto robot_subscription =
      node_->create_subscription<oxebots_interfaces::msg::RobotPosition>(
        "robot_data", 10,
        [&message_robot_received, &received_robot_data](
          const oxebots_interfaces::msg::RobotPosition::SharedPtr msg) {
            received_robot_data.allies = msg->allies;
            received_robot_data.enemies = msg->enemies;
            message_robot_received = true;
        });

    oxebots_interfaces::msg::BallPosition received_ball_data;
    auto ball_subscription =
      node_->create_subscription<oxebots_interfaces::msg::BallPosition>(
        "ball_data", 10,
        [&message_ball_received,
         &received_ball_data](oxebots_interfaces::msg::BallPosition msg) {
            received_ball_data.x = msg.x;
            received_ball_data.y = msg.y;
            received_ball_data.z = msg.z;
            message_ball_received = true;
        });

    boost::asio::ip::udp::endpoint remote_endpoint(
      boost::asio::ip::address::from_string("224.5.23.2"), 10020);
    boost::asio::io_service io_service;
    udp::socket test_socket(io_service, udp::endpoint(udp::v4(), 0));

    test_socket.send_to(boost::asio::buffer(serialized_data_comp),
                        remote_endpoint);

    auto start = std::chrono::steady_clock::now();
    while (
      !message_robot_received && !message_ball_received &&
      (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)))
    {
        executor.spin_some();
    }

    EXPECT_TRUE(message_robot_received);
    EXPECT_TRUE(message_ball_received);
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
