#pragma once
#include <rclcpp/rclcpp.hpp>

#include <vector>

#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "robot_data_sender.hpp"
#include "ssl_simulation_robot_control.pb.h"

class CommsNode : public rclcpp::Node
{
   private:
    RobotDataSender * data_sender;
    std::vector<RobotCommand> proto_command_list;
    int robot_list_size;

    int count;

    rclcpp::Subscription<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_sub;

    void HandleCmd(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg);

   public:
    CommsNode();
};
