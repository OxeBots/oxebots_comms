#pragma once
#include <rclcpp/rclcpp.hpp>

#include <vector>

#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_cmd_data.hpp"
#include "robot_data_sender.hpp"
#include "ssl_simulation_robot_control.pb.h"

class CommsNode : public rclcpp::Node
{
   private:
   std::vector<RobotCommand> proto_command_list;
   int robot_list_size;
   
   rclcpp::Subscription<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_sub;
   
   void HandleCmd(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg);
   
   public:
    std::unique_ptr<RobotDataSender> data_sender; // Depois privar
    CommsNode();
};
