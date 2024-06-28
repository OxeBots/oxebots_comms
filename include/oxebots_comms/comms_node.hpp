#pragma once
#include <rclcpp/rclcpp.hpp>

#include "./send_data.hpp"
#include "oxebots_interfaces/msg/send_data.hpp"
#include "ssl_simulation_robot_control.pb.h"

class CommsNode : public rclcpp::Node
{
   private:
    SendData * send_data;
    int robot_list_size;

    rclcpp::Subscription<oxebots_interfaces::msg::SendData>::SharedPtr
      subscription;

    void HandleSubscription(
      const oxebots_interfaces::msg::SendData::SharedPtr msg);

   public:
    CommsNode();
};
