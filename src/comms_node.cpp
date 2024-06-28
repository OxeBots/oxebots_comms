#include "../include/oxebots_comms/comms_node.hpp"

CommsNode::CommsNode() : rclcpp::Node("oxebots_comms")
{
    RCLCPP_INFO(this->get_logger(), "Starting comms module...");
    this->declare_parameter("host", "127.0.0.1");
    this->declare_parameter("port", "8001");
    this->declare_parameter("topic", "send_comms");
    this->declare_parameter("topic_retention", 10);
    this->declare_parameter("robot_amount", 3);

    RCLCPP_DEBUG(this->get_logger(), "Creating SendData object");
    this->send_data = new SendData(this->get_parameter("host").as_string(),
                                   this->get_parameter("port").as_string());

    RCLCPP_DEBUG(this->get_logger(), "Creating robot command list");
    this->robot_list_size = this->get_parameter("robot_amount").as_int();

    std::string topic = this->get_parameter("topic").as_string();
    RCLCPP_DEBUG(this->get_logger(), "Creating subscription to %s",
                 topic.c_str());
    this->subscription =
      this->create_subscription<oxebots_interfaces::msg::SendData>(
        topic, this->get_parameter("topic_retention").as_int(),
        std::bind(&CommsNode::HandleSubscription, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Comms module started");
}

void CommsNode::HandleSubscription(
  const oxebots_interfaces::msg::SendData::SharedPtr msg)
{
    oxebots_interfaces::msg::RobotData robot_command_list[] = {
      msg->robot1, msg->robot2, msg->robot3};
    RobotControl * robot_control = new RobotControl();
    for (int i = 0; i < this->robot_list_size; i++)
    {
        RobotCommand * robot_command = new RobotCommand();
        robot_command->set_id(robot_command_list[i].id);
        robot_command->set_kick_speed(robot_command_list[i].kick);

        MoveWheelVelocity * move_wheel_velocity = new MoveWheelVelocity();
        move_wheel_velocity->set_front_left(robot_command_list[i].front_left);
        move_wheel_velocity->set_front_right(
          robot_command_list[i].front_right);
        move_wheel_velocity->set_back_left(robot_command_list[i].back_left);
        move_wheel_velocity->set_back_right(robot_command_list[i].back_right);
        RobotMoveCommand * robot_move_command = new RobotMoveCommand();
        robot_move_command->set_allocated_wheel_velocity(move_wheel_velocity);
        robot_command->set_allocated_move_command(robot_move_command);
        robot_control->add_robot_commands()->CopyFrom(*robot_command);
    }

    this->send_data->SendControl(*robot_control);
}
