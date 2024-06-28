#include "oxebots_comms/comms_node.hpp"

CommsNode::CommsNode() : rclcpp::Node("oxebots_comms")
{
    RCLCPP_INFO(get_logger(), "Starting comms module...");

    declare_parameter("host", "127.0.0.1");
    declare_parameter("port", "8001");
    declare_parameter("topic", "send_comms");
    declare_parameter("topic_retention", 10);
    declare_parameter("robot_amount", 3);

    robot_list_size = get_parameter("robot_amount").as_int();
    proto_command_list.resize(robot_list_size);

    RCLCPP_DEBUG(this->get_logger(), "Creating RobotDataSender");

    data_sender = new RobotDataSender(get_parameter("host").as_string(),
                                      get_parameter("port").as_string());

    count = 0;

    std::string topic = get_parameter("topic").as_string();

    cmd_sub = create_subscription<oxebots_interfaces::msg::RobotCmd>(
      topic, get_parameter("topic_retention").as_int(),
      std::bind(&CommsNode::HandleCmd, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Comms module started");
}

void CommsNode::HandleCmd(
  const oxebots_interfaces::msg::RobotCmd::SharedPtr msg)
{
    int real_id = msg->id - 1;
    if (real_id >= robot_list_size || real_id < 0)
    {
        RCLCPP_ERROR(get_logger(), "Robot ID out of bounds");
        return;
    }

    auto & command = proto_command_list[real_id];

    if (command.id() == 0)  // Assuming 0 means uninitialized
    {
        RCLCPP_DEBUG(get_logger(), "New command for robot %d", msg->id);
        count++;
    }

    auto * move_wheel_velocity =
      command.mutable_move_command()->mutable_wheel_velocity();
    move_wheel_velocity->set_front_left(msg->front_left);
    move_wheel_velocity->set_front_right(msg->front_right);
    move_wheel_velocity->set_back_left(msg->back_left);
    move_wheel_velocity->set_back_right(msg->back_right);

    command.set_id(msg->id);
    command.set_kick_speed(msg->kick);

    if (count == robot_list_size)
    {
        RCLCPP_DEBUG(get_logger(), "Sending control to robots");
        RobotControl control;
        for (auto & cmd : proto_command_list)
        {
            control.add_robot_commands()->CopyFrom(cmd);
            cmd.Clear();
        }

        data_sender->SendControl(control);

        count = 0;
    }
}
