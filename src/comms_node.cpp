#include "./headers/comms_node.hpp"

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
    this->robot_command_list =
      std::vector<RobotCommand *>(this->robot_list_size, nullptr);

    this->count = 0;

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
    int real_id = msg->id - 1;
    if (real_id >= this->robot_list_size || real_id < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Robot ID out of bounds");
        return;
    }

    // This is a new command
    if (!this->robot_command_list[real_id])
    {
        RCLCPP_DEBUG(this->get_logger(), "New command for robot %d", msg->id);
        this->count++;
    }

    MoveWheelVelocity * move_wheel_velocity = new MoveWheelVelocity();
    RobotMoveCommand * move_command = new RobotMoveCommand();
    move_command->set_allocated_wheel_velocity(move_wheel_velocity);
    move_wheel_velocity->set_front_left(msg->front_left);
    move_wheel_velocity->set_front_right(msg->front_right);
    move_wheel_velocity->set_back_left(msg->back_left);
    move_wheel_velocity->set_back_right(msg->back_right);

    RobotCommand * command = new RobotCommand();
    command->set_id(msg->id);
    command->set_kick_speed(msg->kick);

    command->set_allocated_move_command(move_command);

    this->robot_command_list[real_id] = command;

    if (this->count == this->robot_list_size)
    {
        RCLCPP_DEBUG(this->get_logger(), "Sending control to robots");
        RobotControl * control = new RobotControl();
        for (int i = 0; i < this->robot_list_size; i++)
        {
            control->add_robot_commands()->CopyFrom(
              *this->robot_command_list[i]);

            delete this->robot_command_list[i]->release_move_command()->release_wheel_velocity();
            delete this->robot_command_list[i]->release_move_command();
            delete this->robot_command_list[i];

            this->robot_command_list[i] = nullptr;
        }

        this->send_data->SendControl(*control);
        delete control;

        this->count = 0;
    }
}
