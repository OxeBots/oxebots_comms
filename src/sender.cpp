#include "oxebots_comms/sender.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Comms>());
    rclcpp::shutdown();
    return 0;
}

Comms::Comms() : rclcpp::Node("oxebots_comms")
{
    RCLCPP_INFO(get_logger(), "Starting comms module...");

    declare_parameter("host", "127.0.0.1");
    declare_parameter("port", 8001);
    declare_parameter("topic", "send_comms");
    declare_parameter("topic_retention", 10);
    declare_parameter("robot_amount", 3);

    robot_list_size = get_parameter("robot_amount").as_int();
    proto_command_list.resize(robot_list_size);

    RCLCPP_DEBUG(get_logger(), "Creating RobotDataSender");

    data_sender = new RobotDataSender(get_parameter("host").as_string(),
                                      get_parameter("port").as_int());

    std::string topic = get_parameter("topic").as_string();

    cmd_sub = create_subscription<oxebots_interfaces::msg::RobotCmd>(
      topic, get_parameter("topic_retention").as_int(),
      std::bind(&Comms::HandleCmd, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Comms module started");
}

void Comms::HandleCmd(
  const oxebots_interfaces::msg::RobotCmd::SharedPtr msg)
{
    RobotControl robot_control;

    for (const auto & robot_command_data : msg->robots)
    {
        RobotCommand robot_command;
        MoveWheelVelocity move_wheel_velocity;
        RobotMoveCommand robot_move_command;

        robot_command.set_id(robot_command_data.id);
        robot_command.set_kick_speed(robot_command_data.kick);
        move_wheel_velocity.set_front_left(robot_command_data.front_left);
        move_wheel_velocity.set_front_right(robot_command_data.front_right);
        move_wheel_velocity.set_back_left(robot_command_data.back_left);
        move_wheel_velocity.set_back_right(robot_command_data.back_right);

        robot_move_command.set_allocated_wheel_velocity(&move_wheel_velocity);
        robot_command.set_allocated_move_command(&robot_move_command);

        robot_control.add_robot_commands()->CopyFrom(robot_command);
    }

    data_sender->SendControl(robot_control);
}
