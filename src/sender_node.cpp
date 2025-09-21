#include "oxebots_comms/sender_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommsNode>());
    rclcpp::shutdown();
    return 0;
}

CommsNode::CommsNode() : rclcpp::Node("oxebots_comms")
{
    RCLCPP_INFO(get_logger(), "Starting comms module...");

    declare_parameter("host", "127.0.0.1");
    declare_parameter("port", 10301);
    declare_parameter("topic", "send_comms");
    declare_parameter("topic_retention", 10);
    declare_parameter("robot_amount", 3);

    robot_list_size = get_parameter("robot_amount").as_int();
    proto_command_list.resize(robot_list_size);

    RCLCPP_DEBUG(get_logger(), "Creating RobotDataSender");
    RCLCPP_INFO(get_logger(), "Creating RobotDataSender");
    std::string host = get_parameter("host").as_string();
    int port = get_parameter("port").as_int();
    
    data_sender = std::make_unique<RobotDataSender>(host,port);

    RCLCPP_INFO(get_logger(), "Create addr: %s port: %u",
                inet_ntoa(data_sender->server_address.sin_addr),
                ntohs(data_sender->server_address.sin_port));
        
    std::string topic = get_parameter("topic").as_string();

    cmd_sub = create_subscription<oxebots_interfaces::msg::RobotCmd>(
      topic, get_parameter("topic_retention").as_int(),
      std::bind(&CommsNode::HandleCmd, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Comms module started");
}

void CommsNode::HandleCmd(
  const oxebots_interfaces::msg::RobotCmd::SharedPtr msg)
{
    RobotControl robot_control;
    RCLCPP_INFO(get_logger(), "Received command");

    for (const auto & robot_command_data : msg->robots)
    {
        RCLCPP_INFO(get_logger(), "Adding o robot_command for id %d",
                    robot_command_data.id);

        RobotCommand* robot_command = robot_control.add_robot_commands();
        robot_command->set_id(robot_command_data.id);
        robot_command->set_kick_speed(robot_command_data.kick);

        RobotMoveCommand* move_command = robot_command->mutable_move_command();
        MoveWheelVelocity* wheel_velocity = move_command->mutable_wheel_velocity();

        wheel_velocity->set_front_left(robot_command_data.front_left);
        wheel_velocity->set_front_right(robot_command_data.front_right);
        wheel_velocity->set_back_left(robot_command_data.back_left);
        wheel_velocity->set_back_right(robot_command_data.back_right);
        RCLCPP_INFO(get_logger(), "Front_Left %f",
                    robot_command_data.front_left);
    }
    RCLCPP_INFO(get_logger(), "Conteúdo do comando a ser enviado:\n---\n%s---",
                robot_control.DebugString().c_str());

    RCLCPP_INFO(get_logger(), "Command send to: %s port: %u",
                inet_ntoa(data_sender->server_address.sin_addr),
                ntohs(data_sender->server_address.sin_port));

     if (!data_sender->SendControl(robot_control))
    {
        RCLCPP_INFO(get_logger(), "Falha ao enviar o pacote de controle do robô!");
        // Aqui você pode decidir o que fazer, como tentar de novo ou apenas ignorar.
    }
    else{
        RCLCPP_INFO(get_logger(), "Pacote de controle do robô enviado com sucesso!");
    }
}
