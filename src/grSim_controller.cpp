// Copyright 2024 Oxebots
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "oxebots_comms/grSim_controller.hpp"

GrSimController::GrSimController() : rclcpp::Node("grSim_controller_node")
{
    RCLCPP_INFO(rclcpp::get_logger("GrSimController"), "Starting grSim controller module...");

    // Declare parameters
    const std::string simulator_ip = declare_parameter<std::string>("simulator_ip", "127.0.0.1");
    const int simulator_port = declare_parameter<int>("simulator_port", 10301);
    const std::string interface_ip = declare_parameter<std::string>("interface_ip", "");
    is_yellow_team_ = declare_parameter<bool>("is_yellow_team", false);
    const std::string command_topic = declare_parameter<std::string>("command_topic", "robot_commands");

    udp_sender_ = std::make_unique<UdpSender<RobotControl>>(simulator_ip, static_cast<uint16_t>(simulator_port), interface_ip);

    // Create subscription for robot commands
    command_subscription_ = create_subscription<oxebots_interfaces::msg::RobotCmd>(
      command_topic, 10, std::bind(&GrSimController::command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("GrSimController"), "Game sender module started, sending to %s:%d as %s team",
                simulator_ip.c_str(), simulator_port, is_yellow_team_ ? "yellow" : "blue");
}

GrSimController::~GrSimController() { RCLCPP_INFO(rclcpp::get_logger("GrSimController"), "Stopping game sender module..."); }

void GrSimController::command_callback(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg)
{
    RCLCPP_DEBUG(rclcpp::get_logger("GrSimController"), "Received command, building control packet...");

    RobotControl packet;

    for (const auto & robot_command_data : msg->robots)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("GrSimController"), "Adding robot_command for id %d", robot_command_data.id);

        auto * robot_cmd = packet.add_robot_commands();

        robot_cmd->set_id(robot_command_data.id);
        robot_cmd->set_kick_speed(robot_command_data.kick_speed);
        robot_cmd->set_dribbler_speed(0);  // No dribbler
        robot_cmd->set_kick_angle(0);      // No kick angle

        auto * move_cmd = robot_cmd->mutable_move_command();
        auto * global_velocity = move_cmd->mutable_global_velocity();

        // Set GLOBAL velocities as requested
        global_velocity->set_x(robot_command_data.x_velocity);
        global_velocity->set_y(robot_command_data.y_velocity);
        global_velocity->set_angular(robot_command_data.angular_velocity);
    }

    if (packet.robot_commands_size() > 0)
    {
        udp_sender_->send(packet);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GrSimController>());
    rclcpp::shutdown();
    return 0;
}
