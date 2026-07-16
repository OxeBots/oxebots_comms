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

#include "oxebots_comms/ra_controller.hpp"
#include <cmath>

RAController::RAController() : rclcpp::Node("ra_controller_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting RA controller module (Local Velocity)...");

    // Declare parameters
    const std::string simulator_ip = declare_parameter<std::string>("simulator_ip", "127.0.0.1");
    int simulator_port = declare_parameter<int>("simulator_port", 10301);
    const std::string interface_ip = declare_parameter<std::string>("interface_ip", "");
    is_yellow_team_ = declare_parameter<bool>("is_yellow_team", false);
    const std::string command_topic = declare_parameter<std::string>("command_topic", "robot_commands");
    const std::string game_data_topic = declare_parameter<std::string>("game_data_topic", "game_data");

    // Port logic for grSim/RA: Blue=10301, Yellow=10302
    if (is_yellow_team_ && simulator_port == 10301) {
        RCLCPP_INFO(get_logger(), "Time amarelo detectado. Ajustando porta para 10302 automaticamente.");
        simulator_port = 10302;
    } else if (!is_yellow_team_ && simulator_port == 10302) {
        RCLCPP_INFO(get_logger(), "Time azul detectado. Ajustando porta para 10301 automaticamente.");
        simulator_port = 10301;
    }

    udp_sender_ = std::make_unique<UdpSender<RobotControl>>(simulator_ip, static_cast<uint16_t>(simulator_port), interface_ip);

    // Create subscription for robot commands
    command_subscription_ = create_subscription<oxebots_interfaces::msg::RobotCmd>(
      command_topic, 10, std::bind(&RAController::command_callback, this, std::placeholders::_1));

    // Create subscription for game data (to get orientation)
    game_data_subscription_ = create_subscription<oxebots_interfaces::msg::GameData>(
      game_data_topic, 10, std::bind(&RAController::game_data_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RA controller module started, sending to %s:%d as %s team",
                simulator_ip.c_str(), simulator_port, is_yellow_team_ ? "yellow" : "blue");
}

RAController::~RAController() { RCLCPP_INFO(this->get_logger(), "Stopping RA controller module..."); }

void RAController::game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg)
{
    // Update orientations and states for our team (allies)
    for (const auto & robot : msg->robots.allies)
    {
        robot_orientations_[robot.id] = robot.orientation;
        robot_states_[robot.id] = RobotState{robot.x, robot.y, robot.orientation};
    }
}

void RAController::command_callback(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg)
{
    RobotControl packet;

    for (const auto & robot_command_data : msg->robots)
    {
        auto * robot_cmd = packet.add_robot_commands();

        robot_cmd->set_id(robot_command_data.id);
        robot_cmd->set_kick_speed(robot_command_data.kick_speed);
        robot_cmd->set_dribbler_speed(0);
        robot_cmd->set_kick_angle(0);

        auto * move_cmd = robot_cmd->mutable_move_command();
        auto * local_velocity = move_cmd->mutable_local_velocity();

        // Get current orientation (default to 0 if not found)
        float theta = 0.0f;
        if (robot_orientations_.find(robot_command_data.id) != robot_orientations_.end()) {
            theta = robot_orientations_[robot_command_data.id];
        }

        // Coordinate transformation: Global to Local
        // v_forward = vx * cos(theta) + vy * sin(theta)
        // v_left = -vx * sin(theta) + vy * cos(theta)
        float vx = robot_command_data.x_velocity;
        float vy = robot_command_data.y_velocity;
        
        clampVelocities(robot_command_data.id, vx, vy);
        
        float v_forward = vx * std::cos(theta) + vy * std::sin(theta);
        float v_left = -vx * std::sin(theta) + vy * std::cos(theta);

        local_velocity->set_forward(v_forward);
        local_velocity->set_left(v_left);
        local_velocity->set_angular(robot_command_data.angular_velocity);

        // Log para debug (aparecerá a cada 1 segundo para não poluir)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "Robot %d: Global(%.2f, %.2f) -> Local(%.2f, %.2f) at theta=%.2f", 
                             robot_command_data.id, vx, vy, v_forward, v_left, theta);
    }

    if (packet.robot_commands_size() > 0)
    {
        udp_sender_->send(packet);
    }
}

void RAController::clampVelocities(uint32_t robot_id, float& vx, float& vy) {
    (void)vy;
    if (robot_states_.find(robot_id) == robot_states_.end()) {
        return; // Don't know where the robot is, can't clamp
    }
    const auto& state = robot_states_[robot_id];
    float rx = state.x;
    float ry = state.y;

    // Penalty Area Dimensions in mm (with safety margin)
    // Goal line is around 2200mm. Penalty area is 500mm deep, 1350mm wide.
    // Safety margin of 110mm (robot radius 90mm + 20mm extra tolerance)
    float area_x_limit = 2200.0f - 500.0f - 110.0f; // 1590 mm
    float area_y_limit = (1350.0f / 2.0f) + 110.0f; // 785 mm

    bool block_positive = true;
    bool block_negative = true;

    if (robot_id == 0) {
        // Goalkeeper is allowed in its own area, but not the opponent's.
        // We determine its own area by which side it is currently on.
        if (rx > 0.0f) {
            block_positive = false; // own side is positive
        } else {
            block_negative = false; // own side is negative
        }
    }

    // 1. Block positive area (X > area_x_limit, |Y| < area_y_limit)
    if (block_positive) {
        // If robot is in/near the area and moving deeper
        if (rx >= area_x_limit && std::abs(ry) < area_y_limit) {
            if (vx > 0.0f) {
                vx = 0.0f;
            }
            // If the robot is already pushed too deep into the area, apply a gentle push back
            if (rx > (area_x_limit + 30.0f)) {
                vx = -0.3f;
            }
        }
    }

    // 2. Block negative area (X < -area_x_limit, |Y| < area_y_limit)
    if (block_negative) {
        // If robot is in/near the area and moving deeper
        if (rx <= -area_x_limit && std::abs(ry) < area_y_limit) {
            if (vx < 0.0f) {
                vx = 0.0f;
            }
            // If the robot is already pushed too deep into the area, apply a gentle push back
            if (rx < -(area_x_limit + 30.0f)) {
                vx = 0.3f;
            }
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RAController>());
    rclcpp::shutdown();
    return 0;
}
