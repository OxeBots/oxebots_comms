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

#include "oxebots_comms/game_receiver.hpp"
#include "oxebots_interfaces/msg/vector2f.hpp"
#include "oxebots_interfaces/msg/ssl_field_line_segment.hpp"
#include "oxebots_interfaces/msg/ssl_circular_arc.hpp"
#include "oxebots_interfaces/msg/ssl_field_size.hpp"
#include "oxebots_interfaces/msg/ssl_camera_calibration.hpp"
#include "oxebots_interfaces/msg/ssl_ball_model_straight_two_phase.hpp"
#include "oxebots_interfaces/msg/ssl_ball_model_chip_fixed_loss.hpp"
#include "oxebots_interfaces/msg/ssl_geometry_models.hpp"


// Helper conversion functions from Protobuf to ROS messages
oxebots_interfaces::msg::Vector2f map_to_ros(const Vector2f& pb_vec) {
    oxebots_interfaces::msg::Vector2f ros_vec;
    ros_vec.x = pb_vec.x();
    ros_vec.y = pb_vec.y();
    return ros_vec;
}

oxebots_interfaces::msg::SSLFieldLineSegment map_to_ros(const SSL_FieldLineSegment& pb_line) {
    oxebots_interfaces::msg::SSLFieldLineSegment ros_line;
    ros_line.name = pb_line.name();
    ros_line.p1 = map_to_ros(pb_line.p1());
    ros_line.p2 = map_to_ros(pb_line.p2());
    ros_line.thickness = pb_line.thickness();
    ros_line.type = pb_line.type();
    return ros_line;
}

oxebots_interfaces::msg::SSLCircularArc map_to_ros(const SSL_FieldCircularArc& pb_arc) {
    oxebots_interfaces::msg::SSLCircularArc ros_arc;
    ros_arc.name = pb_arc.name();
    ros_arc.center = map_to_ros(pb_arc.center());
    ros_arc.radius = pb_arc.radius();
    ros_arc.a1 = pb_arc.a1();
    ros_arc.a2 = pb_arc.a2();
    ros_arc.thickness = pb_arc.thickness();
    ros_arc.type = pb_arc.type();
    return ros_arc;
}

oxebots_interfaces::msg::SSLFieldSize map_to_ros(const SSL_GeometryFieldSize& pb_field) {
    oxebots_interfaces::msg::SSLFieldSize ros_field;
    ros_field.field_length = pb_field.field_length();
    ros_field.field_width = pb_field.field_width();
    ros_field.goal_width = pb_field.goal_width();
    ros_field.goal_depth = pb_field.goal_depth();
    ros_field.boundary_width = pb_field.boundary_width();
    for (const auto& line : pb_field.field_lines()) {
        ros_field.field_lines.push_back(map_to_ros(line));
    }
    for (const auto& arc : pb_field.field_arcs()) {
        ros_field.field_arcs.push_back(map_to_ros(arc));
    }
    if (pb_field.has_penalty_area_depth()) ros_field.penalty_area_depth = pb_field.penalty_area_depth();
    if (pb_field.has_penalty_area_width()) ros_field.penalty_area_width = pb_field.penalty_area_width();
    if (pb_field.has_center_circle_radius()) ros_field.center_circle_radius = pb_field.center_circle_radius();
    if (pb_field.has_line_thickness()) ros_field.line_thickness = pb_field.line_thickness();
    if (pb_field.has_goal_center_to_penalty_mark()) ros_field.goal_center_to_penalty_mark = pb_field.goal_center_to_penalty_mark();
    if (pb_field.has_goal_height()) ros_field.goal_height = pb_field.goal_height();
    if (pb_field.has_ball_radius()) ros_field.ball_radius = pb_field.ball_radius();
    if (pb_field.has_max_robot_radius()) ros_field.max_robot_radius = pb_field.max_robot_radius();
    return ros_field;
}

oxebots_interfaces::msg::SSLCameraCalibration map_to_ros(const SSL_GeometryCameraCalibration& pb_calib) {
    oxebots_interfaces::msg::SSLCameraCalibration ros_calib;
    ros_calib.camera_id = pb_calib.camera_id();
    ros_calib.focal_length = pb_calib.focal_length();
    ros_calib.principal_point_x = pb_calib.principal_point_x();
    ros_calib.principal_point_y = pb_calib.principal_point_y();
    ros_calib.distortion = pb_calib.distortion();
    ros_calib.q0 = pb_calib.q0();
    ros_calib.q1 = pb_calib.q1();
    ros_calib.q2 = pb_calib.q2();
    ros_calib.q3 = pb_calib.q3();
    ros_calib.tx = pb_calib.tx();
    ros_calib.ty = pb_calib.ty();
    ros_calib.tz = pb_calib.tz();
    if (pb_calib.has_derived_camera_world_tx()) ros_calib.derived_camera_world_tx = pb_calib.derived_camera_world_tx();
    if (pb_calib.has_derived_camera_world_ty()) ros_calib.derived_camera_world_ty = pb_calib.derived_camera_world_ty();
    if (pb_calib.has_derived_camera_world_tz()) ros_calib.derived_camera_world_tz = pb_calib.derived_camera_world_tz();
    if (pb_calib.has_pixel_image_width()) ros_calib.pixel_image_width = pb_calib.pixel_image_width();
    if (pb_calib.has_pixel_image_height()) ros_calib.pixel_image_height = pb_calib.pixel_image_height();
    return ros_calib;
}

oxebots_interfaces::msg::SSLBallModelStraightTwoPhase map_to_ros(const SSL_BallModelStraightTwoPhase& pb_model) {
    oxebots_interfaces::msg::SSLBallModelStraightTwoPhase ros_model;
    ros_model.acc_slide = pb_model.acc_slide();
    ros_model.acc_roll = pb_model.acc_roll();
    ros_model.k_switch = pb_model.k_switch();
    return ros_model;
}

oxebots_interfaces::msg::SSLBallModelChipFixedLoss map_to_ros(const SSL_BallModelChipFixedLoss& pb_model) {
    oxebots_interfaces::msg::SSLBallModelChipFixedLoss ros_model;
    ros_model.damping_xy_first_hop = pb_model.damping_xy_first_hop();
    ros_model.damping_xy_other_hops = pb_model.damping_xy_other_hops();
    ros_model.damping_z = pb_model.damping_z();
    return ros_model;
}

oxebots_interfaces::msg::SSLGeometryModels map_to_ros(const SSL_GeometryModels& pb_models) {
    oxebots_interfaces::msg::SSLGeometryModels ros_models;
    if (pb_models.has_straight_two_phase()) ros_models.straight_two_phase = map_to_ros(pb_models.straight_two_phase());
    if (pb_models.has_chip_fixed_loss()) ros_models.chip_fixed_loss = map_to_ros(pb_models.chip_fixed_loss());
    return ros_models;
}

GameReceiver::GameReceiver()
: rclcpp::Node("game_receiver_node"),
  UdpReceiver<SSL_WrapperPacket>(declare_parameter<std::string>("ssl_vision_ip", "224.5.23.2"),
                                 declare_parameter<uint16_t>("ssl_vision_port", 10006),
                                 declare_parameter<std::string>("interface_ip", ""))
{
    RCLCPP_INFO(rclcpp::get_logger("GameReceiver"), "Starting game receiver module...");

    declare_parameter("topic_retention", 10);
    declare_parameter("robot_topic", "robot_data");
    declare_parameter("ball_topic", "ball_data");
    declare_parameter("geometry_topic", "field_geometry");
    declare_parameter("is_yellow_team", false);

    robot_publisher = create_publisher<oxebots_interfaces::msg::RobotPosition>(
      get_parameter("robot_topic").as_string(), get_parameter("topic_retention").as_int());

    ball_publisher = create_publisher<oxebots_interfaces::msg::BallPosition>(get_parameter("ball_topic").as_string(),
                                                                             get_parameter("topic_retention").as_int());
    
    geometry_publisher = create_publisher<oxebots_interfaces::msg::SSLGeometryData>(get_parameter("geometry_topic").as_string(),
                                                                             rclcpp::QoS(1).transient_local().reliable());

    is_yellow_team = get_parameter("is_yellow_team").as_bool();

    RCLCPP_INFO(rclcpp::get_logger("GameReceiver"), "Game receiver module started");
}

GameReceiver::~GameReceiver() { RCLCPP_INFO(rclcpp::get_logger("GameReceiver"), "Stopping game receiver module..."); }

void GameReceiver::on_receive(const SSL_WrapperPacket & packet)
{
    RCLCPP_DEBUG(rclcpp::get_logger("GameReceiver"), "Received packet");
    SSL_DetectionFrame detection = packet.detection();
    std::vector<oxebots_interfaces::msg::RobotGameData> yellow_robots;
    std::vector<oxebots_interfaces::msg::RobotGameData> blue_robots;

    RCLCPP_DEBUG(rclcpp::get_logger("GameReceiver"), "Parsing frame");
    RCLCPP_DEBUG(rclcpp::get_logger("GameReceiver"), "Frame number: %d", detection.frame_number());
    RCLCPP_DEBUG(rclcpp::get_logger("GameReceiver"), "Camera ID: %d", detection.camera_id());

    RCLCPP_DEBUG(rclcpp::get_logger("GameReceiver"), "Parsing packet");
    RCLCPP_DEBUG(rclcpp::get_logger("GameReceiver"), "Yellow robots: %d", detection.robots_yellow_size());
    if (detection.robots_yellow_size() > 0)
    {
        for (auto robot : detection.robots_yellow())
        {
            oxebots_interfaces::msg::RobotGameData robot_data;
            robot_data.id = robot.robot_id();
            robot_data.x = robot.x();
            robot_data.y = robot.y();
            robot_data.orientation = robot.orientation();
            yellow_robots.push_back(robot_data);
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("GameReceiver"), "Blue robots: %d", detection.robots_blue_size());

    if (detection.robots_blue_size() > 0)
    {
        for (auto robot : detection.robots_blue())
        {
            oxebots_interfaces::msg::RobotGameData robot_data;
            robot_data.id = robot.robot_id();
            robot_data.x = robot.x();
            robot_data.y = robot.y();
            robot_data.orientation = robot.orientation();
            blue_robots.push_back(robot_data);
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("GameReceiver"), "Parsing ball");

    if (detection.balls_size() > 0)
    {
        oxebots_interfaces::msg::BallPosition ball_data;
        ball_data.x = detection.balls(0).x();
        ball_data.y = detection.balls(0).y();
        ball_data.z = detection.balls(0).z();
        PublishBallData(ball_data);
    }

    RCLCPP_DEBUG(rclcpp::get_logger("GameReceiver"), "Publishing data");

    if (is_yellow_team)
        PublishRobotData(yellow_robots, blue_robots);
    else
        PublishRobotData(blue_robots, yellow_robots);
    
    if (packet.has_geometry()) {
        PublishGeometryData(packet.geometry());
    }
}

void GameReceiver::PublishRobotData(std::vector<oxebots_interfaces::msg::RobotGameData> allies,
                                    std::vector<oxebots_interfaces::msg::RobotGameData> enemies)
{
    oxebots_interfaces::msg::RobotPosition robot_data;
    robot_data.allies = allies;
    robot_data.enemies = enemies;

    robot_publisher->publish(robot_data);
}

void GameReceiver::PublishBallData(oxebots_interfaces::msg::BallPosition ball_data)
{
    ball_publisher->publish(ball_data);
}

void GameReceiver::PublishGeometryData(const SSL_GeometryData & geometry)
{
    oxebots_interfaces::msg::SSLGeometryData ros_geometry_data;
    ros_geometry_data.field = map_to_ros(geometry.field());
    for (const auto& calib : geometry.calib()) {
        ros_geometry_data.calib.push_back(map_to_ros(calib));
    }
    if (geometry.has_models()) {
        ros_geometry_data.models = map_to_ros(geometry.models());
    }
    geometry_publisher->publish(ros_geometry_data);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameReceiver>());
    rclcpp::shutdown();
    return 0;
}
