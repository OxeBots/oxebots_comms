#include "oxebots_comms/vision_receiver_node.hpp"

VisionReceiverNode::VisionReceiverNode(boost::asio::io_context & io_context)
: rclcpp::Node("oxebots_comms"),
  UdpDriver<SSL_WrapperPacket>(io_context),
  io_context(io_context)
{
    RCLCPP_INFO(get_logger(), "Starting vision receiver module...");

    declare_parameter("host", "224.5.23.2");
    declare_parameter("port", 10006);
    declare_parameter("robot_topic", "robot_data");
    declare_parameter("ball_topic", "ball_data");
    declare_parameter("geometry_topic", "geometry_data");
    declare_parameter("topic_retention", 10);
    declare_parameter("is_yellow_team", false);

    RCLCPP_DEBUG(get_logger(), "Creating RobotDataPublisher");

    robot_publisher = create_publisher<oxebots_interfaces::msg::RobotPosition>(
      get_parameter("robot_topic").as_string(),
      get_parameter("topic_retention").as_int());

    RCLCPP_DEBUG(get_logger(), "Creating BallDataPublisher");

    ball_publisher = create_publisher<oxebots_interfaces::msg::BallPosition>(
      get_parameter("ball_topic").as_string(),
      get_parameter("topic_retention").as_int());

    RCLCPP_DEBUG(get_logger(), "Creating GeometryPublisher");
    geometry_publisher = this->create_publisher<oxebots_interfaces::msg::SSLGeometryData>(
        get_parameter("geometry_topic").as_string(),  
        get_parameter("topic_retention").as_int());

    is_yellow_team = get_parameter("is_yellow_team").as_bool();

    RCLCPP_DEBUG(get_logger(), "Adding host...");

    add_host(get_parameter("host").as_string(),
             get_parameter("port").as_int());

    io_thread = std::thread([this]() { this->io_context.run(); });

    RCLCPP_INFO(get_logger(), "Vision receiver module started");
}

VisionReceiverNode::~VisionReceiverNode()
{
    RCLCPP_INFO(get_logger(), "Stopping vision receiver module...");
    io_context.stop();
    if (io_thread.joinable()) io_thread.join();
    stop();
}

void VisionReceiverNode::on_receive(const SSL_WrapperPacket &packet) {
    RCLCPP_DEBUG(get_logger(), "Received packet");

    if (packet.has_detection()) {
        RCLCPP_DEBUG(get_logger(), "Processing detection data");
        processDetectionData(packet.detection());
    }

    if (packet.has_geometry()) {
        RCLCPP_DEBUG(get_logger(), "Processing geometry data");
        processGeometryData(packet.geometry());
    }
}

void VisionReceiverNode::processGeometryData(const SSL_GeometryData &geometry) {
    // Convert field size
    oxebots_interfaces::msg::SSLFieldSize field_size;
    convertFieldSize(geometry, field_size);

    // Convert camera calibrations
    std::vector<oxebots_interfaces::msg::SSLCameraCalibration> camera_calibrations;
    convertCameraCalibrations(geometry, camera_calibrations);

    // Convert Models
    oxebots_interfaces::msg::SSLGeometryModels geometry_models;
    convertGeometryModels(geometry, geometry_models);

    RCLCPP_DEBUG(get_logger(), "Publishing geometry data");
    publishGeometryData(field_size, camera_calibrations, geometry_models);
}

void VisionReceiverNode::processDetectionData(const SSL_DetectionFrame &detection) {
    std::vector<oxebots_interfaces::msg::RobotGameData> yellow_robots;
    std::vector<oxebots_interfaces::msg::RobotGameData> blue_robots;

    RCLCPP_DEBUG(get_logger(), "Parsing frame");
    RCLCPP_DEBUG(get_logger(), "Frame number: %d", detection.frame_number());
    RCLCPP_DEBUG(get_logger(), "Camera ID: %d", detection.camera_id());

    // Parse yellow robots
    RCLCPP_DEBUG(get_logger(), "Yellow robots: %d", detection.robots_yellow_size());
    for (const auto &robot : detection.robots_yellow()) {
        oxebots_interfaces::msg::RobotGameData robot_data;
        robot_data.id = robot.robot_id();
        robot_data.x = robot.x();
        robot_data.y = robot.y();
        robot_data.orientation = robot.orientation();
        yellow_robots.push_back(robot_data);
    }

    // Parse blue robots
    RCLCPP_DEBUG(get_logger(), "Blue robots: %d", detection.robots_blue_size());
    for (const auto &robot : detection.robots_blue()) {
        oxebots_interfaces::msg::RobotGameData robot_data;
        robot_data.id = robot.robot_id();
        robot_data.x = robot.x();
        robot_data.y = robot.y();
        robot_data.orientation = robot.orientation();
        blue_robots.push_back(robot_data);
    }

    // Parse ball
    if (detection.balls_size() > 0) {
        oxebots_interfaces::msg::BallPosition ball_data;
        ball_data.x = detection.balls(0).x();
        ball_data.y = detection.balls(0).y();
        ball_data.z = detection.balls(0).z();
        PublishBallData(ball_data);
    }

    // Publish robot data based on the team
    RCLCPP_DEBUG(get_logger(), "Publishing data");
    if (is_yellow_team) {
        PublishRobotData(yellow_robots, blue_robots);
    } else {
        PublishRobotData(blue_robots, yellow_robots);
    }
}

void VisionReceiverNode::convertFieldSize(const SSL_GeometryData& geometry, oxebots_interfaces::msg::SSLFieldSize& field_size) {

    field_size.field_length = geometry.field().field_length();
    field_size.field_width = geometry.field().field_width();
    field_size.goal_width = geometry.field().goal_width();
    field_size.goal_depth = geometry.field().goal_depth();
    field_size.boundary_width = geometry.field().boundary_width();

    // Convert field lines
    for (const auto &line : geometry.field().field_lines()) {
        oxebots_interfaces::msg::SSLFieldLineSegment line_segment;
        line_segment.name = line.name();
        line_segment.p1.x = line.p1().x();
        line_segment.p1.y = line.p1().y();
        line_segment.p2.x = line.p2().x();
        line_segment.p2.y = line.p2().y();
        line_segment.thickness = line.thickness();
        if (line.has_type()) {
            line_segment.type = line.type();
        }
        field_size.field_lines.push_back(line_segment);
    }
    
    // Convert field arcs
    for (const auto &arc : geometry.field().field_arcs()) {
        oxebots_interfaces::msg::SSLCircularArc arc_segment;
        arc_segment.name = arc.name();
        arc_segment.center.x = arc.center().x();
        arc_segment.center.y = arc.center().y();
        arc_segment.radius = arc.radius();
        arc_segment.a1 = arc.a1();
        arc_segment.a2 = arc.a2();
        arc_segment.thickness = arc.thickness();
        if (arc.has_type()) {
            arc_segment.type = arc.type();
        }
        field_size.field_arcs.push_back(arc_segment);
    }


    // Optional fields
    if (geometry.field().has_penalty_area_depth()) {
        field_size.penalty_area_depth = geometry.field().penalty_area_depth();
    }

    if (geometry.field().has_penalty_area_width()) {
        field_size.penalty_area_width = geometry.field().penalty_area_width();
    }


    if (geometry.field().has_center_circle_radius()) {
        field_size.center_circle_radius = geometry.field().center_circle_radius();
    }

    if (geometry.field().has_line_thickness()) {
        field_size.line_thickness = geometry.field().line_thickness();
    }   

    if (geometry.field().has_goal_center_to_penalty_mark()) {
        field_size.goal_center_to_penalty_mark = geometry.field().goal_center_to_penalty_mark();
    }

    if (geometry.field().has_goal_height()) {
        field_size.goal_height = geometry.field().goal_height();
    }

    if (geometry.field().has_ball_radius()) {
        field_size.ball_radius = geometry.field().ball_radius();
    }

    if (geometry.field().has_max_robot_radius()) {
        field_size.max_robot_radius = geometry.field().max_robot_radius();
    } 
 }

void VisionReceiverNode::convertCameraCalibrations(const SSL_GeometryData& geometry, std::vector<oxebots_interfaces::msg::SSLCameraCalibration>& camera_calibrations) {
    // Convert camera calibrations
    for (const auto &calib : geometry.calib()) {
        oxebots_interfaces::msg::SSLCameraCalibration camera_calibration;
        
        camera_calibration.camera_id = calib.camera_id();
        camera_calibration.focal_length = calib.focal_length();
        camera_calibration.principal_point_x = calib.principal_point_x();
        camera_calibration.principal_point_y = calib.principal_point_y();
        camera_calibration.distortion = calib.distortion();
        camera_calibration.q0 = calib.q0();
        camera_calibration.q1 = calib.q1();
        camera_calibration.q2 = calib.q2();
        camera_calibration.q3 = calib.q3();
        camera_calibration.tx = calib.tx();
        camera_calibration.ty = calib.ty();
        camera_calibration.tz = calib.tz();
        
        // Optional fields
        if (calib.has_derived_camera_world_tx()) {
            camera_calibration.derived_camera_world_tx = calib.derived_camera_world_tx();
        }
        if (calib.has_derived_camera_world_ty()) {
            camera_calibration.derived_camera_world_ty = calib.derived_camera_world_ty();
        }
        if (calib.has_derived_camera_world_tz()) {
            camera_calibration.derived_camera_world_tz = calib.derived_camera_world_tz();
        }
        camera_calibrations.push_back(camera_calibration);
    }
}

void VisionReceiverNode::convertGeometryModels(const SSL_GeometryData& geometry, oxebots_interfaces::msg::SSLGeometryModels& geometry_models) {
    
    if (geometry.has_models()) {
        const auto &models = geometry.models();

        // Ball models
        if (models.has_straight_two_phase()) {
            oxebots_interfaces::msg::SSLBallModelStraightTwoPhase straight_two_phase;
            straight_two_phase.acc_slide = models.straight_two_phase().acc_slide();
            straight_two_phase.acc_roll = models.straight_two_phase().acc_roll();
            straight_two_phase.k_switch = models.straight_two_phase().k_switch();
            geometry_models.straight_two_phase = straight_two_phase;
        }

        // Chip fixed loss
        if (models.has_chip_fixed_loss()) {
            oxebots_interfaces::msg::SSLBallModelChipFixedLoss chip_fixed_loss;
            chip_fixed_loss.damping_xy_first_hop = models.chip_fixed_loss().damping_xy_first_hop();
            chip_fixed_loss.damping_xy_other_hops = models.chip_fixed_loss().damping_xy_other_hops();
            chip_fixed_loss.damping_z = models.chip_fixed_loss().damping_z();
            geometry_models.chip_fixed_loss = chip_fixed_loss;
        }
    }
}

void VisionReceiverNode::publishGeometryData(
    oxebots_interfaces::msg::SSLFieldSize field_size,
    std::vector<oxebots_interfaces::msg::SSLCameraCalibration> camera_calibrations,
    oxebots_interfaces::msg::SSLGeometryModels geometry_models) {
    
    oxebots_interfaces::msg::SSLGeometryData geometry_data;
    geometry_data.field = field_size;
    geometry_data.models = geometry_models;

    for (const auto &calib : camera_calibrations) {
        geometry_data.calib.push_back(calib);
    }
    geometry_publisher->publish(geometry_data);
}

void VisionReceiverNode::PublishRobotData(
  std::vector<oxebots_interfaces::msg::RobotGameData> allies,
  std::vector<oxebots_interfaces::msg::RobotGameData> enemies)
{
    oxebots_interfaces::msg::RobotPosition robot_data;
    robot_data.allies = allies;
    robot_data.enemies = enemies;

    robot_publisher->publish(robot_data);
}

void VisionReceiverNode::PublishBallData(
  oxebots_interfaces::msg::BallPosition ball_data)
{
    ball_publisher->publish(ball_data);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    boost::asio::io_context io_context;

    rclcpp::spin(std::make_shared<VisionReceiverNode>(io_context));
    rclcpp::shutdown();
    return 0;
}