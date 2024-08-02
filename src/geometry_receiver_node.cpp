#include "oxebots_comms/geometry_receiver_node.hpp"

GeometryReceiverNode::GeometryReceiverNode()
: rclcpp::Node("oxebots_geometry"), socket(io_service) {

    RCLCPP_INFO(get_logger(), "Starting geometry receiver node...");

    declare_parameter("host", "224.5.23.2");
    declare_parameter("port", 10006);
    declare_parameter("topic_retention", 10);
    declare_parameter("topic", "geometry");

    RCLCPP_DEBUG(get_logger(), "Creating GeometryPublisher");
    geometry_publisher = this->create_publisher<oxebots_interfaces::msg::SSLGeometryData>(
        get_parameter("topic").as_string(),  
        get_parameter("topic_retention").as_int());
    
    RCLCPP_DEBUG(get_logger(), "Adding host...");

    RCLCPP_DEBUG(get_logger(), "Creating UDP socket");
    boost::asio::ip::udp::endpoint remote_endpoint(
      boost::asio::ip::address::from_string(get_parameter("host").as_string()),
      get_parameter("port").as_int());

    socket.open(remote_endpoint.protocol());
    socket.bind(remote_endpoint);

    RCLCPP_DEBUG(get_logger(), "Starting UDP thread");
    udp_thread = std::thread(&GeometryReceiverNode::GetGeometryData, this);
    udp_thread.detach();

    RCLCPP_INFO(get_logger(), "Geometry receiver node started");
}

GeometryReceiverNode::~GeometryReceiverNode()
{
    RCLCPP_INFO(get_logger(), "Stopping geometry receiver module...");
    io_service.stop();
    socket.close();
    if (udp_thread.joinable()) udp_thread.join();

    RCLCPP_INFO(get_logger(), "geometry receiver module stopped");
}

void GeometryReceiverNode::GetGeometryData() {
    
    try {
        while (rclcpp::ok()) {
            char data[1024];
            boost::asio::ip::udp::endpoint sender_endpoint;
            size_t len = socket.receive_from(boost::asio::buffer(data, 1024),
                                             sender_endpoint);

            SSL_WrapperPacket packet;
            packet.ParseFromArray(data, len);

            if (packet.has_geometry()) {
                RCLCPP_DEBUG(get_logger(), "Processing packet with geometry data");
                SSL_GeometryData geometry = packet.geometry();

                // Convert field size
                oxebots_interfaces::msg::SSLFieldSize field_size;
                convertFieldSize(geometry, field_size);

                // Convert camera calibrations
                std::vector<oxebots_interfaces::msg::SSLCameraCalibration> camera_calibrations;
                convertCameraCalibrations(geometry, camera_calibrations);
                
                // Convert Models
                oxebots_interfaces::msg::SSLGeometryModels geometry_models;
                convertGeometryModels(geometry, geometry_models);

                // Log the size of the field
                RCLCPP_DEBUG(get_logger(), "Field size: length=%d, width=%d, goal_width=%d, goal_depth=%d, boundary_width=%d",
                            field_size.field_length, field_size.field_width, field_size.goal_width,
                            field_size.goal_depth, field_size.boundary_width);
                
                // Log the camera calibrations
                for (const auto &calib : camera_calibrations) {
                    RCLCPP_DEBUG(get_logger(), "Camera ID %d: focal_length=%f, principal_point=(%f, %f), distortion=%f",
                                calib.camera_id, calib.focal_length, calib.principal_point_x,
                                calib.principal_point_y, calib.distortion);
                }

                RCLCPP_DEBUG(get_logger(), "Publishing geometry data");
                publishGeometryData(field_size, camera_calibrations, geometry_models);
            }
        }
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_logger(), "Error in GetDetectionData: %s", e.what());
    }
}

void GeometryReceiverNode::convertFieldSize(const SSL_GeometryData& geometry, oxebots_interfaces::msg::SSLFieldSize& field_size) {

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

void GeometryReceiverNode::convertCameraCalibrations(const SSL_GeometryData& geometry, std::vector<oxebots_interfaces::msg::SSLCameraCalibration>& camera_calibrations) {
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

void GeometryReceiverNode::convertGeometryModels(const SSL_GeometryData& geometry, oxebots_interfaces::msg::SSLGeometryModels& geometry_models) {
    
    if (geometry.has_models()) {
        const auto &models = geometry.models();
        if (models.has_straight_two_phase()) {
            oxebots_interfaces::msg::SSLBallModelStraightTwoPhase straight_two_phase;
            straight_two_phase.acc_slide = models.straight_two_phase().acc_slide();
            straight_two_phase.acc_roll = models.straight_two_phase().acc_roll();
            straight_two_phase.k_switch = models.straight_two_phase().k_switch();
            geometry_models.straight_two_phase = straight_two_phase;
        }
        if (models.has_chip_fixed_loss()) {
            oxebots_interfaces::msg::SSLBallModelChipFixedLoss chip_fixed_loss;
            chip_fixed_loss.damping_xy_first_hop = models.chip_fixed_loss().damping_xy_first_hop();
            chip_fixed_loss.damping_xy_other_hops = models.chip_fixed_loss().damping_xy_other_hops();
            chip_fixed_loss.damping_z = models.chip_fixed_loss().damping_z();
            geometry_models.chip_fixed_loss = chip_fixed_loss;
        }
    }
}

void GeometryReceiverNode::publishGeometryData(
    oxebots_interfaces::msg::SSLFieldSize field_size,
    std::vector<oxebots_interfaces::msg::SSLCameraCalibration> camera_calibrations,
    oxebots_interfaces::msg::SSLGeometryModels geometry_models) {
    RCLCPP_INFO(get_logger(), "Publishing geometry data");
    
    oxebots_interfaces::msg::SSLGeometryData geometry_data;
    geometry_data.field = field_size;
    geometry_data.models = geometry_models;

    for (const auto &calib : camera_calibrations) {
        geometry_data.calib.push_back(calib);
    }
    geometry_publisher->publish(geometry_data);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeometryReceiverNode>());
    rclcpp::shutdown();
    return 0;
}
