#include "oxebots_comms/game_receiver_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameReceiverNode>());
    rclcpp::shutdown();
    return 0;
}

GameReceiverNode::GameReceiverNode()
: rclcpp::Node("oxebots_comms"), socket(io_service)
{
    RCLCPP_INFO(get_logger(), "Starting game receiver module...");

    declare_parameter("host", "127.0.0.1");
    declare_parameter("port", 8002);
    declare_parameter("robot_topic", "robot_data");
    declare_parameter("ball_topic", "ball_data");
    declare_parameter("topic_retention", 10);
    declare_parameter("robot_amount", 3);
    declare_parameter("timer_interval", 16);
    declare_parameter("is_yellow_team", false);

    RCLCPP_DEBUG(get_logger(), "Creating RobotDataPublisher");

    robot_publisher = create_publisher<oxebots_interfaces::msg::RobotPosition>(
      get_parameter("robot_topic").as_string(),
      get_parameter("topic_retention").as_int());

    RCLCPP_DEBUG(get_logger(), "Creating BallDataPublisher");

    ball_publisher = create_publisher<oxebots_interfaces::msg::BallPosition>(
      get_parameter("ball_topic").as_string(),
      get_parameter("topic_retention").as_int());

    is_yellow_team = get_parameter("is_yellow_team").as_bool();

    RCLCPP_DEBUG(get_logger(), "Creating UDP socket");
    boost::asio::ip::udp::endpoint remote_endpoint(
      boost::asio::ip::address::from_string(get_parameter("host").as_string()),
      get_parameter("port").as_int());

    socket.open(remote_endpoint.protocol());
    socket.bind(remote_endpoint);

    RCLCPP_DEBUG(get_logger(), "Starting UDP thread");
    udp_thread = std::thread(&GameReceiverNode::GetDetectionData, this);
    udp_thread.detach();

    RCLCPP_INFO(get_logger(), "Game receiver module started");
}

GameReceiverNode::~GameReceiverNode()
{
    RCLCPP_INFO(get_logger(), "Stopping game receiver module...");
    io_service.stop();
    socket.close();
    if (udp_thread.joinable()) udp_thread.join();

    RCLCPP_INFO(get_logger(), "Game receiver module stopped");
}

void GameReceiverNode::GetDetectionData()
{
    try
    {
        while (rclcpp::ok())
        {
            char data[1024];
            boost::asio::ip::udp::endpoint sender_endpoint;
            size_t len = socket.receive_from(boost::asio::buffer(data, 1024),
                                             sender_endpoint);

            SSL_WrapperPacket packet;
            packet.ParseFromArray(data, len);
            SSL_DetectionFrame detection = packet.detection();
            std::vector<oxebots_interfaces::msg::RobotGameData> yellow_robots;
            std::vector<oxebots_interfaces::msg::RobotGameData> blue_robots;
            for (auto robot : detection.robots_yellow())
            {
                oxebots_interfaces::msg::RobotGameData robot_data;
                robot_data.id = robot.robot_id();
                robot_data.x = robot.x();
                robot_data.y = robot.y();
                robot_data.orientation = robot.orientation();
                yellow_robots.push_back(robot_data);
            }

            for (auto robot : detection.robots_blue())
            {
                oxebots_interfaces::msg::RobotGameData robot_data;
                robot_data.id = robot.robot_id();
                robot_data.x = robot.x();
                robot_data.y = robot.y();
                robot_data.orientation = robot.orientation();
                blue_robots.push_back(robot_data);
            }

            oxebots_interfaces::msg::BallPosition ball_data;
            ball_data.x = detection.balls(0).x();
            ball_data.y = detection.balls(0).y();
            ball_data.z = detection.balls(0).z();

            if (is_yellow_team)
                return PublishData(yellow_robots, blue_robots, ball_data);

            PublishData(blue_robots, yellow_robots, ball_data);
        }
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_logger(), "Error in GetDetectionData: %s", e.what());
    }
}

void GameReceiverNode::PublishData(
  std::vector<oxebots_interfaces::msg::RobotGameData> allies,
  std::vector<oxebots_interfaces::msg::RobotGameData> enemies,
  oxebots_interfaces::msg::BallPosition ball_data)
{
    oxebots_interfaces::msg::RobotPosition robot_data;
    robot_data.allies = allies;
    robot_data.enemies = enemies;

    robot_publisher->publish(robot_data);
    ball_publisher->publish(ball_data);
}
