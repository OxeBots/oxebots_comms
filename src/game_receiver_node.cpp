#include "oxebots_comms/game_receiver_node.hpp"

GameReceiverNode::GameReceiverNode(boost::asio::io_context & io_context)
: rclcpp::Node("oxebots_comms"),
  UdpDriver<SSL_WrapperPacket>(io_context, "224.5.23.2", 10020)
{
    RCLCPP_INFO(get_logger(), "Starting game receiver module...");

    declare_parameter("robot_topic", "robot_data");
    declare_parameter("ball_topic", "ball_data");
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

    is_yellow_team = get_parameter("is_yellow_team").as_bool();

    RCLCPP_INFO(get_logger(), "Game receiver module started");

    io_context.run();
}

GameReceiverNode::~GameReceiverNode()
{
    RCLCPP_INFO(get_logger(), "Stopping game receiver module...");
}

void GameReceiverNode::on_receive(const SSL_WrapperPacket & packet)
{
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
        PublishData(yellow_robots, blue_robots, ball_data);
    else
        PublishData(blue_robots, yellow_robots, ball_data);
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    boost::asio::io_context io_context;

    rclcpp::spin(std::make_shared<GameReceiverNode>(io_context));
    rclcpp::shutdown();
    return 0;
}
