#include "oxebots_comms/game_receiver_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameReceiverNode>());
    rclcpp::shutdown();
    return 0;
}

GameReceiverNode::GameReceiverNode() : rclcpp::Node("oxebots_comms")
{
    RCLCPP_INFO(get_logger(), "Starting game receiver module...");

    declare_parameter("host", "127.0.0.1");
    declare_parameter("port", "8002");
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

    game_receiver = new DataReceiver<SSL_WrapperPacket>(
      get_parameter("host").as_string(), get_parameter("port").as_string());

    RCLCPP_DEBUG(get_logger(), "Creating timer");
    create_timer(
      std::chrono::milliseconds(get_parameter("timer_interval").as_int()),
      std::bind(&GameReceiverNode::GetDetectionData, this));

    is_yellow_team = get_parameter("is_yellow_team").as_bool();

    RCLCPP_INFO(get_logger(), "Game receiver module started");
}

void GameReceiverNode::GetDetectionData()
{
    SSL_WrapperPacket data = game_receiver->receive_data();
    SSL_DetectionFrame detection = data.detection();
    std::vector<oxebots_interfaces::msg::RobotGameData> robot_yellow;
    std::vector<oxebots_interfaces::msg::RobotGameData> robot_blue;
    for (auto robot : detection.robots_yellow())
    {
        oxebots_interfaces::msg::RobotGameData robot_data;
        robot_data.id = robot.robot_id();
        robot_data.x = robot.x();
        robot_data.y = robot.y();
        robot_data.orientation = robot.orientation();
        robot_yellow.push_back(robot_data);
    }

    for (auto robot : detection.robots_blue())
    {
        oxebots_interfaces::msg::RobotGameData robot_data;
        robot_data.id = robot.robot_id();
        robot_data.x = robot.x();
        robot_data.y = robot.y();
        robot_data.orientation = robot.orientation();
        robot_blue.push_back(robot_data);
    }

    oxebots_interfaces::msg::BallPosition ball_data;
    ball_data.x = detection.balls(0).x();
    ball_data.y = detection.balls(0).y();
    ball_data.z = detection.balls(0).z();

    if (is_yellow_team)
        return PublishData(robot_yellow, robot_blue, ball_data);

    return PublishData(robot_blue, robot_yellow, ball_data);
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
