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

    endpoint = remote_endpoint;

    socket.open(remote_endpoint.protocol());
    socket.bind(remote_endpoint);

    RCLCPP_INFO(get_logger(), "Socket bound to %s:%d",
                remote_endpoint.address().to_string().c_str(),
                remote_endpoint.port());

    RCLCPP_DEBUG(get_logger(), "Starting UDP thread");

    socket.async_receive_from(
      boost::asio::buffer(recv_buffer, 3096), remote_endpoint,
      boost::bind(&GameReceiverNode::GetDetectionData, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));

    io_service.run();

    RCLCPP_INFO(get_logger(), "Game receiver module started");
}

GameReceiverNode::~GameReceiverNode()
{
    RCLCPP_INFO(get_logger(), "Stopping game receiver module...");
    io_service.stop();
    socket.close();

    RCLCPP_INFO(get_logger(), "Game receiver module stopped");
}

void GameReceiverNode::StartReceive()
{
    RCLCPP_DEBUG(get_logger(), "Starting receive");
    socket.async_receive_from(
      boost::asio::buffer(recv_buffer, 3096), endpoint,
      boost::bind(&GameReceiverNode::GetDetectionData, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void GameReceiverNode::GetDetectionData(
  const boost::system::error_code & error, std::size_t bytes_transferred)
{
    RCLCPP_DEBUG(get_logger(), "Handling received data");
    if (!error || error == boost::asio::error::message_size)
    {
        RCLCPP_DEBUG(get_logger(), "Received data");

        SSL_WrapperPacket packet;
        packet.ParseFromArray(recv_buffer.data(), bytes_transferred);
        SSL_DetectionFrame detection = packet.detection();

        RCLCPP_DEBUG(get_logger(), "Received data from frame %d",
                     detection.frame_number());

        RCLCPP_DEBUG(get_logger(), detection.DebugString().c_str());

        std::vector<oxebots_interfaces::msg::RobotGameData> yellow_robots;
        std::vector<oxebots_interfaces::msg::RobotGameData> blue_robots;
        RCLCPP_DEBUG(get_logger(), "Parsing robots");
        RCLCPP_DEBUG(get_logger(), "Yellow robots: %d",
                     detection.robots_yellow_size());
        if (detection.robots_yellow_size() > 0)
        {
            RCLCPP_DEBUG(get_logger(), "yellow robots detected");
            for (auto robot : detection.robots_yellow())
            {
                oxebots_interfaces::msg::RobotGameData robot_data;
                RCLCPP_DEBUG(get_logger(), "Robot ID: %d", robot.robot_id());
                robot_data.id = robot.robot_id();
                robot_data.x = robot.x();
                robot_data.y = robot.y();
                robot_data.orientation = robot.orientation();
                yellow_robots.push_back(robot_data);
            }
        }

        RCLCPP_DEBUG(get_logger(), "Blue robots: %d",
                     detection.robots_blue_size());
        if (detection.robots_blue_size() > 0)
        {
            RCLCPP_DEBUG(get_logger(), "blue robots detected");
            for (auto robot : detection.robots_blue())
            {
                oxebots_interfaces::msg::RobotGameData robot_data;
                RCLCPP_DEBUG(get_logger(), "Robot ID: %d", robot.robot_id());
                robot_data.id = robot.robot_id();
                robot_data.x = robot.x();
                robot_data.y = robot.y();
                robot_data.orientation = robot.orientation();
                blue_robots.push_back(robot_data);
            }
        }

        RCLCPP_DEBUG(get_logger(), "Parsing ball");
        if (detection.balls_size() > 0)
        {
            oxebots_interfaces::msg::BallPosition ball_data;
            ball_data.x = detection.balls(0).x();
            ball_data.y = detection.balls(0).y();
            ball_data.z = detection.balls(0).z();
            RCLCPP_DEBUG(get_logger(), "Ball position: (%f, %f, %f)",
                         ball_data.x, ball_data.y, ball_data.z);
            PublishBallData(ball_data);
        }

        RCLCPP_DEBUG(get_logger(), "Publishing data");
        if (is_yellow_team)
            PublishRobotData(yellow_robots, blue_robots);
        else
            PublishRobotData(blue_robots, yellow_robots);

        StartReceive();
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Error in handle_receive: %s",
                     error.message().c_str());
    }
}

void GameReceiverNode::PublishRobotData(
  std::vector<oxebots_interfaces::msg::RobotGameData> allies,
  std::vector<oxebots_interfaces::msg::RobotGameData> enemies)
{
    oxebots_interfaces::msg::RobotPosition robot_data;
    robot_data.allies = allies;
    robot_data.enemies = enemies;

    robot_publisher->publish(robot_data);
}

void GameReceiverNode::PublishBallData(
  oxebots_interfaces::msg::BallPosition ball_data)
{
    ball_publisher->publish(ball_data);
}