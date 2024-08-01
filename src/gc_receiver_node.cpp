#include "oxebots_comms/gc_receiver_node.hpp"

GCReceiverNode::GCReceiverNode(boost::asio::io_context & io_context)
: rclcpp::Node("oxebots_comms"),
  UdpDriver<Referee>(io_context),
  io_context(io_context)
{
    declare_parameter("host", "224.5.23.1");
    declare_parameter("port", 10003);
    declare_parameter("topic_retention", 10);
    declare_parameter("is_yellow_team", false);
    declare_parameter("gc_topic", "gc_data");

    RCLCPP_DEBUG(get_logger(), "Creating host...");

    add_host(get_parameter("host").as_string(),
             get_parameter("port").as_int());

    RCLCPP_DEBUG(get_logger(), "Creating GC Publisher...");

    io_thread = std::thread([this]() { this->io_context.run(); });

    RCLCPP_DEBUG(get_logger(), "Creating gc publisher...");

    gc_publisher =
      create_publisher<oxebots_interfaces::msg::GameEventProposalGroup>(
        get_parameter("gc_topic").as_string(),
        get_parameter("topic_retention").as_int());

    RCLCPP_INFO(get_logger(), "Game controller receiver module started");
}

GCReceiverNode::~GCReceiverNode()
{
    RCLCPP_INFO(get_logger(), "Stopping game receiver module...");
    io_context.stop();
    if (io_thread.joinable()) io_thread.join();
    stop();
}

void GCReceiverNode::on_receive(const Referee & packet)
{
    RCLCPP_INFO(get_logger(), "%s", packet.DebugString().c_str());

    int timestamp = packet.packet_timestamp();

    std::string source_identifier = nullptr;
    if (packet.has_source_identifier())
        source_identifier = packet.source_identifier();

    MatchType match_type = packet.match_type();
    Referee::Stage stage = packet.stage();

    // TODO: see sint64

    Referee::Command command = packet.command();
    int command_counter = packet.command_counter();
    int command_timestamp = packet.command_timestamp();
    Referee::TeamInfo yellow_team_info = packet.yellow();
    Referee::TeamInfo blue_team_info = packet.blue();
    Referee::Point designated_position;
    if (packet.has_designated_position())
        designated_position = packet.designated_position();
    bool blue_team_on_positive;
    if (packet.has_blue_team_on_positive_half())
        blue_team_on_positive = packet.has_blue_team_on_positive_half();

    Referee::Command next_command = packet.next_command();
    auto game_events = packet.game_events();
    auto game_event_proposal = packet.game_event_proposals();
    int current_action_time_remaining;
    if (packet.has_current_action_time_remaining())
        current_action_time_remaining = packet.current_action_time_remaining();
}

void GCReceiverNode::PublishGCData(
  oxebots_interfaces::msg::GameEventProposalGroup gc_event_group)
{
    gc_publisher->publish(gc_event_group);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    boost::asio::io_context io_context;

    rclcpp::spin(std::make_shared<GCReceiverNode>(io_context));
    rclcpp::shutdown();
    return 0;
}
