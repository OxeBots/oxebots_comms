#include "oxebots_comms/gc_receiver_node.hpp"

GCReceiverNode::GCReceiverNode(boost::asio::io_context & io_context)
: rclcpp::Node("oxebots_comms"),
  UdpDriver<Referee>(io_context),
  io_context(io_context)
{
    declare_parameter("host", "224.5.23.1");
    declare_parameter("port", 10003);
    declare_parameter("topic_retention", 10);

    add_host(get_parameter("host").as_string(),
             get_parameter("port").as_int());

    io_thread = std::thread([this]() { this->io_context.run(); });

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
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    boost::asio::io_context io_context;

    rclcpp::spin(std::make_shared<GCReceiverNode>(io_context));
    rclcpp::shutdown();
    return 0;
}
