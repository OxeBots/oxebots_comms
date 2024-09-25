#include <rclcpp/rclcpp.hpp>
#include "oxebots_comms/game_receiver.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  boost::asio::io_context io_context;
  auto game_receiver = std::make_shared<GameReceiver>(io_context);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(game_receiver);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
