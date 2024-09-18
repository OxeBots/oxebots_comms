#include <rclcpp/rclcpp.hpp>
#include "oxebots_comms/game_receiver.hpp"
#include "oxebots_comms/gc_receiver.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto gc_receiver = std::make_shared<GCReceiver>();
  auto game_receiver = std::make_shared<GameReceiver>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(game_receiver);
  executor.add_node(gc_receiver);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
