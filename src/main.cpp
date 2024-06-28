#include "oxebots_comms/comms_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommsNode>());
    rclcpp::shutdown();
    return 0;
}
