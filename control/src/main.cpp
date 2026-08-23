#include <rclcpp/rclcpp.hpp>

#include "command_interface_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<openrover::CommandInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
