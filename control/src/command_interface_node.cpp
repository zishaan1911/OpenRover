#include "command_interface_node.hpp"

namespace openrover {

CommandInterfaceNode::CommandInterfaceNode()
    : rclcpp::Node("command_interface_node")
{
    velocityPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    commandSubscriber_ = this->create_subscription<std_msgs::msg::String>(
        "rover_command",
        10,
        std::bind(&CommandInterfaceNode::handleCommandMessage, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "CommandInterfaceNode ready. Listening on 'rover_command', publishing to 'cmd_vel'.");
}

void CommandInterfaceNode::setVelocity(double linearVelocityMetersPerSecond, double angularVelocityRadiansPerSecond)
{
    geometry_msgs::msg::Twist velocityMessage;
    velocityMessage.linear.x = linearVelocityMetersPerSecond;
    velocityMessage.angular.z = angularVelocityRadiansPerSecond;
    velocityPublisher_->publish(velocityMessage);

    RCLCPP_INFO(
        this->get_logger(),
        "Set velocity: linear=%.2f m/s, angular=%.2f rad/s",
        linearVelocityMetersPerSecond,
        angularVelocityRadiansPerSecond);
        
}

void CommandInterfaceNode::handleCommandMessage(const std_msgs::msg::String::SharedPtr commandMessage)
{
    const std::string& command = commandMessage->data;

    if (command == "FORWARD") {
        setVelocity(forwardSpeedMetersPerSecond_, 0.0);
    } else if (command == "BACKWARD") {
        setVelocity(-forwardSpeedMetersPerSecond_, 0.0);
    } else if (command == "ROTATE_LEFT") {
        setVelocity(0.0, rotationSpeedRadiansPerSecond_);
    } else if (command == "ROTATE_RIGHT") {
        setVelocity(0.0, -rotationSpeedRadiansPerSecond_);
    } else if (command == "STOP") {
        setVelocity(0.0, 0.0);
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown rover command: %s", command.c_str());
    }
}

} // namespace openrover
