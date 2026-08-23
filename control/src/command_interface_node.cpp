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

    setPoseClient_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>(
        "/world/openrover_world/set_pose");

    resetServiceServer_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_rover",
        std::bind(&CommandInterfaceNode::handleResetRequest, this, std::placeholders::_1, std::placeholders::_2));

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

void CommandInterfaceNode::handleResetRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto setPoseRequest = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    setPoseRequest->entity.name = "rover";
    setPoseRequest->pose.position.x = 0.0;
    setPoseRequest->pose.position.y = 0.0;
    setPoseRequest->pose.position.z = 0.1;  // matches the rover's resting height in model.sdf
    setPoseRequest->pose.orientation.w = 1.0;  // identity rotation — x/y/z default to 0.0

    setPoseClient_->async_send_request(
        setPoseRequest,
        [this](rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture resultFuture)
        {
            RCLCPP_INFO(this->get_logger(), "Reset pose request completed.");
        });

    this->setVelocity(0.0, 0.0);  // stop the rover immediately
    response->success = true;
    response->message = "Reset requested.";
}
} // namespace openrover
