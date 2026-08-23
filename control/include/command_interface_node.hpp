#pragma once

#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace openrover {

// CommandInterfaceNode is a ROS 2 node that turns simple, discrete
// command words (FORWARD, BACKWARD, ROTATE_LEFT, ROTATE_RIGHT, STOP)
// into velocity commands the rover can act on.
//
// It listens on the "rover_command" topic for std_msgs/String messages
// containing one of those five words, and publishes geometry_msgs/Twist
// messages on "cmd_vel" — the same topic our Gazebo bridge is already
// listening to (see simulation/config/ros_gz_bridge.yaml).
//
// This matches the README's Phase 1 plan: discrete keyboard-style
// commands for now, sitting on top of a velocity-based
// setVelocity(linear, angular) API that a future planner could call
// directly instead of going through named commands at all.
class CommandInterfaceNode : public rclcpp::Node
{
public:
    CommandInterfaceNode();
    void setVelocity(double linearVelocityMetersPerSecond, double angularVelocityRadiansPerSecond);

private:
    void handleCommandMessage(const std_msgs::msg::String::SharedPtr commandMessage);

    void handleResetRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPublisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandSubscriber_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetServiceServer_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr setPoseClient_;

    double forwardSpeedMetersPerSecond_ = 0.5;
    double rotationSpeedRadiansPerSecond_ = 0.5;
};

} // namespace openrover
