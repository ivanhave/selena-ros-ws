#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_gripper/mg996r_driver.hpp"

namespace robot_gripper
{

class GripperInterface : public rclcpp::Node
{
public:

    GripperInterface();
    ~GripperInterface();

private:

    // ── Callbacks ─────────────────────────────────────
    void angle_callback(const std_msgs::msg::Int32::SharedPtr msg);

    // ── Driver ────────────────────────────────────────
    MG996RDriver driver_;

    // ── ROS2 interfaces ───────────────────────────────

    // Subscribe: receive angle commands
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr angle_sub_;

    // Publish: current confirmed angle
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr angle_pub_;

    // Publish: true when gripper is open, false when closed
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
};

} // namespace robot_gripper