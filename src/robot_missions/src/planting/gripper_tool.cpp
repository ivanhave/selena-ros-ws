#include "robot_missions/planting/gripper_tool.hpp"
#include <chrono>
#include <thread>

namespace robot_missions
{

GripperTool::GripperTool(rclcpp::Node::SharedPtr node)
: node_(node)
{
    angle_pub_ = node_->create_publisher<std_msgs::msg::Int32>(
        "/gripper/angle", 10);

    // Tray position — loaded from node params, set via tray_positions.yaml.
    // Use has_parameter guard so re-creating GripperTool on the same node is safe.
    if (!node_->has_parameter("gripper_tray.gx"))         node_->declare_parameter("gripper_tray.gx",         0.0);
    if (!node_->has_parameter("gripper_tray.gy"))         node_->declare_parameter("gripper_tray.gy",         0.0);
    if (!node_->has_parameter("gripper_tray.z_approach")) node_->declare_parameter("gripper_tray.z_approach", 0.0);
    if (!node_->has_parameter("gripper_tray.z_pick"))     node_->declare_parameter("gripper_tray.z_pick",     0.0);

    tray_pose_.gx         = static_cast<float>(node_->get_parameter("gripper_tray.gx").as_double());
    tray_pose_.gy         = static_cast<float>(node_->get_parameter("gripper_tray.gy").as_double());
    tray_pose_.z_approach = static_cast<float>(node_->get_parameter("gripper_tray.z_approach").as_double());
    tray_pose_.z_pick     = static_cast<float>(node_->get_parameter("gripper_tray.z_pick").as_double());

    RCLCPP_INFO(node_->get_logger(),
        "GripperTool ready. Tray pose: gx=%.3f gy=%.3f z_approach=%.3f z_pick=%.3f",
        tray_pose_.gx, tray_pose_.gy, tray_pose_.z_approach, tray_pose_.z_pick);
}

void GripperTool::configure(const SeedProfile & profile)
{
    grip_angle_ = static_cast<int>(profile.tool_param);
    RCLCPP_INFO(node_->get_logger(),
        "GripperTool configured for seed '%s': grip_angle=%d deg",
        profile.name.c_str(), grip_angle_);
}

void GripperTool::pick()
{
    RCLCPP_INFO(node_->get_logger(), "Picking — closing gripper to %d deg.", grip_angle_);
    send_angle(grip_angle_);
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
}

void GripperTool::release()
{
    RCLCPP_INFO(node_->get_logger(), "Releasing — opening gripper.");
    send_angle(ANGLE_OPEN);
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
}

bool GripperTool::is_ready() const
{
    return ready_;
}

void GripperTool::send_angle(int angle)
{
    std_msgs::msg::Int32 msg;
    msg.data = angle;
    angle_pub_->publish(msg);
}

} // namespace robot_missions
