#pragma once

#include <memory>
#include "robot_missions/planting/tool_interface.hpp"
#include "robot_missions/planting/seed_database.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace robot_missions
{

// ── GripperTool ───────────────────────────────────────────────────────────────
// Concrete ToolBase using the MG996R servo gripper.
// Communicates via /gripper/angle topic (std_msgs/Int32).
//
// Physical convention:
//   angle 0  = fully open
//   angle 55 = fully closed
//
// Tray position loaded from ROS2 node parameters at startup:
//   gripper_tray.gx / .gy / .z_approach / .z_pick
// Set these from config/tray_positions.yaml once physically measured.

class GripperTool : public ToolBase
{
public:
    static constexpr int ANGLE_OPEN   = 0;
    static constexpr int ANGLE_CLOSED = 55;
    static constexpr int SETTLE_TIME_MS = 600;

    explicit GripperTool(rclcpp::Node::SharedPtr node);
    ~GripperTool() override = default;

    void        configure(const SeedProfile & profile) override;
    void        pick()            override;
    void        release()         override;
    bool        is_ready()  const override;
    TrayPose    get_tray_pose()   const override { return tray_pose_; }
    const char* name()      const override { return "GripperTool"; }

private:
    void send_angle(int angle);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr angle_pub_;
    bool     ready_      = true;
    int      grip_angle_ = ANGLE_CLOSED;
    TrayPose tray_pose_;
};

} // namespace robot_missions
