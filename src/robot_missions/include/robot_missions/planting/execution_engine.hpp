#pragma once

#include <atomic>
#include <memory>
#include <cstdint>
#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_missions/planting/planting_planner.hpp"
#include "robot_missions/planting/tool_interface.hpp"
#include "robot_missions/planting/depth_sensor.hpp"

namespace robot_missions
{

// ── ExecutionEngine ───────────────────────────────────────────────────────────
// Orchestrates hardware to execute the planting plan.
// Knows about: ROS2, JTC, diff_drive, tool.
// Knows nothing about: seed spacing, tool type, global coordinates.
//
// Receives coordinates from PlantingPlanner on demand.
// Calls ToolBase::pick() and ToolBase::release() without knowing tool type.
// Gets tray position from tool_->get_tray_pose() — tool owns its own config.
//
// Sequence per robot stop:
//   move_base(advance_meters)
//   wait_until_stopped()
//   for each seed in stop:
//       move_gantry_to_tray()   [uses tool_->get_tray_pose()]
//       tool.pick()
//       move_gantry_to(seed.gx, seed.gy)
//       lower_z(seed.depth)
//       tool.release()
//       retract_z()

class ExecutionEngine
{
public:
    // paused:  pointer to PlantingMission::paused_ — engine polls it between seeds
    // depth:   optional soil distance sensor; nullptr = use fixed depth from planner
    //          (TODO: wire up real sensor once hardware is available)
    ExecutionEngine(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<ToolBase> tool,
        const std::atomic<bool> * paused  = nullptr,
        DepthSensor *              depth   = nullptr);

    // Execute full planting plan. Returns number of seeds successfully planted.
    uint32_t execute(const PlantingPlanner & planner);

    void stop();

private:
    void move_base_forward(float meters);
    void wait_until_base_stopped();
    void move_gantry_to(float gx, float gy);
    void lower_z(float depth_m);
    void retract_z();
    void move_gantry_to_tray();
    void wait_ms(int ms);
    bool wait_gantry_reached(float gx, float gy, float gz,
                             float tol = 0.001f, int timeout_ms = 10000);
    bool wait_base_advanced(float start_x, float meters,
                            float tol = 0.02f, int timeout_ms = 30000);

    rclcpp::Node::SharedPtr      node_;
    std::shared_ptr<ToolBase>    tool_;
    TrayPose                     tray_;
    bool                         stop_requested_ = false;
    const std::atomic<bool> *    paused_         = nullptr;
    DepthSensor *                depth_sensor_   = nullptr;

    float current_gx_ = 0.0f;
    float current_gy_ = 0.0f;

    std::unordered_map<std::string, double> joint_pos_;
    double odom_x_ = 0.0;
    double odom_y_ = 0.0;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gantry_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr      base_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       joint_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;
};

} // namespace robot_missions
