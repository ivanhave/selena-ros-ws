#pragma once

#include <atomic>
#include <memory>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_missions/planting/planting_planner.hpp"
#include "robot_missions/planting/tool_interface.hpp"
#include "robot_missions/planting/depth_sensor.hpp"
#include "robot_navigation/navigation_provider.hpp"

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
    // nav:     navigation provider — moves the base between planting stops
    // paused:  pointer to PlantingMission::paused_ — engine polls it between seeds
    // depth:   optional soil distance sensor; nullptr = use fixed depth from planner
    ExecutionEngine(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<ToolBase> tool,
        std::shared_ptr<robot_navigation::NavigationProvider> nav,
        const std::atomic<bool> * paused  = nullptr,
        DepthSensor *              depth   = nullptr);

    // Execute full planting plan. Returns number of seeds successfully planted.
    uint32_t execute(const PlantingPlanner & planner);

    void stop();

    uint32_t get_seeds_planted() const { return seeds_planted_.load(); }

private:
    // Hardware velocity caps — must match gantry_hardware_interface.hpp
    static constexpr float X_VEL_CAP = 0.15f;  // m/s (GT2 belt)
    static constexpr float Y_VEL_CAP = 0.06f;  // m/s (lead screw)
    static constexpr float Z_VEL_CAP = 0.04f;  // m/s (lead screw, vertical)
    static constexpr float MOVE_MARGIN_S = 0.8f;  // extra seconds after computed move time

    // ── Gantry motion primitives ──────────────────────────────────────────────
    // Rule: Z must be 0 before X/Y move. X/Y must be at target before Z move.
    //
    // send_xy_command: move X and Y simultaneously, Z held at measured position.
    //   All three joints in one message — no partial-trajectory race.
    //   Waits for the computed trajectory time then for XY to settle before returning.
    // send_z_command:  move Z. X and Y held at their measured positions.
    //   All three joints in one message — no partial-trajectory race.
    //   Waits for computed trajectory time before returning.
    void send_xy_command(float gx, float gy);
    void send_z_command(float gz);

    // Higher-level helpers built from the two primitives:
    void move_gantry_to(float gx, float gy);  // retracts Z first if needed, then XY
    void lower_z(float depth_m);              // send_z_command(depth_m)
    void retract_z();                          // send_z_command(0.0f)
    void move_gantry_to_tray();               // move_gantry_to + lower_z to z_pick
    void wait_ms(int ms);
    void interruptible_sleep(int ms);  // sleep up to ms, waking every 50ms to check stop

    // Timing helpers — 1.5x spline factor so peak velocity equals hardware cap.
    int xy_move_ms(float gx, float gy) const;
    int z_move_ms(float gz) const;

    // Poll joint_states until X and Y are within TOL_M of target for
    // STABLE_COUNT consecutive 50ms samples. Returns true on success, false on timeout.
    bool wait_xy_settled(float gx, float gy);

    rclcpp::Node::SharedPtr      node_;
    std::shared_ptr<ToolBase>    tool_;
    std::shared_ptr<robot_navigation::NavigationProvider> nav_;
    TrayPose                     tray_;
    std::atomic<bool>            stop_requested_ {false};
    const std::atomic<bool> *    paused_         = nullptr;
    DepthSensor *                depth_sensor_   = nullptr;
    std::atomic<uint32_t>        seeds_planted_  {0};

    float current_gx_ = 0.0f;
    float current_gy_ = 0.0f;
    float current_gz_ = 0.0f;

    mutable std::mutex                       state_mutex_;
    std::unordered_map<std::string, double> joint_pos_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gantry_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       joint_state_sub_;
};

} // namespace robot_missions
