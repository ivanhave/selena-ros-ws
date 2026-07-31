#pragma once

#include <atomic>
#include <memory>
#include <cstdint>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_gantry/action/gantry_move.hpp"
#include "robot_missions/planting/planting_planner.hpp"
#include "robot_missions/planting/tool_interface.hpp"
#include "robot_missions/planting/depth_sensor.hpp"
#include "robot_navigation/navigation_provider.hpp"

namespace robot_missions
{

// ── ExecutionEngine ───────────────────────────────────────────────────────────
// Orchestrates hardware to execute the planting plan.
// Gantry motion is delegated exclusively to the /gantry/move action server.
// This class knows nothing about JTC, velocity controllers, or timing constants.
//
// Sequence per robot stop:
//   move_base(advance_meters)
//   for each seed in stop:
//       move_gantry_to_tray()   [uses tool_->get_tray_pose()]
//       tool.pick()
//       move_gantry_to(seed.gx, seed.gy, seed.depth)
//       tool.release()
//       retract_z()

class ExecutionEngine
{
public:
    ExecutionEngine(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<ToolBase> tool,
        std::shared_ptr<robot_navigation::NavigationProvider> nav,
        const std::atomic<bool> * paused  = nullptr,
        DepthSensor *              depth   = nullptr);

    uint32_t execute(const PlantingPlanner & planner);

    void stop();

    uint32_t get_seeds_planted() const { return seeds_planted_.load(); }

private:
    using GantryMove = robot_gantry::action::GantryMove;

    // Send GantryMove goal; block until done or stop_requested_.
    // robot_gantry handles Z retract before XY, and all timing/safety internally.
    void send_gantry_move(double x, double y, double z);

    // Higher-level helpers:
    void move_gantry_to_tray();                  // move to tray XY, lower to z_pick
    void move_gantry_to(float gx, float gy);    // retract Z (via gantry_node), then XY
    void lower_z(float depth_m);
    void retract_z();
    void wait_ms(int ms);

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

    rclcpp_action::Client<GantryMove>::SharedPtr gantry_client_;
};

} // namespace robot_missions
