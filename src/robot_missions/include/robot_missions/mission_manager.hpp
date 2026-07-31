#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "robot_missions/mission_base.hpp"
#include "robot_missions/mission_registry.hpp"
#include "robot_navigation/navigation_provider.hpp"
#include "robot_navigation/field_path_planner.hpp"
#include "robot_gantry/action/gantry_move.hpp"

namespace robot_missions
{

// ── MissionManager ────────────────────────────────────────────────────────────
// Generic orchestrator. Knows nothing about specific mission types.
// Adding a new mission type does not change this class.
//
// Responsibilities:
//   - Receive MissionCommand
//   - Instantiate correct MissionBase subclass via MissionRegistry
//   - Plan a strip-safe path via FieldPathPlanner and navigate to start
//   - Call validate → plan → execute → report
//   - Handle pause / resume / abort
//   - Publish planned path on /mission_planned_path (nav_msgs/Path)

class MissionManager
{
public:
    MissionManager(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<robot_navigation::NavigationProvider> nav);
    ~MissionManager() = default;

    // Main entry point — receives high level command.
    MissionResult run(const MissionCommand & cmd);

    // Operator controls — can be called during execute()
    void pause();
    void resume();
    void abort();

    // Current progress — for action server feedback
    float       get_progress();
    std::string get_current_step();

    // Current robot pose — for action server feedback
    robot_navigation::Pose2D get_robot_pose();

private:
    using GantryMove = robot_gantry::action::GantryMove;

    void load_field_config();
    void load_headland_clearance();
    void publish_planned_path(const std::vector<robot_navigation::Pose2D> & waypoints);
    void retract_gantry_for_travel();

    rclcpp::Node::SharedPtr    node_;
    std::shared_ptr<robot_navigation::NavigationProvider> nav_;
    std::unique_ptr<MissionBase> current_mission_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
    rclcpp_action::Client<GantryMove>::SharedPtr gantry_client_;

    robot_navigation::FieldConfig field_config_;
    float headland_clearance_  = 1.5f;
    bool  field_config_loaded_ = false;
};

} // namespace robot_missions
