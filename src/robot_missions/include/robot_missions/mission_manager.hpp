#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_missions/mission_base.hpp"
#include "robot_missions/mission_registry.hpp"

namespace robot_missions
{

// ── MissionManager ────────────────────────────────────────────────────────────
// Generic orchestrator. Knows nothing about specific mission types.
// Adding a new mission type does not change this class.
//
// Responsibilities:
//   - Receive MissionCommand
//   - Instantiate correct MissionBase subclass via MissionRegistry
//   - Navigate robot to start position (stub — future Nav2 integration)
//   - Call validate → plan → execute → report
//   - Handle pause / resume / abort

class MissionManager
{
public:
    explicit MissionManager(rclcpp::Node::SharedPtr node);
    ~MissionManager() = default;

    // Main entry point — receives high level command.
    // Returns final mission result.
    MissionResult run(const MissionCommand & cmd);

    // Operator controls — can be called during execute()
    void pause();
    void resume();
    void abort();

    // Current progress — for action server feedback
    float       get_progress();
    std::string get_current_step();

private:
    bool navigate_to_start(float world_x, float world_y);

    rclcpp::Node::SharedPtr    node_;
    std::unique_ptr<MissionBase> current_mission_;
};

} // namespace robot_missions
