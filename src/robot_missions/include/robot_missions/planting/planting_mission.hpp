#pragma once

#include <memory>
#include <string>
#include <atomic>
#include "robot_missions/mission_base.hpp"
#include "robot_missions/planting/seed_database.hpp"
#include "robot_missions/planting/planting_planner.hpp"
#include "robot_missions/planting/tool_interface.hpp"
#include "robot_missions/planting/execution_engine.hpp"
#include "robot_missions/planting/flat_ground_depth_sensor.hpp"
#include "robot_navigation/navigation_provider.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_missions
{

// ── PlantingMission ───────────────────────────────────────────────────────────
// Concrete MissionBase for planting/seeding tasks.
// Owns and coordinates all planting-specific pieces:
//   SeedDatabase → PlantingPlanner → ToolBase (gripper or vacuum) → ExecutionEngine
//
// Registered in the MissionRegistry via REGISTER_MISSION("plant", PlantingMission)
// at the bottom of planting_mission.cpp. MissionManager knows nothing about this class.

class PlantingMission : public MissionBase
{
public:
    PlantingMission(
        rclcpp::Node::SharedPtr node,
        const std::string & seed_db_path,
        std::shared_ptr<robot_navigation::NavigationProvider> nav);

    ~PlantingMission() override = default;

    // ── MissionBase interface ─────────────────────────────────────────────────
    bool validate() override;
    bool plan()     override;
    bool execute()  override;
    void pause()    override;
    void resume()   override;
    void abort()    override;
    MissionResult report() override;
    float       get_progress()     override;
    std::string get_current_step() override;
    std::vector<robot_navigation::Pose2D> get_route_waypoints(
        float start_x, float start_y, float start_yaw) const override;

    // ── Planting-specific setup ───────────────────────────────────────────────
    void set_seed_type(const std::string & seed_type);
    void set_strip_length(float length_m);
    void set_seed_count(uint32_t count);

    // ── Registry factory ──────────────────────────────────────────────────────
    // Called by REGISTER_MISSION macro to create a PlantingMission from a command.
    static std::unique_ptr<PlantingMission> create_from_command(
        rclcpp::Node::SharedPtr node,
        const MissionCommand & cmd,
        std::shared_ptr<robot_navigation::NavigationProvider> nav);

private:
    rclcpp::Node::SharedPtr node_;
    std::string             seed_db_path_;
    std::shared_ptr<robot_navigation::NavigationProvider> nav_;

    std::string seed_type_;
    float       strip_length_m_ = 0.0f;
    uint32_t    seed_count_     = 0;

    SeedDatabase                  seed_db_;
    SeedProfile                   profile_;
    PlantingPlanner               planner_;
    std::shared_ptr<ToolBase>           tool_;
    std::unique_ptr<FlatGroundDepthSensor> flat_sensor_;
    std::unique_ptr<ExecutionEngine>    engine_;

    uint32_t    seeds_planted_ = 0;
    float       progress_      = 0.0f;
    std::string current_step_  = "idle";
    std::atomic<bool> paused_ {false};
    std::atomic<bool> aborted_{false};
};

} // namespace robot_missions
