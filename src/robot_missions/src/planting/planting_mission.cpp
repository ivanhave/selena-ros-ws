#include "robot_missions/planting/planting_mission.hpp"
#include "robot_missions/planting/gripper_tool.hpp"
#include "robot_missions/mission_registry.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace robot_missions
{

PlantingMission::PlantingMission(
    rclcpp::Node::SharedPtr node,
    const std::string & seed_db_path)
: node_(node)
, seed_db_path_(seed_db_path)
{
    RCLCPP_INFO(node_->get_logger(), "PlantingMission created.");
}

std::unique_ptr<PlantingMission> PlantingMission::create_from_command(
    rclcpp::Node::SharedPtr node,
    const MissionCommand & cmd)
{
    std::string db_path =
        ament_index_cpp::get_package_share_directory("robot_missions")
        + "/config/seeds.csv";

    auto mission = std::make_unique<PlantingMission>(node, db_path);
    mission->set_seed_type(cmd.target);

    if (cmd.quantity_unit == "meters") {
        mission->set_strip_length(cmd.quantity);
    } else if (cmd.quantity_unit == "seeds") {
        mission->set_seed_count(static_cast<uint32_t>(cmd.quantity));
    }

    return mission;
}

void PlantingMission::set_seed_type(const std::string & seed_type)
{
    seed_type_ = seed_type;
}

void PlantingMission::set_strip_length(float length_m)
{
    strip_length_m_ = length_m;
    seed_count_     = 0;
}

void PlantingMission::set_seed_count(uint32_t count)
{
    seed_count_     = count;
    strip_length_m_ = 0.0f;
}

bool PlantingMission::validate()
{
    current_step_ = "validating";

    if (seed_type_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "No seed type specified.");
        return false;
    }
    if (strip_length_m_ <= 0.0f && seed_count_ == 0) {
        RCLCPP_ERROR(node_->get_logger(), "No quantity specified.");
        return false;
    }
    if (!seed_db_.load(seed_db_path_)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load seed database.");
        return false;
    }
    if (!seed_db_.get_profile(seed_type_, profile_)) {
        RCLCPP_ERROR(node_->get_logger(), "Seed type not found: %s", seed_type_.c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Validation passed for seed: %s", seed_type_.c_str());
    return true;
}

bool PlantingMission::plan()
{
    current_step_ = "planning";

    if (!planner_.init(profile_, strip_length_m_, seed_count_)) {
        RCLCPP_ERROR(node_->get_logger(), "Planner init failed.");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Plan ready: seeds=%u stops=%u",
        planner_.get_total_seeds(), planner_.get_total_stops());
    return true;
}

bool PlantingMission::execute()
{
    current_step_ = "executing";

    // Tool factory — select implementation based on seed profile's tool_id.
    // tool_id 1 = GripperTool (current hardware)
    // tool_id 2 = VacuumTool  (future — for small seeds like tomato, morcov)
    if (profile_.tool_id == 1) {
        tool_ = std::make_shared<GripperTool>(node_);
    } else {
        RCLCPP_ERROR(node_->get_logger(),
            "Unsupported tool_id=%u for seed '%s'. Only gripper (tool_id=1) is implemented.",
            profile_.tool_id, profile_.name.c_str());
        return false;
    }

    tool_->configure(profile_);

    // TODO: replace nullptr with a real DepthSensor once hardware is mounted
    engine_ = std::make_unique<ExecutionEngine>(node_, tool_, &paused_, nullptr);
    seeds_planted_ = engine_->execute(planner_);

    current_step_ = aborted_.load() ? "aborted" : "complete";
    RCLCPP_INFO(node_->get_logger(), "Execution complete. Seeds planted: %u", seeds_planted_);

    return seeds_planted_ == planner_.get_total_seeds();
}

void PlantingMission::pause()
{
    paused_ = true;
    current_step_ = "paused";
    RCLCPP_INFO(node_->get_logger(), "Paused.");
}

void PlantingMission::resume()
{
    paused_ = false;
    current_step_ = "executing";
    RCLCPP_INFO(node_->get_logger(), "Resumed.");
}

void PlantingMission::abort()
{
    aborted_ = true;
    current_step_ = "aborting";
    if (engine_) engine_->stop();
    RCLCPP_WARN(node_->get_logger(), "Aborted.");
}

MissionResult PlantingMission::report()
{
    MissionResult result;
    result.success           = !aborted_.load() && seeds_planted_ == planner_.get_total_seeds();
    result.items_completed   = seeds_planted_;
    result.distance_covered_m =
        planner_.get_total_stops() * planner_.get_robot_advance();
    result.report =
        "Planted " + std::to_string(seeds_planted_) +
        " seeds of " + seed_type_ +
        " over " + std::to_string(result.distance_covered_m) + "m.";
    return result;
}

float PlantingMission::get_progress()
{
    if (planner_.get_total_seeds() == 0) return 0.0f;
    return static_cast<float>(seeds_planted_) /
           static_cast<float>(planner_.get_total_seeds());
}

std::string PlantingMission::get_current_step()
{
    return current_step_;
}

} // namespace robot_missions

// Self-register with MissionRegistry.
// This static initializer runs at startup — MissionManager never needs to import
// PlantingMission directly.
namespace robot_missions {
REGISTER_MISSION("plant", PlantingMission);
} // namespace robot_missions
