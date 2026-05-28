#include "robot_missions/mission_manager.hpp"
#include "robot_missions/mission_registry.hpp"

namespace robot_missions
{

MissionManager::MissionManager(rclcpp::Node::SharedPtr node)
: node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "MissionManager ready.");
}

MissionResult MissionManager::run(const MissionCommand & cmd)
{
    RCLCPP_INFO(node_->get_logger(),
        "Received command: type=%s target=%s quantity=%.2f %s start=(%.2f,%.2f)",
        cmd.mission_type.c_str(), cmd.target.c_str(),
        cmd.quantity, cmd.quantity_unit.c_str(),
        cmd.start_x, cmd.start_y);

    if (!navigate_to_start(cmd.start_x, cmd.start_y)) {
        MissionResult fail;
        fail.success = false;
        fail.report  = "Failed to navigate to start position.";
        return fail;
    }

    current_mission_ = MissionRegistry::instance().create(cmd.mission_type, node_, cmd);
    if (!current_mission_) {
        MissionResult fail;
        fail.success = false;
        fail.report  = "Unknown mission type: " + cmd.mission_type;
        return fail;
    }

    if (!current_mission_->validate()) {
        MissionResult fail;
        fail.success = false;
        fail.report  = "Mission validation failed.";
        return fail;
    }

    if (!current_mission_->plan()) {
        MissionResult fail;
        fail.success = false;
        fail.report  = "Mission planning failed.";
        return fail;
    }

    current_mission_->execute();

    return current_mission_->report();
}

void MissionManager::pause()
{
    if (current_mission_) current_mission_->pause();
}

void MissionManager::resume()
{
    if (current_mission_) current_mission_->resume();
}

void MissionManager::abort()
{
    if (current_mission_) current_mission_->abort();
}

float MissionManager::get_progress()
{
    if (current_mission_) return current_mission_->get_progress();
    return 0.0f;
}

std::string MissionManager::get_current_step()
{
    if (current_mission_) return current_mission_->get_current_step();
    return "idle";
}

bool MissionManager::navigate_to_start(float world_x, float world_y)
{
    // TODO: integrate with Nav2 or odometry-based navigation
    RCLCPP_INFO(node_->get_logger(),
        "Navigating to start (%.2f, %.2f) — stub, robot assumed at position.",
        world_x, world_y);
    return true;
}

} // namespace robot_missions
