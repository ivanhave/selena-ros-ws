#include "robot_missions/mission_registry.hpp"

namespace robot_missions
{

MissionRegistry & MissionRegistry::instance()
{
    static MissionRegistry registry;
    return registry;
}

void MissionRegistry::register_mission(const std::string & type, MissionFactory factory)
{
    factories_[type] = std::move(factory);
    RCLCPP_INFO(rclcpp::get_logger("MissionRegistry"), "Registered mission type: %s", type.c_str());
}

std::unique_ptr<MissionBase> MissionRegistry::create(
    const std::string & type,
    rclcpp::Node::SharedPtr node,
    const MissionCommand & cmd) const
{
    auto it = factories_.find(type);
    if (it == factories_.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("MissionRegistry"),
            "Unknown mission type: '%s'. Registered types: %zu",
            type.c_str(), factories_.size());
        return nullptr;
    }
    return it->second(node, cmd);
}

} // namespace robot_missions
