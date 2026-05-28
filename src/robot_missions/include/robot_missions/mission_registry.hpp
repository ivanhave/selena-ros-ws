#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "robot_missions/mission_base.hpp"

namespace robot_missions
{

// ── MissionRegistry ───────────────────────────────────────────────────────────
// Singleton map from mission_type string → factory function.
// MissionManager calls create() without knowing any concrete mission class.
//
// Adding a new mission type:
//   1. Create a new subdirectory with your MissionBase subclass.
//   2. Add a static create_from_command() method to your class.
//   3. Place REGISTER_MISSION("type", YourClass) at the bottom of your .cpp file.
//   Zero changes to any existing file.

using MissionFactory = std::function<
    std::unique_ptr<MissionBase>(rclcpp::Node::SharedPtr, const MissionCommand &)>;

class MissionRegistry
{
public:
    static MissionRegistry & instance();

    void register_mission(const std::string & type, MissionFactory factory);

    std::unique_ptr<MissionBase> create(
        const std::string & type,
        rclcpp::Node::SharedPtr node,
        const MissionCommand & cmd) const;

private:
    MissionRegistry() = default;
    std::unordered_map<std::string, MissionFactory> factories_;
};

// ── MissionRegistrar ──────────────────────────────────────────────────────────
// Helper struct whose constructor triggers registration at static-init time.
// Use the REGISTER_MISSION macro below — do not instantiate directly.

struct MissionRegistrar
{
    MissionRegistrar(const std::string & type, MissionFactory factory)
    {
        MissionRegistry::instance().register_mission(type, std::move(factory));
    }
};

} // namespace robot_missions

// ── REGISTER_MISSION ─────────────────────────────────────────────────────────
// Place at the bottom of your mission's .cpp file (outside any namespace).
//
// Example:
//   REGISTER_MISSION("plant", PlantingMission)
//
// Requires ClassName to have:
//   static std::unique_ptr<ClassName> create_from_command(
//       rclcpp::Node::SharedPtr, const MissionCommand &);

#define REGISTER_MISSION(type_str, ClassName)                                    \
    static ::robot_missions::MissionRegistrar                                    \
    _mission_registrar_##ClassName(                                              \
        type_str,                                                                \
        [](::rclcpp::Node::SharedPtr node,                                       \
           const ::robot_missions::MissionCommand & cmd)                         \
               -> std::unique_ptr<::robot_missions::MissionBase>                 \
        {                                                                        \
            return ClassName::create_from_command(node, cmd);                    \
        })
