#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include "robot_navigation/navigation_provider.hpp"

namespace robot_missions
{

// ── MissionCommand ────────────────────────────────────────────────────────────
// Universal command structure for all mission types.
// MissionManager reads only mission_type to route to the right class.
// Each subclass interprets target and quantity in its own way.

struct MissionCommand
{
    std::string mission_type;   // "plant", "water", "monitor", "weed"
    std::string target;         // "pea", "naut", "all_plants", "sector_A"
    float       quantity;       // 5.0
    std::string quantity_unit;  // "meters", "seeds", "liters"
    float       start_x;        // world X coordinate (odometry/RTK)
    float       start_y;        // world Y coordinate
    float       start_yaw;      // world heading in radians (0 = robot faces +X)
};

// ── MissionResult ────────────────────────────────────────────────────────────
// Returned by every mission when it completes or is aborted.
struct MissionResult
{
    bool    success;
    std::string report;
    uint32_t items_completed;
    float    distance_covered_m;
};

// ── MissionBase ───────────────────────────────────────────────────────────────
// Abstract interface that every mission type must implement.
// Adding a new mission = create a new class that inherits from MissionBase.
// Mission Manager calls these methods without knowing the mission type.
//
// Lifecycle:
//   validate() → plan() → execute()
//   At any point: pause() → resume() or abort()

class MissionBase
{
public:
    virtual ~MissionBase() = default;

    // -- Lifecycle methods ----------------------------------------------------

    // Validate the command parameters before anything moves.
    // Returns true if safe to proceed.
    virtual bool validate() = 0;

    // Compute the full plan — pure math, no hardware.
    // Called after validate(), before execute().
    virtual bool plan() = 0;

    // Execute the plan — drives hardware.
    // Runs until complete, paused, or aborted.
    virtual bool execute() = 0;

    // Pause mid-mission safely. Preserves state for resume.
    virtual void pause() = 0;

    // Resume from paused state.
    virtual void resume() = 0;

    // Stop immediately and recover to safe state.
    virtual void abort() = 0;

    // Return what was accomplished.
    virtual MissionResult report() = 0;

    // -- Status queries -------------------------------------------------------

    // Progress 0.0 to 1.0
    virtual float get_progress() = 0;

    // Human readable current step description
    virtual std::string get_current_step() = 0;

    // All robot base stop positions in world frame, for path visualisation.
    // Called after plan(). Default returns empty (missions that don't move the base).
    // start_x/y/yaw = the world pose the robot arrived at before execute().
    virtual std::vector<robot_navigation::Pose2D> get_route_waypoints(
        float /*start_x*/, float /*start_y*/, float /*start_yaw*/) const
    { return {}; }
};

} // namespace robot_missions