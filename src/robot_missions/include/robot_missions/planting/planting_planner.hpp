#pragma once

#include <cstdint>
#include "robot_missions/planting/seed_database.hpp"

namespace robot_missions
{

// ── GantryCommand ─────────────────────────────────────────────────────────────
// What the gantry needs to plant one seed.
// Coordinates are in gantry frame, meters from gantry home.

struct GantryCommand
{
    float gx;     // gantry X position (across robot travel)
    float gy;     // gantry Y position (along robot travel)
    float depth;  // planting depth in meters
};

// ── MobileCommand ─────────────────────────────────────────────────────────────
// What the mobile base needs for one robot stop.

struct MobileCommand
{
    float    advance_meters;  // how far to move forward from previous stop
    uint32_t stop_index;      // which stop number this is (0-based)
};

// ── PlantingPlanner ───────────────────────────────────────────────────────────
// Pure mathematics. No ROS2. No hardware. No memory storage of all coordinates.
// Given a seed index, computes coordinates on demand.
//
// Coordinate convention:
//   gx = across robot travel = gantry X axis (y_axis_joint in URDF)
//   gy = along robot travel  = gantry Y axis (x_axis_joint in URDF)
//
// Geometry formulas:
//   seeds_in_x     = floor(x_travel / spacing_x) + 1
//   seeds_in_y     = floor(y_travel / spacing_y) + 1
//   x_offset       = (x_travel - (seeds_in_x-1) * spacing_x) / 2
//   y_offset       = (y_travel - (seeds_in_y-1) * spacing_y) / 2
//   robot_advance  = seeds_in_y * spacing_y

class PlantingPlanner
{
public:
    // Gantry physical limits in meters
    static constexpr float GANTRY_X_TRAVEL = 0.845f;
    static constexpr float GANTRY_Y_TRAVEL = 0.290f;

    PlantingPlanner() = default;

    // Initialize planner with seed profile and mission parameters.
    // Call this before any get_* functions.
    // Either strip_length_m or seed_count must be provided (not both).
    // Pass 0 for the one not used.
    bool init(
        const SeedProfile & profile,
        float strip_length_m,
        uint32_t seed_count);

    // ── On-demand coordinate queries ─────────────────────────────────────────
    // These are pure math functions — no state changes, no storage.
    // Call with any seed_index from 1 to get_total_seeds().

    // Returns gantry position for seed number seed_index.
    // seed_index is 1-based.
    GantryCommand get_seed_position(uint32_t seed_index) const;

    // Returns robot stop info for the stop that contains seed_index.
    // seed_index is 1-based.
    MobileCommand get_robot_stop(uint32_t seed_index) const;

    // Returns which stop index seed_index belongs to (0-based).
    uint32_t get_stop_index(uint32_t seed_index) const;

    // Returns local index of seed within its stop (0-based).
    uint32_t get_local_index(uint32_t seed_index) const;

    // ── Summary queries ───────────────────────────────────────────────────────

    uint32_t get_total_seeds() const;
    uint32_t get_total_stops() const;
    uint32_t get_seeds_per_stop() const;
    float    get_robot_advance() const;

    // Seeds in X direction (across robot travel) per stop
    uint32_t get_seeds_in_x() const;

    // Seeds in Y direction (along robot travel) per stop
    uint32_t get_seeds_in_y() const;

private:
    SeedProfile profile_;
    float       strip_length_m_  = 0.0f;
    uint32_t    total_seeds_     = 0;
    uint32_t    total_stops_     = 0;
    uint32_t    seeds_in_x_      = 0;
    uint32_t    seeds_in_y_      = 0;
    uint32_t    seeds_per_stop_  = 0;
    float       x_offset_        = 0.0f;
    float       y_offset_        = 0.0f;
    float       robot_advance_   = 0.0f;
    bool        initialized_     = false;
};

} // namespace robot_missions