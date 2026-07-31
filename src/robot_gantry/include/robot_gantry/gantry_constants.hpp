#pragma once
#include <cstdint>

namespace gantry_constants {

// Velocity caps (m/s) — experimentally verified on Selena hardware
constexpr double  X_VEL_CAP    = 0.15;   // GT2 belt, 0.845 m travel
constexpr double  Y_VEL_CAP    = 0.06;   // T8 lead screw, 0.290 m travel
constexpr double  Z_VEL_CAP    = 0.04;   // T8 lead screw, 0.250 m travel (vertical)

// MKS motor acceleration byte for velocity-mode commands (0xF6)
constexpr uint8_t VEL_ACC      = 230;

// Soft-limit decel zone — used by GantryHardwareInterface::write()
constexpr double  DECEL_ZONE_M = 0.080;  // sqrt braking begins 80 mm before limit
constexpr double  GUARD_ZONE_M = 0.025;  // hard zero within 25 mm of limit

// Trajectory timing — used by robot_gantry to compute JTC time_from_start
// Peak cubic-spline velocity = SPLINE_FACTOR x (distance / time)
constexpr double  SPLINE_FACTOR  = 1.5;   // so time = SPLINE_FACTOR x dist / cap
constexpr double  MIN_MOVE_S     = 0.5;   // minimum trajectory duration (seconds)
constexpr double  MOVE_MARGIN_S  = 0.8;   // extra buffer beyond computed travel time
constexpr int     SETTLE_MS      = 50;    // two-point Z trajectory: damp-phase duration

// Home position
constexpr double  HOME_Z         = 0.0;   // Z joint value when fully retracted

// XY arrival detection
constexpr double  TOL_M          = 0.002; // 2 mm position tolerance
constexpr int     STABLE_COUNT   = 15;    // consecutive samples within TOL_M (x50 ms = 750 ms)
constexpr int     TIMEOUT_MS     = 5000;  // max wait for JTC arrival confirmation
} // namespace gantry_constants
