#pragma once

// ── WeedingMission — FUTURE PLACEHOLDER ──────────────────────────────────────
//
// This file is a design placeholder. WeedingMission has NOT been implemented yet.
// It documents the intended architecture so the design intent is clear when
// implementation begins.
//
// ── Architecture notes ────────────────────────────────────────────────────────
//
// WeedingMission inherits MissionBase and is self-registered:
//   REGISTER_MISSION("weed", WeedingMission)
//
// MissionManager and the action server need ZERO changes when this is added.
//
// ── Key differences from PlantingMission ─────────────────────────────────────
//
// PlantingMission uses:
//   ToolBase (pick/release from tray) + PlantingPlanner (seed grid) + ExecutionEngine
//
// WeedingMission does NOT use any of those. It will have its own internal pieces:
//
//   WeederActuator   — hardware abstraction for the weeding tool
//     activate()     — engage the weeding mechanism (motor, blade, etc.)
//     deactivate()   — disengage safely
//     No tray, no pick/release cycle.
//
//   WeedingPlanner   — row-based path planner (not a seed grid)
//     Computes gantry sweep paths along crop rows.
//     Input: row spacing, row length, weed detection map (future).
//
//   WeedingEngine    — hardware orchestrator
//     Drives the mobile base along rows.
//     Activates WeederActuator while gantry sweeps the row.
//     Handles pause/resume/abort.
//
// ── MissionCommand interpretation ─────────────────────────────────────────────
//
//   mission_type:  "weed"
//   target:        "all_rows", "sector_A", etc. (TBD)
//   quantity:      row length in meters, or area
//   quantity_unit: "meters" or "sqm"
//   start_x/y:     world start position
//
// ── Implementation steps when ready ──────────────────────────────────────────
//
//   1. Create weeder_actuator.hpp/cpp  (hardware driver, CAN or GPIO)
//   2. Create weeding_planner.hpp/cpp  (path along rows)
//   3. Create weeding_engine.hpp/cpp   (orchestrator)
//   4. Implement WeedingMission with validate/plan/execute/pause/resume/abort/report
//   5. Add REGISTER_MISSION("weed", WeedingMission) at bottom of weeding_mission.cpp
//   6. Add weeding_mission.cpp + weeding_engine.cpp to CMakeLists.txt library sources
//   That is all. No other files change.

// namespace robot_missions {
// class WeedingMission : public MissionBase { ... };
// } // namespace robot_missions
