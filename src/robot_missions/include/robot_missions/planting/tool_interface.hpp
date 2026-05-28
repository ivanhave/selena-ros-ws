#pragma once

namespace robot_missions
{

struct SeedProfile;  // forward declaration — full type in seed_database.hpp

// ── TrayPose ──────────────────────────────────────────────────────────────────
// Position of a tool's seed tray or reservoir in the gantry frame.
// Coordinates relative to gantry home (all axes at 0).
// Each tool loads its own values from ROS2 node parameters at startup.

struct TrayPose
{
    float gx          = 0.0f;  // gantry X position of tray
    float gy          = 0.0f;  // gantry Y position of tray
    float z_approach  = 0.0f;  // Z height to approach tray safely
    float z_pick      = 0.0f;  // Z height to pick seed
};

// ── ToolBase ──────────────────────────────────────────────────────────────────
// Abstract tool interface for planting tools (pick-and-place cycle).
// ExecutionEngine calls only these methods — it does not know the tool type.
//
// Note: this interface is specific to planting tools (gripper, vacuum).
// Other mission types (weeding, monitoring) define their own hardware abstractions
// in their own subdirectory — they do not use ToolBase.
//
// Current implementations:
//   GripperTool — uses MG996R servo gripper
//
// Future implementations:
//   VacuumTool  — uses vacuum pump for small seeds (tool_id=2 in seeds.csv)

class ToolBase
{
public:
    virtual ~ToolBase() = default;

    // Called once per mission with the seed profile before execution starts.
    // Override to apply seed-specific tool settings.
    // Default is a no-op so tools that need no configuration require no changes.
    virtual void configure(const SeedProfile & /*profile*/) {}

    // Pick one seed from current gantry position.
    // Blocks until pick is complete.
    virtual void pick() = 0;

    // Release seed at current gantry position.
    // Blocks until release is complete.
    virtual void release() = 0;

    // Returns true if tool is ready to pick.
    virtual bool is_ready() const = 0;

    // Tray/reservoir position in gantry frame.
    // ExecutionEngine calls this to know where to move before pick().
    virtual TrayPose get_tray_pose() const = 0;

    // Tool name for logging.
    virtual const char * name() const = 0;
};

} // namespace robot_missions
