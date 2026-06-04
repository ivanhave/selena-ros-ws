#pragma once

namespace robot_navigation
{

struct Pose2D
{
    float x   = 0.0f;
    float y   = 0.0f;
    float yaw = 0.0f;  // radians, robot forward direction
};

// ── NavigationProvider ────────────────────────────────────────────────────────
// Abstract interface for mobile base positioning.
// Missions use only these calls — they have no knowledge of odometry,
// Nav2, or any other localization/navigation implementation.

class NavigationProvider
{
public:
    virtual ~NavigationProvider() = default;

    // Current robot pose in the world frame.
    virtual Pose2D get_pose() = 0;

    // Move the base to an absolute world pose.
    // Blocking — returns true on success, false on timeout or failure.
    virtual bool move_to(Pose2D target) = 0;

    // Rotate in place to align the robot forward axis with the given world yaw.
    // Used before precision tasks (planting). Implementations should use a tighter
    // tolerance than the general move_to/rotate_to calls.
    // Default: move_to with same position (triggers in-place rotate_to).
    virtual bool align_to_yaw(float yaw) {
        auto p = get_pose();
        p.yaw = yaw;
        return move_to(p);
    }

    // Stop navigation immediately (thread-safe, called from another thread).
    virtual void abort() {}

    // Clear a previous abort so the next move_to works normally.
    virtual void reset_abort() {}
};

} // namespace robot_navigation
