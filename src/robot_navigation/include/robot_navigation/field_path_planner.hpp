#pragma once
#include "robot_navigation/navigation_provider.hpp"
#include <cmath>
#include <vector>

namespace robot_navigation
{

struct FieldConfig
{
    float origin_x     = 0.0f;
    float origin_y     = 0.0f;
    float origin_yaw   = 0.0f;
    float strip_width  = 0.845f;
    float strip_length = 10.0f;
    float strip_spacing = 1.0f;
};

// Plan a strip-safe path from current to target, routing through the near
// headland (before strip start) when a strip change is required.
//
// Constraints enforced:
//  - When on a strip the robot exits linearly (backward or forward, no rotation).
//  - Rotation only happens in the headland zone (along < 0 or along > strip_length).
//
// move_to() drives backward automatically when angle_diff > π/2, so waypoints
// are simply expressed as world Pose2D values — no special backward flag needed.
inline std::vector<Pose2D> plan_field_path(
    const Pose2D & current,
    const Pose2D & target,
    const FieldConfig & fc,
    float headland_clearance = 1.5f)
{
    const float cosY = std::cos(fc.origin_yaw);
    const float sinY = std::sin(fc.origin_yaw);

    auto toField = [&](float wx, float wy, float & along, float & perp) {
        float dx = wx - fc.origin_x;
        float dy = wy - fc.origin_y;
        along =  dx * cosY + dy * sinY;
        perp  = -dx * sinY + dy * cosY;
    };

    auto toWorld = [&](float along, float perp) -> Pose2D {
        return {
            fc.origin_x + along * cosY - perp * sinY,
            fc.origin_y + along * sinY + perp * cosY,
            0.0f
        };
    };

    float cur_along, cur_perp, tgt_along, tgt_perp;
    toField(current.x, current.y, cur_along, cur_perp);
    toField(target.x,  target.y,  tgt_along, tgt_perp);

    // Strip assignment: round perp to nearest strip index
    auto strip_of = [&](float perp) {
        return static_cast<int>(std::round(perp / fc.strip_spacing));
    };

    int cur_strip = strip_of(cur_perp);
    int tgt_strip = strip_of(tgt_perp);

    // ── Same strip: direct linear move (move_to handles fwd vs bwd) ──────────
    if (cur_strip == tgt_strip) {
        return { target };
    }

    // ── Different strip: route through near headland ──────────────────────────
    const float near = -headland_clearance;
    const float wp1_along = near;
    const float wp2_along = near;

    // WP1: near headland at current strip column
    // WP2: near headland at target strip column
    // WP3: target zone start
    Pose2D wp1 = toWorld(wp1_along, cur_perp);
    Pose2D wp2 = toWorld(wp2_along, tgt_perp);

    // Face toward WP2 when arriving at WP1
    wp1.yaw = std::atan2(wp2.y - wp1.y, wp2.x - wp1.x);
    // Face into the target strip when arriving at WP2
    wp2.yaw = fc.origin_yaw;

    // Already in (or past) near headland: skip WP1
    if (cur_along <= near + 0.05f) {
        if (std::fabs(cur_perp - tgt_perp) < 0.05f) {
            return { target };
        }
        return { wp2, target };
    }

    return { wp1, wp2, target };
}

}  // namespace robot_navigation
