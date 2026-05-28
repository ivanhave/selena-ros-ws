#include "robot_missions/planting/planting_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace robot_missions
{

bool PlantingPlanner::init(
    const SeedProfile & profile,
    float strip_length_m,
    uint32_t seed_count)
{
    profile_ = profile;

    // Convert spacing from mm to meters
    float sx = profile_.spacing_x_mm / 1000.0f;
    float sy = profile_.spacing_y_mm / 1000.0f;

    // ── Seeds per stop ────────────────────────────────────────────────────────
    seeds_in_x_ = static_cast<uint32_t>(std::floor(GANTRY_X_TRAVEL / sx) + 1);
    seeds_in_y_ = static_cast<uint32_t>(std::floor(GANTRY_Y_TRAVEL / sy) + 1);
    seeds_per_stop_ = seeds_in_x_ * seeds_in_y_;

    // ── Centering offsets ─────────────────────────────────────────────────────
    x_offset_ = (GANTRY_X_TRAVEL - (seeds_in_x_ - 1) * sx) / 2.0f;
    y_offset_ = (GANTRY_Y_TRAVEL - (seeds_in_y_ - 1) * sy) / 2.0f;

    // ── Robot advance between stops ───────────────────────────────────────────
    robot_advance_ = seeds_in_y_ * sy;

    // ── Total seeds ───────────────────────────────────────────────────────────
    if (strip_length_m > 0.0f) {
        // Length mode — plant as many seeds as fit in strip_length_m
        uint32_t total_rows = static_cast<uint32_t>(
            std::floor(strip_length_m / sy) + 1);
        total_seeds_ = total_rows * seeds_in_x_;
        strip_length_m_ = strip_length_m;
    } else {
        // Count mode — plant exactly seed_count seeds
        total_seeds_ = seed_count;
        strip_length_m_ = (std::ceil(
            static_cast<float>(seed_count) / seeds_in_x_)) * sy;
    }

    // ── Total stops ───────────────────────────────────────────────────────────
    total_stops_ = static_cast<uint32_t>(
        std::ceil(static_cast<float>(total_seeds_) / seeds_per_stop_));

    initialized_ = true;

    auto log = rclcpp::get_logger("PlantingPlanner");
    RCLCPP_INFO(log, "Initialized: seed=%s seeds_in_x=%u seeds_in_y=%u "
        "seeds_per_stop=%u total_seeds=%u total_stops=%u "
        "x_offset=%.3fm y_offset=%.3fm robot_advance=%.3fm",
        profile_.name.c_str(),
        seeds_in_x_, seeds_in_y_, seeds_per_stop_,
        total_seeds_, total_stops_,
        x_offset_, y_offset_, robot_advance_);

    return true;
}

GantryCommand PlantingPlanner::get_seed_position(uint32_t seed_index) const
{
    // seed_index is 1-based
    uint32_t local = get_local_index(seed_index);

    float sx = profile_.spacing_x_mm / 1000.0f;
    float sy = profile_.spacing_y_mm / 1000.0f;

    uint32_t col = local % seeds_in_x_;
    uint32_t row = local / seeds_in_x_;

    GantryCommand cmd;
    cmd.gx    = x_offset_ + col * sx;
    cmd.gy    = y_offset_ + row * sy;
    cmd.depth = profile_.depth_mm / 1000.0f;

    return cmd;
}

MobileCommand PlantingPlanner::get_robot_stop(uint32_t seed_index) const
{
    MobileCommand cmd;
    cmd.stop_index     = get_stop_index(seed_index);
    cmd.advance_meters = robot_advance_;
    return cmd;
}

uint32_t PlantingPlanner::get_stop_index(uint32_t seed_index) const
{
    // seed_index is 1-based
    return (seed_index - 1) / seeds_per_stop_;
}

uint32_t PlantingPlanner::get_local_index(uint32_t seed_index) const
{
    // seed_index is 1-based
    return (seed_index - 1) % seeds_per_stop_;
}

uint32_t PlantingPlanner::get_total_seeds()    const { return total_seeds_;    }
uint32_t PlantingPlanner::get_total_stops()    const { return total_stops_;    }
uint32_t PlantingPlanner::get_seeds_per_stop() const { return seeds_per_stop_; }
float    PlantingPlanner::get_robot_advance()  const { return robot_advance_;  }
uint32_t PlantingPlanner::get_seeds_in_x()     const { return seeds_in_x_;     }
uint32_t PlantingPlanner::get_seeds_in_y()     const { return seeds_in_y_;     }

} // namespace robot_missions