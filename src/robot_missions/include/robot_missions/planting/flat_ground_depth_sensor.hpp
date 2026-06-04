#pragma once
#include "robot_missions/planting/depth_sensor.hpp"

namespace robot_missions
{

// Concrete DepthSensor for flat terrain (testing, flat floors, lab environment).
// Returns a fixed configured distance from Z-home to soil surface.
// Replace with a real sensor implementation once hardware is available.
class FlatGroundDepthSensor : public DepthSensor
{
public:
    explicit FlatGroundDepthSensor(float soil_surface_z_m)
    : soil_surface_z_m_(soil_surface_z_m) {}

    float measure_soil_distance_m() override { return soil_surface_z_m_; }

private:
    float soil_surface_z_m_;
};

} // namespace robot_missions
