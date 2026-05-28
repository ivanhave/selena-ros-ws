#pragma once

namespace robot_missions
{

// ── DepthSensor ───────────────────────────────────────────────────────────────
// Optional interface for measuring soil surface distance from the gantry Z axis.
//
// WHY THIS EXISTS:
//   The field surface is not perfectly flat. Using a fixed depth_mm from seeds.csv
//   assumes gantry Z-home is at a known fixed height above soil — which breaks on
//   uneven terrain. A real distance sensor (e.g. ultrasonic, ToF, or IR) mounted
//   on the gantry end-effector will report actual soil distance at each XY position.
//
// INTEGRATION POINT:
//   ExecutionEngine holds an optional DepthSensor*. Before lowering Z for each
//   seed, it calls measure_soil_distance_m() if a sensor is present:
//
//     float soil_z = sensor ? sensor->measure_soil_distance_m() : 0.0f;
//     float actual_z = soil_z + seed_depth_m;
//     lower_z(actual_z);
//
//   Without a sensor, soil_z defaults to 0.0 (assumes gantry Z-home is at soil).
//   This is the current behaviour — a safe placeholder until hardware is ready.
//
// TO INTEGRATE A REAL SENSOR:
//   1. Implement this interface (e.g. UltrasonicDepthSensor)
//   2. Subscribe to the sensor topic in the constructor
//   3. Implement measure_soil_distance_m() to return the latest reading
//   4. Pass the sensor to ExecutionEngine in PlantingMission::execute()
//   No other files change.
//
// NOTE: If the sensor reading is unavailable (timeout / no topic), the implementation
// should return 0.0 and log a warning — safe degradation to fixed depth.

class DepthSensor
{
public:
    virtual ~DepthSensor() = default;

    // Returns distance from gantry Z-home (fully retracted) to soil surface, in meters.
    // Positive = soil is below home. Blocks until a fresh reading is available.
    // Returns 0.0 on timeout or hardware error (safe fallback to fixed depth).
    virtual float measure_soil_distance_m() = 0;
};

} // namespace robot_missions
