#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace robot_missions
{

// ── SeedProfile ───────────────────────────────────────────────────────────────
// Data for one seed type. Read from config/seeds.csv.
// Add new seed types by editing the CSV — no code changes needed.

struct SeedProfile
{
    std::string name;           // "naut", "pea", "fasole"
    float       spacing_x_mm;  // spacing across robot travel (gantry X axis)
    float       spacing_y_mm;  // spacing along robot travel  (gantry Y axis)
    float       depth_mm;      // planting depth below soil surface
    uint32_t    tool_id;       // 1=gripper, 2=vacuum (future)
    uint32_t    tool_param;    // tool-interpreted value:
                               //   GripperTool: grip angle in degrees (0=open, 55=closed)
                               //   VacuumTool:  suction hold time in ms (future)
};

// ── SeedDatabase ──────────────────────────────────────────────────────────────
// Reads and stores seed profiles from CSV file.

class SeedDatabase
{
public:
    SeedDatabase() = default;

    bool load(const std::string & csv_path);
    bool get_profile(const std::string & name, SeedProfile & profile) const;
    std::vector<std::string> get_all_names() const;

private:
    std::vector<SeedProfile> profiles_;
};

} // namespace robot_missions
