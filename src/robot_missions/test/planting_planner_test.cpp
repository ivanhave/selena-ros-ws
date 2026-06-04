// Unit tests for PlantingPlanner — multi-stop geometry.
// Case: 8 naut seeds (600mm × 200mm) → 2 stops of 4, robot advances 0.4m between stops.

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "robot_missions/planting/planting_planner.hpp"

namespace robot_missions
{

// naut profile matching seeds.csv
static SeedProfile make_naut()
{
    SeedProfile p;
    p.name         = "naut";
    p.spacing_x_mm = 600.0f;
    p.spacing_y_mm = 200.0f;
    p.depth_mm     = 0.0f;
    p.tool_id      = 1;
    p.tool_param   = 48;
    return p;
}

// ── 8-seed layout ──────────────────────────────────────────────────────────────
// Gantry X=0.845m, Y=0.290m
// seeds_in_x = floor(0.845/0.6)+1 = 2   x_offset = (0.845 - 0.6) / 2 = 0.1225m
// seeds_in_y = floor(0.290/0.2)+1 = 2   y_offset = (0.290 - 0.2) / 2 = 0.045m
// seeds_per_stop = 4   total_stops = 2   robot_advance = 2×0.2 = 0.4m

TEST(PlantingPlannerTest, EightSeedsTwoStops)
{
    PlantingPlanner planner;
    ASSERT_TRUE(planner.init(make_naut(), 0.0f, 8));

    EXPECT_EQ(planner.get_total_seeds(),    8u);
    EXPECT_EQ(planner.get_total_stops(),    2u);
    EXPECT_EQ(planner.get_seeds_per_stop(), 4u);
    EXPECT_EQ(planner.get_seeds_in_x(),     2u);
    EXPECT_EQ(planner.get_seeds_in_y(),     2u);
    EXPECT_NEAR(planner.get_robot_advance(), 0.4f, 0.001f);
}

TEST(PlantingPlannerTest, StopAssignments)
{
    PlantingPlanner planner;
    ASSERT_TRUE(planner.init(make_naut(), 0.0f, 8));

    for (uint32_t i = 1; i <= 4; ++i)
        EXPECT_EQ(planner.get_stop_index(i), 0u) << "seed " << i;
    for (uint32_t i = 5; i <= 8; ++i)
        EXPECT_EQ(planner.get_stop_index(i), 1u) << "seed " << i;
}

TEST(PlantingPlannerTest, GantryPositions)
{
    PlantingPlanner planner;
    ASSERT_TRUE(planner.init(make_naut(), 0.0f, 8));

    const float x_off = 0.1225f;  // (0.845 - 0.6) / 2
    const float y_off = 0.045f;   // (0.290 - 0.2) / 2
    const float sx    = 0.6f;
    const float sy    = 0.2f;
    const float tol   = 0.001f;

    // Seeds 1-4 (stop 0) and seeds 5-8 (stop 1) share the same gantry positions
    // because the base moved — world positions differ, gantry-frame positions repeat.
    struct E { float gx, gy; };
    const E expected[4] = {
        {x_off,      y_off},       // col=0 row=0
        {x_off + sx, y_off},       // col=1 row=0
        {x_off,      y_off + sy},  // col=0 row=1
        {x_off + sx, y_off + sy},  // col=1 row=1
    };

    for (uint32_t stop = 0; stop < 2; ++stop) {
        for (uint32_t local = 0; local < 4; ++local) {
            uint32_t seed_index = stop * 4 + local + 1;
            GantryCommand cmd = planner.get_seed_position(seed_index);
            EXPECT_NEAR(cmd.gx,    expected[local].gx, tol) << "seed " << seed_index << " gx";
            EXPECT_NEAR(cmd.gy,    expected[local].gy, tol) << "seed " << seed_index << " gy";
            EXPECT_NEAR(cmd.depth, 0.0f,               tol) << "seed " << seed_index << " depth";
        }
    }
}

TEST(PlantingPlannerTest, WorldSpacingContinuous)
{
    // Verify that the last row of stop 0 and first row of stop 1 are exactly
    // spacing_y apart in world coordinates.
    //   world_y(seed) = robot_advance * stop_index + gantry_gy
    // Last row of stop 0:  gy = y_offset + (seeds_in_y-1)*sy = 0.045 + 0.2 = 0.245m
    // First row of stop 1: gy = y_offset = 0.045m, robot at +0.4m
    //   gap = 0.4 + 0.045 - 0.245 = 0.2m = spacing_y ✓

    PlantingPlanner planner;
    ASSERT_TRUE(planner.init(make_naut(), 0.0f, 8));

    float adv = planner.get_robot_advance();

    // last seeds of stop 0 (seeds 3 and 4, both at gy = y_off + sy)
    float gy_last_stop0 = planner.get_seed_position(3).gy;  // row=1 of stop 0
    EXPECT_NEAR(gy_last_stop0, planner.get_seed_position(4).gy, 0.001f);

    // first seeds of stop 1 (seeds 5 and 6, both at gy = y_off)
    float gy_first_stop1 = planner.get_seed_position(5).gy;  // row=0 of stop 1
    EXPECT_NEAR(gy_first_stop1, planner.get_seed_position(6).gy, 0.001f);

    // world gap = advance + first_gy_stop1 - last_gy_stop0 = spacing_y
    float world_gap = adv + gy_first_stop1 - gy_last_stop0;
    EXPECT_NEAR(world_gap, 0.2f, 0.001f) << "gap between last row of stop 0 and first row of stop 1";
}

TEST(PlantingPlannerTest, MobileCommand)
{
    PlantingPlanner planner;
    ASSERT_TRUE(planner.init(make_naut(), 0.0f, 8));

    MobileCommand mob = planner.get_robot_stop(5);  // first seed of stop 1
    EXPECT_EQ(mob.stop_index, 1u);
    EXPECT_NEAR(mob.advance_meters, 0.4f, 0.001f);
}

} // namespace robot_missions

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
