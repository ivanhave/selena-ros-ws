#include "robot_missions/planting/execution_engine.hpp"
#include "robot_gantry/gantry_constants.hpp"

static constexpr int SERVICE_WAIT_S = 2;
static constexpr int GOAL_WAIT_S    = 5;
static constexpr int STEP_MS        = 50;
#include <chrono>
#include <cmath>
#include <thread>

namespace robot_missions
{

ExecutionEngine::ExecutionEngine(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ToolBase> tool,
    std::shared_ptr<robot_navigation::NavigationProvider> nav,
    const std::atomic<bool> * paused,
    DepthSensor *              depth)
: node_(node)
, tool_(tool)
, nav_(nav)
, tray_(tool->get_tray_pose())
, paused_(paused)
, depth_sensor_(depth)
{
    gantry_client_ = rclcpp_action::create_client<GantryMove>(node_, "/gantry/move");
    RCLCPP_INFO(node_->get_logger(), "ExecutionEngine ready.");
}

uint32_t ExecutionEngine::execute(const PlantingPlanner & planner)
{
    seeds_planted_ = 0;
    uint32_t current_stop  = 0;

    for (uint32_t seed_index = 1;
         seed_index <= planner.get_total_seeds();
         seed_index++)
    {
        if (stop_requested_) {
            RCLCPP_INFO(node_->get_logger(), "Stop requested. Aborting.");
            break;
        }

        uint32_t stop = planner.get_stop_index(seed_index);
        if (stop != current_stop || seed_index == 1)
        {
            current_stop = stop;
            if (seed_index > 1) {
                MobileCommand mob = planner.get_robot_stop(seed_index);

                robot_navigation::Pose2D cur = nav_->get_pose();
                robot_navigation::Pose2D target;
                target.x   = cur.x + mob.advance_meters * std::cos(cur.yaw);
                target.y   = cur.y + mob.advance_meters * std::sin(cur.yaw);
                target.yaw = cur.yaw;

                retract_z();   // Z must be home before base moves
                RCLCPP_INFO(node_->get_logger(),
                    "Moving base to (%.3f, %.3f) — stop %u",
                    target.x, target.y, stop);
                if (!nav_->move_to(target)) {
                    if (stop_requested_) break;
                    RCLCPP_WARN(node_->get_logger(),
                        "Base movement to stop %u failed.", stop);
                }
            }
        }

        if (stop_requested_) break;

        GantryCommand cmd = planner.get_seed_position(seed_index);

        RCLCPP_INFO(node_->get_logger(), "Seed %u: gx=%.3f gy=%.3f depth=%.3fm",
            seed_index, cmd.gx, cmd.gy, cmd.depth);

        move_gantry_to_tray();
        if (stop_requested_) break;
        tool_->pick();

        move_gantry_to(cmd.gx, cmd.gy);
        if (stop_requested_) break;

        // TODO: replace soil_z with real sensor reading once hardware is mounted.
        float soil_z = depth_sensor_ ? depth_sensor_->measure_soil_distance_m() : 0.0f;
        lower_z(soil_z + cmd.depth);

        tool_->release();
        retract_z();

        ++seeds_planted_;
        RCLCPP_INFO(node_->get_logger(), "Seed %u planted. Total: %u",
            seed_index, seeds_planted_.load());

        while (paused_ && paused_->load() && !stop_requested_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    retract_z();   // ensure Z is home regardless of how the loop exited
    return seeds_planted_.load();
}

void ExecutionEngine::stop()
{
    stop_requested_.store(true);
}

void ExecutionEngine::send_gantry_move(double x, double y, double z)
{
    if (!gantry_client_->wait_for_action_server(
            std::chrono::seconds(SERVICE_WAIT_S))) {
        RCLCPP_ERROR(node_->get_logger(), "GantryMove action server not available");
        return;
    }

    GantryMove::Goal goal;
    goal.x = x;
    goal.y = y;
    goal.z = z;

    auto gh_future = gantry_client_->async_send_goal(goal);
    if (gh_future.wait_for(std::chrono::seconds(GOAL_WAIT_S))
        != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "GantryMove: goal not accepted within timeout");
        return;
    }
    auto gh = gh_future.get();
    if (!gh) {
        RCLCPP_ERROR(node_->get_logger(), "GantryMove: goal rejected by server");
        return;
    }

    auto result_future = gantry_client_->async_get_result(gh);
    while (result_future.wait_for(
               std::chrono::milliseconds(STEP_MS))
           != std::future_status::ready)
    {
        if (stop_requested_.load()) {
            gantry_client_->async_cancel_goal(gh);
            result_future.wait();
            return;
        }
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
        current_gx_ = static_cast<float>(x);
        current_gy_ = static_cast<float>(y);
        current_gz_ = static_cast<float>(z);
    } else {
        RCLCPP_WARN(node_->get_logger(), "GantryMove to (%.3f,%.3f,%.3f) failed: %s",
            x, y, z, result.result ? result.result->message.c_str() : "unknown");
    }
}

void ExecutionEngine::move_gantry_to(float gx, float gy)
{
    // robot_gantry retracts Z before XY automatically
    send_gantry_move(gx, gy, gantry_constants::HOME_Z);
}

void ExecutionEngine::lower_z(float depth_m)
{
    // Pass current XY so robot_gantry sees no XY delta and skips the XY step
    send_gantry_move(current_gx_, current_gy_, depth_m);
}

void ExecutionEngine::retract_z()
{
    send_gantry_move(current_gx_, current_gy_, gantry_constants::HOME_Z);
}

void ExecutionEngine::move_gantry_to_tray()
{
    // Single GantryMove: robot_gantry retracts Z if needed, moves XY, lowers to z_pick
    send_gantry_move(tray_.gx, tray_.gy, tray_.z_pick);
}

void ExecutionEngine::wait_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace robot_missions
