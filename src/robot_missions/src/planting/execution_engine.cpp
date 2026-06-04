#include "robot_missions/planting/execution_engine.hpp"
#include <chrono>
#include <cmath>
#include <thread>
#include <cstdint>

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
    gantry_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            for (size_t i = 0; i < msg->name.size(); ++i) {
                joint_pos_[msg->name[i]] = msg->position[i];
            }
        });

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
        // Sensor should return distance from Z-home to soil surface at current XY.
        // Until then, soil_z = 0.0 assumes Z-home is exactly at soil level.
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

    return seeds_planted_.load();
}

void ExecutionEngine::stop()
{
    stop_requested_.store(true);
}

// Sleep for up to `ms` milliseconds, waking every 50 ms to check stop_requested_.
void ExecutionEngine::interruptible_sleep(int ms)
{
    constexpr int STEP_MS = 50;
    int remaining = ms;
    while (remaining > 0 && !stop_requested_.load()) {
        int chunk = std::min(remaining, STEP_MS);
        std::this_thread::sleep_for(std::chrono::milliseconds(chunk));
        remaining -= chunk;
    }
}


int ExecutionEngine::xy_move_ms(float gx, float gy) const
{
    float dx = std::abs(gx - current_gx_);
    float dy = std::abs(gy - current_gy_);
    float t  = std::max({1.5f * dx / X_VEL_CAP,
                         1.5f * dy / Y_VEL_CAP,
                         0.5f});
    return static_cast<int>((t + MOVE_MARGIN_S) * 1000.0f);
}

int ExecutionEngine::z_move_ms(float gz) const
{
    float dz = std::abs(gz - current_gz_);
    float t  = std::max(1.5f * dz / Z_VEL_CAP, 0.5f);
    return static_cast<int>((t + MOVE_MARGIN_S) * 1000.0f);
}

void ExecutionEngine::send_xy_command(float gx, float gy)
{
    // Z must be retracted before calling this.
    // All three joints are sent in one message so the JTC holds Z at its measured
    // position while X,Y travel — prevents Z ghost movement from the 50-RPM floor.
    int ms = xy_move_ms(gx, gy);

    double mz;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto itz = joint_pos_.find("z_axis_joint");
        mz = (itz != joint_pos_.end()) ? itz->second : 0.0;
    }

    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = node_->now();
    msg.joint_names  = {"x_axis_joint", "y_axis_joint", "z_axis_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions  = {gx, gy, mz};
    point.velocities = {0.0, 0.0, 0.0};
    point.time_from_start.sec     = ms / 1000;
    point.time_from_start.nanosec = (ms % 1000) * 1000000;

    msg.points.push_back(point);
    gantry_pub_->publish(msg);

    current_gx_ = gx;
    current_gy_ = gy;

    RCLCPP_INFO(node_->get_logger(), "XY → (%.3f, %.3f) | Z locked at %.4f | ETA %.1fs",
        gx, gy, mz, ms / 1000.0f);
    interruptible_sleep(ms);
    if (stop_requested_.load()) return;

    wait_xy_settled(gx, gy);
}

void ExecutionEngine::send_z_command(float gz)
{
    // X,Y must be at their target before calling this.
    //
    // Root cause of ghost movement: after an XY move the motor has residual velocity
    // (~0.5 mm/s is normal even after wait_xy_settled). When we issue a single-point
    // trajectory with X=mx, Y=my (same start and end position) but the JTC sees a
    // non-zero START velocity, the spline solver produces a velocity lobe — the
    // commanded X,Y position curves away from mx/my and returns over the full trajectory
    // duration (peak at T/3 ≈ 3 s for a 9-second Z stroke → ~0.7 mm visible motion).
    //
    // Fix: two-point trajectory.
    //   Point 1 (SETTLE_MS): hold X,Y,Z at their measured positions with velocity=0.
    //     The JTC damps any residual XY velocity during this short window.
    //     Peak displacement during damping ≈ v·T·4/27 ≈ 5 mm/s·0.05 s·0.15 ≈ 0.04 mm.
    //   Point 2 (SETTLE_MS + ms): X,Y still at mx/my, Z moves to gz.
    //     Segment 2 starts from (mx,0) → (mx,0) — perfectly flat spline, no lobe.

    static constexpr int SETTLE_MS = 50;    // ms to damp residual XY velocity
    int ms = z_move_ms(gz);

    double mx, my, mz;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto itx = joint_pos_.find("x_axis_joint");
        auto ity = joint_pos_.find("y_axis_joint");
        auto itz = joint_pos_.find("z_axis_joint");
        mx = (itx != joint_pos_.end()) ? itx->second : static_cast<double>(current_gx_);
        my = (ity != joint_pos_.end()) ? ity->second : static_cast<double>(current_gy_);
        mz = (itz != joint_pos_.end()) ? itz->second : static_cast<double>(current_gz_);
    }

    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = node_->now();
    msg.joint_names  = {"x_axis_joint", "y_axis_joint", "z_axis_joint"};

    // Point 1: velocity-damping settle — brings XY to zero velocity
    trajectory_msgs::msg::JointTrajectoryPoint settle;
    settle.positions  = {mx, my, mz};
    settle.velocities = {0.0, 0.0, 0.0};
    settle.time_from_start.sec     = SETTLE_MS / 1000;
    settle.time_from_start.nanosec = (SETTLE_MS % 1000) * 1'000'000;
    msg.points.push_back(settle);

    // Point 2: actual Z stroke — segment starts from (mx,my,0-vel) so X,Y spline is flat
    int total_ms = SETTLE_MS + ms;
    trajectory_msgs::msg::JointTrajectoryPoint stroke;
    stroke.positions  = {mx, my, static_cast<double>(gz)};
    stroke.velocities = {0.0, 0.0, 0.0};
    stroke.time_from_start.sec     = total_ms / 1000;
    stroke.time_from_start.nanosec = (total_ms % 1000) * 1'000'000;
    msg.points.push_back(stroke);

    gantry_pub_->publish(msg);

    current_gz_ = gz;

    RCLCPP_INFO(node_->get_logger(),
        "Z → %.3f | XY locked at (%.4f, %.4f) | settle=%dms stroke=%dms",
        gz, mx, my, SETTLE_MS, ms);
    interruptible_sleep(total_ms);
}

void ExecutionEngine::move_gantry_to(float gx, float gy)
{
    if (current_gz_ > 0.001f) {
        send_z_command(0.0f);   // retract Z before any XY movement
    }
    send_xy_command(gx, gy);
}

void ExecutionEngine::lower_z(float depth_m)
{
    send_z_command(depth_m);
}

void ExecutionEngine::retract_z()
{
    send_z_command(0.0f);
}

void ExecutionEngine::move_gantry_to_tray()
{
    move_gantry_to(tray_.gx, tray_.gy);  // retracts Z if needed, then XY
    send_z_command(tray_.z_pick);          // lower Z to pick height
}

void ExecutionEngine::wait_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

bool ExecutionEngine::wait_xy_settled(float gx, float gy)
{
    using namespace std::chrono;
    constexpr double TOL_M        = 0.002;  // 2 mm
    constexpr int    STABLE_COUNT = 15;     // 15 × 50 ms = 750 ms stable
    constexpr int    TIMEOUT_MS   = 5000;

    auto deadline = steady_clock::now() + milliseconds(TIMEOUT_MS);
    int  stable   = 0;

    while (steady_clock::now() < deadline) {
        if (stop_requested_.load()) return false;

        double x, y;
        bool   have_data;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto itx = joint_pos_.find("x_axis_joint");
            auto ity = joint_pos_.find("y_axis_joint");
            have_data = (itx != joint_pos_.end() && ity != joint_pos_.end());
            x = have_data ? itx->second : 0.0;
            y = have_data ? ity->second : 0.0;
        }
        if (have_data && std::abs(x - gx) <= TOL_M && std::abs(y - gy) <= TOL_M) {
            if (++stable >= STABLE_COUNT) {
                RCLCPP_INFO(node_->get_logger(),
                    "XY settled: x=%.4f y=%.4f (target %.3f, %.3f)", x, y, gx, gy);
                return true;
            }
        } else {
            stable = 0;
        }
        std::this_thread::sleep_for(milliseconds(50));
    }

    double x_final, y_final;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto itx = joint_pos_.find("x_axis_joint");
        auto ity = joint_pos_.find("y_axis_joint");
        x_final = itx != joint_pos_.end() ? itx->second : -1.0;
        y_final = ity != joint_pos_.end() ? ity->second : -1.0;
    }
    RCLCPP_WARN(node_->get_logger(),
        "XY settle timeout: x=%.4f (tgt=%.3f), y=%.4f (tgt=%.3f)",
        x_final, gx, y_final, gy);
    return false;
}

} // namespace robot_missions
