#include "robot_missions/planting/execution_engine.hpp"
#include <chrono>
#include <cmath>
#include <thread>

namespace robot_missions
{

ExecutionEngine::ExecutionEngine(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ToolBase> tool,
    const std::atomic<bool> * paused,
    DepthSensor *              depth)
: node_(node)
, tool_(tool)
, tray_(tool->get_tray_pose())
, paused_(paused)
, depth_sensor_(depth)
{
    gantry_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);

    base_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/diff_drive_controller/cmd_vel", 10);

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                joint_pos_[msg->name[i]] = msg->position[i];
            }
        });

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/diff_drive_controller/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            odom_x_ = msg->pose.pose.position.x;
            odom_y_ = msg->pose.pose.position.y;
        });

    RCLCPP_INFO(node_->get_logger(), "ExecutionEngine ready.");
}

uint32_t ExecutionEngine::execute(const PlantingPlanner & planner)
{
    uint32_t seeds_planted = 0;
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
                RCLCPP_INFO(node_->get_logger(), "Moving base forward %.3fm (stop %u)",
                    mob.advance_meters, stop);
                move_base_forward(mob.advance_meters);
                wait_until_base_stopped();
            }
        }

        GantryCommand cmd = planner.get_seed_position(seed_index);

        RCLCPP_INFO(node_->get_logger(), "Seed %u: gx=%.3f gy=%.3f depth=%.3fm",
            seed_index, cmd.gx, cmd.gy, cmd.depth);

        move_gantry_to_tray();
        tool_->pick();

        move_gantry_to(cmd.gx, cmd.gy);

        // TODO: replace soil_z with real sensor reading once hardware is mounted.
        // Sensor should return distance from Z-home to soil surface at current XY.
        // Until then, soil_z = 0.0 assumes Z-home is exactly at soil level.
        float soil_z = depth_sensor_ ? depth_sensor_->measure_soil_distance_m() : 0.0f;
        lower_z(soil_z + cmd.depth);

        tool_->release();
        retract_z();

        seeds_planted++;
        RCLCPP_INFO(node_->get_logger(), "Seed %u planted. Total: %u",
            seed_index, seeds_planted);

        while (paused_ && paused_->load() && !stop_requested_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    return seeds_planted;
}

void ExecutionEngine::stop()
{
    stop_requested_ = true;
}

void ExecutionEngine::move_base_forward(float meters)
{
    const float SPEED_MS = 0.1f;
    geometry_msgs::msg::TwistStamped msg;
    msg.header.frame_id = "base_footprint";
    msg.twist.linear.x  = SPEED_MS;
    msg.twist.angular.z = 0.0;

    double start_x = odom_x_;
    if (!wait_base_advanced(start_x, meters)) {
        RCLCPP_WARN(node_->get_logger(),
            "Base forward move of %.3fm timed out.", meters);
    }

    msg.twist.linear.x = 0.0;
    msg.header.stamp   = node_->now();
    base_pub_->publish(msg);
}

bool ExecutionEngine::wait_base_advanced(
    float start_x, float meters, float tol, int timeout_ms)
{
    using namespace std::chrono;
    const float SPEED_MS = 0.1f;
    auto deadline = steady_clock::now() + milliseconds(timeout_ms);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = "base_footprint";
    cmd.twist.linear.x  = SPEED_MS;
    cmd.twist.angular.z = 0.0;

    while (steady_clock::now() < deadline) {
        rclcpp::spin_some(node_);
        double advanced = std::abs(odom_x_ - start_x);
        if (advanced >= static_cast<double>(meters) - tol) return true;
        cmd.header.stamp = node_->now();
        base_pub_->publish(cmd);
        std::this_thread::sleep_for(milliseconds(50));
    }
    return false;
}

void ExecutionEngine::wait_until_base_stopped()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void ExecutionEngine::move_gantry_to(float gx, float gy)
{
    current_gx_ = gx;
    current_gy_ = gy;

    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = node_->now();
    msg.joint_names  = {"x_axis_joint", "y_axis_joint", "z_axis_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions  = {gx, gy, 0.0};
    point.velocities = {0.0, 0.0, 0.0};
    point.time_from_start.sec     = 3;
    point.time_from_start.nanosec = 0;

    msg.points.push_back(point);
    gantry_pub_->publish(msg);

    if (!wait_gantry_reached(gx, gy, 0.0f)) {
        RCLCPP_WARN(node_->get_logger(), "Gantry move to (%.3f, %.3f) timed out.", gx, gy);
    }
}

void ExecutionEngine::lower_z(float depth_m)
{
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = node_->now();
    msg.joint_names  = {"x_axis_joint", "y_axis_joint", "z_axis_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions  = {current_gx_, current_gy_, depth_m};
    point.velocities = {0.0, 0.0, 0.0};
    point.time_from_start.sec     = 2;
    point.time_from_start.nanosec = 0;

    msg.points.push_back(point);
    gantry_pub_->publish(msg);

    if (!wait_gantry_reached(current_gx_, current_gy_, depth_m)) {
        RCLCPP_WARN(node_->get_logger(), "lower_z to %.3fm timed out.", depth_m);
    }
}

void ExecutionEngine::retract_z()
{
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = node_->now();
    msg.joint_names  = {"x_axis_joint", "y_axis_joint", "z_axis_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions  = {current_gx_, current_gy_, 0.0};
    point.velocities = {0.0, 0.0, 0.0};
    point.time_from_start.sec     = 2;
    point.time_from_start.nanosec = 0;

    msg.points.push_back(point);
    gantry_pub_->publish(msg);

    if (!wait_gantry_reached(current_gx_, current_gy_, 0.0f)) {
        RCLCPP_WARN(node_->get_logger(), "retract_z timed out.");
    }
}

void ExecutionEngine::move_gantry_to_tray()
{
    move_gantry_to(tray_.gx, tray_.gy);

    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = node_->now();
    msg.joint_names  = {"x_axis_joint", "y_axis_joint", "z_axis_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions  = {tray_.gx, tray_.gy, tray_.z_pick};
    point.velocities = {0.0, 0.0, 0.0};
    point.time_from_start.sec     = 2;
    point.time_from_start.nanosec = 0;

    msg.points.push_back(point);
    gantry_pub_->publish(msg);

    if (!wait_gantry_reached(tray_.gx, tray_.gy, tray_.z_pick)) {
        RCLCPP_WARN(node_->get_logger(), "move_gantry_to_tray pick height timed out.");
    }
}

bool ExecutionEngine::wait_gantry_reached(
    float gx, float gy, float gz, float tol, int timeout_ms)
{
    using namespace std::chrono;
    auto deadline = steady_clock::now() + milliseconds(timeout_ms);

    while (steady_clock::now() < deadline) {
        rclcpp::spin_some(node_);

        auto px = joint_pos_.find("x_axis_joint");
        auto py = joint_pos_.find("y_axis_joint");
        auto pz = joint_pos_.find("z_axis_joint");

        if (px != joint_pos_.end() &&
            py != joint_pos_.end() &&
            pz != joint_pos_.end())
        {
            if (std::abs(px->second - gx) < tol &&
                std::abs(py->second - gy) < tol &&
                std::abs(pz->second - gz) < tol)
            {
                return true;
            }
        }
        std::this_thread::sleep_for(milliseconds(20));
    }
    return false;
}

void ExecutionEngine::wait_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace robot_missions
