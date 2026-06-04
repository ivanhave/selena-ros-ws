#include "robot_navigation/odometry_navigator.hpp"
#include <chrono>
#include <cmath>
#include <thread>

namespace robot_navigation
{

OdometryNavigator::OdometryNavigator(rclcpp::Node::SharedPtr node)
: node_(node)
{
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/diff_drive_controller/cmd_vel", 10);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/diff_drive_controller/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            float qx = msg->pose.pose.orientation.x;
            float qy = msg->pose.pose.orientation.y;
            float qz = msg->pose.pose.orientation.z;
            float qw = msg->pose.pose.orientation.w;
            float yaw = std::atan2(2.0f * (qw * qz + qx * qy),
                                   1.0f - 2.0f * (qy * qy + qz * qz));
            std::lock_guard<std::mutex> lock(mutex_);
            odom_x_   = static_cast<float>(msg->pose.pose.position.x);
            odom_y_   = static_cast<float>(msg->pose.pose.position.y);
            odom_yaw_ = yaw;
            odom_vx_  = static_cast<float>(msg->twist.twist.linear.x);
            odom_vz_  = static_cast<float>(msg->twist.twist.angular.z);
        });

    RCLCPP_INFO(node_->get_logger(), "OdometryNavigator ready.");
}

Pose2D OdometryNavigator::get_pose()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return {odom_x_, odom_y_, odom_yaw_};
}

void OdometryNavigator::abort() { abort_flag_.store(true); publish_velocity(0.0f, 0.0f); }
void OdometryNavigator::reset_abort() { abort_flag_.store(false); }

// ── rotate_to ────────────────────────────────────────────────────────────────

bool OdometryNavigator::rotate_to(float target_yaw, float angle_tol)
{
    using namespace std::chrono;
    auto deadline = steady_clock::now() + milliseconds(ROTATE_TIMEOUT_MS);

    while (steady_clock::now() < deadline) {
        if (abort_flag_.load()) break;
        float cur_yaw;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cur_yaw = odom_yaw_;
        }

        float diff = std::atan2(std::sin(target_yaw - cur_yaw),
                                std::cos(target_yaw - cur_yaw));

        if (std::fabs(diff) < angle_tol) {
            float vz;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                vz = std::fabs(odom_vz_);
            }
            if (vz < 0.01f) break;
        }

        float wz = std::clamp(ROTATE_KP * diff, -ROTATE_SPEED_RADPS, ROTATE_SPEED_RADPS);
        publish_velocity(0.0f, wz);
        std::this_thread::sleep_for(milliseconds(50));
    }

    publish_velocity(0.0f, 0.0f);

    auto stop_dl = steady_clock::now() + milliseconds(STOP_TIMEOUT_MS);
    while (steady_clock::now() < stop_dl) {
        float vz;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vz = std::fabs(odom_vz_);
        }
        if (vz < 0.01f) return true;
        std::this_thread::sleep_for(milliseconds(50));
    }

    RCLCPP_WARN(node_->get_logger(), "OdometryNavigator: rotate_to settle timeout.");
    return true;
}

// ── align_to_yaw ──────────────────────────────────────────────────────────────

bool OdometryNavigator::align_to_yaw(float target_yaw)
{
    float cur_yaw;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cur_yaw = odom_yaw_;
    }
    float diff = std::atan2(std::sin(target_yaw - cur_yaw),
                            std::cos(target_yaw - cur_yaw));
    RCLCPP_INFO(node_->get_logger(),
        "OdometryNavigator: pre-plant yaw align — current %.3f rad, "
        "target %.3f rad, error %.3f rad (%.1f°).",
        cur_yaw, target_yaw, diff, diff * 180.0f / static_cast<float>(M_PI));
    return rotate_to(target_yaw, PLANT_ALIGN_TOL);
}

// ── move_to ──────────────────────────────────────────────────────────────────

bool OdometryNavigator::move_to(Pose2D target)
{
    using namespace std::chrono;

    float start_x, start_y, start_yaw;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        start_x   = odom_x_;
        start_y   = odom_y_;
        start_yaw = odom_yaw_;
    }

    float dx = target.x - start_x;
    float dy = target.y - start_y;
    float target_dist = std::sqrt(dx * dx + dy * dy);

    if (target_dist < ARRIVAL_TOL_M) {
        RCLCPP_INFO(node_->get_logger(),
            "OdometryNavigator: already at target (dist=%.4fm).", target_dist);
        rotate_to(target.yaw);
        return true;
    }

    // Direction from current position to target
    float dir_to_target = std::atan2(dy, dx);
    float angle_diff = std::atan2(std::sin(dir_to_target - start_yaw),
                                   std::cos(dir_to_target - start_yaw));

    // If target is more than 90° behind the robot, drive backward.
    // This allows linear strip exit without rotating (safety constraint).
    bool drive_backward = (std::fabs(angle_diff) > static_cast<float>(M_PI) / 2.0f);

    if (drive_backward) {
        RCLCPP_INFO(node_->get_logger(),
            "OdometryNavigator: driving backward %.3fm to (%.3f, %.3f).",
            target_dist, target.x, target.y);
    } else {
        // Rotate to face target first
        rotate_to(dir_to_target);
        RCLCPP_INFO(node_->get_logger(),
            "OdometryNavigator: driving forward %.3fm to (%.3f, %.3f).",
            target_dist, target.x, target.y);
    }

    // ── Drive with proportional decel and live pure-pursuit heading correction ──
    // "remaining" is measured directly to the target on every tick — correct even
    // when the robot curves, unlike the old (target_dist - advanced) approximation
    // which caused early stops whenever heading correction introduced any arc.
    auto deadline = steady_clock::now() + milliseconds(MOVE_TIMEOUT_MS);

    while (steady_clock::now() < deadline) {
        if (abort_flag_.load()) break;
        float cx, cy, cur_yaw;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cx      = odom_x_;
            cy      = odom_y_;
            cur_yaw = odom_yaw_;
        }

        float remaining = std::sqrt((target.x - cx) * (target.x - cx) +
                                    (target.y - cy) * (target.y - cy));
        if (remaining <= ARRIVAL_TOL_M) break;

        // Pure-pursuit: repoint toward target each tick
        float dir_now = std::atan2(target.y - cy, target.x - cx);
        float desired_yaw = drive_backward
            ? std::atan2(std::sin(dir_now + static_cast<float>(M_PI)),
                         std::cos(dir_now + static_cast<float>(M_PI)))
            : dir_now;

        float heading_err = std::atan2(std::sin(desired_yaw - cur_yaw),
                                       std::cos(desired_yaw - cur_yaw));
        float wz = std::clamp(TRACK_KP * heading_err, -MAX_TRACK_WZ, MAX_TRACK_WZ);

        float scale = std::min(1.0f, remaining / DECEL_DIST_M);
        float spd   = SPEED_MPS * std::max(scale, MIN_SPEED_FRACTION);
        publish_velocity(drive_backward ? -spd : spd, wz);
        std::this_thread::sleep_for(milliseconds(50));
    }

    // ── Stop ────────────────────────────────────────────────────────────────
    publish_velocity(0.0f, 0.0f);

    auto stop_deadline = steady_clock::now() + milliseconds(STOP_TIMEOUT_MS);
    while (steady_clock::now() < stop_deadline) {
        float vx;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vx = std::fabs(odom_vx_);
        }
        if (vx < STOPPED_VEL_MPS) break;
        std::this_thread::sleep_for(milliseconds(50));
    }

    // ── Final heading alignment ──────────────────────────────────────────────
    rotate_to(target.yaw);

    RCLCPP_INFO(node_->get_logger(), "OdometryNavigator: reached (%.3f, %.3f).",
        target.x, target.y);
    return true;
}

void OdometryNavigator::publish_velocity(float vx, float wz)
{
    geometry_msgs::msg::TwistStamped msg;
    msg.header.frame_id  = "base_footprint";
    msg.header.stamp     = node_->now();
    msg.twist.linear.x   = vx;
    msg.twist.angular.z  = wz;
    cmd_vel_pub_->publish(msg);
}

} // namespace robot_navigation
