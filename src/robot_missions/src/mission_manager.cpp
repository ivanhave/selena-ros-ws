#include "robot_missions/mission_manager.hpp"
#include "robot_missions/mission_registry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_navigation/field_path_planner.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <regex>
#include <fstream>
#include <sstream>

namespace robot_missions
{

MissionManager::MissionManager(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<robot_navigation::NavigationProvider> nav)
: node_(node)
, nav_(nav)
{
    rclcpp::QoS path_qos(1);
    path_qos.transient_local();
    planned_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/mission_planned_path", path_qos);

    load_field_config();
    load_headland_clearance();

    RCLCPP_INFO(node_->get_logger(), "MissionManager ready.");
}

MissionResult MissionManager::run(const MissionCommand & cmd)
{
    nav_->reset_abort();
    RCLCPP_INFO(node_->get_logger(),
        "Received command: type=%s target=%s quantity=%.2f %s start=(%.2f,%.2f,%.2f rad)",
        cmd.mission_type.c_str(), cmd.target.c_str(),
        cmd.quantity, cmd.quantity_unit.c_str(),
        cmd.start_x, cmd.start_y, cmd.start_yaw);

    // ── Step 1: create / validate / plan (pure math, before moving) ──────────
    current_mission_ = MissionRegistry::instance().create(cmd.mission_type, node_, cmd, nav_);
    if (!current_mission_) {
        MissionResult fail;
        fail.success = false;
        fail.report  = "Unknown mission type: " + cmd.mission_type;
        return fail;
    }

    if (!current_mission_->validate()) {
        MissionResult fail;
        fail.success = false;
        fail.report  = "Mission validation failed.";
        return fail;
    }

    if (!current_mission_->plan()) {
        MissionResult fail;
        fail.success = false;
        fail.report  = "Mission planning failed.";
        return fail;
    }

    // ── Step 2: compute full planned path and publish before any movement ─────
    // Nav waypoints: current pose → (headland) → zone start
    robot_navigation::Pose2D zone_start { cmd.start_x, cmd.start_y, cmd.start_yaw };
    robot_navigation::Pose2D robot_start = nav_->get_pose();
    std::vector<robot_navigation::Pose2D> nav_wps;
    if (field_config_loaded_) {
        nav_wps = robot_navigation::plan_field_path(
            robot_start, zone_start, field_config_, headland_clearance_);
        RCLCPP_INFO(node_->get_logger(), "Nav path: %zu waypoint(s).", nav_wps.size());
    } else {
        nav_wps = { zone_start };
    }

    // Mission stops: zone start → zone end (all robot base positions during planting)
    auto mission_stops = current_mission_->get_route_waypoints(
        cmd.start_x, cmd.start_y, cmd.start_yaw);

    // Full path: robot's current position → nav waypoints → all planting stops.
    // Starting from the robot's actual pose makes the nav route visible on the map.
    std::vector<robot_navigation::Pose2D> full_path = { robot_start };
    full_path.insert(full_path.end(), nav_wps.begin(), nav_wps.end());
    for (size_t i = 1; i < mission_stops.size(); i++) {
        full_path.push_back(mission_stops[i]);
    }
    publish_planned_path(full_path);

    // ── Step 3: navigate to zone start ───────────────────────────────────────
    RCLCPP_INFO(node_->get_logger(),
        "Navigating to start (%.2f, %.2f, yaw=%.2f rad).", cmd.start_x, cmd.start_y, cmd.start_yaw);
    for (const auto & wp : nav_wps) {
        if (!nav_->move_to(wp)) {
            MissionResult fail;
            fail.success = false;
            fail.report  = "Failed to navigate to start position.";
            return fail;
        }
    }

    // ── Step 4: align to strip yaw before executing ──────────────────────────
    // After navigation the robot should be within ANGLE_TOL_RAD of cmd.start_yaw.
    // This step refines to PLANT_ALIGN_TOL (0.02 rad / ~1°) so the gantry axes
    // are parallel to the strip axes before planting begins.
    nav_->align_to_yaw(cmd.start_yaw);

    // ── Step 5: execute ───────────────────────────────────────────────────────
    current_mission_->execute();

    return current_mission_->report();
}

void MissionManager::pause()
{
    if (current_mission_) current_mission_->pause();
}

void MissionManager::resume()
{
    if (current_mission_) current_mission_->resume();
}

void MissionManager::abort()
{
    nav_->abort();
    if (current_mission_) current_mission_->abort();
}

float MissionManager::get_progress()
{
    if (current_mission_) return current_mission_->get_progress();
    return 0.0f;
}

std::string MissionManager::get_current_step()
{
    if (current_mission_) return current_mission_->get_current_step();
    return "idle";
}

robot_navigation::Pose2D MissionManager::get_robot_pose()
{
    return nav_->get_pose();
}

void MissionManager::load_field_config()
{
    try {
        auto pkg = ament_index_cpp::get_package_share_directory("robot_field");
        std::string path = pkg + "/config/field_config.yaml";
        YAML::Node root = YAML::LoadFile(path);
        auto f = root["field"];
        field_config_.origin_x      = f["origin"]["x"].as<float>(0.0f);
        field_config_.origin_y      = f["origin"]["y"].as<float>(0.0f);
        field_config_.origin_yaw    = f["origin"]["yaw"].as<float>(0.0f);
        field_config_.strip_width   = f["strip_width"].as<float>(0.845f);
        field_config_.strip_length  = f["strip_length"].as<float>(10.0f);
        field_config_.strip_spacing = f["strip_spacing"].as<float>(1.0f);
        field_config_loaded_ = true;
        RCLCPP_INFO(node_->get_logger(),
            "FieldConfig loaded: origin=(%.2f,%.2f,%.2f) spacing=%.2f length=%.2f",
            field_config_.origin_x, field_config_.origin_y, field_config_.origin_yaw,
            field_config_.strip_spacing, field_config_.strip_length);
    } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
            "Could not load field_config.yaml: %s — path planning disabled.", e.what());
    }
}

void MissionManager::load_headland_clearance()
{
    try {
        auto pkg = ament_index_cpp::get_package_share_directory("robot_description");
        std::string xacro_path = pkg + "/urdf/mobile_base.xacro";
        std::ifstream f(xacro_path);
        std::stringstream buf;
        buf << f.rdbuf();
        std::string content = buf.str();

        auto getProp = [&](const std::string & name) -> float {
            std::regex re("<xacro:property\\s+name=\"" + name + "\"\\s+value=\"([^\"]+)\"");
            std::smatch m;
            if (std::regex_search(content, m, re)) {
                try { return std::stof(m[1]); } catch (...) {}
            }
            return 0.0f;
        };

        float front_x = getProp("front_wheel_offset_x");
        float back_x  = getProp("back_wheel_offset_x");
        float whl_r   = getProp("wheel_radius");
        float computed = std::max(front_x, back_x) + whl_r + 0.5f;
        if (computed > 0.5f) {
            headland_clearance_ = computed;
            RCLCPP_INFO(node_->get_logger(),
                "Headland clearance: %.2f m (front=%.3f back=%.3f whl_r=%.3f + 0.5)",
                headland_clearance_, front_x, back_x, whl_r);
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "URDF parse gave zero values — headland clearance stays %.2f m", headland_clearance_);
        }
    } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
            "Could not parse mobile_base.xacro: %s — using %.2f m headland.", e.what(), headland_clearance_);
    }
}

void MissionManager::publish_planned_path(const std::vector<robot_navigation::Pose2D> & waypoints)
{
    nav_msgs::msg::Path msg;
    msg.header.stamp    = node_->now();
    msg.header.frame_id = "odom";
    for (const auto & wp : waypoints) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header            = msg.header;
        ps.pose.position.x   = wp.x;
        ps.pose.position.y   = wp.y;
        ps.pose.position.z   = 0.0;
        ps.pose.orientation.x = 0.0;
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = std::sin(wp.yaw / 2.0f);
        ps.pose.orientation.w = std::cos(wp.yaw / 2.0f);
        msg.poses.push_back(ps);
    }
    planned_path_pub_->publish(msg);
}

} // namespace robot_missions
