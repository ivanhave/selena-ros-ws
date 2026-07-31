#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "urdf/model.h"

#include "robot_gantry/action/gantry_move.hpp"
#include "robot_gantry/srv/gantry_set_mode.hpp"
#include "robot_gantry/gantry_constants.hpp"

static constexpr int SERVICE_WAIT_S = 2;
static constexpr int GOAL_WAIT_S    = 5;
static constexpr int STEP_MS        = 50;

using GantryMove    = robot_gantry::action::GantryMove;
using GantrySetMode = robot_gantry::srv::GantrySetMode;
using FollowJT      = control_msgs::action::FollowJointTrajectory;
using SwitchCtrl    = controller_manager_msgs::srv::SwitchController;
static const std::vector<std::string> JOINT_NAMES = {
    "x_axis_joint", "y_axis_joint", "z_axis_joint"};
static const char * JTC_NAME  = "joint_trajectory_controller";
static const char * VEL_NAME  = "gantry_velocity_controller";

enum class State { POSITION_IDLE, POSITION_MOVING, VELOCITY };

class GantryNode : public rclcpp::Node
{
public:
    GantryNode()
    : Node("gantry_node"),
      state_(State::POSITION_IDLE),
      cx_(0.0), cy_(0.0), cz_(0.0),
      x_lower_(0.0), x_upper_(0.0),
      y_lower_(0.0), y_upper_(0.0),
      z_lower_(0.0), z_upper_(0.0)
    {
        cb_group_ = create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        // Read joint limits from robot_description URDF
        declare_parameter("robot_description", std::string(""));
        load_urdf_limits();

        // Subscribers
        js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(js_mtx_);
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    if (msg->name[i] == "x_axis_joint") cx_ = msg->position[i];
                    else if (msg->name[i] == "y_axis_joint") cy_ = msg->position[i];
                    else if (msg->name[i] == "z_axis_joint") cz_ = msg->position[i];
                }
            }, sub_opts);

        cmd_vel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gantry/cmd_vel", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (state_.load() == State::VELOCITY)
                    vel_pub_->publish(*msg);
            }, sub_opts);

        // Publishers
        vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gantry_velocity_controller/commands", 10);

        // Action server
        move_server_ = rclcpp_action::create_server<GantryMove>(
            this, "/gantry/move",
            [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const GantryMove::Goal>) {
                if (state_.load() != State::POSITION_IDLE) {
                    RCLCPP_WARN(get_logger(),
                        "Gantry busy (state=%d) — goal ignored", static_cast<int>(state_.load()));
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GantryMove>>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<GantryMove>> gh) {
                std::thread([this, gh]() { execute(gh); }).detach();
            },
            rcl_action_server_get_default_options(),
            cb_group_);

        // Service server
        mode_srv_ = create_service<GantrySetMode>(
            "/gantry/set_mode",
            [this](const std::shared_ptr<GantrySetMode::Request> req,
                   std::shared_ptr<GantrySetMode::Response> res) {
                handle_set_mode(req, res);
            },
            rclcpp::ServicesQoS(),
            cb_group_);

        // Action client for JTC
        jtc_client_ = rclcpp_action::create_client<FollowJT>(
            this, "/joint_trajectory_controller/follow_joint_trajectory",
            cb_group_);

        // Service client for controller_manager
        switch_client_ = create_client<SwitchCtrl>(
            "/controller_manager/switch_controller",
            rclcpp::ServicesQoS(),
            cb_group_);

        RCLCPP_INFO(get_logger(),
            "GantryNode ready. Limits: X [%.3f, %.3f]  Y [%.3f, %.3f]  Z [%.3f, %.3f]",
            x_lower_, x_upper_, y_lower_, y_upper_, z_lower_, z_upper_);
    }

private:
    // ── State ──────────────────────────────────────────────────────────────────
    std::atomic<State> state_;
    std::optional<bool> pending_mode_;   // mode switch requested while busy
    std::mutex js_mtx_;
    double cx_, cy_, cz_;

    // Joint limits (from URDF)
    double x_lower_, x_upper_, y_lower_, y_upper_, z_lower_, z_upper_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr          js_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr      cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr         vel_pub_;
    rclcpp_action::Server<GantryMove>::SharedPtr                           move_server_;
    rclcpp::Service<GantrySetMode>::SharedPtr                              mode_srv_;
    rclcpp_action::Client<FollowJT>::SharedPtr                             jtc_client_;
    rclcpp::Client<SwitchCtrl>::SharedPtr                                  switch_client_;

    // ── URDF limits ────────────────────────────────────────────────────────────
    void load_urdf_limits()
    {
        std::string urdf_str = get_parameter("robot_description").as_string();
        if (urdf_str.empty())
            throw std::runtime_error("robot_description parameter is empty — cannot read joint limits");

        urdf::Model model;
        if (!model.initString(urdf_str))
            throw std::runtime_error("Failed to parse robot_description URDF");

        auto read = [&](const std::string & name, double & lo, double & hi) {
            auto j = model.getJoint(name);
            if (!j || !j->limits)
                throw std::runtime_error("Joint '" + name + "' not found in URDF or has no limits");
            lo = j->limits->lower;
            hi = j->limits->upper;
        };
        read("x_axis_joint", x_lower_, x_upper_);
        read("y_axis_joint", y_lower_, y_upper_);
        read("z_axis_joint", z_lower_, z_upper_);
    }

    // ── Joint state helpers ────────────────────────────────────────────────────
    void get_joints(double & x, double & y, double & z) const
    {
        std::lock_guard<std::mutex> lk(const_cast<std::mutex &>(js_mtx_));
        x = cx_; y = cy_; z = cz_;
    }

    // ── Timing ────────────────────────────────────────────────────────────────
    static int xy_ms(double dx, double dy)
    {
        using namespace gantry_constants;
        double t = std::max({SPLINE_FACTOR * dx / X_VEL_CAP,
                             SPLINE_FACTOR * dy / Y_VEL_CAP,
                             MIN_MOVE_S});
        return static_cast<int>((t + MOVE_MARGIN_S) * 1000.0);
    }

    static int z_ms(double dz)
    {
        using namespace gantry_constants;
        double t = std::max(SPLINE_FACTOR * dz / Z_VEL_CAP, MIN_MOVE_S);
        return static_cast<int>((t + MOVE_MARGIN_S) * 1000.0);
    }

    // ── FollowJointTrajectory helpers ─────────────────────────────────────────
    // Single-point XY move, Z held at 0.
    bool send_jtc_xy(double tx, double ty,
                     std::shared_ptr<rclcpp_action::ServerGoalHandle<GantryMove>> gh)
    {
        double cx, cy, cz; get_joints(cx, cy, cz);
        int ms = xy_ms(std::abs(tx - cx), std::abs(ty - cy));

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions  = {tx, ty, gantry_constants::HOME_Z};
        pt.velocities = {0.0, 0.0, 0.0};
        pt.time_from_start.sec     = ms / 1000;
        pt.time_from_start.nanosec = (ms % 1000) * 1'000'000;

        return send_jtc({pt}, gh);
    }

    // Two-point Z move: SETTLE_MS damp phase then stroke, X/Y held at current.
    bool send_jtc_z(double tz,
                    std::shared_ptr<rclcpp_action::ServerGoalHandle<GantryMove>> gh)
    {
        using namespace gantry_constants;
        double cx, cy, cz; get_joints(cx, cy, cz);
        int stroke = z_ms(std::abs(tz - cz));
        int total  = SETTLE_MS + stroke;

        trajectory_msgs::msg::JointTrajectoryPoint p1, p2;
        p1.positions  = {cx, cy, cz};
        p1.velocities = {0.0, 0.0, 0.0};
        p1.time_from_start.sec     = SETTLE_MS / 1000;
        p1.time_from_start.nanosec = (SETTLE_MS % 1000) * 1'000'000;

        p2.positions  = {cx, cy, tz};
        p2.velocities = {0.0, 0.0, 0.0};
        p2.time_from_start.sec     = total / 1000;
        p2.time_from_start.nanosec = (total % 1000) * 1'000'000;

        return send_jtc({p1, p2}, gh);
    }

    // Send a FollowJointTrajectory goal; block until done or cancelled.
    bool send_jtc(
        const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> & points,
        std::shared_ptr<rclcpp_action::ServerGoalHandle<GantryMove>> gh)
    {
        using namespace gantry_constants;

        if (!jtc_client_->wait_for_action_server(
                std::chrono::seconds(SERVICE_WAIT_S))) {
            RCLCPP_ERROR(get_logger(), "JTC action server not available");
            return false;
        }

        FollowJT::Goal goal;
        goal.trajectory.joint_names = JOINT_NAMES;
        goal.trajectory.points      = points;

        auto gh_future = jtc_client_->async_send_goal(goal);
        if (gh_future.wait_for(std::chrono::seconds(GOAL_WAIT_S))
            != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "JTC goal acceptance timed out");
            return false;
        }
        auto jtc_gh = gh_future.get();
        if (!jtc_gh) {
            RCLCPP_ERROR(get_logger(), "JTC rejected goal");
            return false;
        }

        auto result_future = jtc_client_->async_get_result(jtc_gh);

        while (result_future.wait_for(std::chrono::milliseconds(STEP_MS))
               != std::future_status::ready)
        {
            if (gh->is_canceling()) {
                jtc_client_->async_cancel_goal(jtc_gh);
                result_future.wait();
                return false;
            }
        }

        auto result = result_future.get();
        return result.code == rclcpp_action::ResultCode::SUCCEEDED;
    }

    // ── GantryMove execute ────────────────────────────────────────────────────
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<GantryMove>> gh)
    {
        using namespace gantry_constants;
        const auto & goal = *gh->get_goal();
        auto result = std::make_shared<GantryMove::Result>();
        auto fb     = std::make_shared<GantryMove::Feedback>();

        // Bounds check
        if (goal.x < x_lower_ || goal.x > x_upper_ ||
            goal.y < y_lower_ || goal.y > y_upper_ ||
            goal.z < z_lower_ || goal.z > z_upper_)
        {
            result->success = false;
            result->message = "Target out of joint limits";
            gh->abort(result);
            state_.store(State::POSITION_IDLE);
            return;
        }

        state_.store(State::POSITION_MOVING);
        RCLCPP_INFO(get_logger(), "GantryMove: (%.3f, %.3f, %.3f)", goal.x, goal.y, goal.z);

        auto publish_fb = [&](const std::string & phase) {
            double x, y, z; get_joints(x, y, z);
            fb->phase     = phase;
            fb->current_x = x;
            fb->current_y = y;
            fb->current_z = z;
            gh->publish_feedback(fb);
        };

        auto finish = [&](bool success, const std::string & msg) {
            double x, y, z; get_joints(x, y, z);
            result->success = success;
            result->message = msg;
            result->final_x = x;
            result->final_y = y;
            result->final_z = z;
            if (success) gh->succeed(result);
            else         gh->abort(result);
            state_.store(State::POSITION_IDLE);
            apply_pending_mode();
        };

        // Step 1: retract Z if not already at 0
        {
            double cx, cy, cz; get_joints(cx, cy, cz);
            if (cz > TOL_M) {
                publish_fb("z_retract");
                if (!send_jtc_z(gantry_constants::HOME_Z, gh)) { finish(false, "Z retract failed or cancelled"); return; }
            }
        }

        if (gh->is_canceling()) { finish(false, "Cancelled"); return; }

        // Step 2: XY move (skip if already at target)
        {
            double cx, cy, cz; get_joints(cx, cy, cz);
            if (std::abs(goal.x - cx) > TOL_M || std::abs(goal.y - cy) > TOL_M) {
                publish_fb("xy_move");
                if (!send_jtc_xy(goal.x, goal.y, gh)) { finish(false, "XY move failed or cancelled"); return; }
            }
        }

        if (gh->is_canceling()) { finish(false, "Cancelled"); return; }

        // Step 3: lower Z to target (skip if already there)
        {
            double cx, cy, cz; get_joints(cx, cy, cz);
            if (std::abs(goal.z - cz) > TOL_M) {
                publish_fb("z_lower");
                if (!send_jtc_z(goal.z, gh)) { finish(false, "Z lower failed or cancelled"); return; }
            }
        }

        finish(true, "OK");
    }

    // ── Mode switch ───────────────────────────────────────────────────────────
    void handle_set_mode(const std::shared_ptr<GantrySetMode::Request>  req,
                               std::shared_ptr<GantrySetMode::Response> res)
    {
        State cur = state_.load();
        bool to_vel = req->velocity_mode;

        // Already in requested mode
        if ((to_vel && cur == State::VELOCITY) ||
            (!to_vel && cur != State::VELOCITY))
        {
            res->success = true;
            res->message = "Already in requested mode";
            return;
        }

        if (req->urgent) {
            // Cancel in-flight move if any — the execute() loop will see is_canceling()
            // and exit, then apply_pending_mode() would race. Instead do it directly here.
            do_switch(to_vel);
            res->success = true;
            res->message = "Mode switched (urgent)";
        } else if (cur == State::POSITION_MOVING) {
            pending_mode_ = to_vel;
            res->success  = true;
            res->message  = "Mode switch queued — will apply after current move";
        } else {
            do_switch(to_vel);
            res->success = true;
            res->message = "Mode switched";
        }
    }

    void do_switch(bool to_velocity)
    {
        if (!switch_client_->wait_for_service(
                std::chrono::seconds(SERVICE_WAIT_S))) {
            RCLCPP_ERROR(get_logger(), "switch_controller service not available");
            return;
        }

        auto req = std::make_shared<SwitchCtrl::Request>();
        req->strictness = SwitchCtrl::Request::STRICT;

        if (to_velocity) {
            req->activate_controllers   = {VEL_NAME};
            req->deactivate_controllers = {JTC_NAME};
        } else {
            req->activate_controllers   = {JTC_NAME};
            req->deactivate_controllers = {VEL_NAME};
        }

        auto future = switch_client_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(GOAL_WAIT_S))
            == std::future_status::ready && future.get()->ok) {
            state_.store(to_velocity ? State::VELOCITY : State::POSITION_IDLE);
            RCLCPP_INFO(get_logger(), "Mode switched to %s",
                to_velocity ? "VELOCITY" : "POSITION");
        } else {
            RCLCPP_ERROR(get_logger(), "switch_controller failed");
        }
    }

    void apply_pending_mode()
    {
        if (!pending_mode_.has_value()) return;
        bool to_vel = *pending_mode_;
        pending_mode_.reset();
        do_switch(to_vel);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GantryNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
