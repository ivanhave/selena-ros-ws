#include <memory>
#include <thread>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_missions/mission_manager.hpp"
#include "robot_missions/mission_base.hpp"
#include "robot_missions/action/mission_action.hpp"

// Force-include planting registration so the static initializer fires
// and PlantingMission self-registers with MissionRegistry.
// Without this the linker may dead-strip the translation unit.
#include "robot_missions/planting/planting_mission.hpp"

using MissionAction = robot_missions::action::MissionAction;
using GoalHandle    = rclcpp_action::ServerGoalHandle<MissionAction>;

// ── MissionServer ─────────────────────────────────────────────────────────────

class MissionServer
{
public:
    explicit MissionServer(rclcpp::Node::SharedPtr node)
    : node_(node)
    , manager_(node)
    {
        action_server_ = rclcpp_action::create_server<MissionAction>(
            node_,
            "mission",
            std::bind(&MissionServer::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MissionServer::handle_cancel,   this, std::placeholders::_1),
            std::bind(&MissionServer::handle_accepted, this, std::placeholders::_1)
        );

        // Feedback timer at 2 Hz — publishes progress while a mission is running
        feedback_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MissionServer::send_feedback, this));

        RCLCPP_INFO(node_->get_logger(), "Mission action server ready on 'mission'.");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & /*uuid*/,
        std::shared_ptr<const MissionAction::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received goal: type=%s target=%s",
            goal->mission_type.c_str(), goal->target.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> /*goal_handle*/)
    {
        RCLCPP_INFO(node_->get_logger(), "Cancel requested — aborting mission.");
        manager_.abort();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread([this, goal_handle]() {
            execute_mission(goal_handle);
        }).detach();
    }

    void execute_mission(const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();

        robot_missions::MissionCommand cmd;
        cmd.mission_type  = goal->mission_type;
        cmd.target        = goal->target;
        cmd.quantity      = goal->quantity;
        cmd.quantity_unit = goal->quantity_unit;
        cmd.start_x       = goal->start_x;
        cmd.start_y       = goal->start_y;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            active_goal_ = goal_handle;
        }

        robot_missions::MissionResult result = manager_.run(cmd);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            active_goal_.reset();
        }

        auto action_result = std::make_shared<MissionAction::Result>();
        action_result->success          = result.success;
        action_result->report           = result.report;
        action_result->items_completed  = result.items_completed;
        action_result->distance_covered = result.distance_covered_m;

        if (goal_handle->is_active()) {
            if (result.success) {
                goal_handle->succeed(action_result);
            } else {
                goal_handle->abort(action_result);
            }
        }
    }

    void send_feedback()
    {
        std::shared_ptr<GoalHandle> goal_handle;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            goal_handle = active_goal_;
        }
        if (!goal_handle || !goal_handle->is_active()) return;

        auto feedback = std::make_shared<MissionAction::Feedback>();
        feedback->progress_percent       = manager_.get_progress();
        feedback->current_step           = manager_.get_current_step();
        feedback->items_completed_so_far = 0;
        feedback->current_robot_x        = 0.0f;
        feedback->current_robot_y        = 0.0f;

        goal_handle->publish_feedback(feedback);
    }

    rclcpp::Node::SharedPtr                         node_;
    robot_missions::MissionManager                  manager_;
    rclcpp_action::Server<MissionAction>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr                    feedback_timer_;
    std::shared_ptr<GoalHandle>                     active_goal_;
    std::mutex                                      mutex_;
};

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mission_server");
    MissionServer server(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
