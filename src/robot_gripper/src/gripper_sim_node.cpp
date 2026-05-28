#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robot_gripper
{

static constexpr int    ANGLE_MIN     = 0;
static constexpr int    ANGLE_MAX     = 55;
static constexpr double FINGER_STROKE = 0.020;

class GripperSimNode : public rclcpp::Node
{
public:
    GripperSimNode() : Node("gripper_sim_node")
    {
        angle_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/gripper/angle", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                handle_angle(msg->data);
            });

        angle_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/gripper/current_angle", 10);
        state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/gripper/is_open", 10);

        // Commands finger joints via gripper_controller (ForwardCommandController).
        // joint_state_broadcaster owns the /joint_states publishing for these joints.
        cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gripper_controller/commands", 10);

        RCLCPP_INFO(this->get_logger(),
            "Gripper sim ready (no CAN). Listening on /gripper/angle");
    }

private:
    void handle_angle(int angle)
    {
        current_angle_ = std::clamp(angle, ANGLE_MIN, ANGLE_MAX);

        double pos = (static_cast<double>(current_angle_) / ANGLE_MAX) * FINGER_STROKE;

        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = {pos, pos};
        cmd_pub_->publish(cmd);

        std_msgs::msg::Int32 a;
        a.data = current_angle_;
        angle_pub_->publish(a);

        std_msgs::msg::Bool s;
        s.data = (current_angle_ == ANGLE_MAX);
        state_pub_->publish(s);

        RCLCPP_INFO(this->get_logger(), "Gripper -> %d deg (%.3f m) (%s)",
            current_angle_, pos, s.data ? "OPEN" : "CLOSED");
    }

    int current_angle_ = ANGLE_MIN;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr              angle_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                 angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                  state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     cmd_pub_;
};

} // namespace robot_gripper

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_gripper::GripperSimNode>());
    rclcpp::shutdown();
    return 0;
}
