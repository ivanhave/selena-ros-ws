#include "robot_gripper/gripper_interface.hpp"

namespace robot_gripper
{

GripperInterface::GripperInterface()
: Node("gripper_interface_node")
{
    // Open CAN
    if (!driver_.open()) {
        RCLCPP_ERROR(this->get_logger(),
            "Failed to open CAN interface. Check wiring and can0.");
        return;
    }

    // Subscriber — angle commands
    angle_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/gripper/angle", 10,
        std::bind(&GripperInterface::angle_callback, this,
                  std::placeholders::_1));

    // Publisher — current angle
    angle_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "/gripper/current_angle", 10);

    // Publisher — open/closed state
    state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/gripper/is_open", 10);

    RCLCPP_INFO(this->get_logger(),
        "Gripper Interface ready. Listening on /gripper/angle");
}

GripperInterface::~GripperInterface()
{
    driver_.close();
}

void GripperInterface::angle_callback(
    const std_msgs::msg::Int32::SharedPtr msg)
{
    int requested_angle = msg->data;

    RCLCPP_INFO(this->get_logger(),
        "Received angle command: %d degrees", requested_angle);

    bool success = driver_.set_angle(requested_angle);

    if (success) {
        // Publish confirmed angle
        std_msgs::msg::Int32 angle_msg;
        angle_msg.data = driver_.get_angle();
        angle_pub_->publish(angle_msg);

        // Publish open/closed state
        // Convention: angle == ANGLE_MIN → closed, angle == ANGLE_MAX → open
        std_msgs::msg::Bool state_msg;
        state_msg.data = (driver_.get_angle() == ANGLE_MAX);
        state_pub_->publish(state_msg);

        RCLCPP_INFO(this->get_logger(),
            "Gripper moved to %d degrees. State: %s",
            driver_.get_angle(),
            state_msg.data ? "OPEN" : "CLOSED");
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Gripper command failed — no confirmation from Arduino.");
    }
}

} // namespace robot_gripper

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_gripper::GripperInterface>());
    rclcpp::shutdown();
    return 0;
}