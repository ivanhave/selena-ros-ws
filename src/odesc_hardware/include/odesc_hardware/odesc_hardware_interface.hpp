#pragma once

#include <vector>
#include <string>
#include <memory>

#include "odesc_hardware/odesc_driver.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "odesc_hardware/odesc_types.hpp"
#include "odesc_hardware/can_interface.hpp"
#include "std_msgs/msg/float32.hpp"

namespace odesc_hardware
{

  class OdescHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    // ──────────────────────────────────────────
    // ros2_control lifecycle
    // ──────────────────────────────────────────

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    // ──────────────────────────────────────────
    // State and command interfaces
    // ──────────────────────────────────────────

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // ──────────────────────────────────────────
    // Read / Write (called every control loop)
    // ──────────────────────────────────────────

    hardware_interface::return_type read(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

  private:
    // ──────────────────────────────────────────
    // CAN communication
    // ──────────────────────────────────────────
    std::shared_ptr<CanInterface> can_;
    std::shared_ptr<OdescDriver> driver_;

    // ──────────────────────────────────────────
    // Axis configuration (loaded from URDF params)
    // ──────────────────────────────────────────
    std::vector<AxisConfig> axis_configs_;

    // ──────────────────────────────────────────
    // Live state (written by read(), exposed to controllers)
    // ──────────────────────────────────────────
    std::vector<double> hw_positions_;  // radians
    std::vector<double> hw_velocities_; // radians/sec

    // ──────────────────────────────────────────
    // Commands (written by controllers, sent in write())
    // ──────────────────────────────────────────
    std::vector<double> hw_commands_; // radians/sec (velocity mode)

    // ──────────────────────────────────────────
    // Helpers
    // ──────────────────────────────────────────

    // Parses node_id and invert flag from URDF joint parameters
    bool parse_joint_params(const hardware_interface::HardwareInfo &info);

    // CAN frame callback — updates hw_positions_ and hw_velocities_
    void on_can_frame(uint32_t can_id, const uint8_t *data, uint8_t len);

    // CAN interface name loaded from URDF params (default "can0")
    std::string can_interface_name_;

    // Voltage publishing (2 Hz, out-of-band node)
    rclcpp::Node::SharedPtr voltage_pub_node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_right_pub_;
    int voltage_tick_ = 0;
  };

} // namespace odesc_hardware