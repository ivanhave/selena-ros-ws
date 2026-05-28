#ifndef GANTRY_HARDWARE_INTERFACE_HPP
#define GANTRY_HARDWARE_INTERFACE_HPP

#include "MKS_Driver.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <vector>
#include <memory>
#include <limits>
#include <atomic>

namespace gantry_hardware
{
    using CallbackReturn = hardware_interface::CallbackReturn;

    class GantryHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        // --- Lifecycle Methods ---
        CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // --- Core Hardware Loop ---
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // --- Controller Mode Switch Callbacks ---
        // Called automatically by controller manager on every switch_controller call
        // prepare: validate the switch before it happens
        // perform: apply the mode change after controllers have switched
        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

        // --- Interface Exports ---
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    private:
        // --- Motor Drivers ---
        // X axis has two parallel motors (x1 and x2) to prevent racking
        // ros2_control sees 3 joints: X, Y, Z
        std::unique_ptr<MKSDriver> x1_motor_;
        std::unique_ptr<MKSDriver> x2_motor_;
        std::unique_ptr<MKSDriver> y_motor_;
        std::unique_ptr<MKSDriver> z_motor_;

        // --- State Vector ---
        // Layout: [J0_Pos, J0_Vel, J1_Pos, J1_Vel, J2_Pos, J2_Vel]
        // Joint 0 = X, Joint 1 = Y, Joint 2 = Z
        // Access: joint i → position at [i*2], velocity at [i*2+1]
        std::vector<double> hw_states_;

        // --- Command Vector ---
        // Layout: [J0_Pos, J0_Vel, J1_Pos, J1_Vel, J2_Pos, J2_Vel]
        // Joint 0 = X, Joint 1 = Y, Joint 2 = Z
        // Access: joint i → position at [i*2], velocity at [i*2+1]
        // Supports both JTC (position) and forward velocity controllers
        std::vector<double> hw_commands_;

        // --- Control Mode ---
        // true  → gantry_velocity_controller active — setTargetVelocityRadianPerSec()
        // false → joint_trajectory_controller active — setTargetPositionAbsoluteRadian()
        // Set automatically by perform_command_mode_switch() when controller manager
        // switches controllers — no external topic or web app involvement needed
        std::atomic<bool> velocity_mode_{false};

        // Unit conversion: radians (motor) ↔ meters (JTC/URDF)
        // X axis: GT2 belt, 16 tooth pulley → 32mm per revolution
        static constexpr double X_RAD_PER_METER = 2.0 * M_PI / 0.032; // 196.350 rad/m
        // Y axis: T8 lead screw → 8mm per revolution
        static constexpr double Y_RAD_PER_METER = 2.0 * M_PI / 0.008; // 785.398 rad/m
        // Z axis: T8 lead screw → 8mm per revolution
        static constexpr double Z_RAD_PER_METER = 2.0 * M_PI / 0.008; // 785.398 rad/m

        // Max travel limits
        double x_lower_, x_upper_;
        double y_lower_, y_upper_;
        double z_lower_, z_upper_;
    };

} // namespace gantry_hardware
#endif // GANTRY_HARDWARE_INTERFACE_HPP