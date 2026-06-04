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

        // --- Velocity caps (velocity mode) — tune here ---
        // X: GT2 belt, 32 mm/rev — higher torque margin at speed
        // Y/Z: T8 lead screw, 8 mm/rev — lower torque; Z also carries vertical load
        static constexpr double  X_VEL_CAP_MS = 0.15;
        static constexpr double  Y_VEL_CAP_MS = 0.06;
        static constexpr double  Z_VEL_CAP_MS = 0.04;

        // ACC byte for 0xF6 velocity commands.
        // Velocity caps above are the primary stall protection; ACC just avoids instantaneous jumps.
        // Previous tests with ACC=230 worked fine at higher speeds than these caps allow.
        static constexpr uint8_t X_VEL_ACC    = 230;
        static constexpr uint8_t Y_VEL_ACC    = 230;
        static constexpr uint8_t Z_VEL_ACC    = 230;

        // --- Soft-limit decel zone (velocity mode near travel limits) ---
        static constexpr double  DECEL_ZONE_M = 0.080; // decel starts 80 mm before limit
        static constexpr double  GUARD_ZONE_M = 0.025; // zero-velocity dead zone at 25 mm

        // --- Travel limits (populated from URDF in on_init) ---
        double x_lower_, x_upper_;
        double y_lower_, y_upper_;
        double z_lower_, z_upper_;

        // Per-motor X positions (rad) updated every read() — used for sync monitoring
        double x1_pos_rad_ = 0.0;
        double x2_pos_rad_ = 0.0;

        // Set when X1/X2 diverge past HALT threshold; cleared on re-home.
        // While true, write() blocks all commands.
        bool x_sync_fault_ = false;

        // --- Stall auto-recovery state ---
        // -1 = idle; ≥0 = number of recovery attempts so far
        int stall_recovery_attempts_ = -1;
        static constexpr int MAX_STALL_RECOVERY = 5;

        // Coordinated X-axis homing: sends 0x91 to both X motors simultaneously and
        // monitors both in the same loop, aborting if they diverge unsafely.
        bool homeXAxisCoordinated();
    };

} // namespace gantry_hardware
#endif // GANTRY_HARDWARE_INTERFACE_HPP