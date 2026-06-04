#include "mks_servo_hardware/gantry_hardware_interface.hpp"
#include <thread>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <future>
#include "pluginlib/class_list_macros.hpp"

namespace gantry_hardware
{

    CallbackReturn GantryHardwareInterface::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        // 1. Standard Boilerplate
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        // 2. Hardware Parameter Extraction
        // We look for "can_interface" in the <hardware> tag of the ros2_control URDF.
        // Falls back to 'can0' if not specified.
        std::string can_interface = info_.hardware_parameters.count("can_interface") ?
            info_.hardware_parameters.at("can_interface") : "can0";

        // 3. Driver Instantiation
        // X1=1, X2=2 (Parallel gantry), Y=3, Z=4
        // No CAN connection yet — that happens in on_configure
        try
        {
            x1_motor_ = std::make_unique<MKSDriver>(can_interface, 1);
            x2_motor_ = std::make_unique<MKSDriver>(can_interface, 2);
            y_motor_  = std::make_unique<MKSDriver>(can_interface, 3);
            z_motor_  = std::make_unique<MKSDriver>(can_interface, 4);
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                         "Failed to create motor drivers: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // 4. Validate joint count
        // ros2_control sees 3 joints (X, Y, Z) even though X has 2 physical motors
        if (info_.joints.size() != 3)
        {
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                         "Expected 3 joints (X, Y, Z), but found %zu in URDF.",
                         info_.joints.size());
            return CallbackReturn::ERROR;
        }

        // 5. Size state and command vectors
        // States:   [J0_Pos, J0_Vel, J1_Pos, J1_Vel, J2_Pos, J2_Vel]
        // Commands: [J0_Pos, J0_Vel, J1_Pos, J1_Vel, J2_Pos, J2_Vel]
        // Joint i → position at [i*2], velocity at [i*2+1]
        hw_states_.assign(info_.joints.size() * 2, std::numeric_limits<double>::quiet_NaN());
        hw_commands_.assign(info_.joints.size() * 2, 0.0);

        // 6. Retrieve limits from URDF
        for (const auto &joint : info_.joints)
        {
            double lower = 0.0;
            double upper = 0.0;

            for (const auto &interface : joint.command_interfaces)
            {
                if (interface.name == "position")
                {
                    lower = std::stod(interface.min);
                    upper = std::stod(interface.max);
                }
            }

            if (joint.name == "x_axis_joint")      { x_lower_ = lower; x_upper_ = upper; }
            else if (joint.name == "y_axis_joint") { y_lower_ = lower; y_upper_ = upper; }
            else if (joint.name == "z_axis_joint") { z_lower_ = lower; z_upper_ = upper; }
        }

        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "GantryHardwareInterface initialized on %s.", can_interface.c_str());
        return CallbackReturn::SUCCESS;
    }

    // --- prepare_command_mode_switch ---
    // Called by controller manager BEFORE switching controllers.
    // Validate that the requested switch is acceptable.
    // start_interfaces: command interfaces the incoming controller wants to claim
    // stop_interfaces:  command interfaces the outgoing controller is releasing
    hardware_interface::return_type GantryHardwareInterface::prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces)
    {
        (void)stop_interfaces; // not needed for validation

        // Check if incoming controller wants velocity-only interfaces
        // gantry_velocity_controller claims: x/velocity, y/velocity, z/velocity
        // joint_trajectory_controller claims: x/position, x/velocity, y/position, etc.
        bool wants_velocity_only = false;
        bool wants_position      = false;

        for (const auto &iface : start_interfaces)
        {
            if (iface.find("/velocity") != std::string::npos &&
                (iface.find("x_axis") != std::string::npos ||
                 iface.find("y_axis") != std::string::npos ||
                 iface.find("z_axis") != std::string::npos))
            {
                wants_velocity_only = true;
            }
            if (iface.find("/position") != std::string::npos)
            {
                wants_position = true;
            }
        }

        // Accepts:
        // - velocity only              → gantry_velocity_controller
        // - position only              → JTC (position control)
        // - position + velocity        → JTC with velocity feedforward (also valid)
        (void)wants_velocity_only;
        (void)wants_position;
        return hardware_interface::return_type::OK;
    }

    // --- perform_command_mode_switch ---
    // Called by controller manager AFTER controllers have switched.
    // This is where we actually switch the motor mode.
    // start_interfaces: interfaces the newly active controller claimed
    hardware_interface::return_type GantryHardwareInterface::perform_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces)
    {
        (void)stop_interfaces;

        // Detect if the newly active controller is velocity-only
        // (gantry_velocity_controller) or position+velocity (JTC)
        bool has_position_interface = false;
        bool has_velocity_interface = false;

        for (const auto &iface : start_interfaces)
        {
            if (iface.find("/position") != std::string::npos &&
                (iface.find("x_axis") != std::string::npos ||
                 iface.find("y_axis") != std::string::npos ||
                 iface.find("z_axis") != std::string::npos))
            {
                has_position_interface = true;
            }
            if (iface.find("/velocity") != std::string::npos &&
                (iface.find("x_axis") != std::string::npos ||
                 iface.find("y_axis") != std::string::npos ||
                 iface.find("z_axis") != std::string::npos))
            {
                has_velocity_interface = true;
            }
        }

        // gantry_velocity_controller — velocity only, no position
        if (has_velocity_interface && !has_position_interface)
        {
            if (!velocity_mode_.load())
            {
                velocity_mode_.store(true);
                x1_motor_->activateVelocityMode();
                x2_motor_->activateVelocityMode();
                y_motor_->activateVelocityMode();
                z_motor_->activateVelocityMode();
                RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "Controller switch detected — switched to VELOCITY mode.");
            }
        }
        // joint_trajectory_controller — position + velocity
        else if (has_position_interface)
        {
            if (velocity_mode_.load())
            {
                // Seed position commands from current state before write() sees position mode.
                // Prevents motors from receiving stale pre-manual-mode position during the
                // 1-2 control cycles before JTC's first update() writes the correct hold position.
                hw_commands_[0] = hw_states_[0];
                hw_commands_[2] = hw_states_[2];
                hw_commands_[4] = hw_states_[4];
                hw_commands_[1] = 0.0;
                hw_commands_[3] = 0.0;
                hw_commands_[5] = 0.0;

                velocity_mode_.store(false);
                x1_motor_->activateAbsolutePositionMode();
                x2_motor_->activateAbsolutePositionMode();
                y_motor_->activateAbsolutePositionMode();
                z_motor_->activateAbsolutePositionMode();
                RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "Controller switch detected — switched to POSITION mode.");
            }
        }

        return hardware_interface::return_type::OK;
    }

    CallbackReturn GantryHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "Configuring CAN bus connections...");

        // Initialize all 4 motors — establishes SocketCAN binding,
        // spawns listener threads, and auto-detects homing direction (hmDir).
        // Each motor gets its own socket with a CAN ID filter.
        if (!x1_motor_->init()) {
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"), "Motor X1 (ID=1) init failed.");
            return CallbackReturn::ERROR;
        }
        if (!x2_motor_->init()) {
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"), "Motor X2 (ID=2) init failed.");
            return CallbackReturn::ERROR;
        }
        if (!y_motor_->init()) {
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"), "Motor Y (ID=3) init failed.");
            return CallbackReturn::ERROR;
        }
        if (!z_motor_->init()) {
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"), "Motor Z (ID=4) init failed.");
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "All motors initialized. Direction multipliers auto-detected.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn GantryHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // If any motor went offline since last activation, recover first
        auto tryRecover = [&](MKSDriver * motor, const std::string & name) {
            if (motor->isOffline()) {
                RCLCPP_WARN(rclcpp::get_logger("GantryHardwareInterface"),
                    "%s was offline — attempting recovery.", name.c_str());
                if (!motor->recover()) {
                    RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                        "%s recovery failed.", name.c_str());
                    return false;
                }
            }
            return true;
        };

        if (!tryRecover(x1_motor_.get(), "X1") ||
            !tryRecover(x2_motor_.get(), "X2") ||
            !tryRecover(y_motor_.get(),  "Y")  ||
            !tryRecover(z_motor_.get(),  "Z"))
        {
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "Starting Safety Homing Sequence...");

        auto homingSequence = [&](MKSDriver *motor, const std::string &name) -> bool
        {
            motor->activateAbsolutePositionMode();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            if (!motor->goHome()) {
                RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                             "%s homing failed.", name.c_str());
                return false;
            }

            RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                        "%s homing complete.", name.c_str());
            return true;
        };

        // --- Phase 1: Z first ---
        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"), "Homing Z...");
        if (!homingSequence(z_motor_.get(), "Z"))
            return CallbackReturn::ERROR;

        // --- Phase 2: X1+X2 coordinated, Y concurrent ---
        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "Z complete. Homing X1+X2 (coordinated) and Y (concurrent)...");

        // Y homes concurrently with X — it is mechanically independent
        std::future<bool> y_task = std::async(std::launch::async,
            [&]{ return homingSequence(y_motor_.get(), "Y"); });

        // X1 and X2 are mechanically coupled — home them in a single coordinated loop
        // to detect and abort if one motor stalls while the other keeps running.
        x_sync_fault_ = false;
        if (!homeXAxisCoordinated()) {
            y_task.get(); // let Y finish or timeout before returning
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                         "X-axis coordinated homing failed — possible gantry skew.");
            return CallbackReturn::ERROR;
        }

        if (!y_task.get()) {
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                         "Y homing failed.");
            return CallbackReturn::ERROR;
        }

        // Enable stall protection only after all axes are homed
        x1_motor_->enableStallProtection(300, 14000);
        x2_motor_->enableStallProtection(300, 14000);
        y_motor_->enableStallProtection(300, 14000);
        z_motor_->enableStallProtection(300, 14000);

        // Clear any stall state from homing — motors were at mechanical end stop.
        // Retry up to 5 times per motor; CAN acks can be dropped on a busy bus.
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto release_with_retry = [](MKSDriver *m) {
            for (int i = 0; i < 5; ++i) {
                if (m->releaseStall()) return;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        };
        release_with_retry(x1_motor_.get());
        release_with_retry(x2_motor_.get());
        release_with_retry(y_motor_.get());
        release_with_retry(z_motor_.get());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Record each motor's step count at home as zero reference.
        // In stall-detection homing the firmware may not reset the position counter,
        // so we capture whatever value the motor reports and subtract it from all reads/writes.
        x1_motor_->zeroPositionAtHome();
        x2_motor_->zeroPositionAtHome();
        y_motor_->zeroPositionAtHome();
        z_motor_->zeroPositionAtHome();

        // MKS motors stop responding to 0x32 velocity polls while executing 0x91 homing,
        // which causes the online checker to time them out. Reset offline flags now that
        // homing is done and give the listener 200 ms to confirm each motor is alive.
        x1_motor_->clearOfflineFlag();
        x2_motor_->clearOfflineFlag();
        y_motor_->clearOfflineFlag();
        z_motor_->clearOfflineFlag();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Verify no motor went persistently offline (hardware failure vs homing glitch)
        if (x1_motor_->isOffline() || x2_motor_->isOffline() ||
            y_motor_->isOffline()  || z_motor_->isOffline())
        {
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                         "Motor offline after homing — check CAN wiring.");
            return CallbackReturn::ERROR;
        }

        // Post-homing sync check: X1 and X2 must both read ~0 after zeroing.
        // If zeroPositionAtHome() failed to capture the offset for one motor (CAN miss
        // during 0x91 — motor stops responding), the first position read will be the
        // motor's raw absolute encoder value instead of 0, causing an immediate sync fault.
        // Retry zeroing up to 3 times if the divergence is too large.
        for (int retry = 0; retry < 3; ++retry)
        {
            double x1_rad = -x1_motor_->getPositionRadian();
            double x2_rad =  x2_motor_->getPositionRadian();
            double diff = std::abs(x1_rad - x2_rad);
            if (diff <= 1.0)  // under 5 mm — safe to proceed
                break;
            RCLCPP_WARN(rclcpp::get_logger("GantryHardwareInterface"),
                        "Post-homing X sync check: X1=%.3f X2=%.3f diff=%.3f rad — "
                        "offset capture likely missed; re-zeroing (attempt %d/3).",
                        x1_rad, x2_rad, diff, retry + 1);
            x1_motor_->zeroPositionAtHome();
            x2_motor_->zeroPositionAtHome();
        }

        std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "All axes homed. Gantry ready.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn GantryHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "Deactivating: Stopping all motors.");

        // 1. Emergency stop all motors immediately
        x1_motor_->deactivate();
        x2_motor_->deactivate();
        y_motor_->deactivate();
        z_motor_->deactivate();

        // 2. Release any stall locks so motors are free on next activation
        if (!x1_motor_->releaseStall())
            RCLCPP_WARN(rclcpp::get_logger("GantryHardwareInterface"),
                        "X1 stall release failed or not stalled.");
        if (!x2_motor_->releaseStall())
            RCLCPP_WARN(rclcpp::get_logger("GantryHardwareInterface"),
                        "X2 stall release failed or not stalled.");
        if (!y_motor_->releaseStall())
            RCLCPP_WARN(rclcpp::get_logger("GantryHardwareInterface"),
                        "Y stall release failed or not stalled.");
        if (!z_motor_->releaseStall())
            RCLCPP_WARN(rclcpp::get_logger("GantryHardwareInterface"),
                        "Z stall release failed or not stalled.");

        // 3. Clear internal state and command buffers
        std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
        std::fill(hw_states_.begin(), hw_states_.end(),
                  std::numeric_limits<double>::quiet_NaN());

        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "Deactivation complete.");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type GantryHardwareInterface::read(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        // 1. Check if any motor went offline
        if (x1_motor_->isOffline() || x2_motor_->isOffline() ||
            y_motor_->isOffline()  || z_motor_->isOffline())
        {
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                         "Motor offline detected — hardware failure.");
            return hardware_interface::return_type::ERROR;
        }

        // 2. Check for stalls — attempt auto-recovery before giving up.
        //    write() sends stop + releaseStall() each cycle while recovery is active.
        bool any_stalled = x1_motor_->isStalled() || x2_motor_->isStalled() ||
                           y_motor_->isStalled()   || z_motor_->isStalled();
        if (any_stalled) {
            if (stall_recovery_attempts_ < 0) {
                RCLCPP_WARN(rclcpp::get_logger("GantryHardwareInterface"),
                            "Stall detected — attempting auto-recovery (max %d attempts).",
                            MAX_STALL_RECOVERY);
                stall_recovery_attempts_ = 0;
            }
            if (stall_recovery_attempts_ >= MAX_STALL_RECOVERY) {
                RCLCPP_ERROR(rclcpp::get_logger("GantryHardwareInterface"),
                             "Stall not cleared after %d attempts — deactivating.",
                             MAX_STALL_RECOVERY);
                x1_motor_->deactivate();
                x2_motor_->deactivate();
                y_motor_->deactivate();
                z_motor_->deactivate();
                stall_recovery_attempts_ = -1;
                return hardware_interface::return_type::ERROR;
            }
            return hardware_interface::return_type::OK;  // keep alive; write() handles recovery
        }
        if (stall_recovery_attempts_ >= 0) {
            RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                        "Stall cleared after %d attempt(s) — resuming normal operation.",
                        stall_recovery_attempts_);
            stall_recovery_attempts_ = -1;
        }

        // 3. Capture feedback
        // X1 is mechanically inverted — negate its position and velocity
        double x1_pos = -x1_motor_->getPositionRadian();
        double x1_vel = -x1_motor_->getVelocityRadianPerSec();
        double x2_pos =  x2_motor_->getPositionRadian();
        double x2_vel =  x2_motor_->getVelocityRadianPerSec();
        double y_pos  =  y_motor_->getPositionRadian();
        double y_vel  =  y_motor_->getVelocityRadianPerSec();
        double z_pos  =  z_motor_->getPositionRadian();
        double z_vel  =  z_motor_->getVelocityRadianPerSec();

        // 4. Update joint states — convert rad to meters
        // Joint 0: X — average of X1 and X2
        hw_states_[0] = ((x1_pos + x2_pos) / 2.0) / X_RAD_PER_METER;
        hw_states_[1] = ((x1_vel + x2_vel) / 2.0) / X_RAD_PER_METER;
        // Joint 1: Y
        hw_states_[2] = y_pos / Y_RAD_PER_METER;
        hw_states_[3] = y_vel / Y_RAD_PER_METER;
        // Joint 2: Z
        hw_states_[4] = z_pos / Z_RAD_PER_METER;
        hw_states_[5] = z_vel / Z_RAD_PER_METER;

        // 5. X-axis sync monitoring — X1 and X2 must track each other within safe limits.
        //    Both thresholds use radians; X_RAD_PER_METER = 196.35 rad/m → 1 rad ≈ 5.1 mm.
        x1_pos_rad_ = x1_pos;
        x2_pos_rad_ = x2_pos;
        double x_sync_diff = std::abs(x1_pos - x2_pos);

        if (x_sync_diff > 2.00)  // ~10.2 mm — racking risk; stop immediately
        {
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                         "X-AXIS SYNC FAULT: X1=%.3f X2=%.3f diff=%.3f rad — EMERGENCY STOP. "
                         "Re-home required.",
                         x1_pos, x2_pos, x_sync_diff);
            x_sync_fault_ = true;
            x1_motor_->deactivate();
            x2_motor_->deactivate();
            y_motor_->deactivate();
            z_motor_->deactivate();
            return hardware_interface::return_type::ERROR;
        }
        else if (x_sync_diff > 1.00)  // ~5.1 mm — early warning; continue but alert
        {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("GantryHardwareInterface"),
                                 *get_clock(), 1000,
                                 "X-Axis sync warning: X1=%.3f X2=%.3f diff=%.3f rad",
                                 x1_pos, x2_pos, x_sync_diff);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type GantryHardwareInterface::write(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        // 1. Safety: block all motion while X sync fault is active
        if (x_sync_fault_)
        {
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("GantryHardwareInterface"),
                                  *get_clock(), 2000,
                                  "X sync fault active — all commands blocked. Re-home to clear.");
            return hardware_interface::return_type::OK;
        }

        // 2. Safety: ignore NaN commands
        for (const auto &cmd : hw_commands_)
        {
            if (std::isnan(cmd))
                return hardware_interface::return_type::OK;
        }

        // 2b. Stall recovery: stop all motors and attempt releaseStall() each cycle.
        //     releaseStall() blocks ≤200 ms — acceptable for one slow loop iteration.
        if (stall_recovery_attempts_ >= 0)
        {
            x1_motor_->setTargetVelocityRadianPerSec(0.0, X_VEL_ACC);
            x2_motor_->setTargetVelocityRadianPerSec(0.0, X_VEL_ACC);
            y_motor_->setTargetVelocityRadianPerSec(0.0,  Y_VEL_ACC);
            z_motor_->setTargetVelocityRadianPerSec(0.0,  Z_VEL_ACC);
            if (x1_motor_->isStalled()) x1_motor_->releaseStall();
            if (x2_motor_->isStalled()) x2_motor_->releaseStall();
            if (y_motor_->isStalled())  y_motor_->releaseStall();
            if (z_motor_->isStalled())  z_motor_->releaseStall();
            stall_recovery_attempts_++;
            return hardware_interface::return_type::OK;
        }

        // 3. In position mode, clamp commands to URDF travel bounds.
        //    Clamping (not ignoring) keeps both X motors commanded to the same position,
        //    which prevents X1/X2 sync faults if a trajectory overshoots the limit.
        //    Velocity mode is handled by the decel ramp below — no clamping needed.
        if (!velocity_mode_.load())
        {
            // Only warn if outside bounds by more than 1 mm — suppresses floating-point noise
            // from JTC holding at home (commands like -0.0001 m due to spline residuals).
            static constexpr double POS_WARN_TOL = 0.001;
            if (hw_commands_[0] < x_lower_ - POS_WARN_TOL || hw_commands_[0] > x_upper_ + POS_WARN_TOL)
            {
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger("GantryHardwareInterface"),
                                     *get_clock(), 1000,
                                     "X position command %.4f m is outside travel [%.3f, %.3f] — clamped.",
                                     hw_commands_[0], x_lower_, x_upper_);
            }
            hw_commands_[0] = std::clamp(hw_commands_[0], x_lower_, x_upper_);

            if (hw_commands_[2] < y_lower_ - POS_WARN_TOL || hw_commands_[2] > y_upper_ + POS_WARN_TOL)
            {
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger("GantryHardwareInterface"),
                                     *get_clock(), 1000,
                                     "Y position command %.4f m is outside travel [%.3f, %.3f] — clamped.",
                                     hw_commands_[2], y_lower_, y_upper_);
            }
            hw_commands_[2] = std::clamp(hw_commands_[2], y_lower_, y_upper_);

            if (hw_commands_[4] < z_lower_ - POS_WARN_TOL || hw_commands_[4] > z_upper_ + POS_WARN_TOL)
            {
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger("GantryHardwareInterface"),
                                     *get_clock(), 1000,
                                     "Z position command %.4f m is outside travel [%.3f, %.3f] — clamped.",
                                     hw_commands_[4], z_lower_, z_upper_);
            }
            hw_commands_[4] = std::clamp(hw_commands_[4], z_lower_, z_upper_);
        }

        // 4. Extract commands (position already validated above; velocity clamped by decel ramp)
        double x_pos_m = hw_commands_[0];
        double x_vel_m = hw_commands_[1];
        double y_pos_m = hw_commands_[2];
        double y_vel_m = hw_commands_[3];
        double z_pos_m = hw_commands_[4];
        double z_vel_m = hw_commands_[5];


        // 5. Soft-limit decel: sqrt velocity profile inside DECEL_ZONE near each limit.
        //    sqrt matches kinematic braking (v ∝ sqrt(distance)) so the motor naturally
        //    reaches commanded=0 exactly at the guard zone regardless of starting speed.
        //    Reverse movement (away from limit) is always allowed unchanged.
        auto soft_limit_vel = [&](double vel, double pos, double lower, double upper) -> double {
            if (vel > 0.0) {
                if (pos >= upper) return 0.0;          // hard clamp: already at/past upper limit
                const double dist = upper - pos;
                if (dist <= GUARD_ZONE_M) return 0.0;
                if (dist <  DECEL_ZONE_M) {
                    const double ratio = (dist - GUARD_ZONE_M) / (DECEL_ZONE_M - GUARD_ZONE_M);
                    return vel * std::sqrt(ratio);
                }
            } else if (vel < 0.0) {
                if (pos <= lower) return 0.0;          // hard clamp: already at/past lower limit
                const double dist = pos - lower;
                if (dist <= GUARD_ZONE_M) return 0.0;
                if (dist <  DECEL_ZONE_M) {
                    const double ratio = (dist - GUARD_ZONE_M) / (DECEL_ZONE_M - GUARD_ZONE_M);
                    return vel * std::sqrt(ratio);
                }
            }
            return vel;
        };

        // 6. Convert position to radians
        double x_pos_rad = x_pos_m * X_RAD_PER_METER;
        double y_pos_rad = y_pos_m * Y_RAD_PER_METER;
        double z_pos_rad = z_pos_m * Z_RAD_PER_METER;

        // 7. Dispatch commands based on active control mode
        if (velocity_mode_.load())
        {
            // 7a. Hard velocity caps — prevent torque-stall on lead-screw axes
            x_vel_m = std::clamp(x_vel_m, -X_VEL_CAP_MS, X_VEL_CAP_MS);
            y_vel_m = std::clamp(y_vel_m, -Y_VEL_CAP_MS, Y_VEL_CAP_MS);
            z_vel_m = std::clamp(z_vel_m, -Z_VEL_CAP_MS, Z_VEL_CAP_MS);

            // 7b. Soft-limit decel near travel boundaries (sqrt braking profile)
            x_vel_m = soft_limit_vel(x_vel_m, hw_states_[0], x_lower_, x_upper_);
            y_vel_m = soft_limit_vel(y_vel_m, hw_states_[2], y_lower_, y_upper_);
            z_vel_m = soft_limit_vel(z_vel_m, hw_states_[4], z_lower_, z_upper_);

            double x_vel_rad = x_vel_m * X_RAD_PER_METER;
            double y_vel_rad = y_vel_m * Y_RAD_PER_METER;
            double z_vel_rad = z_vel_m * Z_RAD_PER_METER;

            x1_motor_->setTargetVelocityRadianPerSec(x_vel_rad, X_VEL_ACC);
            x2_motor_->setTargetVelocityRadianPerSec(x_vel_rad, X_VEL_ACC);
            y_motor_->setTargetVelocityRadianPerSec(y_vel_rad,  Y_VEL_ACC);
            z_motor_->setTargetVelocityRadianPerSec(z_vel_rad,  Z_VEL_ACC);
        }
        else
        {
            // Position mode: cap JTC feedforward velocity so the motor never moves faster
            // than the axis speed limit, regardless of trajectory time_from_start.
            // Without this, a fast JTC trajectory (e.g., full X travel in 3s) can cause
            // the two X motors to diverge and trigger a sync fault.
            x_vel_m = std::clamp(x_vel_m, -X_VEL_CAP_MS, X_VEL_CAP_MS);
            y_vel_m = std::clamp(y_vel_m, -Y_VEL_CAP_MS, Y_VEL_CAP_MS);
            z_vel_m = std::clamp(z_vel_m, -Z_VEL_CAP_MS, Z_VEL_CAP_MS);

            // ACC=0 lets the motor firmware manage acceleration profile.
            // This keeps X1/X2 in tighter sync than an explicit ramp would.
            double x_vel_rad = x_vel_m * X_RAD_PER_METER;
            double y_vel_rad = y_vel_m * Y_RAD_PER_METER;
            double z_vel_rad = z_vel_m * Z_RAD_PER_METER;
            x1_motor_->setTargetPositionAbsoluteRadian(x_pos_rad, x_vel_rad, 0);
            x2_motor_->setTargetPositionAbsoluteRadian(x_pos_rad, x_vel_rad, 0);
            y_motor_->setTargetPositionAbsoluteRadian(y_pos_rad,  y_vel_rad, 0);
            z_motor_->setTargetPositionAbsoluteRadian(z_pos_rad,  z_vel_rad, 0);
        }

        return hardware_interface::return_type::OK;
    }

    bool GantryHardwareInterface::homeXAxisCoordinated()
    {
        // Prepare both X motors for homing
        x1_motor_->activateAbsolutePositionMode();
        x2_motor_->activateAbsolutePositionMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Send 0x91 to both motors from the same thread, back-to-back (<1 ms apart).
        // This minimises the chance of one motor having a significant head-start.
        x1_motor_->startHoming();
        x2_motor_->startHoming();
        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "X1 and X2 homing commands sent simultaneously.");

        std::this_thread::sleep_for(std::chrono::milliseconds(800)); // let motors begin moving

        // Monitor both motors in a single loop.
        // Each motor needs 10 consecutive velocity readings < 0.05 rad/s (= 1 s of stillness).
        // Abort if one motor has been stationary for > 2 s while the other shows ANY movement
        // — indicates one motor didn't receive the homing command or stalled mid-travel.
        constexpr double STATIONARY_THRESH = 0.05;   // rad/s — "motor stopped"
        constexpr int    DONE_COUNT        = 10;      // 10 × 100 ms = 1 s
        constexpr int    DIVERGE_COUNT     = 20;      // 20 × 100 ms = 2 s

        int x1_count = 0, x2_count = 0;
        bool any_motion_x1 = false, any_motion_x2 = false;  // did each motor actually move?
        auto start = std::chrono::steady_clock::now();

        while (true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            auto elapsed = std::chrono::steady_clock::now() - start;
            if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 60)
            {
                RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                             "X-axis coordinated homing timed out (60 s).");
                return false;
            }

            double x1_vel = std::abs(x1_motor_->getVelocityRadianPerSec());
            double x2_vel = std::abs(x2_motor_->getVelocityRadianPerSec());

            if (x1_vel < STATIONARY_THRESH) x1_count++; else { x1_count = 0; any_motion_x1 = true; }
            if (x2_vel < STATIONARY_THRESH) x2_count++; else { x2_count = 0; any_motion_x2 = true; }

            // Abort if one motor has been stationary 2 s while the other shows any movement.
            // Uses STATIONARY_THRESH (not a higher "clearly moving" threshold) because
            // slow ramp-up or a pulled stall keeps velocity low — catching it early prevents
            // gantry skew damage.
            if (x1_count >= DIVERGE_COUNT && x2_vel > STATIONARY_THRESH)
            {
                RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                             "X-axis homing ABORT: X1 stationary for %.1f s "
                             "but X2 still moving at %.3f rad/s — gantry skew risk.",
                             x1_count * 0.1, x2_vel);
                return false;
            }
            if (x2_count >= DIVERGE_COUNT && x1_vel > STATIONARY_THRESH)
            {
                RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                             "X-axis homing ABORT: X2 stationary for %.1f s "
                             "but X1 still moving at %.3f rad/s — gantry skew risk.",
                             x2_count * 0.1, x1_vel);
                return false;
            }

            // Both stationary long enough — verify both actually moved during homing.
            // A motor that never moved (CAN miss, or pulled stall from coupled beam)
            // will have any_motion=false even though velocity=0 reads as "stationary."
            // Exception: if NEITHER moved, gantry was likely already at home — allow it.
            if (x1_count >= DONE_COUNT && x2_count >= DONE_COUNT)
            {
                if (!any_motion_x1 && !any_motion_x2)
                {
                    // Both stationary the entire time — gantry was already at home position.
                    RCLCPP_WARN(rclcpp::get_logger("GantryHardwareInterface"),
                                "X homing: neither motor moved — gantry was already at home.");
                    return true;
                }
                if (!any_motion_x1)
                {
                    RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                                 "X homing FAILED: X1 never moved during homing — "
                                 "CAN miss or mechanical block while X2 homed correctly.");
                    return false;
                }
                if (!any_motion_x2)
                {
                    RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                                 "X homing FAILED: X2 never moved during homing — "
                                 "CAN miss or mechanical block while X1 homed correctly.");
                    return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                            "X1 and X2 both reached home and stopped.");
                return true;
            }
        }
    }

    std::vector<hardware_interface::StateInterface>
    GantryHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_states_[i * 2]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_states_[i * 2 + 1]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    GantryHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            // Position command — joint i → index [i*2]
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_commands_[i * 2]));
            // Velocity command — joint i → index [i*2+1]
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_commands_[i * 2 + 1]));
        }
        return command_interfaces;
    }

} // namespace gantry_hardware

PLUGINLIB_EXPORT_CLASS(
    gantry_hardware::GantryHardwareInterface,
    hardware_interface::SystemInterface)