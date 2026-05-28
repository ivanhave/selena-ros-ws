#include "mks_servo_hardware/gantry_hardware_interface.hpp"
#include <thread>
#include <chrono>
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

        // If controller wants position interfaces it's JTC — always acceptable
        // If controller wants velocity only it's gantry_velocity_controller — acceptable
        // Both at same time would be a conflict — reject
        if (wants_velocity_only && wants_position)
        {
            RCLCPP_ERROR(rclcpp::get_logger("GantryHardwareInterface"),
                "Rejecting mode switch — cannot claim both position and velocity-only interfaces.");
            return hardware_interface::return_type::ERROR;
        }

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

        // --- Phase 2: X1, X2, Y in parallel ---
        RCLCPP_INFO(rclcpp::get_logger("GantryHardwareInterface"),
                    "Z complete. Homing X1, X2, Y simultaneously...");

        std::future<bool> x1_task = std::async(std::launch::async,
            [&]{ return homingSequence(x1_motor_.get(), "X1"); });
        std::future<bool> x2_task = std::async(std::launch::async,
            [&]{ return homingSequence(x2_motor_.get(), "X2"); });
        std::future<bool> y_task  = std::async(std::launch::async,
            [&]{ return homingSequence(y_motor_.get(),  "Y");  });

        if (!x1_task.get() || !x2_task.get() || !y_task.get()) {
            RCLCPP_FATAL(rclcpp::get_logger("GantryHardwareInterface"),
                         "X1, X2 or Y homing failed.");
            return CallbackReturn::ERROR;
        }

        // Enable stall protection only after all axes are homed
        x1_motor_->enableStallProtection(300, 14000);
        x2_motor_->enableStallProtection(300, 14000);
        y_motor_->enableStallProtection(300, 14000);
        z_motor_->enableStallProtection(300, 14000);

        // Clear any stall state from homing — motors were at mechanical end stop
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        x1_motor_->releaseStall();
        x2_motor_->releaseStall();
        y_motor_->releaseStall();
        z_motor_->releaseStall();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

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

        // 2. Check for stalls
        if (x1_motor_->isStalled() || x2_motor_->isStalled() ||
            y_motor_->isStalled()  || z_motor_->isStalled())
        {
            RCLCPP_ERROR(rclcpp::get_logger("GantryHardwareInterface"),
                         "STALL DETECTED — stopping all motors.");
            x1_motor_->deactivate();
            x2_motor_->deactivate();
            y_motor_->deactivate();
            z_motor_->deactivate();
            return hardware_interface::return_type::ERROR;
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

        // 5. X-axis sync check — warn if X1 and X2 drift apart (belt slip indicator)
        if (std::abs(x1_pos - x2_pos) > 0.15)
        {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("GantryHardwareInterface"),
                                 *get_clock(), 1000,
                                 "X-Axis sync warning: X1=%.3f X2=%.3f diff=%.3f rad",
                                 x1_pos, x2_pos, std::abs(x1_pos - x2_pos));
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type GantryHardwareInterface::write(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        // 1. Safety: ignore NaN commands
        for (const auto &cmd : hw_commands_)
        {
            if (std::isnan(cmd))
                return hardware_interface::return_type::OK;
        }

        // 2. Extract and clamp commands to URDF limits
        double x_pos_m = std::clamp(hw_commands_[0], x_lower_, x_upper_);
        double x_vel_m = hw_commands_[1];
        double y_pos_m = std::clamp(hw_commands_[2], y_lower_, y_upper_);
        double y_vel_m = hw_commands_[3];
        double z_pos_m = std::clamp(hw_commands_[4], z_lower_, z_upper_);
        double z_vel_m = hw_commands_[5];

        // 3. Convert to radians
        double x_pos_rad = x_pos_m * X_RAD_PER_METER;
        double x_vel_rad = x_vel_m * X_RAD_PER_METER;
        double y_pos_rad = y_pos_m * Y_RAD_PER_METER;
        double y_vel_rad = y_vel_m * Y_RAD_PER_METER;
        double z_pos_rad = z_pos_m * Z_RAD_PER_METER;
        double z_vel_rad = z_vel_m * Z_RAD_PER_METER;

        // 4. Dispatch commands based on active control mode
        if (velocity_mode_.load())
        {
            // Velocity mode — joystick teleop via gantry_velocity_controller
            const uint8_t ACC = 235; // smooth acceleration/deceleration ramp
            x1_motor_->setTargetVelocityRadianPerSec(x_vel_rad, ACC);
            x2_motor_->setTargetVelocityRadianPerSec(x_vel_rad, ACC);
            y_motor_->setTargetVelocityRadianPerSec(y_vel_rad, ACC);
            z_motor_->setTargetVelocityRadianPerSec(z_vel_rad, ACC);
        }
        else
        {
            // Position mode — autonomous sequences via JTC
            x1_motor_->setTargetPositionAbsoluteRadian(x_pos_rad, x_vel_rad, 0);
            x2_motor_->setTargetPositionAbsoluteRadian(x_pos_rad, x_vel_rad, 0);
            y_motor_->setTargetPositionAbsoluteRadian(y_pos_rad, y_vel_rad, 0);
            z_motor_->setTargetPositionAbsoluteRadian(z_pos_rad, z_vel_rad, 0);
        }

        return hardware_interface::return_type::OK;
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