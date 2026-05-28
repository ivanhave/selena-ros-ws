#!/usr/bin/env python3
"""
ODrive ODESC v0.5.1 configuration script for hoverboard motors.
Edit the USER SETTINGS section before running, especially ex. CAN_NODE_IDS = {0: 7, 1: 8}
Created for ROS2, employs velocity mode.

No need to colcon build it, just run the script.
Run with: python3 odrive_setup.py

Or for my selena ros setup:
python3 ~/selena_ros_ws/src/odesc_hardware/scripts/odrive_setup.py
"""

import odrive
from odrive.enums import *
import time
import sys

# ══════════════════════════════════════════════════════════
# USER SETTINGS — edit these before running
# ══════════════════════════════════════════════════════════

# Which axes to configure (0, 1, or both)
CONFIGURE_AXES = [0, 1]

# Motor
POLE_PAIRS                       = 15
RESISTANCE_CALIB_MAX_VOLTAGE     = 4
REQUESTED_CURRENT_RANGE          = 10   # increased for hoverboard motors
CURRENT_LIM                      = 10   # increased from 4A — hoverboard motors
                                         # can handle 15-20A peak. 10A is a safe
                                         # working value. Increase to 15A if more
                                         # torque is needed under load.
CURRENT_CONTROL_BANDWIDTH        = 100
TORQUE_CONSTANT                  = 0.04  # recommended for hoverboard

# Encoder (Hall)
ENCODER_CPR                      = 90
ENCODER_CALIB_SCAN_DISTANCE      = 150
ENCODER_BANDWIDTH                = 100

# Velocity controller (used in ROS2 diff drive)
CONTROL_MODE                     = CONTROL_MODE_VELOCITY_CONTROL
INPUT_MODE                       = INPUT_MODE_PASSTHROUGH
VEL_GAIN                         = 0.1   # recommended increase
VEL_INTEGRATOR_GAIN              = 0.2   # recommended increase
VEL_LIMIT                        = 7.0

# Acceleration canceling, or comment it if you need acceleration.
VEL_RAMP_RATE                    = 50.0  # high = instant response for ROS2

# Watchdog — stops motor if no CAN command received within timeout
# This is a safety feature: if CAN connection is lost, motor stops
ENABLE_WATCHDOG                  = False
WATCHDOG_TIMEOUT                 = 0.5   # seconds — stop after 500ms of no commands

# CAN
CAN_BAUD_RATE                    = 500000

# CAN Node IDs per axis
# On the ODESC board: axis0 = M0 connector, axis1 = M1 connector
# Assign unique IDs that don't clash with other CAN devices
# Edit before running for each board:
#   axis 0 (M0): the motor connected to the M0 terminals on the board
#   axis 1 (M1): the motor connected to the M1 terminals on the board
CAN_NODE_IDS = {0: 6, 1: 5}

# Startup
STARTUP_CLOSED_LOOP              = True

# Calibration currents (increase if calibration fails)
CALIBRATION_CURRENT              = 15
RESISTANCE_CALIB_MAX_VOLTAGE_HIGH = 15  # used during calibration

# ══════════════════════════════════════════════════════════
# SCRIPT — do not edit below unless you know what you're doing
# ══════════════════════════════════════════════════════════

def clear_errors(axes):
    print("\n[0/4] Clearing errors...")
    for i, axis in enumerate(axes):
        if axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0:
            print(f"  Axis {i} had errors — clearing:")
            print(f"    axis.error=0x{axis.error:08X}  motor.error=0x{axis.motor.error:08X}  encoder.error=0x{axis.encoder.error:08X}")
            axis.error = 0
            axis.motor.error = 0
            axis.encoder.error = 0
            print(f"  Axis {i} errors cleared.")
        else:
            print(f"  Axis {i} no errors.")

def configure_axis(axis, axis_num):
    print(f"\n── Configuring axis {axis_num} ──────────────────────")

    # Motor
    print("  Setting motor parameters...")
    axis.motor.config.pole_pairs                   = POLE_PAIRS
    axis.motor.config.resistance_calib_max_voltage = RESISTANCE_CALIB_MAX_VOLTAGE
    axis.motor.config.requested_current_range      = REQUESTED_CURRENT_RANGE
    axis.motor.config.current_lim                  = CURRENT_LIM
    axis.motor.config.current_control_bandwidth    = CURRENT_CONTROL_BANDWIDTH
    axis.motor.config.torque_constant              = TORQUE_CONSTANT

    # Encoder
    print("  Setting encoder parameters...")
    axis.encoder.config.mode                = ENCODER_MODE_HALL
    axis.encoder.config.cpr                 = ENCODER_CPR
    axis.encoder.config.calib_scan_distance = ENCODER_CALIB_SCAN_DISTANCE
    axis.encoder.config.bandwidth           = ENCODER_BANDWIDTH

    # Controller
    print("  Setting controller parameters...")
    axis.controller.config.control_mode        = CONTROL_MODE
    axis.controller.config.input_mode          = INPUT_MODE
    axis.controller.config.vel_gain            = VEL_GAIN
    axis.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN
    axis.controller.config.vel_limit           = VEL_LIMIT
    axis.controller.config.vel_ramp_rate       = VEL_RAMP_RATE

    # Watchdog — safety stop if CAN connection is lost
    print("  Setting watchdog...")
    axis.config.enable_watchdog  = ENABLE_WATCHDOG
    axis.config.watchdog_timeout = WATCHDOG_TIMEOUT

    # CAN node ID
    axis.config.can_node_id = CAN_NODE_IDS[axis_num]
    print(f"  CAN node ID set to {CAN_NODE_IDS[axis_num]}")

    # Startup
    axis.config.startup_closed_loop_control = STARTUP_CLOSED_LOOP

    print(f"  Axis {axis_num} configured.")


def calibrate_axis(odrv, axis, axis_num):
    print(f"\n── Calibrating axis {axis_num} ──────────────────────")

    # Temporarily raise voltage for calibration
    axis.motor.config.resistance_calib_max_voltage = RESISTANCE_CALIB_MAX_VOLTAGE_HIGH
    axis.motor.config.calibration_current          = CALIBRATION_CURRENT

    # Motor calibration
    print("  Running motor calibration...")
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.5)

    if axis.motor.error != 0:
        print(f"  ERROR: Motor calibration failed! Error: {hex(axis.motor.error)}")
        return False

    axis.motor.config.pre_calibrated = True
    print("  Motor calibration OK.")

    # Restore voltage limit
    axis.motor.config.resistance_calib_max_voltage = RESISTANCE_CALIB_MAX_VOLTAGE

    # Encoder calibration
    print("  Running encoder offset calibration...")
    axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.5)

    if axis.encoder.error != 0:
        print(f"  ERROR: Encoder calibration failed! Error: {hex(axis.encoder.error)}")
        return False

    axis.encoder.config.pre_calibrated = True
    print("  Encoder calibration OK.")

    return True


def main():
    print("═══════════════════════════════════════════")
    print("  ODrive ODESC Configuration Script")
    print("  Firmware v0.5.1 — Hoverboard Motors")
    print("═══════════════════════════════════════════")

    # Connect
    print("\nSearching for ODrive...")
    try:
        odrv = odrive.find_any(timeout=15)
        print(f"Connected! Serial: {hex(odrv.serial_number)}")
        print(f"Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
        print(f"Bus voltage: {odrv.vbus_voltage:.1f}V")
    except Exception as e:
        print(f"ERROR: Could not connect to ODrive: {e}")
        sys.exit(1)

    axes = [odrv.axis0, odrv.axis1]

    # Step 0: Clear errors
    clear_errors(axes)

    # Step 1: Configure
    print("\n[1/4] Applying configuration...")
    for i in CONFIGURE_AXES:
        configure_axis(axes[i], i)

    # Step 2: CAN baudrate
    print("\n[2/4] Setting CAN baudrate...")
    odrv.can.set_baud_rate(CAN_BAUD_RATE)
    print(f"  CAN baudrate set to {CAN_BAUD_RATE}")

    # Step 3: Save and reboot
    print("\n[3/4] Saving configuration and rebooting...")
    odrv.save_configuration()
    print("  Saved. Waiting for reboot...")
    time.sleep(3)

    # Reconnect
    try:
        odrv = odrive.find_any(timeout=15)
        print(f"  Reconnected! Serial: {hex(odrv.serial_number)}")
    except Exception as e:
        print(f"ERROR: Could not reconnect after reboot: {e}")
        sys.exit(1)

    axes = [odrv.axis0, odrv.axis1]

    # Step 4: Calibrate
    print("\n[4/4] Running calibration...")
    all_ok = True
    for i in CONFIGURE_AXES:
        ok = calibrate_axis(odrv, axes[i], i)
        if not ok:
            all_ok = False

    if not all_ok:
        print("\nERROR: Calibration failed on one or more axes. Check wiring and settings.")
        sys.exit(1)

    # Final save
    print("\nSaving calibration results...")
    odrv.save_configuration()
    time.sleep(3)

    odrv = odrive.find_any(timeout=15)
    axes = [odrv.axis0, odrv.axis1]

    # Verify
    print("\n═══════════════════════════════════════════")
    print("  Verification")
    print("═══════════════════════════════════════════")
    for i in CONFIGURE_AXES:
        ax = axes[i]
        print(f"\nAxis {i}:")
        print(f"  CAN node ID:        {ax.config.can_node_id}")
        print(f"  Motor calibrated:   {ax.motor.config.pre_calibrated}")
        print(f"  Encoder calibrated: {ax.encoder.config.pre_calibrated}")
        print(f"  Control mode:       {ax.controller.config.control_mode}")
        print(f"  Input mode:         {ax.controller.config.input_mode}")
        print(f"  Current limit:      {ax.motor.config.current_lim}A")
        print(f"  Vel gain:           {ax.controller.config.vel_gain}")
        print(f"  Vel ramp rate:      {ax.controller.config.vel_ramp_rate}")
        print(f"  Watchdog enabled:   {ax.config.enable_watchdog}")
        print(f"  Watchdog timeout:   {ax.config.watchdog_timeout}s")
        print(f"  Startup CL ctrl:    {ax.config.startup_closed_loop_control}")

    print("\n✓ Configuration complete. ODrive is ready for ROS2.")


if __name__ == "__main__":
    main()