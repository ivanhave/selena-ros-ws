# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

All commands run from the workspace root (`/home/ivan/selena_ros_ws`), not from `src/`.

```bash
# Build all packages
cd /home/ivan/selena_ros_ws && colcon build

# Build a single package
colcon build --packages-select <package_name>

# Source after building
source install/setup.bash

# Run C++ tests (colcon)
colcon test --packages-select <package_name>
colcon test-result --verbose

# Run Python lint tests
cd src/<package> && python3 -m pytest test/

# Launch real robot (requires can0 and all hardware)
ros2 launch robot_bringup robot.launch.xml

# Launch Gazebo simulation (no hardware needed)
ros2 launch gazebo_sim gz_sim.launch.xml

# View robot in RViz only
ros2 launch robot_description display.launch.xml

# Start web teleop UI (requires rosbridge running separately)
ros2 run robot_web_app serve_webapp
ros2 run rosbridge_server rosbridge_websocket

# Run mission server (real robot — load tray coordinates from YAML)
ros2 run robot_missions mission_server \
  --ros-args --params-file install/robot_missions/share/robot_missions/config/tray_positions.yaml

# Send a planting goal
ros2 action send_goal /mission robot_missions/action/MissionAction \
  "{mission_type: 'plant', target: 'naut', quantity: 1.0, quantity_unit: 'meters', start_x: 0.0, start_y: 0.0}"
```

## Package Overview

| Package | Language | Role |
|---|---|---|
| `robot_description` | URDF/xacro | URDF, meshes, RViz config |
| `robot_bringup` | XML launch | Real robot launch + controller YAML |
| `odesc_hardware` | C++ | ros2_control plugin for ODrive/ODESC wheel motors via SocketCAN |
| `mks_servo_hardware` | C++ | ros2_control plugin for MKS servo stepper gantry via SocketCAN |
| `robot_gripper` | C++ | Standalone gripper node — Arduino servo via CAN |
| `robot_missions` | C++ | Mission framework: MissionBase, self-registration registry, PlantingMission, action server |
| `robot_vision` | Python | Person detection + safety monitor |
| `robot_web_app` | JS/Python | Browser teleop UI via rosbridge/roslibjs |
| `gazebo_sim` | URDF/XML | Gazebo Harmonic simulation (gz_ros2_control — same controllers as real robot) |

## Hardware Architecture

### All hardware uses SocketCAN (`can0`)

Three device groups share the same CAN bus:
- **MKS stepper motors** (gantry): node IDs 1–4 (X1, X2, Y, Z)
- **ODrive ODESC hoverboard motors** (wheels): node IDs 5–8 (LB, LF, RF, RB)
- **Gripper Arduino**: CAN ID `0x10` — standalone, not in ros2_control

CAN auto-starts via udev rule on PEAK-System USB-to-CAN adapter plug-in.
See `mks_servo_hardware/can_connection_info.md` for udev rule details and manual fallback commands.

### ros2_control Hardware Interfaces

Two plugins loaded by `controller_manager`:

**`mks_servo_hardware/GantryHardwareInterface`** (gantry X/Y/Z)
- X axis uses **two physical motors** (X1=ID1, X2=ID2 in parallel). X1 is mechanically inverted — its position and velocity are negated in read/write.
- `on_activate` runs a mandatory homing sequence: **Z first** (blocking), then X1+X2+Y in parallel.
- Supports two control modes switched at runtime: position mode (JTC) and velocity mode (gantry_velocity_controller). Mode switch is detected in `perform_command_mode_switch`.
- State/command vector layout: `[J0_pos, J0_vel, J1_pos, J1_vel, J2_pos, J2_vel]` — joint i at index `[i*2]` and `[i*2+1]`.

**`odesc_hardware/OdescHardwareInterface`** (wheels)
- Each wheel joint has a `can_node_id` and optional `invert` param from URDF.
- Velocity-only command interface; position and velocity state interfaces.
- CAN frame parsing routed via callback: `CanInterface` → `OdescDriver::on_can_frame`.

### Gantry Coordinate Mapping (critical — from ARCHITECTURE.md)

The gantry is rotated 90° from `base_link`:
```
Mobile base X (forward) = Gantry Y axis (x_axis_joint, 0.845m travel)
Mobile base Y (width)   = Gantry X axis (y_axis_joint, 0.290m travel)
Mobile base Z (up)      = Gantry Z axis (z_axis_joint, 0.250m travel, inverted)
```

### Gripper

`robot_gripper` is a standalone ROS2 node (not a ros2_control plugin):
- Subscribe `/gripper/angle` (`Int32`, 0–55 degrees) → sends CAN frame to Arduino → waits for echo confirmation.
- Publish `/gripper/current_angle` (`Int32`) and `/gripper/is_open` (`Bool`).
- Arduino source: `robot_gripper/arduino/gripper_controller/gripper_controller.ino`.
- Physical: MG996R servo on circular-finger mechanism. **0° = fully open, 55° = fully closed.**
- Web app convention: slider 0 = closed (physical 55°), slider 55 = open (physical 0°) — inverted.

## Controllers

Configured in `robot_bringup/config/robot_controllers.yaml`, spawned in `robot.launch.xml`:

| Controller | Type | Default state | Controls |
|---|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | active | publishes all joint states |
| `joint_trajectory_controller` | JointTrajectoryController | active | gantry X/Y/Z position (splines) |
| `diff_drive_controller` | DiffDriveController | active | 4 wheels velocity |
| `gantry_velocity_controller` | ForwardCommandController | **inactive** | gantry X/Y/Z velocity (manual teleop) |
| `gripper_controller` | ForwardCommandController | active | left/right finger joints (sim only) |

Switch between `joint_trajectory_controller` and `gantry_velocity_controller` at runtime — they cannot be active simultaneously (enforced in `prepare_command_mode_switch`).

## Gazebo Simulation

See `SIMULATION_STATUS.md` for full details and file change log.

`gazebo_sim` now uses **`gz_ros2_control`** — the same real ros2_control controllers
(JTC, DiffDrive, gantry_velocity, gripper_controller) run against Gazebo physics.
No CAN hardware required. Mission code runs end-to-end against the same ROS2 topics as the real robot.

```bash
# Terminal 1 — Gazebo + all controllers + RViz
ros2 launch gazebo_sim gz_sim.launch.xml

# Terminal 2 — Web interface
ros2 launch robot_web_app teleop.launch.xml
# Web app: http://localhost:8080
```

Key files:
- `gazebo_sim/urdf/robot_gazebo.urdf.xacro` — uses `gz_ros2_control::GazeboSimROS2ControlPlugin`
- `robot_description/urdf/ros2_control.sim.xacro` — replaces CAN hardware with `GazeboSimSystem`
- `robot_gripper/src/gripper_sim_node.cpp` — bridges `/gripper/angle` → `/gripper_controller/commands`

## Mission Framework

See `robot_missions/ARCHITECTURE.md` for full geometry and design details.

### Core components

`MissionBase` defines the lifecycle interface: `validate() → plan() → execute() → report()` plus `pause/resume/abort/get_progress/get_current_step`.

`MissionManager(node)` is type-agnostic — navigates to start (stub), creates mission via registry, drives lifecycle.

`MissionRegistry` — singleton map from `mission_type` string → factory. Missions self-register via `REGISTER_MISSION` macro at static-init time. **Adding a new mission = one new file, zero changes to existing files.**

`mission_server` — `rclcpp_action::Server<MissionAction>` executable. Serves all mission types on action topic `/mission`. Runs `manager.run()` in a thread; feedback timer at 2 Hz.

### PlantingMission internal pieces
1. **SeedDatabase** — reads `config/seeds.csv`
   - Columns: `name, spacing_x_mm, spacing_y_mm, depth_mm, tool_id, tool_param`
   - `tool_param` is generic: gripper reads it as grip angle (degrees); future VacuumTool reads it as suction ms
2. **PlantingPlanner** — pure math, no ROS2, computes `get_seed_position(index)` and `get_robot_stop(index)` on demand
3. **ToolBase / GripperTool** — abstract tool interface; `pick()` closes gripper, `release()` opens
   - `GripperTool` reads tray coordinates from node params loaded via `config/tray_positions.yaml`
   - Tray coords currently all 0.0 — **must be measured physically before real run**
4. **ExecutionEngine** — owns ROS2 clients for JTC and diff_drive; runs per-stop sequence
   - Optional `DepthSensor*` — nullptr until hardware mounted (uses fixed depth from CSV)
   - Optional `SafetyChecker*` — planned, not yet implemented (will subscribe to `/person_detected`)

### Registering a new mission
```cpp
// At bottom of your_mission.cpp, inside namespace robot_missions:
namespace robot_missions {
REGISTER_MISSION("your_type", YourMission);
}
// Also #include your header in mission_server_node.cpp to force linker to include the TU
```

### Known stubs / TODOs
- `config/tray_positions.yaml` — all 0.0, measure physically
- `navigate_to_start()` — always returns true (Nav2 integration future)
- `DepthSensor` — nullptr, soil depth assumed flat
- `SafetyChecker` — not implemented (camera-based person check before base movement)
- VacuumTool — tool_id=2 in seeds.csv will error until implemented

## Safety

`robot_vision/safety_monitor_node.py` subscribes to `/person_detected` (`Bool`). When `True`, publishes zero `TwistStamped` to `/diff_drive_controller/cmd_vel` at 20 Hz until person is gone.

Future: `SafetyChecker` interface in `ExecutionEngine` will pause base movement during missions when `/person_detected` is True.

## Web App

Served at port 8000 (via `serve_webapp.py`), connects to rosbridge at `ws://<host>:9090`. Key topics published from browser:
- `/diff_drive_controller/cmd_vel` (`TwistStamped`)
- `/joint_trajectory_controller/joint_trajectory` (`JointTrajectory`)
- `/gantry_velocity_controller/commands` (`Float64MultiArray`)
- `/gripper/angle` (`Int32`)

Speed limits are constants at the top of `webapp/app.js` (`BASE_MAX_SPEED_MS`, `GANTRY_MAX_SPEED_MS`).
