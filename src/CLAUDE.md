# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.


You read claude.md, to get acquainted with the current state, then ask the user for the task. Don't initialize by starting a task without consent.

**Read `progress.md`** to see where we are. When you build code, test it before calling it done — on hardware when applicable. If hardware is not connected, ask.

## progress.md maintenance
- Current + previous session: full detail.
- Older sessions: collapse to one-paragraph summary.
- Backlog and Known Hardware Issues: always current.
- Delete "what remains / next steps" from completed entries. When adding a new session entry, collapse the one that is now two sessions old.

---

## Build & Run

All commands run from the workspace root (`/home/ivan/selena_ros_ws`), not from `src/`.

```bash
cd /home/ivan/selena_ros_ws && colcon build                        # build all
colcon build --packages-select <package>                           # single package
source install/setup.bash

colcon test --packages-select <package> && colcon test-result --verbose  # C++ tests
cd src/<package> && python3 -m pytest test/                              # Python tests

# Real robot (requires can0 + all hardware)
ros2 launch robot_bringup robot.launch.py

# Gazebo simulation (no hardware)
ros2 launch gazebo_sim gz_sim.launch.xml

# Web UI (port 8080) — starts rosbridge + webapp
ros2 launch robot_web_app teleop.launch.xml

# Mission server
ros2 run robot_missions mission_server \
  --ros-args --params-file install/robot_missions/share/robot_missions/config/tray_positions.yaml

# Send a planting goal
ros2 action send_goal /mission robot_missions/action/MissionAction \
  "{mission_type: 'plant', target: 'naut', quantity: 8.0, quantity_unit: 'seeds', start_x: 0.0, start_y: 0.0, start_yaw: 0.0}"
```

---

## Package Overview

| Package | Language | Role |
|---|---|---|
| `robot_description` | URDF/xacro | URDF, meshes, RViz config |
| `robot_bringup` | Python launch | Real robot launch + controller YAML |
| `odesc_hardware` | C++ | ros2_control plugin for ODrive/ODESC wheel motors via SocketCAN |
| `mks_servo_hardware` | C++ | ros2_control plugin for MKS servo stepper gantry via SocketCAN |
| `robot_gripper` | C++ | Standalone gripper node — Arduino servo via CAN |
| `robot_missions` | C++ | Mission framework: MissionBase, registry, PlantingMission, action server |
| `robot_field` | YAML/config | Single source of truth for field layout (strips, origin) |
| `robot_navigation` | C++ | NavigationProvider interface + OdometryNavigator; future Nav2Navigator |
| `robot_vision` | Python | Person detection + safety monitor |
| `robot_web_app` | JS/Python | Browser teleop + missions UI via rosbridge/roslibjs |
| `gazebo_sim` | URDF/XML | Gazebo Harmonic simulation (gz_ros2_control — same controllers as real robot) |

---

## Hardware Architecture

### CAN bus (`can0`)
All hardware shares one bus. CAN auto-starts via udev on PEAK-System USB-to-CAN adapter plug-in. See `mks_servo_hardware/can_connection_info.md` for udev details.
- **MKS stepper motors** (gantry): node IDs 1–4 (X1, X2, Y, Z)
- **ODrive ODESC motors** (wheels): node IDs 5–8 (LB, LF, RF, RB)
- **Gripper Arduino**: CAN ID `0x10` — standalone, not in ros2_control

### ros2_control plugins

**`mks_servo_hardware/GantryHardwareInterface`** (gantry X/Y/Z)
- X uses two motors (X1=ID1, X2=ID2). X1 is mechanically inverted — position/velocity negated in read/write.
- `on_activate`: mandatory homing — Z first (blocking), then X1+X2+Y in parallel.
- Two runtime modes: position (JTC) and velocity (gantry_velocity_controller); switched via `perform_command_mode_switch`.
- State/command layout: `[J0_pos, J0_vel, J1_pos, J1_vel, J2_pos, J2_vel]`.

**`odesc_hardware/OdescHardwareInterface`** (wheels) — velocity-only commands; CAN frames via `OdescDriver::on_can_frame`.

### Gantry coordinate mapping (critical)
The gantry is rotated 90° from `base_link`:
```
Mobile base +X (forward) = Gantry Y axis  (x_axis_joint, 0.845 m travel)
Mobile base +Y (left)    = Gantry X axis  (y_axis_joint, 0.290 m travel)
Mobile base +Z (up)      = Gantry Z axis  (z_axis_joint, 0.250 m travel, inverted)
```

### Gripper
Standalone node `/gripper/angle` (`Int32`, 0–55°) → CAN → Arduino → MG996R servo.  
**0° = open, 55° = closed.** Web app slider is inverted (0 = closed, 55 = open).

---

## Controllers

Configured in `robot_bringup/config/robot_controllers.yaml`, spawned by `robot.launch.py`.  
JTC is spawned **after** homing completes (via `--controller-manager-timeout 120`).

| Controller | Type | Default state | Controls |
|---|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | active | all joint states |
| `joint_trajectory_controller` | JointTrajectoryController | active | gantry X/Y/Z position |
| `diff_drive_controller` | DiffDriveController | active | 4 wheels velocity |
| `gantry_velocity_controller` | ForwardCommandController | **inactive** | gantry X/Y/Z velocity (teleop) |
| `gripper_controller` | ForwardCommandController | active | finger joints (sim only) |

JTC and gantry_velocity_controller cannot be active simultaneously.

---

## Gazebo Simulation

Same ros2_control controllers (JTC, DiffDrive, gantry_velocity, gripper) run against Gazebo physics. No CAN needed.
```bash
ros2 launch gazebo_sim gz_sim.launch.xml   # Terminal 1 — Gazebo + controllers + RViz
ros2 launch robot_web_app teleop.launch.xml # Terminal 2 — http://localhost:8080
```
Key files: `gazebo_sim/urdf/robot_gazebo.urdf.xacro`, `robot_description/urdf/ros2_control.sim.xacro`, `robot_gripper/src/gripper_sim_node.cpp`.

See `SIMULATION_STATUS.md` for Gazebo bring-up details, smoke-test commands, and bugs fixed during initial sim setup.

---

## Field Data (`robot_field`)

Single source of truth for all field geometry. Any ROS node that needs field layout adds `robot_field` as a `<depend>` and reads from its share directory via `ament_index_cpp::get_package_share_directory("robot_field")`.

**`config/field_config.yaml`** — static field layout: strips, origin (world frame x/y/yaw), strip_width (fixed 0.845 m = gantry X travel), strip_length, strip_spacing. Served by `GET/POST /api/field_config`.

**`config/mission_zones.yaml`** — session-persistent mission zones overlaid on strips. Each zone: `strip_id`, `start_along`/`end_along` (meters from strip start, snapped to 0.1 m), `mission_type`, `target`, `status`. Served by `GET/POST /api/mission_zones`. No zones may overlap on the same strip.

See `MAP_ARCHITECTURE.md` for the full field coordinate system, canvas rendering strategy, and future snake-path planning algorithm.

---

## Mission Framework

See `robot_missions/ARCHITECTURE.md` for geometry details. See `MAP_ARCHITECTURE.md` for field map and zone planner.

**Lifecycle:** `MissionBase`: `validate() → plan() → execute() → report()` + pause/resume/abort.  
**`MissionManager(node, nav)`** — navigates to start, creates mission via registry, drives lifecycle.  
**`MissionRegistry`** — string → factory map. Self-register with `REGISTER_MISSION` macro; add one file, zero other changes.  
**`mission_server`** — `rclcpp_action::Server<MissionAction>` on `/mission`. Feedback at 2 Hz.  
**`NavigationProvider`** (`robot_navigation`) — `get_pose()` + `move_to()`. `OdometryNavigator` is the impl (0.1 m/s, odom feedback). Swap to `Nav2Navigator` with one line in `mission_server_node.cpp`.

### PlantingMission internals
1. **SeedDatabase** — `seeds.csv` columns: `name, spacing_x_mm, spacing_y_mm, depth_mm, tool_id, tool_param`
2. **PlantingPlanner** — pure math; `get_seed_position(i)` / `get_robot_stop(i)`
3. **GripperTool** — `pick()` closes, `release()` opens; tray coords from `tray_positions.yaml` (must be measured physically)
4. **ExecutionEngine** — JTC only; `send_xy_command` / `send_z_command` are strict primitives (Z retracts before XY; two-point Z trajectory: 400ms velocity-damp settle then stroke, prevents cubic spline ghost movement on X/Y); `FlatGroundDepthSensor` active

### Registering a new mission
```cpp
// Bottom of your_mission.cpp:
namespace robot_missions { REGISTER_MISSION("your_type", YourMission); }
// Also #include the header in mission_server_node.cpp to force linker inclusion.
```

### Active stubs
- `tray_positions.yaml` — tray coords are test values; measure physically before real run
- `FlatGroundDepthSensor` — real sensor not mounted
- `SafetyChecker` — not implemented (will subscribe `/person_detected`)
- VacuumTool — tool_id=2 will error until implemented

### Roadmap (see `MAP_ARCHITECTURE.md` §7)
- **Step 5** — multi-zone / snake path: queue zones across strips, sequential goals
- **Step 6** — `Nav2Navigator : NavigationProvider` (one file, one line swap in `mission_server_node.cpp`)
- **Step 7** — RTK / absolute coordinates replacing odom frame

---

## Web App

Port 8080, rosbridge at `ws://<host>:9090`.

**`index.html` / `app.js`** — teleop: `/diff_drive_controller/cmd_vel`, `/joint_trajectory_controller/joint_trajectory`, `/gantry_velocity_controller/commands`, `/gripper/angle`. JTC trajectory time: `1.5 × dist / hw_cap` per axis (X=0.15, Y=0.06, Z=0.04 m/s).

**`missions.html` / `missions.js` / `missions.css`** — field canvas map, zone planner (2-click placement, overlap validation, seed stats), zone list with ▶ Start / ■ Cancel per zone. Sends goals via `send_action_goal` rosbridge op; feedback via `socket.addEventListener`. HTTP API: `/api/field_config`, `/api/mission_zones`.

**`nav.js`** — shared hamburger/drawer for both pages.

---

## Safety

`robot_vision/safety_monitor_node.py` publishes zero `TwistStamped` to `/diff_drive_controller/cmd_vel` at 20 Hz while `/person_detected` is `True`. Future: `SafetyChecker` in `ExecutionEngine` will pause base movement.

---

## Known Operational Issues

**ODESC wheels not on CAN** — `candump can0` shows no traffic from node IDs 5–8. Gantry (1–4) fine. Check CAN connector on ODESC boards.

**X1 motor CAN reliability** — Intermittently missed CAN commands during homing; X1/X2 sync faults observed. Check CAN connector on X1 (ID=1), consider power-cycling.

**autoDetectDirection() not implemented** — All 4 gantry motors default to `direction_multiplier_=1.0`; `0x90` query timing not resolved. Investigate before adding a 5th motor or replacing a motor.
