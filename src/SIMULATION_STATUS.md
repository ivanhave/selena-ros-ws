# Offline Simulation Stack — Status

## What was built

A full offline simulation where the real ros2_control controllers (JTC, DiffDrive,
gantry_velocity, gripper) run against Gazebo Harmonic physics via gz_ros2_control.
No CAN hardware required. Mission code can be tested end-to-end using the same
ROS2 topics as the real robot.

## How to run

```bash
cd /home/ivan/selena_ros_ws
source install/setup.bash

# Terminal 1 — Gazebo + all controllers + RViz
ros2 launch gazebo_sim gz_sim.launch.xml

# Terminal 2 — Web interface (rosbridge + HTTP server)
ros2 launch robot_web_app teleop.launch.xml
```

Web app: http://localhost:8080
Rosbridge: ws://localhost:9090

## Controllers (all in gz_sim.launch.xml)

| Controller              | State    | Topic / Action                                        |
|-------------------------|----------|-------------------------------------------------------|
| joint_state_broadcaster | active   | publishes /joint_states                               |
| joint_trajectory_controller | active | /joint_trajectory_controller/follow_joint_trajectory |
| diff_drive_controller   | active   | /diff_drive_controller/cmd_vel (TwistStamped)         |
| gripper_controller      | active   | /gripper_controller/commands (Float64MultiArray)      |
| gantry_velocity_controller | inactive (by design) | activate for manual teleop          |

## Quick smoke tests

```bash
# Gripper open/close
ros2 topic pub --once /gripper/angle std_msgs/msg/Int32 "{data: 55}"   # open
ros2 topic pub --once /gripper/angle std_msgs/msg/Int32 "{data: 0}"    # close

# Mobile base
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.3}, angular: {z: 0.0}}}"

# Gantry
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [x_axis_joint, y_axis_joint, z_axis_joint], \
  points: [{positions: [0.1, 0.05, 0.05], time_from_start: {sec: 3}}]}}"
```

## Files created / modified

### NEW
- `robot_description/urdf/gripper.xacro`
  Two-finger parallel-jaw gripper attached to z_axis_link at xyz="0 0 0.55".
  L-shaped jaws (vertical arm + inward tip). finger_stroke=0.020m, arm_offset_y=0.025m.
  No YAML-breaking comments (colon-space in URDF comments breaks ros2 launch YAML parser).

- `robot_description/urdf/ros2_control.sim.xacro`
  Replaces real CAN hardware with gz_ros2_control/GazeboSimSystem.
  Joints: x/y/z_axis (position+velocity), 4 wheels (velocity), left/right_finger (position).

- `robot_gripper/src/gripper_sim_node.cpp`
  Subscribes /gripper/angle (Int32).
  Publishes /gripper_controller/commands (Float64MultiArray) → Gazebo position controller.
  Publishes /gripper/current_angle (Int32), /gripper/is_open (Bool).
  use_sim_time=true (set via launch param) so joint state timestamps match RSP.

### MODIFIED
- `robot_description/urdf/selena_robot.urdf.xacro`
  Added gripper.xacro include (so real robot also shows gripper in RViz).

- `gazebo_sim/urdf/robot_gazebo.urdf.xacro`
  Replaced native Gz plugins (DiffDrive, JointPositionController×3, JointStatePublisher)
  with gz_ros2_control::GazeboSimROS2ControlPlugin pointing at robot_controllers.yaml.
  Includes gripper.xacro and ros2_control.sim.xacro.

- `gazebo_sim/config/gazebo_bridge.yaml`
  Simplified to only /clock bridge (gz_ros2_control owns everything else).

- `gazebo_sim/launch/gz_sim.launch.xml`
  Added controller spawners (JSB, JTC, DiffDrive, gantry_velocity --inactive,
  gripper_controller), gripper_sim_node with use_sim_time=true, RViz.
  JTC spawner uses --switch-timeout 60 (not --activate-given-list-timeout which doesn't exist).

- `robot_bringup/config/robot_controllers.yaml`
  Added gripper_controller (ForwardCommandController, position interface,
  joints: left_finger_joint + right_finger_joint).

- `robot_gripper/CMakeLists.txt`
  Added gripper_sim_node target; removed sensor_msgs dependency (no longer needed).

- `gazebo_sim/package.xml`
  Added deps: robot_bringup, gz_ros2_control, robot_gripper.

## Key bugs fixed along the way

1. YAML parse failure: ros2 launch YAML-parses robot_description param value.
   Any `: ` (colon-space) in URDF XML comments breaks it. Removed all such comments.

2. JTC timeout: --activate-given-list-timeout doesn't exist. Use --switch-timeout 60.

3. Finger TF errors in RViz: gripper_sim_node was using wall-clock timestamps but
   robot_state_publisher uses sim time. Fixed by use_sim_time=true on gripper_sim_node.

4. Gripper sliding in Gazebo: finger joints had no controller, floated as passive joints.
   Fixed by adding them to ros2_control.sim.xacro and spawning gripper_controller.

5. QoS mismatch (early version): gripper_sim_node /joint_states was VOLATILE while
   joint_state_broadcaster was TRANSIENT_LOCAL. Resolved when we moved to controller
   approach (gripper_sim_node no longer publishes /joint_states at all).

## Prerequisite (already installed)

```bash
sudo apt install ros-jazzy-gz-ros2-control
```
