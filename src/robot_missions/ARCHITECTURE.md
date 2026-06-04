# Selena Robot — Mission Architecture
## Decision Log and Strategy Document
*Update this document when decisions change.*

---

## 1. Robot Hardware Reference

### Coordinate Mapping (from URDF)
```
base_footprint → base_link (z up 0.4125m)
base_link      → gantry_origin_link
    origin xyz="${y_axis_travel/2} ${-x_axis_travel/2} 0"
    rpy="0 0 ${pi/2}"
```

Critical mapping — gantry is rotated 90° from base_link:
```
Mobile base X (forward travel)  = Gantry Y axis (x_axis_joint, 0.845m travel)
Mobile base Y (width)           = Gantry X axis (y_axis_joint, 0.290m travel)
Mobile base Z (up)              = Gantry Z axis (z_axis_joint, 0.250m travel, inverted)
```

### Gantry Limits
```
x_axis_joint: 0.0 to 0.845m   (across robot travel, robot Y direction)
y_axis_joint: 0.0 to 0.290m   (along robot travel, robot X direction)
z_axis_joint: 0.0 to 0.250m   (depth, downward)
```

### Gantry Origin in Base Frame
```
x = y_axis_travel / 2  = 0.145m from base center
y = -x_axis_travel / 2 = -0.4225m from base center
```

---

## 2. Mission Framework Architecture

### Design Principle
Modular, scalable, tool-agnostic. Inspired by ros2_control plugin architecture.
Adding a new mission type = one new file with `REGISTER_MISSION` macro. Nothing else changes.

### Abstract Mission Interface (MissionBase)
Every mission type implements:
```
validate()   — is the command valid and safe to execute?
plan()       — compute what needs to be done (no hardware)
execute()    — carry out the plan using hardware
pause()      — stop safely mid-mission, preserve state
resume()     — continue from where paused
abort()      — stop immediately and recover to safe state
report()     — return what was accomplished
get_progress()      — float 0.0–1.0 for action feedback
get_current_step()  — string description for action feedback
```

### MissionCommand (in mission_base.hpp)
```cpp
struct MissionCommand {
    std::string mission_type;   // "plant", "water", "weed"
    std::string target;         // "naut", "pea", "all_plants"
    float       quantity;       // 5.0
    std::string quantity_unit;  // "meters", "seeds", "liters"
    float       start_x;        // world coordinates (odometry / future RTK)
    float       start_y;
    float       start_yaw;      // world heading in radians (0 = robot faces +X)
};
```

### Mission Registry (self-registration)
`MissionRegistry` is a singleton map from mission_type string → factory function.
`MissionManager` calls `MissionRegistry::instance().create(cmd.mission_type, node, cmd, nav)` —
it has zero knowledge of specific mission classes.

Each mission class registers itself at static-init time by placing at the bottom of its `.cpp`:
```cpp
namespace robot_missions {
REGISTER_MISSION("plant", PlantingMission);
}
```

The `REGISTER_MISSION` macro creates a static `MissionRegistrar` object whose constructor
calls `register_mission()` before `main()` runs. Token-pasting note: macro must be used
inside a namespace where `ClassName` is unqualified (no `::` in `##ClassName`).

### Mission Types
```
MissionBase (abstract)
    ├── PlantingMission    ← implemented
    ├── WeedingMission     ← placeholder (include/robot_missions/weeding/)
    ├── WateringMission    ← future
    └── MonitoringMission  ← future
```

### Mission Manager (generic orchestrator)
`MissionManager(rclcpp::Node::SharedPtr node, shared_ptr<NavigationProvider> nav)`
No knowledge of specific mission types or navigation implementation.
Responsibilities:
- Receive MissionCommand
- Navigate robot to start position via NavigationProvider
- Instantiate mission via registry (passes nav through to mission)
- Call validate → plan → execute → report
- Proxy pause/resume/abort/get_progress/get_current_step to active mission

### ROS2 Action Interface
One action server (`mission_server` executable) handles all mission types.

```
action/MissionAction.action
    Goal:
        string  mission_type
        string  target
        float32 quantity
        string  quantity_unit
        float32 start_x       # world X (odometry/RTK)
        float32 start_y       # world Y
        float32 start_yaw     # world heading in radians; 0 = robot faces +X, pi/2 = +Y
    Result:
        bool    success
        string  report
        uint32  items_completed
        float32 distance_covered
    Feedback:
        float32 progress_percent
        string  current_step
        uint32  items_completed_so_far
        float32 current_robot_x
        float32 current_robot_y
```

Send a goal:
```bash
ros2 action send_goal /mission robot_missions/action/MissionAction \
  "{mission_type: 'plant', target: 'naut', quantity: 8.0, quantity_unit: 'seeds', start_x: 0.0, start_y: 0.0, start_yaw: 0.0}"
```

`start_yaw` is the robot's required heading at the start of the mission in world frame radians.
`MissionManager::navigate_to_start()` passes all three fields directly to `NavigationProvider::move_to(Pose2D)`.
If omitted in the goal, the ROS2 IDL default is `0.0` (faces odometry +X axis).

---

## 3. Planting Mission — Internal Architecture

### The Five Pieces

#### Piece 0 — PlantingMission : MissionBase
Implements MissionBase for planting. Owns and coordinates Pieces 1–4.
Registered with `REGISTER_MISSION("plant", PlantingMission)`.
Has `static create_from_command(node, cmd, nav)` — loads db_path via ament_index,
sets seed type from `cmd.target`, quantity from `cmd.quantity`, stores nav for ExecutionEngine.

#### Piece 1 — Seed Database (`config/seeds.csv`)
Static CSV file. Six columns:
```
name, spacing_x_mm, spacing_y_mm, depth_mm, tool_id, tool_param
naut, 500, 250, 30, 1, 35
pea,  200, 150, 25, 1, 30
```
- `spacing_x_mm` — across robot travel (gantry X axis direction)
- `spacing_y_mm` — along robot travel (gantry Y axis direction)
- `tool_id` — 1=gripper, 2=vacuum (future)
- `tool_param` — generic tool parameter; gripper reads it as grip angle in degrees;
  future VacuumTool will read it as suction-on duration in milliseconds

#### Piece 2 — Planting Planner (pure math, no ROS2, no hardware)
Input: seed_profile, strip_length_m or seed_count, gantry travel limits.

Provides on-demand functions (no memory storage of all coordinates):
```
get_seed_position(seed_index) → GantryCommand {gx, gy, depth}
get_robot_stop(seed_index)    → MobileCommand {advance_meters, stop_index}
get_total_seeds()             → int
get_total_stops()             → int
```

#### Piece 3 — Tool Interface (abstract, swappable)
`ToolBase` (in `planting/tool_interface.hpp`) — planting-specific abstract interface:
```cpp
virtual void pick()    = 0;   // grab one seed from tray
virtual void release() = 0;   // deposit seed at plant location
virtual void configure(const SeedProfile &) = 0;
virtual TrayPose get_tray_pose() const = 0;

struct TrayPose { float gx, gy, z_approach, z_pick; };
```

`GripperTool : ToolBase` — reads tray coordinates from ROS2 node parameters
(`gripper_tray.gx/gy/z_approach/z_pick`), configured via `config/tray_positions.yaml`.
Uses `profile.tool_param` as grip angle (degrees). MG996R servo: 0°=open, 55°=closed.
Gripper fingers move in an arc (circular mechanism) — calibration per seed type matters.

`VacuumTool` — future (tool_id=2). Will use `profile.tool_param` as suction ms.

Note: `ToolBase` is a planting concept. Other missions (weeding, watering) will have their
own actuator interfaces that do NOT inherit `ToolBase`.

#### Piece 4 — Execution Engine (gantry orchestrator)
Knows: ROS2, JTC trajectories, gripper topic.
Knows nothing about: seed spacing, tool type, global coordinates, navigation implementation.

Injected dependencies:
- `shared_ptr<NavigationProvider>` — moves the base between stops; missions call get_pose() / move_to()
- `DepthSensor*` — measures soil distance; nullptr = use fixed depth from seeds.csv
- `SafetyChecker*` — checks before base movement; nullptr = no check (planned, not implemented)

Sequence per robot stop:
```
nav->move_to(next_stop_pose)   ← computed from current pose + advance_meters * forward
for each seed in this stop:
    move_gantry(tray_gx, tray_gy, tray_z_approach)
    lower_z(tray_z_pick)
    Tool.pick()
    retract_z()
    move_gantry(seed.gx, seed.gy)
    lower_z(soil_z + seed.depth)  ← soil_z from DepthSensor or 0.0 if nullptr
    Tool.release()
    retract_z()
```

---

## 4. Planting Geometry Formulas

### Seeds per robot stop
```
seeds_in_x    = floor(x_travel / spacing_x) + 1
seeds_in_y    = floor(y_travel / spacing_y) + 1
seeds_per_stop = seeds_in_x * seeds_in_y
```

### Centering offset
```
x_offset = (x_travel - (seeds_in_x - 1) * spacing_x) / 2
y_offset = (y_travel - (seeds_in_y - 1) * spacing_y) / 2
```

### Seed position within a stop (gantry coordinates)
```
col = seed_local_index % seeds_in_x
row = seed_local_index / seeds_in_x
gx  = x_offset + col * spacing_x
gy  = y_offset + row * spacing_y
```

### Robot advance between stops
```
robot_advance = seeds_in_y * spacing_y
```

### Verified example (naut, hardware-tested 2026-05-31)
```
x_travel=0.845, spacing_x=0.60 → seeds_in_x=2, x_offset=0.1225m
y_travel=0.290, spacing_y=0.20 → seeds_in_y=2, y_offset=0.045m
robot_advance = 2 * 0.20 = 0.40m

Stop 0: rows at world_y = 0.045m and 0.245m
Robot moves 0.40m (via NavigationProvider)
Stop 1: rows at world_y = 0.445m and 0.645m
Gap between stops: 0.445 - 0.245 = 0.20m = spacing_y ✓
```

---

## 5. Depth Handling

### Current (DepthSensor = nullptr)
```cpp
float soil_z = 0.0f;  // depth sensor not mounted yet
lower_z(soil_z + seed.depth_m);
```
Assumes Z-home is at a known fixed height above soil. This is safe for flat test surfaces.
In production on bumpy field soil, Z will need the real sensor.

### DepthSensor interface (`include/robot_missions/planting/depth_sensor.hpp`)
```cpp
class DepthSensor {
public:
    virtual ~DepthSensor() = default;
    virtual float measure_soil_distance_m() = 0;
    // Returns distance from gantry Z-home to soil surface.
    // ExecutionEngine formula: lower_z(sensor->measure() + seed.depth_m)
};
```

### Future integration steps
1. Mount sensor on gantry Z end-effector
2. Create `LidarDepthSensor : DepthSensor` (or ultrasonic, etc.)
3. Pass instance to `ExecutionEngine` constructor — zero other changes
4. Update seeds.csv `depth_mm` to real planting depth (currently placeholder values)

---

## 6. Seed Tray

Fixed to gantry frame. Not in URDF. Tray coordinates live in ROS2 node parameters,
loaded at runtime from `config/tray_positions.yaml`. GripperTool reads them via:
```cpp
node->declare_parameter("gripper_tray.gx",       0.0);
node->declare_parameter("gripper_tray.gy",       0.0);
node->declare_parameter("gripper_tray.z_approach", 0.0);
node->declare_parameter("gripper_tray.z_pick",   0.0);
```

Launch with:
```bash
ros2 run robot_missions mission_server \
  --ros-args --params-file install/robot_missions/share/robot_missions/config/tray_positions.yaml
```

`TrayPose` struct (in `planting/tool_interface.hpp`):
```
gx          — gantry X position of tray pick point
gy          — gantry Y position of tray pick point
z_approach  — safe Z height to move above tray before descending
z_pick      — Z depth to descend to when picking seed
```

All values currently 0.0 — physically measure and fill before first real run.

---

## 7. Navigation Architecture

### NavigationProvider (package: `robot_navigation`)

All mobile base positioning goes through a single abstract interface. Missions have zero
knowledge of odometry, Nav2, or any other implementation.

```cpp
struct Pose2D { float x, y, yaw; };

class NavigationProvider {
    virtual Pose2D get_pose()        = 0;  // where is the robot now
    virtual bool   move_to(Pose2D)   = 0;  // go there, block until done
};
```

The concrete implementation is created once in `mission_server_node.cpp` and injected
into `MissionManager`, which passes it through the registry to every mission and down
to `ExecutionEngine`. Swapping implementations is one line:

```cpp
// Current
auto nav = std::make_shared<OdometryNavigator>(node);

// Future — Nav2
auto nav = std::make_shared<Nav2Navigator>(node);
```

### OdometryNavigator (current implementation)
Package: `robot_navigation/src/odometry_navigator.cpp`
- Subscribes to `/diff_drive_controller/odom` for pose and velocity
- `move_to(target)`: publishes 0.1 m/s cmd_vel, polls odometry distance, then polls
  velocity until stopped (< 0.01 m/s) — identical behaviour to previous ExecutionEngine code
- `get_pose()`: returns current `{odom_x, odom_y, odom_yaw}`

### Nav2Navigator (future)
- `move_to(target)`: sends `NavigateToPose` action goal, blocks on result
- `get_pose()`: reads from AMCL / `/tf`
- No changes needed anywhere else

### Localization evolution
```
Now:    OdometryNavigator   → wheel encoders only
Next:   Nav2Navigator       → AMCL + map + obstacle avoidance
Future: Nav2Navigator + RTK → centimetre-accurate field positioning
```
Each step = replace one class, zero mission changes.

---

## 8. First Version Scope
- Single strip, one direction, no turning
- Seed type: naut/chickpea (hardware-tested: 8 seeds, 2 stops, 600×200mm)
- Tool: gripper (MG996R, 0°=open, 55°=closed)
- Depth: fixed from seeds.csv (FlatGroundDepthSensor, soil_z from tray_positions.yaml)
- Navigation: OdometryNavigator (wheel odometry, no Nav2)
- Command: `mission_type="plant"  target="naut"  quantity=8.0  quantity_unit="seeds"  start_yaw=0.0`

---

## 9. Package Structure

### robot_navigation (new — infrastructure, no mission knowledge)
```
robot_navigation/
├── CMakeLists.txt
├── package.xml
├── include/robot_navigation/
│   ├── navigation_provider.hpp    ← Pose2D struct + NavigationProvider abstract interface
│   └── odometry_navigator.hpp     ← OdometryNavigator declaration
└── src/
    └── odometry_navigator.cpp     ← current implementation (odom + cmd_vel)
```

### robot_missions (depends on robot_navigation)
```
robot_missions/
├── ARCHITECTURE.md
├── CMakeLists.txt
├── package.xml
├── action/
│   └── MissionAction.action
├── config/
│   ├── seeds.csv
│   └── tray_positions.yaml        ← gripper tray ROS2 params + soil_surface_z_m
├── include/robot_missions/
│   ├── mission_base.hpp           ← MissionBase lifecycle + MissionCommand + MissionResult
│   ├── mission_manager.hpp        ← orchestrator; takes NavigationProvider
│   ├── mission_registry.hpp       ← singleton registry + REGISTER_MISSION macro
│   ├── planting/
│   │   ├── planting_mission.hpp
│   │   ├── seed_database.hpp
│   │   ├── planting_planner.hpp
│   │   ├── tool_interface.hpp     ← ToolBase abstract + TrayPose struct
│   │   ├── gripper_tool.hpp
│   │   ├── execution_engine.hpp   ← gantry only; NavigationProvider + DepthSensor* injected
│   │   ├── depth_sensor.hpp       ← DepthSensor abstract interface
│   │   └── flat_ground_depth_sensor.hpp
│   └── weeding/
│       └── weeding_mission.hpp    ← placeholder with full design notes
├── src/
│   ├── mission_manager.cpp
│   ├── mission_registry.cpp
│   ├── mission_server_node.cpp    ← creates OdometryNavigator; rclcpp_action server
│   └── planting/
│       ├── planting_mission.cpp   ← contains REGISTER_MISSION("plant", PlantingMission)
│       ├── seed_database.cpp
│       ├── planting_planner.cpp
│       ├── gripper_tool.cpp
│       └── execution_engine.cpp
└── test/
    └── planting_planner_test.cpp  ← 5 gtests (8-seed / 2-stop geometry)
```

---

## 10. Known TODOs / Stubs

| Item | File | Status |
|---|---|---|
| Tray coordinates | `config/tray_positions.yaml` | Filled for floor test (0.800/0.200/0.220m) — re-measure when tray physically mounted |
| naut/pea depth_mm | `config/seeds.csv` | naut=0 (surface test) — measure real planting depth |
| **NavigationProvider hardware test** | `robot_navigation/` | Built and smoke-tested; full 8-seed run pending hardware power-on |
| Nav2Navigator | `robot_navigation/` | Not yet implemented — replace OdometryNavigator when Nav2 is set up |
| DepthSensor | `execution_engine.cpp` | FlatGroundDepthSensor (configured soil_z) — real sensor not mounted |
| SafetyChecker | `execution_engine.hpp` | Not yet implemented — person check before base move |
| VacuumTool | planned | tool_id=2 in seeds.csv will error until implemented |
| WeedingMission | `weeding/weeding_mission.hpp` | Placeholder only |
| Batch seed picking | — | Currently one seed per tray visit |
| Turning between strips | — | Not designed yet |
| ODESC wheels CAN | hardware | No traffic on node IDs 5–8 — check CAN connector |

---

## 11. How to Add a New Mission Type

1. Create `include/robot_missions/<name>/<name>_mission.hpp` — inherit `MissionBase`.
2. Create `src/<name>/<name>_mission.cpp` — implement all lifecycle methods.
3. Add a static factory with the NavigationProvider parameter:
   ```cpp
   static std::unique_ptr<YourMission> create_from_command(
       rclcpp::Node::SharedPtr node,
       const MissionCommand & cmd,
       std::shared_ptr<robot_navigation::NavigationProvider> nav);
   ```
4. At the bottom of the `.cpp`, inside `namespace robot_missions`:
   ```cpp
   namespace robot_missions {
   REGISTER_MISSION("your_type", YourMission);
   }
   ```
5. Add the `.cpp` to `CMakeLists.txt` target sources.
6. `#include` the mission header in `mission_server_node.cpp` to force static registration
   to link even when the linker would otherwise dead-strip the TU.

Missions that don't move the base can accept and ignore the `nav` parameter.
Zero changes to `MissionBase`, `MissionManager`, `MissionRegistry`, or `mission_server_node.cpp`
(other than the include).

---

## 12. First Physical Test Checklist (floor-to-floor 1-seed test)

Goal: pick a chickpea seed from a floor point (simulating tray) and deposit it at another
floor point (simulating planting hole).

### Pre-test measurements needed
1. **floor_z_m** — home gantry, manually lower Z until fingertips touch floor, read `z_axis_joint`
   value from `/joint_states`. This becomes both `z_pick` (for tray) and `depth_mm` (for planting).
2. **tray gx/gy** — manually position gantry over seed pick point, read `x_axis_joint` and
   `y_axis_joint` from `/joint_states`.
3. **z_approach** — a safe hover height above the floor (e.g. floor_z_m − 0.02m).

### Fill before running
- `config/tray_positions.yaml`: set `gx`, `gy`, `z_approach`, `z_pick` from measurements above.
- `config/seeds.csv` naut row: set `depth_mm` ≈ `floor_z_m * 1000` (so deposit also reaches floor).
- `config/seeds.csv` naut row: set `tool_param` to grip angle that holds a chickpea (start with 35).

### Launch sequence (4 terminals)
```bash
# Terminal 1
ros2 launch robot_bringup robot.launch.xml

# Terminal 2
ros2 run robot_gripper gripper_node

# Terminal 3
ros2 run robot_missions mission_server \
  --ros-args --params-file install/robot_missions/share/robot_missions/config/tray_positions.yaml

# Terminal 4 — send goal
ros2 action send_goal /mission robot_missions/action/MissionAction \
  "{mission_type: 'plant', target: 'naut', quantity: 1.0, quantity_unit: 'seeds', start_x: 0.0, start_y: 0.0, start_yaw: 0.0}"
```

### Pre-flight checks
- `can0` up: `ip link show can0` — state UP
- Gantry homed: check `/joint_states` shows all axes at 0.0 after bringup
- Gripper wired: `ros2 topic echo /gripper/current_angle` — should publish after gripper_node starts
- JTC active: `ros2 control list_controllers` — `joint_trajectory_controller` state: active

---

## 13. Camera Safety — Future Integration

Plan: add optional `SafetyChecker*` to `ExecutionEngine`. Before each base movement:
```cpp
if (safety_checker_ && !safety_checker_->is_safe_to_move()) {
    wait_for_safe();
}
```

First implementation: subscribe to `/person_detected` (`Bool`) published by
`robot_vision/safety_monitor_node.py`. Zero changes to `MissionBase`, `MissionManager`,
`MissionRegistry`, or `PlantingMission`. Any future mission using base movement gets
safety for free by passing a `SafetyChecker` to its executor.

---

*Last updated: 2026-05-31 — added start_yaw to MissionAction goal, MissionCommand, and navigate_to_start*
