# Selena Robot — Progress Log

<!-- MAINTENANCE RULE (see CLAUDE.md):
     Keep the current + previous session in full detail.
     Older sessions collapse to a one-paragraph summary.
     Backlog and Known Hardware Issues stay current.
-->

## 2026-06-05 — robot_gantry: single control point for all gantry motion

### What was done

Built `robot_gantry` package and refactored all callers to use it exclusively. Nothing publishes to JTC or `gantry_velocity_controller` directly anymore.

**New package — `robot_gantry`:**
- `action/GantryMove.action` — goal: (x, y, z); result: success/message/final positions; feedback: phase + current positions
- `srv/GantrySetMode.srv` — toggle velocity/position mode with optional urgent flag
- `include/robot_gantry/gantry_constants.hpp` — single source of truth for X/Y/Z velocity caps, ACC byte, decel zone, timing constants; exported as CMake INTERFACE library `gantry_constants`
- `gantry_node` — action server `/gantry/move`, service server `/gantry/set_mode`, subscriber `/gantry/cmd_vel` (velocity mode forwarding), all mode switching, URDF-queried joint limits
- State machine: POSITION_IDLE → POSITION_MOVING → POSITION_IDLE ↔ VELOCITY
- Execute sequence: bounds check (URDF) → Z retract if needed → XY move if |delta|>TOL → Z lower if |delta|>TOL; each step uses JTC FollowJointTrajectory action for native completion confirmation; stop_requested_ checked between steps

**`mks_servo_hardware` refactored:**
- Removed local `constexpr` duplicates (X_VEL_CAP_MS, Y_VEL_CAP_MS, Z_VEL_CAP_MS, VEL_ACC, DECEL_ZONE_M, GUARD_ZONE_M); now aliases to `gantry_constants::` namespace
- Added `<build_depend>robot_gantry</build_depend>` + `target_link_libraries(... robot_gantry::gantry_constants)`

**`robot_missions/execution_engine` refactored:**
- Removed JTC publisher, `joint_state_sub_`, `state_mutex_`, local velocity cap constexpr, `send_xy_command`, `send_z_command`, `wait_xy_settled`, `interruptible_sleep`, `xy_move_ms`, `z_move_ms`
- Added `gantry_client_` (rclcpp_action::Client<GantryMove>)
- New `send_gantry_move(x, y, z)`: async_send_goal → poll async_get_result with STEP_MS interval, cancel goal on stop_requested_
- `move_gantry_to(gx, gy)` → `send_gantry_move(gx, gy, HOME_Z)` — robot_gantry handles Z retract internally
- `lower_z(depth)` → `send_gantry_move(current_gx_, current_gy_, depth)` — XY skipped by robot_gantry (no delta)
- `move_gantry_to_tray()` → single `send_gantry_move(tray.gx, tray.gy, tray.z_pick)` — robot_gantry does Z retract + XY + Z lower atomically

**`robot_web_app/app.js` refactored:**
- Removed: `jointTrajTopic`, `switchControllerService`, `sendGantryPosition()`, `X_HW_CAP/Y_HW_CAP/Z_HW_CAP`
- Added: `gantrySetModeService` (`/gantry/set_mode`), `gantryMoveClient` (ROSLIB.ActionClient on `/gantry/move`)
- `gantryVelTopic` topic changed from `/gantry_velocity_controller/commands` → `/gantry/cmd_vel`
- Mode toggle: calls `gantrySetModeService({velocity_mode: enabled, urgent: enabled})`
- Position text boxes: calls `sendGantryMove(x, y, z)` via action (no timing formula in JS)
- On-connect guard: calls `gantrySetModeService({velocity_mode: false, urgent: false})`

**`robot_bringup/launch/robot.launch.xml`:**
- Added `gantry_node` launch after JTC spawner

**Build:** all 12 packages clean, zero errors or warnings.

### Hardware test results (2026-06-05) — PASSED

**T1 ✅ gantry_node startup** — reads URDF limits correctly: `X [-0.005, 0.845]  Y [-0.005, 0.290]  Z [-0.005, 0.250]`

**T2 ✅ Position mode** — `/gantry/move` action: `(0.1, 0.05, 0.0)` SUCCEEDED, final position within 0.1mm. Two-phase test: Z lower to 0.08m then new XY goal confirms Z retract→XY→Z-lower sequencing correct.

**T3 ✅ Velocity mode** — `/gantry/set_mode {velocity_mode: true}` SUCCEEDED; `/gantry/cmd_vel` published; verified gantry_node forwards immediately to `/gantry_velocity_controller/commands` (monitored with topic echo).

**T4 ✅ Mode switch** — POSITION→VELOCITY→POSITION both directions: hardware interface logs `switched to VELOCITY mode` / `switched to POSITION mode`; gantry_node logs `Mode switched to VELOCITY` / `Mode switched to POSITION`.

**T5 ✅ Mission end-to-end** — 4/4 naut seeds planted; all 20 gantry moves went through `/gantry/move` action exclusively; zero direct JTC traffic from execution_engine; JTC still logs `Goal reached, success!` for each move (gantry_node drives JTC internally).

**Web app** — Updated app.js: no rosbridge import errors (removed ROSLIB.ActionClient), `/gantry/cmd_vel` subscription active, service type `robot_gantry/srv/GantrySetMode` accepted by rosbridge.

**Fix applied during test** — `robot_description` parameter was not passed to `gantry_node` in launch file; added `<param name="robot_description" value="$(var robot_description)" />`.

### Seed pickup attempt (code removed, same session start)

Attempted first end-to-end autonomous seed pickup (floor-mounted laptop camera → calibration → pickup action). Code removed because `execution_engine`, `pickup_server_node`, `calibration_node`, and web app all had their own JTC timing formula — robot_gantry had to be implemented first. Gripper pushed into the box during calibration (no visual confirmation of seed hold).

---

## 2026-06-04 — Navigation accuracy fix + stop-after-refresh fix + status bar redesign

### Problems fixed

1. **Robot stops short of zone start / doesn't follow calculated path** — two bugs in `OdometryNavigator::move_to()`:

   **Bug A — Wrong stopping metric** (`odometry_navigator.cpp`): The loop used `advanced = distance_from_start` and stopped when `advanced >= target_dist - ARRIVAL_TOL_M`. This is only correct for perfectly straight motion. Any heading correction arc makes `advanced` reach `target_dist - 0.02` while the robot is still 10–30 cm away from the target. The decel profile also used `target_dist - advanced` as "remaining", making it decelerate too early on curved paths. **Fix:** replaced `advanced` entirely with `remaining = sqrt((target.x-cx)² + (target.y-cy)²)` — direct Euclidean distance to target, recomputed every tick. Stopping condition: `remaining <= ARRIVAL_TOL_M`. Decel: `scale = remaining / DECEL_DIST_M`. Now the robot converges on the exact target regardless of how much it curves during heading correction.

   **Bug B — 30-second timeout too short** (`odometry_navigator.hpp`): `MOVE_TIMEOUT_MS = 30000`. A 3 m field move at 0.1 m/s + proportional decel takes ~34 s — timeout fired, robot stopped short. Increased to `120000` (2 min), enough for any expected field dimension.

   Also raised `TRACK_KP` 1.5 → 2.0 and `MAX_TRACK_WZ` 0.25 → 0.30 rad/s for crisper heading correction.

   **Build:** `colcon build --packages-select robot_navigation robot_missions` — both clean.

2. **Robot plants with wrong yaw (gantry not parallel to strip)** — after navigation the robot could be up to `ANGLE_TOL_RAD = 0.05 rad` (2.9°) off-strip. Over 0.845 m gantry that's 4 cm cross-track error.

   **Fix (`navigation_provider.hpp`, `odometry_navigator.hpp/cpp`, `mission_manager.cpp`):**
   - Added `virtual bool align_to_yaw(float yaw)` to `NavigationProvider` (default: `move_to` with same position). Future `Nav2Navigator` can override with its own precision alignment.
   - `OdometryNavigator::align_to_yaw()` calls `rotate_to(target_yaw, PLANT_ALIGN_TOL = 0.02 rad / ~1.1°)` — tighter than the general `ANGLE_TOL_RAD = 0.05 rad` used for navigation waypoints.
   - `rotate_to()` now takes an optional `angle_tol` parameter (default = `ANGLE_TOL_RAD`). Logs current yaw, target, and error in degrees before rotating.
   - `MissionManager::run()` calls `nav_->align_to_yaw(cmd.start_yaw)` as an explicit Step 4 between "navigate to start" and "execute". Visible in the robot log: `pre-plant yaw align — current X, target Y, error Z (N°)`.
   - Result: robot aligns to within ~1° of the strip direction before the gantry moves.

3. **Stop button after refresh had no effect** — After a page refresh `activeGoalId` is null and `_activeGoalUuid` was never set (the `/mission/_action/status` topic is TRANSIENT_LOCAL and never reaches rosbridge). So `cancelZoneMission()` reached neither branch and sent nothing to the robot.

   **Fix (`missions.ros.js`):**
   - `_subscribeActionFeedback`: on first feedback message, capture `msg.goal_id.uuid` into `_activeGoalUuid`. The feedback topic works fine with rosbridge; this is now the reliable UUID source in reconnect mode.
   - `cancelZoneMission`: collapsed the `else if (_activeGoalUuid)` branch into a single `else` block that: (a) cancels the 10-second timeout so it doesn't fire redundantly, (b) sends the cancel service call if `_activeGoalUuid` is now known, (c) immediately calls `_applyMissionCompletion(z, 5)` so the UI resolves at once without waiting for a result message that would never arrive.

2. **Partial-completion lost on stop** — `_applyMissionCompletion` always set `z.status = 'planned'` for non-SUCCEEDED outcomes, discarding planted progress.

   **Fix (`missions.ros.js`):** For status codes 5/6 (CANCELED/ABORTED), read `zoneMissionFeedback.progress_percent` (persisted across refresh in sessionStorage). If `pct > 5 %`, shrink `z.end_along` to the planted portion and mark `completed`. Otherwise revert to `planned`. This applies to all paths: user Stop after refresh, status-subscription goal-disappeared, and the 10 s timeout.

3. **Status bar: wrong format and wrong color** — Was `Planting — naut — Progress: 80%` in green.

   **Fix (`missions.ros.js`, `missions.css`):** New format: `planting (naut - zone 2) | progress 80 %` using a `mission_type → display name` map (`plant → planting` etc.) and `getZoneNumber()`. Active color changed from `#00ff88` (green) to `#e07820` (orange) for both the label and text.

### Files changed
- `robot_web_app/webapp/missions.ros.js`
- `robot_web_app/webapp/missions.css`

### Hardware test
Needs hardware test: verify Stop after refresh resolves zone correctly.

---

## 2026-06-03 — Right Wheel Investigation + ODESC Driver Restored

### Finding
Right wheels (nodes 7, 8) silent at startup — CAN bus-off or firmware hang on right ODESC board that occurs during the board's own boot. No CAN command (including ODrive reboot cmd 0x016) can reach it; physical power cycle + robot restart is the only fix. Software recovery attempted but found too complex and unnecessary.

### Action
Reverted all odesc_hardware changes back to the GitHub initial commit. The watchdog, encoder-response tracker, CAN reboot, and startup probe were all removed. The driver is now the same simple version as in the repository.

### Workflow when right wheels fail
Power-cycle the right ODESC board, then restart the robot (or re-run the launch). The board boots into CLOSED_LOOP automatically (`startup_closed_loop_control: True` in ODrive config).

---

## 2026-06-03 — Mission UI + ODESC Robustness Fixes (build passes, needs hardware test)

### Problems addressed

1. **Zone stuck "in_progress" when action server not running** — `_subscribeActionStatus` never fired if mission server was dead. New 10 s timeout: if no status message arrives after connect, stale zone resets to `planned`. `clearTimeout` added on disconnect so stale timer doesn't fire on the next connection's behalf.

2. **Wrong button for stale in_progress zone (X instead of ■)** — Delete button showed for non-active in_progress zones. Fixed: ✕ delete only appears for `planned` status. ■ reset button (orange, `.zone-reset-btn`) appears for any `in_progress` zone that isn't `activeMissionZoneId`. Clicking it immediately sets zone back to `planned` and saves.

3. **Trajectories missing** — Was a consequence of #1 (stale state, no active goal → no path tracking). Fixed by #1.

4. **Manual mode available during mission** — Already implemented in `app.js` via `missionStatusSub`; the stale-state case is fixed by #1 (zone resets within 10 s so teleop page sees no active goal).

5. **ODESC wheel error spam + velocity commands to faulted axes** —
   - `on_can_frame` HEARTBEAT: now only logs when error code *changes* (transition), not every heartbeat.
   - `check_and_recover`: only logs on first fault detection (`fault_count_[i] == 1`); recovery-trigger log unchanged.
   - `set_velocity`: skips axes where `state != 0 && state != CLOSED_LOOP_CONTROL` — prevents CAN spam to a faulted motor from interfering with recovery.
   - Added `last_error_` per-axis vector to `OdescDriver` for transition tracking.

### Files changed
- `robot_web_app/webapp/missions.ros.js` — 10 s timeout, clearTimeout on disconnect
- `robot_web_app/webapp/missions.zones.js` — button logic + zone-reset-btn event handler
- `robot_web_app/webapp/missions.css` — `.zone-reset-btn` style
- `odesc_hardware/include/odesc_hardware/odesc_driver.hpp` — `last_error_` vector
- `odesc_hardware/src/odesc_driver.cpp` — transition logging, set_velocity guard, last_error_ init

### Hardware test results (2026-06-03)

**T1 ✅ Stale ■ button** — page load with in_progress zone shows orange ■ (not ✕) immediately.

**T2 ✅ Click ■ resets** — clicking orange ■ sets zone to planned, ▶ reappears, YAML updated.

**T3 ❌ Status-sub auto-reset** — rosbridge does NOT forward `/mission/_action/status` messages to WebSocket clients (QoS: TRANSIENT_LOCAL RELIABLE, rosbridge subscribes but messages never arrive). Pre-existing issue, not introduced here. Fallback (T4) works.

**T4 ✅ 10-second timeout** — with mission server not running, stale zone resets to planned after ~10 s.

**T5 ✅ Full planting mission (hardware verified)** — 4/4 naut seeds planted, UI: 0%→25%→50%→75%→completed, YAML=completed, cancel button gone, status bar cleared.

**Additional bug found and fixed during test: mission_server segfault**
`wait_for_robot_ready()`: `future.get()` on `std::future<SharedPtr<T>>` moves the value out; the temporary SharedPtr was destroyed before the range-for completed, leaving a dangling reference to `response->controller`. Fix: `auto response = future.get(); for (auto& ctrl : response->controller)`. Rebuilt robot_missions.

---

## 2026-06-03 (earlier) — Mission Cancel Fix + Nav Abort — Summary

Cancel fix (4 bugs): JS result handler (`vals.success` not `msg.result`), C++ execute abort guard, nav abort flag in OdometryNavigator (`abort_flag_`, `reset_abort()` at run() start), confirmed page-refresh safety already in place.

---

## 2026-06-02 — Summary

Strip-safe path planning (`field_path_planner.hpp`: backward-to-headland → lateral → forward), proportional-decel and P-controller `rotate_to()` in OdometryNavigator, full planned path published before robot moves (`get_route_waypoints` override in PlantingMission). Play-button regression fixed: missing `robot_start` in planned path and missing `aborted_` guard in `execute()`. Progress % fix in `mission_server_node.cpp`. Both zones hardware-verified (16/16 seeds, canvas path ✓, status bar ✓).

---

## 2026-06-01 — Post-Homing Drift Fix + Step 4 Web App End-to-End (DONE — 42/42 seeds PASSED)

### Post-homing drift fix (`robot.launch.py`)

After homing, the gantry drifted toward the previous session's commanded position. Root cause: zombie `ros2_control_node` kept JTC alive with stale desired positions; after homing reset encoders to 0, JTC immediately drove motors back to the old position.

**Fix — `robot_bringup/launch/robot.launch.py`** (replaces `robot.launch.xml`):
1. `subprocess.run(['pkill', '-9', '-f', 'ros2_control_node'])` at launch start — kills any zombie
2. JTC spawner uses `--controller-manager-timeout 120` — waits for controller_manager services, which only appear after hardware activation (homing done). JTC always spawns fresh and initialises from position 0. No stale state.

Verified: gantry held at 0 for 9+ seconds post-homing, zero drift.

### Web app ▶ Start protocol fix (`missions.js`)

`ROSLIB.ActionClient`/`ROSLIB.Goal` uses ROS1-style topic publishing, rejected by rosbridge 2.6.0 (Jazzy). Replaced with direct rosbridge ops:
- Send: `ros.callOnConnection({ op: 'send_action_goal', id: goalId, action: '/mission', action_type: 'robot_missions/action/MissionAction', args: zoneToGoal(zone), feedback: true })`
- Listen: `ros.socket.addEventListener('message', onMessage)` — filters `msg.id === goalId`, handles `action_feedback` (updates progress bar) and `action_result` (sets zone `completed`/`failed`)
- Cancel: `ros.callOnConnection({ op: 'cancel_action_goal', id: goalId, action: '/mission' })`
- `_clearActiveGoalListener()` removes the socket listener on result or disconnect

### Hardware test — PASSED

Goal from web app ▶: `plant / naut / 4.0m zone`. PlantingPlanner: 42 seeds / 11 stops.  
Base navigated 4.516m to start. All 42 seeds planted, zero errors, zero sync faults.  
Zone status updated to `completed` in web app. Full path: browser → rosbridge → mission server → gantry → gripper → SUCCEEDED.

---

## 2026-05-31 — Summary

- **Missions page + field map** (`missions.html/js/css`, `serve_webapp.py`): canvas MapRenderer, strip + zone draw layers, 2-click zone placement, overlap validation, live seed count stats, field config editor, HTTP API (`/api/field_config`, `/api/mission_zones`), robot pose from odom. Canvas sizing bug fixed (observe canvas, not parent).
- **NavigationProvider** (`robot_navigation` package): `NavigationProvider` abstract interface + `OdometryNavigator` impl extracted from `ExecutionEngine`. Swap to Nav2Navigator in one line. Tested: 8/8 seeds, 2 stops, SUCCEEDED.
- **Multi-stop seeding**: 5 unit tests (PlantingPlanner). Hardware 8/8 seeds, 2 stops, 0.400m advance. SUCCEEDED.
- **Ghost movement fix**: `send_z_command` uses two-point trajectory (400ms velocity-damp settle + stroke). Peak ΔX during Z down: 0.72mm → 0.02mm. Root cause: residual XY velocity at trajectory end caused cubic Hermite spline lobe. Fix: settle point forces XY velocity = 0 before Z stroke begins.
- **Startup fixes**: `zeroPositionAtHome()` sends active 0x31 nudge per retry (was passively polling stale cache → sync fault). Post-homing X sync check re-zeros up to 3× if X1/X2 diverge >1 rad.
- **`start_yaw` field added** to `MissionAction` goal. `navigate_to_start()` now uses it. Hardware: 8/8 seeds, 2 stops, SUCCEEDED.

---

## 2026-05-30 — Summary

- **4-seed planting** (naut, 600×200mm, 2×2 grid): tray gx=0.800 gy=0.200 z=0.220, soil z=0.220, depth=0. `FlatGroundDepthSensor` added. 3/3 clean runs, SUCCEEDED.
- **JTC motion profile fix**: `sendGantryPosition` now uses `1.5 × dist / hw_cap` per axis (X=0.15, Y=0.06, Z=0.04 m/s). Eliminates motor overshooting trajectory window. 18/18 moves 0mm error.
- **Web app control verified**: JTC position mode + velocity joystick both tested ×5. ≤0.03mm drift on mode toggle. `sendGantryPosition` called after switch-back to hold current position.
- **Ghost movement analysis**: Motor overshoots XY target by ~64mm on large moves; `wait_xy_settled` added — polls until ≤2mm stable for 750ms before Z.
- **Z-first sequencing**: `send_xy_command` / `send_z_command` primitives enforce Z=0 before all XY moves. `allow_partial_joints_goal: true` in JTC config.

---

## 2026-05-29 — Summary

Homing, hardware interface, and teleop brought up from scratch:
- **Homing fixes**: `0xF6 vel=0` enables motor before `0x91`; `clearOfflineFlag()` resets stale offline state post-homing; `zeroPositionAtHome()` captures encoder offset at home.
- **Velocity caps + safety**: X=0.15, Y=0.06, Z=0.04 m/s in both position and velocity mode. ACC=230. Sqrt decel profile in DECEL_ZONE (80mm), hard stop in GUARD_ZONE (25mm). Non-fatal stall recovery (5 attempts).
- **X-axis homing false-success fix**: `any_motion_x1/x2` flags; FATAL if one motor never moved during coordinated X homing.
- **Manual mode toggle fix**: after switch JTC→velocity→JTC, `sendGantryPosition(current)` holds position; `hw_commands_` seeded from `hw_states_` during mode switch.

---

## Known Hardware Issues

- **X1 motor CAN reliability** — Intermittently missed CAN commands during homing; X1/X2 sync faults observed. Check CAN connector on X1 (ID=1), consider power-cycling.
- **Right ODESC wheels (nodes 7, 8) CAN bus-off at startup** — Right ODESC board intermittently enters CAN bus-off or firmware hang during its own boot. No software fix possible. Symptom: right wheels don't move. Fix: power-cycle the right ODESC board and restart the robot.

---

## Backlog

- [x] ~~**robot_gantry**~~ — DONE 2026-06-05: single control point for all gantry motion; 4/4 seeds via action, velocity forwarding, mode switch all hardware-verified
- [ ] **Seed pickup** — re-implement after robot_gantry is in place; needs visual pickup confirmation (compare gripper finger region before/after close)
- [ ] Measure tray coordinates physically → update `config/tray_positions.yaml`
- [ ] **Multi-strip navigation verification** — headland routing untested; same-strip confirmed with 2 missions
- [ ] **Nav2Navigator** — implement once Nav2 is set up (one-line swap from `OdometryNavigator`)
- [ ] `SafetyChecker` — subscribe `/person_detected`, pause base movement during mission
- [ ] `DepthSensor` — real depth sensor not mounted; `FlatGroundDepthSensor` active
- [ ] VacuumTool (tool_id=2 will error until implemented)
- [ ] **autoDetectDirection()** — all 4 motors default to direction_multiplier_=1.0; investigate 0x90 query timing
- [x] ~~Strip-safe path planning + navigation accuracy~~ — done 2026-06-02, hardware verified (same-strip)
- [x] ~~Play button flashes / trajectory not visible~~ — fixed 2026-06-02: planned path includes robot start; abort guard in execute(); 16/16 seeds on 2 zones PASSED
- [x] ~~JTC spawner race / post-homing drift~~ — fixed 2026-06-01 via `robot.launch.py`
- [x] ~~Web app ▶ Start button~~ — fixed 2026-06-01: `send_action_goal` op, 42/42 seeds PASSED
- [x] ~~Ghost movement (Z stroke XY drift)~~ — fixed 2026-05-31: two-point Z trajectory
- [x] ~~NavigationProvider / multi-stop~~ — done 2026-05-31, hardware verified
