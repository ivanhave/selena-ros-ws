# Selena Robot — Field Map Architecture
## Strategy document for field layout, mission planning, and navigation map
*Update this document as decisions are made or change.*

---

## 1. Physical Setup

The robot straddles a strip of land: wheels run on the ground on either side, the gantry
reaches down to work the strip between them.

```
   ←──────────── robot width (wheels outside) ────────────→
           ←──── gantry X travel: 0.845 m ────→
   ██████ |     [strip of soil / crop row]     | ██████
   LEFT   |                                    | RIGHT
   WHEEL  |____________________________________|  WHEEL
           ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
           gantry covers this width per stop
```

Coordinate note — the gantry is rotated 90° relative to `base_link`:
- **Gantry X axis** (`x_axis_joint`, 0.845 m travel) = across the strip = **robot Y direction (width)**
- **Gantry Y axis** (`y_axis_joint`, 0.290 m travel) = along robot travel = **robot X direction (forward)**

- **Strip width** = gantry X travel = **0.845 m** (fixed by hardware, never user-configurable)
- **Gantry reach per stop along strip** = gantry Y travel = **0.290 m** (limits rows per robot stop)
- **Robot advance per stop** = `seeds_in_y × spacing_y` (PlantingPlanner; can exceed 0.290 m)
- **Strip length** = user-defined (field-dependent, e.g. 10 m)
- **Strip spacing** = center-to-center distance between strips (≥ robot width + clearance)

The field contains multiple parallel strips. The robot traverses them in a snake pattern:

```
Strip 0 ──→──→──→──→──→──→── end ─┐
                                   ↓ turn
Strip 1 ──←──←──←──←──←──←── end ─┘
                                   ↓ turn
Strip 2 ──→──→──→──→──→──→── end ─┐
...
```

The robot may also skip strips (e.g. plant every other row). At each robot stop, the gantry
handles a rectangle of `0.845 m (across strip) × 0.290 m (along strip)`.

---

## 2. Map Data Layers

The field map has three independent layers at different time scales.

### Layer 1 — Field Layout (static, user-configured)

Describes the physical field: where strips are, how long, what direction.
Edited once (or rarely), stored on disk. Both the web app and future navigation nodes read this.

```yaml
# robot_web_app/config/field_config.yaml
field:
  origin:
    x: 0.0      # world frame (odometry) x at start of strip 0
    y: 0.0      # world frame y
    yaw: 0.0    # heading of strips in radians (0 = along odom +X)

  strip_width: 0.845      # meters — fixed, equals gantry X travel (x_axis_joint)
  strip_length: 10.0      # meters — length of each strip along the robot travel direction
  strip_spacing: 0.5      # meters — center-to-center spacing between adjacent strips

  strips:
    - id: 0
      enabled: true
    - id: 1
      enabled: true
    - id: 2
      enabled: false    # skip this row (e.g. path, obstacle)
```

Strip `i` center-line in world frame:
```
start_x = origin.x
start_y = origin.y + i * strip_spacing
heading = origin.yaw
```

For non-uniform fields (different lengths per strip, non-parallel): replace `strip_length`
and `strip_spacing` with per-strip `{start, end}` world coordinates. Not needed now.

**Stored at:** `robot_field/config/field_config.yaml`
**Served by:** `serve_webapp.py` as `GET /api/field_config` — reads via `get_package_share_directory('robot_field')`
**Saved by:** `serve_webapp.py` as `POST /api/field_config` — writes to same path
**Any ROS node** that needs field geometry just adds `robot_field` as a `<depend>` and reads from its share directory. No coupling to the web app.

---

### Layer 2 — Mission Zones (session-persistent)

Represents user-defined work areas overlaid on strips. Each zone claims a portion of a strip
for a specific mission type and seed.

```python
MissionZone {
    zone_id:      str       # uuid or sequential id
    strip_id:     int       # which strip
    start_along:  float     # meters from strip start (snapped to 0.1 m)
    end_along:    float     # meters from strip end   (snapped to 0.1 m)
    mission_type: str       # "plant", "weed", "water"
    target:       str       # "naut", "pea", etc.
    status:       str       # "planned", "in_progress", "completed", "failed"
    # derived (computed, not stored):
    # width = strip_width (always 0.845 m)
    # start_world = strip_origin + start_along * forward_vector
    # end_world   = strip_origin + end_along   * forward_vector
}
```

**Constraints enforced by web app:**
- Zones on the same strip cannot overlap (`start/end_along` ranges must not intersect).
- Zone must be fully within strip bounds (0 ≤ start_along < end_along ≤ strip_length).
- Width is always the full strip width — user cannot change it.

**Stored at:** `robot_field/config/mission_zones.yaml`
**Served by:** `GET/POST /api/mission_zones` — same path resolution as field_config

---

### Layer 3 — Live State (ephemeral, runtime only)

Not stored. Updated from ROS topics:

| Data | ROS source | Update rate |
|---|---|---|
| Robot pose | `/diff_drive_controller/odom` | ~10 Hz |
| Mission progress | `/mission` action feedback | 2 Hz |
| Seeds planted so far | feedback `items_completed_so_far` | 2 Hz |

---

## 3. Map Rendering (Web App)

### Canvas-based 2D top-down view

Use HTML5 Canvas (not SVG) — Canvas handles animated robot position at 10 Hz without DOM overhead.

**Coordinate transform: world → screen**
```javascript
// World: meters, origin at field.origin, x=right, y=up (top-down view)
// Screen: pixels, origin at canvas top-left, y=down (flipped)
function worldToScreen(wx, wy) {
    return {
        sx: canvas.width  / 2 + wx * PIXELS_PER_METER,
        sy: canvas.height / 2 - wy * PIXELS_PER_METER
    };
}
```

`PIXELS_PER_METER` controlled by a zoom slider. Pan by dragging canvas.

**Draw order:**
1. Background grid (0.1 m snap lines, faint)
2. Strips (light green rectangles, border only when inactive)
3. Mission zones (filled colored rectangles: blue=planned, orange=in_progress, green=completed)
4. Seed positions (small dots, only when zoomed in enough)
5. Robot (filled triangle pointing in yaw direction, updated from odom)
6. Selection highlight (yellow outline on zone being hovered or selected)

---

## 4. Zone Selection UI

When the user wants to create a new mission zone on the map:

1. **Click strip** → strip becomes active (highlighted)
2. **Click start point** on strip → snaps to 0.1 m along strip, places start marker
3. **Click end point** on strip → snaps to 0.1 m, places end marker
   - End must be after start
   - Must not overlap existing zones on that strip
4. **Side panel updates live** with:
   - Zone dimensions (width=0.845 m fixed, length=end−start)
   - Selected seed type + spacing
   - Computed: total seeds, number of robot stops, estimated duration
5. **"Add Zone" button** → saves zone to mission_zones list, renders on map
6. **Start Mission button** (when a zone is selected) → sends MissionAction goal

**Overlap detection:**
```javascript
function overlaps(newZone, existingZone) {
    if (newZone.strip_id !== existingZone.strip_id) return false;
    return newZone.start_along < existingZone.end_along &&
           newZone.end_along   > existingZone.start_along;
}
```

**Snap to grid:**
```javascript
const SNAP_M = 0.1;
function snapAlongStrip(raw_meters) {
    return Math.round(raw_meters / SNAP_M) * SNAP_M;
}
```

---

## 5. From Zone to MissionAction Goal

The web app converts a `MissionZone` to the existing action format:

```javascript
function zoneToGoal(zone, fieldConfig, seedProfile) {
    const strip    = fieldConfig.strips[zone.strip_id];
    const yaw      = fieldConfig.origin.yaw;
    const length   = zone.end_along - zone.start_along;  // meters

    // Robot's start pose: strip origin + start_along meters along heading
    const start_x  = fieldConfig.origin.x + zone.start_along * Math.cos(yaw)
                   + strip.id * fieldConfig.strip_spacing * Math.sin(yaw);  // perpendicular
    const start_y  = fieldConfig.origin.y + zone.start_along * Math.sin(yaw)
                   - strip.id * fieldConfig.strip_spacing * Math.cos(yaw);

    return {
        mission_type:  zone.mission_type,
        target:        zone.target,
        quantity:      length,             // meters — planner computes seeds from this
        quantity_unit: "meters",
        start_x,
        start_y,
        start_yaw:     yaw
    };
}
```

No changes needed to the MissionAction format or mission server for the first version.
`quantity_unit: "meters"` is already handled by `PlantingPlanner`.

Future: if exact seed count display in the side panel diverges from what the planner computes,
add `end_x`/`end_y` fields to `MissionAction` so the server has the full zone geometry.

---

## 6. Snake Path Planning (Navigation — Future)

When the operator starts missions on multiple strips, the mission server or a separate
planner decides the robot traversal order.

**Algorithm (for later, not now):**
```
sorted_zones = sort zones by (strip_id, start_along)
path = []
current_strip = -1
direction = +1   // +1 = forward (increasing start_along), -1 = reverse

for zone in sorted_zones:
    if zone.strip_id != current_strip:
        path.append(INTER_STRIP_TURN(from=current_end, to=zone.start))
        direction = -direction   // reverse direction on next strip
        current_strip = zone.strip_id
    path.append(MISSION(zone, direction))
    current_end = zone.end_world_pose
```

The turn between strips is an `InterStripNavigation` step: reverse to exit strip, arc turn
(or three-point turn if space is tight), drive to start of next strip. This becomes a new
`NavigationProvider` method or a new `TurnNavigator` class.

**Strip direction flag:**
When traversing strip in reverse (`direction = -1`):
- `start_yaw` = `field_yaw + π`
- Start pose = zone end world position
- Planner still works (seeds placed in same absolute locations)

---

## 7. Implementation Roadmap

### Step 1 — Field config + map canvas ✓ DONE
- [x] Define `field_config.yaml` format (Section 2, Layer 1)
- [x] Add `GET/POST /api/field_config` and `/api/mission_zones` endpoints to `serve_webapp.py`
- [x] Add canvas element to `missions.html` (map panel + sidebar layout)
- [x] `MapRenderer` class in `missions.js`: grid, strips, zones, coordinate transform
- [x] Zoom / pan interaction (mouse wheel + drag + touch + zoom slider)
- [x] Field config editor in side panel (strip count, length, spacing, origin)

### Step 2 — Robot position on map ✓ DONE (included in Step 1 implementation)
- [x] Subscribe to `/diff_drive_controller/odom` in `missions.js`
- [x] Animate robot triangle on canvas at odom pose
- [ ] Show odometry drift warning if odom jumps (> 0.5 m between readings)

### Step 3 — Zone selection UI ✓ DONE
- [x] Click-to-place start/end on strip with 0.1 m snap (2-click flow; Escape clears)
- [x] Overlap validation (red preview + disabled Add Zone button)
- [x] Seed type selector + live computation of seed count, stops, est. time (mirrors PlantingPlanner)
- [x] Save/load zones from `mission_zones.yaml` via HTTP (GET on load, POST on add/delete)

### Step 4 — Wire to mission server ✓ DONE
- [x] Per-zone ▶ Start button → `zoneToGoal()` → roslibjs ActionClient → `/mission`
- [x] Feedback → zone.status = in_progress, orange progress bar, status bar update
- [x] Result → zone.status = completed/failed, save, redraw
- [x] Cancel button while in_progress; other zones disabled; delete blocked for active zone

### Step 5 — Multi-zone / snake path
- [ ] Queue multiple zones across strips
- [ ] Compute traversal order (snake algorithm, Section 6)
- [ ] Send goals sequentially, wait for each to complete

### Step 6 — Nav2 integration
- [ ] Implement `Nav2Navigator : NavigationProvider` (one file, one line in mission_server_node.cpp)
- [ ] Field map config shared with Nav2 costmap (convert strips to nav2_msgs/OccupancyGrid or keep separate)
- [ ] AMCL localization replaces odometry drift

### Step 7 — RTK / absolute coordinates
- [ ] Replace odometry frame with GPS/RTK world frame
- [ ] Field config stores absolute coordinates
- [ ] Map tiles or satellite imagery as background

---

## 8. File Map (after Step 1 is complete)

```
robot_field/                   NEW PACKAGE — owns all field geometry data
├── CMakeLists.txt
├── package.xml
└── config/
    ├── field_config.yaml      NEW — field layout: strips, origin, spacing
    └── mission_zones.yaml     NEW — saved mission zones (planned/completed)

robot_web_app/
├── scripts/
│   └── serve_webapp.py        ADD — /api/field_config, /api/mission_zones endpoints
│                                     (reads/writes robot_field config via ament_index)
└── webapp/
    ├── missions.html          ADD — canvas element
    ├── missions.js            ADD — MapRenderer class, zone selection, field config editor
    └── missions.css           ADD — canvas styling, zoom controls
```

---

## 9. Design Constraints to Keep in Mind

- **Strip width is not user-configurable** — always `gantry_x_travel = 0.290 m`. Displaying
  it in the UI as a label is fine; there must be no input for it.
- **0.1 m snap** on all user-placed points along a strip (start/end of zones, not strip boundaries).
- **One active mission at a time** — the web app enforces this; once a mission is `in_progress`,
  all other zones are greyed out and Start Mission is disabled.
- **World frame = odometry for now** — the field map shifts if the robot does not start at the
  same physical location each session. Future: add a "set robot home" button that resets the
  odometry to a known field origin (or use RTK).
- **Planner computes seeds, web app confirms** — the web app shows a preview seed count using
  the same formula as `PlantingPlanner` (Section 4 of ARCHITECTURE.md). If the server returns
  a different count in feedback, the map updates to match.

---

*Last updated: 2026-06-01 — initial field map and navigation strategy*
