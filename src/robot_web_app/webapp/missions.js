// ══════════════════════════════════════════════════════════
// CONSTANTS (mirrors PlantingPlanner)
// ══════════════════════════════════════════════════════════

let GANTRY_X_TRAVEL = 0.845;   // m — overwritten by /api/robot_config (x_axis_travel)
let GANTRY_Y_TRAVEL = 0.290;   // m — overwritten by /api/robot_config (y_axis_travel)
const SNAP_M        = 0.1;     // zone start/end snap grid

let robotConfig = null;        // populated by loadRobotConfig() at startup

// ══════════════════════════════════════════════════════════
// STATE
// ══════════════════════════════════════════════════════════

const state = {
    seeds: [],   // populated from /api/seeds (seeds.csv) at startup
    selectedSeedIndex: 0,
    missionPhase: 'IDLE',   // 'IDLE' | 'ACTIVE'
    isAdding: false,
    editingSeedIndex: null,
};

// Map / field data
let fieldConfig  = null;
let missionZones = { zones: [] };
let robotPose    = null;   // { x, y, yaw }
let mapRenderer  = null;

// Mission action state
let activeMissionZoneId    = null;
let activeGoalId           = null;
let zoneMissionFeedback    = null;
let _activeMessageListener = null;
let _cancellingZoneId      = null;  // set immediately on cancel press, cleared on action_result

let plannedPath  = [];  // [{x,y},...] from /mission_planned_path topic
let traveledPath = [];  // [{x,y},...] accumulated from odom during active mission

// Zone selection state machine
const zonePreview = {
    phase:       'IDLE',   // 'IDLE' | 'AWAITING_END' | 'ZONE_READY'
    stripId:     null,
    startAlong:  null,     // m, snapped
    endAlong:    null,     // m, snapped
    hoverAlong:  null,     // m, snapped — live preview while hovering
    overlap:     false,
};
