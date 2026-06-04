// ══════════════════════════════════════════════════════════
// MISSION STATE PERSISTENCE  (sessionStorage — survives refresh)
// ══════════════════════════════════════════════════════════

let _persistTimer = null;

function _persistMissionState() {
    clearTimeout(_persistTimer);
    _persistTimer = setTimeout(() => {
        _persistTimer = null;
        if (!activeMissionZoneId) return;
        try {
            sessionStorage.setItem('selena_mission', JSON.stringify({
                zoneId:   activeMissionZoneId,
                feedback: zoneMissionFeedback,
                path:     traveledPath.length > 500 ? traveledPath.slice(-500) : traveledPath,
            }));
        } catch (_) {}
    }, 1000);
}

function _restorePersistedMissionState(zoneId) {
    try {
        const raw = sessionStorage.getItem('selena_mission');
        if (!raw) return;
        const data = JSON.parse(raw);
        if (data.zoneId !== zoneId) return;
        if (Array.isArray(data.path) && data.path.length > 0) traveledPath = data.path;
        if (data.feedback) zoneMissionFeedback = data.feedback;
    } catch (_) {}
}

function _clearPersistedMissionState() {
    clearTimeout(_persistTimer);
    _persistTimer = null;
    try { sessionStorage.removeItem('selena_mission'); } catch (_) {}
}

// ══════════════════════════════════════════════════════════
// MISSION ACTION CLIENT
// ══════════════════════════════════════════════════════════

// How far forward to shift the robot's first stop so the most-backward gantry row
// aligns with zone.start_along instead of planting behind the zone boundary.
// Derived from URDF: y_axis_joint travels backward in base_link (-X direction),
// so the last row of the first stop is (y_offset + (seedsInY-1)*sy) behind the robot,
// but gantry_origin is y_axis_travel/2 = 0.145 m ahead of base_link.
// Net backward reach = (y_axis_travel/2) - y_offset = (seedsInY-1)*sy/2.
function _zoneStartOffset(zone) {
    const seed = state.seeds.find(s => s.name === zone.target);
    if (!seed) return 0;
    const sy = seed.spacing_y_mm / 1000;
    const seedsInY = Math.floor(GANTRY_Y_TRAVEL / sy) + 1;
    return (seedsInY - 1) * sy / 2;
}

function zoneToGoal(zone) {
    const f    = fieldConfig.field;
    const yaw  = f.origin.yaw || 0;
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
    // Strip center start — matches _stripCenterStart in MapRenderer
    const cx = f.origin.x - zone.strip_id * f.strip_spacing * sinY;
    const cy = f.origin.y + zone.strip_id * f.strip_spacing * cosY;
    // Shift robot start forward so the most-backward gantry row plants at zone.start_along
    const sa = zone.start_along + _zoneStartOffset(zone);
    return {
        mission_type:  zone.mission_type,
        target:        zone.target,
        quantity:      parseFloat((zone.end_along - zone.start_along).toFixed(3)),
        quantity_unit: 'meters',
        start_x:       parseFloat((cx + sa * cosY).toFixed(4)),
        start_y:       parseFloat((cy + sa * sinY).toFixed(4)),
        start_yaw:     parseFloat(yaw.toFixed(4)),
        extra_params:  '',
    };
}

function startZoneMission(zoneId) {
    if (activeMissionZoneId) return;
    const zone = (missionZones.zones || []).find(z => z.zone_id === zoneId);
    if (!zone || zone.status !== 'planned') return;
    if (!fieldConfig) return;
    if (!rosConnected) {
        alert('Not connected to robot'); return;
    }

    traveledPath = [];
    _clearPersistedMissionState();
    const goalId = 'goal_' + Date.now();
    activeGoalId        = goalId;
    activeMissionZoneId = zoneId;

    // Persist in_progress immediately so a page refresh can restore this mission
    zone.status = 'in_progress';
    saveMissionZones();

    function onMessage(evt) {
        let msg;
        try { msg = JSON.parse(evt.data); } catch (_) { return; }
        if (msg.id !== goalId) return;

        if (msg.op === 'action_feedback') {
            zoneMissionFeedback = msg.values;
            const z = (missionZones.zones || []).find(z => z.zone_id === zoneId);
            setMissionStatusBar(z || zone);
            renderZonesList();
            _persistMissionState();
        } else if (msg.op === 'action_result') {
            _clearActiveGoalListener();
            _clearPersistedMissionState();
            const z = (missionZones.zones || []).find(z => z.zone_id === zoneId);
            if (z) {
                if (msg.result === false) {
                    // Rosbridge-level error (server unavailable etc.) — revert.
                    z.status = 'planned';
                } else {
                    const vals        = msg.values || {};
                    const itemsDone   = vals.items_completed  ?? 0;
                    const distCovered = vals.distance_covered ?? 0;

                    if (vals.success === true) {
                        z.status = 'completed';
                    } else if (itemsDone > 0 && distCovered > 0.05) {
                        // Partial cancel: shrink zone to planted segment, mark green.
                        // The freed tail becomes available for new zones.
                        const newEnd = parseFloat(
                            (z.start_along + distCovered).toFixed(1));
                        z.end_along = Math.min(newEnd, z.end_along);
                        z.status    = 'completed';
                    } else {
                        // Nothing planted — restore to planned so it can be restarted.
                        z.status = 'planned';
                    }
                }
                saveMissionZones();
            }
            plannedPath         = [];
            traveledPath        = [];
            activeMissionZoneId = null;
            activeGoalId        = null;
            zoneMissionFeedback = null;
            _cancellingZoneId   = null;
            renderZonesList();
            setMissionStatusBar(null);
            if (mapRenderer) mapRenderer.scheduleDraw();
        }
    }

    ros.socket.addEventListener('message', onMessage);
    _activeMessageListener = { socket: ros.socket, fn: onMessage };

    ros.callOnConnection({
        op:          'send_action_goal',
        id:          goalId,
        action:      '/mission',
        action_type: 'robot_missions/action/MissionAction',
        args:        zoneToGoal(zone),
        feedback:    true,
    });

    renderZonesList();
    setMissionStatusBar(zone);
    if (mapRenderer) mapRenderer.scheduleDraw();
}

function cancelZoneMission() {
    if (!activeMissionZoneId) return;
    // Show cancelling badge immediately so the user knows the stop request was received.
    _cancellingZoneId = activeMissionZoneId;
    renderZonesList();
    const textEl = document.getElementById('mission-status-text');
    if (textEl) textEl.textContent = 'Cancelling…';

    if (activeGoalId) {
        // Normal path: still have the original connection's goal ID.
        ros.callOnConnection({ op: 'cancel_action_goal', id: activeGoalId, action: '/mission' });
    } else {
        // Reconnect path (page was refreshed): cancel via service if we have the UUID.
        // Do NOT call _applyMissionCompletion here — the cancelling badge must stay
        // visible while the robot finishes the current seed. _subscribeActionStatus
        // detects the terminal state (within ~1 s) and calls _applyMissionCompletion then.
        if (_activeGoalUuid) {
            const svc = new ROSLIB.Service({
                ros,
                name:        '/mission/_action/cancel_goal',
                serviceType: 'action_msgs/CancelGoal',
            });
            svc.callService({
                goal_info: {
                    goal_id: { uuid: _activeGoalUuid },
                    stamp:   { sec: 0, nanosec: 0 },
                },
            }, () => {});
        }
    }
}

function showCancelZoneModal() {
    if (!activeMissionZoneId) return;
    const zone = (missionZones.zones || []).find(z => z.zone_id === activeMissionZoneId);
    document.getElementById('modal-cancel-text').textContent = zone
        ? 'Cancel planting ' + zone.target + ' on Strip ' + (zone.strip_id + 1) + '? Seeds already planted will be saved as a completed zone.'
        : 'Cancel the current mission?';
    showModal('modal-cancel-confirm');
}

function setMissionStatusBar(zone) {
    const barEl  = document.getElementById('mission-status-bar');
    const textEl = document.getElementById('mission-status-text');
    if (!zone) {
        textEl.textContent = 'None';
        barEl.classList.remove('active');
        return;
    }
    const fb       = zoneMissionFeedback;
    const pct      = fb ? Math.round(fb.progress_percent) : 0;
    const typeMap  = { plant: 'planting', weed: 'weeding', water: 'watering' };
    const typeName = typeMap[zone.mission_type] || zone.mission_type;
    const zoneNum  = getZoneNumber(zone.zone_id);
    textEl.textContent = `${typeName} (${zone.target} - zone ${zoneNum}) | progress ${pct} %`;
    barEl.classList.add('active');
}

function _clearActiveGoalListener() {
    if (_activeMessageListener) {
        try { _activeMessageListener.socket.removeEventListener('message', _activeMessageListener.fn); } catch (_) {}
        _activeMessageListener = null;
    }
}

// ══════════════════════════════════════════════════════════
// ACTION STATUS / FEEDBACK — real-time tracking across page refreshes
// ══════════════════════════════════════════════════════════

let _activeGoalUuid = null;  // uint8[16] array from /mission/_action/status, for UUID-based cancel
let _statusTimeoutId = null; // fires if action server never publishes status after connect
// Set when first status arrives with no active goal but zones weren't loaded yet.
// _checkInProgressZoneRestore reads this on the next loadMissionZones() call.
let _firstStatusWasEmpty = false;
let _firstStatusList = null; // status_list captured alongside _firstStatusWasEmpty for terminal-goal lookup

// Apply a terminal mission outcome: update zone, save YAML, clear all tracking state.
function _applyMissionCompletion(z, statusCode) {
    if (z) {
        if (statusCode === 4) {
            z.status = 'completed';
        } else {
            // Use last known progress to decide between partial-complete and full revert.
            const pct = zoneMissionFeedback?.progress_percent ?? 0;
            if (pct > 5) {
                const zoneLen     = z.end_along - z.start_along;
                const distCovered = parseFloat(((pct / 100) * zoneLen).toFixed(1));
                z.end_along = Math.min(parseFloat((z.start_along + distCovered).toFixed(1)), z.end_along);
                z.status    = 'completed';
            } else {
                z.status = 'planned';
            }
        }
        saveMissionZones();
    }
    _clearPersistedMissionState();
    activeMissionZoneId = null;
    activeGoalId        = null;
    zoneMissionFeedback = null;
    _cancellingZoneId   = null;
    plannedPath         = [];
    traveledPath        = [];
    renderZonesList();
    setMissionStatusBar(null);
    if (mapRenderer) mapRenderer.scheduleDraw();
}

// Called from loadMissionZones() to handle races where status topic fires
// before zones are loaded from the HTTP API.
function _checkInProgressZoneRestore() {
    const inProgressZone = (missionZones.zones || []).find(z => z.status === 'in_progress');
    if (!inProgressZone) return;

    if (_activeGoalUuid) {
        // Status topic already confirmed an active goal — finalize tracking state.
        // (activeMissionZoneId may already be set provisionally from loadMissionZones)
        activeMissionZoneId = inProgressZone.zone_id;
        renderZonesList();
        setMissionStatusBar(inProgressZone);
        if (mapRenderer) mapRenderer.scheduleDraw();
    } else if (_firstStatusWasEmpty) {
        // Status topic reported no active goals before zones loaded.
        // Check the saved status list for a terminal goal before giving up.
        const terminalGoal = (_firstStatusList || []).find(
            s => s.status === 4 || s.status === 5 || s.status === 6);
        if (terminalGoal) {
            _applyMissionCompletion(inProgressZone, terminalGoal.status);
        } else {
            // Outcome unknown — clear provisional state, leave zone as in_progress
            // so user can manually reset via the ■ button if the mission is truly done.
            activeMissionZoneId = null;
            renderZonesList();
            setMissionStatusBar(null);
            if (mapRenderer) mapRenderer.scheduleDraw();
        }
    }
    // else: status topic hasn't fired yet; activeMissionZoneId stays provisional
    // from loadMissionZones() until the first status tick arrives.
}

// Subscribe to the action server's status topic.
// Fires at 1 Hz regardless of whether a goal is active, so it's reliable for reconnect detection.
function _subscribeActionStatus() {
    clearTimeout(_statusTimeoutId);
    let gotFirstStatus = false;
    const sub = new ROSLIB.Topic({
        ros,
        name:          '/mission/_action/status',
        messageType:   'action_msgs/GoalStatusArray',
        throttle_rate: 0,
    });

    // If the mission server is not running, no status messages arrive.
    // After 10 s assume it is dead and reset any stale in_progress zone to planned.
    _statusTimeoutId = setTimeout(() => {
        if (gotFirstStatus) return;
        const inProgressZone = (missionZones.zones || []).find(z => z.status === 'in_progress');
        if (inProgressZone) {
            _applyMissionCompletion(inProgressZone, 5);  // 5 = CANCELED → back to planned
        }
    }, 10000);

    sub.subscribe(msg => {
        clearTimeout(_statusTimeoutId);
        const statusList = msg.status_list || [];
        // ACCEPTED=1, EXECUTING=2, CANCELING=3 — any of these means the robot is working
        const activeGoal = statusList.find(s => s.status === 1 || s.status === 2 || s.status === 3);

        if (!gotFirstStatus) {
            gotFirstStatus = true;
            // On first status after (re)connect: resolve any in_progress zone in the YAML
            const inProgressZone = (missionZones.zones || []).find(z => z.status === 'in_progress');
            if (inProgressZone) {
                if (activeGoal) {
                    // Robot is still running — confirm tracking state for UI and cancel button.
                    // activeMissionZoneId may already be set provisionally from loadMissionZones();
                    // always update _activeGoalUuid so UUID-based cancel works.
                    _activeGoalUuid     = activeGoal.goal_info.goal_id.uuid;
                    activeMissionZoneId = inProgressZone.zone_id;
                    // Restore traveled path + last feedback from sessionStorage.
                    // Covers both page refresh (traveledPath still empty) and WebSocket
                    // reconnect (traveledPath cleared by close handler).
                    _restorePersistedMissionState(inProgressZone.zone_id);
                    renderZonesList();
                    setMissionStatusBar(inProgressZone);
                    if (mapRenderer) mapRenderer.scheduleDraw();
                } else {
                    // No active goal — check for a just-finished terminal status
                    const terminalGoal = statusList.find(
                        s => s.status === 4 || s.status === 5 || s.status === 6);
                    if (terminalGoal) {
                        _applyMissionCompletion(inProgressZone, terminalGoal.status);
                    } else {
                        // Terminal goal not in status list (may have expired or not yet published).
                        // Don't assume CANCELED — leave zone as in_progress so user can
                        // manually reset via the ■ button if needed.
                        activeMissionZoneId = null;
                        renderZonesList();
                        setMissionStatusBar(null);
                        if (mapRenderer) mapRenderer.scheduleDraw();
                    }
                }
            } else if (!activeGoal) {
                // Zones weren't loaded yet (HTTP race). Mark so _checkInProgressZoneRestore
                // can resolve any in_progress zone when loadMissionZones() completes.
                _firstStatusWasEmpty = true;
                _firstStatusList = statusList;  // preserve for terminal-goal lookup
            }
            return;
        }

        if (activeGoal) {
            _activeGoalUuid = activeGoal.goal_info.goal_id.uuid;
        } else {
            // Goal just disappeared — handle completion if we were tracking via status (reconnect mode)
            // (activeGoalId null means no socket-listener path is active)
            if (activeMissionZoneId && !activeGoalId) {
                const terminalGoal = statusList.find(
                    s => s.status === 4 || s.status === 5 || s.status === 6);
                const z = (missionZones.zones || []).find(z => z.zone_id === activeMissionZoneId);
                _applyMissionCompletion(z, terminalGoal ? terminalGoal.status : 5);
            }
            _activeGoalUuid = null;
        }
    });
}

// Subscribe to the action feedback topic for progress updates in reconnect mode.
// The socket listener (send_action_goal path) handles the normal case;
// this subscription only updates the UI when activeGoalId is null (reconnect case).
function _subscribeActionFeedback() {
    const sub = new ROSLIB.Topic({
        ros,
        name:          '/mission/_action/feedback',
        messageType:   'robot_missions/action/MissionAction_FeedbackMessage',
        throttle_rate: 500,
    });
    sub.subscribe(msg => {
        if (!activeMissionZoneId || activeGoalId) return;
        // Capture goal UUID from first feedback so cancel-via-service works in reconnect mode.
        // The status topic (TRANSIENT_LOCAL) doesn't reach rosbridge, so this is the only
        // reliable source of the UUID after a page refresh.
        if (!_activeGoalUuid && msg.goal_id?.uuid) {
            _activeGoalUuid = msg.goal_id.uuid;
        }
        const fb = msg.feedback || {};
        zoneMissionFeedback = {
            progress_percent: fb.progress_percent || 0,
            current_step:     fb.current_step     || '',
        };
        const z = (missionZones.zones || []).find(z => z.zone_id === activeMissionZoneId);
        if (z) setMissionStatusBar(z);
        renderZonesList();
        _persistMissionState();
    });
}

// Flush current mission state to sessionStorage synchronously before the page
// unloads (the debounced timer never fires on a hard refresh).
window.addEventListener('pagehide', () => {
    if (!activeMissionZoneId) return;
    try {
        sessionStorage.setItem('selena_mission', JSON.stringify({
            zoneId:   activeMissionZoneId,
            feedback: zoneMissionFeedback,
            path:     traveledPath.length > 500 ? traveledPath.slice(-500) : traveledPath,
        }));
    } catch (_) {}
});

// ══════════════════════════════════════════════════════════
// ODOM DRIFT WARNING
// ══════════════════════════════════════════════════════════

let _driftTimer = null;
function showOdomDriftWarning() {
    const el = document.getElementById('odom-drift-warning');
    el.classList.remove('hidden');
    clearTimeout(_driftTimer);
    _driftTimer = setTimeout(() => el.classList.add('hidden'), 3000);
}

// ══════════════════════════════════════════════════════════
// ROS CONNECTION
// ══════════════════════════════════════════════════════════

const ros = new ROSLIB.Ros({ url: 'ws://' + location.hostname + ':9090' });
const dot = document.getElementById('connection-dot');
const txt = document.getElementById('connection-text');
let reconnectDelay = 3000;
let odomSub = null;
let rosConnected = false;

ros.on('connection', () => {
    rosConnected = true;
    dot.classList.add('connected'); txt.textContent = 'Connected'; reconnectDelay = 3000;
    _subscribeOdom();
    _subscribePlannedPath();
    _subscribeActionStatus();
    _subscribeActionFeedback();
});
ros.on('error', () => { rosConnected = false; dot.classList.remove('connected'); txt.textContent = 'Error'; });
ros.on('close', () => {
    rosConnected = false;
    dot.classList.remove('connected'); txt.textContent = 'Disconnected';
    clearTimeout(_statusTimeoutId);
    odomSub = null;
    _clearActiveGoalListener();
    plannedPath            = [];
    traveledPath           = [];
    _activeGoalUuid        = null;
    _firstStatusWasEmpty   = false;
    _firstStatusList       = null;
    // Keep in_progress status in YAML — _subscribeActionStatus resolves it on reconnect.
    // Only clear in-memory tracking state; the zone record itself stays in_progress so
    // the UI can restore after the page reconnects.
    _cancellingZoneId = null;
    if (activeMissionZoneId) {
        activeMissionZoneId = null;
        activeGoalId        = null;
        zoneMissionFeedback = null;
        renderZonesList();
        setMissionStatusBar(null);
        if (mapRenderer) mapRenderer.scheduleDraw();
    }
    setTimeout(() => {
        txt.textContent = 'Reconnecting...';
        ros.connect('ws://' + location.hostname + ':9090');
        reconnectDelay = Math.min(reconnectDelay * 1.5, 30000);
    }, reconnectDelay);
});

let _prevOdomPose = null;

function _subscribePlannedPath() {
    const sub = new ROSLIB.Topic({
        ros, name: '/mission_planned_path',
        messageType: 'nav_msgs/Path', throttle_rate: 0,
    });
    sub.subscribe(msg => {
        plannedPath = (msg.poses || []).map(p => ({
            x: p.pose.position.x,
            y: p.pose.position.y,
        }));
        if (mapRenderer) mapRenderer.scheduleDraw();
    });
}

function _subscribeOdom() {
    odomSub = new ROSLIB.Topic({
        ros, name: '/diff_drive_controller/odom',
        messageType: 'nav_msgs/Odometry', throttle_rate: 100,
    });
    odomSub.subscribe(msg => {
        const p = msg.pose.pose.position;
        const q = msg.pose.pose.orientation;
        const yaw = Math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
        const newPose = { x: p.x, y: p.y, yaw };

        if (_prevOdomPose) {
            const dx = newPose.x - _prevOdomPose.x;
            const dy = newPose.y - _prevOdomPose.y;
            if (Math.sqrt(dx*dx + dy*dy) > 0.5) showOdomDriftWarning();
        }
        _prevOdomPose = { x: newPose.x, y: newPose.y };

        robotPose = newPose;
        if (activeMissionZoneId && robotPose) {
            traveledPath.push({ x: robotPose.x, y: robotPose.y });
            _persistMissionState();
        }
        if (mapRenderer) mapRenderer.scheduleDraw();
    });
}
