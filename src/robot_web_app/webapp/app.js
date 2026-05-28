// ══════════════════════════════════════════════════════════
// SPEED LIMITS — edit here only
// ══════════════════════════════════════════════════════════
const BASE_MAX_SPEED_MS = 1.0;
const BASE_MIN_SPEED_MS = BASE_MAX_SPEED_MS * 0.1;
const GANTRY_MAX_SPEED_MS = 0.1;
const GANTRY_MIN_SPEED_MS = GANTRY_MAX_SPEED_MS * 0.1;

// ══════════════════════════════════════════════════════════
// GRIPPER LIMITS — edit here only
// ══════════════════════════════════════════════════════════
const GRIPPER_MIN_DEG = 0;   // physical minimum — fully open
const GRIPPER_MAX_DEG = 55;  // physical maximum — fully closed
// Web slider 0 = closed (physical 55), web slider 55 = open (physical 0)
// Inversion: physical_angle = GRIPPER_MAX_DEG - web_value

// ══════════════════════════════════════════════════════════
// DISPLAY HELPERS
// ══════════════════════════════════════════════════════════

// Collapses -0 and sub-0.5 mm values to "0" to prevent "-0" display
function mmStr(meters) {
    const mm = meters * 1000;
    return String(Math.round(Math.abs(mm) < 0.5 ? 0 : mm));
}

// Collapses -0.00 in odom display
function mStr(v) {
    return (Math.abs(v) < 0.005 ? 0 : v).toFixed(2);
}

// ══════════════════════════════════════════════════════════
// ROS
// ══════════════════════════════════════════════════════════

const ros = new ROSLIB.Ros({ url: 'ws://' + location.hostname + ':9090' });
const dot = document.getElementById('connection-dot');
const txt = document.getElementById('connection-text');

let reconnectDelay = 3000;
ros.on('connection', () => {
    dot.classList.add('connected');
    txt.textContent = 'Connected';
    reconnectDelay = 3000;
    // Ensure hardware is in JTC/position mode on every connect.
    // BEST_EFFORT: no-op if JTC is already active; corrects spawner-race inversion.
    if (!manualMode) {
        switchControllerService.callService(
            new ROSLIB.ServiceRequest({
                activate_controllers: ['joint_trajectory_controller'],
                deactivate_controllers: ['gantry_velocity_controller'],
                strictness: 2
            }),
            () => {}
        );
    }
});
ros.on('error', () => { dot.classList.remove('connected'); txt.textContent = 'Error'; });
ros.on('close', () => {
    dot.classList.remove('connected');
    txt.textContent = 'Disconnected';
    setTimeout(() => {
        txt.textContent = 'Reconnecting...';
        ros.connect('ws://' + location.hostname + ':9090');
        reconnectDelay = Math.min(reconnectDelay * 1.5, 30000);
    }, reconnectDelay);
});

const cmdVelTopic = new ROSLIB.Topic({
    ros, name: '/diff_drive_controller/cmd_vel',
    messageType: 'geometry_msgs/TwistStamped'
});

const jointTrajTopic = new ROSLIB.Topic({
    ros, name: '/joint_trajectory_controller/joint_trajectory',
    messageType: 'trajectory_msgs/JointTrajectory'
});

const gantryVelTopic = new ROSLIB.Topic({
    ros, name: '/gantry_velocity_controller/commands',
    messageType: 'std_msgs/Float64MultiArray'
});

const switchControllerService = new ROSLIB.Service({
    ros,
    name: '/controller_manager/switch_controller',
    serviceType: 'controller_manager_msgs/SwitchController'
});

// ── Gripper topics — defined once at top level ────────────
const gripperAngleTopic = new ROSLIB.Topic({
    ros, name: '/gripper/angle',
    messageType: 'std_msgs/Int32'
});

const gripperCurrentAngleSub = new ROSLIB.Topic({
    ros, name: '/gripper/current_angle',
    messageType: 'std_msgs/Int32'
});

// ── Voltage subscriptions ─────────────────────────────────
const voltageSubs = [
    ['voltage_left',  'voltage-left'],
    ['voltage_right', 'voltage-right'],
].map(([suffix, id]) => {
    const sub = new ROSLIB.Topic({
        ros, name: '/odesc/' + suffix, messageType: 'std_msgs/Float32'
    });
    sub.subscribe((msg) => {
        document.getElementById(id).value = msg.data.toFixed(1);
    });
    return sub;
});

// ══════════════════════════════════════════════════════════
// MANUAL MODE TOGGLE
// ══════════════════════════════════════════════════════════

let manualMode = false;
const manualLabel = document.getElementById('manual-label');
const toggleWrap = document.getElementById('manual-toggle-wrap');
const panels = document.querySelectorAll('.panel');

panels.forEach(p => p.classList.add('disabled'));

// Gripper box starts locked (manual mode is off at page load)
const _gripperBoxInit = document.getElementById('gripper-deg-val');
_gripperBoxInit.setAttribute('readonly', '');
_gripperBoxInit.setAttribute('tabindex', '-1');
_gripperBoxInit.classList.add('no-interact');

function setManualMode(enabled) {
    manualMode = enabled;
    manualLabel.textContent = 'manual mode: ' + (enabled ? 'on' : 'off');

    if (enabled) toggleWrap.classList.add('active');
    else toggleWrap.classList.remove('active');

    panels.forEach(p => {
        if (enabled) p.classList.remove('disabled');
        else p.classList.add('disabled');
    });

    if (baseJoystick) baseJoystick.redraw();
    if (gantryJoystick) gantryJoystick.redraw();

    // Switch ros2_control controllers
    const activate = enabled ? ['gantry_velocity_controller'] : ['joint_trajectory_controller'];
    const deactivate = enabled ? ['joint_trajectory_controller'] : ['gantry_velocity_controller'];

    switchControllerService.callService(
        new ROSLIB.ServiceRequest({
            activate_controllers: activate,
            deactivate_controllers: deactivate,
            strictness: 2
        }),
        (result) => { console.log('Controller switch:', result.ok ? 'OK' : 'FAILED'); }
    );

    // Gantry position boxes: editable in JTC mode only
    ['gantry-x-val', 'gantry-y-val', 'gantry-z-val'].forEach(id => {
        const el = document.getElementById(id);
        if (enabled) {
            el.setAttribute('readonly', '');
            el.setAttribute('tabindex', '-1');
            el.classList.add('no-interact');
        } else {
            el.removeAttribute('readonly');
            el.removeAttribute('tabindex');
            el.classList.remove('no-interact');
        }
    });

    // Gripper box: readonly when manual mode is off (JTC mode — user can't drive gripper here)
    const gripperBox = document.getElementById('gripper-deg-val');
    if (enabled) {
        gripperBox.removeAttribute('readonly');
        gripperBox.removeAttribute('tabindex');
        gripperBox.classList.remove('no-interact');
    } else {
        gripperBox.setAttribute('readonly', '');
        gripperBox.setAttribute('tabindex', '-1');
        gripperBox.classList.add('no-interact');
    }

    if (!enabled) {
        baseLinear = 0.0;
        baseAngular = 0.0;
    }
}

// Toggle handler — works on both desktop and mobile
let toggleTouched = false;
toggleWrap.addEventListener('touchstart', (e) => {
    e.stopPropagation();
    toggleTouched = true;
}, { passive: true });
toggleWrap.addEventListener('touchend', (e) => {
    e.stopPropagation();
    setManualMode(!manualMode);
}, { passive: true });
toggleWrap.addEventListener('click', () => {
    if (toggleTouched) { toggleTouched = false; return; }
    setManualMode(!manualMode);
});

// ══════════════════════════════════════════════════════════
// SLIDERS
// ══════════════════════════════════════════════════════════

const baseSpeedSlider = document.getElementById('base-speed');
const gantrySpeedSlider = document.getElementById('gantry-speed');

baseSpeedSlider.addEventListener('input', () => {
    document.getElementById('base-speed-val').textContent =
        parseFloat(baseSpeedSlider.value).toFixed(2);
});
gantrySpeedSlider.addEventListener('input', () => {
    document.getElementById('gantry-speed-val').textContent =
        parseFloat(gantrySpeedSlider.value).toFixed(3);
});

// ══════════════════════════════════════════════════════════
// JOYSTICK
// ══════════════════════════════════════════════════════════

function createJoystick(canvasId, activeColor, onMove, onEnd) {
    const canvas = document.getElementById(canvasId);
    const ctx = canvas.getContext('2d');
    const size = canvas.width;
    const cx = size / 2;
    const cy = size / 2;
    const maxR = size * 0.32;
    const knobR = size * 0.16;

    let active = false;
    let sx = 0;
    let sy = 0;
    let touchId = null;

    function draw() {
        ctx.clearRect(0, 0, size, size);

        // Color depends on manual mode state
        const color = manualMode ? activeColor : '#2a3a4a';

        // Outer ring
        ctx.beginPath();
        ctx.arc(cx, cy, size * 0.44, 0, Math.PI * 2);
        ctx.strokeStyle = manualMode ? '#1a3a5c' : '#1a2a3a';
        ctx.lineWidth = 1;
        ctx.stroke();

        // Crosshair
        ctx.strokeStyle = manualMode ? '#1a3a5c' : '#1a2a3a';
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(cx - size * 0.38, cy); ctx.lineTo(cx + size * 0.38, cy); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(cx, cy - size * 0.38); ctx.lineTo(cx, cy + size * 0.38); ctx.stroke();

        // Knob
        ctx.beginPath();
        ctx.arc(cx + sx, cy + sy, knobR, 0, Math.PI * 2);
        ctx.fillStyle = color + (active && manualMode ? 'ff' : '44');
        ctx.fill();
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;
        ctx.stroke();
    }

    function getOffset(e) {
        const rect = canvas.getBoundingClientRect();
        const scaleX = canvas.width / rect.width;
        const scaleY = canvas.height / rect.height;

        let clientX, clientY;
        if (e.touches || e.changedTouches) {
            const touches = e.touches.length ? e.touches : e.changedTouches;
            let touch = null;
            if (touchId !== null) {
                for (let t of touches) {
                    if (t.identifier === touchId) { touch = t; break; }
                }
            }
            if (!touch) touch = touches[0];
            if (!touch) return null;
            touchId = touch.identifier;
            clientX = touch.clientX;
            clientY = touch.clientY;
        } else {
            clientX = e.clientX;
            clientY = e.clientY;
        }

        return {
            x: (clientX - rect.left) * scaleX - cx,
            y: (clientY - rect.top) * scaleY - cy
        };
    }

    function onStart(e) {
        e.preventDefault();
        if (!manualMode) return;
        active = true;
        touchId = null;
        onMove(0, 0);
        draw();
    }

    function onMoveEvt(e) {
        e.preventDefault();
        if (!active || !manualMode) return;
        const off = getOffset(e);
        if (!off) return;
        const dist = Math.sqrt(off.x * off.x + off.y * off.y);
        const r = Math.min(dist, maxR);
        const angle = Math.atan2(off.y, off.x);
        sx = Math.cos(angle) * r;
        sy = Math.sin(angle) * r;
        onMove(sx / maxR, sy / maxR);
        draw();
    }

    function onEndEvt(e) {
        e.preventDefault();
        active = false;
        touchId = null;
        sx = 0;
        sy = 0;
        onEnd();
        draw();
    }

    canvas.addEventListener('touchstart', onStart, { passive: false });
    canvas.addEventListener('touchmove', onMoveEvt, { passive: false });
    canvas.addEventListener('touchend', onEndEvt, { passive: false });
    canvas.addEventListener('touchcancel', onEndEvt, { passive: false });
    canvas.addEventListener('mousedown', onStart);
    canvas.addEventListener('mousemove', (e) => { if (active) onMoveEvt(e); });
    canvas.addEventListener('mouseup', onEndEvt);
    canvas.addEventListener('mouseleave', onEndEvt);

    draw();
    return { redraw: draw };
}

// ══════════════════════════════════════════════════════════
// MOBILE BASE
// ══════════════════════════════════════════════════════════

let baseLinear = 0.0, baseAngular = 0.0;
let baseJoystick = null;

setInterval(() => {
    if (!manualMode) return;
    const now = Date.now();
    cmdVelTopic.publish(new ROSLIB.Message({
        header: {
            stamp: { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1000000 },
            frame_id: 'base_footprint'
        },
        twist: {
            linear: { x: Math.max(-BASE_MAX_SPEED_MS, Math.min(BASE_MAX_SPEED_MS, baseLinear)), y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: Math.max(-BASE_MAX_SPEED_MS * 1.5, Math.min(BASE_MAX_SPEED_MS * 1.5, baseAngular)) }
        }
    }));
}, 50);

// ══════════════════════════════════════════════════════════
// GANTRY
// ══════════════════════════════════════════════════════════

// Physical limits — must match URDF
const X_MIN = 0.0, X_MAX = 0.845;
const Y_MIN = 0.0, Y_MAX = 0.29;
const Z_MIN = 0.0, Z_MAX = 0.25;

// Deceleration starts this many metres before each hard limit
const SOFT_ZONE = 0.010;

function softLimit(vel, pos, lo, hi) {
    if (vel > 0) {
        const d = hi - pos;
        if (d <= 0) return 0;
        if (d < SOFT_ZONE) return vel * (d / SOFT_ZONE);
    } else if (vel < 0) {
        const d = pos - lo;
        if (d <= 0) return 0;
        if (d < SOFT_ZONE) return vel * (d / SOFT_ZONE);
    }
    return vel;
}

let currentX = 0.0, currentY = 0.0, currentZ = 0.0;
let gantryNx = 0.0, gantryNy = 0.0;
let zUpPressed = false, zDownPressed = false;
let gantryJoystick = null;

// Track current gantry position from joint states
const jointStateSub = new ROSLIB.Topic({
    ros, name: '/joint_states',
    messageType: 'sensor_msgs/JointState'
});
jointStateSub.subscribe((msg) => {
    for (let i = 0; i < msg.name.length; i++) {
        if (msg.name[i] === 'x_axis_joint') currentX = msg.position[i];
        if (msg.name[i] === 'y_axis_joint') currentY = msg.position[i];
        if (msg.name[i] === 'z_axis_joint') currentZ = msg.position[i];
    }
    const xBox = document.getElementById('gantry-x-val');
    const yBox = document.getElementById('gantry-y-val');
    const zBox = document.getElementById('gantry-z-val');
    if (document.activeElement !== xBox) xBox.value = mmStr(currentX);
    if (document.activeElement !== yBox) yBox.value = mmStr(currentY);
    if (document.activeElement !== zBox) zBox.value = mmStr(currentZ);
});

const odomSub = new ROSLIB.Topic({
    ros, name: '/diff_drive_controller/odom',
    messageType: 'nav_msgs/Odometry'
});
odomSub.subscribe((msg) => {
    const x = msg.pose.pose.position.x;
    const y = msg.pose.pose.position.y;
    const q = msg.pose.pose.orientation;
    const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    document.getElementById('odom-x-val').value = mStr(x);
    document.getElementById('odom-y-val').value = mStr(y);
    document.getElementById('odom-phi-val').value = (Math.abs(yaw) < 0.005 ? 0 : yaw * 180 / Math.PI).toFixed(1);
});

function sendGantryPosition(x, y, z) {
    x = Math.max(X_MIN, Math.min(X_MAX, x));
    y = Math.max(Y_MIN, Math.min(Y_MAX, y));
    z = Math.max(Z_MIN, Math.min(Z_MAX, z));
    const maxDist = Math.max(Math.abs(x - currentX), Math.abs(y - currentY), Math.abs(z - currentZ));
    const secs = Math.max(0.5, maxDist / GANTRY_MAX_SPEED_MS);
    const now = Date.now();
    jointTrajTopic.publish(new ROSLIB.Message({
        header: {
            stamp: { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1000000 },
            frame_id: ''
        },
        joint_names: ['x_axis_joint', 'y_axis_joint', 'z_axis_joint'],
        points: [{ positions: [x, y, z], velocities: [0.0, 0.0, 0.0],
                   time_from_start: { sec: Math.floor(secs), nanosec: Math.round((secs % 1) * 1e9) } }]
    }));
}

[
    ['gantry-x-val', (m) => [m, currentY, currentZ]],
    ['gantry-y-val', (m) => [currentX, m, currentZ]],
    ['gantry-z-val', (m) => [currentX, currentY, m]],
].forEach(([id, toXYZ]) => {
    document.getElementById(id).addEventListener('keydown', (e) => {
        if (e.key === 'Enter') {
            const mm = parseFloat(e.target.value);
            if (!isNaN(mm)) {
                const xyz = toXYZ(mm / 1000);
                // Always switch to JTC first (BEST_EFFORT = no-op if already active),
                // send trajectory only after switch is confirmed.
                switchControllerService.callService(
                    new ROSLIB.ServiceRequest({
                        activate_controllers: ['joint_trajectory_controller'],
                        deactivate_controllers: ['gantry_velocity_controller'],
                        strictness: 2
                    }),
                    () => sendGantryPosition(...xyz)
                );
            }
            e.target.blur();
        }
    });
});

function addButtonEvents(id, onPress, onRelease) {
    const btn = document.getElementById(id);
    btn.addEventListener('touchstart', (e) => { e.preventDefault(); if (manualMode) { onPress(); btn.classList.add('pressed'); } });
    btn.addEventListener('touchend', (e) => { e.preventDefault(); onRelease(); btn.classList.remove('pressed'); });
    btn.addEventListener('touchcancel', (e) => { e.preventDefault(); onRelease(); btn.classList.remove('pressed'); });
    btn.addEventListener('mousedown', () => { if (manualMode) { onPress(); btn.classList.add('pressed'); } });
    btn.addEventListener('mouseup', () => { onRelease(); btn.classList.remove('pressed'); });
    btn.addEventListener('mouseleave', () => { onRelease(); btn.classList.remove('pressed'); });
}

// Z arrows swapped — up button moves Z down, down button moves Z up
addButtonEvents('z-up', () => zDownPressed = true, () => zDownPressed = false);
addButtonEvents('z-down', () => zUpPressed = true, () => zUpPressed = false);

setInterval(() => {
    if (!manualMode) return;

    const speed = parseFloat(gantrySpeedSlider.value);

    let vx = gantryNx * speed;
    let vy = gantryNy * speed;
    let vz = zUpPressed ? speed
        : zDownPressed ? -speed
            : 0.0;

    // Decelerate in the last 10 mm before each physical limit, stop at the boundary
    vx = softLimit(vx, currentX, X_MIN, X_MAX);
    vy = softLimit(vy, currentY, Y_MIN, Y_MAX);
    vz = softLimit(vz, currentZ, Z_MIN, Z_MAX);

    gantryVelTopic.publish(new ROSLIB.Message({
        layout: { dim: [], data_offset: 0 },
        data: [vx, vy, vz]
    }));
}, 50);

// ══════════════════════════════════════════════════════════
// GRIPPER
// ══════════════════════════════════════════════════════════

const gripperSlider = document.getElementById('gripper-slider');
const gripperDegVal = document.getElementById('gripper-deg-val');
const gripperStateLabel = document.getElementById('gripper-state-label');

// Subscribe to confirmed angle feedback from gripper node
gripperCurrentAngleSub.subscribe((msg) => {
    const webVal = GRIPPER_MAX_DEG - msg.data;
    if (document.activeElement !== gripperDegVal) gripperDegVal.value = webVal;
    gripperStateLabel.textContent = (webVal === 0) ? 'closed'
        : (webVal === GRIPPER_MAX_DEG) ? 'open'
            : webVal + '°';
});

gripperSlider.addEventListener('change', () => {
    if (!manualMode) return;

    const webVal = parseInt(gripperSlider.value);
    // Invert — web 0 = closed = physical GRIPPER_MAX_DEG
    const physicalVal = GRIPPER_MAX_DEG - webVal;

    gripperDegVal.value = webVal;
    gripperStateLabel.textContent = (webVal === 0) ? 'closed'
        : (webVal === GRIPPER_MAX_DEG) ? 'open'
            : webVal + '°';

    gripperAngleTopic.publish(new ROSLIB.Message({ data: physicalVal }));
});

gripperDegVal.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
        const webVal = Math.round(Math.max(GRIPPER_MIN_DEG, Math.min(GRIPPER_MAX_DEG,
            parseFloat(e.target.value))));
        if (!isNaN(webVal)) {
            gripperDegVal.value = webVal;
            gripperSlider.value = webVal;
            gripperStateLabel.textContent = (webVal === 0) ? 'closed'
                : (webVal === GRIPPER_MAX_DEG) ? 'open'
                    : webVal + '°';
            gripperAngleTopic.publish(new ROSLIB.Message({ data: GRIPPER_MAX_DEG - webVal }));
        }
        e.target.blur();
    }
});

// ══════════════════════════════════════════════════════════
// SIZING — runs once on load
// computes joystick size from available screen space
// ══════════════════════════════════════════════════════════

window.onload = () => {
    setTimeout(() => {
        const screenH = Math.min(window.screen.width, window.screen.height);
        const isMobile = screenH < 500;
        const jsSize = isMobile ? 100 : 220;

        ['base-joystick', 'gantry-joystick'].forEach(id => {
            const c = document.getElementById(id);
            c.width = jsSize;
            c.height = jsSize;
            c.style.width = jsSize + 'px';
            c.style.height = jsSize + 'px';
        });

        // Set speed slider widths to match joystick size
        document.querySelectorAll('.hslider').forEach(s => {
            s.style.width = jsSize + 'px';
        });

        // Gripper panel width = two joystick panels + gap + z-controls + padding
        const gripperWidth = jsSize * 2 + 20 + 40 + 40;
        document.getElementById('panel-gripper').style.width = gripperWidth + 'px';
        document.getElementById('gripper-slider').style.width = (gripperWidth - 24) + 'px';

        // On mobile reduce gaps to fit footer on screen
        if (isMobile) {
            document.getElementById('main').style.gap = '8px';
            document.getElementById('main').style.padding = '4px';
            document.getElementById('joystick-row').style.gap = '10px';
            document.getElementById('controls-column').style.gap = '8px';
        }

        // Prevent virtual keyboard on permanently-readonly inputs (mobile guard)
        document.querySelectorAll('.no-interact-js').forEach(el => {
            el.addEventListener('focus', e => e.target.blur());
        });

        // Configure base speed slider from constants
        baseSpeedSlider.min = BASE_MIN_SPEED_MS.toFixed(2);
        baseSpeedSlider.max = BASE_MAX_SPEED_MS.toFixed(2);
        baseSpeedSlider.step = BASE_MIN_SPEED_MS.toFixed(2);
        baseSpeedSlider.value = (BASE_MAX_SPEED_MS * 0.5).toFixed(2);
        document.getElementById('base-speed-val').textContent =
            parseFloat(baseSpeedSlider.value).toFixed(2);

        // Configure gantry speed slider from constants
        gantrySpeedSlider.min = GANTRY_MIN_SPEED_MS.toFixed(3);
        gantrySpeedSlider.max = GANTRY_MAX_SPEED_MS.toFixed(3);
        gantrySpeedSlider.step = GANTRY_MIN_SPEED_MS.toFixed(3);
        gantrySpeedSlider.value = (GANTRY_MAX_SPEED_MS * 0.5).toFixed(3);
        document.getElementById('gantry-speed-val').textContent =
            parseFloat(gantrySpeedSlider.value).toFixed(3);

        const zSize = Math.min(Math.floor(jsSize * 0.36), 52);
        document.getElementById('z-controls').style.height = jsSize + 'px';
        document.querySelectorAll('.z-btn').forEach(b => {
            b.style.width = zSize + 'px';
            b.style.height = zSize + 'px';
            b.style.fontSize = Math.floor(zSize * 0.4) + 'px';
        });

        baseJoystick = createJoystick('base-joystick', '#e94560',
            (nx, ny) => {
                if (!manualMode) return;
                const speed = parseFloat(baseSpeedSlider.value);
                baseLinear = -ny * speed;
                baseAngular = -nx * speed * 1.5;
            },
            () => { baseLinear = 0.0; baseAngular = 0.0; }
        );

        gantryJoystick = createJoystick('gantry-joystick', '#00ff88',
            (nx, ny) => { if (manualMode) { gantryNx = nx; gantryNy = -ny; } },
            () => { gantryNx = 0.0; gantryNy = 0.0; }
        );

    }, 100);
};