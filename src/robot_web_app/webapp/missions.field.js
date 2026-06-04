// ══════════════════════════════════════════════════════════
// ROBOT CONFIG (from /api/robot_config — parsed from mobile_base.xacro)
// ══════════════════════════════════════════════════════════

async function loadRobotConfig() {
    try {
        const res = await fetch('/api/robot_config');
        if (!res.ok) return;
        robotConfig = await res.json();
        GANTRY_X_TRAVEL = robotConfig.x_axis_travel ?? GANTRY_X_TRAVEL;
        GANTRY_Y_TRAVEL = robotConfig.y_axis_travel ?? GANTRY_Y_TRAVEL;
        if (mapRenderer) mapRenderer.scheduleDraw();
        // Refresh min constraint now that we have real robot dimensions
        if (fieldConfig) renderFieldConfigEditor();
    } catch (_) {}
}

// ══════════════════════════════════════════════════════════
// FIELD CONFIG API
// ══════════════════════════════════════════════════════════

async function loadFieldConfig() {
    try {
        const res = await fetch('/api/field_config');
        if (!res.ok) return;
        fieldConfig = await res.json();
        renderFieldConfigEditor();
        renderZonePlanner();
        if (mapRenderer) mapRenderer.resetView(fieldConfig);
    } catch (_) {}
}

function hasInProgressZone() {
    return (missionZones.zones || []).some(z => z.status === 'in_progress');
}

function updateFieldConfigLock() {
    const locked = hasInProgressZone();
    ['fc-strip-length', 'fc-strip-spacing', 'fc-strip-count',
     'fc-origin-x', 'fc-origin-y', 'fc-origin-yaw'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = locked;
    });
    ['btn-fc-fit', 'btn-rotate-ccw', 'btn-rotate-cw'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = locked;
    });
}

async function saveFieldConfig() {
    if (activeMissionZoneId) return;
    if (hasInProgressZone()) { showModal('modal-inprogress-lock'); return; }
    try {
        const res = await fetch('/api/field_config', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(fieldConfig),
        });
        const data = await res.json();
        const statusEl = document.getElementById('fc-save-status');
        if (!res.ok || !data.ok) { statusEl.textContent = 'Error'; return; }
        statusEl.textContent = 'Saved';
        setTimeout(() => { statusEl.textContent = ''; }, 2000);
        if (data.wiped_planned > 0) {
            await loadMissionZones();
            const n = data.wiped_planned;
            document.getElementById('modal-zones-wiped-text').textContent =
                `${n} planned zone${n !== 1 ? 's' : ''} were removed because the field configuration changed.`;
            showModal('modal-zones-wiped');
        }
    } catch (_) {
        document.getElementById('fc-save-status').textContent = 'Error';
    }
}

// ══════════════════════════════════════════════════════════
// SEEDS API  (persisted in robot_missions/config/seeds.csv)
// ══════════════════════════════════════════════════════════

async function loadSeeds() {
    try {
        const res = await fetch('/api/seeds');
        if (!res.ok) return;
        state.seeds = await res.json();
        populateSeedSelect();
        renderSeedSpecs();
        populateZpSeedSelect();
    } catch (_) {}
}

async function saveSeeds() {
    try {
        await fetch('/api/seeds', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(state.seeds),
        });
    } catch (_) {}
}

async function loadMissionZones() {
    try {
        const res = await fetch('/api/mission_zones');
        if (!res.ok) return;
        missionZones = await res.json();
        // Reset only 'failed' zones immediately.
        // 'in_progress' zones are left as-is: _subscribeActionStatus resolves them
        // against the live action server state within ~1 s of connecting.
        let dirty = false;
        (missionZones.zones || []).forEach(z => {
            if (z.status === 'failed') { z.status = 'planned'; dirty = true; }
        });
        if (dirty) saveMissionZones();
        // Provisionally restore active mission state immediately so the cancel button
        // and progress bar show at once — status topic confirms/revokes within ~1 s.
        const _ipZone = (missionZones.zones || []).find(z => z.status === 'in_progress');
        if (_ipZone && !activeMissionZoneId) {
            activeMissionZoneId = _ipZone.zone_id;
            _restorePersistedMissionState(_ipZone.zone_id);
            setMissionStatusBar(_ipZone);
        }
        // Handle the race where the action status topic fired before zones were loaded
        if (typeof _checkInProgressZoneRestore === 'function') _checkInProgressZoneRestore();
        renderZonesList();
        if (mapRenderer) mapRenderer.scheduleDraw();
    } catch (_) {}
}

async function saveMissionZones() {
    try {
        await fetch('/api/mission_zones', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(missionZones),
        });
    } catch (_) {}
}

// ══════════════════════════════════════════════════════════
// FIELD CONFIG EDITOR
// ══════════════════════════════════════════════════════════

function getMinStripSpacing() {
    const rc = robotConfig;
    // Distance from robot centre to outer wheel edge, plus strip half-width
    // All values come from mobile_base.xacro via /api/robot_config
    const whlY      = (rc?.leg_offset_y  ?? 0.5795) + (rc?.wheel_offset_y ?? 0.058);
    const whlWidth  =  rc?.wheel_width   ?? 0.10;    // tread width in Y (axle direction)
    const halfStrip = (rc?.x_axis_travel ?? GANTRY_X_TRAVEL) / 2;
    return Math.ceil((whlY + whlWidth / 2 + halfStrip) * 100) / 100;  // round up to cm
}

function _updateSpacingWarning(spacing, minSpacing, clamped) {
    const el = document.getElementById('fc-spacing-warning');
    if (!el) return;
    if (spacing < minSpacing) {
        el.textContent = clamped
            ? `Spacing set to minimum ${minSpacing} m — smaller values would put wheels on the adjacent strip.`
            : `Cannot be less than ${minSpacing} m — wheels would overlap the adjacent strip.`;
        el.classList.remove('hidden');
    } else {
        el.classList.add('hidden');
    }
}

function renderFieldConfigEditor() {
    if (!fieldConfig) return;
    const f = fieldConfig.field;
    const minSpacing = getMinStripSpacing();
    const spacingInput = document.getElementById('fc-strip-spacing');
    spacingInput.value = f.strip_spacing;
    spacingInput.min   = minSpacing;
    document.getElementById('fc-strip-length').value  = f.strip_length;
    document.getElementById('fc-strip-count').value   = (f.strips || []).length;
    document.getElementById('fc-origin-x').value      = f.origin.x;
    document.getElementById('fc-origin-y').value      = f.origin.y;
    document.getElementById('fc-origin-yaw').value    = parseFloat((f.origin.yaw * 180 / Math.PI).toFixed(2));
    _updateSpacingWarning(f.strip_spacing, minSpacing, false);
    updateFieldConfigLock();
}

function readFieldConfigEditor() {
    if (!fieldConfig) return;
    const f = fieldConfig.field;
    const minSpacing = getMinStripSpacing();
    const rawLength = parseFloat(document.getElementById('fc-strip-length').value);
    if (!isNaN(rawLength) && rawLength > 0) f.strip_length = rawLength;
    const spacingInput = document.getElementById('fc-strip-spacing');
    const enteredSpacing = parseFloat(spacingInput.value);
    if (!isNaN(enteredSpacing)) {
        const clamped = enteredSpacing < minSpacing;
        f.strip_spacing = clamped ? minSpacing : enteredSpacing;
        if (clamped) spacingInput.value = minSpacing;
        _updateSpacingWarning(f.strip_spacing, minSpacing, clamped);
    }
    f.origin.x   = parseFloat(document.getElementById('fc-origin-x').value)   || 0;
    f.origin.y   = parseFloat(document.getElementById('fc-origin-y').value)   || 0;
    f.origin.yaw = (parseFloat(document.getElementById('fc-origin-yaw').value) || 0) * Math.PI / 180;
    const newCount = Math.max(1, parseInt(document.getElementById('fc-strip-count').value, 10) || 1);
    const strips = f.strips || [];
    while (strips.length < newCount) strips.push({ id: strips.length, enabled: true });
    if (strips.length > newCount) strips.splice(newCount);
    f.strips = strips;
}

// ══════════════════════════════════════════════════════════
// STRIP ROTATION
// ══════════════════════════════════════════════════════════

function rotateStrips(direction) {   // +1 = CW (+90°), -1 = CCW (-90°)
    if (activeMissionZoneId) return;
    if (hasInProgressZone()) { showModal('modal-inprogress-lock'); return; }
    if (!fieldConfig) return;
    const f = fieldConfig.field;
    let yaw = (f.origin.yaw || 0) + direction * Math.PI / 2;
    // Snap to exact multiples of π/2 to avoid floating-point drift
    yaw = Math.round(yaw / (Math.PI / 2)) * (Math.PI / 2);
    // Normalize to (-π, π]
    while (yaw >  Math.PI) yaw -= 2 * Math.PI;
    while (yaw <= -Math.PI) yaw += 2 * Math.PI;
    f.origin.yaw = yaw;
    document.getElementById('fc-origin-yaw').value = parseFloat((yaw * 180 / Math.PI).toFixed(2));
    if (mapRenderer) { mapRenderer.resetView(fieldConfig); }
    saveFieldConfig();
}

document.getElementById('btn-rotate-ccw').addEventListener('click', () => rotateStrips(-1));
document.getElementById('btn-rotate-cw').addEventListener('click',  () => rotateStrips(+1));

document.getElementById('fc-strip-spacing').addEventListener('input', () => {
    const v = parseFloat(document.getElementById('fc-strip-spacing').value);
    if (!isNaN(v)) _updateSpacingWarning(v, getMinStripSpacing(), false);
});

document.getElementById('fc-strip-count').addEventListener('change', () => {
    const el = document.getElementById('fc-strip-count');
    const v = Math.floor(parseFloat(el.value));
    el.value = isNaN(v) || v < 1 ? 1 : v;
});

document.getElementById('fc-strip-length').addEventListener('change', () => {
    const el = document.getElementById('fc-strip-length');
    const v = parseFloat(el.value);
    if (isNaN(v) || v <= 0) el.value = fieldConfig?.field?.strip_length ?? 1.0;
});

document.getElementById('btn-fc-fit').addEventListener('click', () => {
    readFieldConfigEditor();
    saveFieldConfig();
    renderZonePlanner();
    if (mapRenderer) mapRenderer.resetView(fieldConfig);
});
