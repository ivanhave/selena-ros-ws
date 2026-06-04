// ══════════════════════════════════════════════════════════
// PLANTING STATS (mirrors PlantingPlanner::init)
// ══════════════════════════════════════════════════════════

function computePlantingStats(seed, zoneLengthM) {
    const sx = seed.spacing_x_mm / 1000;
    const sy = seed.spacing_y_mm / 1000;
    const seedsInX    = Math.floor(GANTRY_X_TRAVEL / sx) + 1;
    const seedsInY    = Math.floor(GANTRY_Y_TRAVEL / sy) + 1;
    const seedsPerStop = seedsInX * seedsInY;
    const totalRows   = Math.floor(zoneLengthM / sy) + 1;
    const totalSeeds  = totalRows * seedsInX;
    const totalStops  = Math.ceil(totalSeeds / seedsPerStop);
    const estMinutes  = Math.ceil(totalStops * 30 / 60);  // ~30 s/stop estimate
    return { seedsInX, seedsInY, seedsPerStop, totalStops, totalSeeds, estMinutes };
}

// Returns [{x, y}] world positions of each robot stop for a zone.
// Mirrors the zoneToGoal start-offset and C++ PlantingPlanner advance math.
// The robot advances seedsInY * sy meters per stop along the strip.
// The start is shifted forward by (seedsInY-1)*sy/2 so the last row of stop 0
// aligns with zone.start_along (matching _zoneStartOffset in missions.ros.js).
function computeStopPositions(zone) {
    if (!fieldConfig) return [];
    const seed = state.seeds.find(s => s.name === zone.target);
    if (!seed) return [];

    const sy           = seed.spacing_y_mm / 1000;
    const seedsInY     = Math.floor(GANTRY_Y_TRAVEL / sy) + 1;
    const startOffset  = (seedsInY - 1) * sy / 2;   // same as _zoneStartOffset
    const robotAdvance = seedsInY * sy;

    const stats = computePlantingStats(seed, zone.end_along - zone.start_along);
    if (!stats.totalStops) return [];

    const f    = fieldConfig.field;
    const yaw  = f.origin.yaw || 0;
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
    const cx   = f.origin.x - zone.strip_id * f.strip_spacing * sinY;
    const cy   = f.origin.y + zone.strip_id * f.strip_spacing * cosY;

    const stops = [];
    for (let i = 0; i < stats.totalStops; i++) {
        const d = zone.start_along + startOffset + i * robotAdvance;
        stops.push({ x: cx + d * cosY, y: cy + d * sinY });
    }
    return stops;
}

// ══════════════════════════════════════════════════════════
// ZONE SELECTION — hit test and state machine
// ══════════════════════════════════════════════════════════

function hitTestStrip(wx, wy) {
    if (!fieldConfig) return null;
    const f = fieldConfig.field;
    const yaw = f.origin.yaw || 0;
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
    for (const strip of (f.strips || [])) {
        if (!strip.enabled) continue;
        const c0 = {
            x: f.origin.x - strip.id * f.strip_spacing * sinY,
            y: f.origin.y + strip.id * f.strip_spacing * cosY,
        };
        const dx = wx - c0.x, dy = wy - c0.y;
        const along  = dx * cosY + dy * sinY;
        const across = -dx * sinY + dy * cosY;
        if (along >= 0 && along <= f.strip_length && Math.abs(across) <= f.strip_width / 2) {
            return { stripId: strip.id, alongM: along };
        }
    }
    return null;
}

function snapAlong(along) {
    if (!fieldConfig) return Math.round(along / SNAP_M) * SNAP_M;
    const L = fieldConfig.field.strip_length;
    return Math.max(0, Math.min(L, Math.round(along / SNAP_M) * SNAP_M));
}

function checkOverlap(stripId, startAlong, endAlong) {
    return (missionZones.zones || []).some(z =>
        z.strip_id === stripId &&
        startAlong < z.end_along &&
        endAlong   > z.start_along
    );
}

function handleMapClick(wx, wy) {
    if (activeMissionZoneId) return;
    const hit = hitTestStrip(wx, wy);

    if (zonePreview.phase === 'IDLE') {
        if (!hit) return;
        // First click: place start
        zonePreview.phase      = 'AWAITING_END';
        zonePreview.stripId    = hit.stripId;
        zonePreview.startAlong = snapAlong(hit.alongM);
        zonePreview.endAlong   = null;
        zonePreview.hoverAlong = null;
        zonePreview.overlap    = false;
        renderZonePlanner();
        mapRenderer.scheduleDraw();
        return;
    }

    if (zonePreview.phase === 'AWAITING_END' || zonePreview.phase === 'ZONE_READY') {
        if (!hit) {
            clearZoneSelection();
            return;
        }
        if (hit.stripId !== zonePreview.stripId) {
            // Clicked different strip — restart on it
            zonePreview.phase      = 'AWAITING_END';
            zonePreview.stripId    = hit.stripId;
            zonePreview.startAlong = snapAlong(hit.alongM);
            zonePreview.endAlong   = null;
            zonePreview.hoverAlong = null;
            zonePreview.overlap    = false;
        } else {
            // Same strip — place end
            let snapped = snapAlong(hit.alongM);
            // Swap if end is before start
            if (snapped < zonePreview.startAlong) {
                [snapped, zonePreview.startAlong] = [zonePreview.startAlong, snapped];
            }
            if (snapped - zonePreview.startAlong < SNAP_M) {
                // Zone too small — reset start to clicked position, stay in AWAITING_END
                zonePreview.startAlong = snapAlong(hit.alongM);
                zonePreview.endAlong   = null;
                zonePreview.phase      = 'AWAITING_END';
            } else {
                zonePreview.endAlong = snapped;
                zonePreview.phase    = 'ZONE_READY';
                zonePreview.overlap  = checkOverlap(
                    zonePreview.stripId, zonePreview.startAlong, zonePreview.endAlong);
            }
        }
        renderZonePlanner();
        mapRenderer.scheduleDraw();
    }
}

function handleMapHover(wx, wy) {
    if (activeMissionZoneId) return;
    if (zonePreview.phase !== 'AWAITING_END') return;
    const hit = hitTestStrip(wx, wy);
    const newHover = (hit && hit.stripId === zonePreview.stripId)
        ? snapAlong(hit.alongM)
        : null;
    if (newHover === zonePreview.hoverAlong) return;
    zonePreview.hoverAlong = newHover;
    mapRenderer.scheduleDraw();
}

function handleMapHoverEnd() {
    if (zonePreview.phase !== 'AWAITING_END') return;
    zonePreview.hoverAlong = null;
    mapRenderer.scheduleDraw();
}

function clearZoneSelection() {
    zonePreview.phase      = 'IDLE';
    zonePreview.stripId    = null;
    zonePreview.startAlong = null;
    zonePreview.endAlong   = null;
    zonePreview.hoverAlong = null;
    zonePreview.overlap    = false;
    renderZonePlanner();
    if (mapRenderer) mapRenderer.scheduleDraw();
}

document.addEventListener('keydown', e => { if (e.key === 'Escape') clearZoneSelection(); });

// ══════════════════════════════════════════════════════════
// ZONE PLANNER UI
// ══════════════════════════════════════════════════════════

let zpSeedIndex = 0;

function populateZpSeedSelect() {
    const list  = document.getElementById('zp-seed-list');
    const label = document.getElementById('zp-seed-label');
    if (!list) return;
    // Clamp index if seeds were removed
    if (zpSeedIndex >= state.seeds.length) zpSeedIndex = Math.max(0, state.seeds.length - 1);
    list.innerHTML = '';
    state.seeds.forEach((seed, i) => {
        const item = document.createElement('div');
        item.className = 'custom-select-option' + (i === zpSeedIndex ? ' is-selected' : '');
        item.textContent = seed.name;
        item.addEventListener('click', () => {
            zpSeedIndex = i;
            list.classList.add('hidden');
            renderZonePlanner();
        });
        list.appendChild(item);
    });
    label.textContent = state.seeds[zpSeedIndex]?.name ?? '—';
}

function renderZonePlanner() {
    const instrEl     = document.getElementById('zp-instruction');
    const coordsEl    = document.getElementById('zp-coords');
    const stripEl     = document.getElementById('zp-strip-id');
    const startEl     = document.getElementById('zp-start');
    const endRowEl    = document.getElementById('zp-end-row');
    const endEl       = document.getElementById('zp-end');
    const lenEl       = document.getElementById('zp-length');
    const seedRowEl   = document.getElementById('zp-seed-row');
    const statsEl     = document.getElementById('zp-stats');
    const overlapErr  = document.getElementById('zp-overlap-error');
    const addBtn      = document.getElementById('btn-zp-add');

    const hasStrips = fieldConfig && (fieldConfig.field.strips || []).some(s => s.enabled);

    if (zonePreview.phase === 'IDLE') {
        instrEl.textContent = hasStrips
            ? 'Click a strip on the map to place start'
            : 'No strips defined — configure field first';
        instrEl.classList.remove('hidden');
        coordsEl.classList.add('hidden');
        seedRowEl.classList.add('hidden');
        statsEl.classList.add('hidden');
        overlapErr.classList.add('hidden');
        addBtn.disabled = true;
        return;
    }

    coordsEl.classList.remove('hidden');
    stripEl.textContent = 'Strip ' + (zonePreview.stripId + 1);
    startEl.textContent = zonePreview.startAlong.toFixed(1) + ' m';

    if (zonePreview.phase === 'AWAITING_END') {
        instrEl.textContent = 'Click same strip to place end';
        endRowEl.classList.add('hidden');
        seedRowEl.classList.add('hidden');
        statsEl.classList.add('hidden');
        overlapErr.classList.add('hidden');
        addBtn.disabled = true;
        return;
    }

    // ZONE_READY
    const length = zonePreview.endAlong - zonePreview.startAlong;
    instrEl.textContent = zonePreview.overlap
        ? 'Zone overlaps an existing zone'
        : 'Zone ready — choose seed and confirm';
    endRowEl.classList.remove('hidden');
    endEl.textContent = zonePreview.endAlong.toFixed(1) + ' m';
    lenEl.textContent = length.toFixed(1) + ' m';

    seedRowEl.classList.remove('hidden');
    populateZpSeedSelect();

    const seed = state.seeds[zpSeedIndex];
    if (seed) {
        const stats = computePlantingStats(seed, length);
        statsEl.innerHTML =
            '<span class="spec-label">Seeds / stop</span><span class="spec-value">' + stats.seedsInX + '×' + stats.seedsInY + ' = ' + stats.seedsPerStop + '</span>' +
            '<span class="spec-label">Total stops</span><span class="spec-value">' + stats.totalStops + '</span>'  +
            '<span class="spec-label">Total seeds</span><span class="spec-value">' + stats.totalSeeds + '</span>' +
            '<span class="spec-label">Est. time</span><span class="spec-value">~' + stats.estMinutes + ' min</span>';
        statsEl.classList.remove('hidden');
    }

    overlapErr.classList.toggle('hidden', !zonePreview.overlap);
    addBtn.disabled = zonePreview.overlap;
}

document.getElementById('zp-seed-trigger').addEventListener('click', () => {
    document.getElementById('zp-seed-list').classList.toggle('hidden');
});
document.addEventListener('click', e => {
    if (!e.target.closest('#zp-seed-select-wrap'))
        document.getElementById('zp-seed-list').classList.add('hidden');
});

document.getElementById('btn-zp-clear').addEventListener('click', clearZoneSelection);
document.getElementById('btn-zp-add').addEventListener('click', addZone);

// ══════════════════════════════════════════════════════════
// ZONES LIST
// ══════════════════════════════════════════════════════════

function getZoneNumber(zoneId) {
    const idx = (missionZones.zones || []).findIndex(z => z.zone_id === zoneId);
    return idx >= 0 ? idx + 1 : '?';
}

function renderZonesList() {
    if (mapRenderer) mapRenderer.scheduleDraw();
    updateFieldConfigLock();
    const listEl = document.getElementById('zones-list');
    const zones = missionZones.zones || [];
    if (zones.length === 0) {
        listEl.innerHTML = '<span class="field-hint">No zones defined</span>';
        return;
    }
    listEl.innerHTML = zones.map((zone, zoneIdx) => {
        const zoneNum  = zoneIdx + 1;
        const zoneLen  = zone.end_along - zone.start_along;
        const len      = zoneLen.toFixed(1);
        const area     = (zoneLen * GANTRY_X_TRAVEL).toFixed(1);
        const seed     = state.seeds.find(s => s.name === zone.target);
        const seedsText = seed ? computePlantingStats(seed, zoneLen).totalSeeds + ' seeds' : '—';
        const isActive     = zone.zone_id === activeMissionZoneId;
        const isCancelling = zone.zone_id === _cancellingZoneId;
        const fb       = isActive ? zoneMissionFeedback : null;
        const pct      = fb ? Math.round(fb.progress_percent) : 0;
        const progressEl = isActive
            ? `<div class="zone-progress-bar"><div class="zone-progress-fill" style="width:${pct}%"></div></div>`
            : '';
        let startBtn  = '';
        let cancelBtn = '';
        let deleteBtn = '';
        if (!isCancelling) {
            startBtn = (zone.status === 'planned' && !isActive)
                ? `<button class="zone-start-btn" data-zone-id="${zone.zone_id}"
                           ${activeMissionZoneId ? 'disabled' : ''} title="Start zone mission">▶</button>`
                : '';
            // ■ always appears for in_progress zones (active → cancel; stale → reset to planned)
            cancelBtn = isActive
                ? `<button class="zone-cancel-btn" title="Cancel mission">■</button>`
                : (zone.status === 'in_progress'
                    ? `<button class="zone-reset-btn" data-zone-id="${zone.zone_id}" title="Reset to planned (no mission running)">■</button>`
                    : '');
            // ✕ only for planned zones; never for in_progress or completed
            deleteBtn = (!isActive && zone.status === 'planned' && !activeMissionZoneId)
                ? `<button class="zone-delete-btn" data-zone-id="${zone.zone_id}" title="Delete zone">✕</button>`
                : '';
        }
        const badgeStatus = isCancelling ? 'cancelling' : zone.status;
        return `<div class="zone-item">
            <div class="zone-number-badge">${zoneNum}</div>
            <div class="zone-item-info">
                <span class="zi-primary"><span class="zi-lbl">seed type</span> — ${zone.target}</span>
                <div class="zi-stats-row">
                    <span class="zi-stat-chip"><span class="zi-lbl">length</span> ${len} m</span>
                    <span class="zi-stat-chip"><span class="zi-lbl">area</span> ${area} m²</span>
                    <span class="zi-stat-chip">${seedsText}</span>
                </div>
                ${progressEl}
            </div>
            <div class="zone-item-right">
                <span class="zone-status-badge zone-status--${badgeStatus}">${badgeStatus}</span>
                ${startBtn}${cancelBtn}${deleteBtn}
            </div>
        </div>`;
    }).join('');

    listEl.querySelectorAll('.zone-start-btn').forEach(btn => {
        btn.addEventListener('click', e => { e.stopPropagation(); startZoneMission(btn.dataset.zoneId); });
    });
    listEl.querySelectorAll('.zone-cancel-btn').forEach(btn => {
        btn.addEventListener('click', e => { e.stopPropagation(); showCancelZoneModal(); });
    });
    listEl.querySelectorAll('.zone-reset-btn').forEach(btn => {
        btn.addEventListener('click', e => {
            e.stopPropagation();
            const z = (missionZones.zones || []).find(z => z.zone_id === btn.dataset.zoneId);
            if (z && z.status === 'in_progress') {
                z.status = 'planned';
                saveMissionZones();
                renderZonesList();
                if (mapRenderer) mapRenderer.scheduleDraw();
            }
        });
    });
    listEl.querySelectorAll('.zone-delete-btn').forEach(btn => {
        btn.addEventListener('click', e => { e.stopPropagation(); showDeleteZoneModal(btn.dataset.zoneId); });
    });
}

function addZone() {
    if (activeMissionZoneId) return;
    if (zonePreview.phase !== 'ZONE_READY' || zonePreview.overlap) return;
    const seed = state.seeds[zpSeedIndex];
    if (!seed) return;

    const zone = {
        zone_id:     String(Date.now()),
        strip_id:    zonePreview.stripId,
        start_along: parseFloat(zonePreview.startAlong.toFixed(1)),
        end_along:   parseFloat(zonePreview.endAlong.toFixed(1)),
        mission_type: 'plant',
        target:      seed.name,
        status:      'planned',
    };

    missionZones.zones = missionZones.zones || [];
    missionZones.zones.push(zone);
    saveMissionZones();
    renderZonesList();
    clearZoneSelection();
    mapRenderer.scheduleDraw();
}

function deleteZone(zoneId) {
    if (zoneId === activeMissionZoneId) return;
    missionZones.zones = (missionZones.zones || []).filter(z => z.zone_id !== zoneId);
    saveMissionZones();
    renderZonesList();
    if (mapRenderer) mapRenderer.scheduleDraw();
}

function showDeleteZoneModal(zoneId) {
    const zone = (missionZones.zones || []).find(z => z.zone_id === zoneId);
    if (!zone) return;
    const zoneNum = getZoneNumber(zoneId);
    document.getElementById('modal-delete-title').textContent = 'Delete Zone';
    document.getElementById('modal-delete-text').textContent =
        `Delete Zone ${zoneNum} — ${zone.target}? This cannot be undone.`;
    pendingDeleteAction = () => deleteZone(zoneId);
    showModal('modal-delete-confirm');
}
