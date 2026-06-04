// ══════════════════════════════════════════════════════════
// SEED SELECT (Planting card)
// ══════════════════════════════════════════════════════════

const seedSelectTrigger = document.getElementById('seed-select-trigger');
const seedSelectLabel   = document.getElementById('seed-select-label');
const seedSelectList    = document.getElementById('seed-select-list');
const seedSpecs         = document.getElementById('seed-specs');
const btnStartMission   = document.getElementById('btn-start-mission');
const btnAddSeed        = document.getElementById('btn-add-seed');
const btnEditSeed       = document.getElementById('btn-edit-seed');
const btnDeleteSeed     = document.getElementById('btn-delete-seed');
const missionStatusBar  = document.getElementById('mission-status-bar');
const missionStatusText = document.getElementById('mission-status-text');
const modalOverlay      = document.getElementById('modal-overlay');
const cardWeeding       = document.getElementById('mission-card-weeding');
const cardWatering      = document.getElementById('mission-card-watering');

function populateSeedSelect() {
    seedSelectList.innerHTML = '';
    state.seeds.forEach(function (seed, i) {
        const item = document.createElement('div');
        item.className = 'custom-select-option' + (i === state.selectedSeedIndex ? ' is-selected' : '');
        item.textContent = seed.name;
        item.addEventListener('click', function () {
            state.selectedSeedIndex = i;
            renderSeedSpecs();
            populateSeedSelect();
            seedSelectList.classList.add('hidden');
        });
        seedSelectList.appendChild(item);
    });
    const sel = state.seeds[state.selectedSeedIndex];
    seedSelectLabel.textContent = sel ? sel.name : '—';
}

seedSelectTrigger.addEventListener('click', function () {
    if (this.disabled) return;
    seedSelectList.classList.toggle('hidden');
});

document.addEventListener('click', function (e) {
    if (!e.target.closest('#seed-select-wrap')) seedSelectList.classList.add('hidden');
});

function renderSeedSpecs() {
    const seed = state.seeds[state.selectedSeedIndex];
    if (!seed) { seedSpecs.innerHTML = ''; return; }
    const toolLabel  = seed.tool_id === 1 ? 'Gripper' : 'Vacuum';
    const paramLabel = seed.tool_id === 1 ? seed.tool_param + '°' : seed.tool_param + ' ms';
    function item(label, value) {
        return '<div class="spec-item"><span class="spec-label">' + label + '</span><span class="spec-value">' + value + '</span></div>';
    }
    seedSpecs.innerHTML =
        item('Spacing X', seed.spacing_x_mm + ' mm') +
        item('Spacing Y', seed.spacing_y_mm + ' mm') +
        item('Depth',     seed.depth_mm     + ' mm') +
        item('Tool',      toolLabel)                  +
        item('Param',     paramLabel);
}


// ══════════════════════════════════════════════════════════
// MISSION PHASE
// ══════════════════════════════════════════════════════════

function renderMissionPhase() {
    if (state.missionPhase === 'IDLE') {
        if (!activeMissionZoneId) {
            missionStatusText.textContent = 'None';
            missionStatusBar.classList.remove('active');
        }
        document.getElementById('mission-card-planting').classList.remove('mission-card--active');
        cardWeeding.classList.remove('mission-card--locked');
        cardWatering.classList.remove('mission-card--locked');
        btnStartMission.textContent = 'Start Mission';
        btnStartMission.classList.remove('cancel-mode');
        seedSelectTrigger.disabled = false; btnAddSeed.disabled = false; btnEditSeed.disabled = false; btnDeleteSeed.disabled = false;
    } else {
        if (!activeMissionZoneId) {
            const seedName = state.seeds[state.selectedSeedIndex]?.name || 'Unknown';
            missionStatusText.textContent = 'Planting — ' + seedName;
            missionStatusBar.classList.add('active');
        }
        document.getElementById('mission-card-planting').classList.add('mission-card--active');
        cardWeeding.classList.add('mission-card--locked');
        cardWatering.classList.add('mission-card--locked');
        btnStartMission.textContent = 'Cancel Mission';
        btnStartMission.classList.add('cancel-mode');
        seedSelectTrigger.disabled = true; btnAddSeed.disabled = true; btnEditSeed.disabled = true; btnDeleteSeed.disabled = true;
    }
}

btnStartMission.addEventListener('click', function () {
    if (state.missionPhase === 'IDLE') { state.missionPhase = 'ACTIVE'; renderMissionPhase(); }
    else showCancelModal();
});

// ══════════════════════════════════════════════════════════
// MODALS
// ══════════════════════════════════════════════════════════

function showModal(id) {
    document.querySelectorAll('.modal').forEach(m => m.classList.add('hidden'));
    document.getElementById(id).classList.remove('hidden');
    modalOverlay.classList.remove('hidden');
}

function hideModal() {
    modalOverlay.classList.add('hidden');
    document.querySelectorAll('.modal').forEach(m => m.classList.add('hidden'));
}

modalOverlay.addEventListener('click', e => { if (e.target === modalOverlay) hideModal(); });

function showCancelModal() {
    const seedName = state.seeds[state.selectedSeedIndex]?.name || 'Unknown';
    document.getElementById('modal-cancel-text').textContent =
        'Are you sure you want to cancel the Planting mission (' + seedName + ')?';
    showModal('modal-cancel-confirm');
}

document.getElementById('btn-cancel-yes').addEventListener('click', () => {
    if (activeMissionZoneId) {
        cancelZoneMission();
    } else {
        state.missionPhase = 'IDLE'; renderMissionPhase();
    }
    hideModal();
});
document.getElementById('btn-cancel-no').addEventListener('click', hideModal);

function showAddSeedModal() {
    state.isAdding = true; state.editingSeedIndex = null;
    document.getElementById('modal-seed-form-title').textContent = 'Add Seed';
    document.getElementById('seed-form').reset();
    document.getElementById('seed-form-error').textContent = '';
    document.getElementById('seed-form-error').classList.add('hidden');
    showModal('modal-seed-form');
}

function showEditSeedModal() {
    state.isAdding = false; state.editingSeedIndex = state.selectedSeedIndex;
    const seed = state.seeds[state.selectedSeedIndex];
    document.getElementById('modal-seed-form-title').textContent = 'Edit Seed';
    document.getElementById('form-name').value       = seed.name;
    document.getElementById('form-spacing-x').value  = seed.spacing_x_mm;
    document.getElementById('form-spacing-y').value  = seed.spacing_y_mm;
    document.getElementById('form-depth').value      = seed.depth_mm;
    document.getElementById('form-tool-id').value    = seed.tool_id;
    document.getElementById('form-tool-param').value = seed.tool_param;
    document.getElementById('seed-form-error').textContent = '';
    document.getElementById('seed-form-error').classList.add('hidden');
    showModal('modal-seed-form');
}

document.getElementById('seed-form').addEventListener('submit', function (e) {
    e.preventDefault();
    const name      = document.getElementById('form-name').value.trim();
    const spacingX  = parseInt(document.getElementById('form-spacing-x').value, 10);
    const spacingY  = parseInt(document.getElementById('form-spacing-y').value, 10);
    const depth     = parseInt(document.getElementById('form-depth').value, 10);
    const toolId    = parseInt(document.getElementById('form-tool-id').value, 10);
    const toolParam = parseInt(document.getElementById('form-tool-param').value, 10);
    const errEl     = document.getElementById('seed-form-error');

    if (!name)                                          { errEl.textContent = 'Name is required.';                            errEl.classList.remove('hidden'); return; }
    if (isNaN(spacingX)||spacingX<1||isNaN(spacingY)||spacingY<1) { errEl.textContent = 'Spacing values must be positive.'; errEl.classList.remove('hidden'); return; }
    if (isNaN(depth)||depth<0)                          { errEl.textContent = 'Depth must be 0 or a positive integer.';      errEl.classList.remove('hidden'); return; }
    if (isNaN(toolParam)||toolParam<0)                  { errEl.textContent = 'Tool param must be 0 or a positive integer.'; errEl.classList.remove('hidden'); return; }

    const seed = { name, spacing_x_mm: spacingX, spacing_y_mm: spacingY, depth_mm: depth, tool_id: toolId, tool_param: toolParam };
    if (state.isAdding) {
        state.seeds.push(seed); state.selectedSeedIndex = state.seeds.length - 1;
    } else {
        state.seeds[state.editingSeedIndex] = seed; state.selectedSeedIndex = state.editingSeedIndex;
    }
    populateSeedSelect(); renderSeedSpecs(); saveSeeds(); hideModal();
});

document.getElementById('btn-seed-form-cancel').addEventListener('click', hideModal);

let pendingDeleteAction = null;

function showDeleteSeedModal() {
    const name = state.seeds[state.selectedSeedIndex].name;
    const idx  = state.selectedSeedIndex;
    document.getElementById('modal-delete-title').textContent = 'Delete Seed';
    document.getElementById('modal-delete-text').textContent =
        'Delete seed "' + name + '"? This cannot be undone.';
    pendingDeleteAction = () => {
        state.seeds.splice(idx, 1);
        state.selectedSeedIndex = Math.max(0, Math.min(idx, state.seeds.length - 1));
        populateSeedSelect(); renderSeedSpecs(); saveSeeds();
    };
    showModal('modal-delete-confirm');
}

document.getElementById('btn-delete-confirm').addEventListener('click', () => {
    if (pendingDeleteAction) { pendingDeleteAction(); pendingDeleteAction = null; }
    hideModal();
});
document.getElementById('btn-delete-cancel').addEventListener('click', () => {
    pendingDeleteAction = null; hideModal();
});
document.getElementById('btn-inprogress-ok').addEventListener('click', hideModal);
document.getElementById('btn-zones-wiped-ok').addEventListener('click', hideModal);

btnAddSeed.addEventListener('click', showAddSeedModal);
btnEditSeed.addEventListener('click', showEditSeedModal);
btnDeleteSeed.addEventListener('click', showDeleteSeedModal);

// ══════════════════════════════════════════════════════════
// INIT
// ══════════════════════════════════════════════════════════

mapRenderer = new MapRenderer(document.getElementById('field-canvas'));
mapRenderer.onClick    = handleMapClick;
mapRenderer.onHover    = handleMapHover;
mapRenderer.onHoverEnd = handleMapHoverEnd;

populateSeedSelect();
renderSeedSpecs();
renderMissionPhase();
renderZonePlanner();
renderZonesList();

loadRobotConfig();
loadSeeds();
loadFieldConfig();
loadMissionZones();
