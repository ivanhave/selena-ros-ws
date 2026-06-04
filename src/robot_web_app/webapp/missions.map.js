// ══════════════════════════════════════════════════════════
// MAP RENDERER
// ══════════════════════════════════════════════════════════

class MapRenderer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx    = canvas.getContext('2d');
        this.pixelsPerMeter = 80;
        this.panX = 2.5;
        this.panY = 0.0;
        this._dragging   = false;
        this._lastMouse  = { x: 0, y: 0 };
        this._animHandle = null;

        // Callbacks wired from outside
        this.onClick    = null;
        this.onHover    = null;
        this.onHoverEnd = null;

        this._bindEvents();
        const ro = new ResizeObserver(() => this._fitCanvas());
        ro.observe(canvas);   // observe canvas itself, not parent (parent includes toolbar)
        this._fitCanvas();
    }

    // ── Coordinate transforms ──────────────────────────────

    worldToScreen(wx, wy) {
        return {
            sx: this.canvas.width  / 2 + (wx - this.panX) * this.pixelsPerMeter,
            sy: this.canvas.height / 2 - (wy - this.panY) * this.pixelsPerMeter,
        };
    }

    screenToWorld(sx, sy) {
        return {
            wx: this.panX + (sx - this.canvas.width  / 2) / this.pixelsPerMeter,
            wy: this.panY - (sy - this.canvas.height / 2) / this.pixelsPerMeter,
        };
    }

    // ── Zoom ──────────────────────────────────────────────

    setZoom(ppm, anchorSx, anchorSy) {
        if (anchorSx !== undefined) {
            const w = this.screenToWorld(anchorSx, anchorSy);
            this.pixelsPerMeter = Math.max(10, Math.min(400, ppm));
            this.panX = w.wx - (anchorSx - this.canvas.width  / 2) / this.pixelsPerMeter;
            this.panY = w.wy + (anchorSy - this.canvas.height / 2) / this.pixelsPerMeter;
        } else {
            this.pixelsPerMeter = Math.max(10, Math.min(400, ppm));
        }
        syncZoomSlider(this.pixelsPerMeter);
        this.scheduleDraw();
    }

    resetView(fc) {
        if (!fc) {
            this.panX = 2.5; this.panY = 0; this.pixelsPerMeter = 80;
            syncZoomSlider(80); this.scheduleDraw(); return;
        }
        const f = fc.field;
        const nStrips = (f.strips || []).length || 1;
        const yaw = f.origin.yaw || 0;
        const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
        const midAlong = f.strip_length / 2;
        const midPerp  = (nStrips - 1) * f.strip_spacing / 2;
        this.panX = f.origin.x + midAlong * cosY - midPerp * sinY;
        this.panY = f.origin.y + midAlong * sinY + midPerp * cosY;
        const fieldFwd  = f.strip_length;
        const fieldPerp = (nStrips - 1) * f.strip_spacing + f.strip_width;
        const ppm = Math.min(
            this.canvas.width  * 0.65 / Math.max(fieldFwd,  0.1),
            this.canvas.height * 0.65 / Math.max(fieldPerp, 0.1)
        );
        this.pixelsPerMeter = Math.max(10, Math.min(400, ppm));
        syncZoomSlider(this.pixelsPerMeter);
        this.scheduleDraw();
    }

    scheduleDraw() {
        if (this._animHandle) return;
        this._animHandle = requestAnimationFrame(() => { this._animHandle = null; this._draw(); });
    }

    // ── Events ────────────────────────────────────────────

    _fitCanvas() {
        this.canvas.width  = this.canvas.clientWidth;
        this.canvas.height = this.canvas.clientHeight;
        this.scheduleDraw();
    }

    _bindEvents() {
        const c = this.canvas;
        let dragDist = 0;

        c.addEventListener('mousedown', e => {
            this._dragging = true;
            this._lastMouse = { x: e.clientX, y: e.clientY };
            dragDist = 0;
        });
        c.addEventListener('mousemove', e => {
            if (this._dragging) {
                const dx = e.clientX - this._lastMouse.x;
                const dy = e.clientY - this._lastMouse.y;
                dragDist += Math.abs(dx) + Math.abs(dy);
                this.panX -= dx / this.pixelsPerMeter;
                this.panY += dy / this.pixelsPerMeter;
                this._lastMouse = { x: e.clientX, y: e.clientY };
                this.scheduleDraw();
            }
            const rect = c.getBoundingClientRect();
            const w = this.screenToWorld(e.clientX - rect.left, e.clientY - rect.top);
            if (this.onHover) this.onHover(w.wx, w.wy);
        });
        c.addEventListener('mouseup', e => {
            const wasDrag = dragDist > 6;
            this._dragging = false;
            dragDist = 0;
            if (!wasDrag) {
                const rect = c.getBoundingClientRect();
                const w = this.screenToWorld(e.clientX - rect.left, e.clientY - rect.top);
                if (this.onClick) this.onClick(w.wx, w.wy);
            }
        });
        c.addEventListener('mouseleave', () => {
            this._dragging = false;
            if (this.onHoverEnd) this.onHoverEnd();
        });
        c.addEventListener('wheel', e => {
            e.preventDefault();
            const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
            const rect = c.getBoundingClientRect();
            this.setZoom(this.pixelsPerMeter * factor, e.clientX - rect.left, e.clientY - rect.top);
        }, { passive: false });

        // Touch — pan (1 finger) + pinch-zoom (2 fingers)
        let lastTouch = null, touchMoved = false;
        let lastPinchDist = null, isPinching = false;

        c.addEventListener('touchstart', e => {
            if (e.touches.length === 1) {
                lastTouch = { x: e.touches[0].clientX, y: e.touches[0].clientY };
                touchMoved = false;
                isPinching = false;
                lastPinchDist = null;
            } else if (e.touches.length === 2) {
                lastPinchDist = Math.hypot(
                    e.touches[1].clientX - e.touches[0].clientX,
                    e.touches[1].clientY - e.touches[0].clientY
                );
                isPinching = true;
                lastTouch = null;
            }
        }, { passive: true });

        c.addEventListener('touchmove', e => {
            if (e.touches.length === 2) {
                const dist = Math.hypot(
                    e.touches[1].clientX - e.touches[0].clientX,
                    e.touches[1].clientY - e.touches[0].clientY
                );
                if (lastPinchDist !== null && dist > 0) {
                    const rect = c.getBoundingClientRect();
                    const midX = (e.touches[0].clientX + e.touches[1].clientX) / 2 - rect.left;
                    const midY = (e.touches[0].clientY + e.touches[1].clientY) / 2 - rect.top;
                    this.setZoom(this.pixelsPerMeter * dist / lastPinchDist, midX, midY);
                }
                lastPinchDist = dist;
                return;
            }
            if (e.touches.length !== 1 || !lastTouch || isPinching) return;
            const dx = e.touches[0].clientX - lastTouch.x;
            const dy = e.touches[0].clientY - lastTouch.y;
            if (Math.abs(dx) + Math.abs(dy) > 8) touchMoved = true;
            this.panX -= dx / this.pixelsPerMeter;
            this.panY += dy / this.pixelsPerMeter;
            lastTouch = { x: e.touches[0].clientX, y: e.touches[0].clientY };
            this.scheduleDraw();
        }, { passive: true });

        c.addEventListener('touchend', e => {
            if (e.touches.length === 1 && isPinching) {
                // One finger lifted — transition back to pan without a jump
                lastTouch = { x: e.touches[0].clientX, y: e.touches[0].clientY };
                lastPinchDist = null;
                isPinching = false;
                return;
            }
            if (e.touches.length === 0) {
                if (!touchMoved && !isPinching && e.changedTouches.length === 1) {
                    const t = e.changedTouches[0];
                    const rect = c.getBoundingClientRect();
                    const w = this.screenToWorld(t.clientX - rect.left, t.clientY - rect.top);
                    if (this.onClick) this.onClick(w.wx, w.wy);
                }
                lastTouch = null; touchMoved = false;
                lastPinchDist = null; isPinching = false;
            }
        });
    }

    // ── Strip geometry helpers ─────────────────────────────

    _stripCenterStart(f, stripId) {
        const yaw = f.origin.yaw || 0;
        return {
            x: f.origin.x - stripId * f.strip_spacing * Math.sin(yaw),
            y: f.origin.y + stripId * f.strip_spacing * Math.cos(yaw),
        };
    }

    _stripCorners(f, stripId) {
        const yaw = f.origin.yaw || 0;
        const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
        const hw = f.strip_width / 2, L = f.strip_length;
        const c = this._stripCenterStart(f, stripId);
        return [
            { x: c.x + hw * sinY,           y: c.y - hw * cosY },
            { x: c.x - hw * sinY,           y: c.y + hw * cosY },
            { x: c.x + L*cosY - hw*sinY,    y: c.y + L*sinY + hw*cosY },
            { x: c.x + L*cosY + hw*sinY,    y: c.y + L*sinY - hw*cosY },
        ];
    }

    // ── Draw ──────────────────────────────────────────────

    _draw() {
        const { canvas, ctx } = this;
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = '#07101a';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        this._drawGrid();
        this._drawStrips(fieldConfig);
        this._drawZones(fieldConfig, missionZones);
        this._drawPaths();
        this._drawZonePreview();
        this._drawRobot(robotPose);
        this._drawOriginAxes();
    }

    _drawGrid() {
        const { canvas, ctx } = this;
        const tl = this.screenToWorld(0, 0);
        const br = this.screenToWorld(canvas.width, canvas.height);
        const x0 = tl.wx, x1 = br.wx, y0 = br.wy, y1 = tl.wy;
        const MINOR = 0.5, MAJOR = 1.0;
        ctx.lineWidth = 1;
        for (let x = Math.floor(x0 / MINOR) * MINOR; x <= x1 + MINOR; x += MINOR) {
            ctx.strokeStyle = Math.abs(x % MAJOR) < 0.005 ? '#18283a' : '#101820';
            const a = this.worldToScreen(x, y0), b = this.worldToScreen(x, y1);
            ctx.beginPath(); ctx.moveTo(a.sx, a.sy); ctx.lineTo(b.sx, b.sy); ctx.stroke();
        }
        for (let y = Math.floor(y0 / MINOR) * MINOR; y <= y1 + MINOR; y += MINOR) {
            ctx.strokeStyle = Math.abs(y % MAJOR) < 0.005 ? '#18283a' : '#101820';
            const a = this.worldToScreen(x0, y), b = this.worldToScreen(x1, y);
            ctx.beginPath(); ctx.moveTo(a.sx, a.sy); ctx.lineTo(b.sx, b.sy); ctx.stroke();
        }
    }

    _drawStrips(fc) {
        if (!fc) return;
        const { ctx } = this;
        const f = fc.field;
        const yaw = f.origin.yaw || 0;
        const cosY = Math.cos(yaw), sinY = Math.sin(yaw);

        (f.strips || []).forEach(strip => {
            const isActive = (zonePreview.phase !== 'IDLE' && strip.id === zonePreview.stripId);
            const corners = this._stripCorners(f, strip.id);
            const pts = corners.map(c => this.worldToScreen(c.x, c.y));
            ctx.beginPath();
            ctx.moveTo(pts[0].sx, pts[0].sy);
            pts.slice(1).forEach(p => ctx.lineTo(p.sx, p.sy));
            ctx.closePath();

            if (strip.enabled) {
                ctx.fillStyle   = isActive ? 'rgba(15, 40, 80, 0.70)' : 'rgba(13, 27, 42, 0.85)';
                ctx.strokeStyle = isActive ? '#7ab8d4' : '#0f3460';
                ctx.lineWidth   = isActive ? 2 : 1.5;
            } else {
                ctx.fillStyle   = 'rgba(10, 12, 20, 0.70)';
                ctx.strokeStyle = '#1a2030';
                ctx.lineWidth   = 1;
            }
            ctx.fill(); ctx.stroke();

            {
                const c = this._stripCenterStart(f, strip.id);
                const mid = this.worldToScreen(c.x + f.strip_length * cosY / 2, c.y + f.strip_length * sinY / 2);
                ctx.fillStyle    = strip.enabled ? (isActive ? '#7ab8d4' : '#2a5070') : '#2a2a3a';
                const fontSize   = Math.max(9, Math.min(13, this.pixelsPerMeter * 0.15));
                ctx.font         = `${fontSize}px monospace`;
                ctx.textAlign    = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText('Strip ' + (strip.id + 1), mid.sx, mid.sy);
            }
        });
    }

    _drawPaths() {
        const { ctx } = this;

        if (plannedPath.length >= 2) {
            ctx.save();
            ctx.setLineDash([5, 3]);
            ctx.strokeStyle = 'rgba(100, 160, 255, 0.65)';
            ctx.lineWidth = 2;
            ctx.beginPath();
            const p0 = this.worldToScreen(plannedPath[0].x, plannedPath[0].y);
            ctx.moveTo(p0.sx, p0.sy);
            for (let i = 1; i < plannedPath.length; i++) {
                const p = this.worldToScreen(plannedPath[i].x, plannedPath[i].y);
                ctx.lineTo(p.sx, p.sy);
            }
            ctx.stroke();
            ctx.setLineDash([]);
            ctx.fillStyle = 'rgba(100, 160, 255, 0.85)';
            plannedPath.forEach(pt => {
                const p = this.worldToScreen(pt.x, pt.y);
                ctx.beginPath();
                ctx.arc(p.sx, p.sy, 4, 0, 2 * Math.PI);
                ctx.fill();
            });
            ctx.restore();
        }

        if (traveledPath.length >= 2) {
            ctx.save();
            ctx.setLineDash([]);
            ctx.strokeStyle = '#e8a020';
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            const t0 = this.worldToScreen(traveledPath[0].x, traveledPath[0].y);
            ctx.moveTo(t0.sx, t0.sy);
            for (let i = 1; i < traveledPath.length; i++) {
                const t = this.worldToScreen(traveledPath[i].x, traveledPath[i].y);
                ctx.lineTo(t.sx, t.sy);
            }
            ctx.stroke();
            ctx.restore();
        }
    }

    _drawZones(fc, zones) {
        if (!fc || !zones) return;
        const { ctx } = this;
        const f = fc.field;
        const yaw = f.origin.yaw || 0;
        const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
        const hw = f.strip_width / 2;
        const COLORS = {
            planned:     { fill: 'rgba(40, 100, 200, 0.45)', stroke: '#4a88f0' },
            in_progress: { fill: 'rgba(210, 120, 20, 0.55)',  stroke: '#e07820' },
            completed:   { fill: 'rgba(30, 170, 80, 0.45)',   stroke: '#20b060' },
            failed:      { fill: 'rgba(190, 30, 50, 0.45)',   stroke: '#c02040' },
        };

        (zones.zones || []).forEach((zone, zoneIdx) => {
            const strip = (f.strips || []).find(s => s.id === zone.strip_id);
            if (!strip && zone.status !== 'completed') return;
            const c = this._stripCenterStart(f, zone.strip_id);
            const sa = zone.start_along, ea = zone.end_along;
            const corners = [
                { x: c.x + sa*cosY + hw*sinY, y: c.y + sa*sinY - hw*cosY },
                { x: c.x + sa*cosY - hw*sinY, y: c.y + sa*sinY + hw*cosY },
                { x: c.x + ea*cosY - hw*sinY, y: c.y + ea*sinY + hw*cosY },
                { x: c.x + ea*cosY + hw*sinY, y: c.y + ea*sinY - hw*cosY },
            ];
            const pts = corners.map(pt => this.worldToScreen(pt.x, pt.y));
            const color = COLORS[zone.status] || COLORS.planned;
            ctx.beginPath();
            ctx.moveTo(pts[0].sx, pts[0].sy);
            pts.slice(1).forEach(p => ctx.lineTo(p.sx, p.sy));
            ctx.closePath();
            ctx.fillStyle   = color.fill;
            ctx.strokeStyle = color.stroke;
            ctx.lineWidth   = 1.5;
            ctx.fill(); ctx.stroke();

            if (this.pixelsPerMeter > 20) {
                const zoneNum = zoneIdx + 1;
                const mid = this.worldToScreen(c.x + (sa+ea)/2*cosY, c.y + (sa+ea)/2*sinY);
                const r = Math.max(7, Math.min(12, this.pixelsPerMeter * 0.12));

                // Circle background
                ctx.beginPath();
                ctx.arc(mid.sx, mid.sy, r, 0, 2 * Math.PI);
                ctx.fillStyle = 'rgba(8, 16, 32, 0.82)';
                ctx.fill();
                ctx.strokeStyle = color.stroke;
                ctx.lineWidth = 1.5;
                ctx.stroke();

                // Zone number
                ctx.fillStyle = '#eee';
                ctx.font = `bold ${Math.max(7, Math.round(r * 0.9))}px monospace`;
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText(String(zoneNum), mid.sx, mid.sy);

                // Seed name below circle
                ctx.fillStyle = '#8ab';
                ctx.font = `${Math.max(7, Math.round(r * 0.78))}px monospace`;
                ctx.textBaseline = 'top';
                ctx.fillText(zone.target || '?', mid.sx, mid.sy + r + 2);
            }
        });
    }

    _drawZonePreview() {
        if (!fieldConfig || zonePreview.phase === 'IDLE' || zonePreview.stripId === null) return;
        const f = fieldConfig.field;
        const yaw = f.origin.yaw || 0;
        const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
        const hw = f.strip_width / 2;
        const { ctx } = this;

        const strip = (f.strips || []).find(s => s.id === zonePreview.stripId);
        if (!strip) return;
        const c = this._stripCenterStart(f, strip.id);

        // Start marker
        if (zonePreview.startAlong !== null) {
            this._drawAlongMarker(c, cosY, sinY, hw, zonePreview.startAlong, '#7ab8d4', 2.5);
        }

        // End or hover preview
        const endAlong = zonePreview.phase === 'ZONE_READY'
            ? zonePreview.endAlong
            : zonePreview.hoverAlong;

        if (endAlong !== null && zonePreview.startAlong !== null) {
            const sa = Math.min(zonePreview.startAlong, endAlong);
            const ea = Math.max(zonePreview.startAlong, endAlong);
            if (ea - sa >= SNAP_M) {
                const isOverlap   = zonePreview.overlap;
                const fillColor   = isOverlap ? 'rgba(200, 40, 60, 0.35)' : 'rgba(100, 180, 240, 0.30)';
                const strokeColor = isOverlap ? '#e04050' : '#7ab8d4';
                const corners = [
                    { x: c.x + sa*cosY + hw*sinY, y: c.y + sa*sinY - hw*cosY },
                    { x: c.x + sa*cosY - hw*sinY, y: c.y + sa*sinY + hw*cosY },
                    { x: c.x + ea*cosY - hw*sinY, y: c.y + ea*sinY + hw*cosY },
                    { x: c.x + ea*cosY + hw*sinY, y: c.y + ea*sinY - hw*cosY },
                ];
                const pts = corners.map(pt => this.worldToScreen(pt.x, pt.y));
                ctx.beginPath();
                ctx.moveTo(pts[0].sx, pts[0].sy);
                pts.slice(1).forEach(p => ctx.lineTo(p.sx, p.sy));
                ctx.closePath();
                ctx.fillStyle = fillColor;
                ctx.fill();
                ctx.setLineDash([5, 3]);
                ctx.strokeStyle = strokeColor;
                ctx.lineWidth   = 2;
                ctx.stroke();
                ctx.setLineDash([]);

                // Length label
                if (this.pixelsPerMeter > 20) {
                    const mid = this.worldToScreen(c.x + (sa+ea)/2*cosY, c.y + (sa+ea)/2*sinY);
                    ctx.fillStyle = isOverlap ? '#e04050' : '#7ab8d4';
                    ctx.font = 'bold 11px monospace';
                    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
                    ctx.fillText((ea - sa).toFixed(1) + ' m', mid.sx, mid.sy);
                }
            }
            const endColor = zonePreview.phase === 'ZONE_READY'
                ? (zonePreview.overlap ? '#e04050' : '#7ab8d4')
                : '#5090b0';
            this._drawAlongMarker(c, cosY, sinY, hw, endAlong, endColor, 2);
        }
    }

    _drawAlongMarker(c, cosY, sinY, hw, along, color, lineWidth) {
        const p1 = this.worldToScreen(c.x + along*cosY + hw*sinY, c.y + along*sinY - hw*cosY);
        const p2 = this.worldToScreen(c.x + along*cosY - hw*sinY, c.y + along*sinY + hw*cosY);
        const { ctx } = this;
        ctx.beginPath(); ctx.moveTo(p1.sx, p1.sy); ctx.lineTo(p2.sx, p2.sy);
        ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.stroke();
    }

    _drawRobot(pose) {
        const isGhost = !pose;
        const drawPose = pose ?? { x: 0, y: 0, yaw: 0 };
        const { sx, sy } = this.worldToScreen(drawPose.x, drawPose.y);
        const ppm = this.pixelsPerMeter;
        const { ctx } = this;
        ctx.save();
        ctx.globalAlpha = isGhost ? 0.22 : 1.0;
        ctx.translate(sx, sy);
        ctx.rotate(-drawPose.yaw);

        // Fallback: simple arrow when wheel pixel radius < 3
        const rc = robotConfig;
        if (ppm * (rc?.wheel_radius ?? 0.0825) < 3) {
            const sz = Math.max(6, ppm * 0.4);
            ctx.beginPath();
            ctx.moveTo(sz, 0); ctx.lineTo(-sz * 0.6, -sz * 0.5); ctx.lineTo(-sz * 0.6, sz * 0.5);
            ctx.closePath();
            ctx.fillStyle = '#e94560'; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
            ctx.fill(); ctx.stroke();
            ctx.restore();
            return;
        }

        // Convention: robot +X (forward) → canvas +X,  robot +Y (left) → canvas −Y
        const p = ppm;

        // ── Dimensions from /api/robot_config (mobile_base.xacro) ───────
        const FRONT_X   =  (rc?.front_wheel_offset_x ?? 0.275);
        const BACK_X    = -(rc?.back_wheel_offset_x  ?? 0.305);
        const LEG_Y     =  (rc?.leg_offset_y         ?? 0.5795);
        const WHL_Y     =  (rc?.leg_offset_y         ?? 0.5795) + (rc?.wheel_offset_y ?? 0.058);
        const WHL_R     =  (rc?.wheel_radius         ?? 0.0825);
        const xTravel   =  (rc?.x_axis_travel        ?? 0.845);
        const yTravel   =  (rc?.y_axis_travel        ?? 0.290);
        // Aesthetic constants not in URDF
        const BAR_W     =  0.030;   // bar thickness (3 cm)
        const WHL_TW    =  0.095;   // wheel tread width top-down
        // gantry_origin at (y_axis_travel/2, −x_axis_travel/2, 0) → centred at (0,0)
        const GANTRY_HX = yTravel / 2 + 0.050;
        const GANTRY_HY = xTravel / 2 + 0.050;

        // ── Rounded-rectangle path helper ───────────────────────────────
        const rRect = (x, y, w, h, r) => {
            r = Math.min(r, Math.abs(w) / 2, Math.abs(h) / 2);
            ctx.beginPath();
            ctx.moveTo(x + r, y);
            ctx.lineTo(x + w - r, y);      ctx.arcTo(x + w, y,     x + w, y + r,     r);
            ctx.lineTo(x + w, y + h - r);  ctx.arcTo(x + w, y + h, x + w - r, y + h, r);
            ctx.lineTo(x + r, y + h);      ctx.arcTo(x,     y + h, x,     y + h - r, r);
            ctx.lineTo(x,     y + r);      ctx.arcTo(x,     y,     x + r, y,          r);
            ctx.closePath();
        };

        // ── 1. Leg bars (left_leg_link / right_leg_link) ─────────────────
        // Solid bars at ±leg_offset_y, spanning BACK_X → FRONT_X in robot X.
        // Same thickness as gantry bars: BAR_W → half = BAR_W / 2.
        const LEG_HW = BAR_W / 2;
        ctx.fillStyle = '#7ab8d4';
        ctx.fillRect(BACK_X * p, -(LEG_Y + LEG_HW) * p, (FRONT_X - BACK_X) * p, LEG_HW * 2 * p);
        ctx.fillRect(BACK_X * p,  (LEG_Y - LEG_HW) * p, (FRONT_X - BACK_X) * p, LEG_HW * 2 * p);

        // ── 2. Gantry travel rectangle (the only rectangle) ──────────────
        // Hollow, centred at base_link origin (= centre of travel range).
        // Width  = x_axis_travel + 5 cm each side (robot Y direction).
        // Height = y_axis_travel + 5 cm each side (robot X direction).
        ctx.strokeStyle = '#7ab8d4';
        ctx.lineWidth   = BAR_W * p;
        ctx.lineJoin    = 'miter';
        ctx.strokeRect(-GANTRY_HX * p, -GANTRY_HY * p, GANTRY_HX * 2 * p, GANTRY_HY * 2 * p);

        // ── 3. Wheels ────────────────────────────────────────────────────
        const hw  = WHL_R * p;
        const hty = WHL_TW / 2 * p;
        const wr  = Math.min(hw, hty);
        const drawWheel = (wx, wy) => {
            rRect(wx * p - hw, -wy * p - hty, hw * 2, hty * 2, wr);
            ctx.fillStyle   = '#1a3060';
            ctx.fill();
            ctx.strokeStyle = '#4a72c0';
            ctx.lineWidth   = Math.max(1, p * 0.010);
            ctx.stroke();
        };
        drawWheel( FRONT_X,  WHL_Y);
        drawWheel( BACK_X,   WHL_Y);
        drawWheel( FRONT_X, -WHL_Y);
        drawWheel( BACK_X,  -WHL_Y);

        // ── 4. Coordinate axes at base_link origin ───────────────────────
        const axPx = 0.20 * p;
        const aw   = Math.max(1.5, p * 0.009);
        const ah   = Math.max(3,   axPx * 0.26);

        ctx.lineWidth = aw;
        ctx.strokeStyle = '#ff4444';
        ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(axPx, 0); ctx.stroke();
        ctx.fillStyle = '#ff4444';
        ctx.beginPath(); ctx.moveTo(axPx, 0); ctx.lineTo(axPx - ah, -ah * 0.42); ctx.lineTo(axPx - ah, ah * 0.42); ctx.closePath(); ctx.fill();

        ctx.strokeStyle = '#44cc44';
        ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(0, -axPx); ctx.stroke();
        ctx.fillStyle = '#44cc44';
        ctx.beginPath(); ctx.moveTo(0, -axPx); ctx.lineTo(-ah * 0.42, -axPx + ah); ctx.lineTo(ah * 0.42, -axPx + ah); ctx.closePath(); ctx.fill();

        ctx.fillStyle = '#ffffff';
        ctx.beginPath(); ctx.arc(0, 0, Math.max(2, p * 0.022), 0, Math.PI * 2); ctx.fill();

        ctx.restore();
    }

    _drawOriginAxes() {
        const { sx, sy } = this.worldToScreen(0, 0);
        const W = this.canvas.width, H = this.canvas.height;
        if (sx < -30 || sx > W+30 || sy < -30 || sy > H+30) return;
        const len = 16, { ctx } = this;
        ctx.lineWidth = 1.5;
        ctx.strokeStyle = '#c04040';
        ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(sx+len, sy); ctx.stroke();
        ctx.strokeStyle = '#40c040';
        ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(sx, sy-len); ctx.stroke();
    }
}

// ══════════════════════════════════════════════════════════
// ZOOM SLIDER
// ══════════════════════════════════════════════════════════

const zoomSlider = document.getElementById('zoom-slider');

function ppmToSlider(ppm) { return Math.round(100 * Math.log(ppm / 10) / Math.log(40)); }
function sliderToPpm(v)   { return 10 * Math.pow(40, v / 100); }
function syncZoomSlider(ppm) { zoomSlider.value = ppmToSlider(Math.max(10, Math.min(400, ppm))); }

zoomSlider.addEventListener('input', () => {
    if (mapRenderer) mapRenderer.setZoom(sliderToPpm(parseInt(zoomSlider.value, 10)));
});
document.getElementById('btn-zoom-in').addEventListener('click',  () => { if (mapRenderer) mapRenderer.setZoom(mapRenderer.pixelsPerMeter * 1.3); });
document.getElementById('btn-zoom-out').addEventListener('click', () => { if (mapRenderer) mapRenderer.setZoom(mapRenderer.pixelsPerMeter / 1.3); });
document.getElementById('btn-zoom-fit').addEventListener('click', () => { if (mapRenderer) mapRenderer.resetView(fieldConfig); });
