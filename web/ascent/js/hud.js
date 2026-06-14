// Glass-cockpit HUD rendered on a 2-D canvas overlay.
//
// A best-in-class ascent HUD in the language of a real flight display:
// vertical AIRSPEED + ALTITUDE tapes (PFD style), an attitude indicator
// (artificial horizon) with a pitch ladder and flight-path marker, a live
// ORBIT panel (apoapsis / periapsis / time-to-apoapsis / downrange), the
// SpaceX-webcast vocabulary (mission clock, milestone log) and the derived
// ascent quantities a flight director watches (Mach, Max-Q, g-load,
// throttle, vertical speed). Drawn with corner brackets, status colours and
// a soft glow.
//
// Layout is computed each frame from the viewport so panels, tapes and the
// bottom cluster stay clear of one another at any aspect ratio: the two
// tapes own the far left/right edges and EVERYTHING else lives inboard of
// them. All values come from deriveFlight(); nothing is scripted.

import { deriveFlight, EARTH_RADIUS_M } from "./flight.js";

const C = {
  panel: "rgba(8, 14, 21, 0.72)",
  line: "rgba(127, 212, 255, 0.32)",
  lineSoft: "rgba(127, 212, 255, 0.13)",
  text: "#dbe7f1",
  dim: "#8298ac",
  accent: "#7fd4ff",
  good: "#69e6a0",
  warn: "#ffb86b",
  bad: "#ff6b6b",
  sky: "#1d4a66",
  skyHi: "#356b8c",
  ground: "#5a4528",
  groundLo: "#352915",
};
const MONO = "ui-monospace, 'SF Mono', 'JetBrains Mono', monospace";

export class Hud {
  constructor(root, layout) {
    this.layout = layout;
    this.root = root;
    this.events = [];
    this.flags = {};
    this.maxQ = { value: 0, timeS: 0, marked: false };
    this.outcome = null;
    this.prev = null;
    this.anim = {};

    const inStage1 = (engine) => engine.mounted_to === layout.bodies[0]?.id;
    this.stage1Indices = layout.engines.map((e, i) => (inStage1(e) ? i : -1)).filter((i) => i >= 0);
    this.upperIndices = layout.engines.map((e, i) => (!inStage1(e) ? i : -1)).filter((i) => i >= 0);
    // The booster also carries auxiliary engines (a boostback/landing engine)
    // alongside the main octaweb cluster. The octaweb is the largest group of
    // booster engines that share a thrust rating; the rest are auxiliary and
    // are shown as their own labelled indicators so an idle boostback engine
    // never looks like a dead octaweb engine.
    const byThrust = new Map();
    for (const i of this.stage1Indices) {
      const key = layout.engines[i].max_thrust_n;
      (byThrust.get(key) ?? byThrust.set(key, []).get(key)).push(i);
    }
    const groups = [...byThrust.values()].sort((a, b) => b.length - a.length);
    this.coreIndices = groups[0] ?? this.stage1Indices;
    const coreSet = new Set(this.coreIndices);
    this.auxIndices = [
      ...this.stage1Indices.filter((i) => !coreSet.has(i)).map((i) => ({ i, tag: "BB" })),
      ...this.upperIndices.map((i) => ({ i, tag: "VAC" })),
    ];

    root.innerHTML = "";
    this.canvas = document.createElement("canvas");
    this.canvas.id = "hud-canvas";
    root.appendChild(this.canvas);
    this.ctx = this.canvas.getContext("2d");
    this.resize();
    this._onResize = () => this.resize();
    window.addEventListener("resize", this._onResize);
  }

  resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = this.root.clientWidth || window.innerWidth;
    const h = this.root.clientHeight || window.innerHeight;
    this.w = w;
    this.h = h;
    this.canvas.width = Math.round(w * dpr);
    this.canvas.height = Math.round(h * dpr);
    this.canvas.style.width = `${w}px`;
    this.canvas.style.height = `${h}px`;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  // Responsive layout anchors, recomputed each frame.
  layoutFor(w, h) {
    const M = 14;
    const TOP = 56; // masthead strip reserved for the controls
    const tapeW = 52;
    const compact = w < 1080 || h < 560;
    // bottom cluster: ADI + metric tiles, centred
    const tileW = compact ? 74 : 86;
    const tileH = 44;
    const tileGap = 7;
    const nTiles = 5;
    const tilesW = nTiles * tileW + (nTiles - 1) * tileGap;
    const adiR = compact ? 38 : 46;
    const adiGap = 16;
    const showAdi = w > 940 && h >= 600;
    const mini = h < 440; // too short for tapes/panels — use the telemetry strip
    const clusterW = (showAdi ? adiR * 2 + adiGap : 0) + tilesW;
    const clusterX = Math.round(w / 2 - clusterW / 2);
    const bottomY = h - tileH - 18;
    return {
      M, TOP, tapeW, compact, showAdi, mini,
      tileW, tileH, tileGap, nTiles,
      tapeTop: TOP + 64,
      tapeBottom: h - 96,
      inboardL: M + tapeW + 16,
      inboardR: w - M - tapeW - 16,
      clockY: TOP + 26,
      phaseY: TOP + 44,
      panelTop: TOP + 56,
      adiCx: clusterX + adiR,
      adiCy: bottomY + tileH / 2,
      adiR,
      tilesX: clusterX + (showAdi ? adiR * 2 + adiGap : 0),
      tilesY: bottomY,
    };
  }

  mark(flag, label, timeS, kind = "") {
    if (this.flags[flag]) return;
    this.flags[flag] = true;
    this.events.unshift({ label, timeS, kind });
    if (this.events.length > 6) this.events.length = 6;
  }

  markStimulus(label, timeS) {
    this.events.unshift({ label: `⚡ ${label}`, timeS, kind: "inject" });
    if (this.events.length > 6) this.events.length = 6;
  }

  detectEvents(view, f) {
    const stage1Thrust = this.stage1Indices.reduce((s, i) => s + view.engines[i].fraction, 0);
    const upperThrust = this.upperIndices.reduce((s, i) => s + view.engines[i].fraction, 0);
    if (stage1Thrust > 0.05) this.mark("ignition", "IGNITION", view.timeS, "good");
    if (f.altitude > 2 && this.flags.ignition) this.mark("liftoff", "LIFTOFF", view.timeS, "good");
    if (f.q > this.maxQ.value) {
      this.maxQ.value = f.q;
      this.maxQ.timeS = view.timeS;
    } else if (!this.maxQ.marked && this.maxQ.value > 5000 && f.q < this.maxQ.value * 0.92 && this.flags.liftoff) {
      this.maxQ.marked = true;
      this.mark("maxq", `MAX-Q ${(this.maxQ.value / 1000).toFixed(0)} kPa`, this.maxQ.timeS);
    }
    if (this.flags.liftoff && stage1Thrust < 0.02 && !view.separatedCount) this.mark("meco", "MECO", view.timeS);
    if (view.separatedCount > 0) {
      this.mark("meco", "MECO", view.timeS);
      this.mark("sep", "STAGE SEP", view.timeS);
      this.sepTimeS = this.sepTimeS ?? view.timeS;
    }
    if (this.flags.sep && upperThrust > 0.05) this.mark("ses", "S2 IGNITION", view.timeS, "good");
    if (this.flags.ses && upperThrust < 0.02 && Number.isFinite(view.guidanceTgo) && view.guidanceTgo <= 0.5) {
      this.mark("seco", "SECO — ORBIT", view.timeS, "good");
    }
    if (this.flags.sep && view.timeS > (this.sepTimeS ?? 0) + 2.5) {
      const boosterBurning = view.engines.some(
        (e, i) => e.fraction > 0.05 && this.layout.engines[i].mounted_to === this.layout.bodies[0]?.id,
      );
      if (boosterBurning) this.mark("boostback", "BOOSTBACK BURN", view.timeS);
    }
  }

  update(view) {
    const f = deriveFlight(view, this.layout, this.prev);
    this.prev = view;
    this.detectEvents(view, f);
    this.view = view;
    this.L = this.layoutFor(this.w, this.h);

    const ease = (key, target, k = 0.2) => {
      if (!Number.isFinite(target)) return (this.anim[key] = target);
      const cur = this.anim[key];
      this.anim[key] = Number.isFinite(cur) ? cur + (target - cur) * k : target;
      return this.anim[key];
    };
    f.throttleA = ease("thr", f.throttle);
    f.gLoadA = ease("g", f.gLoad);
    f.machA = ease("mach", f.mach);

    const { ctx, w, h } = this;
    ctx.clearRect(0, 0, w, h);
    ctx.textBaseline = "alphabetic";

    if (this.L.mini) {
      this.drawMini(view, f);
    } else {
      this.drawFraming();
      this.drawClock(view);
      this.drawOrbit(f);
      this.drawNav(view);
      this.drawSpeedTape(f);
      this.drawAltTape(f);
      if (this.L.showAdi) this.drawADI(f);
      this.drawTiles(f);
      this.drawEvents();
      this.drawStages(view);
    }
    if (this.outcome) this.drawBanner();
  }

  // Compact two-line telemetry strip for short/wide viewports (e.g. with
  // DevTools docked) — clock on top, all the key numbers in a centred
  // lower-third, no tapes/ADI/boxes that would collide at this height.
  drawMini(view, f) {
    const cx = this.w / 2;
    this.txt(`T+ ${clock(view.timeS)}`, cx, this.L.TOP + 16, { size: 20, weight: 700, color: C.accent, align: "center", glow: 8 });
    this.txt(phaseLabel(this.layout, view.phase), cx, this.L.TOP + 30, { size: 9, color: C.dim, align: "center", spacing: 1 });
    const km = f.altitude / 1000;
    const nav = view.deadReckoning ? ["NAV", "DEAD-RECKON", C.warn] : ["NAV", "GNSS", C.good];
    const fdir = view.fdirTriggered ? ["FDIR", "TRIP", C.bad] : ["FDIR", "CLEAR", C.good];
    const row1 = [
      ["SPEED", f.groundSpeed.toFixed(0), "m/s", C.accent],
      ["ALT", km.toFixed(km < 10 ? 1 : 0), "km", C.accent],
      ["AP", fmtKm(f.apoAlt), "", C.text],
      ["PE", fmtPeri(f.periAlt), "", f.periAlt > 0 ? C.good : C.dim],
      ["DOWNRANGE", `${(f.downrange / 1000).toFixed(1)}`, "km", C.text],
      ["PITCH", `${f.pitchDeg.toFixed(0)}°`, "", C.text],
    ];
    const row2 = [
      ["THR", `${(f.throttleA * 100).toFixed(0)}%`, "", C.accent],
      ["MACH", Number.isFinite(f.machA) ? f.machA.toFixed(2) : "—", "", f.machA > 0.8 && f.machA < 1.2 ? C.warn : C.text],
      ["MAX-Q", `${(f.q / 1000).toFixed(1)}`, "kPa", f.q > 30000 ? C.warn : C.text],
      ["G", f.gLoadA.toFixed(2), "", f.gLoadA > 4 ? C.warn : C.text],
      ["V/S", `${f.verticalSpeed > 0 ? "+" : ""}${f.verticalSpeed.toFixed(0)}`, "m/s", C.text],
      nav,
      fdir,
    ];
    this.drawMiniRow(row1, this.h - 30);
    this.drawMiniRow(row2, this.h - 13);
  }

  drawMiniRow(segs, y) {
    const ctx = this.ctx;
    const gap = 20;
    const parts = [];
    let total = 0;
    for (const [label, val, unit, col] of segs) {
      ctx.font = `400 9px ${MONO}`;
      const lw = ctx.measureText(`${label} `).width;
      ctx.font = `700 12px ${MONO}`;
      const vw = ctx.measureText(val).width;
      ctx.font = `400 9px ${MONO}`;
      const uw = unit ? ctx.measureText(` ${unit}`).width : 0;
      const w = lw + vw + uw;
      parts.push({ label, val, unit, col, lw, vw, uw, w });
      total += w + gap;
    }
    total -= gap;
    let x = this.w / 2 - total / 2;
    for (const p of parts) {
      this.txt(p.label, x, y, { size: 9, color: C.dim });
      this.txt(p.val, x + p.lw, y, { size: 12, weight: 700, color: p.col });
      if (p.unit) this.txt(p.unit, x + p.lw + p.vw + 2, y, { size: 9, color: C.dim });
      x += p.w + gap;
    }
  }

  // ---- primitives ---------------------------------------------------------

  txt(s, x, y, { size = 12, color = C.text, align = "left", weight = 400, glow = 0, spacing = 0 } = {}) {
    const ctx = this.ctx;
    ctx.font = `${weight} ${size}px ${MONO}`;
    ctx.fillStyle = color;
    ctx.textAlign = align;
    if (glow) { ctx.shadowColor = color; ctx.shadowBlur = glow; }
    if (spacing && align === "left") {
      let cx = x;
      for (const ch of s) { ctx.fillText(ch, cx, y); cx += ctx.measureText(ch).width + spacing; }
    } else {
      ctx.fillText(s, x, y);
    }
    ctx.shadowBlur = 0;
  }

  bracket(x, y, w, h, color = C.line) {
    const ctx = this.ctx;
    const s = Math.min(12, w * 0.3, h * 0.3);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.25;
    ctx.beginPath();
    ctx.moveTo(x, y + s); ctx.lineTo(x, y); ctx.lineTo(x + s, y);
    ctx.moveTo(x + w - s, y); ctx.lineTo(x + w, y); ctx.lineTo(x + w, y + s);
    ctx.moveTo(x + w, y + h - s); ctx.lineTo(x + w, y + h); ctx.lineTo(x + w - s, y + h);
    ctx.moveTo(x + s, y + h); ctx.lineTo(x, y + h); ctx.lineTo(x, y + h - s);
    ctx.stroke();
  }

  panel(x, y, w, h) {
    this.ctx.fillStyle = C.panel;
    this.ctx.fillRect(x, y, w, h);
    this.bracket(x, y, w, h);
  }

  drawFraming() {
    const { ctx, w, h } = this;
    ctx.strokeStyle = C.lineSoft;
    ctx.lineWidth = 1.25;
    const m = 10;
    const s = 22;
    ctx.beginPath();
    for (const [cx, cy, dx, dy] of [
      [m, m + 48, 1, 1], [w - m, m + 48, -1, 1], [m, h - m, 1, -1], [w - m, h - m, -1, -1],
    ]) {
      ctx.moveTo(cx, cy + dy * s); ctx.lineTo(cx, cy); ctx.lineTo(cx + dx * s, cy);
    }
    ctx.stroke();
    const cx = w / 2;
    const cy = h / 2;
    ctx.strokeStyle = "rgba(127,212,255,0.14)";
    ctx.beginPath();
    ctx.moveTo(cx - 13, cy); ctx.lineTo(cx - 5, cy);
    ctx.moveTo(cx + 5, cy); ctx.lineTo(cx + 13, cy);
    ctx.moveTo(cx, cy - 13); ctx.lineTo(cx, cy - 5);
    ctx.moveTo(cx, cy + 5); ctx.lineTo(cx, cy + 13);
    ctx.stroke();
  }

  drawClock(view) {
    const cx = this.w / 2;
    this.txt(`T+ ${clock(view.timeS)}`, cx, this.L.clockY, { size: 32, weight: 700, color: C.accent, align: "center", glow: 10 });
    this.txt(phaseLabel(this.layout, view.phase), cx, this.L.phaseY, { size: 11, color: C.dim, align: "center", spacing: 1.5 });
  }

  drawOrbit(f) {
    const x = this.L.inboardL;
    const y = this.L.panelTop;
    const wd = 188;
    const rows = [
      ["APOAPSIS", fmtKm(f.apoAlt), C.accent],
      ["PERIAPSIS", fmtPeri(f.periAlt), f.periAlt > 0 ? C.good : C.dim],
      ["T-APOAPSIS", Number.isFinite(f.timeToApo) ? `T+${clock(f.timeS + f.timeToApo)}` : "—", C.text],
      ["DOWNRANGE", `${(f.downrange / 1000).toFixed(1)} km`, C.text],
    ];
    const hd = 24 + rows.length * 19 + 6;
    this.panel(x, y, wd, hd);
    this.txt("ORBITAL", x + 12, y + 16, { size: 10, color: C.dim, spacing: 2 });
    rows.forEach(([label, val, col], i) => {
      const ry = y + 38 + i * 19;
      this.txt(label, x + 12, ry, { size: 9.5, color: C.dim });
      this.txt(val, x + wd - 12, ry, { size: 12.5, weight: 600, color: col, align: "right" });
    });
  }

  drawNav(view) {
    const wd = 174;
    const x = this.L.inboardR - wd;
    const y = this.L.panelTop;
    const navError = view.estimateValid
      ? Math.hypot(
          view.estimatePosition[0] - view.position[0],
          view.estimatePosition[1] - view.position[1],
          view.estimatePosition[2] - view.position[2],
        )
      : NaN;
    const rows = [
      ["NAV", view.deadReckoning ? "DEAD-RECKON" : "GNSS-AIDED", view.deadReckoning ? C.warn : C.good],
      ["EKF ERR", Number.isFinite(navError) ? `${navError.toFixed(1)} m` : "—", C.text],
      ["FDIR", view.fdirTriggered ? "TRIPPED" : "CLEAR", view.fdirTriggered ? C.bad : C.good],
    ];
    const hd = 24 + rows.length * 19 + 6;
    this.panel(x, y, wd, hd);
    this.txt("GN&C STATUS", x + 12, y + 16, { size: 10, color: C.dim, spacing: 1 });
    rows.forEach(([label, val, col], i) => {
      const ry = y + 38 + i * 19;
      this.txt(label, x + 12, ry, { size: 9.5, color: C.dim });
      const dotX = x + wd - 14 - this.ctx.measureText(val).width - 10;
      this.ctx.beginPath();
      this.ctx.arc(dotX, ry - 3.5, 3, 0, Math.PI * 2);
      this.ctx.fillStyle = col;
      this.ctx.shadowColor = col;
      this.ctx.shadowBlur = 5;
      this.ctx.fill();
      this.ctx.shadowBlur = 0;
      this.txt(val, x + wd - 12, ry, { size: 11, weight: 600, color: col, align: "right" });
    });
  }

  tape(x, value, label, unit, fmt, step, pxPerUnit) {
    const { ctx } = this;
    const top = this.L.tapeTop;
    const bottom = Math.max(top + 120, this.L.tapeBottom);
    const cy = (top + bottom) / 2;
    const wd = this.L.tapeW;
    ctx.fillStyle = "rgba(6,11,17,0.5)";
    ctx.fillRect(x, top, wd, bottom - top);
    ctx.strokeStyle = C.lineSoft;
    ctx.lineWidth = 1;
    ctx.strokeRect(x, top, wd, bottom - top);
    ctx.save();
    ctx.beginPath();
    ctx.rect(x, top, wd, bottom - top);
    ctx.clip();
    const first = Math.ceil((value - (cy - top) / pxPerUnit) / step) * step;
    for (let val = first, guard = 0; guard < 400; val += step, guard += 1) {
      const ty = cy + (value - val) * pxPerUnit;
      if (ty < top - 20) break;
      if (ty > bottom + 20) continue;
      const major = Math.abs(val % (step * 5)) < 1e-6;
      ctx.strokeStyle = major ? C.line : C.lineSoft;
      ctx.beginPath();
      ctx.moveTo(x + wd - (major ? 13 : 7), ty);
      ctx.lineTo(x + wd, ty);
      ctx.stroke();
      if (major && val >= 0) this.txt(fmt(val), x + wd - 17, ty + 3.5, { size: 9, color: C.dim, align: "right" });
    }
    ctx.restore();
    const boxH = 26;
    ctx.fillStyle = "rgba(8,14,21,0.95)";
    ctx.fillRect(x - 5, cy - boxH / 2, wd + 5, boxH);
    ctx.strokeStyle = C.accent;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(x - 5, cy - boxH / 2, wd + 5, boxH);
    this.txt(fmt(value), x + wd - 7, cy + 4, { size: 15, weight: 700, color: C.accent, align: "right", glow: 7 });
    this.txt(`${label} ${unit}`, x + wd / 2, top - 7, { size: 9.5, color: C.dim, align: "center", spacing: 0.5 });
  }

  drawSpeedTape(f) {
    this.tape(this.L.M, f.groundSpeed, "SPEED", "M/S", (v) => v.toFixed(0), 100, 0.08);
  }

  drawAltTape(f) {
    const km = f.altitude / 1000;
    this.tape(this.w - this.L.M - this.L.tapeW, km, "ALT", "KM", (v) => v.toFixed(km < 10 ? 1 : 0), km < 20 ? 2 : 20, km < 20 ? 4 : 0.4);
  }

  drawADI(f) {
    const { ctx } = this;
    const R = this.L.adiR;
    const cx = this.L.adiCx;
    const cy = this.L.adiCy - 6;
    const pxPerDeg = R / 44;
    const roll = (f.rollDeg * Math.PI) / 180;
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.clip();
    ctx.translate(cx, cy);
    ctx.rotate(-roll);
    const horizon = f.pitchDeg * pxPerDeg;
    const sky = ctx.createLinearGradient(0, horizon - R * 2, 0, horizon);
    sky.addColorStop(0, C.sky);
    sky.addColorStop(1, C.skyHi);
    ctx.fillStyle = sky;
    ctx.fillRect(-R * 2, -R * 2, R * 4, R * 2 + horizon);
    const grd = ctx.createLinearGradient(0, horizon, 0, horizon + R * 2);
    grd.addColorStop(0, C.ground);
    grd.addColorStop(1, C.groundLo);
    ctx.fillStyle = grd;
    ctx.fillRect(-R * 2, horizon, R * 4, R * 2);
    ctx.strokeStyle = C.text;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-R, horizon); ctx.lineTo(R, horizon); ctx.stroke();
    ctx.strokeStyle = "rgba(219,231,241,0.7)";
    ctx.lineWidth = 1;
    ctx.font = `400 8px ${MONO}`;
    ctx.fillStyle = "rgba(219,231,241,0.8)";
    ctx.textAlign = "center";
    for (let p = -30; p <= 90; p += 10) {
      if (p === 0) continue;
      const yy = horizon - p * pxPerDeg;
      if (yy < -R || yy > R) continue;
      const half = p % 30 === 0 ? 16 : 9;
      ctx.beginPath();
      ctx.moveTo(-half, yy); ctx.lineTo(half, yy); ctx.stroke();
      if (p % 30 === 0) ctx.fillText(String(p), 0, yy - 3);
    }
    ctx.restore();
    // fixed boresight
    ctx.strokeStyle = C.accent;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx - 20, cy); ctx.lineTo(cx - 7, cy); ctx.lineTo(cx - 7, cy + 5);
    ctx.moveTo(cx + 20, cy); ctx.lineTo(cx + 7, cy); ctx.lineTo(cx + 7, cy + 5);
    ctx.stroke();
    ctx.fillStyle = C.accent;
    ctx.fillRect(cx - 1.5, cy - 1.5, 3, 3);
    // flight-path marker
    const fpmY = cy + Math.max(-R, Math.min(R, f.aoaDeg * pxPerDeg));
    ctx.strokeStyle = C.good;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(cx, fpmY, 3.5, 0, Math.PI * 2);
    ctx.moveTo(cx + 3.5, fpmY); ctx.lineTo(cx + 8, fpmY);
    ctx.moveTo(cx - 3.5, fpmY); ctx.lineTo(cx - 8, fpmY);
    ctx.moveTo(cx, fpmY - 3.5); ctx.lineTo(cx, fpmY - 7);
    ctx.stroke();
    // rim + roll pointer
    ctx.strokeStyle = C.line;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.stroke();
    const rr = roll - Math.PI / 2;
    ctx.fillStyle = C.accent;
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(rr) * (R - 6), cy + Math.sin(rr) * (R - 6));
    ctx.lineTo(cx + Math.cos(rr - 0.08) * R, cy + Math.sin(rr - 0.08) * R);
    ctx.lineTo(cx + Math.cos(rr + 0.08) * R, cy + Math.sin(rr + 0.08) * R);
    ctx.fill();
    // Attitude-error overlay: commanded (guidance reference) and onboard
    // estimate noses relative to the truth boresight. 5° -> the rim.
    const scale = R / 0.087;
    const place = (off) => [
      cx + Math.max(-R, Math.min(R, off[0] * scale)),
      cy - Math.max(-R, Math.min(R, off[1] * scale)),
    ];
    if (f.attErrRef) {
      const [mx, my] = place(f.attErrRef);
      ctx.strokeStyle = C.warn;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(mx, my, 4, 0, Math.PI * 2);
      ctx.stroke();
    }
    {
      const [mx, my] = place(f.attErrEst);
      ctx.strokeStyle = C.good;
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(mx - 4, my); ctx.lineTo(mx + 4, my);
      ctx.moveTo(mx, my - 4); ctx.lineTo(mx, my + 4);
      ctx.stroke();
    }
    this.txt(`PITCH ${f.pitchDeg.toFixed(0)}°`, cx, cy - R - 7, { size: 9, color: C.dim, align: "center" });
    this.txt("◯CMD ✛EST", cx, cy + R + 26, { size: 8, color: C.dim, align: "center" });
  }

  drawTiles(f) {
    const tiles = [
      ["THROTTLE", `${(f.throttleA * 100).toFixed(0)}`, C.accent, f.throttleA, "%"],
      ["MACH", Number.isFinite(f.machA) ? f.machA.toFixed(2) : "—", f.machA > 0.8 && f.machA < 1.2 ? C.warn : C.text, null, ""],
      ["MAX-Q", `${(f.q / 1000).toFixed(1)}`, f.q > 30000 ? C.warn : C.text, null, "kPa"],
      ["G-LOAD", `${f.gLoadA.toFixed(2)}`, f.gLoadA > 4 ? C.warn : C.text, null, "g"],
      ["V/SPEED", `${f.verticalSpeed > 0 ? "+" : ""}${f.verticalSpeed.toFixed(0)}`, C.text, null, "m/s"],
    ];
    const { tileW: tw, tileH: th, tileGap: gap } = this.L;
    let x = this.L.tilesX;
    const y = this.L.tilesY;
    for (const [label, val, col, bar, unit] of tiles) {
      this.panel(x, y, tw, th);
      this.txt(label, x + tw / 2, y + 14, { size: 8.5, color: C.dim, align: "center", spacing: 0.5 });
      this.ctx.font = `700 18px ${MONO}`;
      const vw = this.ctx.measureText(val).width;
      this.txt(val, x + tw / 2 - (unit ? this.ctx.measureText(unit).width / 2 + 1 : 0), y + 34, { size: 18, weight: 700, color: col, align: "center", glow: 4 });
      if (unit) this.txt(unit, x + tw / 2 + vw / 2 - (this.ctx.measureText(unit).width) / 2 + 2, y + 34, { size: 8.5, color: C.dim, align: "left" });
      if (bar !== null && bar !== undefined) {
        this.ctx.fillStyle = C.lineSoft;
        this.ctx.fillRect(x + 8, y + th - 5, tw - 16, 2.5);
        this.ctx.fillStyle = col;
        this.ctx.fillRect(x + 8, y + th - 5, (tw - 16) * Math.min(1, bar), 2.5);
      }
      x += tw + gap;
    }
  }

  drawEvents() {
    const x = this.L.inboardL;
    const y0 = this.h - 22;
    this.events.forEach((e, i) => {
      const y = y0 - i * 15;
      const col = i === 0 ? (e.kind === "inject" ? C.warn : e.kind === "good" ? C.good : C.accent)
        : e.kind === "inject" ? "rgba(255,184,107,0.7)" : C.dim;
      this.txt(`T+${clock(e.timeS)}`, x, y, { size: 9.5, color: col });
      this.txt(e.label, x + 56, y, { size: 9.5, color: col });
    });
    this.txt("EVENT LOG", x, y0 - this.events.length * 15 - 4, { size: 8.5, color: C.dim, spacing: 2 });
  }

  drawStages(view) {
    const x = this.L.inboardR;
    const railW = this.L.compact ? 116 : 150;
    let y = this.h - 96;
    this.txt("PROPELLANT", x, y - 6, { size: 8.5, color: C.dim, align: "right", spacing: 1.5 });
    this.layout.tanks.forEach((tank, index) => {
      let consumed = 0;
      this.layout.engines.forEach((engine, ei) => {
        if (engine.fuel_tank === tank.id) consumed += view.engines[ei].consumedKg;
      });
      const frac = Math.max(0, 1 - consumed / tank.initial_propellant_kg);
      const ry = y + index * 15;
      this.txt(tank.id.replace(/_/g, " ").toUpperCase(), x - railW - 8, ry + 6, { size: 8, color: C.dim, align: "right" });
      const rx = x - railW;
      this.ctx.strokeStyle = C.lineSoft;
      this.ctx.lineWidth = 1;
      this.ctx.strokeRect(rx, ry, railW, 7);
      this.ctx.fillStyle = frac < 0.15 ? C.warn : C.accent;
      this.ctx.fillRect(rx, ry, railW * frac, 7);
    });
    const cy = this.h - 26;
    // Main octaweb cluster (the booster's primary engines).
    const clusterX = x - railW + 24;
    this.drawCluster(view, this.coreIndices, clusterX, cy, 7);
    // Auxiliary engines (boostback, vacuum upper) as separate labelled dots,
    // so an idle recovery engine never reads as a dead octaweb engine.
    let ax = clusterX + 44;
    for (const { i, tag } of this.auxIndices) {
      this.drawEngineDot(view, i, ax, cy - 2, 4.5);
      this.txt(tag, ax, cy + 12, { size: 7, color: C.dim, align: "center" });
      ax += 28;
    }
  }

  drawEngineDot(view, ei, x, y, r) {
    const ctx = this.ctx;
    const e = view.engines[ei];
    let col = "#16242f";
    let glow = 0;
    if (e.stateIndex === 4) { col = C.bad; glow = 6; }
    else if (e.fraction > 0.02) { col = C.accent; glow = 6; }
    else if (e.stateIndex >= 2) col = "#2d4258";
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fillStyle = col;
    if (glow) { ctx.shadowColor = col; ctx.shadowBlur = glow; }
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.strokeStyle = C.lineSoft;
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  drawCluster(view, indices, cx, cy, r) {
    const n = indices.length;
    const pos = [];
    if (n === 1) pos.push([0, 0]);
    else {
      pos.push([0, 0]);
      const ring = n - 1;
      for (let i = 0; i < ring; i += 1) {
        const a = (i / ring) * Math.PI * 2 - Math.PI / 2;
        pos.push([Math.cos(a) * r * 1.7, Math.sin(a) * r * 1.7]);
      }
    }
    indices.forEach((ei, i) => {
      const [dx, dy] = pos[i] || [0, 0];
      this.drawEngineDot(view, ei, cx + dx, cy + dy, r * 0.4);
    });
  }

  drawBanner() {
    const { ctx, w } = this;
    const cy = this.h * 0.26;
    const col = this.outcome.orbit ? C.good : C.bad;
    ctx.font = `700 21px ${MONO}`;
    const tw = ctx.measureText(this.outcome.text).width + 52;
    const x = w / 2 - tw / 2;
    ctx.fillStyle = "rgba(6,11,17,0.9)";
    ctx.fillRect(x, cy - 25, tw, 50);
    this.bracket(x, cy - 25, tw, 50, col);
    this.txt(this.outcome.text, w / 2, cy + 7, { size: 21, weight: 700, color: col, align: "center", glow: 12 });
  }

  showOutcome(view) {
    const speed = Math.hypot(...view.velocity);
    const altitude = Math.hypot(...view.position) - EARTH_RADIUS_M;
    const orbit = altitude > 120_000 && speed > 7_300;
    this.outcome = {
      orbit,
      text: orbit
        ? `ORBIT · ${(altitude / 1000).toFixed(0)} km · ${speed.toFixed(0)} m/s`
        : altitude < 1000
          ? "VEHICLE LOST"
          : `RUN COMPLETE · ${(altitude / 1000).toFixed(0)} km · ${speed.toFixed(0)} m/s`,
    };
  }

  hideOutcome() {
    this.outcome = null;
  }
}

function clock(timeS) {
  const total = Math.max(0, Math.floor(timeS));
  return `${String(Math.floor(total / 60)).padStart(2, "0")}:${String(total % 60).padStart(2, "0")}`;
}

function fmtKm(m) {
  if (!Number.isFinite(m)) return "—";
  const km = m / 1000;
  if (km >= 1000) return `${(km / 1000).toFixed(2)}k`;
  return `${km.toFixed(km < 100 ? 1 : 0)} km`;
}

// Periapsis reads "—" while the trajectory is still deeply sub-orbital
// (a ballistic ascent has its periapsis far below the surface) instead of
// showing a meaningless large negative number.
function fmtPeri(m) {
  if (!Number.isFinite(m) || m < -100_000) return "—";
  return fmtKm(m);
}

function phaseLabel(layout, phaseId) {
  const phase = layout.phases.find((c) => c.id === phaseId);
  return (phase?.label ?? phaseId ?? "—").toUpperCase();
}
