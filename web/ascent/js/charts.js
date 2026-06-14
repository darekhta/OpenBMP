// Engineering overlay for the flight computer: canvas instruments that
// read the live telemetry the onboard GN&C publishes. Beyond plain strip
// charts it shows the EKF's own ±1σ covariance band around the true error
// (the "this is a calibrated estimator" signal), an innovation-gating
// scatter (the χ² gate accepting/rejecting sensor updates in real time),
// and a sensor-health / FDIR grid. Truth is read only by the renderer;
// the flight computer never sees it.

const CAPACITY = 900;
const ACCENT = "#7fd4ff";
const GOOD = "#69e6a0";
const WARN = "#ffb86b";
const BAD = "#ff6b6b";
const DIM = "#7f97ab";

class StripChart {
  constructor(canvas, label, unit, options = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.label = label;
    this.unit = unit;
    this.color = options.color ?? ACCENT;
    this.logFloor = options.logFloor ?? null;
    this.hasBand = options.band ?? false; // shaded ±1σ envelope
    this.times = new Float64Array(CAPACITY);
    this.values = new Float64Array(CAPACITY);
    this.bands = new Float64Array(CAPACITY);
    this.length = 0;
    this.head = 0;
  }

  push(timeS, value, band = 0) {
    if (!Number.isFinite(value)) return;
    this.times[this.head] = timeS;
    this.values[this.head] = value;
    this.bands[this.head] = Number.isFinite(band) ? band : 0;
    this.head = (this.head + 1) % CAPACITY;
    this.length = Math.min(this.length + 1, CAPACITY);
  }

  reset() {
    this.length = 0;
    this.head = 0;
  }

  draw() {
    const { ctx, canvas } = this;
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "rgba(8, 12, 18, 0.85)";
    ctx.fillRect(0, 0, w, h);
    if (this.length < 2) return;

    let min = Infinity;
    let max = -Infinity;
    let latest = 0;
    let latestBand = 0;
    for (let i = 0; i < this.length; i += 1) {
      const slot = (this.head - 1 - i + 2 * CAPACITY) % CAPACITY;
      const v = this.values[slot];
      min = Math.min(min, v);
      max = Math.max(max, v);
      if (this.hasBand) max = Math.max(max, this.bands[slot]);
      if (i === 0) { latest = v; latestBand = this.bands[slot]; }
    }
    if (this.logFloor !== null) min = Math.max(min, this.logFloor);
    if (max - min < 1e-12) max = min + 1e-12;

    const t1 = this.times[(this.head - 1 + CAPACITY) % CAPACITY];
    const t0 = this.times[(this.head - this.length + CAPACITY) % CAPACITY];
    const span = Math.max(t1 - t0, 1e-6);
    const mapX = (t) => 4 + ((t - t0) / span) * (w - 8);
    const mapY = (value) => {
      const lf = this.logFloor !== null;
      const v = lf ? Math.log10(Math.max(value, this.logFloor)) : value;
      const lo = lf ? Math.log10(min) : min;
      const hi = lf ? Math.log10(max) : max;
      return h - 14 - ((v - lo) / Math.max(hi - lo, 1e-12)) * (h - 26);
    };

    // ±1σ band (filled envelope from baseline to the filter's reported σ).
    if (this.hasBand) {
      ctx.fillStyle = "rgba(127,212,255,0.16)";
      ctx.beginPath();
      const base = mapY(min);
      ctx.moveTo(mapX(t0), base);
      for (let i = this.length - 1; i >= 0; i -= 1) {
        const slot = (this.head - 1 - i + 2 * CAPACITY) % CAPACITY;
        ctx.lineTo(mapX(this.times[slot]), mapY(this.bands[slot]));
      }
      ctx.lineTo(mapX(t1), base);
      ctx.closePath();
      ctx.fill();
    }

    ctx.strokeStyle = this.color;
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    for (let i = this.length - 1; i >= 0; i -= 1) {
      const slot = (this.head - 1 - i + 2 * CAPACITY) % CAPACITY;
      const x = mapX(this.times[slot]);
      const y = mapY(this.values[slot]);
      if (i === this.length - 1) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    ctx.fillStyle = "#9fb4c8";
    ctx.font = "10px ui-monospace, monospace";
    ctx.fillText(this.label, 6, 11);
    ctx.fillStyle = this.color;
    const txt = this.hasBand
      ? `${formatValue(latest)} ±${formatValue(latestBand)} ${this.unit}`
      : `${formatValue(latest)} ${this.unit}`;
    ctx.fillText(txt, w - ctx.measureText(txt).width - 6, 11);
  }
}

// Innovation-gating scatter: each sensor update plotted as a point at its
// χ² (normalized innovation squared); green if the filter accepted it,
// red if the χ² gate rejected it. A dashed line marks the gate.
class GatingChart {
  constructor(canvas, label, gate) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.label = label;
    this.gate = gate;
    this.times = new Float64Array(CAPACITY);
    this.chi2 = new Float64Array(CAPACITY);
    this.accepted = new Uint8Array(CAPACITY);
    this.length = 0;
    this.head = 0;
  }

  push(timeS, chi2, accepted) {
    if (!Number.isFinite(chi2)) return;
    this.times[this.head] = timeS;
    this.chi2[this.head] = chi2;
    this.accepted[this.head] = accepted ? 1 : 0;
    this.head = (this.head + 1) % CAPACITY;
    this.length = Math.min(this.length + 1, CAPACITY);
  }

  reset() {
    this.length = 0;
    this.head = 0;
  }

  draw() {
    const { ctx, canvas } = this;
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "rgba(8, 12, 18, 0.85)";
    ctx.fillRect(0, 0, w, h);
    if (this.length < 1) {
      ctx.fillStyle = "#9fb4c8";
      ctx.font = "10px ui-monospace, monospace";
      ctx.fillText(this.label, 6, 11);
      return;
    }
    let max = this.gate * 1.6;
    let rejects = 0;
    for (let i = 0; i < this.length; i += 1) {
      const slot = (this.head - 1 - i + 2 * CAPACITY) % CAPACITY;
      max = Math.max(max, this.chi2[slot]);
      if (!this.accepted[slot]) rejects += 1;
    }
    const t1 = this.times[(this.head - 1 + CAPACITY) % CAPACITY];
    const t0 = this.times[(this.head - this.length + CAPACITY) % CAPACITY];
    const span = Math.max(t1 - t0, 1e-6);
    const mapX = (t) => 6 + ((t - t0) / span) * (w - 12);
    const mapY = (v) => h - 16 - (Math.min(v, max) / max) * (h - 26);

    // gate line
    ctx.strokeStyle = "rgba(255,107,107,0.6)";
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(6, mapY(this.gate));
    ctx.lineTo(w - 6, mapY(this.gate));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = "rgba(255,107,107,0.7)";
    ctx.font = "8px ui-monospace, monospace";
    ctx.fillText(`χ² gate ${this.gate}`, w - 70, mapY(this.gate) - 3);

    for (let i = 0; i < this.length; i += 1) {
      const slot = (this.head - 1 - i + 2 * CAPACITY) % CAPACITY;
      ctx.fillStyle = this.accepted[slot] ? GOOD : BAD;
      ctx.globalAlpha = this.accepted[slot] ? 0.7 : 1;
      const x = mapX(this.times[slot]);
      const y = mapY(this.chi2[slot]);
      ctx.beginPath();
      ctx.arc(x, y, this.accepted[slot] ? 1.6 : 2.6, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;
    ctx.fillStyle = "#9fb4c8";
    ctx.font = "10px ui-monospace, monospace";
    ctx.fillText(this.label, 6, 11);
    ctx.fillStyle = rejects > 0 ? BAD : GOOD;
    const txt = rejects > 0 ? `${rejects} REJECTED` : "ALL ACCEPTED";
    ctx.fillText(txt, w - ctx.measureText(txt).width - 6, 11);
  }
}

// Sensor-health / FDIR grid: one pill per sensor (updating / gated / idle)
// plus the navigation and FDIR state.
class HealthGrid {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.rows = [];
  }

  set(rows) {
    this.rows = rows;
  }

  reset() {
    this.rows = [];
  }

  draw() {
    const { ctx, canvas } = this;
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "rgba(8, 12, 18, 0.85)";
    ctx.fillRect(0, 0, w, h);
    ctx.fillStyle = "#9fb4c8";
    ctx.font = "10px ui-monospace, monospace";
    ctx.fillText("SENSOR HEALTH / FDIR", 6, 11);
    const rowH = (h - 18) / Math.max(this.rows.length, 1);
    this.rows.forEach((r, i) => {
      const y = 18 + i * rowH;
      ctx.fillStyle = DIM;
      ctx.font = "9px ui-monospace, monospace";
      ctx.fillText(r.label, 6, y + rowH / 2 + 3);
      // pill
      const pw = 78;
      const px = w - pw - 6;
      const py = y + 3;
      const ph = rowH - 6;
      ctx.fillStyle = `${r.color}22`;
      ctx.strokeStyle = r.color;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.roundRect(px, py, pw, ph, 3);
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = r.color;
      ctx.font = "9px ui-monospace, monospace";
      ctx.fillText(r.state, px + (pw - ctx.measureText(r.state).width) / 2, py + ph / 2 + 3);
    });
  }
}

function formatValue(value) {
  const m = Math.abs(value);
  if (m >= 10_000) return value.toExponential(1);
  if (m >= 100) return value.toFixed(0);
  if (m >= 1) return value.toFixed(2);
  return value.toFixed(3);
}

function attitudeErrorDeg(a, b) {
  const dot = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  return (2 * Math.acos(Math.min(dot, 1)) * 180) / Math.PI;
}

export class EngineeringPanel {
  constructor(root) {
    this.root = root;
    root.innerHTML = `
      <h3>FLIGHT COMPUTER — LIVE GN&amp;C</h3>
      <div class="charts"></div>
      <p class="charts-note">truth vs. onboard estimate — the renderer reads truth,
      the flight computer never does. Bands are the EKF's own reported ±1σ.</p>`;
    const host = root.querySelector(".charts");
    const canvas = (hh = 86) => {
      const c = document.createElement("canvas");
      c.width = 300;
      c.height = hh;
      host.appendChild(c);
      return c;
    };
    this.charts = {
      navPosError: new StripChart(canvas(), "NAV POS ERROR vs EKF ±1σ", "m", { color: ACCENT, band: true, logFloor: 0.1 }),
      navVelError: new StripChart(canvas(), "NAV VEL ERROR vs EKF ±1σ", "m/s", { color: GOOD, band: true, logFloor: 0.01 }),
      attitudeError: new StripChart(canvas(), "ATTITUDE EST ERROR", "deg", { color: WARN, logFloor: 1e-4 }),
      gating: new GatingChart(canvas(), "GNSS INNOVATION GATING (χ²)", 16),
      health: new HealthGrid(canvas(96)),
      dynamicPressure: new StripChart(canvas(), "DYNAMIC PRESSURE q", "kPa", { color: "#c8a2ff" }),
      throttle: new StripChart(canvas(), "STAGE THROTTLE (mean burning)", "%", { color: "#9dff7f" }),
    };
    this.accumulator = 0;
    this.drSince = null;
  }

  reset() {
    Object.values(this.charts).forEach((c) => c.reset());
    this.drSince = null;
  }

  update(view) {
    if (view.timeS - this.accumulator < 0.1) return;
    this.accumulator = view.timeS;
    const speed = view.groundSpeed;

    if (view.estimateValid) {
      const posErr = Math.hypot(
        view.estimatePosition[0] - view.position[0],
        view.estimatePosition[1] - view.position[1],
        view.estimatePosition[2] - view.position[2],
      );
      const velErr = Math.hypot(
        view.estimateVelocity[0] - view.velocity[0],
        view.estimateVelocity[1] - view.velocity[1],
        view.estimateVelocity[2] - view.velocity[2],
      );
      const posSigma = Math.sqrt(Math.max(0, view.positionVar[0] + view.positionVar[1] + view.positionVar[2]));
      const velSigma = Math.sqrt(Math.max(0, view.velocityVar[0] + view.velocityVar[1] + view.velocityVar[2]));
      const attSigma = (Math.sqrt(Math.max(0, view.attitudeVar[0] + view.attitudeVar[1] + view.attitudeVar[2])) * 180) / Math.PI;
      this.charts.navPosError.push(view.timeS, posErr, posSigma);
      this.charts.navVelError.push(view.timeS, velErr, velSigma);
      this.charts.attitudeError.push(view.timeS, attitudeErrorDeg(view.estimateAttitude, view.attitude), attSigma);
      // gating: GNSS χ² (= whitened innovation magnitude²), accepted vs gated.
      if (view.gnssUpdated || view.innovationRejected) {
        this.charts.gating.push(view.timeS, view.gnssChi2, view.gnssUpdated && !view.innovationRejected);
      }
    }

    this.charts.dynamicPressure.push(view.timeS, (0.5 * view.densityKgM3 * speed * speed) / 1000);
    const burning = view.engines.filter((e) => e.fraction > 0.02);
    this.charts.throttle.push(
      view.timeS,
      burning.length ? (100 * burning.reduce((s, e) => s + e.fraction, 0)) / burning.length : 0,
    );

    // Sensor health rows.
    if (view.deadReckoning && this.drSince === null) this.drSince = view.timeS;
    if (!view.deadReckoning) this.drSince = null;
    const sensor = (label, chi2, updated) => {
      if (chi2 <= 0 && !updated) return { label, state: "—", color: DIM };
      if (updated) return { label, state: "UPDATING", color: GOOD };
      return { label, state: "IDLE", color: DIM };
    };
    const drDur = this.drSince !== null ? view.timeS - this.drSince : 0;
    this.charts.health.set([
      sensor("GNSS", view.gnssChi2, view.gnssUpdated),
      { label: "IMU", state: view.imuChi2 > 0 ? "FUSING" : "—", color: view.imuChi2 > 0 ? GOOD : DIM },
      { label: "STAR-TRK", state: view.starChi2 > 0 ? "FUSING" : "—", color: view.starChi2 > 0 ? GOOD : DIM },
      {
        label: "NAV",
        state: view.deadReckoning ? `DEAD-RECKON ${drDur.toFixed(0)}s` : "GNSS-AIDED",
        color: view.deadReckoning ? WARN : GOOD,
      },
      { label: "FDIR", state: view.fdirTriggered ? "TRIPPED" : "CLEAR", color: view.fdirTriggered ? BAD : GOOD },
    ]);
  }

  draw() {
    Object.values(this.charts).forEach((c) => c.draw());
  }
}
