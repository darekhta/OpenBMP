// Procedural launch audio — synthesized live with the Web Audio API, no
// sampled assets (license-clean, lightweight, parametric).
//
// What makes synthesized rocket sound read as a *rocket* rather than wind:
//   1. TURBULENCE — the roar churns/pulses; a slow random modulator AMs the
//      engine bus so it never sits as steady hiss.
//   2. CRACKLE — discrete sharp transient pops (shock-cell noise) scheduled
//      as a Poisson-ish stream, not a continuous high-pass band.
//   3. SATURATION — a waveshaper drives the bus into soft clipping for grit
//      that grows with thrust.
//   4. A resonant low-end BODY (peaking ~140 Hz) over sub-bass RUMBLE, with
//      pink-noise spectral tilt.
//
// It stays physical: loudness ∝ total thrust; roar/crackle ∝ air density (so
// it fades toward vacuum); everything ∝ camera distance (a pad camera hears
// the launch recede); aero/wind ∝ dynamic pressure (peaks at Max-Q).
//
// The AudioContext is created/resumed only from a user gesture; every
// failure is swallowed so audio can never break the page or the tests.

const SEA_LEVEL_RHO = 1.225;

export class AudioEngine {
  constructor(layout) {
    this.started = false;
    this.enabled = false; // sound is opt-in; off until the user enables it
    this.ctx = null;
    this.nextPop = 0;
    this.setLayout(layout);
  }

  setLayout(layout) {
    this.layout = layout;
    this.prevLoud = 0;
    this.prevSep = 0;
    this.refThrust =
      layout.engines.reduce(
        (s, e) => s + (e.mounted_to === layout.bodies[0]?.id ? e.max_thrust_n : 0),
        0,
      ) || 3.9e6;
  }

  start() {
    if (this.started) {
      this.ctx?.resume?.().catch(() => {});
      return;
    }
    try {
      const Ctx = window.AudioContext || window.webkitAudioContext;
      if (!Ctx) return;
      const ctx = (this.ctx = new Ctx());
      this.started = true;

      const master = (this.master = ctx.createGain());
      master.gain.value = this.enabled ? 0.7 : 0.0001;
      master.connect(ctx.destination);
      this.analyser = ctx.createAnalyser();
      this.analyser.fftSize = 256;
      master.connect(this.analyser);

      this.pink = this._noise(ctx, 3, "pink");
      this.brown = this._noise(ctx, 3, "brown");
      this.white = this._noise(ctx, 2, "white");
      this.popBuf = this._noise(ctx, 0.06, "white");

      // The WHOLE mix runs through one soft-clip saturation so loud passages
      // grit-and-bound musically instead of hard-clipping at the output.
      const shaper = ctx.createWaveShaper();
      shaper.curve = this._satCurve(2.4);
      shaper.oversample = "2x";
      this.engineBus = ctx.createGain();
      this.engineBus.gain.value = 1.5; // drive into the shaper
      this.engineBus.connect(shaper).connect(master);

      const pinkSrc = ctx.createBufferSource();
      pinkSrc.buffer = this.pink;
      pinkSrc.loop = true;
      const brownSrc = ctx.createBufferSource();
      brownSrc.buffer = this.brown;
      brownSrc.loop = true;
      const whiteSrc = ctx.createBufferSource();
      whiteSrc.buffer = this.white;
      whiteSrc.loop = true;

      // Roar: pink noise → movable lowpass + a resonant low body.
      this.roarLP = ctx.createBiquadFilter();
      this.roarLP.type = "lowpass";
      this.roarLP.frequency.value = 900;
      this.roarLP.Q.value = 0.7;
      this.roarPeak = ctx.createBiquadFilter();
      this.roarPeak.type = "peaking";
      this.roarPeak.frequency.value = 140;
      this.roarPeak.Q.value = 1.1;
      this.roarPeak.gain.value = 7;
      this.roarGain = ctx.createGain();
      this.roarGain.gain.value = 0;
      pinkSrc.connect(this.roarLP).connect(this.roarPeak).connect(this.roarGain).connect(this.engineBus);

      // Sub-bass rumble.
      this.rumLP = ctx.createBiquadFilter();
      this.rumLP.type = "lowpass";
      this.rumLP.frequency.value = 80;
      this.rumGain = ctx.createGain();
      this.rumGain.gain.value = 0;
      brownSrc.connect(this.rumLP).connect(this.rumGain).connect(this.engineBus);

      // Turbulence modulator: slow random wander AM'ing the roar + rumble so
      // the bed churns. A node connected to an AudioParam adds to its value.
      const turbSrc = ctx.createBufferSource();
      turbSrc.buffer = this.brown;
      turbSrc.loop = true;
      const turbLP = ctx.createBiquadFilter();
      turbLP.type = "lowpass";
      turbLP.frequency.value = 7;
      const turbNorm = ctx.createGain();
      turbNorm.gain.value = 26; // bring the heavily-filtered wander up to ~±1
      this.turbDepth = ctx.createGain();
      this.turbDepth.gain.value = 0;
      turbSrc.connect(turbLP).connect(turbNorm).connect(this.turbDepth);
      this.turbDepth.connect(this.roarGain.gain);
      this.turbDepth.connect(this.rumGain.gain);

      // Crackle pops are scheduled live; routed through the saturation bus.
      this.crackleBus = ctx.createGain();
      this.crackleBus.gain.value = 0.55;
      this.crackleBus.connect(this.engineBus);

      // Aero/wind band.
      this.windBP = ctx.createBiquadFilter();
      this.windBP.type = "bandpass";
      this.windBP.frequency.value = 480;
      this.windBP.Q.value = 0.6;
      this.windGain = ctx.createGain();
      this.windGain.gain.value = 0;
      whiteSrc.connect(this.windBP).connect(this.windGain).connect(this.engineBus);

      pinkSrc.start();
      brownSrc.start();
      whiteSrc.start();
      turbSrc.start();
      this.nextPop = ctx.currentTime;
    } catch {
      /* audio unavailable — never break the page */
    }
  }

  setEnabled(on) {
    this.enabled = on;
    if (this.master && this.ctx) {
      this.master.gain.setTargetAtTime(on ? 0.85 : 0.0001, this.ctx.currentTime, 0.05);
    }
    if (on) this.ctx?.resume?.().catch(() => {});
  }

  update(view, camDist, paused) {
    if (!this.started || !this.ctx || !this.master) return;
    const ctx = this.ctx;
    const now = ctx.currentTime;
    const tau = 0.08;
    const live = paused ? 0 : 1;

    let thrust = 0;
    for (let i = 0; i < view.engines.length; i += 1) {
      thrust += view.engines[i].fraction * this.layout.engines[i].max_thrust_n;
    }
    const loud = Math.min(1.1, thrust / this.refThrust);
    const air = Math.min(1, Math.pow(Math.max(view.densityKgM3, 0) / SEA_LEVEL_RHO, 0.4));
    const q = 0.5 * view.densityKgM3 * view.groundSpeed * view.groundSpeed;
    const q01 = Math.min(1.2, q / 33000);
    const ref = 480;
    const dist = camDist > 0 ? (ref * ref) / (ref * ref + camDist * camDist) : 0.7;
    const engineLevel = loud * dist * live; // overall engine loudness this frame

    const set = (pa, v) => pa.setTargetAtTime(Math.max(0, v), now, tau);
    set(this.roarGain.gain, engineLevel * air * 0.42);
    // Faint structure-borne rumble survives in vacuum; the airborne roar does
    // not. Turbulence churn rides the airborne roar, so it fades with air too
    // (otherwise it injects roar where there's no base level).
    set(this.rumGain.gain, engineLevel * (0.12 + 0.88 * air) * 0.5);
    set(this.turbDepth.gain, engineLevel * air * 0.6);
    set(this.windGain.gain, q01 * dist * live * 0.16);
    this.roarLP.frequency.setTargetAtTime(520 + 900 * Math.min(1, loud), now, tau);

    // Crackle: a stream of sharp pops, denser at full thrust, gone in vacuum.
    const cl = engineLevel * air;
    const rate = Math.min(42, cl * 52);
    if (this.nextPop < now) this.nextPop = now;
    let guard = 0;
    while (rate > 0.5 && this.nextPop < now + 0.08 && guard < 14) {
      this._pop(this.nextPop, cl);
      this.nextPop += (0.5 + Math.random()) / rate;
      guard += 1;
    }

    if (!paused) {
      if (loud > 0.06 && this.prevLoud <= 0.06) this._burst("lowpass", 280, 1.6, 0.6);
      if (view.separatedCount > this.prevSep) this._burst("lowpass", 150, 0.5, 0.7);
      this.prevLoud = loud;
      this.prevSep = view.separatedCount;
    }
  }

  _pop(t, level) {
    try {
      const ctx = this.ctx;
      const src = ctx.createBufferSource();
      src.buffer = this.popBuf;
      const bp = ctx.createBiquadFilter();
      bp.type = "bandpass";
      bp.frequency.value = 900 + Math.random() * 2700;
      bp.Q.value = 1.3;
      const g = ctx.createGain();
      src.connect(bp).connect(g).connect(this.crackleBus);
      const peak = Math.max(0.001, (0.45 + Math.random() * 0.55) * level);
      g.gain.setValueAtTime(0.0001, t);
      g.gain.exponentialRampToValueAtTime(peak, t + 0.0016);
      g.gain.exponentialRampToValueAtTime(0.0004, t + 0.012 + Math.random() * 0.03);
      src.start(t);
      src.stop(t + 0.06);
    } catch {
      /* ignore */
    }
  }

  _burst(type, freq, dur, peak) {
    try {
      const ctx = this.ctx;
      const src = ctx.createBufferSource();
      src.buffer = this.brown;
      src.loop = true;
      const f = ctx.createBiquadFilter();
      f.type = type;
      f.frequency.value = freq;
      const g = ctx.createGain();
      g.gain.value = 0;
      src.connect(f).connect(g).connect(this.engineBus);
      const t = ctx.currentTime;
      g.gain.setValueAtTime(0.0001, t);
      g.gain.linearRampToValueAtTime(this.enabled ? peak : 0, t + 0.06);
      g.gain.exponentialRampToValueAtTime(0.0008, t + dur);
      src.start(t);
      src.stop(t + dur + 0.05);
    } catch {
      /* ignore */
    }
  }

  _satCurve(amount) {
    const n = 1024;
    const curve = new Float32Array(n);
    const k = Math.tanh(amount);
    for (let i = 0; i < n; i += 1) {
      const x = (i / (n - 1)) * 2 - 1;
      curve[i] = Math.tanh(amount * x) / k;
    }
    return curve;
  }

  _noise(ctx, seconds, kind) {
    const len = Math.max(1, Math.floor(ctx.sampleRate * seconds));
    const buf = ctx.createBuffer(1, len, ctx.sampleRate);
    const d = buf.getChannelData(0);
    if (kind === "brown") {
      let last = 0;
      for (let i = 0; i < len; i += 1) {
        const w = Math.random() * 2 - 1;
        last = (last + 0.02 * w) / 1.02;
        d[i] = last * 3.2;
      }
    } else if (kind === "pink") {
      let b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0;
      for (let i = 0; i < len; i += 1) {
        const w = Math.random() * 2 - 1;
        b0 = 0.99886 * b0 + w * 0.0555179;
        b1 = 0.99332 * b1 + w * 0.0750759;
        b2 = 0.969 * b2 + w * 0.153852;
        b3 = 0.8665 * b3 + w * 0.3104856;
        b4 = 0.55 * b4 + w * 0.5329522;
        b5 = -0.7616 * b5 - w * 0.016898;
        d[i] = (b0 + b1 + b2 + b3 + b4 + b5 + b6 + w * 0.5362) * 0.11;
        b6 = w * 0.115926;
      }
    } else {
      for (let i = 0; i < len; i += 1) d[i] = Math.random() * 2 - 1;
    }
    return buf;
  }

  rms() {
    if (!this.analyser) return 0;
    const a = new Float32Array(this.analyser.fftSize);
    this.analyser.getFloatTimeDomainData(a);
    let s = 0;
    for (const v of a) s += v * v;
    return Math.sqrt(s / a.length);
  }
}
