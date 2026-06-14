// Front-end orchestrator: spawns the simulation worker, decodes flat
// snapshots through the layout contract, interpolates between sim
// frames for smooth rendering, and wires the webcast HUD, engineering
// overlay, and the malfunction sandbox.

import { AscentScene } from "./scene.js";
import { Hud } from "./hud.js";
import { EngineeringPanel } from "./charts.js";
import { AudioEngine } from "./audio.js";

const params = new URLSearchParams(location.search);
const initialScenario = params.get("scenario") ?? "phalcon9-orbit-boostback";
const sharedTimeline = decodeTimeline(params.get("t"));

const worker = new Worker(new URL("./worker.js", import.meta.url), { type: "module" });

const state = {
  layout: null,
  scene: null,
  hud: null,
  engineering: null,
  frames: [],
  timeline: null,
  timeWarp: Number(params.get("warp") ?? 4),
  fcMode: params.get("fc") ?? "ekf",
  paused: true,
  finished: false,
  launched: false,
  started: performance.now(),
};

const ui = {
  canvas: document.getElementById("scene"),
  hud: document.getElementById("hud"),
  engineering: document.getElementById("engineering"),
  sandbox: document.getElementById("sandbox"),
  toast: document.getElementById("toast"),
  launch: document.getElementById("launch"),
  scenarioSelect: document.getElementById("scenario-select"),
  fcSelect: document.getElementById("fc-select"),
  warpButtons: [...document.querySelectorAll("[data-warp]")],
  cameraButtons: [...document.querySelectorAll("[data-camera]")],
  engineeringToggle: document.getElementById("toggle-engineering"),
  sandboxToggle: document.getElementById("toggle-sandbox"),
  soundToggle: document.getElementById("toggle-sound"),
  ghostToggle: document.getElementById("toggle-ghost"),
  share: document.getElementById("share"),
  reset: document.getElementById("reset"),
};

function decodeTimeline(encoded) {
  if (!encoded) return null;
  try {
    return atob(encoded.replace(/-/g, "+").replace(/_/g, "/"));
  } catch {
    return null;
  }
}

function encodeTimeline(json) {
  return btoa(json).replace(/\+/g, "-").replace(/\//g, "_").replace(/=+$/, "");
}

function toast(message, isError = true) {
  ui.toast.textContent = message;
  ui.toast.classList.toggle("error", isError);
  ui.toast.hidden = false;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => {
    ui.toast.hidden = true;
  }, 5000);
}

// ---------------------------------------------------------------- decode

function decodeSnapshot(buffer, phase) {
  const f = state.layout.fields;
  const read3 = (base) => [buffer[base], buffer[base + 1], buffer[base + 2]];
  const read4 = (base) => [buffer[base], buffer[base + 1], buffer[base + 2], buffer[base + 3]];
  const readN = (base, n) => Array.from({ length: n }, (_, i) => buffer[base + i]);

  const engines = state.layout.engines.map((engine, index) => {
    const base = f.engines_base + index * state.layout.engine_slot_width;
    const thrust = read3(base);
    const norm = Math.hypot(...thrust);
    return {
      fraction: norm / engine.max_thrust_n,
      direction: norm > 1 ? thrust.map((c) => c / norm) : [0, 0, 1],
      stateIndex: buffer[base + 3],
      consumedKg: buffer[base + 4],
    };
  });

  const lanes = new Map();
  state.layout.bodies.forEach((body, index) => {
    const base = f.separated_base + index * state.layout.separated_slot_width;
    if (buffer[base] !== 1) return;
    lanes.set(body.id, {
      propagating: buffer[base + 1] === 1,
      position: read3(base + 2),
      velocity: read3(base + 5),
      attitude: read4(base + 8),
    });
  });

  // Ground-relative speed: subtract the local Earth-rotation velocity
  // ω × r (frame profile is uniform rotation about ECI +z). This is the
  // air-relative speed in still air — the number a launch webcast shows
  // and the one dynamic pressure is computed from.
  const OMEGA_EARTH = 7.2921151467e-5;
  const position = read3(f.position_eci_m);
  const velocity = read3(f.velocity_eci_m_s);
  const groundVelocity = [
    velocity[0] + OMEGA_EARTH * position[1],
    velocity[1] - OMEGA_EARTH * position[0],
    velocity[2],
  ];

  return {
    timeS: buffer[f.time_s],
    phase,
    position,
    velocity,
    groundSpeed: Math.hypot(...groundVelocity),
    attitude: read4(f.attitude_xyzw),
    massKg: buffer[f.mass_kg],
    densityKgM3: buffer[f.atmosphere_density_kg_m3],
    pressurePa: buffer[f.atmosphere_pressure_pa],
    guidanceTgo: buffer[f.guidance_time_to_go_s],
    estimateValid: buffer[f.estimate_valid] === 1,
    estimatePosition: read3(f.estimate_position_eci_m),
    estimateVelocity: read3(f.estimate_velocity_eci_m_s),
    estimateAttitude: read4(f.estimate_attitude_xyzw),
    deadReckoning: buffer[f.dead_reckoning] === 1,
    gnssChi2: buffer[f.gnss_chi2],
    fdirTriggered: buffer[f.fdir_triggered] === 1,
    // Estimator covariance (1σ² diagonals), bias, innovations + gating.
    positionVar: read3(f.estimate_position_var),
    velocityVar: read3(f.estimate_velocity_var),
    attitudeVar: read3(f.estimate_attitude_var),
    accelBias: read3(f.estimate_accel_bias),
    baroChi2: buffer[f.baro_chi2],
    magChi2: buffer[f.mag_chi2],
    imuChi2: buffer[f.imu_chi2],
    starChi2: buffer[f.star_tracker_chi2],
    gnssInnovation: readN(f.gnss_innovation, 6),
    baroInnovation: buffer[f.baro_innovation],
    magInnovation: read3(f.mag_innovation),
    gnssUpdated: buffer[f.gnss_updated] === 1,
    baroUpdated: buffer[f.baro_updated] === 1,
    magUpdated: buffer[f.mag_updated] === 1,
    innovationRejected: buffer[f.innovation_rejected] === 1,
    attitudeUnderObservable: buffer[f.attitude_under_observable] === 1,
    fdirTrippedMask: buffer[f.fdir_tripped_mask],
    fdirTicksSinceTrip: buffer[f.fdir_ticks_since_trip],
    estimatorActiveMode: buffer[f.estimator_active_mode],
    estimatorModeCount: buffer[f.estimator_mode_count],
    estimatorModeProbs: readN(f.estimator_mode_probabilities, 4),
    referenceValid: buffer[f.reference_valid] === 1,
    referenceAttitude: read4(f.reference_attitude_xyzw),
    referencePosition: read3(f.reference_position_eci_m),
    referenceVelocity: read3(f.reference_velocity_eci_m_s),
    referenceOmega: read3(f.reference_omega_body_rad_s),
    separatedCount: buffer[f.separated_count],
    engines,
    lanes,
  };
}

function interpolate(a, b, alpha) {
  if (!a) return b;
  const lerp3 = (u, v) => u.map((value, i) => value + (v[i] - value) * alpha);
  const view = {
    ...b,
    timeS: a.timeS + (b.timeS - a.timeS) * alpha,
    position: lerp3(a.position, b.position),
    velocity: lerp3(a.velocity, b.velocity),
    attitude: slerp(a.attitude, b.attitude, alpha),
    // Interpolate the onboard estimate + guidance reference to the SAME alpha
    // as truth, so est−truth (nav error, the 3D ghost) is consistent rather
    // than mixing interpolated truth with a raw estimate (a one-frame skew is
    // hundreds of metres at orbital speed).
    estimatePosition: lerp3(a.estimatePosition, b.estimatePosition),
    estimateVelocity: lerp3(a.estimateVelocity, b.estimateVelocity),
    estimateAttitude: slerp(a.estimateAttitude, b.estimateAttitude, alpha),
    referencePosition: lerp3(a.referencePosition, b.referencePosition),
    referenceVelocity: lerp3(a.referenceVelocity, b.referenceVelocity),
    referenceAttitude: slerp(a.referenceAttitude, b.referenceAttitude, alpha),
    lanes: new Map(),
  };
  for (const [bodyId, lane] of b.lanes) {
    const previous = a.lanes.get(bodyId);
    view.lanes.set(
      bodyId,
      previous
        ? {
            ...lane,
            position: lerp3(previous.position, lane.position),
            attitude: slerp(previous.attitude, lane.attitude, alpha),
          }
        : lane,
    );
  }
  return view;
}

function slerp(a, b, alpha) {
  let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  const sign = dot < 0 ? -1 : 1;
  dot *= sign;
  if (dot > 0.9995) {
    return a.map((value, i) => value + (sign * b[i] - value) * alpha);
  }
  const theta = Math.acos(Math.min(dot, 1));
  const sinTheta = Math.sin(theta);
  const wa = Math.sin((1 - alpha) * theta) / sinTheta;
  const wb = (Math.sin(alpha * theta) / sinTheta) * sign;
  return a.map((value, i) => wa * value + wb * b[i]);
}

// ---------------------------------------------------------------- worker

worker.onmessage = (event) => {
  const message = event.data;
  switch (message.type) {
    case "ready": {
      state.layout = message.layout;
      state.frames = [];
      state.finished = false;
      state.launched = false;
      state.paused = true;
      state.scene = new AscentScene(ui.canvas, message.layout);
      window.__scene = state.scene.scene;
      window.__ascent = state.scene;
      state.hud = new Hud(ui.hud, message.layout);
      state.engineering = new EngineeringPanel(ui.engineering);
      if (!state.audio) state.audio = new AudioEngine(message.layout);
      else state.audio.setLayout(message.layout);
      window.__audio = state.audio;
      populateScenarioSelect(message.scenarios);
      ui.fcSelect.value = state.fcMode;
      populateSandbox(message.layout);
      ui.launch.hidden = false;
      ui.launch.textContent = "LAUNCH";
      ui.launch.classList.remove("docked");
      document.getElementById("loading").hidden = true;
      pushRate();
      break;
    }
    case "frame": {
      const view = decodeSnapshot(message.state, message.phase);
      state.frames = [state.frames.at(-1) ?? null, view].filter(Boolean).slice(-2);
      if (message.timeline) state.timeline = message.timeline;
      break;
    }
    case "finished": {
      state.finished = true;
      break;
    }
    case "ack": {
      toast(`INJECTED · ${message.label}`, false);
      state.hud?.markStimulus(message.label, message.timeS);
      break;
    }
    case "error": {
      toast(message.message);
      break;
    }
    default:
      break;
  }
};

function initWorker(scenario, seed = undefined, timeline = undefined, fcMode = state.fcMode) {
  document.getElementById("loading").hidden = false;
  state.hud?.hideOutcome();
  worker.postMessage({ type: "init", scenario, seed, timeline, fcMode });
}

function pushRate() {
  worker.postMessage({ type: "rate", timeWarp: state.timeWarp, paused: state.paused });
  ui.warpButtons.forEach((button) =>
    button.classList.toggle("active", Number(button.dataset.warp) === state.timeWarp),
  );
}

// ---------------------------------------------------------------- UI

function populateScenarioSelect(scenarios) {
  if (ui.scenarioSelect.options.length === 0) {
    for (const { key, title } of scenarios) {
      const option = document.createElement("option");
      option.value = key;
      option.textContent = title;
      ui.scenarioSelect.appendChild(option);
    }
  }
  ui.scenarioSelect.value = state.layout.key;
}

function populateSandbox(layout) {
  const engineOptions = layout.engines
    .map((engine) => `<option value="${engine.id}">${engine.id}</option>`)
    .join("");
  const sensorButtons = layout.sensors
    .map(
      (sensor) =>
        `<button data-outage="${sensor}" ${sensor === "imu" ? "title='even this one — try it'" : ""}>${sensor.toUpperCase()} OUT 10s</button>`,
    )
    .join("");
  ui.sandbox.innerHTML = `
    <h3>FLIGHT-TEST SANDBOX</h3>
    <p>Disturb the vehicle; the onboard flight software has to cope.
       Every stimulus is logged — share the link to replay the exact run.</p>
    <div class="sandbox-row">
      <select id="fault-engine">${engineOptions}</select>
      <button id="fault-hardoff">ENGINE OUT</button>
      <button id="fault-thrust">THRUST −40%</button>
      <button id="fault-gimbal">GIMBAL LOCK</button>
    </div>
    <div class="sandbox-row">${sensorButtons}</div>
    <div class="sandbox-row wind-row">
      <label>WIND N <input id="wind-n" type="range" min="-60" max="60" value="0" /></label>
      <label>E <input id="wind-e" type="range" min="-60" max="60" value="0" /></label>
      <button id="wind-apply">SET WIND</button>
      <button id="wind-clear">CALM</button>
    </div>`;

  const engineSelect = ui.sandbox.querySelector("#fault-engine");
  const inject = (kind, magnitude) =>
    worker.postMessage({ type: "inject", engine: engineSelect.value, kind, magnitude });
  ui.sandbox.querySelector("#fault-hardoff").onclick = () => inject("hard_off", 0);
  ui.sandbox.querySelector("#fault-thrust").onclick = () => inject("thrust_loss", 0.6);
  ui.sandbox.querySelector("#fault-gimbal").onclick = () => inject("gimbal_lock", 0);
  ui.sandbox.querySelectorAll("[data-outage]").forEach((button) => {
    button.onclick = () =>
      worker.postMessage({ type: "outage", sensor: button.dataset.outage, durationS: 10 });
  });
  ui.sandbox.querySelector("#wind-apply").onclick = () =>
    worker.postMessage({
      type: "wind",
      north: Number(ui.sandbox.querySelector("#wind-n").value),
      east: Number(ui.sandbox.querySelector("#wind-e").value),
      down: 0,
      enabled: true,
    });
  ui.sandbox.querySelector("#wind-clear").onclick = () =>
    worker.postMessage({ type: "wind", north: 0, east: 0, down: 0, enabled: false });
}

ui.launch.onclick = () => {
  state.paused = !state.paused;
  state.launched = true;
  ui.launch.textContent = state.paused ? "RESUME" : "PAUSE";
  // Once flying, dock the control to the top-left so it stops covering the
  // vehicle; the big centred button is only the pre-launch call-to-action.
  ui.launch.classList.add("docked");
  if (state.audio?.enabled) state.audio.start(); // gesture-gated audio
  pushRate();
};

ui.ghostToggle.onclick = () => {
  const on = !ui.ghostToggle.classList.contains("active");
  state.scene?.setGhost(on);
  ui.ghostToggle.classList.toggle("active", on);
};

ui.soundToggle.onclick = () => {
  if (!state.audio) return;
  const on = !state.audio.enabled;
  if (on) state.audio.start();
  state.audio.setEnabled(on);
  ui.soundToggle.classList.toggle("active", on);
};

ui.warpButtons.forEach((button) => {
  button.onclick = () => {
    state.timeWarp = Number(button.dataset.warp);
    pushRate();
  };
});

ui.cameraButtons.forEach((button) => {
  button.onclick = () => {
    state.scene?.setCameraMode(button.dataset.camera);
    ui.cameraButtons.forEach((candidate) =>
      candidate.classList.toggle("active", candidate === button),
    );
  };
});

ui.engineeringToggle.onclick = () => {
  ui.engineering.hidden = !ui.engineering.hidden;
  ui.engineeringToggle.classList.toggle("active", !ui.engineering.hidden);
};

ui.sandboxToggle.onclick = () => {
  ui.sandbox.hidden = !ui.sandbox.hidden;
  ui.sandboxToggle.classList.toggle("active", !ui.sandbox.hidden);
};

ui.scenarioSelect.onchange = () => initWorker(ui.scenarioSelect.value);

// Reflying with a different onboard estimator is a full reset-with-config: the
// flight software is immutable mid-run, so swapping EKF↔SR-UKF restarts the
// mission under the new navigation filter (the legitimate "FC mode" knob).
ui.fcSelect.onchange = () => {
  state.fcMode = ui.fcSelect.value;
  toast(`NAV FILTER · ${ui.fcSelect.value === "sr_ukf" ? "SR-UKF" : "EKF"} — reflying`, false);
  initWorker(state.layout.key, undefined, undefined, state.fcMode);
};

ui.reset.onclick = () => initWorker(state.layout.key);

ui.share.onclick = async () => {
  if (!state.timeline) return;
  const fcParam = state.fcMode && state.fcMode !== "ekf" ? `&fc=${state.fcMode}` : "";
  const url = `${location.origin}${location.pathname}?scenario=${state.layout.key}${fcParam}&t=${encodeTimeline(state.timeline)}`;
  try {
    await navigator.clipboard.writeText(url);
    toast("Replay link copied — same physics, same faults, bit for bit.", false);
  } catch {
    toast(url, false);
  }
};

window.addEventListener("resize", () => state.scene?.resize());

// Conductor API for console experimenters: disturb + observe, never steer.
window.Ascent = {
  injectEngineFault: (engine, kind = "hard_off", magnitude = 0) =>
    worker.postMessage({ type: "inject", engine, kind, magnitude }),
  injectSensorOutage: (sensor, durationS = 10) =>
    worker.postMessage({ type: "outage", sensor, durationS }),
  setWind: (north, east, down = 0) =>
    worker.postMessage({ type: "wind", north, east, down, enabled: true }),
  clearWind: () => worker.postMessage({ type: "wind", north: 0, east: 0, down: 0, enabled: false }),
  latest: () => state.frames.at(-1),
  layout: () => state.layout,
  timeline: () => state.timeline,
};

// ---------------------------------------------------------------- render

function render() {
  requestAnimationFrame(render);
  const [previous, latest] = [state.frames.at(-2), state.frames.at(-1)];
  if (!latest || !state.scene) return;

  // Interpolate between the two newest sim frames (the worker posts at
  // a fixed cadence, the display runs faster).
  const alpha = previous
    ? Math.min(((performance.now() % (1000 / 30)) / (1000 / 30)) * 1, 1)
    : 1;
  const view = interpolate(previous, latest, alpha);

  for (const bodyId of view.lanes.keys()) state.scene.detachBody(bodyId);
  state.scene.update(view, (performance.now() - state.started) / 1000);
  state.hud.update(view);
  // Camera-to-vehicle distance (vehicle is always at the scene origin) drives
  // the acoustic distance attenuation; a paused sim falls silent.
  if (state.audio) {
    state.audio.update(view, state.scene.camera.position.length(), state.paused);
  }
  if (!ui.engineering.hidden) {
    state.engineering.update(view);
    state.engineering.draw();
  }
  if (state.finished && state.launched) state.hud.showOutcome(view);
}

initWorker(initialScenario, undefined, sharedTimeline ?? undefined, state.fcMode);
render();
