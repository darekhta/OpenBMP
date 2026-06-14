// End-to-end smoke test of the WebAssembly package: load the real wasm
// artifact (no browser, plain Node), fly the flagship scenario to
// completion, and assert the physics milestones a visitor will see —
// stage separation, booster lane tracking, and orbital insertion.
//
// Run from the repository root after a wasm build:
//
//   node web/ascent/test/smoke.mjs
//
// Exits non-zero on any failed expectation.

import { readFile } from "node:fs/promises";
import init, { AscentSim, scenario_list } from "../pkg/openbmp_web.js";

const wasmUrl = new URL("../pkg/openbmp_web_bg.wasm", import.meta.url);
await init(await readFile(wasmUrl));

const failures = [];
const check = (ok, label) => {
  console.log(`${ok ? "ok " : "FAIL"} ${label}`);
  if (!ok) failures.push(label);
};

const scenarios = JSON.parse(scenario_list());
check(scenarios.length >= 2, `scenario list has ${scenarios.length} entries`);

const sim = new AscentSim("phalcon9-orbit-boostback", undefined, undefined);
const layout = JSON.parse(sim.layout_json());
const f = layout.fields;
check(layout.validation === "validated-toy", `validation label: ${layout.validation}`);
check(layout.engines.length >= 10, `${layout.engines.length} engines in layout`);

const vec = (snap, base) => [snap[base], snap[base + 1], snap[base + 2]];
const norm = (v) => Math.hypot(v[0], v[1], v[2]);
const EARTH_RADIUS_M = 6_370_000;

let sawSeparation = false;
let maxSpeed = 0;
const phaseLog = [];
let lastPhase = "";

const started = Date.now();
while (!sim.is_finished()) {
  sim.step_many(500); // 10 s of flight per call at dt = 0.02 s
  const snap = sim.snapshot();
  const t = snap[f.time_s];
  const speed = norm(vec(snap, f.velocity_eci_m_s));
  const altitude = norm(vec(snap, f.position_eci_m)) - EARTH_RADIUS_M;
  maxSpeed = Math.max(maxSpeed, speed);
  if (snap[f.separated_count] > 0) sawSeparation = true;

  const phase = sim.mission_phase();
  if (phase !== lastPhase) {
    phaseLog.push(`${t.toFixed(1)}s → ${phase}`);
    lastPhase = phase;
  }
  if (Math.round(t) % 100 === 0) {
    console.log(
      `   t=${t.toFixed(0).padStart(4)}s  alt=${(altitude / 1000).toFixed(1).padStart(6)} km` +
        `  |v|=${speed.toFixed(0).padStart(4)} m/s  phase=${phase}`,
    );
  }
}

const wall = (Date.now() - started) / 1000;
const snap = sim.snapshot();
const finalTime = snap[f.time_s];
const finalSpeed = norm(vec(snap, f.velocity_eci_m_s));
const finalAltitude = norm(vec(snap, f.position_eci_m)) - EARTH_RADIUS_M;
const rtf = finalTime / wall;

console.log(`\nphase ladder: ${phaseLog.join("  |  ")}`);
console.log(
  `final: t=${finalTime.toFixed(1)}s alt=${(finalAltitude / 1000).toFixed(1)}km ` +
    `|v|=${finalSpeed.toFixed(1)}m/s  (wall ${wall.toFixed(1)}s, ${rtf.toFixed(1)}x realtime)`,
);

check(sim.is_finished(), "run finished");
check(sawSeparation, "booster separation tracked as an independent lane");
check(phaseLog.length >= 3, `mission phase ladder advanced (${phaseLog.length} phases)`);
check(finalAltitude > 150_000, `final altitude ${(finalAltitude / 1000).toFixed(0)} km > 150 km`);
check(finalSpeed > 7_500, `final speed ${finalSpeed.toFixed(0)} m/s is orbital-class`);
check(snap[f.estimate_valid] === 1, "EKF estimate published throughout");

if (failures.length > 0) {
  console.error(`\n${failures.length} smoke expectation(s) failed`);
  process.exit(1);
}
console.log("\nwasm smoke: all expectations met");
