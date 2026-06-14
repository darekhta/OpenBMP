// Simulation worker: owns the WebAssembly session and steps it on a
// fixed cadence, posting flat state snapshots to the render thread.
//
// The protocol is intentionally tiny:
//   main → worker : {type:"init", scenario, seed?, timeline?, fcMode?}
//                   {type:"rate", timeWarp, paused}
//                   {type:"inject", engine, kind, magnitude}
//                   {type:"outage", sensor, durationS}
//                   {type:"wind", north, east, down, enabled}
//   worker → main : {type:"ready", layout, scenarios}
//                   {type:"frame", state:Float64Array (transferred), phase, timeline}
//                   {type:"finished"}
//                   {type:"error", message}
//
// The worker can disturb and observe the simulation; it has no command
// that steers it. Guidance, navigation, and control run inside the
// WebAssembly module — the flight software flies, here as everywhere.

import init, { AscentSim, scenario_list } from "../pkg/openbmp_web.js";

const FRAME_HZ = 30;

let sim = null;
let dtS = 0.02;
let timeWarp = 1;
let paused = true;
let stepCarry = 0;
let timer = null;
let timelineDirty = true;

function post(message, transfer) {
  self.postMessage(message, transfer ?? []);
}

function fail(error) {
  post({ type: "error", message: String(error?.message ?? error) });
}

function frame() {
  if (!sim) return;
  try {
    if (!paused && !sim.is_finished()) {
      stepCarry += timeWarp / (dtS * FRAME_HZ);
      const steps = Math.floor(stepCarry);
      stepCarry -= steps;
      if (steps > 0 && sim.step_many(steps)) {
        post({ type: "finished" });
      }
    }
    const state = new Float64Array(sim.snapshot());
    post(
      {
        type: "frame",
        state,
        phase: sim.mission_phase(),
        timeline: timelineDirty ? sim.timeline_json() : null,
      },
      [state.buffer],
    );
    timelineDirty = false;
  } catch (error) {
    fail(error);
  }
}

self.onmessage = async (event) => {
  const message = event.data;
  try {
    switch (message.type) {
      case "init": {
        if (timer !== null) clearInterval(timer);
        if (!self.__wasmReady) {
          await init();
          self.__wasmReady = true;
        }
        sim?.free?.();
        sim = new AscentSim(
          message.scenario,
          message.seed === undefined ? undefined : BigInt(message.seed),
          message.timeline ?? undefined,
          message.fcMode ?? undefined,
        );
        const layout = JSON.parse(sim.layout_json());
        dtS = layout.dt_s;
        stepCarry = 0;
        paused = true;
        timelineDirty = true;
        post({ type: "ready", layout, scenarios: JSON.parse(scenario_list()) });
        timer = setInterval(frame, 1000 / FRAME_HZ);
        break;
      }
      case "rate": {
        timeWarp = message.timeWarp;
        paused = message.paused;
        break;
      }
      case "inject": {
        sim.inject_engine_fault(message.engine, message.kind, message.magnitude);
        timelineDirty = true;
        post({
          type: "ack",
          label: `${message.engine} ${message.kind.replace(/_/g, " ").toUpperCase()}`,
          timeS: sim.time_s(),
        });
        break;
      }
      case "outage": {
        sim.inject_sensor_outage(message.sensor, message.durationS);
        timelineDirty = true;
        post({
          type: "ack",
          label: `${message.sensor.toUpperCase()} OUTAGE ${message.durationS}s`,
          timeS: sim.time_s(),
        });
        break;
      }
      case "wind": {
        sim.set_wind(message.north, message.east, message.down, message.enabled);
        timelineDirty = true;
        post({
          type: "ack",
          label: message.enabled
            ? `WIND ${message.north.toFixed(0)}N ${message.east.toFixed(0)}E m/s`
            : "WIND CLEARED",
          timeS: sim.time_s(),
        });
        break;
      }
      default:
        break;
    }
  } catch (error) {
    fail(error);
  }
};
