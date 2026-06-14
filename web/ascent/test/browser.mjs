// Browser end-to-end test: drives the served demo in headless Chrome
// over the DevTools protocol (no dependencies — Node's built-in
// WebSocket), launches the flight at high time-warp, injects an
// engine-out through the public conductor API, and captures screenshots
// plus console output.
//
//   1. python3 -m http.server 8741 --directory web/ascent &
//   2. node web/ascent/test/browser.mjs [chrome-binary] [base-url]
//
// Screenshots land in /tmp/ascent-shot-*.png. Exits non-zero if the app
// fails to reach the flying state or logs an uncaught error.

import { spawn } from "node:child_process";
import { writeFile } from "node:fs/promises";
import { setTimeout as sleep } from "node:timers/promises";

const chromeBinary =
  process.argv[2] ?? "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome";
const baseUrl = process.argv[3] ?? "http://localhost:8741/index.html";
const port = 9241;

const chrome = spawn(
  chromeBinary,
  [
    "--headless=new",
    `--remote-debugging-port=${port}`,
    "--no-first-run",
    "--user-data-dir=/tmp/ascent-chrome-profile",
    "--window-size=1480,920",
    "about:blank",
  ],
  { stdio: "ignore" },
);
process.on("exit", () => chrome.kill());

// Wait for the DevTools endpoint, then open a fresh tab.
let target = null;
for (let attempt = 0; attempt < 50 && !target; attempt += 1) {
  await sleep(200);
  try {
    const response = await fetch(`http://localhost:${port}/json/new?about:blank`, {
      method: "PUT",
    });
    target = await response.json();
  } catch {
    /* devtools not up yet */
  }
}
if (!target) {
  console.error("FAIL devtools endpoint never came up");
  process.exit(1);
}

const ws = new WebSocket(target.webSocketDebuggerUrl);
await new Promise((resolve, reject) => {
  ws.onopen = resolve;
  ws.onerror = reject;
});

let nextId = 1;
const pending = new Map();
const consoleLines = [];
const pageErrors = [];

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  if (message.id && pending.has(message.id)) {
    pending.get(message.id)(message);
    pending.delete(message.id);
  } else if (message.method === "Runtime.consoleAPICalled") {
    const text = message.params.args.map((a) => a.value ?? a.description ?? "").join(" ");
    consoleLines.push(`[${message.params.type}] ${text}`);
  } else if (message.method === "Runtime.exceptionThrown") {
    pageErrors.push(message.params.exceptionDetails.exception?.description ?? "page exception");
  }
};

function send(method, params = {}) {
  return new Promise((resolve) => {
    const id = nextId;
    nextId += 1;
    pending.set(id, resolve);
    ws.send(JSON.stringify({ id, method, params }));
  });
}

async function evaluate(expression) {
  const reply = await send("Runtime.evaluate", {
    expression,
    returnByValue: true,
    awaitPromise: true,
  });
  return reply.result?.result?.value;
}

async function shot(name) {
  const reply = await send("Page.captureScreenshot", { format: "png" });
  await writeFile(`/tmp/ascent-shot-${name}.png`, Buffer.from(reply.result.data, "base64"));
  console.log(`shot: /tmp/ascent-shot-${name}.png`);
}

const failures = [];
const check = (ok, label) => {
  console.log(`${ok ? "ok " : "FAIL"} ${label}`);
  if (!ok) failures.push(label);
};

await send("Runtime.enable");
await send("Page.enable");
await send("Emulation.setDeviceMetricsOverride", {
  width: 1480,
  height: 920,
  deviceScaleFactor: 1,
  mobile: false,
});
await send("Page.navigate", { url: baseUrl });

// Wait for the sim worker to come up (loading veil hidden).
let ready = false;
for (let attempt = 0; attempt < 120 && !ready; attempt += 1) {
  await sleep(500);
  ready = await evaluate("document.getElementById('loading')?.hidden === true");
}
check(ready, "flight software loaded (worker ready)");
await shot("1-pad");

// Launch at 64x, open the engineering overlay.
await evaluate("document.querySelector('[data-warp=\"64\"]').click()");
await evaluate("document.getElementById('toggle-engineering').click()");
await evaluate("document.getElementById('launch').click()");
await sleep(4500); // ≈ T+280 s at 64x: past MECO and separation
const phaseAfterAscent = await evaluate("window.Ascent.latest()?.phase ?? ''");
const simTime = await evaluate("window.Ascent.latest()?.timeS ?? 0");
check(simTime > 120, `simulation advanced under warp (T+${Math.round(simTime)}s)`);
await shot("2-ascent");

// Restart, then engine-out at T+20 through the conductor API.
await evaluate("document.getElementById('reset').click()");
await sleep(2500);
await evaluate("document.querySelector('[data-warp=\"4\"]').click()");
await evaluate("document.getElementById('launch').click()");
await sleep(5000); // ≈ T+20 at 4x
await evaluate("window.Ascent.injectEngineFault('eng_s1_3', 'hard_off', 0)");
await sleep(1500);
const failedEngine = await evaluate(
  "window.Ascent.latest().engines.filter(e => e.stateIndex === 4).length",
);
check(failedEngine >= 1, "engine-out visible in telemetry (lifecycle = Failed)");
const timeline = await evaluate("window.Ascent.timeline()");
check(
  typeof timeline === "string" && timeline.includes("hard_off"),
  "stimulus timeline records the fault",
);
await evaluate("document.getElementById('toggle-sandbox').click()");
await sleep(800);
await shot("3-engine-out");

// Flight-computer selector: the demo can refly the mission under a different
// onboard navigation filter (reset-with-config; the FC is immutable mid-run).
const fcOptions = await evaluate(
  "[...document.getElementById('fc-select').options].map((o) => o.value).join(',')",
);
check(fcOptions === "ekf,sr_ukf", `FC selector offers ekf,sr_ukf (got ${fcOptions})`);
// Switch to SR-UKF; that re-inits the worker (loading veil reappears).
await evaluate(
  "(() => { const s = document.getElementById('fc-select'); s.value = 'sr_ukf'; s.dispatchEvent(new Event('change')); })()",
);
let srukfReady = false;
for (let attempt = 0; attempt < 40 && !srukfReady; attempt += 1) {
  await sleep(250);
  srukfReady = await evaluate("document.getElementById('loading')?.hidden === true");
}
check(srukfReady, "SR-UKF reflight re-initialised");
await evaluate("document.querySelector('[data-warp=\"64\"]').click()");
await evaluate("document.getElementById('launch').click()");
await sleep(3500);
const srukfTime = await evaluate("window.Ascent.latest()?.timeS ?? 0");
check(srukfTime > 120, `SR-UKF reflight advances under warp (T+${Math.round(srukfTime)}s)`);

const fatal = pageErrors.filter((line) => !line.includes("favicon"));
check(fatal.length === 0, `no uncaught page errors (${fatal.length})`);
if (fatal.length) console.error(fatal.join("\n"));
if (consoleLines.length) console.log(`console:\n  ${consoleLines.slice(0, 12).join("\n  ")}`);

ws.close();
chrome.kill();
if (failures.length) {
  console.error(`\n${failures.length} browser expectation(s) failed`);
  process.exit(1);
}
console.log("\nbrowser e2e: all expectations met");
