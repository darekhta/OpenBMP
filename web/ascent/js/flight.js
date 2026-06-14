// Flight-instrument derivations from a decoded telemetry `view`.
//
// Everything here is pure math over the snapshot the worker already
// produces (ECI position/velocity, attitude quaternion, mass, atmosphere,
// guidance, per-engine state). It turns that into the quantities a real
// ascent HUD shows: altitude, air/inertial speed, vertical speed, Mach,
// dynamic pressure, axial g-load, throttle, downrange, the live orbit
// (apoapsis/periapsis + time to apoapsis), and attitude angles (pitch,
// roll, flight-path angle, angle of attack) for an artificial horizon.

export const EARTH_RADIUS_M = 6_370_000;
const MU_EARTH = 3.986004418e14; // m^3/s^2
const OMEGA_EARTH = 7.2921151467e-5; // rad/s
export const G0 = 9.80665;

const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const unit = (a) => {
  const n = norm(a);
  return n > 1e-9 ? scale(a, 1 / n) : [0, 0, 0];
};

// Rotate a body-frame vector into world (ECI) by quaternion [x,y,z,w].
function rotateByQuat(q, v) {
  const [x, y, z, w] = q;
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx),
  ];
}

export function deriveFlight(view, layout, prev) {
  const r = view.position;
  const v = view.velocity;
  const rMag = norm(r);
  const rHat = unit(r);
  const altitude = rMag - EARTH_RADIUS_M;
  const inertialSpeed = norm(v);
  const groundSpeed = view.groundSpeed;
  const verticalSpeed = dot(v, rHat); // radial component (rotation-free)

  // Speed of sound a = sqrt(gamma * P / rho); Mach only meaningful in air.
  const gamma = 1.4;
  const inAir = view.densityKgM3 > 2e-4 && view.pressurePa > 1;
  const aSound = inAir ? Math.sqrt((gamma * view.pressurePa) / view.densityKgM3) : NaN;
  const mach = inAir ? groundSpeed / aSound : NaN;
  const q = 0.5 * view.densityKgM3 * groundSpeed * groundSpeed; // dynamic pressure, Pa

  // Thrust / throttle / axial g-load.
  let thrust = 0;
  let firingMax = 0;
  let firing = 0;
  view.engines.forEach((engine, i) => {
    const max = layout.engines[i].max_thrust_n;
    thrust += engine.fraction * max;
    if (engine.fraction > 0.02) {
      firingMax += max;
      firing += 1;
    }
  });
  const throttle = firingMax > 0 ? thrust / firingMax : 0; // 0..1 of the firing set
  const accel = view.massKg > 0 ? thrust / view.massKg : 0; // m/s^2 axial
  const gLoad = accel / G0;

  // Downrange: surface arc from the launch site (lat0 lon0, co-rotating
  // about ECI +z) to the sub-vehicle point.
  const ang = OMEGA_EARTH * view.timeS;
  const siteDir = unit([Math.cos(ang), Math.sin(ang), 0]);
  const downrange = EARTH_RADIUS_M * Math.acos(Math.min(1, Math.max(-1, dot(siteDir, rHat))));

  // Live orbit from the state vector.
  const energy = (inertialSpeed * inertialSpeed) / 2 - MU_EARTH / rMag;
  const hVec = cross(r, v);
  const hMag = norm(hVec);
  const eVec = sub(
    scale(r, (inertialSpeed * inertialSpeed - MU_EARTH / rMag) / MU_EARTH),
    scale(v, dot(r, v) / MU_EARTH),
  );
  const ecc = norm(eVec);
  let apoAlt = NaN;
  let periAlt = NaN;
  let timeToApo = NaN;
  if (energy < 0) {
    const a = -MU_EARTH / (2 * energy); // semi-major axis
    apoAlt = a * (1 + ecc) - EARTH_RADIUS_M;
    periAlt = a * (1 - ecc) - EARTH_RADIUS_M;
    // Time to apoapsis via mean anomaly.
    if (ecc > 1e-4 && a > 0) {
      const nu = Math.acos(
        Math.min(1, Math.max(-1, dot(unit(eVec), rHat))),
      ) * (dot(r, v) < 0 ? -1 : 1);
      const E = Math.atan2(Math.sqrt(1 - ecc * ecc) * Math.sin(nu), ecc + Math.cos(nu));
      let M = E - ecc * Math.sin(E);
      const n = Math.sqrt(MU_EARTH / (a * a * a));
      let t = (Math.PI - M) / n; // apoapsis is at M = PI
      const period = (2 * Math.PI) / n;
      while (t < 0) t += period;
      timeToApo = t;
    }
  }

  // Attitude relative to the local horizon (for the artificial horizon).
  // Body +z is the nose / thrust axis; body +y the reference "up".
  const noseW = rotateByQuat(view.attitude, [0, 0, 1]);
  const upRefW = rotateByQuat(view.attitude, [0, 1, 0]);
  const pitch = Math.asin(Math.min(1, Math.max(-1, dot(unit(noseW), rHat)))); // above horizon
  // Roll: angle between the body-up reference and local up, in the plane
  // normal to the nose axis.
  const localUp = rHat;
  const n1 = unit(sub(localUp, scale(noseW, dot(localUp, noseW))));
  const n2 = unit(sub(upRefW, scale(noseW, dot(upRefW, noseW))));
  const rollSign = dot(cross(n2, n1), noseW) < 0 ? -1 : 1;
  const roll = rollSign * Math.acos(Math.min(1, Math.max(-1, dot(n1, n2))));
  // Flight-path angle (velocity above horizon) and angle of attack.
  const fpa = inertialSpeed > 1 ? Math.asin(Math.min(1, Math.max(-1, verticalSpeed / inertialSpeed))) : 0;
  const aoaDeg = (pitch - fpa) * (180 / Math.PI);

  // Attitude-error markers for the ADI: where another frame's nose points
  // relative to TRUTH's nose, projected onto truth's body right/up axes
  // (small-angle radians). Commanded(reference) offset = control tracking
  // error; estimate offset = nav attitude error.
  const truthRight = unit(rotateByQuat(view.attitude, [1, 0, 0]));
  const truthUp = unit(rotateByQuat(view.attitude, [0, 1, 0]));
  const adiOffset = (q) => {
    const nO = unit(rotateByQuat(q, [0, 0, 1]));
    const d = sub(nO, unit(noseW));
    return [dot(d, truthRight), dot(d, truthUp)]; // [yaw, pitch] radians
  };
  const attErrRef = view.referenceValid ? adiOffset(view.referenceAttitude) : null;
  const attErrEst = adiOffset(view.estimateAttitude);

  return {
    timeS: view.timeS,
    altitude,
    groundSpeed,
    inertialSpeed,
    verticalSpeed,
    mach,
    q,
    throttle,
    gLoad,
    thrust,
    firing,
    downrange,
    apoAlt,
    periAlt,
    timeToApo,
    ecc,
    pitchDeg: pitch * (180 / Math.PI),
    rollDeg: roll * (180 / Math.PI),
    fpaDeg: fpa * (180 / Math.PI),
    aoaDeg,
    attErrRef,
    attErrEst,
    massKg: view.massKg,
  };
}
