// Three.js scene: physically-toned HDR pipeline (linear render → bloom →
// ACES output), a shader Earth (day/night blend, city lights, ocean
// specular, terminator-tinted atmosphere, drifting cloud layer), a
// procedurally detailed vehicle built from the scenario layout, layered
// engine plumes with Mach diamonds and vacuum expansion, a co-rotating
// launch pad with liftoff steam, and damped cinematic cameras.
//
// The scene uses a floating origin anchored at the active vehicle: the
// stack sits at (0,0,0) and everything else (Earth, booster lane, pad)
// is offset by minus the vehicle's ECI position, so f32 precision never
// fights the 6 378 km position magnitudes.
//
// Frames: simulation positions are ECI metres mapped directly onto
// world axes. The scenario's frame profile is uniform Earth rotation
// about ECI +z, so the Earth group (and the launch-site pad anchored to
// it) spins at OMEGA_EARTH.

import * as THREE from "three";
import { EffectComposer } from "../vendor/addons/postprocessing/EffectComposer.js";
import { RenderPass } from "../vendor/addons/postprocessing/RenderPass.js";
import { UnrealBloomPass } from "../vendor/addons/postprocessing/UnrealBloomPass.js";
import { OutputPass } from "../vendor/addons/postprocessing/OutputPass.js";
import { buildAtmosphereLUTs, AtmospherePass } from "./atmosphere.js";
import { FlarePass } from "./flare.js";

const OMEGA_EARTH_RAD_S = 7.2921151467e-5;
const EARTH_RADIUS_M = 6_370_000;
const SEA_LEVEL_PRESSURE_PA = 101_325;
const SUN_DIRECTION = new THREE.Vector3(1, 0.42, 0.18).normalize();

// ----------------------------------------------------------------- noise

const PLUME_NOISE = `
  float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
  float vnoise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1, 0)), u.x),
               mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), u.x), u.y);
  }
  float fbm(vec2 p) { return 0.65 * vnoise(p) + 0.35 * vnoise(p * 2.7); }

  // 3D value noise + 4-octave fbm for turbulent plume body.
  float hash3(vec3 p) {
    p = fract(p * 0.3183099 + 0.1);
    p *= 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
  }
  float vnoise3(vec3 x) {
    vec3 i = floor(x), f = fract(x);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(mix(hash3(i + vec3(0,0,0)), hash3(i + vec3(1,0,0)), f.x),
                   mix(hash3(i + vec3(0,1,0)), hash3(i + vec3(1,1,0)), f.x), f.y),
               mix(mix(hash3(i + vec3(0,0,1)), hash3(i + vec3(1,0,1)), f.x),
                   mix(hash3(i + vec3(0,1,1)), hash3(i + vec3(1,1,1)), f.x), f.y), f.z);
  }
  float fbm3(vec3 p) {
    float s = 0.0, a = 0.5;
    for (int i = 0; i < 4; i++) { s += a * vnoise3(p); p *= 2.02; a *= 0.5; }
    return s;
  }

  // Blackbody colour from temperature (K), Tanner-Helland approximation,
  // returned in LINEAR space (the pipeline tonemaps once at OutputPass).
  vec3 blackbody(float kelvin) {
    float t = clamp(kelvin, 1000.0, 40000.0) / 100.0;
    float r, g, b;
    if (t <= 66.0) { r = 255.0; } else { r = 329.7 * pow(t - 60.0, -0.1332); }
    if (t <= 66.0) { g = 99.47 * log(t) - 161.1; } else { g = 288.1 * pow(t - 60.0, -0.0755); }
    if (t >= 66.0) { b = 255.0; } else if (t <= 19.0) { b = 0.0; } else { b = 138.5 * log(t - 10.0) - 305.0; }
    vec3 c = clamp(vec3(r, g, b) / 255.0, 0.0, 1.0);
    return pow(c, vec3(2.2));
  }
`;

// ---------------------------------------------------------- procedural art

function radialSpriteTexture(stops, size = 128) {
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  const gradient = ctx.createRadialGradient(
    size / 2, size / 2, 0,
    size / 2, size / 2, size / 2,
  );
  for (const [offset, color] of stops) gradient.addColorStop(offset, color);
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, size, size);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

// Hull albedo: white skin, panel seams, weld rings, wordmark, scorched
// aft band. Drawn once per body kind.
function hullTexture({ aftBand, wordmark }) {
  const w = 512;
  const h = 1024;
  const canvas = document.createElement("canvas");
  canvas.width = w;
  canvas.height = h;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "#f4f4f6";
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = "rgba(140,148,158,0.30)";
  ctx.lineWidth = 1;
  for (let x = 0; x <= w; x += 64) {
    ctx.beginPath(); ctx.moveTo(x + 0.5, 0); ctx.lineTo(x + 0.5, h); ctx.stroke();
  }
  for (let y = 0; y <= h; y += 80) {
    ctx.strokeStyle = y % 240 === 0 ? "rgba(120,128,138,0.45)" : "rgba(140,148,158,0.22)";
    ctx.beginPath(); ctx.moveTo(0, y + 0.5); ctx.lineTo(w, y + 0.5); ctx.stroke();
  }
  // Subtle vertical streaking so the skin reads under raking light.
  for (let i = 0; i < 240; i += 1) {
    const x = Math.random() * w;
    const y = Math.random() * h;
    ctx.fillStyle = `rgba(108,118,128,${0.02 + Math.random() * 0.05})`;
    ctx.fillRect(x, y, 1.5, 14 + Math.random() * 60);
  }
  if (aftBand) {
    const grad = ctx.createLinearGradient(0, h - 230, 0, h);
    grad.addColorStop(0, "rgba(30,32,36,0)");
    grad.addColorStop(0.55, "rgba(30,32,36,0.55)");
    grad.addColorStop(1, "rgba(16,17,20,0.95)");
    ctx.fillStyle = grad;
    ctx.fillRect(0, h - 230, w, 230);
  }
  if (wordmark) {
    ctx.save();
    ctx.translate(96, h * 0.46);
    ctx.rotate(Math.PI / 2);
    ctx.font = "700 54px 'Inter', 'Helvetica Neue', sans-serif";
    ctx.fillStyle = "#23262c";
    ctx.letterSpacing = "14px";
    ctx.fillText("PHALCON-9", 0, 0);
    ctx.restore();
    ctx.save();
    ctx.translate(96 + w / 2, h * 0.46);
    ctx.rotate(Math.PI / 2);
    ctx.font = "700 54px 'Inter', 'Helvetica Neue', sans-serif";
    ctx.fillStyle = "#23262c";
    ctx.letterSpacing = "14px";
    ctx.fillText("PHALCON-9", 0, 0);
    ctx.restore();
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.wrapS = THREE.RepeatWrapping;
  texture.anisotropy = 4;
  return texture;
}

// Deterministic 2-D value noise for terrain shaping and ground art.
function makeNoise(seed) {
  const hash = (x, y) => {
    let h = (x * 374761393 + y * 668265263 + seed * 974711) | 0;
    h = (h ^ (h >> 13)) * 1274126177;
    return (((h ^ (h >> 16)) >>> 0) % 100000) / 100000;
  };
  const smooth = (t) => t * t * (3 - 2 * t);
  return (x, y) => {
    const xi = Math.floor(x);
    const yi = Math.floor(y);
    const tx = smooth(x - xi);
    const ty = smooth(y - yi);
    const a = hash(xi, yi);
    const b = hash(xi + 1, yi);
    const c = hash(xi, yi + 1);
    const d = hash(xi + 1, yi + 1);
    return a + (b - a) * tx + (c - a) * ty + (a - b - c + d) * tx * ty;
  };
}

// Analytic wavy coastline, shared between the JS heightfield and the GLSL
// ocean foam so the surf line hugs the actual shore. The landmass is a big
// island whose centre sits inland of the pad, so the pad ends up on a
// coastal shelf near the south shore with open water in front of it.
const COAST_CENTER = [0, 1500];
const COAST_GLSL = `
  const vec2 COAST_CENTER = vec2(0.0, 1500.0);
  float coastR(float a){
    return 1900.0 + 240.0*sin(3.0*a + 0.7) + 110.0*sin(7.0*a + 2.1) + 60.0*sin(13.0*a + 4.0);
  }`;
function coastR(ang) {
  return 1900 + 240 * Math.sin(3 * ang + 0.7) + 110 * Math.sin(7 * ang + 2.1)
    + 60 * Math.sin(13 * ang + 4.0);
}

// Procedural-biome injection for the terrain MeshStandardMaterial: per-pixel
// elevation+slope colouring (wet sand → beach → grass → rock → snow) so the
// landscape reads crisp and varied at any distance, while keeping the
// standard lighting/shadow/fog pipeline. vTerr = object-space position,
// vTerrN = object-space normal (z-up), both supplied by the vertex injection.
const TERRAIN_NOISE_GLSL = `
  float terrHash(vec2 p){ return fract(sin(dot(p, vec2(127.1,311.7)))*43758.5453); }
  float terrNoise(vec2 p){ vec2 i=floor(p), f=fract(p); f=f*f*(3.0-2.0*f);
    float a=terrHash(i), b=terrHash(i+vec2(1.0,0.0)), c=terrHash(i+vec2(0.0,1.0)), d=terrHash(i+vec2(1.0,1.0));
    return mix(mix(a,b,f.x), mix(c,d,f.x), f.y); }
  float terrFbm(vec2 p){ float s=0.0, a=0.5; for(int i=0;i<5;i++){ s+=a*terrNoise(p); p*=2.04; a*=0.5; } return s; }
`;
const TERRAIN_BIOME_GLSL = `
  {
    float h = vTerr.z;
    float slope = clamp(vTerrN.z, 0.0, 1.0);
    vec2 q = vTerr.xy;
    float macro   = terrFbm(q * 0.0016);
    float micro   = terrFbm(q * 0.03);
    float fine    = terrFbm(q * 0.11);
    float patches = terrFbm(q * 0.006 + 4.0);
    float forest  = terrFbm(q * 0.0021 + 17.0);     // where woods cluster
    // Noisy height so biome boundaries scallop instead of forming clean rings.
    float hn = h + 9.0 * micro - 4.5 + 5.0 * fine;
    vec3 wetSand  = vec3(0.17, 0.14, 0.10);
    vec3 sand     = vec3(0.55, 0.47, 0.31);
    vec3 meadow   = vec3(0.26, 0.31, 0.16);
    vec3 grass    = vec3(0.16, 0.24, 0.11);
    vec3 woods    = vec3(0.07, 0.14, 0.07);
    vec3 dry      = vec3(0.40, 0.37, 0.20);
    vec3 dirt     = vec3(0.30, 0.24, 0.16);
    vec3 rock     = vec3(0.15, 0.13, 0.12);
    vec3 rockHi   = vec3(0.30, 0.27, 0.24);
    vec3 snow     = vec3(0.95, 0.97, 1.00);
    // Vegetated band with internal variety: meadow vs grass vs forest patches,
    // plus dry/dirt streaks.
    vec3 veg = mix(grass, meadow, smoothstep(0.45, 0.62, patches));
    veg = mix(veg, woods, smoothstep(0.52, 0.66, forest) * smoothstep(0.55, 0.8, slope));
    veg = mix(veg, dry, smoothstep(0.62, 0.78, micro) * 0.5);
    veg = mix(veg, dirt, smoothstep(0.72, 0.5, slope) * smoothstep(0.4, 0.7, fine) * 0.4);
    vec3 col = sand;
    col = mix(wetSand, sand, smoothstep(-2.0, 1.4, h));
    col = mix(col, veg, smoothstep(2.0, 9.0, hn));
    col = mix(col, mix(rock, rockHi, micro), smoothstep(70.0, 150.0, hn));
    // Steep faces are bare rock at any altitude (cliffs/crags).
    float steep = smoothstep(0.84, 0.5, slope);
    col = mix(col, mix(rock, rockHi, micro), steep * smoothstep(4.0, 12.0, h));
    // Snow as crisp caps high on the peaks (over the now-darker rock), more on
    // flatter shelves than sheer faces, with a scalloped (noisy) snow line.
    float snowLine = smoothstep(250.0, 330.0, hn + 34.0 * macro + 18.0 * fine)
                   * smoothstep(0.40, 0.66, slope);
    col = mix(col, snow, clamp(snowLine, 0.0, 1.0));
    col *= 0.82 + 0.34 * macro;
    col *= 0.90 + 0.18 * micro;
    // Worked pale ground around the pad.
    float padZone = smoothstep(240.0, 95.0, length(q));
    col = mix(col, vec3(0.40, 0.40, 0.41), padZone * 0.55);
    // Aerial perspective: distant terrain hazes toward a cool sky tone.
    float fogT = smoothstep(1400.0, 7000.0, length(vViewPosition));
    col = mix(col, vec3(0.62, 0.70, 0.80), fogT * 0.55);
    diffuseColor.rgb = col;
  }
`;

// Concrete apron: panel grid, expansion joints, scorch radiating from
// the flame trench, weathering stains.
function concreteTexture() {
  const size = 1024;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  const noise = makeNoise(11);
  ctx.fillStyle = "#9a9c9e";
  ctx.fillRect(0, 0, size, size);
  for (let i = 0; i < 9000; i += 1) {
    const x = Math.random() * size;
    const y = Math.random() * size;
    const v = 140 + Math.random() * 40;
    ctx.fillStyle = `rgba(${v},${v},${v + 4},0.12)`;
    ctx.fillRect(x, y, 2.5, 2.5);
  }
  ctx.strokeStyle = "rgba(60,62,66,0.55)";
  ctx.lineWidth = 2;
  for (let g = 0; g <= size; g += 128) {
    ctx.beginPath(); ctx.moveTo(g, 0); ctx.lineTo(g, size); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, g); ctx.lineTo(size, g); ctx.stroke();
  }
  // Scorch fan around the centre (trench axis along v).
  const scorch = ctx.createRadialGradient(size / 2, size / 2, 24, size / 2, size / 2, 330);
  scorch.addColorStop(0, "rgba(28,26,24,0.88)");
  scorch.addColorStop(0.35, "rgba(48,44,40,0.45)");
  scorch.addColorStop(1, "rgba(60,56,50,0)");
  ctx.fillStyle = scorch;
  ctx.fillRect(0, 0, size, size);
  for (let i = 0; i < 60; i += 1) {
    const a = noise(i * 0.7, 3.3) * Math.PI * 2;
    const r = 120 + noise(i * 0.41, 7.7) * 320;
    ctx.fillStyle = `rgba(40,38,34,${0.05 + noise(i, 1) * 0.1})`;
    ctx.beginPath();
    ctx.ellipse(size / 2 + Math.cos(a) * r, size / 2 + Math.sin(a) * r, 36, 14, a, 0, 7);
    ctx.fill();
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 8;
  return texture;
}

// Island ground cover: sand ring, scrub, dry grass, access road.
function islandTexture(heightAt, halfExtent) {
  const size = 1024;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  const noise = makeNoise(29);
  const image = ctx.createImageData(size, size);
  const px = image.data;
  for (let j = 0; j < size; j += 1) {
    for (let i = 0; i < size; i += 1) {
      const wx = (i / size - 0.5) * 2 * halfExtent;
      const wy = (0.5 - j / size) * 2 * halfExtent;
      const h = heightAt(wx, wy);
      const n = noise(wx * 0.01, wy * 0.01);
      const detail = noise(wx * 0.07, wy * 0.07);
      let r;
      let g;
      let b;
      if (h < 0.4) {
        r = 196 + detail * 26; g = 182 + detail * 22; b = 148 + detail * 16; // beach sand
      } else if (h < 3.2) {
        r = 168 + detail * 22; g = 158 + detail * 26; b = 122 + detail * 14; // dry flats
      } else {
        const scrub = 0.45 + 0.55 * noise(wx * 0.025 + 9, wy * 0.025);
        r = 112 + scrub * 38 + detail * 12;
        g = 116 + scrub * 46 + detail * 12;
        b = 84 + scrub * 26 + detail * 8;
      }
      // Pale worked ground near the complex, dark access road south.
      const d = Math.hypot(wx, wy);
      if (d < 220) { const t = 1 - d / 220; r += 60 * t; g += 58 * t; b += 56 * t; }
      if (Math.abs(wx) < 7 && wy < -90 && wy > -halfExtent) { r = 70 + n * 12; g = 70 + n * 12; b = 72 + n * 12; }
      const k = 4 * (j * size + i);
      px[k] = Math.min(r, 255); px[k + 1] = Math.min(g, 255); px[k + 2] = Math.min(b, 255); px[k + 3] = 255;
    }
  }
  ctx.putImageData(image, 0, 0);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 8;
  return texture;
}

// Terrain micro-relief normal map: high-frequency dune/scrub bumps so the
// ground catches directional light between the coarse mesh vertices instead
// of reading as flat faceted polygons.
function terrainNormalTexture(halfExtent) {
  const size = 512;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  const n1 = makeNoise(41);
  const n2 = makeNoise(53);
  const bump = (wx, wy) =>
    n1(wx * 0.06, wy * 0.06) * 1.0 + n2(wx * 0.21, wy * 0.21) * 0.4;
  const image = ctx.createImageData(size, size);
  const px = image.data;
  const span = (2 * halfExtent) / size;
  const strength = 1.4;
  for (let j = 0; j < size; j += 1) {
    for (let i = 0; i < size; i += 1) {
      const wx = (i / size - 0.5) * 2 * halfExtent;
      const wy = (0.5 - j / size) * 2 * halfExtent;
      const hL = bump(wx - span, wy);
      const hR = bump(wx + span, wy);
      const hD = bump(wx, wy - span);
      const hU = bump(wx, wy + span);
      let nx = (hL - hR) * strength;
      let ny = (hD - hU) * strength;
      const nz = 1.0;
      const inv = 1 / Math.hypot(nx, ny, nz);
      nx *= inv; ny *= inv;
      const k = 4 * (j * size + i);
      px[k] = (nx * 0.5 + 0.5) * 255;
      px[k + 1] = (ny * 0.5 + 0.5) * 255;
      px[k + 2] = nz * inv * 255;
      px[k + 3] = 255;
    }
  }
  ctx.putImageData(image, 0, 0);
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.RepeatWrapping;
  return texture;
}

// Tower steel: ribbed panels; masts get aviation paint bands.
function steelTexture(bands) {
  const canvas = document.createElement("canvas");
  canvas.width = 64;
  canvas.height = 512;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "#566073";
  ctx.fillRect(0, 0, 64, 512);
  ctx.fillStyle = "rgba(28,32,40,0.5)";
  for (let y = 0; y < 512; y += 24) ctx.fillRect(0, y, 64, 3);
  if (bands) {
    for (let y = 0; y < 512; y += 128) {
      ctx.fillStyle = y % 256 === 0 ? "#c3402f" : "#e8e6e0";
      ctx.fillRect(0, y, 64, 64);
    }
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

// A single box member spanning two 3-D points (for lattice trusses), its
// local +z aligned to the member axis.
const _strutA = new THREE.Vector3();
const _strutB = new THREE.Vector3();
const _strutZ = new THREE.Vector3(0, 0, 1);
function strut(group, mat, ax, ay, az, bx, by, bz, thick) {
  const dx = bx - ax;
  const dy = by - ay;
  const dz = bz - az;
  const len = Math.hypot(dx, dy, dz);
  const m = new THREE.Mesh(new THREE.BoxGeometry(thick, thick, len), mat);
  m.position.set((ax + bx) / 2, (ay + by) / 2, (az + bz) / 2);
  m.quaternion.setFromUnitVectors(_strutZ, _strutA.set(dx, dy, dz).normalize());
  group.add(m);
  return m;
}

// Tapered square lattice mast along +z (chords + per-bay diagonals + ties);
// used for the transporter-erector and the lightning-protection towers.
function latticeMast(height, baseW, topW, segs, mat) {
  const g = new THREE.Group();
  const corners = [[1, 1], [1, -1], [-1, -1], [-1, 1]];
  const wAt = (z) => (baseW + (topW - baseW) * (z / height)) / 2;
  for (const [sx, sy] of corners) {
    strut(g, mat, sx * wAt(0), sy * wAt(0), 0, sx * wAt(height), sy * wAt(height), height, 0.34);
  }
  const bay = height / segs;
  for (let s = 0; s < segs; s += 1) {
    const z0 = s * bay;
    const z1 = (s + 1) * bay;
    const w0 = wAt(z0);
    const w1 = wAt(z1);
    for (let c = 0; c < 4; c += 1) {
      const a = corners[c];
      const b = corners[(c + 1) % 4];
      strut(g, mat, a[0] * w1, a[1] * w1, z1, b[0] * w1, b[1] * w1, z1, 0.22); // tie
      strut(g, mat, a[0] * w0, a[1] * w0, z0, b[0] * w1, b[1] * w1, z1, 0.18); // diagonal
    }
  }
  return g;
}

// Tangent-ogive nose profile for a LatheGeometry.
function noseProfile(radius, length, segments = 14) {
  const rho = (radius * radius + length * length) / (2 * radius);
  const points = [];
  for (let i = 0; i <= segments; i += 1) {
    const y = (i / segments) * length;
    // Base (full radius) at y = 0 joins the hull; tip at y = length.
    const r = Math.sqrt(Math.max(rho * rho - y * y, 0)) + radius - rho;
    points.push(new THREE.Vector2(Math.max(r, 0.001), y));
  }
  return points;
}

// Engine bell profile.
function bellProfile(throat, exit, length, segments = 10) {
  const points = [];
  for (let i = 0; i <= segments; i += 1) {
    const t = i / segments;
    const r = throat + (exit - throat) * Math.pow(t, 0.7);
    points.push(new THREE.Vector2(r, -t * length));
  }
  return points;
}

// ----------------------------------------------------------------- scene

export class AscentScene {
  constructor(canvas, layout) {
    this.layout = layout;
    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      logarithmicDepthBuffer: true,
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.75));
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.18;
    // Sun shadows ground the launch complex (vehicle + towers cast onto the
    // pad). The vehicle stack is always at world origin, so the shadow
    // frustum stays centred there; the pad is near origin through liftoff.
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x000002);
    this.camera = new THREE.PerspectiveCamera(52, 1, 0.5, 90_000_000);
    this.cameraMode = "chase";
    this.smooth = { position: new THREE.Vector3(0, -120, 60), target: new THREE.Vector3() };
    this.lastElapsed = 0;
    // Global engine-FX brightness multiplier (plume core, glare, flare).
    // 0.70 chosen from a candidate sweep judged against real chase-cam
    // footage: a bright structured orange→white core with a tasteful bloom
    // halo, while the rocket body, ground and horizon stay legible. 1.0
    // overexposes the engine cluster into a white smear.
    this.fxIntensity = 0.7;
    this.tmp = {
      q: new THREE.Quaternion(),
      q2: new THREE.Quaternion(),
      v: new THREE.Vector3(),
      v2: new THREE.Vector3(),
      v3: new THREE.Vector3(),
      m: new THREE.Matrix4(),
      offset: new THREE.Vector3(),
    };

    // The scenario places the BODY ORIGIN at the sphere radius (sea
    // level); the engines hang `padDrop` below it. The visual sea is
    // lowered by the same amount so the launch site stands on ground at
    // engine level — a few metres of planet radius is invisible at
    // every other scale.
    this.padDrop = Math.min(...layout.engines.map((e) => e.mount_point_body_m[2]), 0);

    this.composer = new EffectComposer(this.renderer);
    this.composer.addPass(new RenderPass(this.scene, this.camera));
    this.bloom = new UnrealBloomPass(new THREE.Vector2(960, 540), 0.55, 0.55, 0.85);
    this.composer.addPass(this.bloom);
    this.composer.addPass(new OutputPass());
    // Give each ping-pong target its own DepthTexture so the atmosphere
    // pass can read the exact scene depth RenderPass wrote this frame.
    this.attachDepthTextures();

    this.buildLights();
    this.atmoLUTs = buildAtmosphereLUTs(this.renderer);
    this.buildSky();
    this.buildEarth();
    this.buildPad();
    this.buildVehicles();
    this.buildAtmospherePass();
    this.buildFlarePass();
    this.resize();
  }

  // Attach a private DepthTexture to each EffectComposer ping-pong target.
  // EffectComposer.setSize() resizes attached depth textures, so resize()
  // needs no special handling beyond the existing composer.setSize().
  attachDepthTextures() {
    const size = this.renderer.getDrawingBufferSize(new THREE.Vector2());
    for (const rt of [this.composer.renderTarget1, this.composer.renderTarget2]) {
      if (rt.depthTexture) rt.depthTexture.dispose();
      rt.depthTexture = new THREE.DepthTexture(
        Math.max(1, size.x),
        Math.max(1, size.y),
        THREE.UnsignedIntType,
      );
    }
  }

  // Insert the physically-based atmosphere raymarch between RenderPass
  // and the bloom pass: [RenderPass, atmosphere, bloom, OutputPass].
  buildAtmospherePass() {
    this.atmoPass = new AtmospherePass(this.atmoLUTs);
    this.composer.passes.splice(1, 0, this.atmoPass);
  }

  // Insert the screen-space flare/god-ray pass after the atmosphere pass
  // and before bloom: [RenderPass, atmosphere, flare, bloom, OutputPass]
  // so the flare is bloomed. Reads colour only (no scene depth).
  buildFlarePass() {
    this.flarePass = new FlarePass();
    const bloomIndex = this.composer.passes.indexOf(this.bloom);
    this.composer.passes.splice(bloomIndex, 0, this.flarePass);
    const size = this.renderer.getDrawingBufferSize(new THREE.Vector2());
    this.flarePass.setSize(size.x, size.y);
  }

  buildLights() {
    this.sun = new THREE.DirectionalLight(0xfff4e6, 3.6);
    this.sun.position.copy(SUN_DIRECTION).multiplyScalar(900);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    // Orthographic shadow frustum framing the launch complex around the
    // world origin (where the vehicle always sits).
    const sc = this.sun.shadow.camera;
    sc.left = -360;
    sc.right = 360;
    sc.top = 360;
    sc.bottom = -360;
    sc.near = 200;
    sc.far = 1700;
    sc.updateProjectionMatrix();
    this.sun.shadow.bias = -0.0006;
    this.sun.shadow.normalBias = 0.8;
    this.scene.add(this.sun.target); // target defaults to origin
    // Cool space-bounce fill that follows the camera so the dark side
    // of the hull still reads; a faint warm earthshine from below.
    this.fill = new THREE.DirectionalLight(0x9db8d8, 0.5);
    this.earthshine = new THREE.DirectionalLight(0x5577aa, 0.35);
    // Sky/ground hemisphere fill so shadowed faces at the launch complex
    // catch soft daylight (sky blue from above, warm sand bounce from
    // below) instead of crushing to flat black — grounds the structures.
    this.skyFill = new THREE.HemisphereLight(0xa6c8ee, 0x6f5d44, 0.45);
    this.scene.add(
      this.sun,
      this.fill,
      this.earthshine,
      this.skyFill,
      new THREE.AmbientLight(0x1c2433, 0.4),
    );

    // The sun disc is rendered analytically by the atmosphere pass
    // (transmittance-reddened at the horizon), so no billboard here.
  }

  buildSky() {
    const starCount = 15000;
    const bandCount = 11000;
    const total = starCount + bandCount;
    const positions = new Float32Array(total * 3);
    const colors = new Float32Array(total * 3);
    const sizes = new Float32Array(total);
    let seed = 0x5048414c;
    const rand = () => {
      seed = (seed * 1664525 + 1013904223) >>> 0;
      return seed / 0xffffffff;
    };
    const place = (index, direction, brightness, temperature) => {
      positions[3 * index] = 7e7 * direction.x;
      positions[3 * index + 1] = 7e7 * direction.y;
      positions[3 * index + 2] = 7e7 * direction.z;
      const warm = new THREE.Color().setHSL(
        temperature < 0.5 ? 0.62 : 0.10,
        0.35 * Math.abs(temperature - 0.5) * 2,
        0.75,
      );
      colors[3 * index] = warm.r * brightness;
      colors[3 * index + 1] = warm.g * brightness;
      colors[3 * index + 2] = warm.b * brightness;
      sizes[index] = 1 + brightness * 2.2;
    };
    const v = new THREE.Vector3();
    const bandQ = new THREE.Quaternion().setFromEuler(new THREE.Euler(1.05, 0.2, 0.5));
    for (let i = 0; i < starCount; i += 1) {
      const u = 2 * rand() - 1;
      const phi = 2 * Math.PI * rand();
      const r = Math.sqrt(1 - u * u);
      v.set(r * Math.cos(phi), r * Math.sin(phi), u);
      place(i, v, Math.pow(rand(), 3.2) * (rand() < 0.01 ? 2.4 : 1.0), rand());
    }
    for (let i = 0; i < bandCount; i += 1) {
      const phi = 2 * Math.PI * rand();
      const lat = (rand() + rand() + rand() + rand() - 2) * 0.16;
      v.set(Math.cos(lat) * Math.cos(phi), Math.cos(lat) * Math.sin(phi), Math.sin(lat));
      v.applyQuaternion(bandQ);
      place(starCount + i, v, Math.pow(rand(), 2.4) * 0.4, rand());
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    geometry.setAttribute("aSize", new THREE.BufferAttribute(sizes, 1));
    const stars = new THREE.Points(
        geometry,
        new THREE.ShaderMaterial({
          depthWrite: false,
          transparent: true,
          vertexColors: true,
          vertexShader: `
            attribute float aSize;
            varying vec3 vColor;
            #include <common>
          #include <logdepthbuf_pars_vertex>
            void main() {
              vColor = color;
              gl_PointSize = aSize;
              gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
              #include <logdepthbuf_vertex>
            }`,
          fragmentShader: `
            varying vec3 vColor;
            #include <common>
          #include <logdepthbuf_pars_fragment>
            void main() {
              #include <logdepthbuf_fragment>
              vec2 c = gl_PointCoord - 0.5;
              float a = smoothstep(0.5, 0.08, length(c));
              gl_FragColor = vec4(vColor, a);
            }`,
        }),
      );
    stars.renderOrder = -2;
    this.scene.add(stars);
  }


  buildEarth() {
    const loader = new THREE.TextureLoader();
    const load = (path, srgb = true) => {
      const texture = loader.load(path);
      if (srgb) texture.colorSpace = THREE.SRGBColorSpace;
      texture.anisotropy = 8;
      return texture;
    };
    const dayMap = load("assets/earth_day_5400.jpg");
    const nightMap = load("assets/earth_night_3600.jpg");
    // The raw cloud-fraction map is mid-grey almost everywhere; push it
    // through a contrast curve so the deck reads as broken cloud fields
    // instead of uniform haze.
    const cloudMap = new THREE.TextureLoader().load("assets/clouds_2048.jpg", (texture) => {
      const image = texture.image;
      const canvas = document.createElement("canvas");
      canvas.width = image.width;
      canvas.height = image.height;
      const ctx = canvas.getContext("2d");
      ctx.drawImage(image, 0, 0);
      const data = ctx.getImageData(0, 0, canvas.width, canvas.height);
      const px = data.data;
      for (let i = 0; i < px.length; i += 4) {
        const t = px[i] / 255;
        const curved = Math.max(0, Math.min(1, (t - 0.32) / (0.78 - 0.32)));
        const a = Math.round(curved * curved * (3 - 2 * curved) * 255);
        px[i] = a; px[i + 1] = a; px[i + 2] = a;
      }
      ctx.putImageData(data, 0, 0);
      texture.image = canvas;
      texture.needsUpdate = true;
    });

    // Cloud-shadow registration: the cloud shell spins at earthAngle*0.12
    // relative to the surface (see update()), so the surface samples the
    // cloud map shifted by that longitude fraction to cast soft shadows.
    this.earthUniforms = {
      dayMap: { value: dayMap },
      nightMap: { value: nightMap },
      cloudMap: { value: cloudMap },
      sunDirection: { value: SUN_DIRECTION },
      uCloudOffset: { value: 0 },
    };
    this.earth = new THREE.Mesh(
      new THREE.SphereGeometry(EARTH_RADIUS_M + this.padDrop, 160, 100),
      new THREE.ShaderMaterial({
        uniforms: this.earthUniforms,
        vertexShader: `
          varying vec2 vUv;
          varying vec3 vWorldNormal;
          varying vec3 vWorldPosition;
          #include <common>
          #include <logdepthbuf_pars_vertex>
          void main() {
            vUv = uv;
            vWorldNormal = normalize(mat3(modelMatrix) * normal);
            vec4 world = modelMatrix * vec4(position, 1.0);
            vWorldPosition = world.xyz;
            gl_Position = projectionMatrix * viewMatrix * world;
            #include <logdepthbuf_vertex>
          }`,
        // Albedo + day/night + ocean glint + cloud shadow ONLY. All
        // atmospheric colour (blue sky, lit limb, terminator tint, aerial
        // haze) is produced by the unified atmosphere pass — the old fake
        // Rayleigh rim is gone so it is not double-counted.
        fragmentShader: `
          uniform sampler2D dayMap;
          uniform sampler2D nightMap;
          uniform sampler2D cloudMap;
          uniform vec3 sunDirection;
          uniform float uCloudOffset;
          varying vec2 vUv;
          varying vec3 vWorldNormal;
          varying vec3 vWorldPosition;
          #include <common>
          #include <logdepthbuf_pars_fragment>
          void main() {
            #include <logdepthbuf_fragment>
            vec3 N = normalize(vWorldNormal);
            vec3 V = normalize(cameraPosition - vWorldPosition);
            float sunDot = dot(N, sunDirection);
            float daylight = smoothstep(-0.05, 0.05, sunDot);

            vec3 day = texture2D(dayMap, vUv).rgb;
            vec3 night = texture2D(nightMap, vUv).rgb;

            // Soft cloud shadow: the cloud above this surface point sits at
            // texture-u shifted by the cloud's relative spin.
            float cloud = texture2D(cloudMap, vec2(vUv.x - uCloudOffset, vUv.y)).r;
            float shadow = max(1.0 - cloud, 0.25);

            // Ocean glint: blue-dominant water mask, tight clamped hotspot.
            float ocean = smoothstep(0.02, 0.18, day.b - max(day.r, day.g) + 0.06);
            vec3 H = normalize(sunDirection + V);
            float spec = pow(max(dot(N, H), 0.0), 180.0) * ocean * daylight;

            vec3 lit = day * (0.04 + max(sunDot, 0.0)) * shadow;
            vec3 cities = night * vec3(1.25, 1.05, 0.78) * 1.6 * (1.0 - daylight);

            vec3 color = lit + cities + vec3(1.0, 0.97, 0.88) * spec * 0.6;
            gl_FragColor = vec4(color, 1.0);
          }`,
      }),
    );
    this.earth.rotation.y = -Math.PI / 2;

    // Sun-lit, terminator-aware cloud shell at a realistic ~10 km so the
    // cloud limb agrees with the atmosphere limb from orbit. Part of the
    // scene (tDiffuse), so the atmosphere pass applies aerial perspective
    // to it.
    this.clouds = new THREE.Mesh(
      new THREE.SphereGeometry(EARTH_RADIUS_M * 1.0016, 120, 80),
      new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: false,
        side: THREE.FrontSide,
        uniforms: {
          cloudMap: { value: cloudMap },
          sunDirection: { value: SUN_DIRECTION },
          // Faded out when the camera is below/at the ~10 km cloud deck:
          // from inside the shell the sphere renders as a dark tangent-ring
          // band across the sky. Clouds only read correctly seen from above.
          uCloudFade: { value: 0 },
        },
        vertexShader: `
          varying vec2 vUv;
          varying vec3 vWorldNormal;
          #include <common>
          #include <logdepthbuf_pars_vertex>
          void main() {
            vUv = uv;
            vWorldNormal = normalize(mat3(modelMatrix) * normal);
            vec4 world = modelMatrix * vec4(position, 1.0);
            gl_Position = projectionMatrix * viewMatrix * world;
            #include <logdepthbuf_vertex>
          }`,
        fragmentShader: `
          uniform sampler2D cloudMap;
          uniform vec3 sunDirection;
          uniform float uCloudFade;
          varying vec2 vUv;
          varying vec3 vWorldNormal;
          #include <common>
          #include <logdepthbuf_pars_fragment>
          void main() {
            #include <logdepthbuf_fragment>
            float cover = texture2D(cloudMap, vUv).r;
            float lit = 1.0 / (1.0 + exp(-18.0 * dot(normalize(vWorldNormal), sunDirection)));
            gl_FragColor = vec4(vec3(lit), cover * lit * 0.9 * uCloudFade);
          }`,
      }),
    );
    this.cloudMaterial = this.clouds.material;
    this.clouds.rotation.y = -Math.PI / 2;

    this.earthGroup = new THREE.Group();
    this.earthGroup.add(this.earth, this.clouds);
    this.scene.add(this.earthGroup);
  }

  // Coastal ocean around the launch island: a procedural water surface with
  // animated ripples, sun glint, fresnel sky tint and distance haze. Output
  // is linear HDR (ACES is applied once at OutputPass), and it includes the
  // logarithmic-depth chunks like every other custom material here.
  buildOcean(half) {
    const size = Math.max(half * 14, 30000); // reach toward the hazed horizon
    const geo = new THREE.PlaneGeometry(size, size, 1, 1);
    this.oceanUniforms = {
      uTime: { value: 0 },
      uSunDir: { value: SUN_DIRECTION.clone() },
      uDeep: { value: new THREE.Color(0x06222e) },
      uShallow: { value: new THREE.Color(0x14586a) },
      uSky: { value: new THREE.Color(0x8fbbe6) },
    };
    const mat = new THREE.ShaderMaterial({
      uniforms: this.oceanUniforms,
      vertexShader: `
        varying vec3 vWorldPosition;
        varying vec2 vLocalPos;
        varying vec3 vN0;
        varying vec3 vTx;
        varying vec3 vTy;
        #include <common>
        #include <logdepthbuf_pars_vertex>
        void main() {
          vLocalPos = position.xy;
          vec4 wp = modelMatrix * vec4(position, 1.0);
          vWorldPosition = wp.xyz;
          // normalMatrix is only available in the vertex stage; pass the
          // (constant, flat plane) world basis to the fragment as varyings.
          vN0 = normalize(normalMatrix * vec3(0.0, 0.0, 1.0));
          vTx = normalize(normalMatrix * vec3(1.0, 0.0, 0.0));
          vTy = normalize(normalMatrix * vec3(0.0, 1.0, 0.0));
          gl_Position = projectionMatrix * viewMatrix * wp;
          #include <logdepthbuf_vertex>
        }`,
      fragmentShader: `
        uniform float uTime;
        uniform vec3 uSunDir;
        uniform vec3 uDeep;
        uniform vec3 uShallow;
        uniform vec3 uSky;
        varying vec3 vWorldPosition;
        varying vec2 vLocalPos;
        varying vec3 vN0;
        varying vec3 vTx;
        varying vec3 vTy;
        #include <common>
        #include <logdepthbuf_pars_fragment>
        float hash(vec2 p){ return fract(sin(dot(p, vec2(127.1,311.7))) * 43758.5453); }
        float vnoise(vec2 p){
          vec2 i = floor(p), f = fract(p); f = f*f*(3.0-2.0*f);
          float a = hash(i), b = hash(i+vec2(1.0,0.0)), c = hash(i+vec2(0.0,1.0)), d = hash(i+vec2(1.0,1.0));
          return mix(mix(a,b,f.x), mix(c,d,f.x), f.y);
        }
        float fbmHelper(vec2 p){ float s=0.0, a=0.5; for(int i=0;i<5;i++){ s+=a*vnoise(p); p*=2.03; a*=0.5; } return s; }
        float waves(vec2 p, vec2 flow){ return fbmHelper(p+flow) + 0.5*fbmHelper(p*3.1 - flow*1.3); }
${COAST_GLSL}
        void main() {
          #include <logdepthbuf_fragment>
          vec3 N0 = normalize(vN0);
          vec3 Tx = normalize(vTx);
          vec3 Ty = normalize(vTy);
          vec3 toCam = cameraPosition - vWorldPosition;
          float dist = length(toCam);
          vec3 V = toCam / max(dist, 1e-4);
          vec2 p = vLocalPos * 0.05;
          vec2 flow = vec2(uTime*0.06, uTime*0.04);
          float e = 0.6;
          float h  = waves(p, flow);
          float hx = waves(p+vec2(e,0.0), flow);
          float hy = waves(p+vec2(0.0,e), flow);
          float atten = 1.0 / (1.0 + dist*0.0016);
          float amp = 1.1 * atten;
          vec3 N = normalize(N0 - Tx*(hx-h)*amp - Ty*(hy-h)*amp);
          float ndv = max(dot(N, V), 0.0);
          float fres = mix(0.02, 1.0, pow(1.0 - ndv, 5.0));
          vec3 H = normalize(V + uSunDir);
          float spec = pow(max(dot(N,H),0.0), 240.0) * 80.0 * atten; // tight glint
          float sheen = pow(max(dot(N,H),0.0), 22.0) * 1.6 * atten;  // broad sun sheen
          float sunUp = max(dot(N0, uSunDir), 0.0);
          // Distance offshore from the actual (shared) coastline.
          vec2 cp = vLocalPos - COAST_CENTER;
          float dc = length(cp);
          float shoreR = coastR(atan(cp.y, cp.x));
          float off = dc - shoreR;                 // >0 = open water beyond the shore
          float shallow = 1.0 - smoothstep(20.0, 540.0, off);
          vec3 water = mix(uDeep, uShallow, ndv*0.5 + shallow*0.5);
          vec3 col = mix(water, uSky, fres);
          col += vec3(1.0,0.93,0.76) * (spec + sheen) * sunUp;
          // Surf: a broad noisy foam band hugging the shoreline plus a broken,
          // high-frequency breaking-wave crest right at the water's edge.
          float band = smoothstep(150.0, 0.0, abs(off));
          float churn = 0.4 + 0.6 * fbmHelper(vLocalPos * 0.05 + flow * 3.0);
          float foam = band * churn;
          float crestN = fbmHelper(vLocalPos * 0.22 + flow * 6.0);
          float breakLine = smoothstep(28.0, 0.0, abs(off - 16.0))
                          * smoothstep(0.38, 0.66, crestN);
          foam = clamp(max(foam * 0.7, breakLine), 0.0, 1.0);
          col = mix(col, vec3(0.95, 0.975, 1.0), foam * shallow);
          float haze = clamp((dist-2500.0)/22000.0, 0.0, 1.0);
          col = mix(col, uSky*0.92, haze*0.75);
          col *= (0.5 + 0.7*sunUp);
          gl_FragColor = vec4(col, 1.0);
        }`,
    });
    const ocean = new THREE.Mesh(geo, mat);
    ocean.position.z = -1.4; // sea level, just below the beach band
    ocean.renderOrder = -1;
    this.ocean = ocean;
    this.pad.add(ocean);
  }

  // Launch complex anchored to the rotating Earth: a procedural launch
  // island (flattened plateau, scrub, beach ring), textured concrete
  // apron with flame trench, strongback, painted lightning masts, tank
  // farm, floodlit steam pool.
  buildPad() {
    this.pad = new THREE.Group();
    // ---- terrain: a coastal landscape ----
    // A big island whose centre sits inland (COAST_CENTER) so the pad lands on
    // a graded coastal shelf near the south shore with open water in front of
    // it, while the land rises through rolling hills into an inland mountain
    // range behind. Elevation is the geometry; colour is a per-pixel biome
    // shader (see TERRAIN_BIOME_GLSL).
    const HALF = 4200;
    const noise = makeNoise(7);
    const ridged = (a, b) => {
      const n = 1 - Math.abs(2 * noise(a, b) - 1);
      return n * n;
    };
    const smooth = THREE.MathUtils.smoothstep;
    const lerp = THREE.MathUtils.lerp;
    const heightAt = (x, y) => {
      const dx = x - COAST_CENTER[0];
      const dy = y - COAST_CENTER[1];
      const dc = Math.hypot(dx, dy);
      const inward = coastR(Math.atan2(dy, dx)) - dc; // >0 inside the island
      const island = smooth(inward, -40, 170);
      // domain-warped rolling hills
      const wx = x + 150 * (noise(x * 0.0009 + 11, y * 0.0009) - 0.5);
      const wy = y + 150 * (noise(x * 0.0009, y * 0.0009 + 7) - 0.5);
      const hills = noise(wx * 0.0016, wy * 0.0016) * 0.55
        + noise(wx * 0.0042, wy * 0.0042) * 0.3
        + noise(wx * 0.012, wy * 0.012) * 0.15;
      const hillAmp = 95 * smooth(inward, 60, 1300);
      // ridged inland mountain range — multi-octave for sharper crests/spurs
      const mtn = ridged(wx * 0.0011 + 3, wy * 0.0011) * 0.58
        + ridged(wx * 0.0026, wy * 0.0026 + 5) * 0.28
        + ridged(wx * 0.0058, wy * 0.0058 + 9) * 0.14;
      const mtnAmp = 560 * smooth(inward, 850, 2500);
      const land = 2.0 + hills * hillAmp + mtn * mtnAmp;
      const seaFloor = -10 - 24 * smooth(-inward, 0, 700);
      let h = lerp(seaFloor, land, island);
      // graded pad shelf around the launch complex
      const grade = smooth(Math.hypot(x, y), 150, 380);
      h = lerp(1.7, h, grade);
      return h;
    };
    const terrainGeometry = new THREE.PlaneGeometry(2 * HALF, 2 * HALF, 400, 400);
    const positions = terrainGeometry.attributes.position;
    for (let i = 0; i < positions.count; i += 1) {
      positions.setZ(i, heightAt(positions.getX(i), positions.getY(i)) - 1.2);
    }
    terrainGeometry.computeVertexNormals();
    const terrainNormal = terrainNormalTexture(HALF);
    terrainNormal.repeat.set(120, 120);
    const terrainMat = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      normalMap: terrainNormal,
      normalScale: new THREE.Vector2(0.6, 0.6),
      roughness: 0.97,
      metalness: 0,
    });
    terrainMat.onBeforeCompile = (shader) => {
      shader.vertexShader = "varying vec3 vTerr;\nvarying vec3 vTerrN;\n"
        + shader.vertexShader
          .replace("#include <beginnormal_vertex>", "#include <beginnormal_vertex>\n  vTerrN = objectNormal;")
          .replace("#include <begin_vertex>", "#include <begin_vertex>\n  vTerr = transformed;");
      shader.fragmentShader = "varying vec3 vTerr;\nvarying vec3 vTerrN;\n"
        + TERRAIN_NOISE_GLSL
        + shader.fragmentShader
          .replace("#include <color_fragment>", "#include <color_fragment>\n" + TERRAIN_BIOME_GLSL);
    };
    const terrain = new THREE.Mesh(terrainGeometry, terrainMat);
    terrain.receiveShadow = true;
    this.pad.add(terrain);
    this.buildOcean(HALF);

    // ---- vegetation: an instanced conifer forest on the lower/mid slopes ----
    // Clustered into woods by a low-frequency mask, kept off the beaches, the
    // pad, and steep rock. One InstancedMesh = one draw call.
    const treeGeo = new THREE.ConeGeometry(3.4, 12, 6);
    treeGeo.translate(0, 6, 0);
    treeGeo.rotateX(Math.PI / 2); // base on the ground, apex up (+z)
    const treeMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.92, metalness: 0 });
    const MAX_TREES = 2400;
    const trees = new THREE.InstancedMesh(treeGeo, treeMat, MAX_TREES);
    trees.castShadow = false;
    trees.receiveShadow = false;
    const dummy = new THREE.Object3D();
    const tcol = new THREE.Color();
    let seed = 1337;
    const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
    let nt = 0;
    for (let i = 0; i < MAX_TREES * 5 && nt < MAX_TREES; i += 1) {
      const x = (rnd() * 2 - 1) * HALF * 0.72;
      const y = (rnd() * 2 - 1) * HALF * 0.72;
      if (Math.hypot(x, y) < 230) continue; // clear of the complex
      const h = heightAt(x, y);
      if (h < 6 || h > 120) continue; // above beach, below the rock line
      const e = 4;
      const gx = (heightAt(x + e, y) - heightAt(x - e, y)) / (2 * e);
      const gy = (heightAt(x, y + e) - heightAt(x, y - e)) / (2 * e);
      const nz = 1 / Math.sqrt(gx * gx + gy * gy + 1);
      if (nz < 0.84) continue; // too steep
      const forest = noise(x * 0.0021 + 17, y * 0.0021)
        + 0.35 * (noise(x * 0.008 + 3, y * 0.008) - 0.5); // broken grove edges
      // Cluster into coherent woods; only the occasional lone tree on open land.
      if (forest < 0.5 && rnd() > 0.06) continue;
      // Thin the treeline out as it approaches the rock/snow.
      if (rnd() > 1 - THREE.MathUtils.smoothstep(h, 70, 110)) continue;
      const sc = 0.5 + rnd() * 1.5;
      dummy.position.set(x, y, h - 1.2);
      dummy.rotation.set(0, 0, rnd() * Math.PI * 2);
      dummy.scale.set(sc * (0.8 + rnd() * 0.5), sc * (0.8 + rnd() * 0.5), sc * (0.8 + rnd() * 0.7));
      dummy.updateMatrix();
      trees.setMatrixAt(nt, dummy.matrix);
      // Hue/value variety: dark conifers through lighter younger greens.
      const v = rnd();
      tcol.setRGB(0.04 + 0.10 * v, 0.10 + 0.16 * rnd(), 0.04 + 0.06 * v);
      trees.setColorAt(nt, tcol);
      nt += 1;
    }
    trees.count = nt;
    trees.instanceMatrix.needsUpdate = true;
    if (trees.instanceColor) trees.instanceColor.needsUpdate = true;
    this.pad.add(trees);

    // ---- materials ----
    const steel = new THREE.MeshStandardMaterial({ map: steelTexture(false), roughness: 0.5, metalness: 0.6 });
    const paintedSteel = new THREE.MeshStandardMaterial({ map: steelTexture(true), roughness: 0.55, metalness: 0.35 });
    const trussMat = new THREE.MeshStandardMaterial({ color: 0x97a0ac, roughness: 0.48, metalness: 0.66 });
    const darkMetal = new THREE.MeshStandardMaterial({ color: 0x20242b, roughness: 0.72, metalness: 0.4 });
    const tankWhite = new THREE.MeshStandardMaterial({ color: 0xe9ebee, roughness: 0.35, metalness: 0.1 });
    const concrete = new THREE.MeshStandardMaterial({ map: concreteTexture(), roughness: 0.9, metalness: 0 });
    const roadMat = new THREE.MeshStandardMaterial({ color: 0x3a3b3f, roughness: 0.92, metalness: 0 });
    const mats = { steel, paintedSteel, trussMat, darkMetal, tankWhite, concrete, roadMat };
    this.trussMat = trussMat;

    // ---- concrete apron, recessed flame trench + deflector, launch table ----
    const apron = new THREE.Mesh(new THREE.CylinderGeometry(58, 61, 1.6, 56), concrete);
    apron.rotation.x = Math.PI / 2;
    apron.position.z = 0.4;
    apron.receiveShadow = true;
    this.pad.add(apron);
    // Recessed flame trench (a dark pit) running south along -y.
    const trench = new THREE.Mesh(new THREE.BoxGeometry(15, 80, 13), darkMetal);
    trench.position.set(0, -10, -6.6);
    this.pad.add(trench);
    // Angled flame deflector wedge kicking the exhaust down the trench.
    const deflector = new THREE.Mesh(
      new THREE.BoxGeometry(13, 24, 2.6),
      new THREE.MeshStandardMaterial({ color: 0x2a2d33, roughness: 0.85, metalness: 0.2 }),
    );
    deflector.position.set(0, -3, -2.6);
    deflector.rotation.x = -Math.PI * 0.17;
    this.pad.add(deflector);
    // Launch table: two concrete shoulders flanking the trench opening.
    for (const sx of [-1, 1]) {
      const tbl = new THREE.Mesh(new THREE.BoxGeometry(20, 26, 4.4), concrete);
      tbl.position.set(sx * 17, -4, 1.6);
      this.pad.add(tbl);
    }
    // Steel launch ring + hold-down clamps at the vehicle base.
    const ring = new THREE.Mesh(new THREE.CylinderGeometry(5.6, 6.4, 2.4, 28, 1, true), steel);
    ring.rotation.x = Math.PI / 2;
    ring.position.z = 1.2;
    this.pad.add(ring);
    for (let m = 0; m < 8; m += 1) {
      const a = (m / 8) * Math.PI * 2;
      const clamp = new THREE.Mesh(new THREE.BoxGeometry(0.8, 1.0, 3.4), steel);
      clamp.position.set(Math.cos(a) * 4.7, Math.sin(a) * 4.7, 1.7);
      clamp.rotation.z = a;
      this.pad.add(clamp);
    }

    this.buildStrongback(mats);
    this.buildLightningTowers(mats);
    this.buildFacilities(mats);

    // Solid pad hardware casts and receives sun shadows; terrain
    // (receive-only), ocean (neither) and thin truss members (trussMat,
    // negligible shadow) are excluded to keep the shadow pass light.
    this.pad.traverse((o) => {
      if (o.isMesh && o !== terrain && o !== this.ocean && o.material !== trussMat) {
        o.castShadow = true;
        o.receiveShadow = true;
      }
    });

    this.scene.add(this.pad);

    // Liftoff steam pool: recycled billboards fed while thrust washes
    // over the pad.
    this.steamTexture = radialSpriteTexture([
      [0, "rgba(235,238,242,0.55)"],
      [0.5, "rgba(225,229,235,0.30)"],
      [1, "rgba(220,224,230,0)"],
    ], 256);
    this.steam = [];
    for (let i = 0; i < 42; i += 1) {
      const sprite = new THREE.Sprite(
        new THREE.SpriteMaterial({
          map: this.steamTexture,
          transparent: true,
          depthWrite: false,
          opacity: 0,
        }),
      );
      sprite.userData = { age: 1e9, life: 1, drift: new THREE.Vector3() };
      this.pad.add(sprite);
      this.steam.push(sprite);
    }
    this.steamClock = 0;
  }

  // Transporter-erector (strongback): a lattice beam standing alongside the
  // vehicle with cradle arms and an umbilical run, hinged at the base.
  buildStrongback({ steel, trussMat, darkMetal }) {
    const te = new THREE.Group();
    const hinge = new THREE.Mesh(new THREE.BoxGeometry(5.5, 5.5, 3.2), darkMetal);
    hinge.position.set(0, -8.5, 1.6);
    te.add(hinge);
    const beam = latticeMast(74, 3.8, 2.8, 12, trussMat);
    beam.position.set(0, -8.5, 2.8);
    te.add(beam);
    // Cradle arms + clamp heads reaching toward the vehicle.
    for (const z of [13, 31, 49, 65]) {
      const arm = new THREE.Mesh(new THREE.BoxGeometry(2.4, 6.8, 1.3), steel);
      arm.position.set(0, -4.6, z);
      te.add(arm);
      const head = new THREE.Mesh(new THREE.BoxGeometry(3.6, 1.6, 2.2), steel);
      head.position.set(0, -1.7, z);
      te.add(head);
    }
    // Umbilical / cable tray up the face of the beam.
    const tray = new THREE.Mesh(new THREE.CylinderGeometry(0.4, 0.4, 70, 8), darkMetal);
    tray.rotation.x = Math.PI / 2;
    tray.position.set(1.6, -9.6, 37);
    te.add(tray);
    this.pad.add(te);
  }

  // Three tall lattice lightning-protection towers ringing the pad, with a
  // sagging catenary cable strung between their fibreglass mast tips.
  buildLightningTowers({ trussMat, paintedSteel }) {
    const tops = [];
    for (let i = 0; i < 3; i += 1) {
      const a = Math.PI / 2 + i * ((Math.PI * 2) / 3);
      const x = Math.cos(a) * 60;
      const y = Math.sin(a) * 60;
      const tower = latticeMast(116, 7.5, 1.8, 12, trussMat);
      tower.position.set(x, y, 0);
      this.pad.add(tower);
      const tip = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.7, 24, 8), paintedSteel);
      tip.rotation.x = Math.PI / 2;
      tip.position.set(x, y, 116 + 12);
      this.pad.add(tip);
      tops.push(new THREE.Vector3(x, y, 116 + 24));
    }
    const wireMat = new THREE.MeshStandardMaterial({ color: 0x12161c, roughness: 0.7, metalness: 0.3 });
    for (let i = 0; i < 3; i += 1) {
      const a = tops[i];
      const b = tops[(i + 1) % 3];
      const mid = a.clone().add(b).multiplyScalar(0.5);
      mid.z -= 28; // catenary sag
      const curve = new THREE.QuadraticBezierCurve3(a, mid, b);
      const tube = new THREE.Mesh(new THREE.TubeGeometry(curve, 24, 0.14, 6, false), wireMat);
      this.pad.add(tube);
    }
  }

  // Support facilities: propellant tank farm, integration hangar, blockhouse,
  // deluge water tower, floodlight stanchions, and the road/causeway network.
  buildFacilities({ steel, tankWhite, darkMetal, concrete, roadMat }) {
    // ---- road network + access causeway (scale + connective tissue) ----
    // Perimeter ring road around the apron.
    const ring = new THREE.Mesh(new THREE.RingGeometry(70, 78, 64), roadMat);
    ring.position.z = 0.72;
    this.pad.add(ring);
    // Straight access roads from the ring out to the hangar and tank farm.
    const roadTo = (x, y, w) => {
      const len = Math.hypot(x, y) - 74;
      const r = new THREE.Mesh(new THREE.PlaneGeometry(w, len), roadMat);
      r.position.set(x / 2 * (1 - 74 / Math.hypot(x, y)), y / 2 * (1 - 74 / Math.hypot(x, y)), 0.72);
      r.rotation.z = Math.atan2(y, x) - Math.PI / 2;
      this.pad.add(r);
    };
    roadTo(168, 88, 12); // to the hangar
    roadTo(100, -42, 8); // to the tank farm
    roadTo(-128, 74, 8); // to the blockhouse
    // Access causeway striking out across the water toward the mainland.
    const causeway = new THREE.Mesh(new THREE.BoxGeometry(18, 1300, 2), roadMat);
    causeway.position.set(-180, -640, 0.2);
    causeway.rotation.z = 0.32;
    this.pad.add(causeway);
    // Spherical LOX tank on a skirt + horizontal RP-1 tanks with saddles.
    const sphere = new THREE.Mesh(new THREE.SphereGeometry(9, 28, 20), tankWhite);
    sphere.position.set(100, -42, 11);
    this.pad.add(sphere);
    const skirt = new THREE.Mesh(new THREE.CylinderGeometry(7.2, 7.6, 5, 22, 1, true), steel);
    skirt.rotation.x = Math.PI / 2;
    skirt.position.set(100, -42, 2.5);
    this.pad.add(skirt);
    for (let t = 0; t < 3; t += 1) {
      const tank = new THREE.Mesh(new THREE.CylinderGeometry(3, 3, 21, 22), tankWhite);
      tank.rotation.z = Math.PI / 2;
      tank.position.set(82 + t * 10, -88, 3.6);
      this.pad.add(tank);
      for (const sy of [-7.5, 7.5]) {
        const saddle = new THREE.Mesh(new THREE.BoxGeometry(2, 1.6, 3.2), steel);
        saddle.position.set(82 + t * 10, -88 + sy, 1.6);
        this.pad.add(saddle);
      }
    }
    const pipe = new THREE.Mesh(new THREE.BoxGeometry(1.0, 64, 1.0), steel);
    pipe.position.set(86, -36, 1.2);
    this.pad.add(pipe);

    // Shared building trim materials.
    const roofMat = new THREE.MeshStandardMaterial({ color: 0x60666e, roughness: 0.6, metalness: 0.3 });
    const glassMat = new THREE.MeshStandardMaterial({ color: 0x1a2c3a, roughness: 0.18, metalness: 0.6 });
    const doorMat = new THREE.MeshStandardMaterial({ color: 0x4a4f57, roughness: 0.7, metalness: 0.25 });
    const trimMat = new THREE.MeshStandardMaterial({ color: 0xcfd3d8, roughness: 0.6, metalness: 0.15 });
    // A building with an overhanging parapet roof cap, a window band, and a
    // big contrasting roll-up door on the pad-facing (−y) end.
    const makeBuilding = (cx, cy, w, d, h, rot, bodyMat, doorW) => {
      const g = new THREE.Group();
      const body = new THREE.Mesh(new THREE.BoxGeometry(w, d, h), bodyMat);
      body.position.z = h / 2;
      g.add(body);
      const cap = new THREE.Mesh(new THREE.BoxGeometry(w + 1.6, d + 1.6, 1.6), roofMat);
      cap.position.z = h + 0.4;
      g.add(cap);
      const win = new THREE.Mesh(new THREE.BoxGeometry(w + 0.12, d + 0.12, h * 0.16), glassMat);
      win.position.z = h * 0.62;
      g.add(win);
      const door = new THREE.Mesh(new THREE.BoxGeometry(doorW, 0.5, h * 0.66), doorMat);
      door.position.set(0, -d / 2 - 0.25, (h * 0.66) / 2);
      g.add(door);
      const frame = new THREE.Mesh(new THREE.BoxGeometry(doorW + 1.4, 0.45, h * 0.66 + 1.2), trimMat);
      frame.position.set(0, -d / 2 - 0.18, (h * 0.66 + 1.2) / 2);
      g.add(frame);
      g.position.set(cx, cy, 0);
      g.rotation.z = rot;
      this.pad.add(g);
      return g;
    };
    // Horizontal integration hangar (HIF) and reinforced control blockhouse.
    makeBuilding(168, 88, 74, 26, 17, 0.32,
      new THREE.MeshStandardMaterial({ color: 0xc3c7cd, roughness: 0.68, metalness: 0.14 }), 20);
    makeBuilding(-128, 74, 26, 18, 9, -0.2, concrete, 5);

    // Deluge water tower on legs.
    const wt = new THREE.Mesh(new THREE.SphereGeometry(6.5, 20, 16), tankWhite);
    wt.position.set(-66, -78, 42);
    this.pad.add(wt);
    for (let l = 0; l < 4; l += 1) {
      const a = (l / 4) * Math.PI * 2 + 0.6;
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.9, 40), steel);
      leg.position.set(-66 + Math.cos(a) * 4.4, -78 + Math.sin(a) * 4.4, 20);
      this.pad.add(leg);
    }

    // Floodlight stanchions ringing the apron.
    for (const [x, y] of [[46, 30], [-48, 28], [44, -36], [-44, -34], [12, 50]]) {
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.4, 0.55, 32, 8), steel);
      pole.rotation.x = Math.PI / 2;
      pole.position.set(x, y, 16);
      this.pad.add(pole);
      const lamp = new THREE.Mesh(new THREE.BoxGeometry(3.2, 1.3, 0.9), darkMetal);
      lamp.position.set(x, y, 31.5);
      this.pad.add(lamp);
    }
  }

  buildVehicles() {
    const { bodies, engines } = this.layout;
    const lowestMount = Math.min(...engines.map((e) => e.mount_point_body_m[2]), 0);
    // The nose cone belongs on the last full-diameter body (the fairing
    // when one is declared); smaller bodies after it (payloads) ride
    // hidden inside the fairing until the simulation separates them.
    const maxDiameter = Math.max(...bodies.map((b) => b.diameter_m));
    let noseIndex = bodies.length - 1;
    for (let i = bodies.length - 1; i >= 0; i -= 1) {
      if (bodies[i].diameter_m >= 0.95 * maxDiameter) {
        noseIndex = i;
        break;
      }
    }

    this.stack = new THREE.Group();
    this.scene.add(this.stack);
    this.bodyMeshes = new Map();
    this.gridFins = [];
    // The top full-diameter body is the vacuum (upper) stage; its engine
    // gets the big MVac-class nozzle and an aft engine bay. The bottom body
    // is the booster that occludes that bay until it separates.
    this.upperBodyId = bodies[noseIndex]?.id;
    this.boosterBodyId = bodies[0]?.id;

    const darkMaterial = new THREE.MeshStandardMaterial({
      color: 0x191c20,
      roughness: 0.62,
      metalness: 0.35,
    });
    // Machined engine-section grey for the upper-stage thrust structure —
    // dark and matte so it does not wash out to a pale "lampshade" cone.
    const engineBayMaterial = new THREE.MeshStandardMaterial({
      color: 0x32373f,
      roughness: 0.6,
      metalness: 0.35,
    });

    let z = lowestMount;
    let nestedBase = 0;
    bodies.forEach((body, index) => {
      const group = new THREE.Group();
      const radius = body.diameter_m / 2;
      const isTop = index === noseIndex;
      const isNested = index > noseIndex;
      const skin = new THREE.MeshStandardMaterial({
        map: hullTexture({ aftBand: index === 0, wordmark: true }),
        roughness: 0.46,
        metalness: 0.1,
      });

      const hull = new THREE.Mesh(
        new THREE.CylinderGeometry(radius, radius, body.length_m, 48, 1, isTop && !isNested),
        skin,
      );
      hull.rotation.x = Math.PI / 2;
      hull.position.z = (isNested ? nestedBase : z) + body.length_m / 2;
      group.add(hull);
      if (isNested) {
        // Stowed payload: co-located inside the fairing, revealed when
        // the simulation separates it into its own lane.
        group.visible = false;
        this.bodyMeshes.set(body.id, group);
        this.stack.add(group);
        return;
      }
      nestedBase = z + 1.0;

      if (isTop) {
        const nose = new THREE.Mesh(
          new THREE.LatheGeometry(noseProfile(radius, radius * 3.0), 48),
          skin,
        );
        nose.rotation.x = Math.PI / 2;
        nose.position.z = z + body.length_m;
        group.add(nose);

        // Aft engine bay: a tapered thrust structure bridging the tank base
        // (full diameter) down to the engine mount, so the vacuum nozzle
        // reads as attached to the stage instead of floating below it in the
        // void left when the booster + interstage separate. The bay nests
        // inside the booster/interstage and is occluded until separation.
        const aftEngine = engines.find((e) => e.mounted_to === body.id);
        if (aftEngine) {
          const engZ = aftEngine.mount_point_body_m[2];
          const bayTop = z - 0.2; // just under the tank base
          // A substantial boattail that stays chunky down to the engine
          // (not a thin "neck") so the nozzle reads as part of a coherent
          // engine section rather than a bulb hung on a stalk. Its wide
          // mouth sleeves the nozzle throat, hiding the join.
          const bayBottom = engZ - 0.2;
          const bayRBottom = 0.82;
          const bayLen = Math.max(bayTop - bayBottom, 0.5);
          const bay = new THREE.Mesh(
            new THREE.CylinderGeometry(radius * 0.96, bayRBottom, bayLen, 48, 1, true),
            engineBayMaterial,
          );
          bay.rotation.x = Math.PI / 2;
          bay.position.z = (bayTop + bayBottom) / 2;
          group.add(bay);
          // A short aft dome closing the tank base so the bay does not look
          // like an open can when viewed from below near separation.
          const dome = new THREE.Mesh(
            new THREE.SphereGeometry(radius * 0.97, 36, 12, 0, Math.PI * 2, Math.PI * 0.5, Math.PI * 0.5),
            engineBayMaterial,
          );
          dome.rotation.x = Math.PI / 2;
          dome.position.z = bayTop;
          group.add(dome);
          // Structural collar bands around the engine bay give it panel-line
          // detail and self-shadowing instead of a bare smooth cone. Each
          // ring hugs the cone radius at its station.
          const rAt = (zz) => {
            const f = (bayTop - zz) / bayLen;
            return radius * 0.96 + (bayRBottom - radius * 0.96) * f;
          };
          for (const f of [0.26, 0.58, 0.86]) {
            const zc = bayTop - bayLen * f;
            const rc = rAt(zc);
            const collar = new THREE.Mesh(
              new THREE.CylinderGeometry(rc + 0.06, rc + 0.06, 0.18, 36, 1, true),
              darkMaterial,
            );
            collar.rotation.x = Math.PI / 2;
            collar.position.z = zc;
            group.add(collar);
          }
        }
      } else {
        const interstage = new THREE.Mesh(
          new THREE.CylinderGeometry(radius * 1.004, radius * 1.004, 2.6, 48, 1, true),
          darkMaterial,
        );
        interstage.rotation.x = Math.PI / 2;
        interstage.position.z = z + body.length_m - 1.3;
        group.add(interstage);

        // Folded grid fins at the top of the booster; they deploy when
        // the body separates.
        for (let f = 0; f < 4; f += 1) {
          const fin = new THREE.Group();
          const panel = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.16, 2.1), darkMaterial);
          const lattice = new THREE.Mesh(
            new THREE.BoxGeometry(1.3, 0.22, 0.18), darkMaterial,
          );
          const lattice2 = lattice.clone();
          lattice.position.z = 0.5;
          lattice2.position.z = -0.5;
          panel.position.z = -1.05;
          lattice.position.z = -0.55;
          lattice2.position.z = -1.55;
          fin.add(panel, lattice, lattice2);
          const angle = (f / 4) * Math.PI * 2 + Math.PI / 4;
          fin.position.set(
            Math.cos(angle) * (radius + 0.1),
            Math.sin(angle) * (radius + 0.1),
            z + body.length_m - 2.4,
          );
          fin.rotation.z = angle;
          fin.userData.deploy = 0;
          group.add(fin);
          this.gridFins.push(fin);
        }

        // Octaweb skirt ribs.
        for (let r = 0; r < 8; r += 1) {
          const rib = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.5, 3.2), darkMaterial);
          const angle = (r / 8) * Math.PI * 2;
          rib.position.set(
            Math.cos(angle) * radius * 0.99,
            Math.sin(angle) * radius * 0.99,
            z + 1.6,
          );
          rib.rotation.z = angle;
          group.add(rib);
        }

        // Folded landing legs along the aft skirt.
        for (let l = 0; l < 4; l += 1) {
          const leg = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.22, 11), darkMaterial);
          const angle = (l / 4) * Math.PI * 2;
          leg.position.set(
            Math.cos(angle) * (radius + 0.13),
            Math.sin(angle) * (radius + 0.13),
            z + 6.4,
          );
          leg.rotation.z = angle;
          group.add(leg);
        }
      }
      this.bodyMeshes.set(body.id, group);
      this.stack.add(group);
      z += body.length_m;
    });

    this.buildPlumes();
    this.laneGroups = new Map();

    // One hot point light per flying group lights the hull and pad.
    this.stackLight = new THREE.PointLight(0xffb070, 0, 400, 1.6);
    this.stackLight.position.set(0, 0, lowestMount - 6);
    this.stack.add(this.stackLight);

    // Transonic condensation collar (Prandtl-Glauert): a soft white shell
    // that blooms around the vehicle ONLY while crossing Mach 1 low in the
    // atmosphere. Driven by uOpacity in updatePlumes from Mach + altitude;
    // never appears in vacuum.
    const vlen = z - lowestMount;
    this.vaporCone = new THREE.Mesh(
      new THREE.ConeGeometry(maxDiameter * 1.7, vlen * 0.7, 40, 1, true),
      new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: false,
        side: THREE.DoubleSide,
        uniforms: { uOpacity: { value: 0 }, uTime: { value: 0 } },
        vertexShader: `
          varying vec2 vUv;
          varying vec3 vN;
          varying vec3 vWP;
          #include <common>
          #include <logdepthbuf_pars_vertex>
          void main() {
            vUv = uv;
            vN = normalize(mat3(modelMatrix) * normal);
            vec4 wp = modelMatrix * vec4(position, 1.0);
            vWP = wp.xyz;
            gl_Position = projectionMatrix * viewMatrix * wp;
            #include <logdepthbuf_vertex>
          }`,
        fragmentShader: `
          uniform float uOpacity;
          uniform float uTime;
          varying vec2 vUv;
          varying vec3 vN;
          varying vec3 vWP;
          ${PLUME_NOISE}
          #include <common>
          #include <logdepthbuf_pars_fragment>
          void main() {
            #include <logdepthbuf_fragment>
            vec3 V = normalize(cameraPosition - vWP);
            float fres = pow(1.0 - abs(dot(normalize(vN), V)), 1.5);
            float n = fbm3(vec3(vUv * 6.0, uTime * 0.6));
            float band = smoothstep(0.0, 0.25, vUv.y) * smoothstep(1.0, 0.55, vUv.y);
            float a = uOpacity * fres * (0.45 + 0.55 * n) * band;
            gl_FragColor = vec4(vec3(0.95, 0.97, 1.0), a);
          }`,
      }),
    );
    this.vaporCone.rotation.x = -Math.PI / 2; // cone axis +y -> body +z (apex toward nose)
    this.vaporCone.position.z = lowestMount + vlen * 0.6;
    this.vaporCone.visible = false;
    this.stack.add(this.vaporCone);

    this.buildGhost(lowestMount, z, maxDiameter * 0.5);
  }

  // Translucent "ghost" of the vehicle drawn at the onboard EKF's ESTIMATED
  // pose (vs the truth body at the world origin), plus a position-covariance
  // ellipsoid. It hugs the real vehicle while the estimate is good and
  // visibly drifts when navigation degrades (e.g. a GNSS outage) — the SIL
  // truth-vs-onboard boundary made visceral. Off by default.
  buildGhost(z0, z1, rad) {
    this.ghostEnabled = false;
    this.ghost = new THREE.Group();
    const mat = new THREE.MeshBasicMaterial({
      color: 0x7fd4ff,
      transparent: true,
      opacity: 0.22,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    const body = new THREE.Mesh(new THREE.CylinderGeometry(rad, rad, z1 - z0, 16, 1, true), mat);
    body.rotation.x = Math.PI / 2;
    body.position.z = (z0 + z1) / 2;
    this.ghost.add(body);
    const nose = new THREE.Mesh(new THREE.ConeGeometry(rad, rad * 2.4, 16, 1, true), mat);
    nose.rotation.x = -Math.PI / 2;
    nose.position.z = z1 + rad * 1.2;
    this.ghost.add(nose);
    // covariance ellipsoid (1σ position uncertainty, exaggerated for sight)
    this.covSphere = new THREE.Mesh(
      new THREE.SphereGeometry(1, 20, 14),
      new THREE.MeshBasicMaterial({
        color: 0xffb86b,
        transparent: true,
        opacity: 0.16,
        depthWrite: false,
        wireframe: true,
      }),
    );
    this.covSphere.position.z = (z0 + z1) / 2;
    this.ghost.add(this.covSphere);
    this.ghost.visible = false;
    this.scene.add(this.ghost);
  }

  setGhost(on) {
    this.ghostEnabled = on;
    if (this.ghost) this.ghost.visible = on;
  }

  buildPlumes() {
    // Sea-level engines: short, dark, slightly machined metal nozzles
    // (toned down from a mirror finish so the rim does not blow out).
    const bellMaterialBase = new THREE.MeshStandardMaterial({
      color: 0x26292e,
      roughness: 0.46,
      metalness: 0.58,
    });
    // Vacuum (upper-stage) engine: a large, deep MVac-class nozzle with a
    // coppery niobium extension — warm and metallic, never a specular dot.
    const vacBellMaterialBase = new THREE.MeshStandardMaterial({
      color: 0x6a4a30,
      roughness: 0.55,
      metalness: 0.45,
      side: THREE.DoubleSide,
    });
    // Dark inner liner sits just inside the bell so looking into the mouth
    // shows an unmistakable dark cavity (the DoubleSide tan wall alone gets
    // filled by ambient and reads solid).
    const vacBellInnerMaterial = new THREE.MeshStandardMaterial({
      color: 0x140f0b,
      roughness: 0.85,
      metalness: 0.2,
      side: THREE.DoubleSide,
    });
    const bellGeometry = new THREE.LatheGeometry(bellProfile(0.28, 0.62, 1.5), 24);
    // Shorter, less bulbous MVac bell whose throat (0.42) emerges from the
    // chunky boattail mouth (0.82) — no thin-neck-then-bulb silhouette.
    const VAC_BELL_LEN = 3.4;
    const vacBellGeometry = new THREE.LatheGeometry(
      bellProfile(0.42, 1.06, VAC_BELL_LEN, 22),
      44,
    );
    const vacBellInnerGeometry = new THREE.LatheGeometry(
      bellProfile(0.36, 0.98, VAC_BELL_LEN - 0.12, 20),
      44,
    );
    const vacExitLipGeometry = new THREE.TorusGeometry(1.05, 0.05, 10, 44);

    // Incandescent core: a blackbody temperature gradient (hot white near
    // the throat → orange tip) with 3D turbulence and pressure-driven shock
    // diamonds; near-opaque in dense air, translucent in vacuum where it
    // shifts blue-violet. Normal-blended HDR so it composites over the hull
    // and blooms. uAmb = ambient pressure fraction (1 = sea level, 0 = vac).
    const corePlume = () =>
      new THREE.ShaderMaterial({
        transparent: true,
        blending: THREE.NormalBlending,
        depthWrite: false,
        uniforms: {
          uTime: { value: 0 },
          uIntensity: { value: 0 },
          uAmb: { value: 1 },
          uCoreHDR: { value: 6 },
        },
        vertexShader: `
          varying vec2 vUv;
          varying vec3 vPos;
          #include <common>
          #include <logdepthbuf_pars_vertex>
          void main() {
            vUv = uv;
            vPos = position;
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
            #include <logdepthbuf_vertex>
          }`,
        fragmentShader: `
          uniform float uTime;
          uniform float uIntensity;
          uniform float uAmb;
          uniform float uCoreHDR;
          varying vec2 vUv;
          varying vec3 vPos;
          ${PLUME_NOISE}
          #include <common>
          #include <logdepthbuf_pars_fragment>
          void main() {
            #include <logdepthbuf_fragment>
            float axial = 1.0 - vUv.y;            // 1 at nozzle, 0 at tip
            float radial = abs(vUv.x - 0.5) * 2.0;
            float vacuumness = 1.0 - uAmb;
            float n = fbm3(vPos * vec3(3.0, 2.2, 3.0) + vec3(0.0, -uTime * 6.0, 0.0));

            float Tref = 2800.0;
            float T = mix(1700.0, 3300.0, smoothstep(0.0, 0.5, axial)) * (0.85 + 0.3 * n);
            vec3 col = blackbody(T);
            // Vacuum recombination glow tints the tip blue-violet.
            col = mix(col, vec3(0.42, 0.50, 1.10), vacuumness * smoothstep(0.2, 1.0, 1.0 - axial) * 0.85);

            // Shock diamonds: bright periodic nodes, strong and tightly
            // spaced in dense air, gone in vacuum.
            float spacing = mix(26.0, 8.0, uAmb);
            float diamonds = clamp(uAmb * 1.4, 0.0, 1.0)
              * pow(abs(sin(axial * spacing + 0.6)), 8.0) * smoothstep(0.15, 0.85, axial);

            float bright = pow(T / Tref, 4.0);
            float shape = smoothstep(1.0, 0.22, radial) * smoothstep(0.0, 0.16, axial) * (0.7 + 0.5 * n);
            float alpha = shape * mix(0.20, 0.92, uAmb) * uIntensity;
            vec3 outc = col * bright * uCoreHDR * uIntensity * (1.0 + diamonds * 2.5);
            gl_FragColor = vec4(outc, alpha);
          }`,
      });

    // Outer expansion sheath: a wide faint additive glow. Tight and orange
    // in dense air; balloons huge and blue-violet in vacuum (underexpanded).
    const sheathPlume = () =>
      new THREE.ShaderMaterial({
        transparent: true,
        blending: THREE.AdditiveBlending,
        depthWrite: false,
        side: THREE.DoubleSide,
        uniforms: {
          uTime: { value: 0 },
          uIntensity: { value: 0 },
          uAmb: { value: 1 },
        },
        vertexShader: `
          varying vec2 vUv;
          varying vec3 vPos;
          #include <common>
          #include <logdepthbuf_pars_vertex>
          void main() {
            vUv = uv;
            vPos = position;
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
            #include <logdepthbuf_vertex>
          }`,
        fragmentShader: `
          uniform float uTime;
          uniform float uIntensity;
          uniform float uAmb;
          varying vec2 vUv;
          varying vec3 vPos;
          ${PLUME_NOISE}
          #include <common>
          #include <logdepthbuf_pars_fragment>
          void main() {
            #include <logdepthbuf_fragment>
            float axial = 1.0 - vUv.y;
            float radial = abs(vUv.x - 0.5) * 2.0;
            float vacuumness = 1.0 - uAmb;
            float n = fbm3(vPos * vec3(2.0, 1.5, 2.0) + vec3(0.0, -uTime * 4.0, 0.0));
            vec3 sea = vec3(0.95, 0.55, 0.22);
            vec3 vac = vec3(0.45, 0.55, 1.10);
            vec3 col = mix(vac, sea, uAmb);
            float body = smoothstep(0.02, 0.3, axial) * smoothstep(1.05, 0.40, axial)
              * smoothstep(1.0, 0.2, radial);
            float alpha = body * (0.10 + 0.13 * n) * uIntensity * (0.55 + 0.7 * vacuumness);
            gl_FragColor = vec4(col * (0.6 + 0.6 * vacuumness), alpha);
          }`,
      });

    // Engine glare: a camera-facing additive HDR quad at the nozzle exit.
    // depthTest true so the vehicle/terrain occlude it via the hardware
    // depth buffer — that free, correct occlusion is what the screen-space
    // flare pass inherits (an occluded glare simply isn't in the colour
    // buffer to flare). Lives in a non-rotated world-origin group and is
    // billboarded + positioned at each engine's world position per frame.
    this.glareGroup = new THREE.Group();
    this.scene.add(this.glareGroup);
    const glareGeometry = new THREE.PlaneGeometry(1, 1);
    const glareMaterial = () =>
      new THREE.ShaderMaterial({
        transparent: true,
        blending: THREE.AdditiveBlending,
        depthWrite: false,
        depthTest: true,
        uniforms: {
          uHDR: { value: 0 },
          uColor: { value: new THREE.Color(1.0, 0.82, 0.55) },
          uSpikes: { value: 1 },
        },
        vertexShader: `
          varying vec2 vUv;
          #include <common>
          #include <logdepthbuf_pars_vertex>
          void main() {
            vUv = uv - 0.5;
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
            #include <logdepthbuf_vertex>
          }`,
        fragmentShader: `
          uniform float uHDR;
          uniform float uSpikes;
          uniform vec3 uColor;
          varying vec2 vUv;
          #include <common>
          #include <logdepthbuf_pars_fragment>
          void main() {
            #include <logdepthbuf_fragment>
            float r = length(vUv) * 2.0;
            float core = exp(-r * r * 24.0);
            // Thin cross spikes (the broad anamorphic streak + ghosts are
            // added by the screen-space flare pass).
            float hx = exp(-vUv.y * vUv.y * 900.0) * exp(-vUv.x * vUv.x * 7.0);
            float vy = exp(-vUv.x * vUv.x * 900.0) * exp(-vUv.y * vUv.y * 7.0);
            float i = core + (hx + vy) * 0.35 * uSpikes;
            gl_FragColor = vec4(uColor * i * uHDR, clamp(i, 0.0, 1.0));
          }`,
      });

    this.plumes = this.layout.engines.map((engine) => {
      const isVac = engine.mounted_to === this.upperBodyId;
      const bellMaterial = (isVac ? vacBellMaterialBase : bellMaterialBase).clone();
      bellMaterial.emissive = new THREE.Color(0xff7733);
      bellMaterial.emissiveIntensity = 0;
      const bell = new THREE.Mesh(isVac ? vacBellGeometry : bellGeometry, bellMaterial);
      // Mouth points aft (−z, toward the exhaust); +π/2 puts the wide exit
      // below the throat at the mount.
      bell.rotation.x = Math.PI / 2;
      bell.position.set(...engine.mount_point_body_m);
      if (isVac) {
        // Dark interior liner + a defined exit-plane lip so the big bell
        // reads as a hollow cavity, not a solid cone. Children inherit the
        // bell's mount transform; geometry is in the same lathe (y-up) frame.
        const inner = new THREE.Mesh(vacBellInnerGeometry, vacBellInnerMaterial);
        bell.add(inner);
        const lip = new THREE.Mesh(vacExitLipGeometry, bellMaterial);
        lip.rotation.x = Math.PI / 2; // ring in the lathe revolution plane (axis = y)
        lip.position.y = -VAC_BELL_LEN; // exit plane
        bell.add(lip);
      }

      const core = new THREE.Mesh(
        new THREE.ConeGeometry(0.42, 1, 18, 6, true),
        corePlume(),
      );
      const sheath = new THREE.Mesh(
        new THREE.ConeGeometry(0.8, 1, 18, 4, true),
        sheathPlume(),
      );
      core.visible = false;
      sheath.visible = false;

      const mount = new THREE.Group();
      mount.position.set(...engine.mount_point_body_m);
      // Plume emerges from the nozzle exit plane: deep in the big vacuum
      // bell, just under the short sea-level nozzle.
      mount.position.z -= isVac ? VAC_BELL_LEN - 0.5 : 1.4;
      mount.add(core, sheath);

      const glare = new THREE.Mesh(glareGeometry, glareMaterial());
      glare.visible = false;
      glare.frustumCulled = false;
      this.glareGroup.add(glare);

      const owner = this.bodyMeshes.get(engine.mounted_to) ?? this.stack;
      owner.add(bell);
      this.stack.add(mount);
      return { engine, mount, core, sheath, bellMaterial, glare };
    });

    // Opaque vehicle hardware casts sun shadows onto the launch complex;
    // the plume/glare ShaderMaterials are skipped (transparent FX).
    this.stack.traverse((o) => {
      if (o.isMesh && o.material && o.material.isMeshStandardMaterial) {
        o.castShadow = true;
      }
    });
  }

  detachBody(bodyId) {
    if (this.laneGroups.has(bodyId)) return;
    const mesh = this.bodyMeshes.get(bodyId);
    if (!mesh) return;
    const lane = new THREE.Group();
    this.stack.remove(mesh);
    mesh.visible = true;
    lane.add(mesh);
    this.scene.add(lane);
    this.laneGroups.set(bodyId, lane);
    this.plumes
      .filter(({ engine }) => engine.mounted_to === bodyId)
      .forEach(({ mount }) => {
        this.stack.remove(mount);
        lane.add(mount);
      });
    const laneLight = new THREE.PointLight(0xffb070, 0, 320, 1.6);
    laneLight.position.copy(mesh.position).z -= 18;
    lane.add(laneLight);
    lane.userData.light = laneLight;
  }

  setCameraMode(mode) {
    this.cameraMode = mode;
  }

  update(view, elapsedS) {
    const dt = Math.min(Math.max(elapsedS - this.lastElapsed, 0.001), 0.1);
    this.lastElapsed = elapsedS;
    const { offset } = this.tmp;

    offset.set(view.position[0], view.position[1], view.position[2]);
    this.stack.position.set(0, 0, 0);
    this.stack.quaternion.set(...view.attitude);

    // Estimate-vs-truth ghost: truth vehicle sits at the world origin, so the
    // ghost goes at (estimate − truth) with the estimated attitude; the
    // covariance bubble is the EKF's reported position 1σ (exaggerated).
    if (this.ghostEnabled && this.ghost) {
      if (view.estimateValid && view.estimatePosition) {
        this.ghost.visible = true;
        this.ghost.position.set(
          view.estimatePosition[0] - view.position[0],
          view.estimatePosition[1] - view.position[1],
          view.estimatePosition[2] - view.position[2],
        );
        if (view.estimateAttitude) this.ghost.quaternion.set(...view.estimateAttitude);
        if (this.covSphere && view.positionVar) {
          const sigma = Math.sqrt(Math.max(0, ...view.positionVar));
          this.covSphere.scale.setScalar(Math.max(2, sigma * 14));
        }
      } else {
        this.ghost.visible = false;
      }
    }

    const earthAngle = OMEGA_EARTH_RAD_S * view.timeS;
    this.earthGroup.position.copy(offset).negate();
    this.earthGroup.rotation.z = earthAngle;
    this.clouds.rotation.z = earthAngle * 0.12; // slow relative drift
    this.earthUniforms.uCloudOffset.value = (earthAngle * 0.12) / (2.0 * Math.PI);

    this.updatePad(view, offset, earthAngle, dt);
    this.updateLanes(view, offset, dt);
    this.updatePlumes(view, elapsedS);
    this.updateCamera(view, offset, dt);

    // Feed the unified atmosphere pass. Planet center is at world -offset,
    // so the camera relative to the center (km) is (camera.position +
    // offset)/1000; the sun is a fixed world-space direction.
    // Fade the cloud shell in only once the camera climbs above the
    // ~10 km deck (below it, the shell renders as a dark band across the
    // sky from inside). Above ~45 km it reads as a deck over the surface.
    const cameraAltitude =
      this.tmp.v.copy(this.camera.position).add(offset).length() - EARTH_RADIUS_M;
    const cloudFade = THREE.MathUtils.clamp((cameraAltitude - 12_000) / 33_000, 0, 1);
    this.cloudMaterial.uniforms.uCloudFade.value = cloudFade;
    this.clouds.visible = cloudFade > 0.001;

    this.camera.updateMatrixWorld();
    const a = this.atmoPass.uniforms;
    a.uInvProj.value.copy(this.camera.projectionMatrixInverse);
    a.uInvView.value.copy(this.camera.matrixWorld);
    a.uCamPosKm.value.copy(this.camera.position).add(offset).multiplyScalar(0.001);
    a.uSunDirWorld.value.copy(SUN_DIRECTION);
    a.uLogDepthBufFC.value = 2.0 / Math.log2(this.camera.far + 1.0);

    this.fill.position.copy(this.camera.position).normalize();
    this.earthshine.position.copy(offset).normalize().negate();
    this.composer.render();
  }

  updatePad(view, offset, earthAngle, dt) {
    // Pad sits at lat 0, lon 0: ECI +x at t = 0, co-rotating since.
    const { v, q, v2, v3 } = this.tmp;
    const cos = Math.cos(earthAngle);
    const sin = Math.sin(earthAngle);
    v.set(EARTH_RADIUS_M * cos, EARTH_RADIUS_M * sin, 0).sub(offset);
    const altitude = offset.length() - EARTH_RADIUS_M;
    this.pad.visible = altitude < 60_000 && v.length() < 220_000;
    if (!this.pad.visible) return;
    if (this.oceanUniforms) this.oceanUniforms.uTime.value = this.lastElapsed;
    this.pad.position.copy(v).addScaledVector(v2.set(cos, sin, 0), this.padDrop);
    // Local frame: +z radial up, +y east(ish): rotate identity so that
    // body z aligns with the radial direction at the pad longitude.
    // (The from/to arguments must be DISTINCT vectors — aliasing one
    // temp through both .set() calls collapses the rotation to identity
    // and lays the whole complex on its side.)
    q.setFromUnitVectors(v3.set(0, 0, 1), v2.set(cos, sin, 0).normalize());
    this.pad.quaternion.copy(q);

    // Steam while the cluster fires near the ground.
    const thrustFraction = view.engines.reduce((sum, engine) => sum + engine.fraction, 0);
    const feeding = altitude < 220 && thrustFraction > 0.4;
    this.steamClock -= dt;
    for (const sprite of this.steam) {
      const data = sprite.userData;
      data.age += dt;
      if (feeding && this.steamClock <= 0 && data.age > data.life) {
        data.age = 0;
        data.life = 2.6 + Math.random() * 2.2;
        const angle = Math.random() * Math.PI * 2;
        const radial = 6 + Math.random() * 6;
        sprite.position.set(Math.cos(angle) * radial, Math.sin(angle) * radial, 2.5);
        data.drift.set(Math.cos(angle) * (14 + Math.random() * 16), Math.sin(angle) * (14 + Math.random() * 16), 3.5 + Math.random() * 5);
        sprite.scale.setScalar(6 + Math.random() * 6);
        this.steamClock = 0.02;
      }
      const t = data.age / data.life;
      if (t >= 1) {
        sprite.material.opacity = 0;
        continue;
      }
      sprite.position.addScaledVector(data.drift, dt);
      sprite.scale.addScalar(dt * 26);
      sprite.material.opacity = 0.5 * (1 - t) * (t < 0.12 ? t / 0.12 : 1);
    }
  }

  updateLanes(view, offset, dt) {
    for (const [bodyId, lane] of this.laneGroups) {
      const laneView = view.lanes.get(bodyId);
      if (!laneView) {
        lane.visible = false;
        continue;
      }
      lane.visible = true;
      lane.position.set(
        laneView.position[0] - offset.x,
        laneView.position[1] - offset.y,
        laneView.position[2] - offset.z,
      );
      lane.quaternion.set(...laneView.attitude);
    }
    // Grid fins deploy once their body flies free.
    for (const fin of this.gridFins) {
      const separated = !this.stack.children.includes(fin.parent) ? 1 : 0;
      fin.userData.deploy = THREE.MathUtils.lerp(
        fin.userData.deploy,
        separated || this.laneGroups.size > 0 ? 1 : 0,
        1 - Math.exp(-dt * 2.2),
      );
      fin.rotation.x = (-Math.PI / 2.2) * fin.userData.deploy;
    }
  }

  updatePlumes(view, elapsedS) {
    const amb = Math.min(view.pressurePa / SEA_LEVEL_PRESSURE_PA, 1);
    const vacuum = 1 - amb;
    // Vacuum underexpansion balloons the sheath; sea level keeps it tight.
    const expansion = THREE.MathUtils.lerp(1.05, 4.4, vacuum);
    const { q, v, v2 } = this.tmp;

    let stackGlow = 0;
    let laneGlow = 0;
    const wp = this.tmp.v3;
    this.flareSources = this.flareSources ?? [];
    this.flareSources.length = 0;
    for (let i = 0; i < this.plumes.length; i += 1) {
      const { core, sheath, bellMaterial, mount, glare } = this.plumes[i];
      const thrust = view.engines[i];
      const fraction = thrust.fraction;
      bellMaterial.emissiveIntensity = THREE.MathUtils.lerp(
        bellMaterial.emissiveIntensity,
        fraction > 0.02 ? 2.8 : 0,
        0.3,
      );
      if (fraction <= 0.005) {
        core.visible = false;
        sheath.visible = false;
        glare.visible = false;
        continue;
      }

      // Engine glare core: billboard at the nozzle exit (world position),
      // HDR ramped by throttle, dimmer/cooler toward vacuum. Project to
      // screen for the flare pass.
      mount.getWorldPosition(wp);
      glare.position.copy(wp);
      glare.quaternion.copy(this.camera.quaternion);
      const glareRadius = (1.1 + 0.8 * fraction) * (1.0 - 0.35 * vacuum);
      glare.scale.setScalar(glareRadius);
      glare.material.uniforms.uHDR.value =
        (0.6 + 1.6 * fraction) * (1.0 - 0.45 * vacuum) * this.fxIntensity;
      glare.material.uniforms.uColor.value.setRGB(
        1.0,
        0.82 - 0.06 * vacuum,
        0.55 + 0.32 * vacuum,
      );
      glare.visible = true;
      this.collectFlareSource(wp, fraction);
      if (mount.parent === this.stack) stackGlow += fraction;
      else laneGlow += fraction;

      // Sea level: long incandescent core, tight sheath, tight shock
      // diamonds. Vacuum: core shortens and cools while the sheath balloons
      // into a wide, faint blue-violet expansion bell.
      const coreLength = (6 + 24 * fraction) * (1 - 0.3 * vacuum);
      const sheathLength = coreLength * (1.3 + 1.9 * vacuum);
      core.visible = true;
      sheath.visible = true;
      core.scale.set(1.15, coreLength, 1.15);
      sheath.scale.set(expansion, sheathLength, expansion);

      v.set(-thrust.direction[0], -thrust.direction[1], -thrust.direction[2]).normalize();
      q.setFromUnitVectors(v2.set(0, 1, 0), v);
      const coreHDR = (1.5 + 4.0 * fraction) * this.fxIntensity;
      for (const part of [core, sheath]) {
        part.quaternion.copy(q);
        part.position.set(0, 0, 0);
        part.translateY(part === core ? coreLength / 2 : sheathLength / 2);
        part.material.uniforms.uTime.value = elapsedS;
        part.material.uniforms.uAmb.value = amb;
        part.material.uniforms.uIntensity.value = 0.55 + 0.45 * fraction;
      }
      core.material.uniforms.uCoreHDR.value = coreHDR;
    }
    // Engine-lit hull/ground: warm in air, cooler/dimmer toward vacuum.
    this.stackLight.intensity = Math.min(stackGlow, 9) * 90;
    this.stackLight.color.setRGB(1.0, 0.69 + 0.12 * vacuum, 0.44 + 0.4 * vacuum);
    for (const lane of this.laneGroups.values()) {
      if (lane.userData.light) {
        lane.userData.light.intensity = Math.min(laneGlow, 9) * 90;
      }
    }

    // Feed the screen-space flare/god-ray pass: the firing-engine cluster
    // centroid in screen UV, total intensity, and ambient pressure (so
    // god-rays vanish in vacuum). amb is read by the pass for the early-out.
    const f = this.flareSources;
    if (this.flarePass) {
      if (f.length === 0) {
        this.flarePass.uniforms.uIntensity.value = 0;
      } else {
        let cx = 0;
        let cy = 0;
        let sum = 0;
        for (const s of f) {
          cx += s.x * s.intensity;
          cy += s.y * s.intensity;
          sum += s.intensity;
        }
        this.flarePass.uniforms.uSource.value.set(cx / sum, cy / sum);
        this.flarePass.uniforms.uIntensity.value = Math.min(sum * 0.12, 0.35) * this.fxIntensity;
        this.flarePass.uniforms.uAmb.value = amb;
      }
    }

    // Transonic vapor collar: Mach ~ groundSpeed / 340 (sea-level approx,
    // not from the sim's atmosphere model), gated to the transonic band AND
    // low altitude so it never persists or shows in vacuum.
    if (this.vaporCone) {
      const mach = view.groundSpeed / 340.0;
      const vehAlt = Math.hypot(...view.position) - EARTH_RADIUS_M;
      const transonic =
        THREE.MathUtils.smoothstep(mach, 0.82, 0.96) *
        (1.0 - THREE.MathUtils.smoothstep(mach, 1.08, 1.5));
      const lowAir = 1.0 - THREE.MathUtils.smoothstep(vehAlt, 5000, 16000);
      const target = transonic * lowAir * 0.8;
      const u = this.vaporCone.material.uniforms;
      u.uOpacity.value += (target - u.uOpacity.value) * 0.12;
      u.uTime.value = elapsedS;
      this.vaporCone.visible = u.uOpacity.value > 0.01;
    }
  }

  // Project an engine's world position to screen UV for the flare pass.
  collectFlareSource(worldPos, intensity) {
    this._flareTmp = this._flareTmp ?? new THREE.Vector3();
    const ndc = this._flareTmp.copy(worldPos).project(this.camera);
    if (ndc.z > 1.0) return; // behind camera / beyond far plane
    this.flareSources.push({ x: ndc.x * 0.5 + 0.5, y: ndc.y * 0.5 + 0.5, intensity });
  }

  updateCamera(view, offset, dt) {
    // Cameras are anchored: the anchor (stack origin, booster lane, pad,
    // or orbit frame) is tracked EXACTLY each frame, and only the offset
    // from the anchor is damped. Smoothing absolute positions would lag
    // kilometres behind targets that move at km/s.
    const { v, v2, v3 } = this.tmp;
    const altitude = offset.length() - EARTH_RADIUS_M;
    const up = v2.copy(offset).normalize();
    const anchor = v3.set(0, 0, 0);
    const desired = v.set(0, 0, 0);
    let lookHeight = 26;
    let stiffness = 4.0;

    switch (this.cameraMode) {
      case "pad": {
        const angle = OMEGA_EARTH_RAD_S * view.timeS;
        anchor.set(
          EARTH_RADIUS_M * Math.cos(angle) - 250 * Math.sin(angle),
          EARTH_RADIUS_M * Math.sin(angle) + 250 * Math.cos(angle),
          26,
        ).sub(offset);
        desired.set(0, 0, 0);
        lookHeight = Math.min(altitude * 0.3, 80);
        stiffness = 1e3; // rigid mount
        break;
      }
      case "booster": {
        const lane = view.lanes.values().next().value;
        if (!lane) {
          this.cameraMode = "chase";
          this.updateCamera(view, offset, dt);
          return;
        }
        anchor.set(
          lane.position[0] - offset.x,
          lane.position[1] - offset.y,
          lane.position[2] - offset.z,
        );
        desired.copy(up).multiplyScalar(34);
        desired.x += 52;
        desired.y += 18;
        lookHeight = 0;
        break;
      }
      case "orbit": {
        // Astronaut's-window vista: sit just behind/above the vehicle and
        // look out along the LOCAL HORIZONTAL so the curved limb and its
        // thin atmosphere band stretch across the horizon (from low orbit
        // the planet is too large to frame as a disc).
        this.scratch = this.scratch ?? new THREE.Vector3();
        this.scratch2 = this.scratch2 ?? new THREE.Vector3();
        const vel = this.scratch2.set(view.velocity[0], view.velocity[1], view.velocity[2]);
        const fwd = this.scratch.copy(vel).addScaledVector(up, -vel.dot(up));
        if (fwd.lengthSq() < 1.0) fwd.set(0, 0, 1).cross(up);
        fwd.normalize();
        desired.copy(up).multiplyScalar(80).addScaledVector(fwd, -220);
        // Look far along the horizontal, dipped slightly below so the limb
        // sits a touch above centre with space above it.
        this.orbitLook = (this.orbitLook ?? new THREE.Vector3())
          .copy(fwd).multiplyScalar(1_500_000).addScaledVector(up, -120_000);
        stiffness = 2.2;
        break;
      }
      case "chase":
      default: {
        const velocity = v.set(view.velocity[0], view.velocity[1], view.velocity[2]);
        const back =
          velocity.lengthSq() > 25
            ? velocity.normalize().negate()
            : v.set(0, -1, 0).normalize();
        const distance = 95 + Math.min(altitude / 3000, 160);
        desired
          .copy(back)
          .multiplyScalar(distance)
          .addScaledVector(up, distance * 0.35);
        // Slight lateral slide keeps the plume off-axis and the frame alive.
        this.scratch = this.scratch ?? new THREE.Vector3();
        desired.addScaledVector(this.scratch.crossVectors(up, back).normalize(), distance * 0.35);
        lookHeight = 4;
        break;
      }
    }

    const blend = 1 - Math.exp(-dt * stiffness);
    this.smooth.position.lerp(desired, Math.min(blend, 1));
    this.camera.position.copy(anchor).add(this.smooth.position);
    this.camera.up.copy(up);
    this.lookPoint = this.lookPoint ?? new THREE.Vector3();
    this.lookPoint.copy(anchor);
    if (this.cameraMode === "pad") {
      // Pad camera tracks the vehicle (the world origin), aiming a few
      // metres up the stack along LOCAL up so the frame centres on the
      // hull at the pad and on the vehicle once it climbs out.
      this.lookPoint.copy(up).multiplyScalar(Math.min(18, 18 + altitude * 0.0));
    } else if (this.cameraMode === "orbit") {
      // Look out along the local horizontal toward the limb.
      this.lookPoint.copy(anchor).add(this.orbitLook);
    } else {
      this.lookPoint.addScaledVector(up, lookHeight);
    }
    this.camera.lookAt(this.lookPoint);
  }

  resize() {
    const canvas = this.renderer.domElement;
    const width = canvas.clientWidth || canvas.parentElement?.clientWidth || 960;
    const height = canvas.clientHeight || canvas.parentElement?.clientHeight || 540;
    this.renderer.setSize(width, height, false);
    this.composer.setSize(width, height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }
}
