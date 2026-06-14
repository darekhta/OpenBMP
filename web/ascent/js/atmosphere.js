// Physically-based atmosphere: a single full-screen raymarch pass with a
// Hillaire-2020 transmittance + multiple-scattering LUT pair, replacing
// the old ad-hoc sky dome and additive shells. One unified model gives a
// blue noon sky from the pad AND a thin blue limb from orbit, purely as a
// function of camera altitude and view direction — no altitude branch.
//
// Conventions: all scattering math is in KILOMETRES (so f32 magnitudes
// are ~6400, not 6.4e6). The scene's floating origin puts the planet
// CENTER at world -offset, so the camera position relative to the planet
// center (in km) is fed as uCamPosKm = (camera.position + offset)/1000.
// The sun is a fixed world-space direction; the LUTs are camera- and
// world-orientation-independent because the per-pixel march parameterises
// the sun angle against the LOCAL up at each sample (dot(normalize(pos),
// sunDir)) — that is what makes a single init-time LUT valid at all
// altitudes.
//
// Output is LINEAR HDR radiance: the pass runs between RenderPass and
// UnrealBloom inside the existing EffectComposer, and ACES tonemapping is
// applied once, last, by OutputPass. No tonemapping/colorspace chunks
// here.

import * as THREE from "three";
import { ShaderPass } from "../vendor/addons/postprocessing/ShaderPass.js";
import { FullScreenQuad } from "../vendor/addons/postprocessing/Pass.js";

// Earth atmosphere constants (km, km^-1). Rg/Rt are shifted up 10 km from
// the canonical 6360/6460 so the scattering ground sphere coincides with
// the rendered Earth sphere (EARTH_RADIUS_M = 6_370_000), avoiding a halo
// gap or surface poke-through at the limb.
export const ATMO = {
  Rg: 6370.0,
  Rt: 6470.0,
};

// Shared GLSL: constants, densities, phase functions, a numerically
// stable ray-sphere (perpendicular-distance form — the naive
// c = dot(o,o)-r*r cancels catastrophically at planet scale and
// mis-classifies the pad as underground), and the transmittance-LUT
// mapping shared by every shader so the LUT built with one mapping is
// sampled with the same one.
const SCATTER_GLSL = `
  #define PI 3.141592653589793
  uniform float uRg;
  uniform float uRt;
  const vec3  BETA_R = vec3(0.005802, 0.013558, 0.033100);
  const float BETA_M = 0.003996;
  const float BETA_M_EXT = 0.004440;
  const float MIE_G = 0.8;
  const vec3  OZONE_ABS = vec3(0.000650, 0.001881, 0.000085);
  const float HR = 8.0;
  const float HM = 1.2;

  // (Rayleigh, Mie, ozone) relative densities at altitude h (km).
  vec3 densities(float h) {
    return vec3(exp(-h / HR), exp(-h / HM), max(0.0, 1.0 - abs(h - 25.0) / 15.0));
  }
  vec3 extinction(float h) {
    vec3 d = densities(h);
    return BETA_R * d.x + vec3(BETA_M_EXT) * d.y + OZONE_ABS * d.z;
  }
  float rayleighPhase(float mu) { return 3.0 / (16.0 * PI) * (1.0 + mu * mu); }
  float miePhase(float mu) {
    float g2 = MIE_G * MIE_G;
    float num = (1.0 - g2) * (1.0 + mu * mu);
    float den = (2.0 + g2) * pow(1.0 + g2 - 2.0 * MIE_G * mu, 1.5);
    return 3.0 / (8.0 * PI) * num / den;
  }
  // Stable ray-sphere: o origin, d unit dir, r radius. Returns vec2(t0,t1)
  // with t1 < t0 on a miss. dot(o,o)-b*b is the squared perpendicular
  // distance (well conditioned), not the cancelling |o|^2 - r^2 form.
  vec2 raySphere(vec3 o, vec3 d, float r) {
    float b = dot(o, d);
    float c = dot(o, o) - b * b;
    float disc = r * r - c;
    // Miss sentinel must be NEGATIVE in BOTH components: callers test
    // hits with ".x > 0.0" (ground clip, LUT ground check), so a positive
    // miss value would be mistaken for a 1 km ground hit and clip the
    // march — producing a thin dark line of under-marched pixels exactly
    // at the horizon tangent.
    if (disc < 0.0) return vec2(-1.0, -1.0);
    float s = sqrt(disc);
    return vec2(-b - s, -b + s);
  }
  // LUT parameterisation: v = normalized altitude, u = remapped view/sun
  // zenith cosine in [-1,1]. Shared by transmittance and multiscatter.
  vec2 lutUV(float r, float mu) {
    return vec2(clamp(mu * 0.5 + 0.5, 0.0, 1.0),
                clamp((r - uRg) / (uRt - uRg), 0.0, 1.0));
  }
  vec3 sampleTransmittance(sampler2D lut, float r, float mu) {
    return texture2D(lut, lutUV(r, mu)).rgb;
  }
`;

const FS_VERT = `
  varying vec2 vUv;
  void main() {
    vUv = uv;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`;

// Transmittance LUT: for a point at altitude r looking along zenith
// cosine mu, the RGB transmittance to the top of the atmosphere (0 when
// the ray hits the ground = sun below local horizon).
const TRANS_FRAG = SCATTER_GLSL + `
  varying vec2 vUv;
  void main() {
    float r = uRg + vUv.y * (uRt - uRg);
    float mu = vUv.x * 2.0 - 1.0;
    vec3 P = vec3(0.0, 0.0, r);
    vec3 D = vec3(sqrt(max(0.0, 1.0 - mu * mu)), 0.0, mu);
    vec2 tg = raySphere(P, D, uRg);
    vec2 tt = raySphere(P, D, uRt);
    bool ground = tg.x > 0.0;
    float tEnd = ground ? tg.x : tt.y;
    const int N = 40;
    float dt = tEnd / float(N);
    vec3 od = vec3(0.0);
    for (int i = 0; i < N; i++) {
      vec3 pos = P + D * (float(i) + 0.5) * dt;
      od += extinction(length(pos) - uRg) * dt;
    }
    vec3 T = ground ? vec3(0.0) : exp(-od);
    gl_FragColor = vec4(T, 1.0);
  }
`;

// Multiple-scattering LUT (Hillaire): for sun-zenith mu_s at altitude r,
// the infinite-order isotropic multiple-scattering radiance psi_ms =
// L_2nd / (1 - f_ms), sampled over a uniform sphere of directions.
const MS_FRAG = SCATTER_GLSL + `
  varying vec2 vUv;
  uniform sampler2D uTransLUT;
  const float GROUND_ALBEDO = 0.3;
  void main() {
    float r = uRg + vUv.y * (uRt - uRg);
    float mu_s = vUv.x * 2.0 - 1.0;
    vec3 P = vec3(0.0, 0.0, r);
    vec3 S = vec3(sqrt(max(0.0, 1.0 - mu_s * mu_s)), 0.0, mu_s);
    vec3 lumAcc = vec3(0.0);
    vec3 fmsAcc = vec3(0.0);
    const int SQRT_N = 8;
    const int STEPS = 20;
    float invN = 1.0 / float(SQRT_N * SQRT_N);
    for (int i = 0; i < SQRT_N; i++) {
      for (int j = 0; j < SQRT_N; j++) {
        float u0 = (float(i) + 0.5) / float(SQRT_N);
        float u1 = (float(j) + 0.5) / float(SQRT_N);
        float cosT = 1.0 - 2.0 * u0;
        float sinT = sqrt(max(0.0, 1.0 - cosT * cosT));
        float phi = 2.0 * PI * u1;
        vec3 W = vec3(sinT * cos(phi), sinT * sin(phi), cosT);
        vec2 tt = raySphere(P, W, uRt);
        vec2 tg = raySphere(P, W, uRg);
        bool ground = tg.x > 0.0;
        float tEnd = ground ? tg.x : tt.y;
        float dt = tEnd / float(STEPS);
        vec3 thru = vec3(1.0);
        vec3 L = vec3(0.0);
        vec3 fms = vec3(0.0);
        for (int s = 0; s < STEPS; s++) {
          vec3 pos = P + W * (float(s) + 0.5) * dt;
          float h = length(pos) - uRg;
          vec3 dens = densities(h);
          vec3 sca = BETA_R * dens.x + vec3(BETA_M) * dens.y;
          vec3 ext = extinction(h);
          vec3 sampleT = exp(-ext * dt);
          float muSun = dot(normalize(pos), S);
          vec3 Tsun = sampleTransmittance(uTransLUT, length(pos), muSun);
          float pu = 1.0 / (4.0 * PI);
          vec3 sIn = Tsun * sca * pu;
          L += thru * (sIn - sIn * sampleT) / max(ext, vec3(1e-6));
          vec3 msC = sca * pu;
          fms += thru * (msC - msC * sampleT) / max(ext, vec3(1e-6));
          thru *= sampleT;
        }
        if (ground) {
          vec3 gp = P + W * tEnd;
          float muG = dot(normalize(gp), S);
          if (muG > 0.0) {
            vec3 Tg = sampleTransmittance(uTransLUT, uRg, muG);
            L += thru * GROUND_ALBEDO * muG * Tg / PI;
          }
        }
        lumAcc += L * invN;
        fmsAcc += fms * invN;
      }
    }
    vec3 psi = lumAcc / (vec3(1.0) - fmsAcc);
    gl_FragColor = vec4(psi, 1.0);
  }
`;

// Main full-screen pass: reconstruct the world-space view ray, read scene
// depth (log-encoded), raymarch single + multiple scattering, and
// composite. Geometry pixels get aerial perspective (sceneColor * Tr +
// inScatter). Sky pixels pass sceneColor through (so additive plumes /
// stars survive) and add in-scatter + the transmittance-reddened sun
// disc.
const PASS_FRAG = SCATTER_GLSL + `
  varying vec2 vUv;
  uniform sampler2D tDiffuse;
  uniform sampler2D tDepth;
  uniform sampler2D uTransLUT;
  uniform sampler2D uMsLUT;
  uniform mat4 uInvProj;
  uniform mat4 uInvView;
  uniform vec3 uCamPosKm;
  uniform vec3 uSunDirWorld;
  uniform float uLogDepthBufFC;
  uniform float uInScatterScale;
  uniform float uSunDiscLum;

  float ign(vec2 p) {
    return fract(52.9829189 * fract(0.06711056 * p.x + 0.00583715 * p.y));
  }

  void main() {
    vec3 sceneColor = texture2D(tDiffuse, vUv).rgb;

    // View-space ray direction. Take (invProj * clip).xyz directly as a
    // direction — do NOT divide by w: at the far plane (clip.z = 1) the
    // inverse projection gives w = 0, and dividing produces a NaN ray.
    vec4 clip = vec4(vUv * 2.0 - 1.0, 1.0, 1.0);
    vec3 viewDir = (uInvProj * clip).xyz;
    vec3 worldDir = normalize((uInvView * vec4(viewDir, 0.0)).xyz);
    vec3 camFwd = normalize((uInvView * vec4(0.0, 0.0, -1.0, 0.0)).xyz);

    float d = texture2D(tDepth, vUv).x;
    bool isSky = d > 0.999999;
    float sceneDistKm = 1.0e9;
    if (!isSky) {
      // Invert three.js logarithmic depth: gl_FragDepth =
      // log2(1+w_eye) / log2(far+1), and uLogDepthBufFC = 2/log2(far+1).
      float wEye = exp2(2.0 * d / uLogDepthBufFC) - 1.0; // metres, eye-perp
      float along = max(dot(worldDir, camFwd), 1e-4);
      sceneDistKm = (wEye / along) * 0.001;              // -> km along ray
    }

    vec3 o = uCamPosKm;
    vec2 atmo = raySphere(o, worldDir, uRt);
    vec3 outColor = sceneColor;
    vec3 L = vec3(0.0);
    vec3 thru = vec3(1.0);

    if (atmo.y > 0.0) {
      float t0 = max(atmo.x, 0.0);
      float t1 = atmo.y;
      vec2 grd = raySphere(o, worldDir, uRg);
      if (grd.x > 0.0) t1 = min(t1, grd.x);
      t1 = min(t1, sceneDistKm);

      if (t1 > t0) {
        float tMax = t1 - t0;
        int N = 32;
        float muV = dot(worldDir, uSunDirWorld);
        float pr = rayleighPhase(muV);
        float pm = miePhase(muV);
        // Quadratic sample spacing: t grows as (k/N)^2 so samples cluster
        // near the camera where the air is densest. Uniform spacing skips
        // the 8 km-scale-height layer on long grazing rays (huge dt) and
        // produces a dark/bright horizon band at low altitude.
        float invN = 1.0 / float(N);
        float prevT = t0;
        for (int i = 0; i < 32; i++) {
          float ft = float(i + 1) * invN;
          float curT = t0 + tMax * ft * ft;
          float segDt = curT - prevT;
          float midT = 0.5 * (prevT + curT);
          prevT = curT;
          vec3 pos = o + worldDir * midT;
          float rr = length(pos);
          float h = rr - uRg;
          vec3 dens = densities(h);
          vec3 scaR = BETA_R * dens.x;
          vec3 scaM = vec3(BETA_M) * dens.y;
          vec3 ext = extinction(h);
          vec3 sampleT = exp(-ext * segDt);
          float muSun = dot(pos / rr, uSunDirWorld);
          vec3 Tsun = sampleTransmittance(uTransLUT, rr, muSun);
          vec3 single = (scaR * pr + scaM * pm) * Tsun;
          vec3 multi = (scaR + scaM) * texture2D(uMsLUT, lutUV(rr, muSun)).rgb;
          vec3 sIn = single + multi;
          L += thru * (sIn - sIn * sampleT) / max(ext, vec3(1e-6));
          thru *= sampleT;
        }

        if (isSky) {
          outColor = sceneColor + L * uInScatterScale;
          float cosSun = dot(worldDir, uSunDirWorld);
          float disc = smoothstep(cos(0.0050), cos(0.0043), cosSun);
          if (disc > 0.0) {
            float muc = dot(normalize(o), uSunDirWorld);
            vec3 Tcam = sampleTransmittance(uTransLUT, length(o), muc);
            float ld = 0.3 + 0.93 * max(muc, 0.0) - 0.23 * max(muc, 0.0) * max(muc, 0.0);
            outColor += disc * Tcam * uSunDiscLum * max(ld, 0.2);
          }
        } else {
          outColor = sceneColor * thru + L * uInScatterScale;
        }
      }
    }

    outColor += (ign(gl_FragCoord.xy) - 0.5) / 255.0;
    gl_FragColor = vec4(outColor, 1.0);
  }
`;

function lutTarget(w, h) {
  return new THREE.WebGLRenderTarget(w, h, {
    type: THREE.HalfFloatType,
    format: THREE.RGBAFormat,
    minFilter: THREE.LinearFilter,
    magFilter: THREE.LinearFilter,
    wrapS: THREE.ClampToEdgeWrapping,
    wrapT: THREE.ClampToEdgeWrapping,
    depthBuffer: false,
  });
}

// Build the transmittance (256x64) then multiple-scattering (32x32) LUTs
// once at init (the sun is fixed in world space and the LUTs are
// camera-independent, so per-frame cost is zero). Must build in this
// order — the MS shader samples the transmittance LUT.
export function buildAtmosphereLUTs(renderer) {
  const transLUT = lutTarget(256, 64);
  const msLUT = lutTarget(32, 32);
  const prevTarget = renderer.getRenderTarget();
  const quad = new FullScreenQuad();

  const common = { uRg: { value: ATMO.Rg }, uRt: { value: ATMO.Rt } };

  quad.material = new THREE.ShaderMaterial({
    uniforms: { ...common },
    vertexShader: FS_VERT,
    fragmentShader: TRANS_FRAG,
    depthTest: false,
    depthWrite: false,
  });
  renderer.setRenderTarget(transLUT);
  quad.render(renderer);
  quad.material.dispose();

  quad.material = new THREE.ShaderMaterial({
    uniforms: { ...common, uTransLUT: { value: transLUT.texture } },
    vertexShader: FS_VERT,
    fragmentShader: MS_FRAG,
    depthTest: false,
    depthWrite: false,
  });
  renderer.setRenderTarget(msLUT);
  quad.render(renderer);
  quad.material.dispose();

  quad.dispose();
  renderer.setRenderTarget(prevTarget);
  return { transLUT, msLUT };
}

// The full-screen atmosphere pass. Binds tDepth from the CURRENT
// readBuffer each frame (the composer ping-pongs renderTarget1/2, each
// with its own DepthTexture) so the depth always matches the colour
// RenderPass just wrote.
export class AtmospherePass extends ShaderPass {
  constructor({ transLUT, msLUT }) {
    // Pass a prebuilt ShaderMaterial (NOT a plain shader object): ShaderPass
    // only runs UniformsUtils.clone() on plain objects, and that clone
    // detaches render-target textures (the LUTs) and warns. With a real
    // material it uses the uniforms as-is, so the live LUT textures stay
    // bound.
    super(
      new THREE.ShaderMaterial({
        uniforms: {
          tDiffuse: { value: null },
          tDepth: { value: null },
          uTransLUT: { value: transLUT.texture },
          uMsLUT: { value: msLUT.texture },
          uRg: { value: ATMO.Rg },
          uRt: { value: ATMO.Rt },
          uInvProj: { value: new THREE.Matrix4() },
          uInvView: { value: new THREE.Matrix4() },
          uCamPosKm: { value: new THREE.Vector3() },
          uSunDirWorld: { value: new THREE.Vector3() },
          uLogDepthBufFC: { value: 0.0757 },
          uInScatterScale: { value: 6.0 },
          uSunDiscLum: { value: 25.0 },
        },
        vertexShader: FS_VERT,
        fragmentShader: PASS_FRAG,
        depthTest: false,
        depthWrite: false,
      }),
    );
  }

  render(renderer, writeBuffer, readBuffer, deltaTime, maskActive) {
    this.uniforms.tDepth.value = readBuffer.depthTexture;
    super.render(renderer, writeBuffer, readBuffer, deltaTime, maskActive);
  }
}
