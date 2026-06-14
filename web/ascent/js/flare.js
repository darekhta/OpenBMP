// Screen-space engine flare + atmospheric god-rays.
//
// This pass reads ONLY the colour buffer — never scene depth — which
// sidesteps the ping-pong depth-staleness that bites any pass after the
// atmosphere pass. Occlusion is inherited for free: the in-scene engine
// glare cores are hardware depth-tested by the vehicle/terrain, so an
// occluded engine simply isn't bright in the colour buffer and produces
// no flare or rays.
//
// It runs at composer index 2 — [RenderPass, Atmosphere, Flare, Bloom,
// OutputPass] — so the flare is itself bloomed and consistent with the
// glare cores, and it reads the post-atmosphere (pre-bloom) colour.
//
// Contributions, all derived from a bright-pass threshold of the colour:
//  - lens-flare ghosts (Chapman: mirrored, scaled samples through centre)
//  - a soft halo ring
//  - a broad anamorphic horizontal streak
//  - radial god-rays from the firing-engine cluster (Mitchell volumetric
//    light scattering), GATED by ambient pressure so they vanish in vacuum
//
// Output is linear HDR added to the scene; ACES is applied once at
// OutputPass.

import * as THREE from "three";
import { ShaderPass } from "../vendor/addons/postprocessing/ShaderPass.js";

const FLARE_SHADER = {
  uniforms: {
    tDiffuse: { value: null },
    uResolution: { value: new THREE.Vector2(1, 1) },
    uSource: { value: new THREE.Vector2(0.5, 0.5) }, // engine cluster screen UV
    uIntensity: { value: 0 }, // 0 → pass is a no-op
    uAmb: { value: 1 }, // ambient pressure fraction; god-rays ∝ amb
    uThreshold: { value: 1.3 }, // HDR bright-pass threshold
  },
  vertexShader: `
    varying vec2 vUv;
    void main() {
      vUv = uv;
      gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
    }`,
  fragmentShader: `
    uniform sampler2D tDiffuse;
    uniform vec2 uResolution;
    uniform vec2 uSource;
    uniform float uIntensity;
    uniform float uAmb;
    uniform float uThreshold;
    varying vec2 vUv;

    // Bright-pass: HDR energy above threshold (occluded glares are already
    // absent from the colour, so this is also the occlusion mask).
    vec3 bright(vec2 uv) {
      if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return vec3(0.0);
      vec3 c = texture2D(tDiffuse, uv).rgb;
      return max(c - uThreshold, 0.0);
    }

    void main() {
      vec3 color = texture2D(tDiffuse, vUv).rgb;
      if (uIntensity <= 0.001) { gl_FragColor = vec4(color, 1.0); return; }

      float aspect = uResolution.x / uResolution.y;
      vec3 flare = vec3(0.0);
      // NOTE: deliberately NO mirrored lens "ghosts" — a scaled copy of the
      // blinding engine source lands on or above the rocket depending on
      // framing and reads as a bug. The flare is the engine's own glare:
      // an anamorphic streak + atmospheric god-rays, both anchored AT the
      // engine source, never mirrored across the frame.

      // --- Anamorphic horizontal streak from the source. ---
      vec3 streak = vec3(0.0);
      for (int i = -16; i <= 16; i++) {
        float o = float(i) / 16.0;
        vec2 uv = uSource + vec2(o * 0.22, 0.0);
        float w = 1.0 - abs(o);
        streak += bright(uv) * w * w;
      }
      float streakBand = exp(-pow((vUv.y - uSource.y) * aspect * 60.0, 2.0));
      flare += streak * streakBand * vec3(0.6, 0.75, 1.0) * 0.018;

      // --- Atmospheric god-rays from the engine cluster (Mitchell). ---
      // Radial march from this pixel toward the source; vanishes in vacuum.
      float airshafts = clamp((uAmb - 0.02) / 0.2, 0.0, 1.0);
      if (airshafts > 0.0) {
        const int SAMPLES = 48;
        vec2 dir = (uSource - vUv) / float(SAMPLES);
        vec2 uv = vUv;
        float decay = 1.0;
        vec3 rays = vec3(0.0);
        for (int i = 0; i < SAMPLES; i++) {
          uv += dir;
          rays += bright(uv) * decay;
          decay *= 0.96;
        }
        flare += rays * (1.0 / float(SAMPLES)) * airshafts * vec3(1.0, 0.85, 0.6) * 0.4;
      }

      gl_FragColor = vec4(color + flare * uIntensity, 1.0);
    }`,
};

export class FlarePass extends ShaderPass {
  constructor() {
    super(
      new THREE.ShaderMaterial({
        uniforms: THREE.UniformsUtils.clone(FLARE_SHADER.uniforms),
        vertexShader: FLARE_SHADER.vertexShader,
        fragmentShader: FLARE_SHADER.fragmentShader,
      }),
    );
  }

  setSize(width, height) {
    this.uniforms.uResolution.value.set(width, height);
  }
}
