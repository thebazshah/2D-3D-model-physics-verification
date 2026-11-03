#!/usr/bin/env node
/**
 * Example: Programmatic Verification of a Procedural 2D/3D Animation Loop
 *
 * Shows how to sample a Three.js / Babylon.js / Canvas procedural animation
 * function over time and verify its physical and biomechanical realism using
 * `verifyScene` from `../scripts/verify_physics.ts`.
 *
 * Run with:
 *   node examples/verify_threejs_example.ts
 */

import {
  verifyScene,
  formatMarkdownReport,
  type MotionFrame,
  type SceneSpec,
} from "../scripts/verify_physics.ts";

// Simulate a procedural bird flight animation (cruising at v = 8 m/s, f = 5 Hz, A = 0.48 m => St = 0.30)
function sampleProceduralBirdFlight(durationSec = 1.0, fps = 30): MotionFrame[] {
  const frames: MotionFrame[] = [];
  const totalFrames = Math.round(durationSec * fps);
  const speed = 8.0; // m/s forward
  const freq = 5.0; // Hz wingbeat
  const strokeAmp = 0.48; // m peak-to-peak

  for (let i = 0; i <= totalFrames; i++) {
    const t = i / fps;
    const phase = 2 * Math.PI * freq * t;
    // Wingtip oscillates +/- 0.24 m
    const wingtipY = 0.5 * strokeAmp * Math.sin(phase);
    // Body heave opposes wing stroke (180 degrees out of phase: rises during downstroke)
    const bodyY = 10.0 - 0.025 * Math.sin(phase);

    frames.push({
      t: Number(t.toFixed(4)),
      position: [Number((speed * t).toFixed(4)), Number(bodyY.toFixed(4)), 0.0],
      grounded: false,
      wingtipY: Number(wingtipY.toFixed(4)),
    });
  }
  return frames;
}

// Simulate a procedural sedan cornering at 12 m/s with Ackermann steering (wheelbase = 2.7 m, steer = 7 deg)
function sampleProceduralCarTurn(durationSec = 1.0, fps = 30): MotionFrame[] {
  const frames: MotionFrame[] = [];
  const totalFrames = Math.round(durationSec * fps);
  const speed = 12.0; // m/s
  const wheelbase = 2.7; // m
  const wheelRadius = 0.33; // m
  const steerDeg = 7.0;
  const steerRad = (steerDeg * Math.PI) / 180.0;
  const yawRateRad = (speed * Math.tan(steerRad)) / wheelbase; // rad/s

  let x = 0.0;
  let z = 0.0;
  for (let i = 0; i <= totalFrames; i++) {
    const t = i / fps;
    const dt = i === 0 ? 0 : 1 / fps;
    const yawRad = yawRateRad * t;
    x += speed * Math.cos(yawRad) * dt;
    z += speed * Math.sin(yawRad) * dt;

    frames.push({
      t: Number(t.toFixed(4)),
      position: [Number(x.toFixed(4)), 0.55, Number(z.toFixed(4))],
      rotation: [0.0, Number(((yawRad * 180) / Math.PI).toFixed(3)), -2.2],
      grounded: true,
      wheels: {
        omegaRadPerSec: Number((speed / wheelRadius).toFixed(3)),
        steerAngleDeg: steerDeg,
        steerInnerDeg: 7.4,
        steerOuterDeg: 6.6,
      },
    });
  }
  return frames;
}

const scene: SceneSpec = {
  scene: {
    dimension: "3d",
    upAxis: "y",
    gravity: 9.80665,
    groundLevel: 0.0,
    metersPerUnit: 1.0,
  },
  entities: [
    {
      id: "peregrine_cruise_valid",
      type: "bird",
      params: { wingspan: 1.0, mass: 0.9 },
      frames: sampleProceduralBirdFlight(),
    },
    {
      id: "sedan_cornering_valid",
      type: "wheeled_vehicle",
      params: { wheelRadius: 0.33, wheelbase: 2.7, trackWidth: 1.55, tireFrictionMu: 0.85 },
      frames: sampleProceduralCarTurn(),
    },
  ],
};

const report = verifyScene(scene, true);
console.log(formatMarkdownReport(report));
if (!report.overall_passed) {
  process.exit(1);
}
