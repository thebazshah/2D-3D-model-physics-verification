#!/usr/bin/env node
/**
 * 2D/3D Model, Rig & Movement Physics Verification Engine (TypeScript / Node.js)
 *
 * Analyzes 2D and 3D models, skeletal rigs, vertex skinning, animation curves,
 * and movements against:
 *   1. Industry 2D/3D Rigging, Skinning & Animation Standards (glTF 2.0, OpenUSD
 *      UsdSkel/UsdPhysics, VRM/Mecanim Humanoid, URDF/MuJoCo inertia tensors,
 *      quaternion hemisphere continuity, LBS candy-wrapper twist, 2-bone IK pop,
 *      Bezier time monotonicity, and 2D mesh triangle winding preservation)
 *   2. Classical Mechanics (kinematics, ballistic gravity, CoM/ZMP balance,
 *      squash-and-stretch volume conservation)
 *   3. Human Biomechanics (Winter's anthropometry, ISB joint RoM, Froude number,
 *      duty factor, footskate, jump crouch impulse)
 *   4. Animal Locomotion (Hildebrand quadruped gaits, avian/aquatic Strouhal laws)
 *   5. Vehicle & Transportation Dynamics (wheel slip, Ackermann steering, car
 *      roll vs. bike lean, aircraft bank/stall, rotorcraft thrust-tilt coupling)
 *
 * Compatible with direct execution in Node.js 22.6+/24+, tsx, bun, and deno:
 *   node scripts/verify_physics.ts <input.json> [--json] [--strict]
 */

import * as fs from "node:fs";
import * as path from "node:path";
import { fileURLToPath } from "node:url";

export type Severity = "ERROR" | "WARNING" | "INFO";
export type Category =
  | "rig_standard"
  | "anim_standard"
  | "kinematics"
  | "gravity"
  | "balance"
  | "volume"
  | "human"
  | "animal"
  | "vehicle";

export interface Issue {
  severity: Severity;
  category: Category;
  code: string;
  message: string;
  frame_time?: number | null;
  expected?: string | null;
  actual?: string | null;
  formula?: string | null;
}

export interface JointAngles {
  flexion?: number;
  abduction?: number;
  rotation?: number;
  [axis: string]: number | undefined;
}

export interface FootState {
  position?: number[];
  grounded?: boolean;
}

export interface WheelState {
  omegaRadPerSec?: number;
  steerAngleDeg?: number;
  steerInnerDeg?: number;
  steerOuterDeg?: number;
}

export interface IKTargetState {
  rootPos?: number[];
  targetPos?: number[];
  poleVector?: number[];
}

export interface BoneSpec {
  name: string;
  parent?: string | null;
  length: number;
  mirrorBone?: string;
}

export interface SkinningSpec {
  maxInfluencesPerVertex?: number;
  unnormalizedVertexCount?: number;
  negativeWeightCount?: number;
  minWeightSum?: number;
  maxWeightSum?: number;
  nonInvertibleBindMatrices?: number;
  negativeDeterminantBindMatrices?: number;
}

export interface InertiaTensorSpec {
  ixx: number;
  iyy: number;
  izz: number;
}

export interface IKChainSpec {
  name: string;
  upperLength: number;
  lowerLength: number;
}

export interface RigSpec {
  standard?: string;
  bones?: BoneSpec[];
  skinning?: SkinningSpec;
  inertiaTensor?: InertiaTensorSpec;
  ikChains?: IKChainSpec[];
}

export interface MotionFrame {
  t?: number;
  position: number[];
  rotation?: number[];
  quaternions?: Record<string, number[]>;
  scale?: number[];
  grounded?: boolean;
  contacts?: number[][];
  joints?: Record<string, JointAngles>;
  boneTwists?: Record<string, number>;
  ikTargets?: Record<string, IKTargetState>;
  bezierHandles?: number[];
  meshTriangles2D?: number[][][];
  feet?: Record<string, FootState>;
  wheels?: WheelState;
  wingtipY?: number;
}

export interface EntitySpec {
  id: string;
  type: string;
  params?: Record<string, unknown>;
  rig?: RigSpec;
  frames: MotionFrame[];
}

export interface SceneSpec {
  scene?: {
    dimension?: "2d" | "3d";
    upAxis?: "y" | "z" | "-y";
    handedness?: "right" | "left";
    gravity?: number;
    groundLevel?: number;
    metersPerUnit?: number;
  };
  entities: EntitySpec[];
}

export interface EntityReport {
  entity_id: string;
  entity_type: string;
  frame_count: number;
  duration_sec: number;
  score: number;
  passed: boolean;
  metrics: Record<string, unknown>;
  issues: Issue[];
}

export interface VerificationResult {
  overall_passed: boolean;
  overall_physics_score: number;
  entity_count: number;
  entities: EntityReport[];
}

type Vec3 = [number, number, number];
type Vec2 = [number, number];

// ---------------------------------------------------------------------------
// Vector & Geometry Utilities
// ---------------------------------------------------------------------------

export function to3DMeters(
  vec: number[],
  dimension: string,
  upAxis: string,
  metersPerUnit: number
): Vec3 {
  const s = vec.map((c) => Number(c) * metersPerUnit);
  if (dimension === "2d" || s.length === 2) {
    const x = s[0] ?? 0;
    const yRaw = s[1] ?? 0;
    const y = upAxis === "-y" ? -yRaw : yRaw;
    return [x, y, 0.0];
  }
  const x = s[0] ?? 0;
  const y = s[1] ?? 0;
  const z = s[2] ?? 0;
  if (upAxis === "z") return [x, z, -y];
  if (upAxis === "-y") return [x, -y, z];
  return [x, y, z];
}

function vecSub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function vecNorm(a: Vec3): number {
  return Math.hypot(a[0], a[1], a[2]);
}

function vecHorizNorm(a: Vec3): number {
  return Math.hypot(a[0], a[2]);
}

function angleDiffDeg(a: number, b: number): number {
  return ((((a - b + 180.0) % 360.0) + 360.0) % 360.0) - 180.0;
}

function median(values: number[]): number {
  if (values.length === 0) return 0.0;
  const s = [...values].sort((a, b) => a - b);
  const mid = Math.floor(s.length / 2);
  return s.length % 2 === 1 ? s[mid] : 0.5 * (s[mid - 1] + s[mid]);
}

function cross2D(o: Vec2, a: Vec2, b: Vec2): number {
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

function convexHull2D(points: Vec2[]): Vec2[] {
  const pts = [...points].sort((a, b) => (a[0] !== b[0] ? a[0] - b[0] : a[1] - b[1]));
  if (pts.length <= 2) return pts;
  const lower: Vec2[] = [];
  for (const p of pts) {
    while (lower.length >= 2 && cross2D(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) {
      lower.pop();
    }
    lower.push(p);
  }
  const upper: Vec2[] = [];
  for (let i = pts.length - 1; i >= 0; i--) {
    const p = pts[i];
    while (upper.length >= 2 && cross2D(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) {
      upper.pop();
    }
    upper.push(p);
  }
  lower.pop();
  upper.pop();
  return lower.concat(upper);
}

function pointToSegmentDist2D(p: Vec2, a: Vec2, b: Vec2): number {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const l2 = dx * dx + dy * dy;
  if (l2 === 0.0) return Math.hypot(p[0] - a[0], p[1] - a[1]);
  const t = Math.max(0.0, Math.min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2));
  return Math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy));
}

function isInsideSupportRegion(
  pt: Vec2,
  contacts: Vec2[],
  tolerance = 0.035
): { ok: boolean; distOut: number } {
  if (contacts.length === 0) return { ok: true, distOut: 0.0 };
  const hull = convexHull2D(contacts);
  if (hull.length === 1) {
    const d = Math.hypot(pt[0] - hull[0][0], pt[1] - hull[0][1]);
    return { ok: d <= tolerance, distOut: Math.max(0.0, d - tolerance) };
  }
  if (hull.length === 2) {
    const d = pointToSegmentDist2D(pt, hull[0], hull[1]);
    return { ok: d <= tolerance, distOut: Math.max(0.0, d - tolerance) };
  }
  let inside = true;
  let minEdgeDist = Infinity;
  for (let i = 0; i < hull.length; i++) {
    const a = hull[i];
    const b = hull[(i + 1) % hull.length];
    if (cross2D(a, b, pt) < 0) inside = false;
    const ed = pointToSegmentDist2D(pt, a, b);
    if (ed < minEdgeDist) minEdgeDist = ed;
  }
  if (inside || minEdgeDist <= tolerance) return { ok: true, distOut: 0.0 };
  return { ok: false, distOut: minEdgeDist - tolerance };
}

// ---------------------------------------------------------------------------
// 0. 2D/3D Rigging, Skinning & Animation Standards Verification
// ---------------------------------------------------------------------------

function verifyRigAndAnimationStandards(
  rig: RigSpec,
  frames: MotionFrame[],
  times: number[],
  dimension: string,
  upAxis: string,
  metersPerUnit: number,
  issues: Issue[],
  metrics: Record<string, unknown>
): void {
  const bones = rig.bones ?? [];
  const boneMap = new Map<string, BoneSpec>();

  if (bones.length > 0) {
    metrics.rig_bone_count = bones.length;
    for (const b of bones) {
      const bName = String(b.name ?? "");
      const bLen = Number(b.length ?? 0.0) * metersPerUnit;
      boneMap.set(bName, b);
      if (bLen <= 1e-4) {
        issues.push({
          severity: "ERROR",
          category: "rig_standard",
          code: "RIG_ZERO_LENGTH_BONE",
          message: `Bone '${bName}' has near-zero length (${bLen.toFixed(6)}m); causes NaN in IK solvers and skinning heatmaps.`,
          expected: "length > 0.0001 m",
          actual: `${bLen.toFixed(6)} m`,
          formula: "L_bone = ||p_child - p_parent|| > 1e-4 m",
        });
      }
    }

    for (const b of bones) {
      const visited = new Set<string>();
      let curr: string | null | undefined = b.name;
      while (curr) {
        if (visited.has(curr)) {
          issues.push({
            severity: "ERROR",
            category: "rig_standard",
            code: "RIG_CYCLIC_HIERARCHY",
            message: `Cycle detected in skeletal hierarchy involving bone '${curr}'. Skeleton must be a Directed Acyclic Graph (DAG).`,
            expected: "Acyclic parent-child tree",
            actual: `Cycle at '${curr}'`,
          });
          break;
        }
        visited.add(curr);
        curr = boneMap.get(curr)?.parent;
      }
    }

    const checkedPairs = new Set<string>();
    for (const b of bones) {
      const bName = String(b.name ?? "");
      const mirror = b.mirrorBone;
      if (mirror && boneMap.has(mirror)) {
        const pairKey = [bName, mirror].sort().join("::");
        if (checkedPairs.has(pairKey)) continue;
        checkedPairs.add(pairKey);
        const l1 = Number(b.length ?? 0.0);
        const l2 = Number(boneMap.get(mirror)!.length ?? 0.0);
        const maxL = Math.max(l1, l2, 1e-6);
        const asym = Math.abs(l1 - l2) / maxL;
        if (asym > 0.015) {
          issues.push({
            severity: "WARNING",
            category: "rig_standard",
            code: "RIG_BILATERAL_ASYMMETRY",
            message: `Bilateral bone pair ('${bName}'=${l1.toFixed(3)} vs '${mirror}'=${l2.toFixed(3)}) differs in length by ${(asym * 100).toFixed(1)}% (> 1.5% VRM/Mecanim symmetry standard).`,
            expected: "<= 1.5% length difference",
            actual: `${(asym * 100).toFixed(1)}%`,
            formula: "|L_left - L_right| / max(L_left, L_right) <= 0.015",
          });
        }
      }
    }

    const rigStd = String(rig.standard ?? "").toLowerCase();
    if (rigStd === "vrm_humanoid" || rigStd === "mecanim_humanoid") {
      const lowerNames = Array.from(boneMap.keys()).map((n) => n.toLowerCase());
      const requiredKeywords = ["hips", "spine", "head"];
      const missing = requiredKeywords.filter((kw) => !lowerNames.some((ln) => ln.includes(kw)));
      if (missing.length > 0) {
        issues.push({
          severity: "WARNING",
          category: "rig_standard",
          code: "RIG_MISSING_HUMANOID_BONES",
          message: `Humanoid rig ('${rigStd}') is missing required core bone(s): [${missing.join(", ")}].`,
          expected: "hips, spine, head, limbs present",
          actual: `Missing [${missing.join(", ")}]`,
        });
      }
    }
  }

  const skin = rig.skinning;
  if (skin) {
    const unnorm = Number(skin.unnormalizedVertexCount ?? 0);
    const negW = Number(skin.negativeWeightCount ?? 0);
    const minSum = Number(skin.minWeightSum ?? 1.0);
    const maxSum = Number(skin.maxWeightSum ?? 1.0);
    const nonInvIbm = Number(skin.nonInvertibleBindMatrices ?? 0);
    const negDetIbm = Number(skin.negativeDeterminantBindMatrices ?? 0);

    if (unnorm > 0 || Math.abs(minSum - 1.0) > 1e-3 || Math.abs(maxSum - 1.0) > 1e-3) {
      issues.push({
        severity: "ERROR",
        category: "rig_standard",
        code: "SKIN_WEIGHTS_NOT_NORMALIZED",
        message: `Vertex skin weights violate Partition of Unity (${unnorm} unnormalized vertices, weight sum range [${minSum.toFixed(4)}, ${maxSum.toFixed(4)}]); causes mesh shrinkage or origin pulling.`,
        expected: "sum(w_j) == 1.0 (±0.0001)",
        actual: `[${minSum.toFixed(4)}, ${maxSum.toFixed(4)}] (${unnorm} vertices)`,
        formula: "sum_{j=1}^K w_{v,j} = 1.0",
      });
    }
    if (negW > 0) {
      issues.push({
        severity: "ERROR",
        category: "rig_standard",
        code: "SKIN_NEGATIVE_WEIGHTS",
        message: `Mesh contains ${negW} vertices with negative bone weights (w_j < 0).`,
        expected: "w_j >= 0",
        actual: `${negW} negative weights`,
      });
    }
    if (nonInvIbm > 0 || negDetIbm > 0) {
      issues.push({
        severity: "ERROR",
        category: "rig_standard",
        code: "BIND_MATRIX_SINGULAR_OR_MIRRORED",
        message: `Rig contains ${nonInvIbm} singular/non-invertible and ${negDetIbm} negative-determinant (mirrored scale) inverseBindMatrices.`,
        expected: "det(M_IBM) > 0 for all joints",
        actual: `singular=${nonInvIbm}, neg_det=${negDetIbm}`,
        formula: "det(inverseBindMatrix) > 0",
      });
    }
  }

  const inertia = rig.inertiaTensor;
  if (inertia) {
    const ixx = Number(inertia.ixx ?? 0);
    const iyy = Number(inertia.iyy ?? 0);
    const izz = Number(inertia.izz ?? 0);
    if (ixx <= 0 || iyy <= 0 || izz <= 0) {
      issues.push({
        severity: "ERROR",
        category: "rig_standard",
        code: "INERTIA_NON_POSITIVE_PRINCIPAL_MOMENT",
        message: `Principal moments of inertia (Ixx=${ixx}, Iyy=${iyy}, Izz=${izz}) must be strictly positive.`,
        expected: "Ixx > 0, Iyy > 0, Izz > 0",
        actual: `(${ixx}, ${iyy}, ${izz})`,
      });
    } else if (ixx + iyy < izz - 1e-6 || iyy + izz < ixx - 1e-6 || izz + ixx < iyy - 1e-6) {
      issues.push({
        severity: "ERROR",
        category: "rig_standard",
        code: "INERTIA_TRIANGLE_INEQUALITY_VIOLATION",
        message: `Principal moments of inertia (Ixx=${ixx}, Iyy=${iyy}, Izz=${izz}) violate the 3D Triangle Inequality; impossible for any real positive-density mass distribution and causes physics solvers to explode.`,
        expected: "I_a + I_b >= I_c for all permutations",
        actual: `Ixx=${ixx}, Iyy=${iyy}, Izz=${izz}`,
        formula: "Ixx + Iyy >= Izz, Iyy + Izz >= Ixx, Izz + Ixx >= Iyy",
      });
    }
  }

  const ikChains = new Map<string, [number, number]>();
  for (const ch of rig.ikChains ?? []) {
    const l1 = Number(ch.upperLength ?? 0) * metersPerUnit;
    const l2 = Number(ch.lowerLength ?? 0) * metersPerUnit;
    if (ch.name && l1 + l2 > 0) {
      ikChains.set(ch.name, [l1, l2]);
    }
  }

  const prevQuats = new Map<string, [number, number, number, number]>();
  const quatNormReported = new Set<string>();
  const quatFlipReported = new Set<string>();
  let gimbalReported = false;
  const twistReported = new Set<string>();
  const ikPopReported = new Set<string>();
  const ikPoleReported = new Set<string>();
  let bezierReported = false;
  let meshFlipReported = false;
  let bindTriSigns: number[] | null = null;

  for (let i = 0; i < frames.length; i++) {
    const f = frames[i];
    const t = times[i];

    const quats = f.quaternions ?? {};
    for (const [jName, q] of Object.entries(quats)) {
      if (!Array.isArray(q) || q.length !== 4) continue;
      const [qx, qy, qz, qw] = [Number(q[0]), Number(q[1]), Number(q[2]), Number(q[3])];
      const norm = Math.hypot(qx, qy, qz, qw);
      if (Math.abs(norm - 1.0) > 2e-3 && !quatNormReported.has(jName)) {
        quatNormReported.add(jName);
        issues.push({
          severity: "ERROR",
          category: "anim_standard",
          code: "QUATERNION_NOT_UNIT_NORM",
          message: `Quaternion on bone '${jName}' is not unit length (||q||=${norm.toFixed(4)}); causes skewing/scaling artifacts in skinning.`,
          frame_time: t,
          expected: "||q|| = 1.0 (±0.002)",
          actual: `||q|| = ${norm.toFixed(4)}`,
          formula: "sqrt(x^2 + y^2 + z^2 + w^2) = 1",
        });
      }
      if (prevQuats.has(jName) && !quatFlipReported.has(jName)) {
        const pq = prevQuats.get(jName)!;
        const dot = qx * pq[0] + qy * pq[1] + qz * pq[2] + qw * pq[3];
        if (dot < -0.05) {
          quatFlipReported.add(jName);
          issues.push({
            severity: "ERROR",
            category: "anim_standard",
            code: "QUATERNION_ANTIPODAL_FLIP",
            message: `Consecutive quaternions on bone '${jName}' have negative dot product (q_i . q_{i-1} = ${dot.toFixed(3)} < 0); causes 360° long-way-around spin flip during interpolation.`,
            frame_time: t,
            expected: "q_i . q_{i-1} >= 0 (same hemisphere)",
            actual: `dot = ${dot.toFixed(3)}`,
            formula: "q_i . q_{i-1} >= 0",
          });
        }
      }
      prevQuats.set(jName, [qx, qy, qz, qw]);
    }

    const rot = f.rotation;
    if (rot && rot.length >= 3 && !gimbalReported) {
      const midAngle = Number(rot[1]);
      if (Math.abs(midAngle) >= 87.0 && Math.abs(midAngle) <= 93.0 && i > 0) {
        const prevRot = frames[i - 1].rotation ?? [0, 0, 0];
        const dOuter = Math.abs(angleDiffDeg(Number(rot[0]), Number(prevRot[0])));
        const dInner = Math.abs(angleDiffDeg(Number(rot[2]), Number(prevRot[2])));
        if (dOuter > 45.0 || dInner > 45.0) {
          gimbalReported = true;
          issues.push({
            severity: "WARNING",
            category: "anim_standard",
            code: "EULER_GIMBAL_LOCK_SINGULARITY",
            message: `Middle Euler axis is near ±90° (${midAngle.toFixed(1)}°) with rapid outer/inner axis flip (Δ=${Math.max(dOuter, dInner).toFixed(1)}°), indicating Euler Gimbal Lock.`,
            frame_time: t,
            expected: "|theta_mid| != 90° or use Quaternion SLERP",
            actual: `theta_mid = ${midAngle.toFixed(1)}°, axis jump = ${Math.max(dOuter, dInner).toFixed(1)}°`,
            formula: "cos(theta_mid) -> 0 at +-90 deg",
          });
        }
      }
    }

    const boneTwists = f.boneTwists ?? {};
    for (const [bName, twistDeg] of Object.entries(boneTwists)) {
      const val = Math.abs(Number(twistDeg));
      if (val > 90.0 && !twistReported.has(bName)) {
        twistReported.add(bName);
        const collapsePct = (1.0 - Math.cos(((Math.min(val, 180.0) * Math.PI) / 180.0) * 0.5)) * 100.0;
        issues.push({
          severity: "WARNING",
          category: "anim_standard",
          code: "LBS_CANDY_WRAPPER_TWIST_EXCEEDED",
          message: `Bone '${bName}' has single-segment axial twist of ${val.toFixed(1)}° (> 90°), causing ${collapsePct.toFixed(0)}% cross-sectional 'candy-wrapper' collapse under Linear Blend Skinning.`,
          frame_time: t,
          expected: "<= 90° per bone segment (or use twist bones / DQS)",
          actual: `${val.toFixed(1)}° (${collapsePct.toFixed(0)}% radius loss)`,
          formula: "r(psi) = r_0 * cos(psi / 2)",
        });
      }
    }

    const ikTargets = f.ikTargets ?? {};
    for (const [cName, ikData] of Object.entries(ikTargets)) {
      if (!ikData) continue;
      if (ikData.rootPos && ikData.targetPos) {
        const rPos = to3DMeters(ikData.rootPos, dimension, upAxis, metersPerUnit);
        const tPos = to3DMeters(ikData.targetPos, dimension, upAxis, metersPerUnit);
        const diff = vecSub(tPos, rPos);
        const dist = vecNorm(diff);
        if (ikChains.has(cName) && !ikPopReported.has(cName)) {
          const [l1, l2] = ikChains.get(cName)!;
          const maxReach = l1 + l2;
          if (dist > 0.992 * maxReach) {
            ikPopReported.add(cName);
            issues.push({
              severity: dist > maxReach ? "ERROR" : "WARNING",
              category: "anim_standard",
              code: "IK_CHAIN_OVEREXTENSION_POP",
              message: `2-bone IK chain '${cName}' target distance (${dist.toFixed(3)}m) is at/beyond full limb extension (${maxReach.toFixed(3)}m); causes infinite angular velocity 'IK knee/elbow pop'.`,
              frame_time: t,
              expected: `D <= ${(0.992 * maxReach).toFixed(3)} m (99.2% of L1+L2)`,
              actual: `D = ${dist.toFixed(3)} m (${((dist / maxReach) * 100).toFixed(1)}%)`,
              formula: "d(theta)/dD = D / (L1 * L2 * sin(theta)) -> inf as D -> L1+L2",
            });
          }
        }
        if (ikData.poleVector && dist > 1e-5 && !ikPoleReported.has(cName)) {
          const pVec = to3DMeters(ikData.poleVector, dimension, upAxis, 1.0);
          const pNorm = vecNorm(pVec);
          if (pNorm > 1e-5) {
            const cosAng = Math.abs(
              (diff[0] * pVec[0] + diff[1] * pVec[1] + diff[2] * pVec[2]) / (dist * pNorm)
            );
            if (cosAng > 0.98) {
              ikPoleReported.add(cName);
              issues.push({
                severity: "ERROR",
                category: "anim_standard",
                code: "IK_POLE_VECTOR_COLLINEAR",
                message: `IK chain '${cName}' target vector is nearly collinear with pole vector (|cos(theta)|=${cosAng.toFixed(3)} > 0.98); causes 180° swivel plane flip.`,
                frame_time: t,
                expected: "|u_target . u_pole| <= 0.98",
                actual: cosAng.toFixed(3),
                formula: "n_plane = u_target x u_pole != 0",
              });
            }
          }
        }
      }
    }

    const bezier = f.bezierHandles;
    if (bezier && bezier.length >= 4 && !bezierReported) {
      const x1 = Number(bezier[0]);
      const x2 = Number(bezier[2]);
      if (x1 < 0.0 || x1 > 1.0 || x2 < 0.0 || x2 > 1.0) {
        bezierReported = true;
        issues.push({
          severity: "ERROR",
          category: "anim_standard",
          code: "BEZIER_TIME_NON_MONOTONIC",
          message: `Cubic Bezier timing curve handles (x1=${x1.toFixed(2)}, x2=${x2.toFixed(2)}) lie outside [0, 1]; causes time axis to fold backward (non-monotonic time).`,
          frame_time: t,
          expected: "0.0 <= x1 <= 1.0 and 0.0 <= x2 <= 1.0",
          actual: `x1=${x1.toFixed(2)}, x2=${x2.toFixed(2)}`,
          formula: "dt(u)/du >= 0 requires x1, x2 in [0, 1]",
        });
      }
    }

    const tris = f.meshTriangles2D;
    if (Array.isArray(tris) && tris.length > 0) {
      const curSigns: number[] = [];
      for (const tri of tris) {
        if (tri.length === 3) {
          const a: Vec2 = [Number(tri[0][0]), Number(tri[0][1])];
          const b: Vec2 = [Number(tri[1][0]), Number(tri[1][1])];
          const c: Vec2 = [Number(tri[2][0]), Number(tri[2][1])];
          curSigns.push(cross2D(a, b, c));
        }
      }
      if (bindTriSigns === null) {
        bindTriSigns = curSigns;
      } else if (!meshFlipReported && curSigns.length === bindTriSigns.length) {
        for (let idx = 0; idx < curSigns.length; idx++) {
          const s0 = bindTriSigns[idx];
          const st = curSigns[idx];
          if (Math.abs(s0) > 1e-6 && s0 * st <= 0) {
            meshFlipReported = true;
            issues.push({
              severity: "ERROR",
              category: "anim_standard",
              code: "MESH_2D_TRIANGLE_FLIP",
              message: `2D deforming mesh triangle #${idx} inverted its winding orientation at t=${t.toFixed(3)}s (inside-out mesh fold / self-intersection).`,
              frame_time: t,
              expected: "sign(Area(t)) == sign(Area(0))",
              actual: `Area(0)=${s0.toFixed(2)}, Area(t)=${st.toFixed(2)}`,
              formula: "((B - A) x (C - A))_t * ((B - A) x (C - A))_0 > 0",
            });
            break;
          }
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 1. Classical Mechanics Verification
// ---------------------------------------------------------------------------

function verifyClassicalMechanics(
  frames: MotionFrame[],
  positions: Vec3[],
  times: number[],
  entityType: string,
  sceneGravity: number,
  groundLevel: number,
  dimension: string,
  upAxis: string,
  metersPerUnit: number,
  issues: Issue[],
  metrics: Record<string, unknown>
): { velocities: Vec3[]; accelerations: Vec3[]; jerks: Vec3[] } {
  const n = frames.length;
  const velocities: Vec3[] = Array.from({ length: n }, () => [0, 0, 0]);
  const accelerations: Vec3[] = Array.from({ length: n }, () => [0, 0, 0]);
  const jerks: Vec3[] = Array.from({ length: n }, () => [0, 0, 0]);

  if (n < 2) return { velocities, accelerations, jerks };

  const speeds: number[] = [];
  for (let i = 1; i < n; i++) {
    const dt = times[i] - times[i - 1];
    if (dt <= 1e-7) {
      issues.push({
        severity: "ERROR",
        category: "kinematics",
        code: "NON_INCREASING_TIMESTAMP",
        message: `Timestamp at frame ${i} (${times[i]}s) is not greater than frame ${i - 1} (${times[i - 1]}s).`,
        frame_time: times[i],
      });
      continue;
    }
    const dp = vecSub(positions[i], positions[i - 1]);
    const v: Vec3 = [dp[0] / dt, dp[1] / dt, dp[2] / dt];
    velocities[i] = v;
    speeds.push(vecNorm(v));
  }
  velocities[0] = [...velocities[1]];

  const accelMags: number[] = [];
  for (let i = 1; i < n - 1; i++) {
    const dtAvg = 0.5 * (times[i] - times[i - 1] + (times[i + 1] - times[i]));
    if (dtAvg <= 1e-7) continue;
    const vNext = velocities[i + 1];
    const vCur = velocities[i];
    const a: Vec3 = [
      (vNext[0] - vCur[0]) / dtAvg,
      (vNext[1] - vCur[1]) / dtAvg,
      (vNext[2] - vCur[2]) / dtAvg,
    ];
    accelerations[i] = a;
    accelMags.push(vecNorm(a));
  }
  if (n >= 3) {
    accelerations[0] = [...accelerations[1]];
    accelerations[n - 1] = [...accelerations[n - 2]];
  }

  const jerkMags: number[] = [];
  for (let i = 2; i < n - 1; i++) {
    const dt = times[i] - times[i - 1];
    if (dt <= 1e-7) continue;
    const da = vecSub(accelerations[i], accelerations[i - 1]);
    const j: Vec3 = [da[0] / dt, da[1] / dt, da[2] / dt];
    jerks[i] = j;
    jerkMags.push(vecNorm(j));
  }

  const medSpeed = median(speeds);
  const maxSpeed = speeds.length ? Math.max(...speeds) : 0;
  const maxAccel = accelMags.length ? Math.max(...accelMags) : 0;
  const maxJerk = jerkMags.length ? Math.max(...jerkMags) : 0;

  metrics.max_speed_mps = Number(maxSpeed.toFixed(3));
  metrics.median_speed_mps = Number(medSpeed.toFixed(3));
  metrics.max_accel_mps2 = Number(maxAccel.toFixed(3));
  metrics.max_jerk_mps3 = Number(maxJerk.toFixed(3));

  for (let i = 1; i < n; i++) {
    const spd = vecNorm(velocities[i]);
    if (medSpeed > 0.1 && spd > Math.max(25.0, 6.0 * medSpeed)) {
      issues.push({
        severity: "ERROR",
        category: "kinematics",
        code: "C0_TELEPORTATION_SPIKE",
        message: `Instantaneous velocity spike (${spd.toFixed(2)} m/s vs median ${medSpeed.toFixed(2)} m/s) indicates position discontinuity/teleportation.`,
        frame_time: times[i],
        expected: `<= ${Math.max(25.0, 6.0 * medSpeed).toFixed(2)} m/s`,
        actual: `${spd.toFixed(2)} m/s`,
        formula: "v_i = ||p_i - p_{i-1}|| / dt",
      });
    }
  }

  if (entityType !== "fish" && entityType !== "watercraft") {
    for (let i = 0; i < n; i++) {
      if (positions[i][1] < groundLevel - 0.02) {
        issues.push({
          severity: "ERROR",
          category: "kinematics",
          code: "GROUND_PENETRATION",
          message: `Entity root/CoM penetrated below ground plane (y=${positions[i][1].toFixed(3)}m < ground=${groundLevel.toFixed(3)}m).`,
          frame_time: times[i],
          expected: `>= ${groundLevel.toFixed(3)} m`,
          actual: `${positions[i][1].toFixed(3)} m`,
        });
        break;
      }
    }
  }

  const isFlyer = ["bird", "aircraft", "rotorcraft", "fish", "watercraft"].includes(entityType);
  if (!isFlyer && n >= 4) {
    const airborneAy: number[] = [];
    const airborneAhoriz: Array<{ t: number; ah: number }> = [];
    for (let i = 1; i < n - 1; i++) {
      const gPrev = frames[i - 1].grounded ?? true;
      const gCur = frames[i].grounded ?? true;
      const gNext = frames[i + 1].grounded ?? true;
      if (gPrev === false && gCur === false && gNext === false) {
        airborneAy.push(accelerations[i][1]);
        airborneAhoriz.push({
          t: times[i],
          ah: Math.hypot(accelerations[i][0], accelerations[i][2]),
        });
      }
    }
    if (airborneAy.length > 0) {
      const avgAy = airborneAy.reduce((a, b) => a + b, 0) / airborneAy.length;
      const expectedAy = -sceneGravity;
      metrics.airborne_vertical_accel_mps2 = Number(avgAy.toFixed(3));
      const relErr = Math.abs(avgAy - expectedAy) / Math.max(sceneGravity, 1e-5);
      if (relErr > 0.25) {
        const kind = avgAy > expectedAy ? "floaty/moon" : "excessively heavy";
        issues.push({
          severity: relErr > 0.4 ? "ERROR" : "WARNING",
          category: "gravity",
          code: "BALLISTIC_GRAVITY_MISMATCH",
          message: `Airborne vertical acceleration (${avgAy.toFixed(2)} m/s^2) deviates from gravity (${expectedAy.toFixed(2)} m/s^2) by ${(relErr * 100).toFixed(1)}% (${kind} arc).`,
          expected: `${expectedAy.toFixed(2)} m/s^2 (±25%)`,
          actual: `${avgAy.toFixed(2)} m/s^2`,
          formula: "a_y = d^2y/dt^2 = -g",
        });
      }
      for (const item of airborneAhoriz) {
        if (item.ah > 2.5) {
          issues.push({
            severity: "WARNING",
            category: "gravity",
            code: "MID_AIR_HORIZONTAL_PROPULSION",
            message: `Unexplained horizontal acceleration (${item.ah.toFixed(2)} m/s^2) while airborne without external contact or thruster.`,
            frame_time: item.t,
            expected: "<= 2.50 m/s^2 (air drag only)",
            actual: `${item.ah.toFixed(2)} m/s^2`,
            formula: "F_{horiz,airborne} = -0.5 * rho * C_d * A * v^2",
          });
          break;
        }
      }
    }
  }

  for (let i = 0; i < n; i++) {
    const scale = frames[i].scale;
    if (scale && scale.length >= 2) {
      const is2D = dimension === "2d" || scale.length === 2;
      const vol = is2D ? scale[0] * scale[1] : scale[0] * scale[1] * (scale[2] ?? 1.0);
      const formulaStr = is2D ? "A_ratio = s_x * s_y = 1.0" : "V_ratio = s_x * s_y * s_z = 1.0";
      if (Math.abs(vol - 1.0) > 0.15) {
        issues.push({
          severity: "WARNING",
          category: "volume",
          code: "VOLUME_CONSERVATION_VIOLATION",
          message: `Squash-and-stretch scale [${scale.join(", ")}] violates volume/area conservation (product=${vol.toFixed(3)}, expected ~1.0).`,
          frame_time: times[i],
          expected: "1.00 (±0.15)",
          actual: vol.toFixed(3),
          formula: formulaStr,
        });
        break;
      }
    }
  }

  for (let i = 0; i < n; i++) {
    const rawContacts = frames[i].contacts;
    if (!rawContacts || rawContacts.length === 0 || (frames[i].grounded ?? true) === false) {
      continue;
    }
    const contacts2D: Vec2[] = rawContacts.map((c) => {
      const c3 = to3DMeters(c, dimension, upAxis, metersPerUnit);
      return [c3[0], c3[2]];
    });
    const pos = positions[i];
    const vel = velocities[i];
    const acc = accelerations[i];
    const isQuasiStatic = vecNorm(vel) < 0.15 && vecNorm(acc) < 0.5;

    if (isQuasiStatic) {
      const { ok, distOut } = isInsideSupportRegion([pos[0], pos[2]], contacts2D, 0.03);
      if (!ok) {
        issues.push({
          severity: "ERROR",
          category: "balance",
          code: "STATIC_COM_OUTSIDE_BOS",
          message: `Static Center of Mass projection (${pos[0].toFixed(2)}, ${pos[2].toFixed(2)}) lies ${distOut.toFixed(3)}m outside Base of Support polygon; model will tip over.`,
          frame_time: times[i],
          expected: "Inside Base of Support convex hull",
          actual: `${distOut.toFixed(3)}m outside hull`,
          formula: "p_{CoM,xz} in ConvexHull(contacts)",
        });
        break;
      }
    } else {
      const denom = acc[1] + sceneGravity;
      if (denom > 1.0) {
        const hCom = Math.max(0.0, pos[1] - groundLevel);
        const zmpX = pos[0] - (hCom / denom) * acc[0];
        const zmpZ = pos[2] - (hCom / denom) * acc[2];
        const { ok, distOut } = isInsideSupportRegion([zmpX, zmpZ], contacts2D, 0.08);
        if (!ok && distOut > 0.1) {
          issues.push({
            severity: "WARNING",
            category: "balance",
            code: "DYNAMIC_ZMP_OUTSIDE_BOS",
            message: `Dynamic Zero Moment Point (${zmpX.toFixed(2)}, ${zmpZ.toFixed(2)}) lies ${distOut.toFixed(2)}m outside support polygon during stance.`,
            frame_time: times[i],
            expected: "ZMP inside Base of Support",
            actual: `${distOut.toFixed(2)}m outside`,
            formula: "x_ZMP = x_CoM - (y_CoM / (a_y + g)) * a_x",
          });
          break;
        }
      }
    }
  }

  return { velocities, accelerations, jerks };
}

// ---------------------------------------------------------------------------
// 2. Human Biomechanics Verification
// ---------------------------------------------------------------------------

const HUMAN_JOINT_LIMITS: Record<string, Record<string, [number, number]>> = {
  neck: { flexion: [-65, 75], abduction: [-55, 55], rotation: [-90, 90] },
  spine: { flexion: [-40, 95], abduction: [-45, 45], rotation: [-55, 55] },
  shoulder: { flexion: [-70, 190], abduction: [-55, 185], rotation: [-85, 100] },
  elbow: { flexion: [-5, 155], abduction: [-5, 5], rotation: [-95, 95] },
  wrist: { flexion: [-85, 90], abduction: [-25, 45], rotation: [-15, 15] },
  hip: { flexion: [-40, 135], abduction: [-35, 60], rotation: [-50, 55] },
  knee: { flexion: [-5, 155], abduction: [-5, 5], rotation: [-20, 20] },
  ankle: { flexion: [-60, 35], abduction: [-25, 25], rotation: [-25, 25] },
};

function canonicalJointType(jointName: string): string | null {
  const lower = jointName.toLowerCase();
  for (const key of Object.keys(HUMAN_JOINT_LIMITS)) {
    if (lower.includes(key)) return key;
  }
  return null;
}

function verifyHumanBiomechanics(
  frames: MotionFrame[],
  positions: Vec3[],
  velocities: Vec3[],
  times: number[],
  params: Record<string, unknown>,
  sceneGravity: number,
  groundLevel: number,
  dimension: string,
  upAxis: string,
  metersPerUnit: number,
  issues: Issue[],
  metrics: Record<string, unknown>
): void {
  const height = Number(params.height ?? 1.75);
  const legLength = Number(params.legLength ?? 0.53 * height);
  const expectedGait = String(params.expectedGait ?? "").toLowerCase();

  const reportedJoints = new Set<string>();
  for (let i = 0; i < frames.length; i++) {
    const joints = frames[i].joints ?? {};
    for (const [jName, jAngles] of Object.entries(joints)) {
      const jType = canonicalJointType(jName);
      if (!jType || !jAngles) continue;
      const limits = HUMAN_JOINT_LIMITS[jType];
      for (const [axis, [minDeg, maxDeg]] of Object.entries(limits)) {
        const val = jAngles[axis];
        if (typeof val === "number" && (val < minDeg || val > maxDeg)) {
          const key = `${jName}:${axis}`;
          if (!reportedJoints.has(key)) {
            reportedJoints.add(key);
            const isHyper = (jType === "knee" || jType === "elbow") && axis === "flexion" && val < minDeg;
            const isLateralHinge = (jType === "knee" || jType === "elbow") && axis === "abduction";
            const desc = isHyper
              ? `Hinge joint '${jName}' hyperextended (${val.toFixed(1)}°)`
              : isLateralHinge
                ? `Hinge joint '${jName}' bent sideways (${val.toFixed(1)}°)`
                : `Joint '${jName}' ${axis} (${val.toFixed(1)}°) exceeds human anatomical RoM`;
            issues.push({
              severity: "ERROR",
              category: "human",
              code: "HUMAN_JOINT_ROM_VIOLATION",
              message: `${desc} [allowed ${minDeg}° to ${maxDeg}°].`,
              frame_time: times[i],
              expected: `[${minDeg}°, ${maxDeg}°]`,
              actual: `${val.toFixed(1)}°`,
            });
          }
        }
      }
    }
  }

  const horizSpeeds = velocities.slice(1).map(vecHorizNorm);
  const avgHorizSpeed = horizSpeeds.length
    ? horizSpeeds.reduce((a, b) => a + b, 0) / horizSpeeds.length
    : 0.0;
  const froude = (avgHorizSpeed * avgHorizSpeed) / (sceneGravity * Math.max(legLength, 0.1));
  metrics.avg_horiz_speed_mps = Number(avgHorizSpeed.toFixed(3));
  metrics.froude_number = Number(froude.toFixed(3));

  if (expectedGait === "walk" && froude > 0.6) {
    issues.push({
      severity: "WARNING",
      category: "human",
      code: "WALK_FROUDE_EXCEEDED",
      message: `Walking gait at speed ${avgHorizSpeed.toFixed(2)} m/s yields Froude number Fr=${froude.toFixed(2)} (> 0.50 walk-to-run transition threshold); inverted pendulum vault cannot stay grounded.`,
      expected: "Fr <= 0.50 for walking",
      actual: `Fr = ${froude.toFixed(2)}`,
      formula: "Fr = v^2 / (g * L_leg)",
    });
  } else if (expectedGait === "run" && froude < 0.25 && avgHorizSpeed > 0.1) {
    issues.push({
      severity: "WARNING",
      category: "human",
      code: "RUN_FROUDE_TOO_LOW",
      message: `Running gait at speed ${avgHorizSpeed.toFixed(2)} m/s yields Froude number Fr=${froude.toFixed(2)} (< 0.50); too slow for natural running mechanics.`,
      expected: "Fr >= 0.45 for running",
      actual: `Fr = ${froude.toFixed(2)}`,
      formula: "Fr = v^2 / (g * L_leg)",
    });
  }

  const n = frames.length;
  const hasFeet = frames.some((f) => f.feet !== undefined);
  if (hasFeet && n >= 3) {
    let leftGrounded = 0;
    let rightGrounded = 0;
    let doubleSupport = 0;
    let flightFrames = 0;
    const prevFootPos: Record<string, Vec3> = {};
    let footskateReported = false;

    for (let i = 0; i < n; i++) {
      const feet = frames[i].feet ?? {};
      const lInfo = feet.left ?? {};
      const rInfo = feet.right ?? {};
      if (lInfo.grounded === true) leftGrounded++;
      if (rInfo.grounded === true) rightGrounded++;
      if (lInfo.grounded === true && rInfo.grounded === true) doubleSupport++;
      if (lInfo.grounded === false && rInfo.grounded === false) flightFrames++;

      for (const [side, info] of [
        ["left", lInfo],
        ["right", rInfo],
      ] as const) {
        if (!info.position) continue;
        const fPos = to3DMeters(info.position, dimension, upAxis, metersPerUnit);
        const isG = info.grounded ?? fPos[1] <= groundLevel + 0.02;
        if (isG && prevFootPos[side] && i > 0) {
          const dt = times[i] - times[i - 1];
          if (dt > 1e-6) {
            const slipSpeed =
              Math.hypot(fPos[0] - prevFootPos[side][0], fPos[2] - prevFootPos[side][2]) / dt;
            if (slipSpeed > Math.max(0.2, 0.15 * avgHorizSpeed) && !footskateReported) {
              footskateReported = true;
              issues.push({
                severity: "ERROR",
                category: "human",
                code: "FOOTSKATE_SLIDING",
                message: `Grounded '${side}' foot slid horizontally at ${slipSpeed.toFixed(2)} m/s during stance phase (footskate).`,
                frame_time: times[i],
                expected: "<= 0.15 m/s while grounded",
                actual: `${slipSpeed.toFixed(2)} m/s`,
                formula: "||v_{foot,xz}|| = 0 during stance",
              });
            }
          }
        }
        if (isG) {
          prevFootPos[side] = fPos;
        } else {
          delete prevFootPos[side];
        }
      }
    }

    const avgDuty = 0.5 * (leftGrounded / n + rightGrounded / n);
    metrics.duty_factor_avg = Number(avgDuty.toFixed(3));
    metrics.double_support_ratio = Number((doubleSupport / n).toFixed(3));
    metrics.flight_phase_ratio = Number((flightFrames / n).toFixed(3));

    if (expectedGait === "walk" && (avgDuty < 0.5 || flightFrames > 0)) {
      issues.push({
        severity: "ERROR",
        category: "human",
        code: "WALK_DUTY_FACTOR_INVALID",
        message: `Walking gait must have duty factor > 0.50 and zero flight phase (got beta=${avgDuty.toFixed(2)}, flight_frames=${flightFrames}).`,
        expected: "beta in [0.52, 0.70], flight_frames = 0",
        actual: `beta=${avgDuty.toFixed(2)}, flight_frames=${flightFrames}`,
        formula: "beta = t_stance / T_stride > 0.5",
      });
    } else if (expectedGait === "run" && (avgDuty >= 0.52 || doubleSupport > Math.floor(0.1 * n))) {
      issues.push({
        severity: "WARNING",
        category: "human",
        code: "RUN_DUTY_FACTOR_INVALID",
        message: `Running gait should have duty factor < 0.50 and an aerial flight phase (got beta=${avgDuty.toFixed(2)}).`,
        expected: "beta in [0.25, 0.48]",
        actual: `beta=${avgDuty.toFixed(2)}`,
        formula: "beta = t_stance / T_stride < 0.5",
      });
    }
  }

  if ((expectedGait === "walk" || expectedGait === "run") && n >= 5) {
    const yVals = positions.map((p) => p[1]);
    const yAmp = Math.max(...yVals) - Math.min(...yVals);
    metrics.pelvis_vertical_excursion_m = Number(yAmp.toFixed(4));
    if (yAmp < 0.012) {
      issues.push({
        severity: "WARNING",
        category: "human",
        code: "FLAT_PELVIS_TRAJECTORY",
        message: `Pelvis vertical oscillation (${(yAmp * 100).toFixed(1)} cm) is unnaturally flat during ${expectedGait} ('groucho walk'); natural ${expectedGait} has 3-8 cm vertical bob.`,
        expected: "0.025 m to 0.090 m",
        actual: `${yAmp.toFixed(3)} m`,
      });
    }
  }

  if (expectedGait === "jump" && n >= 4) {
    let takeoffIdx: number | null = null;
    for (let i = 1; i < n; i++) {
      if ((frames[i - 1].grounded ?? true) && !(frames[i].grounded ?? true)) {
        takeoffIdx = i - 1;
        break;
      }
    }
    if (takeoffIdx !== null) {
      const preY = positions.slice(0, takeoffIdx + 1).map((p) => p[1]);
      const airY = positions.slice(takeoffIdx).map((p) => p[1]);
      const yTakeoff = positions[takeoffIdx][1];
      const yMinCrouch = Math.min(...preY);
      const crouchDepth = Math.max(preY[0] - yMinCrouch, yTakeoff - yMinCrouch);
      const jumpHeight = Math.max(...airY) - yTakeoff;
      metrics.jump_height_m = Number(jumpHeight.toFixed(3));
      metrics.crouch_depth_m = Number(crouchDepth.toFixed(3));
      if (jumpHeight > 0.15) {
        if (crouchDepth < 0.04) {
          issues.push({
            severity: "ERROR",
            category: "human",
            code: "JUMP_MISSING_ANTICIPATION_CROUCH",
            message: `Jump of height ${jumpHeight.toFixed(2)}m occurred with negligible anticipation crouch (${(crouchDepth * 100).toFixed(1)}cm); violates impulse-momentum work requirement.`,
            frame_time: times[takeoffIdx],
            expected: "crouch_depth >= 0.08 m",
            actual: `${crouchDepth.toFixed(3)} m`,
            formula: "a_push = g * (h_jump / d_crouch)",
          });
        } else {
          const reqAccelG = jumpHeight / crouchDepth;
          if (reqAccelG > 4.0) {
            issues.push({
              severity: "WARNING",
              category: "human",
              code: "JUMP_EXCESSIVE_PUSH_G_FORCE",
              message: `Jump height (${jumpHeight.toFixed(2)}m) relative to shallow crouch (${crouchDepth.toFixed(2)}m) requires ${reqAccelG.toFixed(1)}g average leg push-off acceleration (human limit ~3.5g).`,
              frame_time: times[takeoffIdx],
              expected: "a_push <= 3.5 g",
              actual: `${reqAccelG.toFixed(1)} g`,
              formula: "a_push / g = h_jump / d_crouch",
            });
          }
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 3. Animal Locomotion Verification
// ---------------------------------------------------------------------------

const QUADRUPED_GAIT_PHASES: Record<string, { duty: [number, number]; phases: number[] }> = {
  walk: { duty: [0.58, 0.8], phases: [0.0, 0.25, 0.5, 0.75] },
  trot: { duty: [0.35, 0.56], phases: [0.0, 0.5, 0.5, 0.0] },
  pace: { duty: [0.35, 0.56], phases: [0.0, 0.0, 0.5, 0.5] },
  canter: { duty: [0.32, 0.5], phases: [0.0, 0.3, 0.3, 0.65] },
  transverse_gallop: { duty: [0.2, 0.4], phases: [0.0, 0.45, 0.15, 0.6] },
  rotary_gallop: { duty: [0.18, 0.38], phases: [0.0, 0.55, 0.15, 0.4] },
};

function phaseDistance(p1: number, p2: number): number {
  const d = Math.abs(p1 - p2) % 1.0;
  return Math.min(d, 1.0 - d);
}

function verifyAnimalLocomotion(
  entityType: string,
  frames: MotionFrame[],
  positions: Vec3[],
  velocities: Vec3[],
  times: number[],
  params: Record<string, unknown>,
  metersPerUnit: number,
  issues: Issue[],
  metrics: Record<string, unknown>
): void {
  const n = frames.length;
  const horizSpeeds = velocities.slice(1).map(vecHorizNorm);
  const avgSpeed = horizSpeeds.length
    ? horizSpeeds.reduce((a, b) => a + b, 0) / horizSpeeds.length
    : 0.0;
  metrics.avg_horiz_speed_mps = Number(avgSpeed.toFixed(3));

  if (entityType === "quadruped") {
    const expectedGait = String(params.expectedGait ?? "").toLowerCase();
    const limbs = ["LH", "LF", "RH", "RF"] as const;
    const groundedCounts: Record<string, number> = { LH: 0, LF: 0, RH: 0, RF: 0 };
    const firstStrikeTime: Record<string, number> = {};

    for (let i = 0; i < n; i++) {
      const feet = frames[i].feet ?? {};
      for (const limb of limbs) {
        const isG = feet[limb]?.grounded;
        if (isG === true) {
          groundedCounts[limb]++;
          const prevG = i > 0 ? (frames[i - 1].feet?.[limb]?.grounded ?? false) : false;
          if ((!prevG || i === 0) && firstStrikeTime[limb] === undefined) {
            firstStrikeTime[limb] = times[i];
          }
        }
      }
    }

    const totalGrounded = Object.values(groundedCounts).reduce((a, b) => a + b, 0);
    if (totalGrounded > 0 && n > 0) {
      const avgDuty = totalGrounded / (4.0 * n);
      metrics.quadruped_duty_factor = Number(avgDuty.toFixed(3));

      const spec = QUADRUPED_GAIT_PHASES[expectedGait];
      if (spec) {
        const [dMin, dMax] = spec.duty;
        if (avgDuty < dMin || avgDuty > dMax) {
          issues.push({
            severity: "WARNING",
            category: "animal",
            code: "QUADRUPED_DUTY_FACTOR_MISMATCH",
            message: `Quadruped '${expectedGait}' duty factor (${avgDuty.toFixed(2)}) is outside Hildebrand biomechanical range [${dMin.toFixed(2)}, ${dMax.toFixed(2)}].`,
            expected: `[${dMin.toFixed(2)}, ${dMax.toFixed(2)}]`,
            actual: avgDuty.toFixed(2),
            formula: "beta = t_stance / T_stride",
          });
        }

        const duration = times[n - 1] - times[0];
        const stridePeriod = Number(params.stridePeriodSec ?? duration);
        if (Object.keys(firstStrikeTime).length === 4 && stridePeriod > 1e-4) {
          const tLH = firstStrikeTime.LH;
          const actualPhases = limbs.map(
            (l) => ((((firstStrikeTime[l] - tLH) / stridePeriod) % 1.0) + 1.0) % 1.0
          );
          metrics.quadruped_limb_phases = actualPhases.map((p) => Number(p.toFixed(3)));
          const maxErr = Math.max(
            ...actualPhases.map((p, idx) => phaseDistance(p, spec.phases[idx]))
          );
          if (maxErr > 0.14) {
            issues.push({
              severity: "ERROR",
              category: "animal",
              code: "QUADRUPED_PHASE_SEQUENCE_ERROR",
              message: `Quadruped footfall phase sequence [LH, LF, RH, RF] = [${actualPhases.map((p) => p.toFixed(2)).join(", ")}] does not match '${expectedGait}' pattern [${spec.phases.join(", ")}] (max error=${maxErr.toFixed(2)}).`,
              expected: `[${spec.phases.join(", ")}]`,
              actual: `[${actualPhases.map((p) => p.toFixed(2)).join(", ")}]`,
              formula: "phi_i = (t_strike,i - t_strike,LH) / T_stride mod 1",
            });
          }
        }
      }
    }
  } else if (entityType === "bird") {
    let freq = params.wingbeatFreqHz !== undefined ? Number(params.wingbeatFreqHz) : undefined;
    let amp = params.strokeAmplitudeM !== undefined ? Number(params.strokeAmplitudeM) : undefined;

    const wingY = frames
      .filter((f) => typeof f.wingtipY === "number")
      .map((f) => Number(f.wingtipY) * metersPerUnit);

    if (wingY.length >= 4) {
      if (amp === undefined) {
        amp = Math.max(...wingY) - Math.min(...wingY);
      }
      if (freq === undefined) {
        const meanW = 0.5 * (Math.max(...wingY) + Math.min(...wingY));
        let crossings = 0;
        for (let i = 1; i < wingY.length; i++) {
          if (wingY[i - 1] < meanW && wingY[i] >= meanW) crossings++;
        }
        const dur = times[n - 1] - times[0];
        if (dur > 0 && crossings > 0) freq = crossings / dur;
      }

      if (wingY.length === n) {
        let inPhaseFrames = 0;
        let checkedFrames = 0;
        for (let i = 1; i < n; i++) {
          const dw = wingY[i] - wingY[i - 1];
          const db = positions[i][1] - positions[i - 1][1];
          if (Math.abs(dw) > 1e-4 && Math.abs(db) > 1e-4) {
            checkedFrames++;
            if (dw * db > 0) inPhaseFrames++;
          }
        }
        if (checkedFrames >= 4 && inPhaseFrames / checkedFrames > 0.8) {
          issues.push({
            severity: "WARNING",
            category: "animal",
            code: "AVIAN_BODY_HEAVE_IN_PHASE_ERROR",
            message:
              "Bird torso moves upward during wing upstroke and downward during downstroke (in-phase); aerodynamically, lift peaks on the downstroke so body heave should oppose wing motion.",
            expected: "Body heave ~180° out of phase with wing vertical stroke",
            actual: "In-phase motion detected",
          });
        }
      }
    }

    if (freq !== undefined && amp !== undefined && avgSpeed > 0.5) {
      const strouhal = (freq * amp) / avgSpeed;
      metrics.strouhal_number = Number(strouhal.toFixed(3));
      if (strouhal < 0.15 || strouhal > 0.48) {
        issues.push({
          severity: "WARNING",
          category: "animal",
          code: "AVIAN_STROUHAL_OUT_OF_RANGE",
          message: `Flapping flight Strouhal number St=${strouhal.toFixed(2)} (f=${freq.toFixed(1)}Hz, A=${amp.toFixed(2)}m, v=${avgSpeed.toFixed(2)}m/s) lies outside natural efficient range [0.20, 0.40].`,
          expected: "0.20 <= St <= 0.40",
          actual: `St = ${strouhal.toFixed(2)}`,
          formula: "St = (f * A) / v",
        });
      }
    }
  } else if (entityType === "fish") {
    const freq = params.tailBeatFreqHz !== undefined ? Number(params.tailBeatFreqHz) : undefined;
    const amp = params.tailAmplitudeM !== undefined ? Number(params.tailAmplitudeM) : undefined;
    const waveSpeed =
      params.bodyWaveSpeedMps !== undefined ? Number(params.bodyWaveSpeedMps) : undefined;

    if (freq !== undefined && amp !== undefined && avgSpeed > 0.05) {
      const strouhal = (freq * amp) / avgSpeed;
      metrics.strouhal_number = Number(strouhal.toFixed(3));
      if (strouhal < 0.15 || strouhal > 0.48) {
        issues.push({
          severity: "WARNING",
          category: "animal",
          code: "AQUATIC_STROUHAL_OUT_OF_RANGE",
          message: `Undulatory swimming Strouhal number St=${strouhal.toFixed(2)} lies outside natural biological range [0.20, 0.40].`,
          expected: "0.20 <= St <= 0.40",
          actual: `St = ${strouhal.toFixed(2)}`,
          formula: "St = (f * A) / v",
        });
      }
    }
    if (waveSpeed !== undefined && waveSpeed > 0) {
      const slip = avgSpeed / waveSpeed;
      metrics.wave_slip_ratio = Number(slip.toFixed(3));
      if (slip >= 1.0 || slip < 0.35) {
        issues.push({
          severity: slip >= 1.0 ? "ERROR" : "WARNING",
          category: "animal",
          code: "AQUATIC_WAVE_SLIP_INVALID",
          message: `Body undulation wave speed (${waveSpeed.toFixed(2)} m/s) vs forward speed (${avgSpeed.toFixed(2)} m/s) gives slip ratio v/v_wave=${slip.toFixed(2)}; backward-propagating wave must travel faster than forward swimming speed (0.55 <= v/v_wave <= 0.85).`,
          expected: "0.55 <= v / v_wave <= 0.85",
          actual: slip.toFixed(2),
          formula: "slip = v_forward / v_wave < 1.0",
        });
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 4. Vehicle & Transportation Verification
// ---------------------------------------------------------------------------

function verifyVehicleDynamics(
  entityType: string,
  frames: MotionFrame[],
  velocities: Vec3[],
  accelerations: Vec3[],
  times: number[],
  params: Record<string, unknown>,
  sceneGravity: number,
  issues: Issue[]
): void {
  const n = frames.length;
  if (n < 3) return;

  const pitches = frames.map((f) => Number(f.rotation?.[0] ?? 0));
  const yaws = frames.map((f) => Number(f.rotation?.[1] ?? 0));
  const rolls = frames.map((f) => Number(f.rotation?.[2] ?? 0));

  const yawRatesRad: number[] = Array.from({ length: n }, () => 0);
  for (let i = 1; i < n; i++) {
    const dt = times[i] - times[i - 1];
    if (dt > 1e-6) {
      const dyaw = angleDiffDeg(yaws[i], yaws[i - 1]);
      yawRatesRad[i] = (dyaw * Math.PI) / 180.0 / dt;
    }
  }
  yawRatesRad[0] = yawRatesRad[1];

  if (entityType === "wheeled_vehicle" || entityType === "two_wheeled_vehicle") {
    const wheelRadius = Number(params.wheelRadius ?? 0.33);
    const wheelbase = Number(params.wheelbase ?? 2.7);
    const mu = Number(params.tireFrictionMu ?? 0.85);

    let slipReported = false;
    let steerReported = false;
    let ackermannReported = false;
    let pitchReported = false;
    let rollReported = false;
    let gripReported = false;

    for (let i = 1; i < n - 1; i++) {
      const vHoriz = vecHorizNorm(velocities[i]);
      const yawRate = yawRatesRad[i];
      const latAccel = vHoriz * Math.abs(yawRate);
      const longAccel =
        (vecHorizNorm(velocities[i + 1]) - vecHorizNorm(velocities[i - 1])) /
        Math.max(times[i + 1] - times[i - 1], 1e-6);

      const wheels = frames[i].wheels ?? {};
      if (typeof wheels.omegaRadPerSec === "number" && vHoriz > 0.5 && !slipReported) {
        const omega = wheels.omegaRadPerSec;
        const vWheel = omega * wheelRadius;
        const slipRatio = (vWheel - vHoriz) / Math.max(Math.abs(vHoriz), Math.abs(vWheel), 1e-5);
        if (Math.abs(slipRatio) > 0.2 && Math.abs(longAccel) < 0.5 * mu * sceneGravity) {
          slipReported = true;
          issues.push({
            severity: "ERROR",
            category: "vehicle",
            code: "WHEEL_ROLLING_SLIP_MISMATCH",
            message: `Wheel tangential speed (omega*R = ${vWheel.toFixed(2)} m/s) mismatches vehicle speed (${vHoriz.toFixed(2)} m/s), slip ratio kappa=${slipRatio.toFixed(2)} during non-skidding motion.`,
            frame_time: times[i],
            expected: `omega = v / R = ${(vHoriz / wheelRadius).toFixed(2)} rad/s`,
            actual: `${omega.toFixed(2)} rad/s`,
            formula: "omega_wheel = v_x / R_wheel",
          });
        }
      }

      if (typeof wheels.steerAngleDeg === "number" && vHoriz > 1.0 && !steerReported) {
        const steerRad = (wheels.steerAngleDeg * Math.PI) / 180.0;
        const expectedYawRate = (vHoriz * Math.tan(steerRad)) / Math.max(wheelbase, 0.1);
        if (Math.abs(expectedYawRate - yawRate) > 0.25 && latAccel <= mu * sceneGravity) {
          steerReported = true;
          issues.push({
            severity: "ERROR",
            category: "vehicle",
            code: "ACKERMANN_YAW_RATE_MISMATCH",
            message: `Vehicle yaw rate (${((yawRate * 180) / Math.PI).toFixed(1)}°/s) mismatches steering angle (${wheels.steerAngleDeg.toFixed(1)}°) and wheelbase (${wheelbase}m); expected ${((expectedYawRate * 180) / Math.PI).toFixed(1)}°/s.`,
            frame_time: times[i],
            expected: `${((expectedYawRate * 180) / Math.PI).toFixed(1)} deg/s`,
            actual: `${((yawRate * 180) / Math.PI).toFixed(1)} deg/s`,
            formula: "psi_dot = (v * tan(delta)) / L_wheelbase",
          });
        }
      }

      if (
        typeof wheels.steerInnerDeg === "number" &&
        typeof wheels.steerOuterDeg === "number" &&
        !ackermannReported
      ) {
        const dIn = Math.abs(wheels.steerInnerDeg);
        const dOut = Math.abs(wheels.steerOuterDeg);
        if (dIn > 3.0 && dIn <= dOut) {
          ackermannReported = true;
          issues.push({
            severity: "WARNING",
            category: "vehicle",
            code: "ACKERMANN_INNER_OUTER_ANGLE_ERROR",
            message: `Inner front wheel steer angle (${dIn.toFixed(1)}°) must be sharper (larger) than outer front wheel (${dOut.toFixed(1)}°) to satisfy Ackermann geometry.`,
            frame_time: times[i],
            expected: `delta_inner > delta_outer (${dOut.toFixed(1)}°)`,
            actual: `delta_inner = ${dIn.toFixed(1)}°`,
            formula: "cot(delta_outer) - cot(delta_inner) = W_track / L_wheelbase",
          });
        }
      }

      if (latAccel > mu * sceneGravity * 1.15 && !gripReported) {
        gripReported = true;
        issues.push({
          severity: "WARNING",
          category: "vehicle",
          code: "LATERAL_TIRE_GRIP_EXCEEDED",
          message: `Cornering lateral acceleration (${latAccel.toFixed(2)} m/s^2 = ${(latAccel / sceneGravity).toFixed(2)}g) exceeds tire friction limit (mu*g = ${(mu * sceneGravity).toFixed(2)} m/s^2) without drift/skid.`,
          frame_time: times[i],
          expected: `<= ${(mu * sceneGravity).toFixed(2)} m/s^2 (${mu}g)`,
          actual: `${latAccel.toFixed(2)} m/s^2 (${(latAccel / sceneGravity).toFixed(2)}g)`,
          formula: "a_lat = v^2 / R <= mu * g",
        });
      }

      if (Math.abs(longAccel) > 2.5 && !pitchReported) {
        const pitch = pitches[i];
        if ((longAccel > 2.5 && pitch < -1.5) || (longAccel < -2.5 && pitch > 1.5)) {
          pitchReported = true;
          const action = longAccel > 0 ? "acceleration" : "braking";
          const expectedDir = longAccel > 0 ? "nose-up squat (>= 0°)" : "nose-down dive (<= 0°)";
          issues.push({
            severity: "WARNING",
            category: "vehicle",
            code: "INVERTED_SUSPENSION_LOAD_TRANSFER_PITCH",
            message: `Chassis pitched ${pitch.toFixed(1)}° during longitudinal ${action} (${longAccel.toFixed(2)} m/s^2); inertial load transfer requires ${expectedDir}.`,
            frame_time: times[i],
            expected: expectedDir,
            actual: `pitch = ${pitch.toFixed(1)}°`,
            formula: "Delta_F_z = (m * a_x * h_CoM) / L_wheelbase",
          });
        }
      }

      if (entityType === "two_wheeled_vehicle" && latAccel > 1.5 && !rollReported) {
        const expectedLeanDeg = (Math.atan2(latAccel, sceneGravity) * 180) / Math.PI;
        const actualLeanMag = Math.abs(rolls[i]);
        if (actualLeanMag < 0.5 * expectedLeanDeg || rolls[i] * yawRate < 0) {
          rollReported = true;
          issues.push({
            severity: "ERROR",
            category: "vehicle",
            code: "TWO_WHEEL_TURN_LEAN_VIOLATION",
            message: `Two-wheeled vehicle in turn (a_lat=${latAccel.toFixed(2)} m/s^2) has roll=${rolls[i].toFixed(1)}°, but requires inward lean of ${expectedLeanDeg.toFixed(1)}° for roll equilibrium.`,
            frame_time: times[i],
            expected: `Inward lean ~${expectedLeanDeg.toFixed(1)}°`,
            actual: `${rolls[i].toFixed(1)}°`,
            formula: "theta_lean = arctan(v^2 / (R * g))",
          });
        }
      }
    }
  } else if (entityType === "aircraft") {
    const stallSpeed = Number(params.stallSpeedMps ?? 15.0);
    let bankReported = false;
    let stallReported = false;
    for (let i = 1; i < n - 1; i++) {
      const vHoriz = vecHorizNorm(velocities[i]);
      const yawRate = yawRatesRad[i];
      const latAccel = vHoriz * Math.abs(yawRate);
      const expectedBankDeg = (Math.atan2(latAccel, sceneGravity) * 180) / Math.PI;
      const actualBankDeg = Math.abs(rolls[i]);

      if (vHoriz < stallSpeed && !(frames[i].grounded ?? false) && !stallReported) {
        stallReported = true;
        issues.push({
          severity: "ERROR",
          category: "vehicle",
          code: "AIRCRAFT_BELOW_STALL_SPEED",
          message: `Fixed-wing aircraft airborne at ${vHoriz.toFixed(1)} m/s below minimum stall speed (${stallSpeed.toFixed(1)} m/s).`,
          frame_time: times[i],
          expected: `>= ${stallSpeed.toFixed(1)} m/s`,
          actual: `${vHoriz.toFixed(1)} m/s`,
          formula: "v >= v_stall * sqrt(1 / cos(phi_bank))",
        });
      }

      if (
        expectedBankDeg > 10.0 &&
        Math.abs(actualBankDeg - expectedBankDeg) > 12.0 &&
        !bankReported
      ) {
        bankReported = true;
        issues.push({
          severity: "WARNING",
          category: "vehicle",
          code: "UNCOORDINATED_AIRCRAFT_TURN_BANK",
          message: `Aircraft turning with centripetal accel ${latAccel.toFixed(2)} m/s^2 has bank angle ${actualBankDeg.toFixed(1)}°, expected coordinated bank of ${expectedBankDeg.toFixed(1)}°.`,
          frame_time: times[i],
          expected: `${expectedBankDeg.toFixed(1)}°`,
          actual: `${actualBankDeg.toFixed(1)}°`,
          formula: "phi_bank = arctan((v * psi_dot) / g)",
        });
      }
    }
  } else if (entityType === "rotorcraft") {
    let pitchCouplingReported = false;
    for (let i = 1; i < n - 1; i++) {
      if (frames[i].grounded ?? false) continue;
      const yawRad = (yaws[i] * Math.PI) / 180.0;
      const fwdX = Math.cos(yawRad);
      const fwdZ = Math.sin(yawRad);
      const acc = accelerations[i];
      const aFwd = acc[0] * fwdX + acc[2] * fwdZ;
      const denom = Math.max(acc[1] + sceneGravity, 1.0);
      const expectedPitchDeg = -(Math.atan2(aFwd, denom) * 180) / Math.PI;
      const actualPitchDeg = pitches[i];

      if (
        Math.abs(aFwd) > 1.8 &&
        Math.abs(actualPitchDeg - expectedPitchDeg) > 8.0 &&
        !pitchCouplingReported
      ) {
        pitchCouplingReported = true;
        issues.push({
          severity: "ERROR",
          category: "vehicle",
          code: "ROTORCRAFT_THRUST_TILT_DECOUPLED",
          message: `Quadcopter/helicopter horizontal acceleration (${aFwd.toFixed(2)} m/s^2) requires pitch=${expectedPitchDeg.toFixed(1)}° (nose-down to accelerate, nose-up flare to brake), but model pitch is ${actualPitchDeg.toFixed(1)}°.`,
          frame_time: times[i],
          expected: `${expectedPitchDeg.toFixed(1)}°`,
          actual: `${actualPitchDeg.toFixed(1)}°`,
          formula: "theta_pitch = -arctan(a_forward / (g + a_y))",
        });
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Public API & CLI Entry Point
// ---------------------------------------------------------------------------

export function verifyScene(data: SceneSpec, strict = false): VerificationResult {
  const scene = data.scene ?? {};
  const dimension = String(scene.dimension ?? "3d").toLowerCase();
  const upAxis = String(scene.upAxis ?? "y").toLowerCase();
  const sceneGravity = Number(scene.gravity ?? 9.80665);
  const metersPerUnit = Number(scene.metersPerUnit ?? 1.0);
  const groundLevel = Number(scene.groundLevel ?? 0.0) * metersPerUnit;

  const entityReports: EntityReport[] = [];

  for (const ent of data.entities ?? []) {
    const entId = String(ent.id ?? "unnamed_entity");
    const entType = String(ent.type ?? "rigid_body").toLowerCase();
    const params = ent.params ?? {};
    const rig = ent.rig ?? {};
    const frames = ent.frames ?? [];

    const issues: Issue[] = [];
    const metrics: Record<string, unknown> = {};

    const times = frames.map((f, idx) => Number(f.t ?? idx / 30.0));
    const positions = frames.map((f) =>
      to3DMeters(f.position ?? [0, 0, 0], dimension, upAxis, metersPerUnit)
    );

    // 0. Verify 2D/3D Rigging, Skinning & Animation Standards
    verifyRigAndAnimationStandards(
      rig,
      frames,
      times,
      dimension,
      upAxis,
      metersPerUnit,
      issues,
      metrics
    );

    // 1. Verify Universal Classical Mechanics
    const { velocities, accelerations } = verifyClassicalMechanics(
      frames,
      positions,
      times,
      entType,
      sceneGravity,
      groundLevel,
      dimension,
      upAxis,
      metersPerUnit,
      issues,
      metrics
    );

    // 2-4. Domain-Specific Biomechanics & Vehicle Dynamics
    if (entType === "human") {
      verifyHumanBiomechanics(
        frames,
        positions,
        velocities,
        times,
        params,
        sceneGravity,
        groundLevel,
        dimension,
        upAxis,
        metersPerUnit,
        issues,
        metrics
      );
    } else if (["quadruped", "bird", "fish"].includes(entType)) {
      verifyAnimalLocomotion(
        entType,
        frames,
        positions,
        velocities,
        times,
        params,
        metersPerUnit,
        issues,
        metrics
      );
    } else if (
      ["wheeled_vehicle", "two_wheeled_vehicle", "aircraft", "rotorcraft", "watercraft"].includes(
        entType
      )
    ) {
      verifyVehicleDynamics(
        entType,
        frames,
        velocities,
        accelerations,
        times,
        params,
        sceneGravity,
        issues
      );
    }

    let penalty = 0.0;
    let hasError = false;
    let hasWarning = false;
    for (const iss of issues) {
      if (iss.severity === "ERROR") {
        penalty += 25.0;
        hasError = true;
      } else if (iss.severity === "WARNING") {
        penalty += 10.0;
        hasWarning = true;
      } else {
        penalty += 2.0;
      }
    }

    const score = Math.max(0.0, Number((100.0 - penalty).toFixed(1)));
    const passed = strict ? !hasError && !hasWarning : !hasError;
    const duration =
      times.length >= 2 ? Number((times[times.length - 1] - times[0]).toFixed(4)) : 0.0;

    entityReports.push({
      entity_id: entId,
      entity_type: entType,
      frame_count: frames.length,
      duration_sec: duration,
      score,
      passed,
      metrics,
      issues,
    });
  }

  const overallScore =
    entityReports.length > 0
      ? Number(
          (entityReports.reduce((acc, r) => acc + r.score, 0) / entityReports.length).toFixed(1)
        )
      : 100.0;
  const overallPassed = entityReports.every((r) => r.passed);

  return {
    overall_passed: overallPassed,
    overall_physics_score: overallScore,
    entity_count: entityReports.length,
    entities: entityReports,
  };
}

export function formatMarkdownReport(result: VerificationResult): string {
  const lines: string[] = [];
  const statusStr = result.overall_passed ? "PASSED" : "FAILED";
  lines.push(`# 2D/3D Physics, Rigging & Biomechanics Verification Report: **${statusStr}**`);
  lines.push(`- **Overall Realism Score**: \`${result.overall_physics_score} / 100\``);
  lines.push(`- **Entities Analyzed**: \`${result.entity_count}\`\n`);

  for (const ent of result.entities) {
    const badge = ent.passed ? "PASS" : "FAIL";
    lines.push(
      `## [${badge}] Entity \`${ent.entity_id}\` (type: \`${ent.entity_type}\`, score: \`${ent.score}/100\`)`
    );
    lines.push(`- **Frames**: \`${ent.frame_count}\` | **Duration**: \`${ent.duration_sec}s\``);
    const metricEntries = Object.entries(ent.metrics);
    if (metricEntries.length > 0) {
      const pairs = metricEntries.map(([k, v]) => `\`${k}=${JSON.stringify(v)}\``).join(", ");
      lines.push(`- **Computed Metrics**: ${pairs}`);
    }
    if (ent.issues.length === 0) {
      lines.push(
        "- **Findings**: All rigging, animation, physical, biomechanical, and dynamical standards satisfied.\n"
      );
    } else {
      lines.push("- **Detected Standards, Physics & Movement Violations**:");
      for (const iss of ent.issues) {
        const tStr =
          typeof iss.frame_time === "number" ? ` @ t=${iss.frame_time.toFixed(3)}s` : "";
        lines.push(
          `  - **[${iss.severity}] \`${iss.code}\` (${iss.category}${tStr})**: ${iss.message}`
        );
        const details: string[] = [];
        if (iss.expected) details.push(`Expected: \`${iss.expected}\``);
        if (iss.actual) details.push(`Actual: \`${iss.actual}\``);
        if (iss.formula) details.push(`Law/Formula: \`${iss.formula}\``);
        if (details.length > 0) {
          lines.push(`    - ${details.join(" | ")}`);
        }
      }
      lines.push("");
    }
  }
  return lines.join("\n");
}

// CLI Runner
const isMain =
  process.argv[1] &&
  path.resolve(process.argv[1]) === path.resolve(fileURLToPath(import.meta.url));

if (isMain) {
  const args = process.argv.slice(2);
  const jsonFlag = args.includes("--json");
  const strictFlag = args.includes("--strict");
  const positional = args.filter((a) => !a.startsWith("--"));

  if (positional.length === 0) {
    console.error("Usage: node scripts/verify_physics.ts <input.json> [--json] [--strict]");
    process.exit(2);
  }

  const inputPath = path.resolve(positional[0]);
  if (!fs.existsSync(inputPath)) {
    console.error(`Error: Input file '${inputPath}' not found.`);
    process.exit(2);
  }

  const data = JSON.parse(fs.readFileSync(inputPath, "utf-8")) as SceneSpec;
  const result = verifyScene(data, strictFlag);

  if (jsonFlag) {
    console.log(JSON.stringify(result, null, 2));
  } else {
    console.log(formatMarkdownReport(result));
  }

  process.exit(result.overall_passed ? 0 : 1);
}
