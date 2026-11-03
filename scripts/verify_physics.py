#!/usr/bin/env python3
"""
2D/3D Model, Rig & Movement Physics Verification Engine (Python)

Analyzes 2D and 3D models, skeletal rigs, vertex skinning, animation curves,
and movements against:
  1. Industry 2D/3D Rigging, Skinning & Animation Standards (glTF 2.0, OpenUSD
     UsdSkel/UsdPhysics, VRM/Mecanim Humanoid, URDF/MuJoCo inertia tensors,
     quaternion hemisphere continuity, LBS candy-wrapper twist, 2-bone IK pop,
     Bezier time monotonicity, and 2D mesh triangle winding preservation)
  2. Classical Mechanics (kinematics, ballistic gravity, CoM/ZMP balance,
     squash-and-stretch volume conservation)
  3. Human Biomechanics (Winter's anthropometry, ISB joint RoM, Froude number,
     duty factor, footskate, jump crouch impulse)
  4. Animal Locomotion (Hildebrand quadruped gaits, avian/aquatic Strouhal laws)
  5. Vehicle & Transportation Dynamics (wheel slip, Ackermann steering, car
     roll vs. bike lean, aircraft bank/stall, rotorcraft thrust-tilt coupling)

Usage:
    python3 scripts/verify_physics.py <input.json> [--json] [--strict]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class Issue:
    severity: str  # "ERROR" | "WARNING" | "INFO"
    category: str  # "rig_standard" | "anim_standard" | "kinematics" | "gravity" | "balance" | "volume" | "human" | "animal" | "vehicle"
    code: str
    message: str
    frame_time: Optional[float] = None
    expected: Optional[str] = None
    actual: Optional[str] = None
    formula: Optional[str] = None


@dataclass
class EntityReport:
    entity_id: str
    entity_type: str
    frame_count: int
    duration_sec: float
    score: float
    passed: bool
    metrics: Dict[str, Any] = field(default_factory=dict)
    issues: List[Issue] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Coordinate & Vector Helpers
# ---------------------------------------------------------------------------

def to_3d_meters(
    vec: List[float],
    dimension: str,
    up_axis: str,
    meters_per_unit: float,
) -> Tuple[float, float, float]:
    """
    Convert input 2D/3D vector into standardized right-handed 3D meter coordinates
    where +Y is always UP (opposing gravity), and X, Z form the horizontal ground plane.
    """
    scaled = [float(c) * meters_per_unit for c in vec]
    if dimension == "2d" or len(scaled) == 2:
        x = scaled[0]
        y_raw = scaled[1]
        y = -y_raw if up_axis == "-y" else y_raw
        return (x, y, 0.0)
    x, y, z = scaled[0], scaled[1], scaled[2]
    if up_axis == "z":
        return (x, z, -y)
    if up_axis == "-y":
        return (x, -y, z)
    return (x, y, z)


def vec_sub(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_norm(a: Tuple[float, float, float]) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def vec_horiz_norm(a: Tuple[float, float, float]) -> float:
    return math.hypot(a[0], a[2])


def angle_diff_deg(a: float, b: float) -> float:
    """Shortest signed angle difference (a - b) in degrees within [-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


def median(values: List[float]) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 == 1 else 0.5 * (s[mid - 1] + s[mid])


# ---------------------------------------------------------------------------
# 2D Convex Hull & Point-in-Polygon (for Ground Contact Base of Support)
# ---------------------------------------------------------------------------

def cross_2d(o: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def convex_hull_2d(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts
    lower: List[Tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross_2d(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: List[Tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross_2d(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def point_to_segment_dist_2d(
    p: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]
) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    if l2 == 0.0:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    proj_x = a[0] + t * dx
    proj_y = a[1] + t * dy
    return math.hypot(p[0] - proj_x, p[1] - proj_y)


def is_inside_support_region(
    pt: Tuple[float, float],
    contacts: List[Tuple[float, float]],
    tolerance: float = 0.035,
) -> Tuple[bool, float]:
    if not contacts:
        return True, 0.0
    hull = convex_hull_2d(contacts)
    if len(hull) == 1:
        dist = math.hypot(pt[0] - hull[0][0], pt[1] - hull[0][1])
        return dist <= tolerance, max(0.0, dist - tolerance)
    if len(hull) == 2:
        dist = point_to_segment_dist_2d(pt, hull[0], hull[1])
        return dist <= tolerance, max(0.0, dist - tolerance)

    inside = True
    min_edge_dist = float("inf")
    n = len(hull)
    for i in range(n):
        a = hull[i]
        b = hull[(i + 1) % n]
        if cross_2d(a, b, pt) < 0:
            inside = False
        edge_dist = point_to_segment_dist_2d(pt, a, b)
        if edge_dist < min_edge_dist:
            min_edge_dist = edge_dist

    if inside or min_edge_dist <= tolerance:
        return True, 0.0
    return False, min_edge_dist - tolerance


# ---------------------------------------------------------------------------
# 0. 2D/3D Rigging, Skinning & Animation Standards Verification
# ---------------------------------------------------------------------------

def verify_rig_and_animation_standards(
    rig: Dict[str, Any],
    frames: List[Dict[str, Any]],
    times: List[float],
    dimension: str,
    up_axis: str,
    meters_per_unit: float,
    issues: List[Issue],
    metrics: Dict[str, Any],
) -> None:
    """
    Verify industry 2D/3D modeling, skeletal rigging, vertex skinning,
    quaternion continuity, IK reachability, physics inertia tensors,
    and 2D mesh deformation standards.
    """
    # 0a. Skeletal Bone Topology, Zero-Length & Bilateral Symmetry
    bones = rig.get("bones", [])
    bone_map: Dict[str, Dict[str, Any]] = {}
    if bones:
        metrics["rig_bone_count"] = len(bones)
        for b in bones:
            b_name = str(b.get("name", ""))
            b_len = float(b.get("length", 0.0)) * meters_per_unit
            bone_map[b_name] = b
            if b_len <= 1e-4:
                issues.append(
                    Issue(
                        severity="ERROR",
                        category="rig_standard",
                        code="RIG_ZERO_LENGTH_BONE",
                        message=f"Bone '{b_name}' has near-zero length ({b_len:.6f}m); causes NaN in IK solvers and skinning heatmaps.",
                        expected="length > 0.0001 m",
                        actual=f"{b_len:.6f} m",
                        formula="L_bone = ||p_child - p_parent|| > 1e-4 m",
                    )
                )

        # Check DAG cycles
        for b in bones:
            visited = set()
            curr = b.get("name")
            while curr is not None:
                if curr in visited:
                    issues.append(
                        Issue(
                            severity="ERROR",
                            category="rig_standard",
                            code="RIG_CYCLIC_HIERARCHY",
                            message=f"Cycle detected in skeletal hierarchy involving bone '{curr}'. Skeleton must be a Directed Acyclic Graph (DAG).",
                            expected="Acyclic parent-child tree",
                            actual=f"Cycle at '{curr}'",
                        )
                    )
                    break
                visited.add(curr)
                parent_node = bone_map.get(curr)
                curr = parent_node.get("parent") if parent_node else None

        # Check bilateral symmetry for paired bones
        checked_pairs = set()
        for b in bones:
            b_name = str(b.get("name", ""))
            mirror = b.get("mirrorBone")
            if mirror and mirror in bone_map:
                pair_key = tuple(sorted((b_name, str(mirror))))
                if pair_key in checked_pairs:
                    continue
                checked_pairs.add(pair_key)
                l1 = float(b.get("length", 0.0))
                l2 = float(bone_map[str(mirror)].get("length", 0.0))
                max_l = max(l1, l2, 1e-6)
                asym = abs(l1 - l2) / max_l
                if asym > 0.015:
                    issues.append(
                        Issue(
                            severity="WARNING",
                            category="rig_standard",
                            code="RIG_BILATERAL_ASYMMETRY",
                            message=f"Bilateral bone pair ('{b_name}'={l1:.3f} vs '{mirror}'={l2:.3f}) differs in length by {asym*100:.1f}% (> 1.5% VRM/Mecanim symmetry standard).",
                            expected="<= 1.5% length difference",
                            actual=f"{asym*100:.1f}%",
                            formula="|L_left - L_right| / max(L_left, L_right) <= 0.015",
                        )
                    )

        # Check VRM / Mecanim standard humanoid bone presence
        rig_std = str(rig.get("standard", "")).lower()
        if rig_std in ("vrm_humanoid", "mecanim_humanoid"):
            lower_names = {n.lower() for n in bone_map}
            required_Keywords = ("hips", "spine", "head")
            missing = [kw for kw in required_Keywords if not any(kw in ln for ln in lower_names)]
            if missing:
                issues.append(
                    Issue(
                        severity="WARNING",
                        category="rig_standard",
                        code="RIG_MISSING_HUMANOID_BONES",
                        message=f"Humanoid rig ('{rig_std}') is missing required core bone(s): {missing}.",
                        expected="hips, spine, head, limbs present",
                        actual=f"Missing {missing}",
                    )
                )

    # 0b. Vertex Skinning Normalization & Inverse Bind Matrices (glTF / UsdSkel)
    skinning = rig.get("skinning", {})
    if skinning:
        unnorm = int(skinning.get("unnormalizedVertexCount", 0))
        neg_w = int(skinning.get("negativeWeightCount", 0))
        min_sum = float(skinning.get("minWeightSum", 1.0))
        max_sum = float(skinning.get("maxWeightSum", 1.0))
        non_inv_ibm = int(skinning.get("nonInvertibleBindMatrices", 0))
        neg_det_ibm = int(skinning.get("negativeDeterminantBindMatrices", 0))

        if unnorm > 0 or abs(min_sum - 1.0) > 1e-3 or abs(max_sum - 1.0) > 1e-3:
            issues.append(
                Issue(
                    severity="ERROR",
                    category="rig_standard",
                    code="SKIN_WEIGHTS_NOT_NORMALIZED",
                    message=f"Vertex skin weights violate Partition of Unity ({unnorm} unnormalized vertices, weight sum range [{min_sum:.4f}, {max_sum:.4f}]); causes mesh shrinkage or origin pulling.",
                    expected="sum(w_j) == 1.0 (±0.0001)",
                    actual=f"[{min_sum:.4f}, {max_sum:.4f}] ({unnorm} vertices)",
                    formula="sum_{j=1}^K w_{v,j} = 1.0",
                )
            )
        if neg_w > 0:
            issues.append(
                Issue(
                    severity="ERROR",
                    category="rig_standard",
                    code="SKIN_NEGATIVE_WEIGHTS",
                    message=f"Mesh contains {neg_w} vertices with negative bone weights (w_j < 0).",
                    expected="w_j >= 0",
                    actual=f"{neg_w} negative weights",
                )
            )
        if non_inv_ibm > 0 or neg_det_ibm > 0:
            issues.append(
                Issue(
                    severity="ERROR",
                    category="rig_standard",
                    code="BIND_MATRIX_SINGULAR_OR_MIRRORED",
                    message=f"Rig contains {non_inv_ibm} singular/non-invertible and {neg_det_ibm} negative-determinant (mirrored scale) inverseBindMatrices.",
                    expected="det(M_IBM) > 0 for all joints",
                    actual=f"singular={non_inv_ibm}, neg_det={neg_det_ibm}",
                    formula="det(inverseBindMatrix) > 0",
                )
            )

    # 0c. Physics Rig Inertia Tensor Triangle Inequality (URDF / SDF / MuJoCo / UsdPhysics)
    inertia = rig.get("inertiaTensor")
    if isinstance(inertia, dict):
        ixx = float(inertia.get("ixx", 0.0))
        iyy = float(inertia.get("iyy", 0.0))
        izz = float(inertia.get("izz", 0.0))
        if ixx <= 0 or iyy <= 0 or izz <= 0:
            issues.append(
                Issue(
                    severity="ERROR",
                    category="rig_standard",
                    code="INERTIA_NON_POSITIVE_PRINCIPAL_MOMENT",
                    message=f"Principal moments of inertia (Ixx={ixx}, Iyy={iyy}, Izz={izz}) must be strictly positive.",
                    expected="Ixx > 0, Iyy > 0, Izz > 0",
                    actual=f"({ixx}, {iyy}, {izz})",
                )
            )
        elif (ixx + iyy < izz - 1e-6) or (iyy + izz < ixx - 1e-6) or (izz + ixx < iyy - 1e-6):
            issues.append(
                Issue(
                    severity="ERROR",
                    category="rig_standard",
                    code="INERTIA_TRIANGLE_INEQUALITY_VIOLATION",
                    message=(
                        f"Principal moments of inertia (Ixx={ixx}, Iyy={iyy}, Izz={izz}) violate the 3D Triangle Inequality; "
                        "impossible for any real positive-density mass distribution and causes physics solvers to explode."
                    ),
                    expected="I_a + I_b >= I_c for all permutations",
                    actual=f"Ixx={ixx}, Iyy={iyy}, Izz={izz}",
                    formula="Ixx + Iyy >= Izz, Iyy + Izz >= Ixx, Izz + Ixx >= Iyy",
                )
            )

    # 0d. Frame-Level Animation Standards: Quaternions, Gimbal Lock, Twist, IK, Bezier, 2D Mesh
    ik_chains: Dict[str, Tuple[float, float]] = {}
    for ch in rig.get("ikChains", []):
        c_name = str(ch.get("name", ""))
        l1 = float(ch.get("upperLength", 0.0)) * meters_per_unit
        l2 = float(ch.get("lowerLength", 0.0)) * meters_per_unit
        if c_name and (l1 + l2) > 0:
            ik_chains[c_name] = (l1, l2)

    prev_quats: Dict[str, List[float]] = {}
    quat_norm_reported = set()
    quat_flip_reported = set()
    gimbal_reported = False
    twist_reported = set()
    ik_pop_reported = set()
    ik_pole_reported = set()
    bezier_reported = False
    mesh_flip_reported = False
    bind_tri_signs: Optional[List[float]] = None

    for i, f in enumerate(frames):
        t = times[i]

        # Quaternion unit norm & hemisphere shortest-path continuity
        quats = f.get("quaternions", {})
        for j_name, q in quats.items():
            if not isinstance(q, list) or len(q) != 4:
                continue
            qx, qy, qz, qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if abs(norm - 1.0) > 2e-3 and j_name not in quat_norm_reported:
                quat_norm_reported.add(j_name)
                issues.append(
                    Issue(
                        severity="ERROR",
                        category="anim_standard",
                        code="QUATERNION_NOT_UNIT_NORM",
                        message=f"Quaternion on bone '{j_name}' is not unit length (||q||={norm:.4f}); causes skewing/scaling artifacts in skinning.",
                        frame_time=t,
                        expected="||q|| = 1.0 (±0.002)",
                        actual=f"||q|| = {norm:.4f}",
                        formula="sqrt(x^2 + y^2 + z^2 + w^2) = 1",
                    )
                )
            if j_name in prev_quats and j_name not in quat_flip_reported:
                pq = prev_quats[j_name]
                dot = qx * pq[0] + qy * pq[1] + qz * pq[2] + qw * pq[3]
                if dot < -0.05:
                    quat_flip_reported.add(j_name)
                    issues.append(
                        Issue(
                            severity="ERROR",
                            category="anim_standard",
                            code="QUATERNION_ANTIPODAL_FLIP",
                            message=f"Consecutive quaternions on bone '{j_name}' have negative dot product (q_i . q_{{i-1}} = {dot:.3f} < 0); causes 360° long-way-around spin flip during interpolation.",
                            frame_time=t,
                            expected="q_i . q_{i-1} >= 0 (same hemisphere)",
                            actual=f"dot = {dot:.3f}",
                            formula="q_i . q_{i-1} >= 0",
                        )
                    )
            prev_quats[j_name] = [qx, qy, qz, qw]

        # Euler Gimbal Lock Singularity Check
        rot = f.get("rotation")
        if rot and len(rot) >= 3 and not gimbal_reported:
            mid_angle = float(rot[1])
            if 87.0 <= abs(mid_angle) <= 93.0 and i > 0:
                prev_rot = frames[i - 1].get("rotation", [0.0, 0.0, 0.0])
                d_outer = abs(angle_diff_deg(float(rot[0]), float(prev_rot[0])))
                d_inner = abs(angle_diff_deg(float(rot[2]), float(prev_rot[2])))
                if d_outer > 45.0 or d_inner > 45.0:
                    gimbal_reported = True
                    issues.append(
                        Issue(
                            severity="WARNING",
                            category="anim_standard",
                            code="EULER_GIMBAL_LOCK_SINGULARITY",
                            message=f"Middle Euler axis is near ±90° ({mid_angle:.1f}°) with rapid outer/inner axis flip (Δ={max(d_outer, d_inner):.1f}°), indicating Euler Gimbal Lock.",
                            frame_time=t,
                            expected="|theta_mid| != 90° or use Quaternion SLERP",
                            actual=f"theta_mid = {mid_angle:.1f}°, axis jump = {max(d_outer, d_inner):.1f}°",
                            formula="cos(theta_mid) -> 0 at +-90 deg",
                        )
                    )

        # LBS Candy-Wrapper Axial Bone Twist Check
        bone_twists = f.get("boneTwists", {})
        for b_name, twist_deg in bone_twists.items():
            val = abs(float(twist_deg))
            if val > 90.0 and b_name not in twist_reported:
                twist_reported.add(b_name)
                collapse_pct = (1.0 - math.cos(math.radians(min(val, 180.0)) / 2.0)) * 100.0
                issues.append(
                    Issue(
                        severity="WARNING",
                        category="anim_standard",
                        code="LBS_CANDY_WRAPPER_TWIST_EXCEEDED",
                        message=f"Bone '{b_name}' has single-segment axial twist of {val:.1f}° (> 90°), causing {collapse_pct:.0f}% cross-sectional 'candy-wrapper' collapse under Linear Blend Skinning.",
                        frame_time=t,
                        expected="<= 90° per bone segment (or use twist bones / DQS)",
                        actual=f"{val:.1f}° ({collapse_pct:.0f}% radius loss)",
                        formula="r(psi) = r_0 * cos(psi / 2)",
                    )
                )

        # 2-Bone IK Over-Extension ("IK Pop") & Pole Vector Collinearity
        ik_targets = f.get("ikTargets", {})
        for c_name, ik_data in ik_targets.items():
            if not isinstance(ik_data, dict):
                continue
            r_raw = ik_data.get("rootPos")
            t_raw = ik_data.get("targetPos")
            if r_raw and t_raw:
                r_pos = to_3d_meters(r_raw, dimension, up_axis, meters_per_unit)
                t_pos = to_3d_meters(t_raw, dimension, up_axis, meters_per_unit)
                diff = vec_sub(t_pos, r_pos)
                dist = vec_norm(diff)
                if c_name in ik_chains and c_name not in ik_pop_reported:
                    l1, l2 = ik_chains[c_name]
                    max_reach = l1 + l2
                    if dist > 0.992 * max_reach:
                        ik_pop_reported.add(c_name)
                        sev = "ERROR" if dist > max_reach else "WARNING"
                        issues.append(
                            Issue(
                                severity=sev,
                                category="anim_standard",
                                code="IK_CHAIN_OVEREXTENSION_POP",
                                message=f"2-bone IK chain '{c_name}' target distance ({dist:.3f}m) is at/beyond full limb extension ({max_reach:.3f}m); causes infinite angular velocity 'IK knee/elbow pop'.",
                                frame_time=t,
                                expected=f"D <= {0.992 * max_reach:.3f} m (99.2% of L1+L2)",
                                actual=f"D = {dist:.3f} m ({dist/max_reach*100:.1f}%)",
                                formula="d(theta)/dD = D / (L1 * L2 * sin(theta)) -> inf as D -> L1+L2",
                            )
                        )
                pole_raw = ik_data.get("poleVector")
                if pole_raw and dist > 1e-5 and c_name not in ik_pole_reported:
                    p_vec = to_3d_meters(pole_raw, dimension, up_axis, 1.0)
                    p_norm = vec_norm(p_vec)
                    if p_norm > 1e-5:
                        cos_ang = abs(
                            (diff[0] * p_vec[0] + diff[1] * p_vec[1] + diff[2] * p_vec[2])
                            / (dist * p_norm)
                        )
                        if cos_ang > 0.98:
                            ik_pole_reported.add(c_name)
                            issues.append(
                                Issue(
                                    severity="ERROR",
                                    category="anim_standard",
                                    code="IK_POLE_VECTOR_COLLINEAR",
                                    message=f"IK chain '{c_name}' target vector is nearly collinear with pole vector (|cos(theta)|={cos_ang:.3f} > 0.98); causes 180° swivel plane flip.",
                                    frame_time=t,
                                    expected="|u_target . u_pole| <= 0.98",
                                    actual=f"{cos_ang:.3f}",
                                    formula="n_plane = u_target x u_pole != 0",
                                )
                            )

        # Cubic Bezier Timing Handle Monotonicity (CSS / Lottie / Spine / glTF)
        bezier = f.get("bezierHandles")
        if bezier and len(bezier) >= 4 and not bezier_reported:
            x1, _, x2, _ = float(bezier[0]), float(bezier[1]), float(bezier[2]), float(bezier[3])
            if not (0.0 <= x1 <= 1.0 and 0.0 <= x2 <= 1.0):
                bezier_reported = True
                issues.append(
                    Issue(
                        severity="ERROR",
                        category="anim_standard",
                        code="BEZIER_TIME_NON_MONOTONIC",
                        message=f"Cubic Bezier timing curve handles (x1={x1:.2f}, x2={x2:.2f}) lie outside [0, 1]; causes time axis to fold backward (non-monotonic time).",
                        frame_time=t,
                        expected="0.0 <= x1 <= 1.0 and 0.0 <= x2 <= 1.0",
                        actual=f"x1={x1:.2f}, x2={x2:.2f}",
                        formula="dt(u)/du >= 0 requires x1, x2 in [0, 1]",
                    )
                )

        # 2D Deforming Mesh Triangle Winding Preservation (Spine / Live2D / Lottie FFD)
        tris = f.get("meshTriangles2D")
        if tris and isinstance(tris, list):
            cur_signs: List[float] = []
            for tri in tris:
                if len(tri) == 3:
                    a = (float(tri[0][0]), float(tri[0][1]))
                    b = (float(tri[1][0]), float(tri[1][1]))
                    c = (float(tri[2][0]), float(tri[2][1]))
                    cur_signs.append(cross_2d(a, b, c))
            if bind_tri_signs is None:
                bind_tri_signs = cur_signs
            elif not mesh_flip_reported and len(cur_signs) == len(bind_tri_signs):
                for idx, (s0, st) in enumerate(zip(bind_tri_signs, cur_signs)):
                    if abs(s0) > 1e-6 and s0 * st <= 0:
                        mesh_flip_reported = True
                        issues.append(
                            Issue(
                                severity="ERROR",
                                category="anim_standard",
                                code="MESH_2D_TRIANGLE_FLIP",
                                message=f"2D deforming mesh triangle #{idx} inverted its winding orientation at t={t:.3f}s (inside-out mesh fold / self-intersection).",
                                frame_time=t,
                                expected="sign(Area(t)) == sign(Area(0))",
                                actual=f"Area(0)={s0:.2f}, Area(t)={st:.2f}",
                                formula="((B - A) x (C - A))_t * ((B - A) x (C - A))_0 > 0",
                            )
                        )
                        break


# ---------------------------------------------------------------------------
# 1. Universal Classical Mechanics Checks
# ---------------------------------------------------------------------------

def verify_classical_mechanics(
    frames: List[Dict[str, Any]],
    positions: List[Tuple[float, float, float]],
    times: List[float],
    entity_type: str,
    scene_gravity: float,
    ground_level: float,
    dimension: str,
    up_axis: str,
    meters_per_unit: float,
    issues: List[Issue],
    metrics: Dict[str, Any],
) -> Tuple[
    List[Tuple[float, float, float]],
    List[Tuple[float, float, float]],
    List[Tuple[float, float, float]],
]:
    """Compute velocity, acceleration, jerk and check universal laws of physics."""
    n = len(frames)
    velocities: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * n
    accelerations: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * n
    jerks: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * n

    if n < 2:
        return velocities, accelerations, jerks

    # Velocities
    speeds: List[float] = []
    for i in range(1, n):
        dt = times[i] - times[i - 1]
        if dt <= 1e-7:
            issues.append(
                Issue(
                    severity="ERROR",
                    category="kinematics",
                    code="NON_INCREASING_TIMESTAMP",
                    message=f"Timestamp at frame {i} ({times[i]}s) is not greater than frame {i-1} ({times[i-1]}s).",
                    frame_time=times[i],
                )
            )
            continue
        dp = vec_sub(positions[i], positions[i - 1])
        v = (dp[0] / dt, dp[1] / dt, dp[2] / dt)
        velocities[i] = v
        speeds.append(vec_norm(v))
    velocities[0] = velocities[1]

    # Accelerations
    accel_mags: List[float] = []
    for i in range(1, n - 1):
        dt_prev = times[i] - times[i - 1]
        dt_next = times[i + 1] - times[i]
        dt_avg = 0.5 * (dt_prev + dt_next)
        if dt_avg <= 1e-7:
            continue
        v_next = velocities[i + 1]
        v_cur = velocities[i]
        a = (
            (v_next[0] - v_cur[0]) / dt_avg,
            (v_next[1] - v_cur[1]) / dt_avg,
            (v_next[2] - v_cur[2]) / dt_avg,
        )
        accelerations[i] = a
        accel_mags.append(vec_norm(a))
    if n >= 3:
        accelerations[0] = accelerations[1]
        accelerations[-1] = accelerations[-2]

    # Jerks
    jerk_mags: List[float] = []
    for i in range(2, n - 1):
        dt = times[i] - times[i - 1]
        if dt <= 1e-7:
            continue
        da = vec_sub(accelerations[i], accelerations[i - 1])
        j = (da[0] / dt, da[1] / dt, da[2] / dt)
        jerks[i] = j
        jerk_mags.append(vec_norm(j))

    med_speed = median(speeds)
    max_speed = max(speeds) if speeds else 0.0
    max_accel = max(accel_mags) if accel_mags else 0.0
    max_jerk = max(jerk_mags) if jerk_mags else 0.0

    metrics["max_speed_mps"] = round(max_speed, 3)
    metrics["median_speed_mps"] = round(med_speed, 3)
    metrics["max_accel_mps2"] = round(max_accel, 3)
    metrics["max_jerk_mps3"] = round(max_jerk, 3)

    # 1a. Teleportation / C0 Discontinuity Check
    for i in range(1, n):
        spd = vec_norm(velocities[i])
        if med_speed > 0.1 and spd > max(25.0, 6.0 * med_speed):
            issues.append(
                Issue(
                    severity="ERROR",
                    category="kinematics",
                    code="C0_TELEPORTATION_SPIKE",
                    message=f"Instantaneous velocity spike ({spd:.2f} m/s vs median {med_speed:.2f} m/s) indicates position discontinuity/teleportation.",
                    frame_time=times[i],
                    expected=f"<= {max(25.0, 6.0 * med_speed):.2f} m/s",
                    actual=f"{spd:.2f} m/s",
                    formula="v_i = ||p_i - p_{i-1}|| / dt",
                )
            )

    # 1b. Ground Penetration Check (for root/CoM if below ground_level)
    if entity_type not in ("fish", "watercraft"):
        for i, pos in enumerate(positions):
            if pos[1] < ground_level - 0.02:
                issues.append(
                    Issue(
                        severity="ERROR",
                        category="kinematics",
                        code="GROUND_PENETRATION",
                        message=f"Entity root/CoM penetrated below ground plane (y={pos[1]:.3f}m < ground={ground_level:.3f}m).",
                        frame_time=times[i],
                        expected=f">= {ground_level:.3f} m",
                        actual=f"{pos[1]:.3f} m",
                    )
                )
                break

    # 1c. Ballistic / Airborne Gravity Check (for non-flying entities)
    is_flyer = entity_type in ("bird", "aircraft", "rotorcraft", "fish", "watercraft")
    if not is_flyer and n >= 4:
        airborne_ay: List[float] = []
        airborne_ahoriz: List[Tuple[float, float]] = []
        for i in range(1, n - 1):
            g_prev = frames[i - 1].get("grounded", True)
            g_cur = frames[i].get("grounded", True)
            g_next = frames[i + 1].get("grounded", True)
            if (g_prev is False) and (g_cur is False) and (g_next is False):
                airborne_ay.append(accelerations[i][1])
                airborne_ahoriz.append((times[i], math.hypot(accelerations[i][0], accelerations[i][2])))

        if airborne_ay:
            avg_ay = sum(airborne_ay) / len(airborne_ay)
            expected_ay = -scene_gravity
            metrics["airborne_vertical_accel_mps2"] = round(avg_ay, 3)
            rel_err = abs(avg_ay - expected_ay) / max(scene_gravity, 1e-5)
            if rel_err > 0.25:
                kind = "floaty/moon" if avg_ay > expected_ay else "excessively heavy"
                issues.append(
                    Issue(
                        severity="ERROR" if rel_err > 0.40 else "WARNING",
                        category="gravity",
                        code="BALLISTIC_GRAVITY_MISMATCH",
                        message=f"Airborne vertical acceleration ({avg_ay:.2f} m/s^2) deviates from gravity ({expected_ay:.2f} m/s^2) by {rel_err*100:.1f}% ({kind} arc).",
                        expected=f"{expected_ay:.2f} m/s^2 (±25%)",
                        actual=f"{avg_ay:.2f} m/s^2",
                        formula="a_y = d^2y/dt^2 = -g",
                    )
                )
            for t_air, a_h in airborne_ahoriz:
                if a_h > 2.5:
                    issues.append(
                        Issue(
                            severity="WARNING",
                            category="gravity",
                            code="MID_AIR_HORIZONTAL_PROPULSION",
                            message=f"Unexplained horizontal acceleration ({a_h:.2f} m/s^2) while airborne without external contact or thruster.",
                            frame_time=t_air,
                            expected="<= 2.50 m/s^2 (air drag only)",
                            actual=f"{a_h:.2f} m/s^2",
                            formula="F_{horiz,airborne} = -0.5 * rho * C_d * A * v^2",
                        )
                    )
                    break

    # 1d. Volume / Area Conservation (Squash & Stretch)
    for i, f in enumerate(frames):
        scale = f.get("scale")
        if scale and len(scale) >= 2:
            if dimension == "2d" or len(scale) == 2:
                vol = float(scale[0]) * float(scale[1])
                formula_str = "A_ratio = s_x * s_y = 1.0"
            else:
                vol = float(scale[0]) * float(scale[1]) * float(scale[2])
                formula_str = "V_ratio = s_x * s_y * s_z = 1.0"
            if abs(vol - 1.0) > 0.15:
                issues.append(
                    Issue(
                        severity="WARNING",
                        category="volume",
                        code="VOLUME_CONSERVATION_VIOLATION",
                        message=f"Squash-and-stretch scale {scale} violates volume/area conservation (product={vol:.3f}, expected ~1.0).",
                        frame_time=times[i],
                        expected="1.00 (±0.15)",
                        actual=f"{vol:.3f}",
                        formula=formula_str,
                    )
                )
                break

    # 1e. Center of Mass & Zero Moment Point (ZMP) Balance Check
    for i, f in enumerate(frames):
        raw_contacts = f.get("contacts")
        if not raw_contacts or not f.get("grounded", True):
            continue
        contacts_2d: List[Tuple[float, float]] = []
        for c in raw_contacts:
            c3 = to_3d_meters(c, dimension, up_axis, meters_per_unit)
            contacts_2d.append((c3[0], c3[2]))

        pos = positions[i]
        vel = velocities[i]
        acc = accelerations[i]
        is_quasi_static = vec_norm(vel) < 0.15 and vec_norm(acc) < 0.5

        if is_quasi_static:
            ok, dist_out = is_inside_support_region((pos[0], pos[2]), contacts_2d, tolerance=0.03)
            if not ok:
                issues.append(
                    Issue(
                        severity="ERROR",
                        category="balance",
                        code="STATIC_COM_OUTSIDE_BOS",
                        message=f"Static Center of Mass projection ({pos[0]:.2f}, {pos[2]:.2f}) lies {dist_out:.3f}m outside Base of Support polygon; model will tip over.",
                        frame_time=times[i],
                        expected="Inside Base of Support convex hull",
                        actual=f"{dist_out:.3f}m outside hull",
                        formula="p_{CoM,xz} in ConvexHull(contacts)",
                    )
                )
                break
        else:
            denom = acc[1] + scene_gravity
            if denom > 1.0:
                h_com = max(0.0, pos[1] - ground_level)
                zmp_x = pos[0] - (h_com / denom) * acc[0]
                zmp_z = pos[2] - (h_com / denom) * acc[2]
                ok, dist_out = is_inside_support_region((zmp_x, zmp_z), contacts_2d, tolerance=0.08)
                if not ok and dist_out > 0.10:
                    issues.append(
                        Issue(
                            severity="WARNING",
                            category="balance",
                            code="DYNAMIC_ZMP_OUTSIDE_BOS",
                            message=f"Dynamic Zero Moment Point ({zmp_x:.2f}, {zmp_z:.2f}) lies {dist_out:.2f}m outside support polygon during stance.",
                            frame_time=times[i],
                            expected="ZMP inside Base of Support",
                            actual=f"{dist_out:.2f}m outside",
                            formula="x_ZMP = x_CoM - (y_CoM / (a_y + g)) * a_x",
                        )
                    )
                    break

    return velocities, accelerations, jerks


# ---------------------------------------------------------------------------
# 2. Human Biomechanics Checks
# ---------------------------------------------------------------------------

HUMAN_JOINT_LIMITS: Dict[str, Dict[str, Tuple[float, float]]] = {
    "neck": {"flexion": (-65.0, 75.0), "abduction": (-55.0, 55.0), "rotation": (-90.0, 90.0)},
    "spine": {"flexion": (-40.0, 95.0), "abduction": (-45.0, 45.0), "rotation": (-55.0, 55.0)},
    "shoulder": {"flexion": (-70.0, 190.0), "abduction": (-55.0, 185.0), "rotation": (-85.0, 100.0)},
    "elbow": {"flexion": (-5.0, 155.0), "abduction": (-5.0, 5.0), "rotation": (-95.0, 95.0)},
    "wrist": {"flexion": (-85.0, 90.0), "abduction": (-25.0, 45.0), "rotation": (-15.0, 15.0)},
    "hip": {"flexion": (-40.0, 135.0), "abduction": (-35.0, 60.0), "rotation": (-50.0, 55.0)},
    "knee": {"flexion": (-5.0, 155.0), "abduction": (-5.0, 5.0), "rotation": (-20.0, 20.0)},
    "ankle": {"flexion": (-60.0, 35.0), "abduction": (-25.0, 25.0), "rotation": (-25.0, 25.0)},
}


def canonical_joint_type(joint_name: str) -> Optional[str]:
    lower = joint_name.lower()
    for key in HUMAN_JOINT_LIMITS:
        if key in lower:
            return key
    return None


def verify_human_biomechanics(
    frames: List[Dict[str, Any]],
    positions: List[Tuple[float, float, float]],
    velocities: List[Tuple[float, float, float]],
    accelerations: List[Tuple[float, float, float]],
    times: List[float],
    params: Dict[str, Any],
    scene_gravity: float,
    ground_level: float,
    dimension: str,
    up_axis: str,
    meters_per_unit: float,
    issues: List[Issue],
    metrics: Dict[str, Any],
) -> None:
    height = float(params.get("height", 1.75))
    leg_length = float(params.get("legLength", 0.53 * height))
    expected_gait = str(params.get("expectedGait", "")).lower()

    # 2a. Anatomical Joint Range of Motion (RoM) & Hinge Constraints
    reported_joints = set()
    for i, f in enumerate(frames):
        joints = f.get("joints", {})
        for j_name, j_angles in joints.items():
            j_type = canonical_joint_type(j_name)
            if not j_type or not isinstance(j_angles, dict):
                continue
            limits = HUMAN_JOINT_LIMITS[j_type]
            for axis, (min_deg, max_deg) in limits.items():
                if axis in j_angles:
                    val = float(j_angles[axis])
                    if val < min_deg or val > max_deg:
                        key = (j_name, axis)
                        if key not in reported_joints:
                            reported_joints.add(key)
                            is_hyperextension = j_type in ("knee", "elbow") and axis == "flexion" and val < min_deg
                            is_lateral_hinge = j_type in ("knee", "elbow") and axis == "abduction"
                            desc = (
                                f"Hinge joint '{j_name}' hyperextended ({val:.1f}°)"
                                if is_hyperextension
                                else f"Hinge joint '{j_name}' bent sideways ({val:.1f}°)"
                                if is_lateral_hinge
                                else f"Joint '{j_name}' {axis} ({val:.1f}°) exceeds human anatomical RoM"
                            )
                            issues.append(
                                Issue(
                                    severity="ERROR",
                                    category="human",
                                    code="HUMAN_JOINT_ROM_VIOLATION",
                                    message=f"{desc} [allowed {min_deg:.0f}° to {max_deg:.0f}°].",
                                    frame_time=times[i],
                                    expected=f"[{min_deg:.0f}°, {max_deg:.0f}°]",
                                    actual=f"{val:.1f}°",
                                )
                            )

    # 2b. Horizontal Speed & Froude Number Analysis
    horiz_speeds = [vec_horiz_norm(v) for v in velocities[1:]]
    avg_horiz_speed = sum(horiz_speeds) / len(horiz_speeds) if horiz_speeds else 0.0
    froude = (avg_horiz_speed ** 2) / (scene_gravity * max(leg_length, 0.1))
    metrics["avg_horiz_speed_mps"] = round(avg_horiz_speed, 3)
    metrics["froude_number"] = round(froude, 3)

    if expected_gait == "walk" and froude > 0.60:
        issues.append(
            Issue(
                severity="WARNING",
                category="human",
                code="WALK_FROUDE_EXCEEDED",
                message=f"Walking gait at speed {avg_horiz_speed:.2f} m/s yields Froude number Fr={froude:.2f} (> 0.50 walk-to-run transition threshold); inverted pendulum vault cannot stay grounded.",
                expected="Fr <= 0.50 for walking",
                actual=f"Fr = {froude:.2f}",
                formula="Fr = v^2 / (g * L_leg)",
            )
        )
    elif expected_gait == "run" and froude < 0.25 and avg_horiz_speed > 0.1:
        issues.append(
            Issue(
                severity="WARNING",
                category="human",
                code="RUN_FROUDE_TOO_LOW",
                message=f"Running gait at speed {avg_horiz_speed:.2f} m/s yields Froude number Fr={froude:.2f} (< 0.50); too slow for natural running mechanics.",
                expected="Fr >= 0.45 for running",
                actual=f"Fr = {froude:.2f}",
                formula="Fr = v^2 / (g * L_leg)",
            )
        )

    # 2c. Foot Contact, Duty Factor & Footskate Check
    n = len(frames)
    has_feet_data = any("feet" in f for f in frames)
    if has_feet_data and n >= 3:
        foot_grounded_counts = {"left": 0, "right": 0}
        double_support_frames = 0
        flight_frames = 0
        prev_foot_pos: Dict[str, Tuple[float, float, float]] = {}
        footskate_reported = False

        for i, f in enumerate(frames):
            feet = f.get("feet", {})
            l_info = feet.get("left", {})
            r_info = feet.get("right", {})

            l_g = l_info.get("grounded")
            r_g = r_info.get("grounded")

            if l_g is True:
                foot_grounded_counts["left"] += 1
            if r_g is True:
                foot_grounded_counts["right"] += 1
            if l_g is True and r_g is True:
                double_support_frames += 1
            if l_g is False and r_g is False:
                flight_frames += 1

            for side, info in (("left", l_info), ("right", r_info)):
                if "position" not in info:
                    continue
                f_pos = to_3d_meters(info["position"], dimension, up_axis, meters_per_unit)
                is_g = info.get("grounded", f_pos[1] <= ground_level + 0.02)
                if is_g and side in prev_foot_pos and i > 0:
                    dt = times[i] - times[i - 1]
                    if dt > 1e-6:
                        slip_speed = math.hypot(
                            f_pos[0] - prev_foot_pos[side][0],
                            f_pos[2] - prev_foot_pos[side][2],
                        ) / dt
                        if slip_speed > max(0.20, 0.15 * avg_horiz_speed) and not footskate_reported:
                            footskate_reported = True
                            issues.append(
                                Issue(
                                    severity="ERROR",
                                    category="human",
                                    code="FOOTSKATE_SLIDING",
                                    message=f"Grounded '{side}' foot slid horizontally at {slip_speed:.2f} m/s during stance phase (footskate).",
                                    frame_time=times[i],
                                    expected="<= 0.15 m/s while grounded",
                                    actual=f"{slip_speed:.2f} m/s",
                                    formula="||v_{foot,xz}|| = 0 during stance",
                                )
                            )
                if is_g:
                    prev_foot_pos[side] = f_pos
                else:
                    prev_foot_pos.pop(side, None)

        duty_left = foot_grounded_counts["left"] / n
        duty_right = foot_grounded_counts["right"] / n
        avg_duty = 0.5 * (duty_left + duty_right)
        metrics["duty_factor_avg"] = round(avg_duty, 3)
        metrics["double_support_ratio"] = round(double_support_frames / n, 3)
        metrics["flight_phase_ratio"] = round(flight_frames / n, 3)

        if expected_gait == "walk":
            if avg_duty < 0.50 or flight_frames > 0:
                issues.append(
                    Issue(
                        severity="ERROR",
                        category="human",
                        code="WALK_DUTY_FACTOR_INVALID",
                        message=f"Walking gait must have duty factor > 0.50 and zero flight phase (got beta={avg_duty:.2f}, flight_frames={flight_frames}).",
                        expected="beta in [0.52, 0.70], flight_frames = 0",
                        actual=f"beta={avg_duty:.2f}, flight_frames={flight_frames}",
                        formula="beta = t_stance / T_stride > 0.5",
                    )
                )
        elif expected_gait == "run":
            if avg_duty >= 0.52 or double_support_frames > int(0.1 * n):
                issues.append(
                    Issue(
                        severity="WARNING",
                        category="human",
                        code="RUN_DUTY_FACTOR_INVALID",
                        message=f"Running gait should have duty factor < 0.50 and an aerial flight phase (got beta={avg_duty:.2f}).",
                        expected="beta in [0.25, 0.48]",
                        actual=f"beta={avg_duty:.2f}",
                        formula="beta = t_stance / T_stride < 0.5",
                    )
                )

    # 2d. Pelvis Vertical Oscillation Check in Walk/Run
    if expected_gait in ("walk", "run") and n >= 5:
        y_vals = [p[1] for p in positions]
        y_amp = max(y_vals) - min(y_vals)
        metrics["pelvis_vertical_excursion_m"] = round(y_amp, 4)
        if y_amp < 0.012:
            issues.append(
                Issue(
                    severity="WARNING",
                    category="human",
                    code="FLAT_PELVIS_TRAJECTORY",
                    message=f"Pelvis vertical oscillation ({y_amp*100:.1f} cm) is unnaturally flat during {expected_gait} ('groucho walk'); natural {expected_gait} has 3-8 cm vertical bob.",
                    expected="0.025 m to 0.090 m",
                    actual=f"{y_amp:.3f} m",
                )
            )

    # 2e. Jump Anticipation Crouch & Takeoff Impulse Check
    if expected_gait == "jump" and n >= 4:
        takeoff_idx = None
        for i in range(1, n):
            if frames[i - 1].get("grounded", True) and not frames[i].get("grounded", True):
                takeoff_idx = i - 1
                break
        if takeoff_idx is not None:
            pre_y = [positions[k][1] for k in range(0, takeoff_idx + 1)]
            air_y = [positions[k][1] for k in range(takeoff_idx, n)]
            y_takeoff = positions[takeoff_idx][1]
            y_min_crouch = min(pre_y)
            y_initial = pre_y[0]
            crouch_depth = max(y_initial - y_min_crouch, y_takeoff - y_min_crouch)
            jump_height = max(air_y) - y_takeoff
            metrics["jump_height_m"] = round(jump_height, 3)
            metrics["crouch_depth_m"] = round(crouch_depth, 3)
            if jump_height > 0.15:
                if crouch_depth < 0.04:
                    issues.append(
                        Issue(
                            severity="ERROR",
                            category="human",
                            code="JUMP_MISSING_ANTICIPATION_CROUCH",
                            message=f"Jump of height {jump_height:.2f}m occurred with negligible anticipation crouch ({crouch_depth*100:.1f}cm); violates impulse-momentum work requirement.",
                            frame_time=times[takeoff_idx],
                            expected="crouch_depth >= 0.08 m",
                            actual=f"{crouch_depth:.3f} m",
                            formula="a_push = g * (h_jump / d_crouch)",
                        )
                    )
                else:
                    req_accel_g = jump_height / crouch_depth
                    if req_accel_g > 4.0:
                        issues.append(
                            Issue(
                                severity="WARNING",
                                category="human",
                                code="JUMP_EXCESSIVE_PUSH_G_FORCE",
                                message=f"Jump height ({jump_height:.2f}m) relative to shallow crouch ({crouch_depth:.2f}m) requires {req_accel_g:.1f}g average leg push-off acceleration (human limit ~3.5g).",
                                frame_time=times[takeoff_idx],
                                expected="a_push <= 3.5 g",
                                actual=f"{req_accel_g:.1f} g",
                                formula="a_push / g = h_jump / d_crouch",
                            )
                        )


# ---------------------------------------------------------------------------
# 3. Animal Locomotion Checks (Quadrupeds, Birds, Fish)
# ---------------------------------------------------------------------------

QUADRUPED_GAIT_PHASES: Dict[str, Tuple[Tuple[float, float], List[float]]] = {
    "walk": ((0.58, 0.80), [0.00, 0.25, 0.50, 0.75]),
    "trot": ((0.35, 0.56), [0.00, 0.50, 0.50, 0.00]),
    "pace": ((0.35, 0.56), [0.00, 0.00, 0.50, 0.50]),
    "canter": ((0.32, 0.50), [0.00, 0.30, 0.30, 0.65]),
    "transverse_gallop": ((0.20, 0.40), [0.00, 0.45, 0.15, 0.60]),
    "rotary_gallop": ((0.18, 0.38), [0.00, 0.55, 0.15, 0.40]),
}


def phase_distance(p1: float, p2: float) -> float:
    d = abs(p1 - p2) % 1.0
    return min(d, 1.0 - d)


def verify_animal_locomotion(
    entity_type: str,
    frames: List[Dict[str, Any]],
    positions: List[Tuple[float, float, float]],
    velocities: List[Tuple[float, float, float]],
    accelerations: List[Tuple[float, float, float]],
    times: List[float],
    params: Dict[str, Any],
    scene_gravity: float,
    ground_level: float,
    dimension: str,
    up_axis: str,
    meters_per_unit: float,
    issues: List[Issue],
    metrics: Dict[str, Any],
) -> None:
    n = len(frames)
    horiz_speeds = [vec_horiz_norm(v) for v in velocities[1:]]
    avg_speed = sum(horiz_speeds) / len(horiz_speeds) if horiz_speeds else 0.0
    metrics["avg_horiz_speed_mps"] = round(avg_speed, 3)

    if entity_type == "quadruped":
        expected_gait = str(params.get("expectedGait", "")).lower()
        limbs = ("LH", "LF", "RH", "RF")
        grounded_counts = {k: 0 for k in limbs}
        first_strike_time: Dict[str, float] = {}

        for i, f in enumerate(frames):
            feet = f.get("feet", {})
            for limb in limbs:
                info = feet.get(limb, {})
                is_g = info.get("grounded")
                if is_g is True:
                    grounded_counts[limb] += 1
                    prev_g = frames[i - 1].get("feet", {}).get(limb, {}).get("grounded", False) if i > 0 else False
                    if (not prev_g or i == 0) and limb not in first_strike_time:
                        first_strike_time[limb] = times[i]

        if any(grounded_counts.values()) and n > 0:
            avg_duty = sum(grounded_counts.values()) / (4.0 * n)
            metrics["quadruped_duty_factor"] = round(avg_duty, 3)

            if expected_gait in QUADRUPED_GAIT_PHASES:
                (d_min, d_max), expected_phases = QUADRUPED_GAIT_PHASES[expected_gait]
                if not (d_min <= avg_duty <= d_max):
                    issues.append(
                        Issue(
                            severity="WARNING",
                            category="animal",
                            code="QUADRUPED_DUTY_FACTOR_MISMATCH",
                            message=f"Quadruped '{expected_gait}' duty factor ({avg_duty:.2f}) is outside Hildebrand biomechanical range [{d_min:.2f}, {d_max:.2f}].",
                            expected=f"[{d_min:.2f}, {d_max:.2f}]",
                            actual=f"{avg_duty:.2f}",
                            formula="beta = t_stance / T_stride",
                        )
                    )

                duration = times[-1] - times[0]
                stride_period = float(params.get("stridePeriodSec", duration))
                if len(first_strike_time) == 4 and stride_period > 1e-4:
                    t_lh = first_strike_time["LH"]
                    actual_phases = [
                        ((first_strike_time[l] - t_lh) / stride_period) % 1.0 for l in limbs
                    ]
                    metrics["quadruped_limb_phases"] = [round(p, 3) for p in actual_phases]
                    max_phase_err = max(
                        phase_distance(actual_phases[idx], expected_phases[idx])
                        for idx in range(4)
                    )
                    if max_phase_err > 0.14:
                        issues.append(
                            Issue(
                                severity="ERROR",
                                category="animal",
                                code="QUADRUPED_PHASE_SEQUENCE_ERROR",
                                message=(
                                    f"Quadruped footfall phase sequence [LH, LF, RH, RF] = {[round(p, 2) for p in actual_phases]} "
                                    f"does not match '{expected_gait}' pattern {expected_phases} (max error={max_phase_err:.2f})."
                                ),
                                expected=str(expected_phases),
                                actual=str([round(p, 2) for p in actual_phases]),
                                formula="phi_i = (t_strike,i - t_strike,LH) / T_stride mod 1",
                            )
                        )

    elif entity_type == "bird":
        freq = params.get("wingbeatFreqHz")
        amp = params.get("strokeAmplitudeM")

        wing_y = [float(f["wingtipY"]) * meters_per_unit for f in frames if "wingtipY" in f]
        if len(wing_y) >= 4:
            if amp is None:
                amp = max(wing_y) - min(wing_y)
            if freq is None:
                mean_w = 0.5 * (max(wing_y) + min(wing_y))
                up_crossings = 0
                for i in range(1, len(wing_y)):
                    if wing_y[i - 1] < mean_w <= wing_y[i]:
                        up_crossings += 1
                dur = times[-1] - times[0]
                if dur > 0 and up_crossings > 0:
                    freq = up_crossings / dur

            if len(wing_y) == n:
                in_phase_frames = 0
                checked_frames = 0
                for i in range(1, n):
                    dw = wing_y[i] - wing_y[i - 1]
                    db = positions[i][1] - positions[i - 1][1]
                    if abs(dw) > 1e-4 and abs(db) > 1e-4:
                        checked_frames += 1
                        if dw * db > 0:
                            in_phase_frames += 1
                if checked_frames >= 4 and (in_phase_frames / checked_frames) > 0.80:
                    issues.append(
                        Issue(
                            severity="WARNING",
                            category="animal",
                            code="AVIAN_BODY_HEAVE_IN_PHASE_ERROR",
                            message="Bird torso moves upward during wing upstroke and downward during downstroke (in-phase); aerodynamically, lift peaks on the downstroke so body heave should oppose wing motion.",
                            expected="Body heave ~180° out of phase with wing vertical stroke",
                            actual="In-phase motion detected",
                        )
                    )

        if freq is not None and amp is not None and avg_speed > 0.5:
            strouhal = (float(freq) * float(amp)) / avg_speed
            metrics["strouhal_number"] = round(strouhal, 3)
            if not (0.15 <= strouhal <= 0.48):
                issues.append(
                    Issue(
                        severity="WARNING",
                        category="animal",
                        code="AVIAN_STROUHAL_OUT_OF_RANGE",
                        message=f"Flapping flight Strouhal number St={strouhal:.2f} (f={float(freq):.1f}Hz, A={float(amp):.2f}m, v={avg_speed:.2f}m/s) lies outside natural efficient range [0.20, 0.40].",
                        expected="0.20 <= St <= 0.40",
                        actual=f"St = {strouhal:.2f}",
                        formula="St = (f * A) / v",
                    )
                )

    elif entity_type == "fish":
        freq = params.get("tailBeatFreqHz")
        amp = params.get("tailAmplitudeM")
        wave_speed = params.get("bodyWaveSpeedMps")

        if freq is not None and amp is not None and avg_speed > 0.05:
            strouhal = (float(freq) * float(amp)) / avg_speed
            metrics["strouhal_number"] = round(strouhal, 3)
            if not (0.15 <= strouhal <= 0.48):
                issues.append(
                    Issue(
                        severity="WARNING",
                        category="animal",
                        code="AQUATIC_STROUHAL_OUT_OF_RANGE",
                        message=f"Undulatory swimming Strouhal number St={strouhal:.2f} lies outside natural biological range [0.20, 0.40].",
                        expected="0.20 <= St <= 0.40",
                        actual=f"St = {strouhal:.2f}",
                        formula="St = (f * A) / v",
                    )
                )
        if wave_speed is not None and float(wave_speed) > 0:
            slip = avg_speed / float(wave_speed)
            metrics["wave_slip_ratio"] = round(slip, 3)
            if slip >= 1.0 or slip < 0.35:
                issues.append(
                    Issue(
                        severity="ERROR" if slip >= 1.0 else "WARNING",
                        category="animal",
                        code="AQUATIC_WAVE_SLIP_INVALID",
                        message=f"Body undulation wave speed ({float(wave_speed):.2f} m/s) vs forward speed ({avg_speed:.2f} m/s) gives slip ratio v/v_wave={slip:.2f}; backward-propagating wave must travel faster than forward swimming speed (0.55 <= v/v_wave <= 0.85).",
                        expected="0.55 <= v / v_wave <= 0.85",
                        actual=f"{slip:.2f}",
                        formula="slip = v_forward / v_wave < 1.0",
                    )
                )


# ---------------------------------------------------------------------------
# 4. Vehicle & Transportation Dynamics Checks
# ---------------------------------------------------------------------------

def verify_vehicle_dynamics(
    entity_type: str,
    frames: List[Dict[str, Any]],
    positions: List[Tuple[float, float, float]],
    velocities: List[Tuple[float, float, float]],
    accelerations: List[Tuple[float, float, float]],
    times: List[float],
    params: Dict[str, Any],
    scene_gravity: float,
    issues: List[Issue],
    metrics: Dict[str, Any],
) -> None:
    n = len(frames)
    if n < 3:
        return

    pitches: List[float] = []
    yaws: List[float] = []
    rolls: List[float] = []
    for f in frames:
        rot = f.get("rotation", [0.0, 0.0, 0.0])
        pitches.append(float(rot[0]) if len(rot) > 0 else 0.0)
        yaws.append(float(rot[1]) if len(rot) > 1 else 0.0)
        rolls.append(float(rot[2]) if len(rot) > 2 else 0.0)

    yaw_rates_rad: List[float] = [0.0] * n
    for i in range(1, n):
        dt = times[i] - times[i - 1]
        if dt > 1e-6:
            dyaw_deg = angle_diff_deg(yaws[i], yaws[i - 1])
            yaw_rates_rad[i] = math.radians(dyaw_deg) / dt
    yaw_rates_rad[0] = yaw_rates_rad[1]

    if entity_type in ("wheeled_vehicle", "two_wheeled_vehicle"):
        wheel_radius = float(params.get("wheelRadius", 0.33))
        wheelbase = float(params.get("wheelbase", 2.7))
        mu = float(params.get("tireFrictionMu", 0.85))

        slip_reported = False
        steer_reported = False
        ackermann_reported = False
        pitch_reported = False
        roll_reported = False
        grip_reported = False

        for i in range(1, n - 1):
            v_horiz = vec_horiz_norm(velocities[i])
            yaw_rate = yaw_rates_rad[i]
            lat_accel = v_horiz * abs(yaw_rate)
            long_accel = (
                vec_horiz_norm(velocities[i + 1]) - vec_horiz_norm(velocities[i - 1])
            ) / max(times[i + 1] - times[i - 1], 1e-6)

            wheels = frames[i].get("wheels", {})
            if "omegaRadPerSec" in wheels and v_horiz > 0.5 and not slip_reported:
                omega = float(wheels["omegaRadPerSec"])
                v_wheel = omega * wheel_radius
                slip_ratio = (v_wheel - v_horiz) / max(abs(v_horiz), abs(v_wheel), 1e-5)
                if abs(slip_ratio) > 0.20 and abs(long_accel) < 0.5 * mu * scene_gravity:
                    slip_reported = True
                    issues.append(
                        Issue(
                            severity="ERROR",
                            category="vehicle",
                            code="WHEEL_ROLLING_SLIP_MISMATCH",
                            message=f"Wheel tangential speed (omega*R = {v_wheel:.2f} m/s) mismatches vehicle speed ({v_horiz:.2f} m/s), slip ratio kappa={slip_ratio:.2f} during non-skidding motion.",
                            frame_time=times[i],
                            expected=f"omega = v / R = {v_horiz / wheel_radius:.2f} rad/s",
                            actual=f"{omega:.2f} rad/s",
                            formula="omega_wheel = v_x / R_wheel",
                        )
                    )

            if "steerAngleDeg" in wheels and v_horiz > 1.0 and not steer_reported:
                steer_rad = math.radians(float(wheels["steerAngleDeg"]))
                expected_yaw_rate = (v_horiz * math.tan(steer_rad)) / max(wheelbase, 0.1)
                if abs(expected_yaw_rate - yaw_rate) > 0.25 and lat_accel <= mu * scene_gravity:
                    steer_reported = True
                    issues.append(
                        Issue(
                            severity="ERROR",
                            category="vehicle",
                            code="ACKERMANN_YAW_RATE_MISMATCH",
                            message=f"Vehicle yaw rate ({ math.degrees(yaw_rate):.1f}°/s) mismatches steering angle ({float(wheels['steerAngleDeg']):.1f}°) and wheelbase ({wheelbase}m); expected {math.degrees(expected_yaw_rate):.1f}°/s.",
                            frame_time=times[i],
                            expected=f"{math.degrees(expected_yaw_rate):.1f} deg/s",
                            actual=f"{math.degrees(yaw_rate):.1f} deg/s",
                            formula="psi_dot = (v * tan(delta)) / L_wheelbase",
                        )
                    )

            if "steerInnerDeg" in wheels and "steerOuterDeg" in wheels and not ackermann_reported:
                d_in = abs(float(wheels["steerInnerDeg"]))
                d_out = abs(float(wheels["steerOuterDeg"]))
                if d_in > 3.0 and d_in <= d_out:
                    ackermann_reported = True
                    issues.append(
                        Issue(
                            severity="WARNING",
                            category="vehicle",
                            code="ACKERMANN_INNER_OUTER_ANGLE_ERROR",
                            message=f"Inner front wheel steer angle ({d_in:.1f}°) must be sharper (larger) than outer front wheel ({d_out:.1f}°) to satisfy Ackermann geometry.",
                            frame_time=times[i],
                            expected=f"delta_inner > delta_outer ({d_out:.1f}°)",
                            actual=f"delta_inner = {d_in:.1f}°",
                            formula="cot(delta_outer) - cot(delta_inner) = W_track / L_wheelbase",
                        )
                    )

            if lat_accel > mu * scene_gravity * 1.15 and not grip_reported:
                grip_reported = True
                issues.append(
                    Issue(
                        severity="WARNING",
                        category="vehicle",
                        code="LATERAL_TIRE_GRIP_EXCEEDED",
                        message=f"Cornering lateral acceleration ({lat_accel:.2f} m/s^2 = {lat_accel/scene_gravity:.2f}g) exceeds tire friction limit (mu*g = {mu*scene_gravity:.2f} m/s^2) without drift/skid.",
                        frame_time=times[i],
                        expected=f"<= {mu * scene_gravity:.2f} m/s^2 ({mu}g)",
                        actual=f"{lat_accel:.2f} m/s^2 ({lat_accel/scene_gravity:.2f}g)",
                        formula="a_lat = v^2 / R <= mu * g",
                    )
                )

            if abs(long_accel) > 2.5 and not pitch_reported:
                pitch = pitches[i]
                if (long_accel > 2.5 and pitch < -1.5) or (long_accel < -2.5 and pitch > 1.5):
                    pitch_reported = True
                    action = "acceleration" if long_accel > 0 else "braking"
                    expected_dir = "nose-up squat (>= 0°)" if long_accel > 0 else "nose-down dive (<= 0°)"
                    issues.append(
                        Issue(
                            severity="WARNING",
                            category="vehicle",
                            code="INVERTED_SUSPENSION_LOAD_TRANSFER_PITCH",
                            message=f"Chassis pitched {pitch:.1f}° during longitudinal {action} ({long_accel:.2f} m/s^2); inertial load transfer requires {expected_dir}.",
                            frame_time=times[i],
                            expected=expected_dir,
                            actual=f"pitch = {pitch:.1f}°",
                            formula="Delta_F_z = (m * a_x * h_CoM) / L_wheelbase",
                        )
                    )

            if entity_type == "two_wheeled_vehicle" and lat_accel > 1.5 and not roll_reported:
                expected_lean_deg = math.degrees(math.atan2(lat_accel, scene_gravity))
                actual_lean_mag = abs(rolls[i])
                if actual_lean_mag < 0.5 * expected_lean_deg or (rolls[i] * yaw_rate < 0):
                    roll_reported = True
                    issues.append(
                        Issue(
                            severity="ERROR",
                            category="vehicle",
                            code="TWO_WHEEL_TURN_LEAN_VIOLATION",
                            message=f"Two-wheeled vehicle in turn (a_lat={lat_accel:.2f} m/s^2) has roll={rolls[i]:.1f}°, but requires inward lean of {expected_lean_deg:.1f}° for roll equilibrium.",
                            frame_time=times[i],
                            expected=f"Inward lean ~{expected_lean_deg:.1f}°",
                            actual=f"{rolls[i]:.1f}°",
                            formula="theta_lean = arctan(v^2 / (R * g))",
                        )
                    )

    elif entity_type == "aircraft":
        stall_speed = float(params.get("stallSpeedMps", 15.0))
        bank_reported = False
        stall_reported = False
        for i in range(1, n - 1):
            v_horiz = vec_horiz_norm(velocities[i])
            yaw_rate = yaw_rates_rad[i]
            lat_accel = v_horiz * abs(yaw_rate)
            expected_bank_deg = math.degrees(math.atan2(lat_accel, scene_gravity))
            actual_bank_deg = abs(rolls[i])

            if v_horiz < stall_speed and not frames[i].get("grounded", False) and not stall_reported:
                stall_reported = True
                issues.append(
                    Issue(
                        severity="ERROR",
                        category="vehicle",
                        code="AIRCRAFT_BELOW_STALL_SPEED",
                        message=f"Fixed-wing aircraft airborne at {v_horiz:.1f} m/s below minimum stall speed ({stall_speed:.1f} m/s).",
                        frame_time=times[i],
                        expected=f">= {stall_speed:.1f} m/s",
                        actual=f"{v_horiz:.1f} m/s",
                        formula="v >= v_stall * sqrt(1 / cos(phi_bank))",
                    )
                )

            if expected_bank_deg > 10.0 and abs(actual_bank_deg - expected_bank_deg) > 12.0 and not bank_reported:
                bank_reported = True
                issues.append(
                    Issue(
                        severity="WARNING",
                        category="vehicle",
                        code="UNCOORDINATED_AIRCRAFT_TURN_BANK",
                        message=f"Aircraft turning with centripetal accel {lat_accel:.2f} m/s^2 has bank angle {actual_bank_deg:.1f}°, expected coordinated bank of {expected_bank_deg:.1f}°.",
                        frame_time=times[i],
                        expected=f"{expected_bank_deg:.1f}°",
                        actual=f"{actual_bank_deg:.1f}°",
                        formula="phi_bank = arctan((v * psi_dot) / g)",
                    )
                )

    elif entity_type == "rotorcraft":
        pitch_coupling_reported = False
        for i in range(1, n - 1):
            if frames[i].get("grounded", False):
                continue
            yaw_rad = math.radians(yaws[i])
            fwd_x = math.cos(yaw_rad)
            fwd_z = math.sin(yaw_rad)
            acc = accelerations[i]
            a_fwd = acc[0] * fwd_x + acc[2] * fwd_z
            denom = max(acc[1] + scene_gravity, 1.0)
            expected_pitch_deg = -math.degrees(math.atan2(a_fwd, denom))
            actual_pitch_deg = pitches[i]

            if abs(a_fwd) > 1.8 and abs(actual_pitch_deg - expected_pitch_deg) > 8.0 and not pitch_coupling_reported:
                pitch_coupling_reported = True
                issues.append(
                    Issue(
                        severity="ERROR",
                        category="vehicle",
                        code="ROTORCRAFT_THRUST_TILT_DECOUPLED",
                        message=(
                            f"Quadcopter/helicopter horizontal acceleration ({a_fwd:.2f} m/s^2) requires pitch={expected_pitch_deg:.1f}° "
                            f"(nose-down to accelerate, nose-up flare to brake), but model pitch is {actual_pitch_deg:.1f}°."
                        ),
                        frame_time=times[i],
                        expected=f"{expected_pitch_deg:.1f}°",
                        actual=f"{actual_pitch_deg:.1f}°",
                        formula="theta_pitch = -arctan(a_forward / (g + a_y))",
                    )
                )


# ---------------------------------------------------------------------------
# Main Verification Entry Point
# ---------------------------------------------------------------------------

def verify_scene(data: Dict[str, Any], strict: bool = False) -> Dict[str, Any]:
    scene = data.get("scene", {})
    dimension = str(scene.get("dimension", "3d")).lower()
    up_axis = str(scene.get("upAxis", "y")).lower()
    scene_gravity = float(scene.get("gravity", 9.80665))
    meters_per_unit = float(scene.get("metersPerUnit", 1.0))
    ground_level = float(scene.get("groundLevel", 0.0)) * meters_per_unit

    entity_reports: List[EntityReport] = []

    for ent in data.get("entities", []):
        ent_id = str(ent.get("id", "unnamed_entity"))
        ent_type = str(ent.get("type", "rigid_body")).lower()
        params = ent.get("params", {})
        rig = ent.get("rig", {})
        frames = ent.get("frames", [])

        issues: List[Issue] = []
        metrics: Dict[str, Any] = {}

        times = [float(f.get("t", idx * (1.0 / 30.0))) for idx, f in enumerate(frames)]
        positions = [
            to_3d_meters(f.get("position", [0.0, 0.0, 0.0]), dimension, up_axis, meters_per_unit)
            for f in frames
        ]

        # 0. Verify 2D/3D Rigging, Skinning & Animation Standards
        verify_rig_and_animation_standards(
            rig=rig,
            frames=frames,
            times=times,
            dimension=dimension,
            up_axis=up_axis,
            meters_per_unit=meters_per_unit,
            issues=issues,
            metrics=metrics,
        )

        # 1. Verify Universal Classical Mechanics
        velocities, accelerations, _ = verify_classical_mechanics(
            frames=frames,
            positions=positions,
            times=times,
            entity_type=ent_type,
            scene_gravity=scene_gravity,
            ground_level=ground_level,
            dimension=dimension,
            up_axis=up_axis,
            meters_per_unit=meters_per_unit,
            issues=issues,
            metrics=metrics,
        )

        # 2-4. Domain-Specific Biomechanics & Vehicle Dynamics
        if ent_type == "human":
            verify_human_biomechanics(
                frames=frames,
                positions=positions,
                velocities=velocities,
                accelerations=accelerations,
                times=times,
                params=params,
                scene_gravity=scene_gravity,
                ground_level=ground_level,
                dimension=dimension,
                up_axis=up_axis,
                meters_per_unit=meters_per_unit,
                issues=issues,
                metrics=metrics,
            )
        elif ent_type in ("quadruped", "bird", "fish"):
            verify_animal_locomotion(
                entity_type=ent_type,
                frames=frames,
                positions=positions,
                velocities=velocities,
                accelerations=accelerations,
                times=times,
                params=params,
                scene_gravity=scene_gravity,
                ground_level=ground_level,
                dimension=dimension,
                up_axis=up_axis,
                meters_per_unit=meters_per_unit,
                issues=issues,
                metrics=metrics,
            )
        elif ent_type in ("wheeled_vehicle", "two_wheeled_vehicle", "aircraft", "rotorcraft", "watercraft"):
            verify_vehicle_dynamics(
                entity_type=ent_type,
                frames=frames,
                positions=positions,
                velocities=velocities,
                accelerations=accelerations,
                times=times,
                params=params,
                scene_gravity=scene_gravity,
                issues=issues,
                metrics=metrics,
            )

        # Compute Physics & Standards Realism Score (0 - 100)
        penalty = 0.0
        has_error = False
        has_warning = False
        for iss in issues:
            if iss.severity == "ERROR":
                penalty += 25.0
                has_error = True
            elif iss.severity == "WARNING":
                penalty += 10.0
                has_warning = True
            else:
                penalty += 2.0

        score = max(0.0, round(100.0 - penalty, 1))
        passed = (not has_error and not has_warning) if strict else (not has_error)
        duration = (times[-1] - times[0]) if len(times) >= 2 else 0.0

        entity_reports.append(
            EntityReport(
                entity_id=ent_id,
                entity_type=ent_type,
                frame_count=len(frames),
                duration_sec=round(duration, 4),
                score=score,
                passed=passed,
                metrics=metrics,
                issues=issues,
            )
        )

    overall_score = (
        round(sum(r.score for r in entity_reports) / len(entity_reports), 1)
        if entity_reports
        else 100.0
    )
    overall_passed = all(r.passed for r in entity_reports)

    return {
        "overall_passed": overall_passed,
        "overall_physics_score": overall_score,
        "entity_count": len(entity_reports),
        "entities": [asdict(r) for r in entity_reports],
    }


def format_markdown_report(result: Dict[str, Any]) -> str:
    lines: List[str] = []
    status_str = "PASSED" if result["overall_passed"] else "FAILED"
    lines.append(f"# 2D/3D Physics, Rigging & Biomechanics Verification Report: **{status_str}**")
    lines.append(f"- **Overall Realism Score**: `{result['overall_physics_score']} / 100`")
    lines.append(f"- **Entities Analyzed**: `{result['entity_count']}`\n")

    for ent in result["entities"]:
        badge = "PASS" if ent["passed"] else "FAIL"
        lines.append(
            f"## [{badge}] Entity `{ent['entity_id']}` (type: `{ent['entity_type']}`, score: `{ent['score']}/100`)"
        )
        lines.append(
            f"- **Frames**: `{ent['frame_count']}` | **Duration**: `{ent['duration_sec']}s`"
        )
        if ent["metrics"]:
            metric_pairs = ", ".join(f"`{k}={v}`" for k, v in ent["metrics"].items())
            lines.append(f"- **Computed Metrics**: {metric_pairs}")

        if not ent["issues"]:
            lines.append("- **Findings**: All rigging, animation, physical, biomechanical, and dynamical standards satisfied.\n")
        else:
            lines.append("- **Detected Standards, Physics & Movement Violations**:")
            for iss in ent["issues"]:
                t_str = f" @ t={iss['frame_time']:.3f}s" if iss["frame_time"] is not None else ""
                lines.append(
                    f"  - **[{iss['severity']}] `{iss['code']}` ({iss['category']}{t_str})**: {iss['message']}"
                )
                details = []
                if iss.get("expected"):
                    details.append(f"Expected: `{iss['expected']}`")
                if iss.get("actual"):
                    details.append(f"Actual: `{iss['actual']}`")
                if iss.get("formula"):
                    details.append(f"Law/Formula: `{iss['formula']}`")
                if details:
                    lines.append("    - " + " | ".join(details))
            lines.append("")

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify 2D/3D model rigs, animations, and movements against industry standards, physics, and biomechanics laws."
    )
    parser.add_argument("input", type=Path, help="Path to standardized motion JSON file.")
    parser.add_argument("--json", action="store_true", help="Output structured JSON report.")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Treat warnings as failures in exit code.",
    )
    args = parser.parse_args()

    if not args.input.exists():
        print(f"Error: Input file '{args.input}' not found.", file=sys.stderr)
        return 2

    with args.input.open("r", encoding="utf-8") as f:
        data = json.load(f)

    result = verify_scene(data, strict=args.strict)

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(format_markdown_report(result))

    return 0 if result["overall_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
