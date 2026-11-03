#!/usr/bin/env python3
"""
Extract 2D/3D Rig, Skinning & Animation Tracks from .gltf, .glb, .bvh, or .urdf
into Standardized Motion & Rig Verification JSON.

Uses only the Python standard library (struct, json, base64, math, xml.etree.ElementTree).

Usage:
    python3 scripts/extract_gltf_motion.py <input.gltf|input.glb|input.bvh|input.urdf> \
        [--entity-type human] [--expected-gait walk] [-o output_motion.json]
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Quaternion & Matrix Helpers
# ---------------------------------------------------------------------------

def quat_to_euler_deg(qx: float, qy: float, qz: float, qw: float) -> List[float]:
    """Convert quaternion [x, y, z, w] to [pitch, yaw, roll] in degrees."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    pitch = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        yaw = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        yaw = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    roll = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return [round(pitch, 3), round(yaw, 3), round(roll, 3)]


def mat4_det_3x3(m: List[float]) -> float:
    """Compute upper-left 3x3 determinant of a column-major 4x4 glTF matrix."""
    if len(m) < 16:
        return 1.0
    a00, a10, a20 = m[0], m[1], m[2]
    a01, a11, a21 = m[4], m[5], m[6]
    a02, a12, a22 = m[8], m[9], m[10]
    return (
        a00 * (a11 * a22 - a12 * a21)
        - a01 * (a10 * a22 - a12 * a20)
        + a02 * (a10 * a21 - a11 * a20)
    )


# ---------------------------------------------------------------------------
# glTF 2.0 / GLB Parser (Rig, Skinning, Bind Matrices & Animation Tracks)
# ---------------------------------------------------------------------------

COMPONENT_TYPES: Dict[int, Tuple[str, int]] = {
    5120: ("b", 1),  # BYTE
    5121: ("B", 1),  # UNSIGNED_BYTE
    5122: ("h", 2),  # SHORT
    5123: ("H", 2),  # UNSIGNED_SHORT
    5125: ("I", 4),  # UNSIGNED_INT
    5126: ("f", 4),  # FLOAT
}

TYPE_COUNTS: Dict[str, int] = {
    "SCALAR": 1,
    "VEC2": 2,
    "VEC3": 3,
    "VEC4": 4,
    "MAT4": 16,
}


def load_gltf_or_glb(path: Path) -> Tuple[Dict[str, Any], List[bytes]]:
    raw = path.read_bytes()
    if path.suffix.lower() == ".glb" or raw[:4] == b"glTF":
        magic, version, total_len = struct.unpack_from("<4sII", raw, 0)
        if magic != b"glTF":
            raise ValueError(f"Invalid GLB magic header in {path}")
        offset = 12
        json_doc: Dict[str, Any] = {}
        bin_chunk: bytes = b""
        while offset < len(raw):
            chunk_len, chunk_type = struct.unpack_from("<II", raw, offset)
            offset += 8
            chunk_data = raw[offset : offset + chunk_len]
            offset += chunk_len
            if chunk_type == 0x4E4F534A:  # JSON
                json_doc = json.loads(chunk_data.decode("utf-8"))
            elif chunk_type == 0x004E4942:  # BIN
                bin_chunk = chunk_data
        buffers: List[bytes] = []
        for idx, buf_spec in enumerate(json_doc.get("buffers", [])):
            uri = buf_spec.get("uri")
            if uri is None and idx == 0:
                buffers.append(bin_chunk)
            elif uri and uri.startswith("data:"):
                b64_part = uri.split(",", 1)[1]
                buffers.append(base64.b64decode(b64_part))
            elif uri:
                buffers.append((path.parent / uri).read_bytes())
        return json_doc, buffers
    else:
        json_doc = json.loads(raw.decode("utf-8"))
        buffers = []
        for buf_spec in json_doc.get("buffers", []):
            uri = buf_spec.get("uri", "")
            if uri.startswith("data:"):
                b64_part = uri.split(",", 1)[1]
                buffers.append(base64.b64decode(b64_part))
            elif uri:
                buffers.append((path.parent / uri).read_bytes())
        return json_doc, buffers


def read_accessor(
    gltf: Dict[str, Any], buffers: List[bytes], accessor_idx: int
) -> List[List[float]]:
    acc = gltf["accessors"][accessor_idx]
    bv = gltf["bufferViews"][acc["bufferView"]]
    buf = buffers[bv.get("buffer", 0)]

    comp_fmt, comp_size = COMPONENT_TYPES[acc["componentType"]]
    num_comp = TYPE_COUNTS[acc["type"]]
    count = acc["count"]

    byte_offset = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = bv.get("byteStride", comp_size * num_comp)
    unpack_fmt = "<" + (comp_fmt * num_comp)

    out: List[List[float]] = []
    for i in range(count):
        pos = byte_offset + i * stride
        vals = struct.unpack_from(unpack_fmt, buf, pos)
        out.append([float(v) for v in vals])
    return out


def extract_gltf_rig_and_skinning(
    gltf: Dict[str, Any], buffers: List[bytes]
) -> Dict[str, Any]:
    nodes = gltf.get("nodes", [])
    skins = gltf.get("skins", [])
    meshes = gltf.get("meshes", [])

    # Parent lookup
    parent_of: Dict[int, int] = {}
    for p_idx, nd in enumerate(nodes):
        for c_idx in nd.get("children", []):
            parent_of[c_idx] = p_idx

    rig_bones: List[Dict[str, Any]] = []
    joint_indices = set()
    non_inv_ibm = 0
    neg_det_ibm = 0

    for sk in skins:
        for j_idx in sk.get("joints", []):
            joint_indices.add(j_idx)
        ibm_acc = sk.get("inverseBindMatrices")
        if ibm_acc is not None and buffers:
            mats = read_accessor(gltf, buffers, ibm_acc)
            for m in mats:
                det = mat4_det_3x3(m)
                if abs(det) <= 1e-6:
                    non_inv_ibm += 1
                elif det < 0:
                    neg_det_ibm += 1

    target_nodes = sorted(joint_indices) if joint_indices else list(range(len(nodes)))
    for idx in target_nodes:
        if idx >= len(nodes):
            continue
        nd = nodes[idx]
        name = nd.get("name", f"node_{idx}")
        p_idx = parent_of.get(idx)
        p_name = nodes[p_idx].get("name", f"node_{p_idx}") if p_idx is not None else None
        t = nd.get("translation", [0.0, 0.0, 0.0])
        length = math.sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2])
        if p_idx is not None or length > 0:
            rig_bones.append(
                {
                    "name": name,
                    "parent": p_name,
                    "length": round(max(length, 0.05) if p_idx is None else length, 5),
                }
            )

    # Audit vertex skinning weights (WEIGHTS_0)
    unnorm_count = 0
    neg_w_count = 0
    min_sum = 1.0
    max_sum = 1.0
    found_weights = False

    if buffers:
        for mesh in meshes:
            for prim in mesh.get("primitives", []):
                attrs = prim.get("attributes", {})
                if "WEIGHTS_0" in attrs:
                    found_weights = True
                    w_rows = read_accessor(gltf, buffers, attrs["WEIGHTS_0"])
                    for row in w_rows:
                        if any(w < -1e-5 for w in row):
                            neg_w_count += 1
                        s = sum(row)
                        min_sum = min(min_sum, s)
                        max_sum = max(max_sum, s)
                        if abs(s - 1.0) > 1e-3:
                            unnorm_count += 1

    rig_obj: Dict[str, Any] = {"standard": "gltf_skel"}
    if rig_bones:
        rig_obj["bones"] = rig_bones
    if found_weights or skins:
        rig_obj["skinning"] = {
            "unnormalizedVertexCount": unnorm_count,
            "negativeWeightCount": neg_w_count,
            "minWeightSum": round(min_sum, 5),
            "maxWeightSum": round(max_sum, 5),
            "nonInvertibleBindMatrices": non_inv_ibm,
            "negativeDeterminantBindMatrices": neg_det_ibm,
        }
    return rig_obj


def extract_from_gltf(
    path: Path, entity_type: str, expected_gait: Optional[str]
) -> Dict[str, Any]:
    gltf, buffers = load_gltf_or_glb(path)
    nodes = gltf.get("nodes", [])
    animations = gltf.get("animations", [])
    rig_obj = extract_gltf_rig_and_skinning(gltf, buffers)

    if not animations:
        frames = [
            {
                "t": 0.0,
                "position": nodes[0].get("translation", [0.0, 0.0, 0.0]) if nodes else [0.0, 0.0, 0.0],
                "scale": nodes[0].get("scale", [1.0, 1.0, 1.0]) if nodes else [1.0, 1.0, 1.0],
                "grounded": True,
            }
        ]
        return {
            "scene": {"dimension": "3d", "upAxis": "y", "handedness": "right", "gravity": 9.80665, "groundLevel": 0.0},
            "entities": [
                {
                    "id": path.stem,
                    "type": entity_type,
                    "params": {"expectedGait": expected_gait} if expected_gait else {},
                    "rig": rig_obj,
                    "frames": frames,
                }
            ],
        }

    anim = animations[0]
    samplers = anim.get("samplers", [])
    channels = anim.get("channels", [])

    time_grid: List[float] = []
    node_tracks: Dict[int, Dict[str, Tuple[List[float], List[List[float]]]]] = {}

    for ch in channels:
        target = ch.get("target", {})
        node_idx = target.get("node", 0)
        prop_path = target.get("path", "")
        sampler = samplers[ch["sampler"]]
        in_times = [r[0] for r in read_accessor(gltf, buffers, sampler["input"])]
        out_vals = read_accessor(gltf, buffers, sampler["output"])
        if len(in_times) > len(time_grid):
            time_grid = in_times
        node_tracks.setdefault(node_idx, {})[prop_path] = (in_times, out_vals)

    root_idx = 0
    for n_idx, trks in node_tracks.items():
        if "translation" in trks:
            root_idx = n_idx
            break

    frames: List[Dict[str, Any]] = []
    root_trk = node_tracks.get(root_idx, {})
    default_pos = nodes[root_idx].get("translation", [0.0, 0.0, 0.0]) if nodes else [0.0, 0.0, 0.0]
    default_scale = nodes[root_idx].get("scale", [1.0, 1.0, 1.0]) if nodes else [1.0, 1.0, 1.0]

    for f_idx, t in enumerate(time_grid):
        pos = default_pos
        if "translation" in root_trk and f_idx < len(root_trk["translation"][1]):
            pos = [round(v, 5) for v in root_trk["translation"][1][f_idx]]

        rot_euler = [0.0, 0.0, 0.0]
        quats_map: Dict[str, List[float]] = {}
        if "rotation" in root_trk and f_idx < len(root_trk["rotation"][1]):
            q = root_trk["rotation"][1][f_idx]
            rot_euler = quat_to_euler_deg(q[0], q[1], q[2], q[3])
            r_name = nodes[root_idx].get("name", f"node_{root_idx}") if root_idx < len(nodes) else "root"
            quats_map[r_name] = [round(v, 5) for v in q[:4]]

        scl = default_scale
        if "scale" in root_trk and f_idx < len(root_trk["scale"][1]):
            scl = [round(v, 5) for v in root_trk["scale"][1][f_idx]]

        joints: Dict[str, Dict[str, float]] = {}
        for n_idx, trks in node_tracks.items():
            if n_idx == root_idx or n_idx >= len(nodes):
                continue
            n_name = nodes[n_idx].get("name", f"node_{n_idx}").lower()
            if "rotation" in trks and f_idx < len(trks["rotation"][1]):
                q = trks["rotation"][1][f_idx]
                quats_map[n_name] = [round(v, 5) for v in q[:4]]
                eul = quat_to_euler_deg(q[0], q[1], q[2], q[3])
                if any(k in n_name for k in ("knee", "elbow", "hip", "shoulder", "ankle", "wrist", "neck", "spine")):
                    joints[n_name] = {
                        "flexion": abs(eul[0]) if ("knee" in n_name or "elbow" in n_name) else eul[0],
                        "abduction": eul[2],
                        "rotation": eul[1],
                    }

        frame_obj: Dict[str, Any] = {
            "t": round(t, 5),
            "position": pos,
            "rotation": rot_euler,
            "scale": scl,
            "grounded": pos[1] <= 0.15,
        }
        if quats_map:
            frame_obj["quaternions"] = quats_map
        if joints:
            frame_obj["joints"] = joints
        frames.append(frame_obj)

    params: Dict[str, Any] = {}
    if expected_gait:
        params["expectedGait"] = expected_gait

    return {
        "scene": {"dimension": "3d", "upAxis": "y", "handedness": "right", "gravity": 9.80665, "groundLevel": 0.0},
        "entities": [
            {
                "id": anim.get("name", path.stem),
                "type": entity_type,
                "params": params,
                "rig": rig_obj,
                "frames": frames,
            }
        ],
    }


# ---------------------------------------------------------------------------
# BVH (Biovision Hierarchy) Parser
# ---------------------------------------------------------------------------

def extract_from_bvh(
    path: Path, entity_type: str, expected_gait: Optional[str]
) -> Dict[str, Any]:
    lines = [ln.strip() for ln in path.read_text(encoding="utf-8").splitlines() if ln.strip()]
    motion_idx = lines.index("MOTION")

    # Parse HIERARCHY bones and offsets
    rig_bones: List[Dict[str, Any]] = []
    stack: List[str] = []
    current_bone: Optional[str] = None
    parent_map: Dict[str, Optional[str]] = {}

    for ln in lines[:motion_idx]:
        parts = ln.split()
        if parts[0] in ("ROOT", "JOINT") and len(parts) >= 2:
            b_name = parts[1]
            parent_map[b_name] = stack[-1] if stack else None
            current_bone = b_name
        elif parts[0] == "{":
            if current_bone:
                stack.append(current_bone)
        elif parts[0] == "}":
            if stack:
                stack.pop()
        elif parts[0] == "OFFSET" and len(parts) >= 4 and current_bone:
            ox, oy, oz = float(parts[1]), float(parts[2]), float(parts[3])
            length = math.sqrt(ox * ox + oy * oy + oz * oz)
            p_name = parent_map.get(current_bone)
            if p_name is not None or length > 0:
                rig_bones.append(
                    {
                        "name": current_bone,
                        "parent": p_name,
                        "length": round(max(length, 0.05) if p_name is None else length, 5),
                    }
                )
            current_bone = None

    frame_time = 1.0 / 30.0
    data_start = motion_idx + 1
    for i in range(motion_idx + 1, len(lines)):
        if lines[i].startswith("Frames:"):
            continue
        elif lines[i].startswith("Frame Time:"):
            frame_time = float(lines[i].split(":")[1].strip())
            data_start = i + 1
            break

    frames: List[Dict[str, Any]] = []
    for idx, line in enumerate(lines[data_start:]):
        vals = [float(x) for x in line.split()]
        if len(vals) < 6:
            continue
        pos = [round(vals[0], 4), round(vals[1], 4), round(vals[2], 4)]
        rot = [round(vals[4], 3), round(vals[5], 3), round(vals[3], 3)]
        frames.append(
            {
                "t": round(idx * frame_time, 5),
                "position": pos,
                "rotation": rot,
                "grounded": True,
            }
        )

    params: Dict[str, Any] = {}
    if expected_gait:
        params["expectedGait"] = expected_gait

    return {
        "scene": {"dimension": "3d", "upAxis": "y", "handedness": "right", "gravity": 9.80665, "groundLevel": 0.0},
        "entities": [
            {
                "id": path.stem,
                "type": entity_type,
                "params": params,
                "rig": {"standard": "bvh_mocap", "bones": rig_bones},
                "frames": frames,
            }
        ],
    }


# ---------------------------------------------------------------------------
# URDF (Unified Robot Description Format) Parser
# ---------------------------------------------------------------------------

def extract_from_urdf(path: Path, entity_type: str) -> Dict[str, Any]:
    tree = ET.parse(path)
    root = tree.getroot()
    entities: List[Dict[str, Any]] = []

    for link in root.findall("link"):
        link_name = link.attrib.get("name", "unnamed_link")
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass_el = inertial.find("mass")
        inertia_el = inertial.find("inertia")
        mass = float(mass_el.attrib.get("value", 1.0)) if mass_el is not None else 1.0
        rig_obj: Dict[str, Any] = {"standard": "urdf"}
        if inertia_el is not None:
            rig_obj["inertiaTensor"] = {
                "ixx": float(inertia_el.attrib.get("ixx", 0.0)),
                "iyy": float(inertia_el.attrib.get("iyy", 0.0)),
                "izz": float(inertia_el.attrib.get("izz", 0.0)),
            }
        entities.append(
            {
                "id": link_name,
                "type": entity_type,
                "params": {"mass": mass},
                "rig": rig_obj,
                "frames": [{"t": 0.0, "position": [0.0, 0.0, 0.0], "grounded": True}],
            }
        )

    return {
        "scene": {"dimension": "3d", "upAxis": "z", "handedness": "right", "gravity": 9.80665, "groundLevel": 0.0},
        "entities": entities,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Extract rig, skinning, and motion tracks from .gltf, .glb, .bvh, or .urdf into standardized verification JSON."
    )
    parser.add_argument("input", type=Path, help="Input .gltf, .glb, .bvh, or .urdf file.")
    parser.add_argument(
        "--entity-type",
        default="human",
        help="Entity type (human, quadruped, bird, fish, wheeled_vehicle, two_wheeled_vehicle, aircraft, rotorcraft, rigid_body).",
    )
    parser.add_argument("--expected-gait", default=None, help="Expected gait (walk, run, jump, trot, etc.).")
    parser.add_argument("-o", "--output", type=Path, default=None, help="Output JSON file path.")
    args = parser.parse_args()

    if not args.input.exists():
        print(f"Error: File '{args.input}' not found.", file=sys.stderr)
        return 2

    ext = args.input.suffix.lower()
    if ext in (".gltf", ".glb"):
        doc = extract_from_gltf(args.input, args.entity_type, args.expected_gait)
    elif ext == ".bvh":
        doc = extract_from_bvh(args.input, args.entity_type, args.expected_gait)
    elif ext in (".urdf", ".xml"):
        doc = extract_from_urdf(args.input, args.entity_type)
    else:
        print(f"Unsupported file extension '{ext}'. Expected .gltf, .glb, .bvh, or .urdf.", file=sys.stderr)
        return 2

    out_str = json.dumps(doc, indent=2)
    if args.output:
        args.output.write_text(out_str + "\n", encoding="utf-8")
        print(f"Wrote extracted motion JSON to {args.output}")
    else:
        print(out_str)
    return 0


if __name__ == "__main__":
    sys.exit(main())
