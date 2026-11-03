# Standardized Motion & Rig Verification JSON Schema

Both [`scripts/verify_physics.py`](../scripts/verify_physics.py) and [`scripts/verify_physics.ts`](../scripts/verify_physics.ts) accept a unified JSON document describing one or more 2D or 3D rigged models, their skeletal/skinning/physics properties, and their sampled animation keyframes.

When analyzing source code (Three.js, Unity C#, Unreal C++, Blender Python, CSS/SVG/Canvas/Lottie/Spine 2D) or binary asset files (`.gltf`, `.glb`, `.bvh`, `.urdf`), either:
1. Extract or sample the rig and animation into this JSON format and run the verifier script, or
2. Use [`scripts/extract_gltf_motion.py`](../scripts/extract_gltf_motion.py) for `.gltf` / `.glb` / `.bvh` / `.urdf` files.

---

## Top-Level Structure

```json
{
  "scene": {
    "dimension": "3d",
    "upAxis": "y",
    "handedness": "right",
    "gravity": 9.80665,
    "groundLevel": 0.0,
    "metersPerUnit": 1.0
  },
  "entities": [
    {
      "id": "hero_character",
      "type": "human",
      "params": {
        "mass": 75.0,
        "height": 1.75,
        "legLength": 0.92,
        "expectedGait": "walk"
      },
      "rig": {
        "standard": "vrm_humanoid",
        "bones": [
          { "name": "hips", "parent": null, "length": 0.12 },
          { "name": "upperLeg_L", "parent": "hips", "length": 0.43, "mirrorBone": "upperLeg_R" },
          { "name": "upperLeg_R", "parent": "hips", "length": 0.43, "mirrorBone": "upperLeg_L" },
          { "name": "lowerLeg_L", "parent": "upperLeg_L", "length": 0.43, "mirrorBone": "lowerLeg_R" },
          { "name": "lowerLeg_R", "parent": "upperLeg_R", "length": 0.43, "mirrorBone": "lowerLeg_L" }
        ],
        "skinning": {
          "maxInfluencesPerVertex": 4,
          "unnormalizedVertexCount": 0,
          "negativeWeightCount": 0,
          "minWeightSum": 1.0,
          "maxWeightSum": 1.0,
          "nonInvertibleBindMatrices": 0,
          "negativeDeterminantBindMatrices": 0
        },
        "inertiaTensor": {
          "ixx": 1.2,
          "iyy": 1.1,
          "izz": 0.4
        },
        "ikChains": [
          { "name": "leg_L", "upperLength": 0.43, "lowerLength": 0.43 }
        ]
      },
      "frames": [
        {
          "t": 0.0,
          "position": [0.0, 0.95, 0.0],
          "rotation": [0.0, 0.0, 0.0],
          "quaternions": {
            "hips": [0.0, 0.0, 0.0, 1.0],
            "upperLeg_L": [0.1736, 0.0, 0.0, 0.9848]
          },
          "scale": [1.0, 1.0, 1.0],
          "grounded": true,
          "contacts": [[-0.1, 0.0, 0.05], [0.1, 0.0, -0.15]],
          "joints": {
            "knee_l": { "flexion": 12.0, "abduction": 0.0 },
            "knee_r": { "flexion": 5.0, "abduction": 0.0 }
          },
          "boneTwists": {
            "forearm_l": 35.0
          },
          "ikTargets": {
            "leg_L": {
              "rootPos": [-0.1, 0.85, 0.0],
              "targetPos": [-0.1, 0.0, 0.05],
              "poleVector": [0.0, 0.0, 1.0]
            }
          },
          "bezierHandles": [0.42, 0.0, 0.58, 1.0],
          "feet": {
            "left": { "position": [-0.1, 0.0, 0.25], "grounded": true },
            "right": { "position": [0.1, 0.0, -0.25], "grounded": true }
          }
        }
      ]
    }
  ]
}
```

---

## Field Definitions

### `scene` Object
| Field | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `dimension` | `"2d"` \| `"3d"` | `"3d"` | Coordinate dimensionality. In `"2d"`, positions can be `[x, y]`. |
| `upAxis` | `"y"` \| `"z"` \| `"-y"` | `"y"` | Vertical axis opposing gravity (`"-y"` for 2D screen coordinates where $+Y$ points down). |
| `handedness` | `"right"` \| `"left"` | `"right"` | Coordinate system handedness (`"right"` for glTF/OpenUSD/Blender/ROS, `"left"` for Unity/Unreal). |
| `gravity` | `number` | `9.80665` | Gravitational acceleration magnitude in $\text{m/s}^2$. |
| `groundLevel` | `number` | `0.0` | Height of the flat support plane along `upAxis` (in meters). |
| `metersPerUnit` | `number` | `1.0` | Conversion factor from scene units/pixels to meters (e.g., `0.01` for cm/Unreal, `0.005` for $200\text{ px/m}$). |

### `rig` Object (Optional per Entity — Rigging, Skinning & Physics Standards)
| Field | Type | Description |
| :--- | :--- | :--- |
| `standard` | `string` | Optional target rig profile: `"vrm_humanoid"`, `"gltf_skel"`, `"usd_skel"`, `"urdf"`, `"spine_2d"`, `"lottie_2d"`. |
| `bones` | `array` | List of `{ name: string, parent: string \| null, length: number, mirrorBone?: string }`. Verifies DAG acyclicity, single root, non-zero bone lengths ($L > 10^{-4}\text{ m}$), bilateral left-right symmetry ($\le 1.5\%$ difference), and required humanoid bones. |
| `skinning` | `object` | Summary of vertex skinning audit (`unnormalizedVertexCount`, `negativeWeightCount`, `minWeightSum`, `maxWeightSum`, `nonInvertibleBindMatrices`, `negativeDeterminantBindMatrices`). |
| `inertiaTensor` | `object` | Principal moments of inertia `{ ixx, iyy, izz }` in $\text{kg}\cdot\text{m}^2$. Verifies positivity and the 3D Inertia Triangle Inequality ($I_{xx} + I_{yy} \ge I_{zz}$). |
| `ikChains` | `array` | List of 2-bone IK chains `{ name, upperLength, lowerLength }` used to check reachability and over-extension pop against `frames[].ikTargets`. |

### Additional Per-Frame Fields (`frames[]`) for Animation Standards
- `quaternions` (`Record<string, [x, y, z, w]>`): Joint rotation quaternions. Verifies unit norm ($\|q\| = 1 \pm 10^{-3}$) and hemisphere shortest-path continuity ($q_i \cdot q_{i-1} \ge 0$).
- `eulerOrder` (`string`, default `"XYZ"`): Intrinsic Euler order used by `rotation` to check for middle-axis $\pm 90^\circ$ **Gimbal Lock**.
- `boneTwists` (`Record<string, number>`): Single-bone axial twist angles in degrees. Flags **LBS candy-wrapper collapse** when $|\Delta\psi| > 90^\circ$ without twist redistribution bones.
- `ikTargets` (`Record<string, { rootPos: number[], targetPos: number[], poleVector?: number[] }>`): Evaluates 2-bone IK target distance $D$ against $L_1 + L_2$ (flagging over-extension pop when $D > 0.992(L_1 + L_2)$) and pole-vector collinearity ($|\hat{u}_{\text{target}} \cdot \hat{u}_{\text{pole}}| > 0.98$).
- `bezierHandles` (`[x1, y1, x2, y2]`): Cubic Bezier timing control points (CSS / Lottie / Spine / glTF spline). Verifies time monotonicity $x_1, x_2 \in [0, 1]$.
- `meshTriangles2D` (`array` of `[[ax, ay], [bx, by], [cx, cy]]`): 2D deforming mesh triangles (Spine/Live2D/Lottie). Verifies that no triangle flips its winding orientation relative to Frame 0.
