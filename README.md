# 2D & 3D Model, Rigging, Animation & Physics Verification Skill

A community-shareable **Claude Skill / Agent Skill** (`2d-3d-model-physics-verification`) that audits, analyzes, and verifies 2D and 3D models, skeletal rigs, vertex skinning, keyframe tracks, and procedural animation code against **industry 2D/3D rigging & animation standards, classical mechanics, human biomechanics, animal locomotion, and vehicle/transportation dynamics**.

Includes zero-dependency verification engines in both **Python** ([`scripts/verify_physics.py`](./scripts/verify_physics.py)) and **TypeScript / Node.js** ([`scripts/verify_physics.ts`](./scripts/verify_physics.ts)), plus an asset & rig extractor for `.gltf`, `.glb`, `.bvh`, and `.urdf` files ([`scripts/extract_gltf_motion.py`](./scripts/extract_gltf_motion.py)).

---

## Standards & Domains Verified

| Domain | Standards, Laws, Rules & Formulas Checked | Reference Manual |
| :--- | :--- | :--- |
| **2D/3D Modeling, Rigging & Animation Standards** | **glTF 2.0 (ISO/IEC 12113)**, **OpenUSD (`UsdGeom`/`UsdSkel`/`UsdPhysics`)**, **VRM 1.0 / Unity Mecanim / UE5 Humanoid**, **URDF (`ROS REP-103`) / SDF / MuJoCo**, **BVH**, **Spine 2D**, **Lottie (ISO/IEC 21794) / SVG / CSS**. Checks DAG acyclicity, zero-length bones, bilateral limb symmetry ($\le 1.5\%$), Partition of Unity skinning weight normalization ($\sum w_j = 1, w_j \ge 0$), bind matrix invertibility ($\det > 0$), LBS candy-wrapper axial twist limit ($\le 90^\circ$), unit quaternion norm & **hemisphere shortest-path continuity** ($q_i \cdot q_{i-1} \ge 0$), Euler gimbal lock, 2-bone **IK over-extension pop** ($D \le 0.992(L_1+L_2)$) & pole collinearity, **3D Inertia Tensor Triangle Inequality** ($I_{xx}+I_{yy} \ge I_{zz}$), Cubic Bezier time monotonicity ($x_1, x_2 \in [0,1]$), and 2D deforming mesh triangle winding flip. | [`references/modeling-rigging-and-animation-standards.md`](./references/modeling-rigging-and-animation-standards.md) |
| **Classical Mechanics** | $C^0/C^1$ continuity (teleportation & velocity spikes), minimum-jerk trajectory profiles, ballistic free-fall gravity ($a_y = -g$), flight time vs. apex height ($T = \sqrt{8\Delta h/g}$), static **Center of Mass (CoM)** vs. **Base of Support (BoS)** convex hull, dynamic **Zero Moment Point (ZMP)**, squash-and-stretch volume/area conservation ($s_x s_y s_z = 1$). | [`references/classical-mechanics-formulas.md`](./references/classical-mechanics-formulas.md) |
| **Human Biomechanics** | Winter's anthropometric segment proportions, ISB clinical joint Range of Motion (RoM) & hinge locks (knee/elbow hyperextension $< -5^\circ$ and lateral varus/valgus lock), Alexander **Froude Number** ($Fr = v^2 / (g L_{\text{leg}})$, walk-to-run transition at $Fr \approx 0.5$), gait **Duty Factor** ($\beta > 0.5$ walk with double support vs. $\beta < 0.5$ run with flight phase), pelvis vertical oscillation, stance **footskate** (sliding), jump anticipation crouch impulse ($\bar{a}_{\text{push}} = g \Delta h / \Delta y_{\text{crouch}}$), proximal-to-distal throw sequencing. | [`references/human-biomechanics.md`](./references/human-biomechanics.md) |
| **Animal Locomotion** | **Hildebrand Quadruped Gait Matrix** (duty factor $\beta$ & 4-limb phase offsets $[\phi_{\text{LH}}, \phi_{\text{LF}}, \phi_{\text{RH}}, \phi_{\text{RF}}]$ for walk, trot, pace, canter, transverse/rotary gallop), digitigrade/unguligrade joint bending directions, avian/insect flapping **Strouhal Number** ($0.20 \le St = fA/v \le 0.40$), wing-stroke vs. body-heave reaction phase, aquatic undulatory Strouhal & wave slip ratio ($v / v_{\text{wave}} \in [0.55, 0.85]$). | [`references/animal-locomotion.md`](./references/animal-locomotion.md) |
| **Vehicle & Transport Dynamics** | **SAE J670 / ISO 8855** wheel no-slip rolling ($\omega = v_x / R_{\text{wheel}}$, slip ratio $\kappa$), **Ackermann steering geometry** ($\dot{\psi} = v \tan\delta / L_{\text{wb}}$, $\delta_{\text{inner}} > \delta_{\text{outer}}$), lateral tire grip limit ($a_{\text{lat}} \le \mu g$), longitudinal suspension pitch (brake dive vs. acceleration squat), 4-wheel outward roll vs. **2-wheel (motorcycle/bicycle) inward lean** ($\theta = \arctan(v^2 / Rg)$), fixed-wing stall speed & coordinated bank angle, **quadcopter/helicopter thrust-vector pitch/roll coupling** ($\theta = -\arctan(a_{\text{fwd}} / (g + a_y))$). | [`references/vehicle-and-transport-dynamics.md`](./references/vehicle-and-transport-dynamics.md) |

---

## Installation

### Option 1: Claude Code (Global or Project Skill)
Clone or copy this folder into your Claude skills directory:

```bash
# Personal (available across all projects)
git clone https://github.com/thebazshah/2D-3D-model-physics-verification.git ~/.claude/skills/2d-3d-model-physics-verification

# Or Project-scoped
git clone https://github.com/thebazshah/2D-3D-model-physics-verification.git .claude/skills/2d-3d-model-physics-verification
```

### Option 2: Agent Skills / Antigravity Standard (`.agents/skills/`)
```bash
git clone https://github.com/thebazshah/2D-3D-model-physics-verification.git .agents/skills/2d-3d-model-physics-verification
```

### Option 3: Claude.ai / Claude Desktop Upload
Zip the contents of this directory (`zip -r 2d-3d-model-physics-verification.zip SKILL.md references/ scripts/ examples/`) and upload it under **Settings -> Capabilities -> Skills** in Claude.

---

## Quick Start: Running the Verification Scripts

### 1. Run with Python 3 (Zero Dependencies)
```bash
python3 scripts/verify_physics.py examples/sample_motions.json
```
Add `--json` for machine-readable JSON output or `--strict` to fail on warnings:
```bash
python3 scripts/verify_physics.py examples/sample_motions.json --json
```

### 2. Run with TypeScript / Node.js (Node 22.6+/24+, `tsx`, `bun`, or `deno`)
```bash
node scripts/verify_physics.ts examples/sample_motions.json
```

### 3. Verify a Procedural Three.js / Web Animation in Code
```bash
node examples/verify_threejs_example.ts
```

### 4. Extract & Verify a `.gltf`, `.glb`, `.bvh`, or `.urdf` File
```bash
python3 scripts/extract_gltf_motion.py character_walk.glb \
  --entity-type human --expected-gait walk -o extracted_motion.json

python3 scripts/verify_physics.py extracted_motion.json
```

---

## Repository Structure

```text
2D-3D-model-physics-verification/
├── SKILL.md                                                 # Core skill instructions & workflow for Claude
├── README.md                                                # Community installation & usage guide
├── package.json                                             # Node/TypeScript package configuration
├── scripts/
│   ├── verify_physics.py                                    # Standalone Python 3 standards & physics verifier
│   ├── verify_physics.ts                                    # Standalone TypeScript standards & physics verifier
│   └── extract_gltf_motion.py                               # .gltf / .glb / .bvh / .urdf rig & motion extractor
├── references/
│   ├── modeling-rigging-and-animation-standards.md          # glTF/USD/VRM/URDF/Spine/Lottie rig, skinning, IK & curve standards
│   ├── classical-mechanics-formulas.md                      # Kinematics, gravity, CoM/ZMP balance, collisions, squash & stretch
│   ├── human-biomechanics.md                                # Joint RoM limits, Froude number, duty factor, footskate, jumps
│   ├── animal-locomotion.md                                 # Quadruped Hildebrand gaits, avian/insect Strouhal flight, fish swimming
│   ├── vehicle-and-transport-dynamics.md                    # Wheel slip, Ackermann steering, car roll vs bike lean, drone pitch coupling
│   └── motion-schema.md                                     # Standardized JSON input schema for 2D & 3D rigs and animations
└── examples/
    ├── sample_motions.json                                  # Multi-entity benchmark (rig standards, human, quadruped, bike, drone)
    └── verify_threejs_example.ts                            # Programmatic verification example for procedural 3D loops
```
