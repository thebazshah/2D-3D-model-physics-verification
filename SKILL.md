---
name: 2d-3d-model-physics-verification
description: >-
  Analyzes and verifies 2D and 3D models, skeletal rigs, vertex skinning, keyframe
  animations, and procedural movement code against industry 2D/3D rigging/animation
  standards (glTF 2.0, OpenUSD UsdSkel/UsdPhysics, VRM/Mecanim Humanoid, URDF/MuJoCo,
  BVH, Spine 2D, Lottie/SVG/CSS), classical mechanics, human biomechanics, animal
  locomotion, and transportation/vehicle dynamics. Use this skill whenever the user
  asks to create, review, debug, or validate 2D/3D rigs, animations, character/creature
  movements, vehicle physics, or .gltf/.glb/.bvh/.urdf files so movements look natural.
---

# 2D & 3D Model, Rigging, Animation & Physics Verification Skill

Use this skill to audit, verify, and fix 2D/3D models, skeletal rigs, skinning weights, and defined movements so that every asset and motion obeys industry 2D/3D rigging standards, classical mechanics, anatomical constraints, biological gait laws, and vehicle dynamics.

---

## Workflow

### Step 1: Identify the Model, Rig & Movement Source
Determine what the user is building or inspecting:
1. **Procedural Animation / Engine Code** (Three.js, React Three Fiber, Babylon.js, Unity C#, Unreal C++, Godot GDScript, Blender Python, HTML5 Canvas/SVG/CSS/GSAP/Lottie/Spine 2D):
   - Extract the coordinate convention (`dimension`: `"2d"` or `"3d"`, `upAxis`: `"y"`, `"z"`, or `"-y"` for 2D screen space, `handedness`: `"right"` or `"left"`, and `metersPerUnit`).
   - Sample the rig metadata and update loop / keyframe evaluator over $1\text{--}2$ full motion cycles ($30\text{--}60\text{ FPS}$) into the standardized JSON schema in [motion-schema.md](./references/motion-schema.md).
2. **3D Asset / Motion Capture / Physics Rig Files** (`.gltf`, `.glb`, `.bvh`, `.urdf`, `.xml`):
   - Extract the skeletal hierarchy, skinning audit, inertia tensors, and animation tracks into standardized JSON using:
     ```bash
     python3 scripts/extract_gltf_motion.py <path/to/asset.glb> --entity-type <type> --expected-gait <gait> -o motion.json
     ```
3. **Direct Keyframe / Trajectory JSON**:
   - Map the rig and keyframes to the schema in [motion-schema.md](./references/motion-schema.md).

---

### Step 2: Run the Automated Standards, Physics & Biomechanics Verifier
Execute either the **Python** or **TypeScript** verification engine depending on the project runtime:

- **Python CLI** (zero external dependencies):
  ```bash
  python3 scripts/verify_physics.py motion.json
  ```
- **TypeScript / Node.js CLI** (runs natively in Node 22.6+/24+, `tsx`, `bun`, or `deno`):
  ```bash
  node scripts/verify_physics.ts motion.json
  ```
- **Programmatic Import in TypeScript / JavaScript Tests**:
  See [verify_threejs_example.ts](./examples/verify_threejs_example.ts) to import `verifyScene` from [verify_physics.ts](./scripts/verify_physics.ts) directly inside a Three.js or Web test suite.

---

### Step 3: Domain-Specific Standards & Physics Checklists

Consult the reference manuals below when designing rigs/movements or diagnosing verifier findings:

#### 0. 2D & 3D Modeling, Rigging, Skinning & Animation Standards ([modeling-rigging-and-animation-standards.md](./references/modeling-rigging-and-animation-standards.md))
- **Coordinate System & Unit Standards**:
  - **glTF 2.0 / VRM / Three.js**: Right-Handed, $+Y$ Up, $1\text{ unit} = 1\text{ m}$.
  - **OpenUSD (`UsdGeom`/`UsdSkel`)**: Right-Handed, $+Y$ or $+Z$ Up, `metersPerUnit` ($0.01$ or $1.0$).
  - **Unreal Engine**: Left-Handed, $+Z$ Up, $+X$ Forward, `metersPerUnit = 0.01` ($\text{cm}$).
  - **Unity (`Mecanim`)**: Left-Handed, $+Y$ Up, $+Z$ Forward, `metersPerUnit = 1.0`.
  - **Blender / URDF (`ROS REP-103`) / MuJoCo**: Right-Handed, $+Z$ Up, `metersPerUnit = 1.0`.
  - **Lottie / SVG / CSS / Canvas 2D**: Screen-space $+X$ Right, $+Y$ Down (`upAxis: "-y"`). **Spine 2D**: $+X$ Right, $+Y$ Up (`upAxis: "y"`).
- **Skeletal Topology (`VRM 1.0` / `UsdSkel` / `Mecanim`)**:
  - Strict Directed Acyclic Graph (no parent-child cycles), non-zero bone lengths ($L_{\text{bone}} > 10^{-4}\text{ m}$), bilateral left-right limb symmetry ($\le 1.5\%$ difference), and invertible bind matrices with positive determinant ($\det(\mathbf{M}_{\text{IBM}}) > 0$).
- **Vertex Skinning Partition of Unity (`glTF JOINTS_0/WEIGHTS_0`, `UsdSkel`)**:
  - Every vertex must have non-negative bone weights ($w_j \ge 0$) summing to $\sum_{j=1}^K w_j = 1.0 \pm 10^{-4}$.
  - Single-bone axial twist under Linear Blend Skinning (LBS) must not exceed $|\Delta\psi| \le 90^\circ$ without twist helper bones or Dual Quaternion Skinning (DQS) to prevent **"candy-wrapper" volume collapse** ($r = r_0 \cos(\Delta\psi / 2)$).
- **Rotation & Curve Interpolation Standards**:
  - Quaternions must be unit length ($\|q\| = 1 \pm 10^{-3}$) and maintain **hemisphere shortest-path continuity** ($q_i \cdot q_{i-1} \ge 0$) so consecutive keyframes never spin $360^\circ$ the long way around.
  - Euler angles must avoid $\pm 90^\circ$ middle-axis **Gimbal Lock**.
  - Cubic Bezier timing handles `cubic-bezier(x1, y1, x2, y2)` must keep $x_1, x_2 \in [0, 1]$ for monotonic time.
- **Inverse Kinematics (IK) Standards**:
  - 2-bone IK chains ($L_1 + L_2$) must keep target distance $D \le 0.992(L_1 + L_2)$ to avoid infinite angular velocity **"IK knee/elbow pop"** at full extension, and keep the pole vector non-collinear ($|\hat{u}_{\text{target}} \cdot \hat{u}_{\text{pole}}| \le 0.98$).
- **Physics Rig Inertia Tensor (`URDF` / `SDF` / `MuJoCo` / `UsdPhysics`)**:
  - Principal moments of inertia $(I_{xx}, I_{yy}, I_{zz})$ must be strictly positive and satisfy the **3D Triangle Inequality**: $I_{xx} + I_{yy} \ge I_{zz}$, $I_{yy} + I_{zz} \ge I_{xx}$, $I_{zz} + I_{xx} \ge I_{yy}$.
- **2D Deforming Mesh Winding (`Spine 2D` / `Live2D` / `Lottie`)**:
  - Deforming 2D triangles must preserve their bind-pose signed cross-product area orientation (no inside-out triangle flips).

#### 1. Universal Classical Mechanics ([classical-mechanics-formulas.md](./references/classical-mechanics-formulas.md))
- **Continuity & Minimum Jerk**: No $C^0$ teleportation spikes ($|\vec{v}_i| \le 6 \cdot \text{median}(|\vec{v}|)$). Smooth biological/mechanical point-to-point motion follows a minimum-jerk profile $x(\tau) = x_0 + \Delta x(10\tau^3 - 15\tau^4 + 6\tau^5)$.
- **Ballistic Gravity Arcs**: Airborne entities without thrust/lift must accelerate vertically at $a_y = -g \approx -9.81\text{ m/s}^2$ ($\pm 25\%$) with near-zero horizontal acceleration ($|\vec{a}_{xz}| \le 2.5\text{ m/s}^2$).
- **Static & Dynamic Balance (CoM & ZMP)**:
  - Stationary poses must project the Center of Mass ($\vec{p}_{\text{CoM},xz}$) inside the 2D convex hull of active ground contacts (**Base of Support**).
  - Dynamic movements must keep the **Zero Moment Point** $x_{\text{ZMP}} = x_{\text{CoM}} - \frac{y_{\text{CoM}}}{a_y + g}a_x$ inside the support polygon during stance.
- **Volume Conservation (Squash & Stretch)**: Incompressible tissue deformations must satisfy $s_x \cdot s_y \cdot s_z \approx 1.0$ in 3D ($s_x \cdot s_y \approx 1.0$ in 2D).

#### 2. Human Biomechanics ([human-biomechanics.md](./references/human-biomechanics.md))
- **Anatomical Joint Range of Motion (RoM) & Hinge Locks**:
  - Knees and elbows are modified **hinge joints**: flexion $\in [0^\circ, 150^\circ]$, no hyperextension ($< -5^\circ$), and no lateral varus/valgus bending ($|\text{abduction}| \le 5^\circ$).
- **Bipedal Gait Laws**:
  - **Froude Number**: $Fr = \frac{v^2}{g \cdot L_{\text{leg}}}$. Walking requires $Fr < 0.50$; walk-to-run transition occurs at $Fr \approx 0.50$.
  - **Duty Factor ($\beta = t_{\text{stance}} / T_{\text{stride}}$)**: Walking requires $\beta > 0.50$ (with double-support intervals and zero flight phase); Running requires $\beta < 0.50$ (with aerial flight intervals).
  - **Pelvis Vertical Bob**: Walking uses an inverted-pendulum vault (pelvis highest at mid-stance, amplitude $3\text{--}6\text{ cm}$); running uses a spring-loaded inverted pendulum (pelvis lowest at mid-stance compression).
  - **Zero Footskate**: Grounded feet must remain stationary in world space ($|\vec{v}_{\text{foot},xz}| \approx 0$) without penetrating $y < y_{\text{ground}}$.
- **Jumping & Throwing**:
  - Jumps of height $\Delta h$ require an anticipation crouch of depth $\Delta y_{\text{crouch}}$ such that average leg push-off acceleration $\bar{a}_{\text{push}} = g \frac{\Delta h}{\Delta y_{\text{crouch}}} \le 3.5 g$, followed by landing flexion absorption.

#### 3. Animal Locomotion ([animal-locomotion.md](./references/animal-locomotion.md))
- **Quadruped Hildebrand Gaits**: Verify duty factor $\beta$ and 4-limb phase offsets $[\phi_{\text{LH}}, \phi_{\text{LF}}, \phi_{\text{RH}}, \phi_{\text{RF}}]$ for walk, trot, pace, canter, and transverse/rotary gallop.
- **Avian Flight & Aquatic Swimming Strouhal Law**:
  - Cruising Strouhal number $St = \frac{f \cdot A}{v}$ must lie in $[0.20, 0.40]$.
  - Bird torso heave must oppose wing vertical stroke ($180^\circ$ phase offset: body accelerates upward on downstroke).
  - Fish/snake undulatory wave speed $v_{\text{wave}}$ must exceed forward swimming speed $v$ ($\text{slip} = v / v_{\text{wave}} \in [0.55, 0.85]$).

#### 4. Transportation & Vehicle Dynamics ([vehicle-and-transport-dynamics.md](./references/vehicle-and-transport-dynamics.md))
- **Wheel Rolling No-Slip**: $\omega_{\text{wheel}} = \frac{v_x}{R_{\text{wheel}}}$ (slip ratio $|\kappa| \le 0.15$ unless skidding).
- **Ackermann Turning & Lateral Grip**: Yaw rate must match steering geometry $\dot{\psi} = \frac{v \tan\delta}{L_{\text{wheelbase}}}$, inner wheel steers sharper than outer wheel ($\delta_{\text{inner}} > \delta_{\text{outer}}$), and centripetal acceleration $a_{\text{lat}} = \frac{v^2}{R} \le \mu g$ unless drifting.
- **Suspension Load Transfer & Cornering Lean**:
  - Cars/trucks squat (pitch nose-up) under forward acceleration, dive (pitch nose-down) under braking, and roll **outward** in turns.
  - Motorcycles/bicycles **must lean inward** into turns at $\phi_{\text{lean}} = \arctan\left(\frac{v^2}{R g}\right)$.
- **Aircraft & Rotorcraft (Quadcopters/Helicopters)**:
  - Fixed-wing planes bank inward at $\phi_{\text{bank}} = \arctan\left(\frac{v \dot{\psi}}{g}\right)$ and stay above stall speed $v \ge v_{\text{stall}}\sqrt{n}$.
  - Quadcopters and helicopters **must tilt their thrust vector** in the direction of horizontal acceleration: pitch nose-down ($\theta = -\arctan\left(\frac{a_{\text{fwd}}}{g + a_y}\right)$) to accelerate forward, and pitch nose-up (flare) to decelerate/stop.

---

### Step 4: Remediate & Re-Verify
Whenever the verifier script reports issues (or score $< 100$):
1. Present a concise diagnostic table showing each `code`, `frame_time`, `actual` vs. `expected` value, and the governing physical or rigging standard.
2. Modify the rig hierarchy, skin weights, keyframes, or procedural animation math to satisfy the standard.
3. Re-run `scripts/verify_physics.py` or `scripts/verify_physics.ts` to confirm the fix achieves `[PASS]` with a score of `100/100`.
