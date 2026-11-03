# 2D & 3D Modeling, Rigging, Skinning & Animation Standards Reference

This reference documents the industry-wide technical standards for 2D/3D models, skeletal rigs, vertex skinning, rotation interpolation, inverse kinematics (IK), physics descriptions, and animation principles enforced by the skill.

---

## 1. Coordinate Systems, Units & File Format Standards

Mismatched handedness, up-axes, or unit scales are the #1 cause of broken gravity ($9.81\text{ m/s}^2$ vs. $981\text{ cm/s}^2$) and inverted rotations.

| Standard / Engine / Format | Dimension | Handedness | Up Axis | Forward Axis | Default Unit Scale (`metersPerUnit`) | Governing Specification |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **glTF 2.0 / GLB / VRM 1.0** | 3D | Right-Handed | $+Y$ | $+Z$ (VRM) / $-Z$ (Cam) | $1.0\text{ m}$ | Khronos glTF 2.0 (ISO/IEC 12113) |
| **Three.js / Babylon.js / WebXR** | 3D | Right-Handed | $+Y$ | $-Z$ / $+Z$ | $1.0\text{ m}$ | W3C WebXR / Khronos WebGL |
| **OpenUSD (`UsdGeom` / `UsdSkel`)** | 3D | Right-Handed | $+Y$ or $+Z$ | $-Z$ or $+Y$ | `metersPerUnit` metadata ($0.01$ or $1.0$) | Alliance for OpenUSD (AOUSD) |
| **Unity (`Humanoid` / `Mecanim`)** | 3D | Left-Handed | $+Y$ | $+Z$ | $1.0\text{ m}$ | Unity Humanoid Avatar Standard |
| **Unreal Engine (`UE5 Skeleton`)** | 3D | Left-Handed | $+Z$ | $+X$ | $0.01\text{ m}$ ($1\text{ cm}$) | Epic Games UE Skeleton Standard |
| **Blender / 3ds Max** | 3D | Right-Handed | $+Z$ | $-Y$ | $1.0\text{ m}$ | Blender Armature / FBX Export |
| **URDF / SDF / ROS (`REP-103`)** | 3D | Right-Handed | $+Z$ | $+X$ | $1.0\text{ m}$ (SI: kg, m, s, rad) | ROS REP-103 / Open Robotics SDF |
| **MuJoCo (`MJCF` XML)** | 3D | Right-Handed | $+Z$ | $+X$ | $1.0\text{ m}$ | DeepMind MuJoCo MJCF |
| **BVH (Biovision Hierarchy)** | 3D | Right-Handed | $+Y$ | $+Z$ | Typically cm ($0.01$) or in ($0.0254$) | Biovision Mocap Standard |
| **Spine 2D / DragonBones** | 2D | Right-Handed 2D | $+Y$ (Up) | $+X$ (Right) | Pixels ($\sim 0.005\text{--}0.01\text{ m/px}$) | Esoteric Software Spine JSON/Binary |
| **Lottie (Bodymovin) / SVG / CSS / Canvas** | 2D | Screen-Space 2D | $-Y$ ($+Y$ Down) | $+X$ (Right) | Pixels (`upAxis: "-y"`) | ISO/IEC 21794 (Lottie) / W3C SVG & CSS |

---

## 2. Skeletal Hierarchy & Humanoid Rigging Standards (`VRM`, `UsdSkel`, `Mecanim`)

### 2.1 Directed Acyclic Graph (DAG) & Bone Topology
1. **Single Root Requirement**: An articulated skeleton must form a strict tree (DAG with no parent-child cycles) rooted at a single pelvis/hips/root node (or `world` $\to$ `root` $\to$ `hips`).
2. **Non-Zero Bone Length**: Every articulated bone segment (parent joint $\vec{p}_{\text{parent}}$ to child joint $\vec{p}_{\text{child}}$) must have strictly positive length:
   $$L_{\text{bone}} = \|\vec{p}_{\text{child}} - \vec{p}_{\text{parent}}\| > 10^{-4}\text{ m}$$
   Zero-length bones cause division-by-zero (`NaN`) in IK solvers, look-at constraints, and automatic skinning weight heatmaps.
3. **Bilateral Symmetry**: Left and right homologous limb segments (`upperLeg_L` vs `upperLeg_R`, `upperArm_L` vs `upperArm_R`, etc.) must match in rest-pose length within $1.5\%$:
   $$\frac{|L_{\text{left}} - L_{\text{right}}|}{\max(L_{\text{left}}, L_{\text{right}})} \le 0.015$$
4. **Invertible Bind Matrices (`inverseBindMatrices`)**: In glTF 2.0 and `UsdSkel`, every joint's $4 \times 4$ inverse bind matrix $\mathbf{M}_{\text{IBM}}$ must be finite and non-singular ($|\det(\mathbf{M}_{\text{IBM}})| > 10^{-6}$) with a valid last row $[0, 0, 0, 1]$. Negative determinants ($\det < 0$) indicate unapplied negative mirroring scale, which flips mesh normals inside-out.

### 2.2 Standard Humanoid Bone Naming & Hierarchy (`VRM 1.0` / `Mecanim`)
- **Required Core Bones**: `hips` $\to$ `spine` $\to$ `chest` $\to$ `neck` $\to$ `head`; `leftUpperArm` $\to$ `leftLowerArm` $\to$ `leftHand`; `rightUpperArm` $\to$ `rightLowerArm` $\to$ `rightHand`; `leftUpperLeg` $\to$ `leftUpperLeg` $\to$ `leftFoot`; `rightUpperLeg` $\to$ `rightLowerLeg` $\to$ `rightFoot`.

---

## 3. Vertex Skinning & Mesh Deformation Standards

### 3.1 Partition of Unity (Skin Weight Normalization)
In **Linear Blend Skinning (LBS)** and **Dual Quaternion Skinning (DQS)** (glTF `JOINTS_0` / `WEIGHTS_0`, `UsdSkel:jointWeights`), a vertex $\vec{v}$ deformed by $K$ joint matrices $\mathbf{T}_j$ is:
$$\vec{v}' = \sum_{j=1}^{K} w_j \mathbf{T}_j \vec{v}$$
- **Non-Negativity**: $w_j \ge 0$ for all $j$.
- **Partition of Unity**:
  $$\sum_{j=1}^{K} w_j = 1.0 \quad (\text{tolerance } \pm 10^{-4})$$
- **Violation (`SKIN_WEIGHTS_NOT_NORMALIZED`)**: If $\sum w_j \neq 1.0$, translating the character away from the world origin $(0,0,0)$ scales or shears the vertex toward the origin!

### 3.2 Axial Twist & "Candy-Wrapper" Collapse Limit
Under Linear Blend Skinning (LBS), twisting a single bone axially by angle $\Delta\psi$ shrinks the mid-segment cross-sectional radius by:
$$r(\Delta\psi) = r_0 \cos\left(\frac{\Delta\psi}{2}\right)$$
At $\Delta\psi = 180^\circ$, $\cos(90^\circ) = 0$ (the mesh collapses to a zero-width point — the **"candy-wrapper artifact"**).
- **Standard Rule (`LBS_CANDY_WRAPPER_TWIST_EXCEEDED`)**: Any single bone executing axial twist $|\Delta\psi| > 90^\circ$ (where volume shrinks by $>30\%$) must either distribute the twist across 2–3 twist/roll helper bones (`forearm_twist_01`, `forearm_twist_02`) or use Dual Quaternion Skinning (DQS).

---

## 4. Rotation & Keyframe Interpolation Standards

### 4.1 Unit Quaternion Validity
Every quaternion keyframe $q = (x, y, z, w)$ must be unit-length:
$$\|q\| = \sqrt{x^2 + y^2 + z^2 + w^2} = 1.0 \quad (\text{tolerance } \pm 10^{-3})$$

### 4.2 Quaternion Hemisphere Continuity (Shortest-Path SLERP / NLERP)
In $\text{SO}(3)$, $q$ and $-q$ encode the exact same 3D orientation. However, keyframe samplers (`LINEAR` or `CUBICSPLINE` in glTF/USD) that interpolate directly between consecutive keyframes $q_i$ and $q_{i+1}$ where the dot product is negative:
$$q_i \cdot q_{i+1} = x_i x_{i+1} + y_i y_{i+1} + z_i z_{i+1} + w_i w_{i+1} < 0$$
force the bone to spin the **long way around ($360^\circ - \theta$)** over a single interval, causing violent 1-frame joint flips.
- **Standard Rule (`QUATERNION_ANTIPODAL_FLIP`)**: Consecutive keyframes on the same rotation channel must satisfy $q_i \cdot q_{i+1} \ge 0$ (negate $q_{i+1} \leftarrow -q_{i+1}$ during export if $q_i \cdot q_{i+1} < 0$).

### 4.3 Euler Gimbal Lock Avoidance (BVH / FBX / Three.js Euler)
In intrinsic 3-axis Tait-Bryan Euler rotations (e.g., `XYZ`, `ZXY`, `YXZ`), when the middle rotation angle $\theta_{\text{mid}}$ approaches $\pm 90^\circ$:
$$\cos(\theta_{\text{mid}}) \to 0 \quad (|\theta_{\text{mid}}| \in [87^\circ, 93^\circ])$$
the first and third rotation axes align (**Gimbal Lock**), causing instantaneous $180^\circ$ wrist/shoulder/hip flips during interpolation.
- **Standard Rule (`EULER_GIMBAL_LOCK_SINGULARITY`)**: Convert Euler tracks to quaternions or choose a rotation order per joint where the middle axis is the anatomical axis with the smallest Range of Motion (e.g., `ZXY` or `YZX` so abduction/adduction is the middle axis).

### 4.4 Cubic Bezier / Hermite Curve Tangents (Lottie / CSS / Spine / glTF)
For normalized cubic Bezier timing curves `cubic-bezier(x1, y1, x2, y2)`:
- **Time Monotonicity (`BEZIER_TIME_NON_MONOTONIC`)**: The horizontal (time) control coordinates **must** satisfy:
  $$0 \le x_1 \le 1 \quad \text{and} \quad 0 \le x_2 \le 1$$
  If $x_1 < 0$ or $x_2 > 1$, the time curve $t(u)$ folds backward ($\frac{dt}{du} < 0$), creating a multi-valued trajectory in time!

---

## 5. Inverse Kinematics (IK) & Constraint Standards

### 5.1 Two-Bone Analytical IK & Over-Extension ("IK Knee/Elbow Pop")
For a 2-bone limb of upper length $L_1$ and lower length $L_2$ solving for target distance $D = \|\vec{p}_{\text{target}} - \vec{p}_{\text{root}}\|$ via the Law of Cosines:
$$\cos\theta_{\text{interior}} = \frac{L_1^2 + L_2^2 - D^2}{2 L_1 L_2}$$
As the limb approaches full extension ($D \to L_1 + L_2$, $\theta_{\text{interior}} \to 180^\circ$), the Jacobian derivative diverges:
$$\left|\frac{d\theta_{\text{interior}}}{dD}\right| = \frac{D}{L_1 L_2 \sin\theta_{\text{interior}}} \xrightarrow{D \to L_1 + L_2} \infty$$
Even a smooth, constant-velocity foot or hand target produces an **infinite angular velocity spike ("IK pop")** at the knee or elbow when crossing full extension!
- **Standard Rule (`IK_CHAIN_OVEREXTENSION_POP`)**: Maintain soft-IK damping or keep target distance within $D \le 0.992 \cdot (L_1 + L_2)$ (minimum flexion $\ge 6^\circ\text{--}8^\circ$) during locomotion.

### 5.2 Pole Vector Singularity
The unit vector from root to target $\hat{u}_{\text{target}}$ and the unit vector from root to the pole target $\hat{u}_{\text{pole}}$ define the bending plane via $\hat{n} = \hat{u}_{\text{target}} \times \hat{u}_{\text{pole}}$.
- **Standard Rule (`IK_POLE_VECTOR_COLLINEAR`)**: $|\hat{u}_{\text{target}} \cdot \hat{u}_{\text{pole}}|$ must remain $\le 0.98$ ($\ge 11^\circ$ separation), otherwise the joint flips $180^\circ$ across the plane singularity.

---

## 6. Physics Rig & Articulated Dynamics Standards (`URDF`, `SDF`, `MuJoCo`, `UsdPhysics`, `KHR_physics_rigid_bodies`)

When a 3D model includes rigid-body physics properties for ragdolls, robotics, or simulation:
1. **Strictly Positive Mass**: $m > 0$ for all dynamic links.
2. **Symmetric Positive-Definite Inertia Tensor**: Principal moments of inertia $(I_{xx}, I_{yy}, I_{zz})$ about the Center of Mass must satisfy $I_{xx} > 0, I_{yy} > 0, I_{zz} > 0$.
3. **3D Inertia Triangle Inequality (`INERTIA_TRIANGLE_INEQUALITY_VIOLATION`)**:
   Because $I_{xx} = \int (y^2 + z^2)\,dm$, $I_{yy} = \int (x^2 + z^2)\,dm$, $I_{zz} = \int (x^2 + y^2)\,dm$, the sum of any two principal moments of inertia **must** be greater than or equal to the third:
   $$I_{xx} + I_{yy} \ge I_{zz}, \quad I_{yy} + I_{zz} \ge I_{xx}, \quad I_{zz} + I_{xx} \ge I_{yy}$$
   Violating this inequality is mathematically impossible for any real positive-density 3D object and causes physics engines (PhysX, Bullet, Havok, MuJoCo) to suffer numerical instability or explosion.

---

## 7. 2D Mesh Deformation & Traditional Animation Principles

### 7.1 2D Deforming Mesh Winding Preservation (`Spine`, `Live2D`, `Lottie`)
For any 2D triangle $(A, B, C)$ in a deforming 2D mesh, its signed area $2\mathcal{A}(t) = (B_x - A_x)(C_y - A_y) - (B_y - A_y)(C_x - A_x)$ must maintain the **same sign** as the bind pose $\mathcal{A}(0)$:
$$\mathcal{A}(t) \cdot \mathcal{A}(0) > 0$$
If $\mathcal{A}(t) \cdot \mathcal{A}(0) \le 0$, the triangle has inverted/folded over itself (`MESH_2D_TRIANGLE_FLIP`), causing texture tearing and z-order corruption.

### 7.2 Secondary Motion / Follow-Through & Overlapping Action Phase Lag
Passive secondary chains (tails, capes, hair, clothing, ears, antennas) driven by a primary parent bone must obey damped forced-oscillator phase lag:
- **Standard Rule (`SECONDARY_MOTION_PHASE_LEAD_ERROR`)**: Peak displacement/velocity of a passive distal secondary bone must occur **after** ($\Delta t_{\text{lag}} = t_{\text{peak,secondary}} - t_{\text{peak,primary}} > 0$) the parent bone's acceleration/velocity change, never leading the parent bone.
