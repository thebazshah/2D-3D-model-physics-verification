# Classical Mechanics & Animation Physics Reference

This reference defines the mathematical laws, numerical derivatives, and physical tolerance thresholds used to evaluate 2D and 3D model movements.

---

## 1. Discrete Kinematics (Finite Differences)

Given a discrete trajectory of 2D or 3D positions $\vec{p}_i$ at timestamps $t_i$ with timestep $\Delta t_i = t_i - t_{i-1}$:

- **Velocity** ($\text{m/s}$ or $\text{units/s}$):
  $$\vec{v}_i = \frac{\vec{p}_i - \vec{p}_{i-1}}{\Delta t_i}$$
- **Acceleration** ($\text{m/s}^2$):
  $$\vec{a}_i = \frac{\vec{v}_i - \vec{v}_{i-1}}{\frac{1}{2}(\Delta t_i + \Delta t_{i-1})}$$
- **Jerk** ($\text{m/s}^3$):
  $$\vec{j}_i = \frac{\vec{a}_i - \vec{a}_{i-1}}{\Delta t_i}$$
- **Angular Velocity & Acceleration** ($\text{rad/s}$, $\text{rad/s}^2$):
  For quaternions $q_i$, relative rotation $\Delta q_i = q_i \cdot q_{i-1}^{-1}$ yields angle $\Delta\theta_i = 2\arccos(\text{clamp}(|\Delta q_{i,w}|, 0, 1))$ and angular speed $\omega_i = \frac{\Delta\theta_i}{\Delta t_i}$.

### Continuity & Minimum-Jerk Checks
1. **$C^0$ Position Discontinuity (Teleportation)**:
   Flag when $|\vec{v}_i| > v_{\max}$ for a single frame while adjacent frames have normal speeds ($|\vec{v}_i| > 5 \cdot \text{median}(|\vec{v}|)$).
2. **$C^1$ Velocity Discontinuity (Unexplained Impulse)**:
   Instantaneous velocity direction/magnitude flips without an active collision or ground contact event.
3. **Excessive Jerk / Linear Keyframe Snapping**:
   Biological and mechanical systems cannot change acceleration instantaneously due to muscle activation dynamics and actuator inductance.
   - Smooth reaching/locomotion follows a **minimum-jerk trajectory**:
     $$x(t) = x_0 + (x_f - x_0)\left(10\tau^3 - 15\tau^4 + 6\tau^5\right), \quad \tau = \frac{t - t_0}{T}$$
   - Peak velocity of a minimum-jerk point-to-point motion is $v_{\text{peak}} = 1.875 \cdot \frac{\Delta x}{T}$. Constant-velocity linear interpolation ($v(t) = \text{const}$ with infinite jerk at endpoints) looks robotic.

---

## 2. Gravity, Ballistics & Projectile Motion

When a model or rigid body is airborne (no ground contact, no wing/rotor/thruster lift, no buoyancy):

- **Vertical Acceleration**:
  $$a_y(t) = -g + \frac{F_{\text{drag},y}}{m}, \quad g \approx 9.80665\text{ m/s}^2 \text{ (or } 980.665\text{ cm/s}^2\text{)}$$
  *(Note: In 2D screen space where $+Y$ points downward, $a_y = +g_{\text{px}}$ where $g_{\text{px}} = g \cdot \text{pixelsPerMeter}$.)*
- **Horizontal Acceleration (Low Drag)**:
  $$a_x(t) \approx 0, \quad a_z(t) \approx 0$$
- **Apex & Flight Time Relationship**:
  For takeoff and landing at the same elevation ($y_{\text{takeoff}} = y_{\text{land}}$):
  $$h_{\text{apex}} - y_{\text{takeoff}} = \frac{1}{8} g T_{\text{flight}}^2 \implies T_{\text{flight}} = \sqrt{\frac{8 \Delta h}{g}}$$
  **Common Animation Violations**:
  - **Floaty Gravity**: $g_{\text{eff}} = \frac{8 \Delta h}{T_{\text{flight}}^2} < 0.6 g$ (character hangs in the air too long relative to jump height).
  - **Heavy / Crushed Gravity**: $g_{\text{eff}} > 1.8 g$ without downward propulsion.
  - **Mid-Air Course Correction**: Non-zero horizontal acceleration $|\vec{a}_{xz}| > 1.5\text{ m/s}^2$ while airborne without external force or thruster.
  - **Asymmetric Arc Without Drag**: Time to rise $t_{\text{up}}$ differs from time to fall $t_{\text{down}}$ by more than $15\%$ for equal start/end heights.

---

## 3. Center of Mass (CoM), Static & Dynamic Balance

For an articulated figure with $N$ segments of mass fraction $w_k$ ($\sum_{k=1}^N w_k = 1$) and segment centers $\vec{c}_k$:
$$\vec{p}_{\text{CoM}} = \sum_{k=1}^N w_k \vec{c}_k$$

### Static Balance (Stationary Pose: $|\vec{v}_{\text{CoM}}| < 0.05\text{ m/s}$, $|\vec{a}_{\text{CoM}}| < 0.2\text{ m/s}^2$)
- Project $\vec{p}_{\text{CoM}}$ onto the support ground plane to obtain $\vec{p}_{\text{CoM},\perp}$.
- Compute the 2D convex hull of all active contact points (feet, hands, wheels, paws) to form the **Base of Support (BoS)**.
- **Rule**: $\vec{p}_{\text{CoM},\perp}$ must lie inside the BoS (within a margin $\epsilon \approx 0.02\text{ m}$). If outside, an uncompensated gravitational torque $\vec{\tau}_g = (\vec{p}_{\text{CoM}} - \vec{p}_{\text{pivot}}) \times m\vec{g}$ exists and the model must tip over.

### Dynamic Balance (Zero Moment Point & Extrapolated CoM)
When accelerating or turning, inertial forces shift the required support point:
- **Zero Moment Point (ZMP)** on a flat horizontal ground ($y = 0$, $+Y$ up):
  $$x_{\text{ZMP}} = x_{\text{CoM}} - \frac{y_{\text{CoM}}}{a_{y,\text{CoM}} + g} a_{x,\text{CoM}}, \quad z_{\text{ZMP}} = z_{\text{CoM}} - \frac{y_{\text{CoM}}}{a_{y,\text{CoM}} + g} a_{z,\text{CoM}}$$
  For dynamically balanced contact without tipping, $(x_{\text{ZMP}}, z_{\text{ZMP}})$ must remain inside the Base of Support polygon during stance.
- **Extrapolated Center of Mass (XCoM)**:
  $$\vec{\xi} = \vec{p}_{\text{CoM},xz} + \frac{\vec{v}_{\text{CoM},xz}}{\omega_0}, \quad \omega_0 = \sqrt{\frac{g}{l_{\text{CoM}}}}$$
  where $l_{\text{CoM}}$ is the vertical height of the CoM above the support surface. The next foot placement must capture $\vec{\xi}$ to arrest a fall.

---

## 4. Conservation Laws, Collisions & Deformation

### Linear Momentum & Collisions
For two colliding bodies $A$ and $B$ with no external impulse during impact:
$$m_A \vec{v}_{A,1} + m_B \vec{v}_{B,1} = m_A \vec{v}_{A,2} + m_B \vec{v}_{B,2}$$
Along the collision normal $\hat{n}$, the **Coefficient of Restitution ($e$)** is:
$$e = -\frac{(\vec{v}_{B,2} - \vec{v}_{A,2}) \cdot \hat{n}}{(\vec{v}_{B,1} - \vec{v}_{A,1}) \cdot \hat{n}}$$
- **Rule**: $0 \le e \le 1$ for passive materials ($e > 1$ violates kinetic energy conservation unless an active explosion/muscle push occurs).
- Typical values: Human body / flesh $e \in [0.0, 0.15]$, wood $e \in [0.4, 0.6]$, steel $e \in [0.6, 0.85]$, rubber ball $e \in [0.75, 0.9]$.

### Angular Momentum Conservation in Free Flight
When airborne ($\sum \vec{\tau}_{\text{ext}} = \vec{0}$):
$$\vec{L} = \mathbf{I}(t)\vec{\omega}(t) = \text{const}$$
- If a character tucks its limbs inward ($I$ decreases by factor $k$), angular speed $\omega$ must **increase** by approximately factor $k$.
- A character cannot initiate net angular momentum in mid-air from zero spin (though zero-angular-momentum cat-righting twists of counter-rotating body segments can reorient attitude).

### Volume / Area Conservation (Squash and Stretch)
Biological tissues are nearly incompressible (Poisson's ratio $\nu \approx 0.49$).
- **3D Scale Factors** $(s_x, s_y, s_z)$:
  $$V_{\text{ratio}} = s_x \cdot s_y \cdot s_z \approx 1.0 \quad (\text{tolerance } [0.85, 1.15])$$
  If a model stretches vertically by $s_y = 1.4$, its transverse scales must contract by $s_x = s_z = \frac{1}{\sqrt{1.4}} \approx 0.845$.
- **2D Scale Factors** $(s_x, s_y)$:
  $$A_{\text{ratio}} = s_x \cdot s_y \approx 1.0 \quad (\text{tolerance } [0.85, 1.15])$$

---

## 5. Pendulum & Turning Lean Mechanics

### Passive Limb / Tail / Pendulum Swing
A suspended limb or hanging rope/chain of effective length $L_{\text{eff}}$ has natural period:
$$T_{\text{pend}} = 2\pi \sqrt{\frac{L_{\text{eff}}}{g}}$$
For a uniform rod of length $L$ pivoted at one end, $L_{\text{eff}} = \frac{2}{3}L$. Swinging significantly faster than $T_{\text{pend}}$ requires active muscular or motorized torque.

### Centripetal Lean Angle in Turns
Any runner, animal, cyclist, or aircraft executing a sustained turn of radius $R$ at speed $v$ experiences lateral acceleration $a_{\text{lat}} = \frac{v^2}{R}$.
To align the resultant ground reaction force with the body's longitudinal axis (preventing tipping torque), the body must lean **inward** toward the center of curvature by:
$$\theta_{\text{lean}} = \arctan\left(\frac{v^2}{R \cdot g}\right)$$
- **Violation**: Leaning outward on a two-wheeled or bipedal turn, or staying strictly vertical ($\theta_{\text{lean}} \approx 0^\circ$) when $\frac{v^2}{R g} > 0.15$ ($\theta_{\text{expected}} > 8.5^\circ$).
