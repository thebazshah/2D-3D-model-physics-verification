# Vehicle & Transportation Dynamics Reference

Use these mechanical and aerodynamic formulas to verify 2D and 3D animations of cars, trucks, bicycles, motorcycles, aircraft, drones, boats, and trains.

---

## 1. Wheeled Ground Vehicles (Cars, Trucks, Bicycles, Motorcycles)

### No-Slip Rolling Kinematics
For a wheel of effective rolling radius $R_{\text{wheel}}$ moving forward with longitudinal hub speed $v_x$:
$$\omega_{\text{expected}} = \frac{v_x}{R_{\text{wheel}}} \quad (\text{rad/s}), \quad \Delta\theta_{\text{wheel}} = \frac{\Delta s}{R_{\text{wheel}}}$$
The **Longitudinal Slip Ratio ($\kappa$)** is:
$$\kappa = \frac{\omega \cdot R_{\text{wheel}} - v_x}{\max(|v_x|, |\omega \cdot R_{\text{wheel}}|)}$$
- **Normal Traction**: $|\kappa| \le 0.10$ (wheel rotation matches ground distance within $10\%$).
- **Violation**:
  - $\omega = 0$ while $v_x > 0.5\text{ m/s}$ without tire skid marks/smoke or braking deceleration $a_x \approx -\mu g$.
  - $\omega$ rotating backward ($\omega \cdot v_x < 0$) while moving forward.
  - $|\kappa| > 0.15$ during steady cruising ($a_x \approx 0$).

### Ackermann Steering & Non-Holonomic Turning
A rigid-axle or Ackermann-steered vehicle of wheelbase $L_{\text{wb}}$ (distance between front and rear axles) and track width $W_{\text{track}}$ turning with average front steer angle $\delta$:
- **Instantaneous Turning Radius**:
  $$R_{\text{turn}} = \frac{L_{\text{wb}}}{\tan\delta}$$
- **Yaw Rate ($\dot{\psi}$)**:
  $$\dot{\psi}_{\text{expected}} = \frac{v_x}{R_{\text{turn}}} = \frac{v_x \tan\delta}{L_{\text{wb}}}$$
- **Inner vs. Outer Front Wheel Angles (Ackermann Condition)**:
  $$\cot\delta_{\text{outer}} - \cot\delta_{\text{inner}} = \frac{W_{\text{track}}}{L_{\text{wb}}}$$
  *(The inner wheel must steer at a sharper angle than the outer wheel to prevent tire scrubbing.)*
- **Non-Holonomic Lateral Constraint**:
  Under normal grip ($a_{\text{lat}} = \frac{v_x^2}{R_{\text{turn}}} \le \mu g$, where $\mu \approx 0.7\text{--}1.0$ on dry asphalt, $0.2\text{--}0.4$ on gravel/snow), the rear axle velocity vector must align with the vehicle's longitudinal heading (slip angle $|\beta_{\text{slip}}| < 5^\circ$).
  - **Violation**: Vehicle rotating in place ($\dot{\psi} \neq 0$ while $v_x = 0$), crab-walking sideways without drift physics, or turning with $\dot{\psi} > 0$ while front wheels point straight ($\delta = 0$) or opposite ($\delta < 0$, unless counter-steering in a high-slip drift).

### Longitudinal Load Transfer & Suspension Pitch
When a 4-wheeled vehicle of mass $m$, CoM height $h_{\text{CoM}}$, and wheelbase $L_{\text{wb}}$ accelerates at $a_x$:
$$\Delta F_z = \frac{m \cdot a_x \cdot h_{\text{CoM}}}{L_{\text{wb}}}$$
- **Acceleration ($a_x > 0$)**: Rear normal load increases, front decreases $\implies$ chassis **squats (pitches nose-up)** by $\theta_{\text{pitch}} \propto +a_x$.
- **Braking ($a_x < 0$)**: Front normal load increases, rear decreases $\implies$ chassis **dives (pitches nose-down)** by $\theta_{\text{pitch}} \propto -a_x$.
- **Violation**: Nose pitching down during forward acceleration or pitching up during hard braking.

### Lateral Roll: Four-Wheeled Cars vs. Two-Wheeled Motorcycles/Bicycles
During a turn of radius $R$ at speed $v$ (centripetal acceleration $a_{\text{lat}} = \frac{v^2}{R}$):
1. **Four-Wheeled Vehicles (Cars, Trucks)**:
   - Because $h_{\text{CoM}}$ is above the suspension roll center $h_{\text{RC}}$, centrifugal force creates an **outward** overturning moment.
   - **Rule**: Chassis rolls **outward** (away from the center of the turn) by $\phi_{\text{roll}} \approx 1^\circ\text{--}6^\circ$ per $g$ of lateral acceleration.
   - **Rollover Threshold**: Static Stability Factor $\text{SSF} = \frac{W_{\text{track}}}{2 h_{\text{CoM}}}$. If $\frac{v^2}{R \cdot g} > \text{SSF}$, the inside wheels lift off and the vehicle rolls over.
2. **Two-Wheeled Vehicles (Bicycles, Motorcycles)**:
   - Have zero lateral base of support width ($W_{\text{track}} = 0$). To balance gravitational torque against centrifugal torque, the vehicle + rider CoM **must lean inward** toward the turn center:
     $$\phi_{\text{lean}} = \arctan\left(\frac{v^2}{R \cdot g}\right)$$
   - **Violation**: Motorcycle/bicycle remaining upright ($\phi_{\text{lean}} \approx 0^\circ$) or leaning outward during a turn at speed.

---

## 2. Aircraft & Rotorcraft (Planes, Helicopters, Quadcopters/Drones)

### Fixed-Wing Aircraft Coordinated Turn
To generate centripetal force $m\frac{v^2}{R}$ without side-slipping (skidding or slipping), a fixed-wing aircraft banks inward at angle $\phi_{\text{bank}}$:
$$\phi_{\text{bank}} = \arctan\left(\frac{v^2}{R \cdot g}\right), \quad \dot{\psi} = \frac{g \tan\phi_{\text{bank}}}{v}$$
- **Load Factor ($G$-load)**:
  $$n = \frac{L_{\text{lift}}}{m g} = \frac{1}{\cos\phi_{\text{bank}}}$$
  At $\phi_{\text{bank}} = 60^\circ$, $n = 2g$. Standard airliners stay below $\phi_{\text{bank}} \le 30^\circ$; fighters/aerobatic planes reach $75^\circ\text{--}85^\circ$ ($4g\text{--}9g$).
- **Minimum Stall Speed**:
  $$v_{\text{stall}}(\phi) = v_{\text{stall},0}\sqrt{n} = \sqrt{\frac{2 m g n}{\rho S C_{L,\max}}}$$
  - **Violation**: Fixed-wing plane hovering stationary ($v = 0$) without VTOL thrusters, or making flat yaw-only turns ($\dot{\psi} > 0, \phi_{\text{bank}} = 0$) at high speed like a turret.

### Multi-Rotor / Quadcopter & Helicopter Attitude-Acceleration Coupling
A standard underactuated quadcopter or helicopter directs its net thrust vector $\vec{T}$ perpendicular to the rotor disc plane.
For horizontal acceleration $\vec{a}_{xz} = (a_x, a_z)$ in world coordinates (with $+Y$ up and aerodynamic drag $\vec{F}_{\text{drag}}$):
$$\vec{T} = m(\vec{a} - \vec{g}) - \vec{F}_{\text{drag}}$$
Therefore, the rotor disc tilt angles (pitch $\theta$ in forward direction, roll $\phi$ in lateral direction) **must** satisfy:
$$\tan\theta_{\text{pitch}} = -\frac{a_{\text{forward}} + F_{\text{drag,fwd}}/m}{a_y + g}, \quad \tan\phi_{\text{roll}} = \frac{a_{\text{right}} + F_{\text{drag,right}}/m}{a_y + g}$$
- **Accelerating Forward ($a_{\text{forward}} > 0$)**: Drone/helicopter **must pitch nose-down** ($\theta_{\text{pitch}} < 0$).
- **Steady Fast Forward Cruise ($a_{\text{forward}} = 0, v_{\text{forward}} \gg 0$)**: Must maintain slight nose-down pitch to overcome parasitic drag $F_{\text{drag}} = \frac{1}{2}\rho v^2 C_d A$.
- **Braking / Stopping from Forward Flight ($a_{\text{forward}} < 0$)**: Drone/helicopter **must flare (pitch nose-up, $\theta_{\text{pitch}} > 0$)** to arrest forward velocity before leveling out into a hover.
- **Violations**:
  - Drone translating horizontally back and forth while its chassis stays completely flat ($\theta = 0^\circ, \phi = 0^\circ$).
  - Drone stopping instantaneously from high speed without a nose-up braking flare.

---

## 3. Watercraft & Boats

- **Archimedes Buoyancy & Heave Equilibrium**: Displaced water volume $V_{\text{disp}} = \frac{m}{\rho_{\text{water}}}$ ($\rho_{\text{water}} \approx 1000\text{--}1025\text{ kg/m}^3$).
- **Bow Rise Under Thrust**: Because the propeller thrust line lies below the hull's CoM and hydrodynamic drag center, forward acceleration pitches the **bow up** (especially transitioning from displacement to planing at hull Froude number $Fr_{\text{hull}} = \frac{v}{\sqrt{g L_{\text{WL}}}} > 0.4$).
- **Turn Banking**:
  - **V-Hull Planing Speedboats**: Bank **inward** into the turn like an aircraft due to hydrodynamic lift on the chine.
  - **Deep-Keel Sailboats / Tall Heavy Displacement Ships**: Roll **outward** in a turn (and heel leeward under wind force).
