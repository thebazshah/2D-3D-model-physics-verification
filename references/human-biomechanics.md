# Human Biomechanics & Locomotion Reference

Use these biomechanical constraints, anthropometric tables, and kinematic laws to verify 2D and 3D human character animations, procedural rigs, and ragdolls.

---

## 1. Anthropometric Segment Mass & Length Proportions (Winter's Standard)

For a total body mass $M$ and total standing height $H$, standard segment mass fractions $w_k = m_k / M$ and proximal-to-CoM ratios are:

| Body Segment | Mass Fraction ($w_k$) | Segment Length / $H$ | CoM from Proximal Joint |
| :--- | :--- | :--- | :--- |
| Head & Neck | $0.081$ | $0.130$ | $0.500$ (from C7/T1) |
| Trunk (Thorax + Abdomen + Pelvis) | $0.497$ | $0.288$ | $0.500$ (hip to shoulder) |
| Upper Arm (each) | $0.028$ | $0.186$ | $0.436$ (from shoulder) |
| Forearm (each) | $0.016$ | $0.146$ | $0.430$ (from elbow) |
| Hand (each) | $0.006$ | $0.108$ | $0.506$ (from wrist) |
| Thigh (each) | $0.100$ | $0.245$ | $0.433$ (from hip) |
| Shank / Lower Leg (each) | $0.0465$ | $0.246$ | $0.433$ (from knee) |
| Foot (each) | $0.0145$ | $0.152$ (length) | $0.500$ (from ankle) |

- **Standing Whole-Body CoM**: Located at approximately $0.55 \times H$ above the ground (just anterior to the second sacral vertebra S2).
- **Leg Length ($L_{\text{leg}}$)**: Greater trochanter (hip) to ground $\approx 0.53 \times H$.

---

## 2. Anatomical Joint Range of Motion (RoM) & Degrees of Freedom (DoF)

Angles are measured relative to the neutral anatomical posture ($0^\circ$). Violations beyond physiological maximums indicate broken IK solvers, gimbal lock flips, or unphysical rigs.

| Joint | Axis / Movement | Normal Range (deg) | Hard Anatomical Limit (deg) | Common Rig / Animation Errors |
| :--- | :--- | :--- | :--- | :--- |
| **Cervical Spine (Neck)** | Flexion / Extension | $-50^\circ \text{ to } +60^\circ$ | $-65^\circ \text{ to } +75^\circ$ | Head spinning $>90^\circ$ yaw |
| | Lateral Bending | $-45^\circ \text{ to } +45^\circ$ | $-55^\circ \text{ to } +55^\circ$ | |
| | Axial Rotation (Yaw) | $-80^\circ \text{ to } +80^\circ$ | $-90^\circ \text{ to } +90^\circ$ | |
| **Thoraco-Lumbar Spine** | Flexion / Extension | $-30^\circ \text{ to } +80^\circ$ | $-40^\circ \text{ to } +95^\circ$ | Single-bone spine bending $>60^\circ$ at one vertebra |
| | Lateral Bending | $-35^\circ \text{ to } +35^\circ$ | $-45^\circ \text{ to } +45^\circ$ | |
| | Axial Rotation (Yaw) | $-45^\circ \text{ to } +45^\circ$ | $-55^\circ \text{ to } +55^\circ$ | |
| **Shoulder (Glenohumeral + Scapular)** | Flexion / Extension | $-60^\circ \text{ to } +180^\circ$ | $-70^\circ \text{ to } +190^\circ$ | Raising arm $>90^\circ$ without clavicle/scapula elevation ($1:2$ scapulohumeral rhythm) |
| | Abduction / Adduction | $-45^\circ \text{ to } +180^\circ$ | $-55^\circ \text{ to } +185^\circ$ | |
| | Internal / External Rot. | $-70^\circ \text{ to } +90^\circ$ | $-85^\circ \text{ to } +100^\circ$ | |
| **Elbow (Hinge)** | Flexion / Extension | $0^\circ \text{ to } +145^\circ$ | $-5^\circ \text{ to } +155^\circ$ | **Hyperextension** ($< -5^\circ$) or lateral varus/valgus bend ($> 5^\circ$) |
| **Forearm (Radioulnar)** | Pronation / Supination | $-85^\circ \text{ to } +85^\circ$ | $-95^\circ \text{ to } +95^\circ$ | Twisting wrist bone instead of distributing twist along forearm |
| **Wrist** | Flexion / Extension | $-70^\circ \text{ to } +80^\circ$ | $-85^\circ \text{ to } +90^\circ$ | |
| | Radial / Ulnar Dev. | $-20^\circ \text{ to } +35^\circ$ | $-25^\circ \text{ to } +45^\circ$ | |
| **Hip (Ball & Socket)** | Flexion / Extension | $-30^\circ \text{ to } +120^\circ$ | $-40^\circ \text{ to } +135^\circ$ | Extending thigh backward $>30^\circ$ without anterior pelvic tilt |
| | Abduction / Adduction | $-30^\circ \text{ to } +45^\circ$ | $-35^\circ \text{ to } +60^\circ$ | |
| | Internal / External Rot. | $-40^\circ \text{ to } +45^\circ$ | $-50^\circ \text{ to } +55^\circ$ | |
| **Knee (Modified Hinge)** | Flexion / Extension | $0^\circ \text{ to } +140^\circ$ | $-5^\circ \text{ to } +155^\circ$ | **Knee pop / hyperextension** ($< -5^\circ$) or sideways bend ($> 5^\circ$) |
| **Ankle (Talocrural)** | Dorsiflexion / Plantarflexion | $-50^\circ \text{ to } +20^\circ$ | $-60^\circ \text{ to } +30^\circ$ | Excessive dorsiflexion ($> 30^\circ$) without heel lift |

---

## 3. Bipedal Gait Laws (Walking vs. Running)

### Froude Number & Gait Selection
The dimensionless **Alexander Froude Number** governs dynamic similarity in legged locomotion:
$$Fr = \frac{v^2}{g \cdot L_{\text{leg}}}$$
- **Walking**: $0 < Fr < 0.5$ (for an adult human with $L_{\text{leg}} = 0.9\text{ m}$, walk speed $v < 2.1\text{ m/s}$).
- **Walk-to-Run Transition**: Occurs naturally around $Fr \approx 0.5$ ($v \approx 2.0\text{--}2.2\text{ m/s}$). Walking at $Fr > 0.7$ requires downward centripetal acceleration exceeding gravity ($v^2 / L_{\text{leg}} > g$), making an inverted-pendulum vault physically impossible without lifting off the ground.
- **Running / Sprinting**: $Fr \ge 0.5$ (up to $Fr \approx 1.3$ for endurance running, and $Fr \approx 10\text{--}12$ at elite sprint speed $v \approx 10\text{ m/s}$).

### Duty Factor ($\beta$) & Support Phases
Let $T_{\text{stride}}$ be the full cycle period of one leg (heel-strike to ipsilateral heel-strike) and $t_{\text{stance}}$ be the duration that foot is in contact with the ground:
$$\beta = \frac{t_{\text{stance}}}{T_{\text{stride}}}$$
- **Walk ($\beta > 0.5$, typically $0.55\text{--}0.65$)**:
  - Must have **two double-support intervals** per stride cycle (each lasting $(2\beta - 1)\frac{T_{\text{stride}}}{2} \approx 10\%\text{--}15\%$ of the cycle).
  - Never has a flight phase (at least one foot is always grounded).
- **Run ($\beta < 0.5$, typically $0.25\text{--}0.45$)**:
  - Must have **two aerial/flight phases** per stride cycle (neither foot grounded).
  - Never has double support.

### Vertical Pelvis / CoM Oscillation Phase
- **Walking (Inverted Pendulum Vault)**:
  - Pelvis/CoM reaches its **highest vertical point at mid-stance** (when the supporting leg passes vertically under the hip) and its **lowest point during double support** (heel strike / toe-off).
  - Peak-to-peak vertical displacement $\Delta y_{\text{CoM}} \approx 0.03\text{--}0.06\text{ m}$.
  - *Common Error*: Flat horizontal pelvis translation ("groucho walk" to avoid IK knee pop) or inverted phase.
- **Running (Spring-Loaded Inverted Pendulum - SLIP)**:
  - Pelvis/CoM reaches its **lowest vertical point at mid-stance** (knee flexed $\sim 40^\circ$ absorbing impact) and its **highest point during the aerial flight phase**.
  - Phase of vertical CoM vs. leg stance in running is **$180^\circ$ out of phase** with walking.

### Contralateral Phase & Pelvic Rotations
- **Left-Right Leg Phase Offset**: Symmetric gaits require $\Delta\phi_{\text{legs}} = 0.50 \pm 0.05$ ($180^\circ$ out of phase).
- **Contralateral Arm Swing**: Left arm swings forward with right leg ($\Delta\phi_{\text{arm,ipsi-leg}} \approx 0.50$). Arm swing counters the vertical angular momentum generated by the swinging legs.
- **Pelvic List (Coronal Tilt)**: During single-leg stance in walking, the pelvis drops slightly ($4^\circ\text{--}5^\circ$) on the **swing-leg side** (Trendelenburg drop controlled by hip abductors).
- **Pelvic Transverse Rotation**: Pelvis rotates forward ($4^\circ\text{--}8^\circ$ yaw) on the advancing swing-leg side, while the thorax counter-rotates in opposite yaw.

### Foot Contact & Footskate (Sliding) Check
During the stance window $[t_{\text{contact\_start}}, t_{\text{contact\_end}}]$ when $y_{\text{foot}} \le y_{\text{ground}} + \epsilon_y$:
1. **No Ground Penetration**: $y_{\text{foot}} \ge y_{\text{ground}} - 0.005\text{ m}$.
2. **Zero Horizontal Slip**: $|\vec{v}_{\text{foot},xz}| \le 0.05 \cdot v_{\text{body}}$ (ankle/heel stays locked to ground during heel-rocker, ball of foot stays locked during forefoot rocker).
3. **Stride Length Consistency**: Distance traveled by the root/pelvis over one full stride $T_{\text{stride}}$ must equal the spatial step-to-step placement distance $L_{\text{stride}} = v_{\text{avg}} \cdot T_{\text{stride}}$. Mismatch causes treadmill "moonwalking" or footskate.

---

## 4. Jumping, Landing & Impulse Verification

### Anticipation Crouch & Takeoff Impulse
To achieve a vertical jump height $\Delta h = y_{\text{apex}} - y_{\text{takeoff}}$, the required vertical takeoff speed is:
$$v_{\text{takeoff}} = \sqrt{2 g \Delta h}$$
If the character squats downward by crouch depth $\Delta y_{\text{crouch}}$ and pushes off over time $\Delta t_{\text{push}}$:
- **Work-Energy Equivalence**:
  $$\bar{a}_{\text{push}} = g \frac{\Delta h}{\Delta y_{\text{crouch}}}, \quad \Delta t_{\text{push}} \approx \frac{2 \Delta y_{\text{crouch}}}{v_{\text{takeoff}}}$$
- **Human Physiological Limit**: Maximum human concentric leg extension acceleration is $\bar{a}_{\text{push}} \le 2.5 g\text{--}3.5 g$ for normal humans (up to $4g$ for explosive athletes).
- **Violations**:
  - **Zero-Anticipation Jump**: Jumping $\Delta h > 0.15\text{ m}$ with $\Delta y_{\text{crouch}} < 0.05\text{ m}$ requires unphysical infinite force.
  - **Paused Crouch Mismatch**: Holding the bottom of a crouch for $>0.5\text{ s}$ dissipates elastic stretch-shortening energy, requiring visible upward acceleration ramp during push-off.

### Landing Impact & Recovery
Touching down from fall height $\Delta h_{\text{fall}} = y_{\text{apex}} - y_{\text{land}}$ produces impact speed $v_{\text{impact}} = \sqrt{2 g \Delta h_{\text{fall}}}$.
- The CoM must decelerate over a flexion distance $\Delta y_{\text{absorb}}$ over duration $\Delta t_{\text{absorb}} \approx \frac{2 \Delta y_{\text{absorb}}}{v_{\text{impact}}}$.
- **Rule**: If $\Delta y_{\text{absorb}} < 0.05\text{ m}$ for $\Delta h_{\text{fall}} > 0.25\text{ m}$, deceleration exceeds $5g$ ("stiff-legged landing"), which looks robotic and injurious.

---

## 5. Proximal-to-Distal Kinematic Chain (Throwing, Striking, Kicking)

High-velocity open-chain movements (throwing a ball, swinging a bat/sword, kicking) transfer momentum from high-mass proximal segments to low-mass distal segments:
- **Peak Angular Velocity Sequence**:
  $$t_{\text{peak}}(\text{Pelvis}) < t_{\text{peak}}(\text{Thorax}) < t_{\text{peak}}(\text{Shoulder}) < t_{\text{peak}}(\text{Elbow}) < t_{\text{peak}}(\text{Wrist/Hand})$$
- Typical inter-segment delay is $15\text{--}45\text{ ms}$ per link.
- **Violation**: Simultaneous peak velocity across all joints ("single-hinge arm throw") or distal-to-proximal ordering during a power strike.
