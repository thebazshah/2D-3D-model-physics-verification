# Animal Locomotion & Biomechanics Reference

Use these biomechanical laws and gait matrices to verify quadrupeds, birds, winged creatures, fish, and reptiles/insects in 2D and 3D animations.

---

## 1. Quadruped Locomotion & Hildebrand Gait Analysis

Every quadruped gait is uniquely defined by two parameters per limb:
1. **Duty Factor ($\beta$)**: Fraction of total stride duration $T_{\text{stride}}$ that a foot is in contact with the ground ($\beta = t_{\text{stance}} / T_{\text{stride}}$).
2. **Relative Phase ($\phi \in [0, 1)$)**: Fraction of the stride cycle at which each foot strikes the ground relative to the **Left Hind (LH)** foot ($\phi_{\text{LH}} = 0.00$).

### Standard Quadruped Gait Phase & Duty Factor Table

| Gait | Symmetry | Duty Factor ($\beta$) | Froude ($Fr = v^2 / gL$) | Footfall Phase $[\phi_{\text{LH}}, \phi_{\text{LF}}, \phi_{\text{RH}}, \phi_{\text{RF}}]$ | Beat Count & Support Pattern |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Lateral-Sequence Walk** (dogs, cats, horses, cattle) | Symmetric | $0.60\text{--}0.78$ | $< 0.5$ | $[0.00, 0.25, 0.50, 0.75]$ ($\pm 0.06$) | 4-beat: $\text{LH} \to \text{LF} \to \text{RH} \to \text{RF}$. Alternates between 3-legged and 2-legged support. |
| **Diagonal-Sequence Walk** (primates, lemurs) | Symmetric | $0.60\text{--}0.75$ | $< 0.5$ | $[0.00, 0.75, 0.50, 0.25]$ ($\pm 0.06$) | 4-beat: $\text{LH} \to \text{RF} \to \text{RH} \to \text{LF}$. |
| **Trot** (horses, dogs, cats) | Symmetric | $0.38\text{--}0.55$ | $0.5\text{--}2.5$ | $[0.00, 0.50, 0.50, 0.00]$ ($\pm 0.05$) | 2-beat diagonal pairs: $(\text{LH}+\text{RF})$ then $(\text{RH}+\text{LF})$, with 2 suspension phases when $\beta < 0.5$. |
| **Pace** (camels, giraffes, standardbred horses) | Symmetric | $0.38\text{--}0.55$ | $0.5\text{--}2.5$ | $[0.00, 0.00, 0.50, 0.50]$ ($\pm 0.05$) | 2-beat ipsilateral pairs: $(\text{LH}+\text{LF})$ then $(\text{RH}+\text{RF})$. High lateral roll stability requirement. |
| **Canter** (Right Lead) | Asymmetric | $0.35\text{--}0.48$ | $1.5\text{--}4.0$ | $[0.00, 0.30, 0.30, 0.65]$ ($\pm 0.08$) | 3-beat: Trailing hind ($\text{LH}$) $\to$ diagonal pair ($\text{RH}+\text{LF}$) $\to$ leading fore ($\text{RF}$) $\to$ gathered suspension. |
| **Transverse Gallop** (horses, ungulates; Right Lead) | Asymmetric | $0.22\text{--}0.38$ | $> 2.5$ | $[0.00, 0.45, 0.15, 0.60]$ ($\pm 0.08$) | 4-beat: $\text{LH} \to \text{RH} \to \text{LF} \to \text{RF} \to$ gathered suspension. |
| **Rotary Gallop** (cheetahs, greyhounds; Right Lead) | Asymmetric | $0.20\text{--}0.35$ | $> 3.0$ | $[0.00, 0.55, 0.15, 0.40]$ ($\pm 0.08$) | 4-beat circular order: $\text{LH} \to \text{RH} \to \text{RF} \to \text{LF}$, with **two** suspension phases (gathered + extended). |

### Quadruped Skeletal & Dynamic Rules
1. **Center of Mass Bias**: Quadruped CoM lies closer to the forelimbs ($55\%\text{--}60\%$ of body weight on forelimbs in horses/dogs) because the head and neck cantilever forward of the shoulders.
2. **Digitigrade vs. Unguligrade Joint Directions**:
   - What appears visually as a backward-bending "knee" on a dog/cat/horse hind leg is the **ankle/hock (tarsus)**; the true **knee (stifle)** bends forward high near the flank.
   - On the foreleg, the mid-leg hinge that bends backward is the **wrist (carpus)**; the **elbow** is high near the chest and bends backward (olecranon points posteriorly).
   - **Violation**: Bending a quadruped's carpal (wrist) joint forward or bending its tarsal (hock) joint backward.
3. **Spinal Flexion-Extension in Galloping**:
   - **Carnivores (Felines, Canines)**: Flexible lumbar spine flexes deeply during gathered suspension (hind paws pass outside and ahead of forepaws) and extends during extended suspension, adding up to $15\%\text{--}20\%$ to stride length.
   - **Heavy Ungulates (Horses, Rhino, Cattle)**: Relatively rigid thoracolumbar spine; stride length comes primarily from hip and scapular rotation rather than large spinal bending.
4. **Head & Neck Bobbing**:
   - In a **walk** or **canter/gallop**, the head and neck nod out of phase with the trunk to conserve mechanical energy and stabilize vision.
   - In a healthy symmetric **trot**, head vertical excursion should be minimal and symmetric between left and right diagonal beats (asymmetric head bob indicates lameness).

---

## 2. Avian, Bat & Insect Flight Mechanics

### Strouhal Number ($St$) for Flapping Propulsion
For any flapping flyer in cruising forward flight at speed $v$ ($\text{m/s}$), wingbeat frequency $f$ ($\text{Hz}$), and wingtip peak-to-peak vertical excursion $A \approx 2 b_{\text{semi}} \sin(\Delta\theta_{\text{stroke}} / 2)$ ($\text{m}$):
$$St = \frac{f \cdot A}{v}$$
- **Universal Biological Cruise Window**:
  $$0.20 \le St \le 0.40 \quad (\text{optimal vortex shedding at } St \approx 0.25\text{--}0.35)$$
- **Violations**:
  - $St < 0.12$ during powered level flight (wings flapping far too slowly for forward speed; looks like gliding with lazy twitching).
  - $St > 0.60$ in fast forward cruise (hyper-frantic flapping decoupled from forward translation; valid only in hovering or near-stall takeoff where $v \to 0$).

### Allometric Wingbeat Frequency Scaling (Pennycuick's Law)
For geometrically similar birds of mass $m$, wingspan $b$, wing area $S$, air density $\rho$:
$$f \propto m^{3/8} g^{1/2} b^{-23/24} S^{-1/3} \rho^{-3/8}$$
Approximate realistic wingbeat frequencies:
- Hummingbird ($4\text{ g}$): $40\text{--}80\text{ Hz}$
- Sparrow / Pigeon ($0.03\text{--}0.3\text{ kg}$): $8\text{--}15\text{ Hz}$
- Crow / Duck ($0.5\text{--}1.2\text{ kg}$): $4\text{--}7\text{ Hz}$
- Eagle / Albatross / Heron ($2\text{--}8\text{ kg}$): $2.0\text{--}3.5\text{ Hz}$

### Upstroke vs. Downstroke Asymmetry & Body Heave
1. **Wing Folding on Upstroke**: During the downstroke, wings are fully extended to maximize surface area $S$ and downward/forward thrust. During the upstroke, the elbow and wrist flex (bringing wingtips closer to the body) to reduce drag and negative lift. Rigid plank wings flapping symmetrically up and down violate aerodynamic lift generation.
2. **Body Heave Phase**: The bird's torso accelerates upward during the **downstroke** (lift peak) and settles downward during the **upstroke**.
   - **Violation**: Torso moving up when wings move up (in-phase oscillation) instead of opposing the wing stroke ($180^\circ$ reaction phase).

---

## 3. Aquatic & Undulatory Locomotion (Fish, Cetaceans, Snakes)

### Swimming Strouhal Number & Propulsive Wave
For undulatory swimmers (fish tail-beat frequency $f$, tail-tip peak-to-peak amplitude $A \approx 0.15\text{--}0.25 L_{\text{body}}$, forward speed $v$):
$$St = \frac{f \cdot A}{v} \in [0.20, 0.40]$$

### Travelling Body Wave vs. Standing Wave
Undulatory propulsion requires a posteriorly propagating wave along the body axis $x \in [0, L]$:
$$y(x, t) = A(x) \sin\left(2\pi \left(\frac{x}{\lambda} - f \cdot t\right)\right)$$
- **Wave Speed ($v_{\text{wave}} = f \cdot \lambda$)**: Must exceed forward swimming speed $v$ so that water is pushed backward:
  $$\text{Slip Ratio } = \frac{v}{v_{\text{wave}}} \in [0.55, 0.85]$$
- **Amplitude Envelope $A(x)$**: Minimal near the head/pectoral girdle and grows quadratically toward the caudal peduncle/tail tip.
- **Violation**: Standing wave oscillation (all spine joints crossing zero at the exact same instant, $v_{\text{wave}} = \infty$) produces zero net forward thrust and looks like a wagging stick.
