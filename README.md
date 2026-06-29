# AeroEval: Multi-Domain Physics-Based MDO Sizing Engine for Multirotor UAVs

[![C++17](https://img.shields.io/badge/Language-C%2B%2B17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Next.js](https://img.shields.io/badge/Frontend-Next.js-black.svg)](https://nextjs.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Developer: Akshay Gupta Burela (GitHub: [AKSHAY-RSOL](https://github.com/AKSHAY-RSOL))

AeroEval is a physics-based, Multidisciplinary Design Optimization (MDO) engine designed to solve the tightly coupled sizing equations of multirotor Unmanned Aerial Vehicles (UAVs). It resolves the mutual couplings between structural mass, battery electrochemistry, rotor aerodynamics, and flight kinematics within a unified fixed-point solver, avoiding the sequential "Mass Snowball" divergence.

AeroEval maps continuous engineering optimization coordinates directly onto commercial off-the-shelf (COTS) components (propellers, motors, and batteries) by matching computed physical requirements against runtime-editable hardware databases.

---

## Key Features and MDO Philosophy

* **Conservative Lower-Bound Design**: Sized around standard Glauert momentum theory, explicit safety factors, and COTS components. If the engine converges, the design is guaranteed to fly.
* **Banach Fixed-Point Mass Solver**: Resolves the circular coupling of structural weight, battery mass, and power requirements using a contraction mapping iteration loop.
* **COTS Component Matchmaking**: Automatically resolves continuous optima into discrete COTS selections using a guided exhaustive search with multi-objective J-score optimization.
* **Declarative Hardware Locks**: Force specific batteries, propellers, or motors; the engine dynamically sizes all remaining degrees of freedom around the constraint.
* **Dual Interface Workspaces**:
  * **Mission Workspace (formerly KMODE)**: A requirements-first taxonomy-driven interface for non-expert users (Imaging, Package Delivery, Agricultural Spray, Search & Rescue, Inspection, and FPV Racing).
  * **Physics Workspace**: A high-fidelity, 57-parameter input editor for expert aerospace designers.
* **Path-Dependent Mass-Shedding**: Supports discrete cargo drop (Delivery) and continuous spraying discharge (Agriculture), reducing structural weight by up to 40.6%.
* **Dual-Run Comparative Diagnostics**: Each invocation executes a null-role baseline (dead-weight hover) and a custom role-specific run side-by-side to isolate payload-specific overhead.

---

## Repository Directory Structure

The repository files and folders are organized as follows:

```
AeroEval/
├── CMakeLists.txt              # CMake build configuration
├── LICENSE                     # MIT License
├── README.md                   # Setup and physics engine documentation
├── .gitignore                  # Git ignore file excluding builds and dependencies
├── data/                       # Configuration, chemistries, and COTS databases
│   ├── chemistry_profiles.json # Tremblay-Dessaint battery parameters
│   ├── cots_database.json      # COTS motor and propeller specifications
│   ├── esc_profiles.json       # ESC current and voltage limits
│   └── user_input_example.json # Input template for sizing runs
├── include/                    # C++ Header files
│   ├── constants/
│   │   └── PhysicalConstants.hpp
│   ├── module1_feasibility/
│   │   └── FeasibilityGate.hpp
│   ├── module2_kinematics/
│   │   └── MissionProfiler.hpp
│   ├── module3_physics/
│   │   ├── Aerodynamics.hpp
│   │   ├── MassIteration.hpp
│   │   ├── Structures.hpp
│   │   └── Thermodynamics.hpp
│   ├── module4_optimizer/
│   │   └── DiscreteMINLP.hpp
│   ├── module5_validation/
│   │   └── SensitivityAnalysis.hpp
│   ├── roles/
│   │   ├── IPayloadRole.hpp
│   │   └── TaxonomyRoles.hpp
│   └── utils/
│       └── JSONHelper.hpp
├── src/                        # C++ Source code files
│   ├── main.cpp                # entry point, JSON configuration parsing
│   ├── module1_feasibility/
│   │   └── FeasibilityGate.cpp # pre-solve geometry and disk loading guards
│   ├── module2_kinematics/
│   │   └── MissionProfiler.cpp # mission kinematics calculations
│   ├── module3_physics/
│   │   ├── Aerodynamics.cpp    # Glauert induced velocity Newton-Raphson
│   │   ├── MassIteration.cpp   # Banach fixed-point loop iterations
│   │   ├── Structures.cpp      # Euler-Bernoulli arms and resonance checks
│   │   └── Thermodynamics.cpp  # Tremblay-Dessaint OCV thermal simulation
│   ├── module4_optimizer/
│   │   └── DiscreteMINLP.cpp   # J-score guided grid search matching
│   └── utils/
│       └── JSONHelper.cpp      # JSON helper methods
├── tests/                      # Testing and sweep scripts
│   ├── parameter_sweeps.py     # automated validation sensitivity sweeps
│   └── realistic_test.py       # realistic configuration testing suite
└── web_ui/                     # Next.js web dashboard frontend
```

---

## Installation and Building

### Downloading the Repository

Clone the repository to your local machine:
```bash
git clone https://github.com/AKSHAY-RSOL/AeroEval.git
cd AeroEval
```

### Prerequisites
* **C++ Compiler**: A compiler supporting C++17 (e.g., GCC 9+, Clang 10+, or MSVC 2019+).
* **CMake**: Version 3.15 or higher.
* **Node.js**: Version 18.x or higher (for the Web UI).

### Building the C++ Physics Engine
From the repository root directory, run:
```bash
# Configure CMake build system
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Compile the target executable
cmake --build build --config Release
```
The compiled binary will be placed at `build/DroneEngine` (or `build/Release/DroneEngine.exe` on Windows).

### Setting Up the Next.js Frontend
```bash
# Navigate to the frontend directory
cd web_ui

# Install dependencies
npm install

# Run the development server
npm run dev
```
Open `http://localhost:3000` in your web browser to access the dashboard workspace.

---

## Usage and API JSON Contract

### Interactive Web UI (Recommended)

While the C++ physics engine functions as a standalone CLI solver, the primary and recommended way to use AeroEval is through the interactive Next.js Web UI dashboard (located in the `web_ui/` directory). The Web UI abstracts the low-level JSON configuration into a user-friendly, high-fidelity engineering workbench.

#### Web UI Key Features:
* **Dual Design Workspaces**:
  * **Mission Workspace**: A requirements-first, taxonomy-driven console for non-experts. Specify general requirements for specific UAV roles (Imaging, Package Delivery, Agricultural Spray, Search & Rescue, Inspection, and FPV Racing).
  * **Physics Workspace**: A full-fidelity editor with 47 adjustable parameters for expert aerospace designers.
* **Declarative Component Locking**: Toggles to lock down specific hardware components (batteries, motors, or propellers) from the database. The solver dynamically calculates and sizes all remaining degrees of freedom around these locked parameters.
* **Side-by-Side Sizing Comparative Diagnostics**: Clear visual side-by-side comparisons of the baseline (null dead-weight hover) and custom role-specific payload sizing to isolate payload overhead.
* **Dynamic Validation Charts**: Renders interactive plots to compare your design's physical properties against AeroEval's 20-platform empirical verification matrix.
* **Automated Solver Integration**: Generates the underlying input JSON configurations, invokes the compiled C++ binary, and parses the structured output JSON into visual performance charts and metrics in real-time.

---

### Command Line Interface (CLI)

For headless execution or automated sweep script integration:

```bash
# Run simulation with default config
./build/DroneEngine
```

### Abbreviated Input JSON Contract (`data/user_input.json`)
```json
{
  "payload_kg": 2.5,
  "max_diameter_m": 1.2,
  "v_forward_ms": 15.0,
  "v_climb_ms": 3.0,
  "t_climb_s": 60.0,
  "t_forward_s": 1200.0,
  "t_hover_s": 300.0,
  "altitude_m": 100.0,
  "ambient_temp_c": 25.0,
  "battery_chemistry": "LiPo",
  "locks": {
    "is_battery_locked": false,
    "locked_capacity_mah": -1,
    "locked_cell_count": -1,
    "is_prop_locked": false,
    "locked_diameter_in": -1,
    "locked_pitch_in": -1,
    "is_motor_locked": false,
    "locked_kv": -1,
    "auto_reduce_flight_time": true
  },
  "payload_role": {
    "type": "agriculture",
    "aux_power_w": 10.0,
    "added_drag_m2": 0.05,
    "cg_shift_m": 0.0,
    "thrust_margin_override": -1.0
  }
}
```

---

## Mathematical Formulations

AeroEval implements a mathematically rigorous sizing formulation based on aerospace engineering principles, structural dynamics, and electrochemistry.

### 1. Banach Fixed-Point Contraction Mapping
Sizing is formulated as searching for the fixed-point $M^*$ on the complete metric space $(X, d)$ with $d(x_1, x_2) = |x_1 - x_2|$:


$$
M^{(k+1)} = f(M^{(k)}) = M_{\text{payload}} + M_{\text{battery}}(M^{(k)}) + M_{\text{frame}}(M^{(k)}) + M_{\text{prop/motor}}(M^{(k)})
$$



Convergence is certificate-guaranteed when the mass-growth Lipschitz constant satisfies:


$$
L \approx \frac{\partial f(M)}{\partial M} = \frac{\partial M_{\text{bat}}}{\partial M} + \frac{\partial M_{\text{frame}}}{\partial M} + \frac{\partial M_{\text{prop/mot}}}{\partial M} < 1
$$



To damp numerical limit-cycle oscillations induced by discrete database lookups and cell integer rounding, dynamic under-relaxation is applied to the battery capacity target ($\alpha = 0.5$):


$$
C^{(k+1)} = \alpha \cdot C^{(k)}_{\text{target}} + (1-\alpha) \cdot C^{(k)}
$$



A ratchet floor prevents capacity regression:


$$
C^{(k+1)} \geq C_{\text{max-failed}} \times 1.05
$$



#### Altitude-Corrected Initial Mass Seed
At high altitudes, lower air density increases hover power. Seeding the solver with a sea-level estimate can result in local attractor convergence. AeroEval scales the initial seed by local density ratio:


$$
M^{(0)} = \frac{M_{\text{payload}}}{0.60} \cdot \sqrt{\frac{\rho_{\text{SL}}}{\rho(h)}}, \quad \rho_{\text{SL}} = 1.225 \text{ kg/m}^3
$$



---

### 2. Aerodynamic and Kinematic Models
Hub-to-hub distance between adjacent rotors for a diagonal wheelbase $W$ and rotor count $N_r$:


$$
d_{\text{adj}} = W \sin\left(\frac{\pi}{N_r}\right)
$$



Imposing tip clearance safety factor $\varepsilon = 0.05$ prevents propeller overlap:


$$
D_p \leq (1-\varepsilon)\,W\sin\left(\frac{\pi}{N_r}\right)
$$



#### Actuator Disk Induced Velocity
Rotor induced velocity $v_i$ is solved using a Newton-Raphson numerical scheme over Glauert's relation:


$$
v_i = \frac{v_{i,0}^2}{\sqrt{(v_{\infty} \cos \alpha)^2 + (v_{\infty} \sin \alpha + v_i)^2}}, \quad v_{i,0} = \sqrt{\frac{T}{2\rho A_s}}
$$



The total downwash velocity $v_{\text{dw}}$ is constrained to protect crops (Agricultural role):


$$
v_{\text{dw}} = 2v_i = 2\sqrt{\frac{M_{\text{to}}\,g}{2\rho N_r A_s}} \leq 8.0 \text{ m/s}
$$



#### Profile Power & Reynolds Scaling
Profile power $P_o$ incorporates Reynolds number scaling:


$$
P_o = \frac{1}{8}\,\sigma\,C_{d,0}\,\rho\,A_s\,V_{\text{tip}}^3, \quad \sigma = \frac{0.28}{\pi}
$$




$$
C_{d,0} = \begin{cases} 0.012\left(\frac{10^5}{Re_{\text{tip}}}\right)^{0.5} & Re_{\text{tip}} < 10^5 \\ 0.012\left(\frac{10^5}{Re_{\text{tip}}}\right)^{0.2} & Re_{\text{tip}} \geq 10^5 \end{cases}
$$



#### Installed Figure of Merit (FoM)
Theoretical FoM is corrected for installed structural blockages ($\kappa$) and tip Reynolds constraints:


$$
\text{FoM}_{\text{installed}} = \frac{\text{FoM}_{\text{isolated}}}{\kappa}, \quad \kappa \in \{1.20_{\text{consumer}}, 1.25_{\text{enterprise}}, 1.30_{\text{MicroAIO}}, 1.46_{\text{industrial}}\}
$$




$$
\text{FoM}(Re) = \text{FoM}_{\text{max}}\left(1 - e^{-Re/20000}\right)
$$



#### Rotor-Arm Aerodynamic Interference
Rotor thrust is adjusted based on arm clearance type and structure width $w_a$:


$$
T_{\text{corr}} = T\cdot k_{\text{int}}, \quad k_{\text{int}} = \begin{cases} 1.0 + 0.6\,w_a/D_p & \text{offset} \\ 1.0 & \text{folding} \\ 1.0 + 1.5\,w_a/D_p & \text{under-rotor} \end{cases}
$$



---

### 3. Structural Sizing and Resonance
Arms are sized as hollow circular CFRP cantilever tubes under bending moment $M_b$:


$$
\alpha_w = \frac{d_{\text{in}}}{d_{\text{out}}} = 0.90
$$




$$
M_b = F_{\text{arm}}\,L_{\text{arm}}, \quad F_{\text{arm}} = T_{\text{motor}} \cdot \text{load-factor}, \quad L_{\text{arm}} = W/2
$$



Thrust per motor is corrected for asymmetric payload CG offsets ($x_{\text{cg}}$):


$$
T_{\text{motor}} = \frac{M_{\text{to}}\,g}{N_r}\left(1 + \frac{x_{\text{cg}}}{L_{\text{arm}}\cos(\pi/N_r)}\right)
$$



Solving for outer diameter $d_{\text{out}}$ under fatigue-adjusted allowable stress:


$$
\sigma_{\text{all}} = 160 \text{ MPa}
$$




$$
d_{\text{out}} = \sqrt[3]{\frac{32\,M_b}{\pi(1-\alpha_w^4)\sigma_{\text{all}}}}
$$



#### Cantilever Frequency Verification
Propeller blade pass frequency (BPF) must not excite structural arm resonances:


$$
f_{\text{nat}} = \frac{1}{2\pi}\sqrt{\frac{3E_{\text{CF}}I_x}{L_{\text{arm}}^3(M_{\text{tip}} + 0.24\mu L_{\text{arm}})}} \geq 1.5\,f_{\text{bpf}} = 1.5\,N_b\,\text{RPS}_{\text{peak}}
$$


where:
* $E_{\text{CF}} = 70 \text{ GPa}$ is the Young's modulus of carbon fiber.
* $\mu = \rho_{\text{CF}} A_{\text{wall}}$ is the mass per unit length of the arm.
* $N_b = 2$ is the number of blades per propeller.

---

### 4. Electrochemical and Battery Dynamics
The terminal voltage is calculated using a modified Tremblay-Dessaint formulation with C-rate scaling:


$$
V_{\text{term}} = V_{\text{oc}}(\text{SOC}) - I\,R_{\text{sys}}
$$




$$
V_{\text{oc}} = V_{\text{exp}} + (V_{\text{full}}-V_{\text{exp}})e^{-B_{\text{eff}}(1-\text{SOC})} - K\left(\frac{1}{\text{SOC}}-1\right)
$$




$$
B_{\text{eff}} = B(1 + 0.18\ln C_{\text{rate}}) \quad \text{for } C_{\text{rate}} > 1
$$



#### Dynamic Internal Resistance
System resistance $R_{\text{sys}}$ includes cell internal resistance scaling (temperature/aging) and ESC resistance:


$$
R_{\text{sys}} = 10^{-3}\,R_{i,\text{base}}\cdot S \cdot \frac{C_{\text{ref}}}{C_{\text{pack}}}\cdot f_{\text{temp}}\cdot f_{\text{aging}} + \frac{R_{\text{ds,on}}}{N_r}
$$




$$
f_{\text{temp}} = e^{-0.013(T_b-25)}, \quad f_{\text{aging}} = 1 + k_{\text{ag}}\,N_{\text{cyc}}
$$



#### Transient Thermal Cooling
Battery convective heat dissipation incorporates capacity-proportional packaging scaling:


$$
C_{\text{th,b}}\frac{dT_b}{dt} = I^2 R_{\text{sys}} - hA_c(T_b - T_a), \quad C_{\text{th,b}} = M_{\text{bat}} C_p, \quad C_p = 795 \text{ J/(kg K)}
$$




$$
hA_c = h \cdot 6\left(\frac{M_{\text{bat}}}{2400}\right)^{2/3}, \quad h = \begin{cases} 12 \text{ W/(m}^2\text{K)} & \text{Standard} \\ 120 \text{ W/(m}^2\text{K)} & \text{FPV Racing} \end{cases}
$$



#### Voltage Collapse & Power Overflow Check
Current draw is determined from quadratic power balance:


$$
I = \frac{V_{\text{oc}} - \sqrt{V_{\text{oc}}^2 - 4R_{\text{sys}}P_{\text{req}}}}{2R_{\text{sys}}}
$$


If the discriminant is negative:


$$
V_{\text{oc}}^2 - 4R_{\text{sys}}P_{\text{req}} < 0
$$


or if the electrical motor power exceeds threshold:


$$
P_{e,\text{motor}} > 10^7 \text{ W}
$$


a **Voltage Collapse** condition is triggered.

---

### 5. Discrete Matchmaking Optimization (J-Score)
The continuous theoretical optimum serves as coordinate search keys over the COTS database:


$$
\vec{x}_{\text{opt}} = (D_{\text{ideal}}, P_{\text{ideal}}, KV_{\text{ideal}})
$$


AeroEval minimizes the J-score:


$$
J = w_1\Delta D + w_2\Delta KV + w_3\Delta p + \text{Pen}_a + \text{Pen}_\theta
$$




$$
\Delta D = \frac{|D_t-D_{\text{ideal}}|}{D_{\text{ideal}}}, \quad \Delta KV = \frac{|KV_t-KV_{\text{ideal}}|}{KV_{\text{ideal}}}, \quad \Delta p = \frac{|p_t-p_{\text{ideal}}|}{p_{\text{ideal}}}
$$


where weights are $`w_1 = 100, w_2 = 10, w_3 = 50`$, and penalties enforce thrust reserves and pitch constraints:


$$
\text{Pen}_a = 1000\max(0,\;\theta_{\text{hvr}} - \theta_{\text{role}}), \quad \text{Pen}_\theta = 10\max(0,\;\theta_{\text{cr}} - 20^\circ)
$$



---

## Empirical Validation and Performance

AeroEval is validated against a 20-platform matrix spanning 485 g quadcopters to 76 kg industrial configurations:

* **MTOW Accuracy**: Predicted takeoff mass aligns within **4.4% Mean Absolute Percentage Error (MAPE)**.
* **Root Mean Square Error (RMSE)**: **0.63 kg** across the validation matrix.
* **Complexity Scaling Factor $k$**: Defined as:
  

$$
k = \frac{\text{MTOW}_{\text{actual}}}{\text{MTOW}_{\text{predicted}}}
$$


  It converges to a mean complexity of $k \approx 1.03 \pm 0.08$.

### 20-Platform Validation Results

| Platform | Actual (kg) | Predicted (kg) | Error (%) | $k$-factor |
| :--- | :---: | :---: | :---: | :---: |
| Agras T10 (Empty Tank Return) | 16.800 | 16.537 | -1.6% | 1.02 |
| Custom 5-inch FPV | 0.650 | 0.605 | -6.9% | 1.07 |
| Custom 7-inch FPV | 1.200 | 1.216 | +1.3% | 0.99 |
| DJI Agras T30 | 76.000 | 74.723 | -1.7% | 1.02 |
| DJI Air 3 | 0.720 | 0.741 | +2.9% | 0.97 |
| DJI Avata 2 | 0.377 | 0.383 | +1.6% | 0.98 |
| DJI FlyCart 30 | 40.000 | 40.056 | +0.1% | 1.00 |
| DJI FPV Combo | 0.795 | 0.817 | +2.8% | 0.97 |
| DJI Inspire 2 | 4.250 | 4.238 | -0.3% | 1.00 |
| DJI Inspire 3 | 3.995 | 3.719 | -6.9% | 1.07 |
| DJI Matrice 300 RTK | 9.000 | 7.379 | -18.0% | 1.22 |
| DJI Matrice 350 RTK | 9.200 | 7.337 | -20.2% | 1.25 |
| DJI Mavic 3 Classic | 0.895 | 0.929 | +3.8% | 0.96 |
| DJI Mavic 3 Enterprise | 1.050 | 1.050 | +0.0% | 1.00 |
| DJI Phantom 4 Pro V2.0 | 1.375 | 1.450 | +5.4% | 0.95 |
| Parrot Anafi USA | 0.485 | 0.471 | -2.9% | 1.03 |
| Skydio X10 | 2.490 | 2.363 | -5.1% | 1.05 |
| Sony Airpeak S1 | 4.400 | 4.433 | +0.7% | 0.99 |
| Teal Golden Eagle | 1.400 | 1.330 | -5.0% | 1.05 |
| Watts Innovations Prism | 20.360 | 20.348 | -0.1% | 1.00 |

### 14-Parameter Sensitivity Sweep Summary

| Parameter | MTOW Sensitivity | Flight Time Sensitivity |
| :--- | :--- | :--- |
| Payload mass | +1.692 kg/kg | -1.703 min/kg |
| Propeller diameter limit | +0.080 kg/in | +0.011 min/in |
| Forward speed (to 25.5 m/s) | +0.062 kg/(m/s) | -0.691 min/(m/s) |
| Forward speed (> 25.5 m/s) | *diverges* | *diverges* |
| Climb speed | +0.128 kg/(m/s) | -0.461 min/(m/s) |
| Altitude (0-5000 m) | +0.030 kg/1000 m | -0.200 min/1000 m |
| Ambient temp (+10°C) | +0.004 kg/10°C | +0.037 min/10°C |
| Aux power (+10 W) | +0.200 kg/10 W | -0.385 min/10 W |
| Added drag area | +0.062 kg/0.1 m² | -2.531 min/0.1 m² |
| Center of gravity shift | 0.000 kg/5 cm | -0.169 min/5 cm |
| Battery capacity lock | +0.100 kg/1000 mAh | +0.100 min/1000 mAh |
| Propeller diameter lock | -0.150 kg/in | 0.000 min/in |
| Delivery drop mass | -0.022 kg/kg | 0.000 min/kg |
| Delivery drop time ratio | +0.033 kg/0.1 ratio | 0.000 min/0.1 ratio |
| Spray rate | -0.490 kg/(0.005 kg/s) | 0.000 min/(0.005 kg/s) |

---

## Verification Testing

AeroEval features automated verification scripts to validate configurations:
* **Realistic Testing**: Runs randomized simulations across realistic payload weight classes and role envelopes to guarantee correct convergence.

Execute the suite using:
```bash
python tests/realistic_test.py
```

---

## Developer

* **Akshay Gupta Burela** (GitHub: [AKSHAY-RSOL](https://github.com/AKSHAY-RSOL))

---

## Citation

If you use AeroEval in your research or design projects, please cite:

```bibtex
@article{aeroeval2026,
  title={Multi-Domain Physics-Based MDO of Multirotor UAVs: A Deterministic Framework for Discrete COTS Sizing},
  author={Burela, Akshay Gupta and Sujit, P. B.},
  journal={arXiv preprint arXiv:2606.XXXXX},
  year={2026}
}
```
