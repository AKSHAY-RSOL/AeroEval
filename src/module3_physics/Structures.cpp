#include "../../include/module3_physics/Structures.hpp"
#include "../../include/constants/PhysicalConstants.hpp"
#include <cmath>
#include <algorithm>

namespace Module3 {

    /**
     * Calculates the total airframe structure mass (carbon-fiber arms + central hub + fasteners).
     * 
     * Structural Design & Physics Principles:
     * 1. Euler-Bernoulli Cantilever Beam Sizing: 
     *    Rotor arms are modeled as cantilever beams with a point load at the tip (rotor thrust). 
     *    Under bending, maximum stress occurs at the root: sigma = (M * y) / I_x, where M = Force * Length.
     *    Sizing ensures sigma does not exceed the material's yield strength (YIELD_STRENGTH_CF_PA), 
     *    solving for the required outer diameter: d_out = cbrt( (32 * M) / (pi * yield_strength * (1 - ratio^4)) ).
     * 2. Asymmetric Load Distribution (CG-Shift): 
     *    If the center of gravity (CG) shifts from the geometric center (due to camera gimbals or payload offsets), 
     *    the drone must produce more thrust on the front arms to balance the pitch moment: T_front = T_mean * (1 + x_cg/d_y).
     *    This raises the root bending moment of the most heavily loaded arms.
     * 3. Dynamic Load Factor (Load Multiplier): 
     *    To handle dynamic flight maneuvers (high-speed banking, wind gusts, recovery climbs), 
     *    the static thrust is scaled by a dynamic load factor (typically 1.5x - 2.5x depending on mission role) 
     *    to prevent structural failure under load.
     * 4. Cantilever Natural Frequency (Resonance Clearance): 
     *    To prevent destructive aeroelastic resonance, the arm's fundamental structural natural frequency (f_nat) 
     *    must clear the propeller's blade pass frequency (f_bpf = RPS * blades) by at least a 1.5x margin. 
     *    f_nat is solved using the Euler-Bernoulli cantilever frequency constant: f_nat = (1.8751^2 / (2 * pi * L^2)) * sqrt(E*I_x / mu).
     *    If f_nat < 1.5 * f_bpf, the outer diameter is iteratively incremented to increase rigidity.
     * 5. Structural Mass Floor & Hub Sizing: 
     *    A structural floor (min_struct_frac) ensures thin-walled or small arms do not result in unphysically light hubs. 
     *    This floor is CG-aware, scaling with (1 + cg_ratio)^1.5 to account for asymmetric structural reinforcements.
     */
    double StructuresSolver::calculateFrameMass(
        double current_total_mass_kg, double max_diameter_m, double max_thrust_n, 
        int num_rotors, const EngineData::IPayloadRole* active_role, const EngineData::StructuralOverrides& overrides,
        double* out_d_out_m, double* out_t_wall_m, bool coaxial_layout,
        const std::string& airframe_class) 
    {
        int num_arms = num_rotors;
        if (coaxial_layout) {
            num_arms = num_rotors / 2;
        }

        // 1. Sizing arm geometries: Determine length of each cantilever arm
        double arm_length_m = max_diameter_m / 2.0;
        int num_rotors_val = num_rotors > 0 ? num_rotors : 4;
        int n_eff_arms = coaxial_layout ? (num_rotors_val / 2) : num_rotors_val;
        if (n_eff_arms < 3) n_eff_arms = 4;
        double d_y = arm_length_m * std::cos(M_PI / n_eff_arms); // projection distance to front motor axis

        // 2. Fetch Center of Mass (CG) shift from the active role
        double x_cg = 0.0;
        if (active_role != nullptr) {
            x_cg = std::abs(active_role->getCenterOfMassShiftM());
        }

        // Safety limit: if CG shifts past front/rear motors, clamp it to 95% of motor distance
        // to prevent mathematical singularity or unfeasible negative rear thrust solutions.
        if (x_cg >= d_y) {
            x_cg = d_y * 0.95; 
        }

        // 3. Asymmetric thrust distribution based on pitch moment equilibrium
        double thrust_per_front_arm_n = (max_thrust_n / num_arms) * (1.0 + (x_cg / d_y));

        // Scale by G-force dynamic safety limit (dynamic load factor represents max expected maneuver acceleration)
        double load_factor = active_role ? active_role->getDynamicLoadFactor() : Physics::DYNAMIC_LOAD_FACTOR;
        double force_per_front_arm_n = thrust_per_front_arm_n * load_factor;

        // 4. Bending moment at the root of the most stressed arm: Moment = Force * ArmLength
        double bending_moment_nm = force_per_front_arm_n * arm_length_m;

        // Scale bending moment for large optimized structures (tapered arms, carbon trusses, dual-plates)
        // larger spans naturally utilize structural optimizations that escape simple solid beam mass growth.
        if (max_diameter_m > 0.8) {
            bending_moment_nm *= std::sqrt(0.8 / max_diameter_m);
        }

        // 5. Solve for required Tube Geometry using Euler-Bernoulli bending stress model:
        // required_d_out = cbrt( (32 * M) / (pi * geometry_constant * yield_strength) )
        double wall_ratio = (overrides.wall_thickness_ratio > 0.0) ? overrides.wall_thickness_ratio : 0.90; // d_in / d_out
        // Clamp wall_ratio strictly below 1.0: a ratio of exactly 1.0 (zero-thickness wall) drives
        // geometry_constant -> 0 and produces an infinite required outer diameter below.
        if (wall_ratio >= 0.999) wall_ratio = 0.999;
        double geometry_constant = (overrides.geometry_constant > 0.0) ? overrides.geometry_constant : (1.0 - std::pow(wall_ratio, 4));
        // BUGFIX: guard against a non-positive geometry_constant (user override of 0/negative, or
        // wall_ratio -> 1) which would make the cbrt() below divide by zero and return inf.
        if (geometry_constant < 1e-6) geometry_constant = 1e-6;
        double required_d_out_m = std::cbrt(
            (32.0 * bending_moment_nm) / 
            (M_PI * geometry_constant * Physics::YIELD_STRENGTH_CF_PA)
        );
        double required_d_in_m = required_d_out_m * wall_ratio;

        // Apply wall thickness floor (minimum 1.5mm wall thickness for manufacturing robustness)
        double t_yield = (required_d_out_m - required_d_in_m) / 2.0;
        const double T_WALL_MIN_M = 0.0015; 
        double t_final = std::max(t_yield, T_WALL_MIN_M);
        
        // Rebuild geometry based on final wall thickness
        required_d_in_m = required_d_out_m - 2.0 * t_final;
        if (required_d_in_m < 0.0) required_d_in_m = 0.0;

        // Cantilever natural frequency clearance check: f_nat >= 1.5 * f_bpf
        // Carbon fiber physical parameters
        double E_cf = 70e9; // 70 GPa carbon fiber axial modulus
        double rho_cf = Physics::RHO_CARBON_FIBER_KG_M3; // 1600 kg/m3

        // Estimate peak RPM from max thrust per motor
        int n_eff = coaxial_layout ? (num_rotors_val / 2) : num_rotors_val;
        double overlap_ratio = M_SQRT1_2;
        if (n_eff >= 3) {
            overlap_ratio = 0.95 * std::sin(M_PI / n_eff);
        }
        double prop_diam_m = max_diameter_m * overlap_ratio;
        double thrust_max_motor = max_thrust_n / num_rotors_val;
        double max_rps = std::sqrt(std::max(0.1, thrust_max_motor) / (0.12 * 1.225 * std::pow(prop_diam_m, 4)));
        double blades = (overrides.num_blades > 0.0) ? overrides.num_blades : 2.0;
        double f_bpf = max_rps * blades;
        double f_req = 1.5 * f_bpf; // 1.5x frequency clearance factor

        double D_test = required_d_out_m;
        bool converged = false;
        for (int iter = 0; iter < 100; ++iter) {
            double D_in_test = D_test - 2.0 * t_final;
            if (D_in_test < 0.0) D_in_test = 0.0;

            double I_x = (M_PI / 64.0) * (std::pow(D_test, 4) - std::pow(D_in_test, 4)); // Area Moment of Inertia
            double A_test = (M_PI / 4.0) * (std::pow(D_test, 2) - std::pow(D_in_test, 2)); // Cross-sectional area
            double mu = rho_cf * A_test; // mass per unit length (kg/m)

            // Cantilever beam fundamental frequency formula
            double f_nat = (1.8751 * 1.8751) / (2.0 * M_PI * std::pow(arm_length_m, 2))
                         * std::sqrt((E_cf * I_x) / mu);

            if (f_nat >= f_req) {
                required_d_out_m = D_test;
                required_d_in_m = D_in_test;
                t_final = (required_d_out_m - required_d_in_m) / 2.0;
                converged = true;
                break;
            }
            D_test += 0.001; // Increment outer diameter by 1mm to increase stiffness
        }
        if (!converged) {
            required_d_out_m = D_test;
            required_d_in_m = D_test - 2.0 * t_final;
            if (required_d_in_m < 0.0) required_d_in_m = 0.0;
            t_final = (required_d_out_m - required_d_in_m) / 2.0;
        }

        if (out_d_out_m != nullptr) {
            *out_d_out_m = required_d_out_m;
        }
        if (out_t_wall_m != nullptr) {
            *out_t_wall_m = t_final;
        }

        // 6. Calculate total volume and mass of all carbon arms
        double cross_sectional_area = (M_PI / 4.0) * (std::pow(required_d_out_m, 2) - std::pow(required_d_in_m, 2));
        double volume_one_arm_m3 = cross_sectional_area * arm_length_m;
        
        // Symmetrical arms sized identically for simple manufacturing
        double total_arms_mass_kg = volume_one_arm_m3 * Physics::RHO_CARBON_FIBER_KG_M3 * num_arms;

        // 7. Sizing central hub mass based on arm mass scales
        double body_multiplier = (overrides.body_mass_multiplier > 0.0) ? overrides.body_mass_multiplier : 1.5;
        static const double BASE_BODY_MULT = 1.5;

        // Dynamic structural fraction based on drone size/diameter (square-cube law scaling)
        double min_struct_frac = 0.05 + 0.05 * max_diameter_m;
        if (min_struct_frac < 0.06) min_struct_frac = 0.06;
        if (min_struct_frac > 0.20) min_struct_frac = 0.20;

        // BUG-02 fix: The minimum structural fraction floor is made CG-aware.
        // When the floor dominates (common for mid-sized drones), the CG effect
        // on bending moment was silently absorbed. We now scale the floor by the
        // same asymmetric-thrust factor (1+cg_ratio)^1.5 used in step 3 above,
        // so a 5% CG offset raises the floor by ~7.5% — consistent with physics.
        double cg_ratio = (d_y > 0.0) ? std::min(x_cg / d_y, 0.95) : 0.0;
        double cg_floor_factor = std::pow(1.0 + cg_ratio, 1.5); // matches thrust distribution exponent

        double min_arm_mass = (current_total_mass_kg * min_struct_frac * cg_floor_factor) / BASE_BODY_MULT;
        double effective_arm_mass = std::max(min_arm_mass, total_arms_mass_kg);
        double spar_mass = effective_arm_mass * body_multiplier;

        // 8. Apply airframe class complexity modifiers (folding joints, locking hardware, landing gears)
        double gamma = 0.0;
        std::string eff_class = airframe_class;
        if (active_role != nullptr && active_role->getRoleType() == "agriculture") {
            eff_class = "Agricultural";
        }
        
        if (eff_class == "ConsumerFolding") {
            gamma = 0.15; // folding joints and structural latches
        } else if (eff_class == "EnterpriseRugged") {
            gamma = 0.50; // landing gear, camera gimbal brackets, ruggedized weather sealings
        } else if (eff_class == "Agricultural" || eff_class == "agricultural") {
            gamma = 1.20; // heavy landing gears, carbon tank mounts, plumbing fixtures
        }
        return spar_mass * (1.0 + gamma);
    }

}