#include "../../include/module3_physics/Structures.hpp"
#include "../../include/constants/PhysicalConstants.hpp"
#include <cmath>
#include <algorithm>

namespace Module3 {

    /**
     * Sizes the rotor support arms and center hub weight using a cantilever bending model
     * and natural frequency check to avoid propeller resonance.
     * TODO: unify CG projections between structures and thermal modules.
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

        // clamp CG shift inside motor boundary to avoid negative thrust
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

        // Scale bending moment for larger spans to model advanced truss structures
        if (max_diameter_m > 0.8) {
            bending_moment_nm *= std::sqrt(0.8 / max_diameter_m);
        }

        // 5. Solve for required Tube Geometry using Euler-Bernoulli bending stress model:
        // required_d_out = cbrt( (32 * M) / (pi * geometry_constant * allowable_stress) )
        // Uses the fatigue-and-FoS-adjusted allowable stress (192 MPa), derived from the
        // 720 MPa ultimate strength via ALLOWABLE_STRESS_CF_PA (C3-b reconciliation).
        double wall_ratio = (overrides.wall_thickness_ratio > 0.0) ? overrides.wall_thickness_ratio : 0.90; // d_in / d_out
        double geometry_constant = (overrides.geometry_constant > 0.0) ? overrides.geometry_constant : (1.0 - std::pow(wall_ratio, 4));
        if (geometry_constant < 1e-6) geometry_constant = 1e-6;
        double required_d_out_m = std::cbrt(
            (32.0 * bending_moment_nm) /
            (M_PI * geometry_constant * Physics::ALLOWABLE_STRESS_CF_PA)
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
        double f_req = 1.5 * f_bpf;

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

        // --- Minimum OD Floor by Airframe Class ---
        // sourcing limits and crashworthiness floors
        {
            double min_od_m = 0.006; // absolute floor regardless of class
            if (airframe_class == "MicroAIO")              min_od_m = 0.006;
            else if (airframe_class == "ConsumerFolding")  min_od_m = 0.010;
            else if (airframe_class == "EnterpriseRugged") min_od_m = 0.014;
            else if (airframe_class == "Agricultural"
                  || (active_role && active_role->getRoleType() == "agriculture"))
                                                           min_od_m = 0.016;

            if (required_d_out_m < min_od_m) {
                required_d_out_m = min_od_m;
                // Recompute inner diameter to maintain wall thickness consistency
                required_d_in_m = required_d_out_m - 2.0 * t_final;
                if (required_d_in_m < 0.0) required_d_in_m = 0.0;
            }
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

        // minimum structural fraction floor (CG-aware)
        double cg_ratio = (d_y > 0.0) ? std::min(x_cg / d_y, 0.95) : 0.0;
        double cg_floor_factor = std::pow(1.0 + cg_ratio, 1.5);

        double min_arm_mass = (current_total_mass_kg * min_struct_frac * cg_floor_factor) / BASE_BODY_MULT;
        double effective_arm_mass = std::max(min_arm_mass, total_arms_mass_kg);
        // std::cout << "DEBUG Struct: arms=" << total_arms_mass_kg << " min_arm=" << min_arm_mass << std::endl;
        double spar_mass = effective_arm_mass * body_multiplier;

        // Airframe class complexity scale (hinges, landing gear, etc.)
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