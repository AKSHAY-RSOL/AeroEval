#include "../../include/module3_physics/Thermodynamics.hpp"
#include "../../include/roles/IPayloadRole.hpp"
#include "../../include/constants/PhysicalConstants.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>

namespace Module3 {

    // Resolves BLDC motor efficiency under specific load levels.
    // Linear approximation derived from typical BLDC efficiency maps (Koehl et al., 2012).
    double ThermodynamicsSolver::getMotorEfficiency(double throttle_fraction) {
        double eta = ETA_BASE + ETA_SLOPE * throttle_fraction;
        // Clamp to physically meaningful range
        if (eta < 0.50) eta = 0.50;
        if (eta > 0.95) eta = 0.95;
        return eta;
    }

    // Resolves ESC electrical efficiency under specific load levels.
    double ThermodynamicsSolver::getEscEfficiency(double throttle_fraction) {
        double eta_peak = 0.97;
        double t_opt = 0.50;  // Peak efficiency throttle point
        double eta = eta_peak - 0.8 * std::pow(throttle_fraction - t_opt, 2);
        return std::max(eta, 0.90);
    }

     /**
      * Simulates the transient battery pack thermal and OCV states across the mission.
      * Uses lumped-capacitance thermal model and Tremblay polarization.
      */
    ThermodynamicResult ThermodynamicsSolver::simulateMissionThermodynamics(
        const std::vector<double>& phase_durations_s,
        const std::vector<double>& phase_mechanical_powers_w,
        const EngineData::BatteryChemistry& chem,
        const EngineData::ESCHardware& esc,
        double ambient_temp_c,
        int cell_count,
        double total_capacity_mah,
        const EngineData::IPayloadRole* role,
        double max_diameter_m,
        int battery_cycle_count,
        int num_rotors,
        const std::string &role_type,
        const std::string &airframe_class)
    {
        ThermodynamicResult result;
        result.survived = true;
        result.max_battery_temp_c = ambient_temp_c;
        result.survived_time_s = 0.0;

        // Determine effective capacity and total charge in Ampere-seconds
        double effective_capacity_mah = (total_capacity_mah > 1.0) ? total_capacity_mah : 10000.0;
        double actual_capacity_as = (effective_capacity_mah / 1000.0) * 3600.0;

        int nr = (num_rotors > 0) ? num_rotors : 4;
        double r_i_mult = 1.0;
        if (role_type == "racing" || (role != nullptr && role->getDynamicLoadFactor() > 1.5) || airframe_class == "MicroAIO") {
            r_i_mult = 0.25; // 4x lower internal resistance for high-C racing batteries
        }
        // Scale internal resistance: R_internal scales inversely with battery capacity (assuming base 3000 mAh cell sizing)
        double r_cell_total = chem.r_i_base_ohms * cell_count * (3000.0 / effective_capacity_mah) * r_i_mult;
        double r_motor_winding = 0.008; // 8 mOhm nominal winding resistance
        double r_motor_initial = r_motor_winding * (1.0 + 0.00393 * (ambient_temp_c - 25.0)); // temperature-dependent copper resistance
        double r_total_system = r_cell_total + ((esc.r_ds_on_ohms + r_motor_initial) / static_cast<double>(nr));
        
        // Pack thermal mass and heat capacity
        double pack_energy_wh = (effective_capacity_mah / 1000.0) * (chem.v_nom_cell * cell_count);
        double battery_thermal_mass_kg = std::max(pack_energy_wh / chem.e_spec_max_wh_kg, cell_count * CELL_MASS_KG);
        double battery_thermal_cap_j_k = battery_thermal_mass_kg * CP_LITHIUM_J_KG_K;
        
        // Pack volume and surface area estimation
        double pack_volume_m3 = battery_thermal_mass_kg / 2400.0;
        double pack_surface_m2 = 6.0 * std::pow(pack_volume_m3, 2.0 / 3.0);
        double h_convective = (role_type == "racing" || airframe_class == "MicroAIO") ? 120.0 : 12.0; // Exposed high-speed airflow cooling vs enclosed fuselage
        double hA_effective = h_convective * pack_surface_m2;

        double current_soc = 1.0;
        double consumed_amp_seconds = 0.0;
        double current_temp_c = ambient_temp_c;

        // Solve peak mechanical power across all phases for normalization
        double p_mech_max = 1.0;
        for (double p : phase_mechanical_powers_w) {
            if (p > p_mech_max) p_mech_max = p;
        }

        // Integration timestep resolution (0.1 seconds)
        const double DT = 0.1; 

        for (size_t i = 0; i < phase_durations_s.size(); ++i) {
            double duration = phase_durations_s[i];
            double p_mech   = phase_mechanical_powers_w[i];
            if (duration <= 0.0 || p_mech < 0.0) continue;

            // Apply dynamic load factor to simulate peak maneuver power/current draw in thermal loop.
            if (role != nullptr && role->getDynamicLoadFactor() > 1.0 && (i > 0 || role_type == "racing")) {
                double lf = role->getDynamicLoadFactor();
                if (role_type == "racing") {
                    lf = std::min(lf, 4.0);
                }
                p_mech *= (0.85 + 0.15 * std::pow(lf, 1.5));
            }

            // CG offset calculations for front/rear motor thrust scaling
            double x_cg = 0.0;
            if (role != nullptr) {
                x_cg = std::abs(role->getCenterOfMassShiftM());
            }
            double arm_length_m = max_diameter_m / 2.0;
            // E2 FIX: pitch-axis lever arm d_y = L*cos(pi/N_eff), matching Structures.cpp.
            // Previously hardcoded to L*0.7071 = L*cos(pi/4), correct only for a quad and
            // wrong for hex/octo (over-stated the lever, distorting CG-shifted thrust split).
            int nr_eff = (nr >= 3) ? nr : 4;
            double d_y = arm_length_m * std::cos(M_PI / static_cast<double>(nr_eff));

            if (x_cg >= d_y) x_cg = d_y * 0.95;

            // Thrust required is split to front/rear motor pairs due to CG offset
            double cg_ratio = (d_y > 0.0) ? (x_cg / d_y) : 0.0;
            double half_rotors = static_cast<double>(nr) / 2.0;
            double p_mech_single_symm = p_mech / static_cast<double>(nr);
            
            double p_mech_front = p_mech_single_symm * std::pow(1.0 + cg_ratio, 1.5);
            double p_mech_rear  = p_mech_single_symm * std::pow(1.0 - cg_ratio, 1.5);
            
            // Front/rear throttle fractions based on CG shift
            double p_mech_single_max = p_mech_max / static_cast<double>(nr);
            double throttle_front = std::min(1.0, p_mech_front / p_mech_single_max);
            double throttle_rear  = std::min(1.0, p_mech_rear / p_mech_single_max);
            double eta_front = getMotorEfficiency(throttle_front);
            double eta_rear  = getMotorEfficiency(throttle_rear);
            
            double esc_eff_front = getEscEfficiency(throttle_front);
            double esc_eff_rear  = getEscEfficiency(throttle_rear);
            
            // Sum electrical power required by front and rear motors + ESC conversion efficiency
            double p_elec_motor = (half_rotors * p_mech_front / (eta_front * esc_eff_front)) + (half_rotors * p_mech_rear / (eta_rear * esc_eff_rear));

            // Guard: detect power overflow (NaN/Inf or physically impossible value).
            if (!std::isfinite(p_elec_motor) || p_elec_motor > 1.0e7) {
                result.survived = false;
                result.failure_reason = "Voltage Collapse. Cell count too low for converged drone mass. "
                    "(p_elec=" + std::to_string(p_elec_motor) + "W, cells=" + std::to_string(cell_count) + "S)";
                return result;
            }

            double last_current = p_elec_motor / (chem.v_nom_cell * cell_count);
            int steps = static_cast<int>(std::floor(duration / DT));
            double remainder = duration - steps * DT;
            int total_steps = steps + (remainder > 1e-6 ? 1 : 0);
            
            for (int t = 0; t < total_steps; ++t) {
                double dt_actual = (t == steps) ? remainder : DT;
                double c_rate = last_current / (effective_capacity_mah / 1000.0);
                double v_oc_system = getTremblayVoc(current_soc, chem, c_rate) * cell_count;

                // ESC parasitic switching losses (switching loss = 0.5 * f_sw * C_oss * V_oc^2)
                double p_switching_loss = 0.5 * PWM_FREQUENCY_HZ * esc.c_oss_farads * std::pow(v_oc_system, 2);

                // Constant sensor/payload auxiliary electrical power draw
                double p_auxiliary = (role != nullptr) ? role->getAuxiliaryPowerW() : 0.0;

                // Total electrical demand on the battery pack
                double p_total_req = p_elec_motor + p_switching_loss + p_auxiliary;

                // Solve current (amps) using Thevenin quadratic, applying age and temperature adjustments
                // Cycle-degradation aging factor on internal resistance (k_ag ~ 0.0012 /cycle;
                // literature-consistent ~20% IR rise at ~150 cycles). C4 reconciliation.
                double aging_mult = 1.0 + 0.0012 * battery_cycle_count;
                double temp_mult = std::exp(-0.013 * (current_temp_c - 25.0)); // higher temperature decreases internal resistance
                double r_cell_dynamic = chem.r_i_base_ohms * cell_count * (3000.0 / effective_capacity_mah) * aging_mult * temp_mult * r_i_mult;
                double r_motor_winding = 0.008; // 8 mOhm nominal winding resistance
                double r_motor_dynamic = r_motor_winding * (1.0 + 0.00393 * (current_temp_c - 25.0)); // copper winding resistance rises with temperature
                double r_total_system_dynamic = r_cell_dynamic + ((esc.r_ds_on_ohms + r_motor_dynamic) / static_cast<double>(nr)); 

                // Guard: NaN or Inf in total required power (overflow from bad cell count / mass)
                if (!std::isfinite(p_total_req) || p_total_req > 1.0e7) {
                    result.survived = false;
                    result.failure_reason = "Voltage Collapse. Power requirement overflow "
                        "(p_req=" + std::to_string(p_total_req) + "W, Voc="
                        + std::to_string(v_oc_system) + "V). Cell count may be too low.";
                    return result;
                }

                double current_amps;
                try {
                    // Solve quadratic loop for current (using cell resistance)
                    // FIXME: verify whether ESC resistance should also be folded in here
                    current_amps = solveQuadraticCurrent(p_total_req, v_oc_system, r_cell_dynamic);
                } catch (const std::runtime_error& e) {
                    result.survived = false;
                    result.failure_reason = "Voltage Collapse. (p_req="
                        + std::to_string(p_total_req) + "W, Voc="
                        + std::to_string(v_oc_system) + "V, Rcell="
                        + std::to_string(r_cell_dynamic) + ")";
                    return result;
                }

                // Consume charge
                consumed_amp_seconds += current_amps * dt_actual;
                current_soc = 1.0 - (consumed_amp_seconds / actual_capacity_as);
                if (current_soc < 0.01) current_soc = 0.01; 

                // Ohmic Joule heating (I^2 * R) - only includes battery and ESC resistance in the battery thermal model
                double r_battery_esc = r_cell_dynamic + (esc.r_ds_on_ohms / static_cast<double>(nr));
                double heat_j = std::pow(current_amps, 2) * r_battery_esc * dt_actual;
                
                // Convective surface cooling (lumped thermal model scaled by pack surface area)
                double cooling_j = hA_effective * (current_temp_c - ambient_temp_c) * dt_actual;
                double delta_temp = (heat_j - cooling_j) / battery_thermal_cap_j_k;
                current_temp_c += delta_temp;

                if (current_temp_c > result.max_battery_temp_c) {
                    result.max_battery_temp_c = current_temp_c;
                }
                
                // Check thermal runaway safety boundaries
                double limit_temp = Physics::MAX_THERMAL_LIMIT_C;
                if (role_type == "racing" || airframe_class == "MicroAIO") {
                    limit_temp = 80.0; // High-C racing cells can tolerate up to 80C
                }
                if (current_temp_c > limit_temp && result.survived) {
                    result.survived = false;
                    result.failure_reason = "Thermal Runaway! Temp: "
                        + std::to_string(current_temp_c) + "C > Limit: "
                        + std::to_string(limit_temp) + "C";
                }
                
                result.survived_time_s += dt_actual;
                last_current = current_amps;
            }
        }
        
        result.required_capacity_mah = (consumed_amp_seconds / 3600.0) * 1000.0;
        double total_dur = 0.0;
        for (double d : phase_durations_s) {
            total_dur += d;
        }
        if (total_dur > 0.0 && result.required_capacity_mah <= 0.0) {
            result.survived = false;
            result.failure_reason = "Zero energy drained during non-zero duration mission.";
        }
        return result;
    }

    // Tremblay-Dessaint OCV implementation
    double ThermodynamicsSolver::getTremblayVoc(double soc, const EngineData::BatteryChemistry& chem, double c_rate) {
        double calc_soc = std::clamp(soc, 0.02, 1.0);
        
        double b_eff = chem.tremblay_exp_rate;
        double calc_c_rate = std::max(1e-3, c_rate);
        if (calc_c_rate > 1.0) {
            b_eff = chem.tremblay_exp_rate * (1.0 + 0.18 * std::log(calc_c_rate)); // high C-rate accelerates polarization
        }
        
        double exponential_zone = (chem.v_full_cell - chem.v_exp_cell) * std::exp(-b_eff * (1.0 - calc_soc));
        double discharge_curve  = chem.tremblay_polarization_coeff * (1.0 / calc_soc - 1.0);
        double voc = chem.v_exp_cell + exponential_zone - discharge_curve;

        // Enforce physical minimum threshold to avoid negative open-circuit voltages
        double min_physical_voc = chem.v_nom_cell * 0.87;
        return std::max(min_physical_voc, voc);
    }

    // Solves R * I^2 - Voc * I + P = 0 for high-efficiency branch
    double ThermodynamicsSolver::solveQuadraticCurrent(double p_req, double v_oc, double r_tot) {
        double discriminant = std::pow(v_oc, 2) - (4.0 * r_tot * p_req);
        if (discriminant < 0.0) {
            throw std::runtime_error("Voltage Collapse.");
        }
        return (v_oc - std::sqrt(discriminant)) / (2.0 * r_tot);
    }

}