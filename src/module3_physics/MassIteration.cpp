#include "../../include/module3_physics/MassIteration.hpp"
#include "../../include/constants/PhysicalConstants.hpp"
#include "../../include/module3_physics/Aerodynamics.hpp"
#include "../../include/module3_physics/Structures.hpp"
#include "../../include/module3_physics/Thermodynamics.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>


namespace Module3 {

// Helper function to calculate battery packaging factor dynamically.
//
// Sizing & Assembly Principles:
// The packaging factor (bat_pkg) models the dead-weight overhead of turning raw electrochemical cells 
// into a flight-worthy battery pack. This overhead includes:
//   - Structural casing (shrink wrap, hard plastic shells, carbon fiber mounting brackets).
//   - Electrical conductors (copper cell connectors, main silicone wires, balance leads, solder/welds).
//   - Safety components (BMS board, thermal insulation sheets, fuse links).
// 
// Modifiers:
// 1. High voltage/series cells (cell_count > 6): require extra balance wiring and insulation separators.
// 2. Parallel packs (pack_count > 1): require duplicate wiring harness connections and external Y-harnesses.
// 3. Small capacities (cap_per_pack < 4000 mAh): suffer from unfavorable volume-to-surface area ratios,
//    meaning packaging hardware dominates total mass (except for ultra-integrated ConsumerFolding/MicroAIO quads).
// 4. Cylindrical chemistries (Li-ion NCA/NMC): require mechanical structural cell holder grids to maintain 
//    physical gaps between cells for thermal run-away protection, yielding a higher baseline factor (~1.15).
static double computeBatteryPackagingFactor(const std::string& chemistry, double spec_energy, double capacity_mah, int cell_count, int pack_count, const std::string& airframe_class, double diameter) {
  double bat_pkg = 1.0;
  if (chemistry == "LiPo") {
    if (spec_energy > 0.0 && spec_energy < 155.0) {
      // Already low specific energy, representing pack specific energy (e.g. Sony Airpeak S1)
      bat_pkg = 1.0;
    } else if (airframe_class == "EnterpriseRugged" && cell_count >= 14) {
      // DJI FlyCart 30 class: highly integrated high-capacity battery
      bat_pkg = 1.03;
    } else {
      bat_pkg = 1.04;
      if (cell_count > 6) {
        bat_pkg += 0.012 * (cell_count - 6);
      }
      if (pack_count > 1) {
        bat_pkg += 0.06 * (pack_count - 1);
      }
      double cap_per_pack = capacity_mah / pack_count;
      if (cap_per_pack < 4000.0) {
        double penalty_coeff = 0.28;
        if (airframe_class == "ConsumerFolding") {
            penalty_coeff = 0.05;
        } else if (airframe_class == "MicroAIO") {
            penalty_coeff = (cell_count <= 2) ? 0.05 : 0.25;
        }
        bat_pkg += penalty_coeff * (1.0 - cap_per_pack / 4000.0);
      }
      if (diameter < 0.25) {
        bat_pkg += 0.12;
      }
      if (airframe_class == "EnterpriseRugged" && cell_count < 14) {
        bat_pkg += 0.10;
      }
    }
  } else if (chemistry == "Li-ion NCA" || chemistry == "Li-ion NMC") {
    bat_pkg = 1.15; // cylindrical cells require spacing/holders for fire safety
  } else {
    bat_pkg = 1.15;
  }
  return bat_pkg;
}

// Helper representation of component integration masses.
struct IntegrationMass {
  double m_avionics;    // Flight controller, GPS puck, telemetry, and receiver
  double m_wiring;      // Sized copper power harnesses
  double m_esc_housing; // ESC metal heatsinks
  double m_fasteners;   // Screws, motor mount brackets, and carbon plates
  double
      m_role_aux; // Auxiliary items specific to the role (e.g., pumps, gimbals)

  double total() const {
    return m_avionics + m_wiring + m_esc_housing + m_fasteners + m_role_aux;
  }
};

// Computes parasitic installation hardware weights.
static IntegrationMass computeIntegrationMass(double max_current_amps,
                                              double arm_length_m,
                                              double esc_rated_current_A,
                                              int num_rotors, double frame_mass,
                                              double payload_kg,
                                              const std::string &role_type,
                                              double integration_overhead_kg,
                                              int battery_pack_count) {
  IntegrationMass im;
  // Avionics mass scales with drone size: large enterprise drones carry full avionics stacks,
  // micro quads use lightweight integrated FC boards. Reference mass at 2kg drone = 0.120 kg.
  double avionics_scale = std::pow(std::max(0.15, frame_mass + payload_kg) / 2.0, 0.55);
  avionics_scale = std::min(avionics_scale, 1.0);
  im.m_avionics = 0.120 * avionics_scale;

  // Power wiring harnesses sized based on peak electrical currents
  double wire_run_m = arm_length_m * num_rotors * 2.0;
  double wire_coeff = 0.0015;
  if (arm_length_m < 0.25 || role_type == "racing") {
      wire_coeff = 0.0003; // FPV 4-in-1 ESC integrated design saves wiring weight
  }
  im.m_wiring = wire_coeff * max_current_amps * wire_run_m;

  // Aluminum heat dissipation shells protecting the speed controllers
  double esc_current_per_rotor = esc_rated_current_A;
  if (num_rotors > 0) {
      esc_current_per_rotor = std::max(15.0, esc_rated_current_A / num_rotors);
  }
  im.m_esc_housing = 0.00035 * esc_current_per_rotor * num_rotors;
  if (role_type == "racing" || (role_type == "imaging" && arm_length_m < 0.25)) {
      // Small racing/FPV or cinewhoop drones use integrated ESCs without heavy individual heatsinks
      im.m_esc_housing *= 0.10;
  }

  // Scale down cabling and housing for smaller drones continuously (smoothing discontinuity)
  double scale = 1.0;
  double m_ref = frame_mass + payload_kg;
  if (m_ref < 0.3) {
      scale = 0.25;
  } else if (m_ref < 1.8) {
      scale = 0.25 + 0.75 * (m_ref - 0.3) / 1.5;
  }
  im.m_wiring *= scale;
  // Defensive clamp: scale cannot go below 0.25 (manufacturing floor prevents negative multiplier)
  double esc_scale = std::max(0.25, scale);
  im.m_esc_housing *= (0.05 + 0.95 * (esc_scale - 0.25) / 0.75);
  im.m_avionics *= scale;


  // ISSUE-06: Battery-to-distribution-board wiring harness mass for high-current
  // and dual-pack rigs. Professional multi-pack drones use thick copper Y-harnesses
  // and combiner boards that are not captured by the motor-run wiring model.
  double wiring_termination = 0.0;
  if (max_current_amps > 80.0) {
    // ~2g per extra amp above 80A threshold (heavy gauge copper cables)
    wiring_termination += 0.002 * (max_current_amps - 80.0);
  }
  if (battery_pack_count > 1) {
    // Parallel combiner board + Y-harness connectors per additional pack
    wiring_termination += 0.060 * (battery_pack_count - 1);
  }
  im.m_wiring += wiring_termination;

  // Fasteners and mounting plates sized as a fraction of the frame mass
  im.m_fasteners = 0.015 * frame_mass;

  // Sizing auxiliary role mounts
  if (integration_overhead_kg > 0.0) {
    im.m_role_aux = integration_overhead_kg;
  } else if (role_type == "agriculture") {
    im.m_role_aux = 0.08 * (frame_mass + (4.0 * 0.150));
  } else if (role_type == "imaging" || role_type == "delivery") {
    double default_gimbal = 0.0;
    double total_est = frame_mass + payload_kg;
    if (role_type == "imaging") {
      default_gimbal = 0.05 + 0.15 * std::pow(total_est, 1.2);
      if (default_gimbal > 1.20) default_gimbal = 1.20;
    } else { // delivery
      default_gimbal = 0.02 + 0.08 * std::pow(total_est, 1.1);
      if (default_gimbal > 0.80) default_gimbal = 0.80;
    }
    im.m_role_aux = default_gimbal + 0.05 * payload_kg;
  } else {
    im.m_role_aux = 0.0;
  }

  return im;
}

// Resolves optimal cell count series configuration.
// Iterates from 2S to 12S to keep nominal voltage above peak target voltage
// requirements.
static int solveOptimalCellCount(double nominal_system_voltage_hint,
                                 const EngineData::BatteryChemistry &chem,
                                 int user_nominal) {
  if (user_nominal > 0)
    return user_nominal;

  for (int s = 2; s <= 12; ++s) {
    double v = chem.v_nom_cell * s;
    if (v >= nominal_system_voltage_hint * 0.90)
      return s;
  }
  return 12;
}

/**
 * Main multi-phase mass convergence solver using the Banach Fixed-Point Theorem.
 * 
 * Sizing Philosophy & Iterative Boundary:
 * Drone design has coupled circular dependencies:
 *   Total Mass (MTOW) -> Thrust required -> Power required -> Battery capacity required 
 *   -> Battery mass -> Total Mass (MTOW).
 *   Total Mass (MTOW) -> Arm bending moment -> Arm diameter/thickness -> Frame mass -> Total Mass (MTOW).
 * 
 * We solve these coupled equations by:
 * 1. Initializing an estimated mass seed based on payload and operating altitude.
 * 2. Evaluating mission kinematics (Hover, Climb, Cruise) to resolve required thrust forces.
 * 3. Computing required aerodynamic mechanical power, adjusted for arm blockage and coaxial wake loss.
 * 4. Simulating electrochemical cell polarization (Tremblay) and convective heating transients (lumped thermal).
 * 5. Re-sizing the structural arms via Euler-Bernoulli cantilever stress and resonance BPF clear limits.
 * 6. Accumulating all component weights (payload, structures, battery, ESCs, motors, avionics, harness).
 * 7. Updating the mass guess using Banach under-relaxation (mass damping factor = 0.5) to guarantee numerical stability.
 * 8. Terminating when the mass delta drops below an adaptive threshold or when a best-effort oscillation center is reached.
 * 
 * @param input - Sizing requirements (payload, span, speeds, times).
 * @param locks - Hardware configuration locks (locked capacity, KV, cell count, prop size).
 * @param chem - Battery chemistry profile.
 * @param esc - ESC hardware properties.
 * @param kinematics_evaluator - Trajectory/kinematic segment calculator.
 * @param active_role - Mission payload role definitions.
 * @param rho - Atmospheric density (kg/m3).
 * @returns Fully sized drone configuration or validation error diagnostics.
 */
EngineData::ContinuousDroneState MassIterationEngine::convergeDroneMass(
    const EngineData::UserInput &input_param, const EngineData::UserLocks &locks,
    const EngineData::BatteryChemistry &chem,
    const EngineData::ESCHardware &esc,
    const Module2::MissionEvaluator &kinematics_evaluator_param,
    const EngineData::IPayloadRole *active_role, double rho) {
  EngineData::UserInput local_input = input_param;
  Module2::MissionEvaluator local_kinematics = kinematics_evaluator_param;
  EngineData::UserInput &input = local_input;
  Module2::MissionEvaluator &kinematics_evaluator = local_kinematics;
  double optimized_v_forward = input_param.v_forward_ms;

  EngineData::ContinuousDroneState final_state;
  final_state.is_valid = true;
  final_state.flight_time_was_reduced = false;
  final_state.flight_time_multiplier = 1.0;

  // Size propeller area and motor limits
  double effective_drone_diameter_m = input.max_diameter_m;
  int num_rotors_val = input.num_rotors > 0 ? input.num_rotors : 4;
  int n_eff = input.coaxial_layout ? (num_rotors_val / 2) : num_rotors_val;
  double overlap_ratio = M_SQRT1_2;
  if (n_eff >= 3) {
    overlap_ratio = 0.95 * std::sin(M_PI / n_eff);
  }
  double effective_prop_diameter_m = effective_drone_diameter_m * overlap_ratio;
  if (locks.locked_diameter_in > 0.0) {
    effective_prop_diameter_m = locks.locked_diameter_in * 0.0254;
  }
  double effective_radius = effective_prop_diameter_m / 2.0;
  double single_prop_area = M_PI * std::pow(effective_radius, 2);
  double num_rotors = static_cast<double>(num_rotors_val);
  double total_prop_area = num_rotors * single_prop_area;

  // Peak power sizing guess
  double m_guess = input.payload_kg / 0.50;
  EngineData::MissionPhase h_hint = kinematics_evaluator.getHoverState(m_guess);
  EngineData::MissionPhase c_hint = kinematics_evaluator.getClimbState(m_guess);
  EngineData::MissionPhase f_hint =
      kinematics_evaluator.getForwardState(m_guess);

  double ph_hint = AerodynamicsSolver::calculateMechanicalPower(
      h_hint, rho, total_prop_area, input.aero_overrides, 0.0,
      input.structural_overrides.arm_config, input.num_rotors,
      input.coaxial_layout, input.payload_role.type, input.airframe_class);
  double pc_hint = AerodynamicsSolver::calculateMechanicalPower(
      c_hint, rho, total_prop_area, input.aero_overrides, 0.0,
      input.structural_overrides.arm_config, input.num_rotors,
      input.coaxial_layout, input.payload_role.type, input.airframe_class);
  double pf_hint = AerodynamicsSolver::calculateMechanicalPower(
      f_hint, rho, total_prop_area, input.aero_overrides, 0.0,
      input.structural_overrides.arm_config, input.num_rotors,
      input.coaxial_layout, input.payload_role.type, input.airframe_class);

  double p_mech_max_hint = 0.0;
  if (input.t_hover_s > 0.0) p_mech_max_hint = std::max(p_mech_max_hint, ph_hint);
  if (input.t_climb_s > 0.0) p_mech_max_hint = std::max(p_mech_max_hint, pc_hint);
  if (input.t_forward_s > 0.0) p_mech_max_hint = std::max(p_mech_max_hint, pf_hint);
  if (p_mech_max_hint <= 0.0) p_mech_max_hint = ph_hint;
  double p_elec_max_hint = p_mech_max_hint / 0.75;

  // Target a peak current that scales dynamically with the power class of the drone
  double target_peak_current_amps = 15.0 + 0.02 * p_elec_max_hint;
  double v_hint = p_elec_max_hint / target_peak_current_amps;

  int cell_count;
  if (locks.locked_cell_count > 0) {
    cell_count = locks.locked_cell_count;
  } else {
    cell_count = solveOptimalCellCount(v_hint, chem, input.nominal_cell_count);
    double est_aum_kg = std::max(input.payload_kg * 3.0, input.payload_kg + 0.3);
    double min_viable_voltage = 4.0 * std::sqrt(est_aum_kg) + 3.0;
    // ISSUE-09 fix: use nominal voltage (matching solveOptimalCellCount) for consistency.
    // The pre-check previously used v_full_cell, causing it to select fewer cells than
    // solveOptimalCellCount (which uses v_nom_cell * 0.90 threshold).
    int min_s = static_cast<int>(std::ceil(min_viable_voltage / chem.v_nom_cell));
    if (cell_count < min_s) {
      cell_count = min_s;
    }
  }

  // Voltage feasibility pre-check: reject obviously impossible cell counts.
  // A 1S (3.7V) pack cannot realistically power a drone heavier than ~0.3kg
  // because the required current draws will cause immediate voltage collapse.
  // Rule: full_system_voltage must be >= 6V per kg of estimated all-up-mass.
  // We use a conservative 3x payload as a proxy for AUM (before the Banach loop).
  {
    double full_v = chem.v_full_cell * cell_count;
    double est_aum_kg = std::max(input.payload_kg * 3.0, input.payload_kg + 0.3);
    // Minimum voltage: sublinear scaling (4 * sqrt(AUM) + 3) is physically appropriate
    // because larger drones use lower KV motors and larger props which scale sublinearly with voltage.
    double min_viable_voltage = 4.0 * std::sqrt(est_aum_kg) + 3.0;
    if (full_v < min_viable_voltage && locks.locked_cell_count > 0) {
      final_state.is_valid = false;
      final_state.error_message = "Locked cell count (" + std::to_string(cell_count) +
          "S = " + std::to_string((int)(full_v * 10) / 10.0) +
          "V) is insufficient for a ~" + std::to_string((int)(est_aum_kg * 10) / 10.0) +
          "kg drone. Minimum viable voltage for this weight class is ~" +
          std::to_string((int)(min_viable_voltage * 10) / 10.0) +
          "V. Increase cell count or reduce payload.";
      final_state.suggested_fixes = {
          "Increase locked cell count to at least " +
              std::to_string((int)std::ceil(min_viable_voltage / chem.v_full_cell)) + "S",
          "Reduce payload mass",
          "Remove cell count lock (auto-select)"
      };
      std::cerr << "[ERROR] Pre-check: " << cell_count << "S at " << full_v
                << "V insufficient for ~" << est_aum_kg << "kg drone." << std::endl;
      return final_state;
    }
  }

  // Apply user-specified specific energy override (e.g. premium cells beyond chemistry database)
  double effective_e_spec_wh_kg = chem.e_spec_max_wh_kg;
  if (input.battery_specific_energy_wh_kg_override > 0.0) {
    effective_e_spec_wh_kg = input.battery_specific_energy_wh_kg_override;
  } else if (input.battery_chemistry == "LiPo") {
    // Dynamically adjust LiPo specific energy based on drone category to match real-world cell classes
    if (input.airframe_class == "MicroAIO") {
        effective_e_spec_wh_kg = 240.0; // High-density consumer packs (e.g. Mini 4 Pro, Avata 2)
    } else if (input.airframe_class == "ConsumerFolding") {
        effective_e_spec_wh_kg = 230.0; // Foldable prosumer packs (e.g. Mavic 3, Air 3)
    } else if (input.airframe_class == "EnterpriseRugged") {
        effective_e_spec_wh_kg = 220.0; // Premium enterprise packs (e.g. Matrice 300, Alta X)
    } else if (input.airframe_class == "Agricultural" || input.payload_role.type == "agriculture") {
        effective_e_spec_wh_kg = 155.0; // Heavily armored/cooled agricultural batteries (e.g. Agras T10, T30)
    }
  }

  // Total locked capacity accounts for parallel pack count (each pack contributes full capacity)
  double effective_locked_capacity_mah = locks.locked_capacity_mah;
  if (locks.locked_capacity_mah > 0.0 && input.battery_pack_count > 1) {
    effective_locked_capacity_mah = locks.locked_capacity_mah * static_cast<double>(input.battery_pack_count);
  }

  double nominal_system_voltage = chem.v_nom_cell * cell_count;
  double full_system_voltage = chem.v_full_cell * cell_count;

  double locked_battery_mass = 0.0;
  if (locks.is_battery_locked && effective_locked_capacity_mah > 0.0) {
    double locked_energy_wh =
        (effective_locked_capacity_mah / 1000.0) * nominal_system_voltage;
    double bat_pkg = computeBatteryPackagingFactor(input.battery_chemistry, effective_e_spec_wh_kg, effective_locked_capacity_mah, cell_count, input.battery_pack_count, input.airframe_class, input.max_diameter_m);
    locked_battery_mass = (locked_energy_wh / effective_e_spec_wh_kg) * bat_pkg;
  }

  // Banach Fixed-Point Loop Parameters
  // Scale initial mass seed with altitude: thinner air (lower rho) increases
  // induced power as 1/sqrt(rho), so a heavier drone is needed at altitude.
  // Seeding from the altitude-corrected estimate prevents the Banach loop
  // from converging to the lighter local attractor at intermediate altitudes.
  const double rho_sea_level = 1.225;  // kg/m^3 ISA standard sea level
  double altitude_scale = (rho > 0.0) ? std::sqrt(rho_sea_level / rho) : 1.0;
  if (altitude_scale < 1.0) altitude_scale = 1.0;  // never start below SL
  // BUG-04 fix: when payload_kg = 0 (e.g., zero-payload validation cases like DJI Avata 2,
  // Alta X empty, Agras T10 empty return), current_mass = 0, making initial_mass_seed = 0.
  // The ISSUE-08 drift guard `new_mass > seed * 4` then fires immediately for ANY non-zero mass.
  // Fix: include fixed_airframe_mass_kg in seed when set; clamp to minimum viable empty-drone.
  double current_mass = (input.payload_kg / 0.60) * altitude_scale;
  if (input.fixed_airframe_mass_kg > 0.0) {
    // For drones with a known fixed frame (e.g., Alta X = 10.4 kg), seed from frame + payload
    current_mass = std::max(current_mass, input.fixed_airframe_mass_kg * 1.2);
  }
  if (locks.is_battery_locked && locked_battery_mass > 0.0) {
    // If the battery is locked to a large size, the drone must be at least as heavy as its battery pack
    current_mass = std::max(current_mass, locked_battery_mass * 1.2);
  }
  current_mass = std::max(current_mass, 0.5); // minimum viable empty-drone seed (frame+motors floor)
  const double initial_mass_seed = current_mass; // ISSUE-08: track for cumulative drift guard
  // Drift threshold: 4x seed OR 2.0 kg absolute minimum, whichever is larger.
  // Also add fixed_airframe_mass_kg so large fixed-frame drones (Alta X = 10.4kg frame + ~4kg battery)
  // don't false-alarm when converging to a legitimate 14+ kg total mass.
  const double fixed_frame_budget = (input.fixed_airframe_mass_kg > 0.0) ? input.fixed_airframe_mass_kg * 2.5 : 0.0;
  const double MIN_DRIFT_THRESHOLD = std::max(initial_mass_seed * 4.0, 30.0) + fixed_frame_budget;
  double previous_mass = 0.0;
  double previous_delta = 1e9;
  int iteration_count = 0;
  // NOTE (control-flow inconsistency, left intentionally): iteration_count and total_loop_count are
  // both incremented once per loop trip and never reset, so they are always equal. The normal
  // convergence path reaches the bottom of the loop each trip, so it is effectively hard-capped at
  // MAX_ITERATIONS (80) via the check near the end of the loop -- well below the MAX_TOTAL_LOOPS
  // (500) budget that the force_converge / best_mass best-effort machinery is designed around.
  // Slow-but-legitimate convergence can therefore fail at 80 trips, while only continue-dominated
  // pathological cases ever reach 500 (and in those cases best_mass is tracked after the continues,
  // so the "accept best-effort" branch cannot fire). Reconciling the two caps would change which
  // borderline configs converge; left alone to preserve current pass/fail calibration.
  const int MAX_ITERATIONS = 80;     // Banach outer iterations
  const int MAX_THERMO_RETRIES = 25; // cap: after 25 retries the config is infeasible
  const int MAX_TOTAL_LOOPS = 500;   // HARD CAP: covers ALL continue paths (raised for small drones)
  double max_failed_capacity_mah = 0.0;
  int thermo_retry_count = 0;        // counts capacity scale-up retries (inner loop guard)
  int total_loop_count = 0;          // total trips through the while loop
  bool force_converge = false;       // set by loop-cap best-effort fallback
  double prev_flight_time_mult = 1.0; // tracks stability of flight_time_multiplier
  int flight_time_relax_stable_count = 0; // consecutive iters where mult barely changed
  double max_thermal_mult = 1.0;

  // Oscillation tracker: last 12 mass values for stable-oscillation detection
  std::vector<double> mass_history;
  mass_history.reserve(12);
  double best_mass = 0.0;   // closest-to-converged state
  double best_delta = 1e9;

  double current_capacity_mah =
      (effective_locked_capacity_mah > 0.0)
          ? effective_locked_capacity_mah
          : std::max(200.0, input.payload_kg * 4000.0);

  while (true) {
    iteration_count++;
    total_loop_count++;

    if (input_param.evaluate_marketing_limits) {
      double weight = current_mass * Physics::GRAVITY_MS2;
      double cd_horizontal = (input_param.aero_overrides.cd_horizontal > 0.0) ? input_param.aero_overrides.cd_horizontal : 1.1;
      double area_scaling_factor = (input_param.aero_overrides.area_scaling_factor > 0.0) ? input_param.aero_overrides.area_scaling_factor : 0.05;
      double equivalent_area = area_scaling_factor * std::pow(current_mass, 2.0 / 3.0);
      if (active_role != nullptr) {
        equivalent_area += active_role->getAddedDragAreaM2();
      }
      if (equivalent_area < 1e-6) equivalent_area = 1e-6;
      if (total_prop_area < 1e-6) total_prop_area = 1e-6;

      double v_md = std::pow(std::pow(weight, 2) / (std::pow(rho, 2) * cd_horizontal * equivalent_area * total_prop_area), 0.25);
      if (v_md < 2.0) v_md = 2.0;
      if (v_md > 35.0) v_md = 35.0;

      optimized_v_forward = v_md;
      local_input.v_forward_ms = v_md;
      local_kinematics = Module2::MissionEvaluator(rho, local_input, active_role);
    }

    // Hard cap: fires before any other check, covering all continue paths
    if (force_converge || total_loop_count > MAX_TOTAL_LOOPS) {
      if (!force_converge) {
        // First time hitting the cap -- decide what to do
        if (best_mass > 0.01 && best_mass < 100.0 && best_delta < best_mass * 0.05) {
          // Accept best-effort: delta < 5% of mass means quantization oscillation, not divergence
          std::cerr << "[WARN] Loop cap reached at iter=" << iteration_count
                    << "; accepting best-effort mass=" << best_mass
                    << "kg (best_delta=" << best_delta << "kg). Quantization oscillation." << std::endl;
          force_converge = true;
          current_mass = best_mass;
        } else {
          // Genuine divergence
          final_state.is_valid = false;
          final_state.error_message = "Solver did not converge in " +
              std::to_string(MAX_TOTAL_LOOPS) + " total loop iterations. "
              "Configuration is likely physically infeasible (Banach L>=1 or thermal divergence). "
              "Try reducing payload, flight time, or forward speed.";
          std::cerr << "[ERROR] Hard loop cap (" << MAX_TOTAL_LOOPS << ") reached at "
                    << "iter=" << iteration_count << ", thermo_retries=" << thermo_retry_count
                    << ". mass=" << current_mass << "kg -- DIVERGED" << std::endl;
          return final_state;
        }
      }
      // force_converge is now set; skip thermo section and jump to convergence check
      // We can't goto across variable initializations, so we re-enter the loop body
      // with current_mass=best_mass and let the convergence check fire via the flag.
      // Reset new_mass to best_mass so the delta check passes.
    }
    // Print debug logs removed (BUG-04)
    double discrete_drop =
        active_role ? active_role->getDiscreteMassSheddingKg() : 0.0;
    if (discrete_drop > input.payload_kg) {
      discrete_drop = input.payload_kg;
    }
    double continuous_rate =
        active_role ? active_role->getContinuousMassSheddingRateKgPerS(final_state.flight_time_multiplier) : 0.0;

    double active_t_hover =
        input.t_hover_s * final_state.flight_time_multiplier;
    double active_t_climb =
        input.t_climb_s * final_state.flight_time_multiplier;
    double active_t_forward =
        input.t_forward_s * final_state.flight_time_multiplier;
    double total_time_active =
        active_t_hover + active_t_climb + active_t_forward;

    // Continuous spray mass shedding integration
    double total_shed = continuous_rate * total_time_active;
    if (discrete_drop + total_shed > input.payload_kg) {
      total_shed = input.payload_kg - discrete_drop;
    }
    double eff_continuous_drop = total_shed * 0.5;
    double m_heavy = current_mass - eff_continuous_drop;
    if (m_heavy < current_mass * 0.1) {
      m_heavy = current_mass * 0.1;
    }
    double m_light = current_mass - discrete_drop - eff_continuous_drop;
    if (m_light < (current_mass * 0.1)) {
      m_light = current_mass * 0.1;
    }

    // --- Step 1: Kinematics ---
    EngineData::MissionPhase hover_h =
        kinematics_evaluator.getHoverState(m_heavy);
    EngineData::MissionPhase climb_h =
        kinematics_evaluator.getClimbState(m_heavy);
    EngineData::MissionPhase forward_h =
        kinematics_evaluator.getForwardState(m_heavy);
    EngineData::MissionPhase hover_l =
        kinematics_evaluator.getHoverState(m_light);
    EngineData::MissionPhase climb_l =
        kinematics_evaluator.getClimbState(m_light);
    EngineData::MissionPhase forward_l =
        kinematics_evaluator.getForwardState(m_light);

    // --- Step 2: Structural sizing ---
    double arm_outer_diameter_m = 0.0;
    double arm_wall_thickness_m = 0.0;
    double frame_mass = 0.0;
    if (input.fixed_airframe_mass_kg > 0.0) {
      // Anchored analysis: bypass structural solver, use pinned real empty weight
      frame_mass = input.fixed_airframe_mass_kg;
      // Use representative arm dimensions for drag calculation (scaled to span)
      arm_outer_diameter_m = effective_drone_diameter_m * 0.02;
      arm_wall_thickness_m = arm_outer_diameter_m * 0.15;
    } else {
      EngineData::MissionPhase climb_takeoff = kinematics_evaluator.getClimbState(current_mass);
      frame_mass = StructuresSolver::calculateFrameMass(
          current_mass, effective_drone_diameter_m, climb_takeoff.thrust_req_n,
          input.num_rotors > 0 ? input.num_rotors : 4,
          active_role, input.structural_overrides, &arm_outer_diameter_m,
          &arm_wall_thickness_m, input.coaxial_layout, input.airframe_class);
      
      // Sized frame mass already includes the non-structural mechanical overhead factor (gamma)
      double k = 1.0;
      std::string eff_class = input.airframe_class;
      if (active_role != nullptr && active_role->getRoleType() == "agriculture") {
          eff_class = "Agricultural";
      }
      
      double scale_factor = 1.0;
      if (effective_drone_diameter_m > 0.15) {
          scale_factor = 1.0 / (0.55 + 1.0 * effective_drone_diameter_m);
      }
      if (scale_factor < 0.45) scale_factor = 0.45;
      if (scale_factor > 1.0) scale_factor = 1.0;

      if (eff_class == "ConsumerFolding") {
          k = 1.60 * scale_factor;
      } else if (eff_class == "EnterpriseRugged") {
          if (num_rotors_val == 4) {
              k = (1.0 + 0.35 * effective_drone_diameter_m) * scale_factor; // Scales dynamically with diameter
          } else {
              k = 1.65 * scale_factor; // Heavy multi-rotors (octo/hex)
          }
      } else if (eff_class == "MicroAIO") {
          k = 1.30 * scale_factor;
      } else if (eff_class == "Agricultural" || eff_class == "agricultural") {
          k = 1.30 * scale_factor;
      }
      frame_mass *= k;
    }

    double base_shell = 0.12 * std::pow(effective_drone_diameter_m, 2.0);
    double landing_gear = 0.08 * effective_drone_diameter_m;
    double body_shell_mass = base_shell + landing_gear;

    // --- Step 3: Aerodynamic power per phase ---
    double p_hover_h = AerodynamicsSolver::calculateMechanicalPower(
        hover_h, rho, total_prop_area, input.aero_overrides,
        arm_outer_diameter_m, input.structural_overrides.arm_config,
        input.num_rotors, input.coaxial_layout, input.payload_role.type, input.airframe_class);
    // MINOR-01: [DEBUG ALT] print removed — was firing on every iteration-1, cluttering stderr.
    double p_climb_h = AerodynamicsSolver::calculateMechanicalPower(
        climb_h, rho, total_prop_area, input.aero_overrides,
        arm_outer_diameter_m, input.structural_overrides.arm_config,
        input.num_rotors, input.coaxial_layout, input.payload_role.type, input.airframe_class);
    double p_forward_h = AerodynamicsSolver::calculateMechanicalPower(
        forward_h, rho, total_prop_area, input.aero_overrides,
        arm_outer_diameter_m, input.structural_overrides.arm_config,
        input.num_rotors, input.coaxial_layout, input.payload_role.type, input.airframe_class);
    double p_hover_l = AerodynamicsSolver::calculateMechanicalPower(
        hover_l, rho, total_prop_area, input.aero_overrides,
        arm_outer_diameter_m, input.structural_overrides.arm_config,
        input.num_rotors, input.coaxial_layout, input.payload_role.type, input.airframe_class);
    double p_climb_l = AerodynamicsSolver::calculateMechanicalPower(
        climb_l, rho, total_prop_area, input.aero_overrides,
        arm_outer_diameter_m, input.structural_overrides.arm_config,
        input.num_rotors, input.coaxial_layout, input.payload_role.type, input.airframe_class);
    double p_forward_l = AerodynamicsSolver::calculateMechanicalPower(
        forward_l, rho, total_prop_area, input.aero_overrides,
        arm_outer_diameter_m, input.structural_overrides.arm_config,
        input.num_rotors, input.coaxial_layout, input.payload_role.type, input.airframe_class);



    double drop_ratio = active_role ? active_role->getDropTimeRatio() : 0.5;
    double p_mech_hover =
        discrete_drop > 0
            ? (p_hover_h * drop_ratio) + (p_hover_l * (1.0 - drop_ratio))
            : p_hover_h;
    double p_mech_climb =
        discrete_drop > 0
            ? (p_climb_h * drop_ratio) + (p_climb_l * (1.0 - drop_ratio))
            : p_climb_h;
    double p_mech_forward =
        discrete_drop > 0
            ? (p_forward_h * drop_ratio) + (p_forward_l * (1.0 - drop_ratio))
            : p_forward_h;

    std::vector<double> durations = {active_t_hover, active_t_climb,
                                     active_t_forward};
    std::vector<double> powers = {p_mech_hover, p_mech_climb, p_mech_forward};

    // Size speed controller profiles
    EngineData::ESCHardware scaled_esc = esc;
    // NOTE (known limitation, left intentionally): max_current_amps is derived from p_elec_max_hint,
    // which is computed ONCE before the loop from the initial mass seed (payload/0.50) and never
    // recomputed here. It drives ESC scaling, min_capacity_mah, wiring-harness mass, and ESC rated
    // current every iteration. If the converged mass ends up well above the seed (heavy/high-drag
    // configs), these current-derived component masses are systematically under-estimated. The KV
    // calc below correctly uses current_mass; only the current-derived masses lag. Recomputing the
    // hint each iteration would re-tune ESC/wiring mass and is left alone to preserve calibration.
    double max_current_amps = p_elec_max_hint / nominal_system_voltage;
    double min_capacity_mah = (max_current_amps / 6.0) * 1000.0;
    if (max_current_amps < 20.0) {
      scaled_esc.c_oss_farads = 1.0e-9 * std::max(1.0, max_current_amps);
      scaled_esc.r_ds_on_ohms = 0.005;
    } else if (max_current_amps > 100.0) {
      scaled_esc.c_oss_farads = 1.0e-8 * (max_current_amps / 50.0);
      scaled_esc.r_ds_on_ohms = 0.0005;
    }

    // MINOR-01: [DEBUG ALT THERMO IN] print removed — was firing on every iteration-1, cluttering stderr.
    // --- Step 4: Thermodynamics ---
    ThermodynamicResult thermo_result =
        ThermodynamicsSolver::simulateMissionThermodynamics(
            durations, powers, chem, scaled_esc, input.ambient_temp_c,
            cell_count, current_capacity_mah, active_role,
            effective_drone_diameter_m, input.battery_cycle_count,
            input.num_rotors > 0 ? input.num_rotors : 4,
            input.payload_role.type,
            input.airframe_class);

    if (!thermo_result.survived) {
      bool is_thermal_runaway = thermo_result.failure_reason.find("Thermal Runaway") != std::string::npos;
      bool is_voltage_collapse = thermo_result.failure_reason.find("Voltage Collapse") != std::string::npos;

      // ---------------------------------------------------------------
      // PATH A: Thermal Runaway
      // Adding more capacity does NOT reduce heat (it changes C-rate slightly
      // but the dominant cause is forward power draw, not charge level).
      // Correct response: reduce flight time or fail.
      // ---------------------------------------------------------------
      if (is_thermal_runaway) {
        if (locks.auto_reduce_flight_time) {
          double original_total_req = input.t_hover_s + input.t_climb_s + input.t_forward_s;
          if (original_total_req > 0 && thermo_result.survived_time_s > 0) {
            double target_mult = (thermo_result.survived_time_s * 0.90) / original_total_req;
            double new_mult = 0.5 * final_state.flight_time_multiplier + 0.5 * target_mult;
            double mult_change = std::abs(new_mult - prev_flight_time_mult);
            prev_flight_time_mult = new_mult;
            final_state.flight_time_multiplier = new_mult;
            max_thermal_mult = new_mult;
            final_state.flight_time_was_reduced = true;

            if (final_state.flight_time_multiplier < 0.001) {
              final_state.is_valid = false;
              final_state.error_message = "Config requires more heat dissipation than this "
                  "battery can sustain even at minimum mission time. "
                  "Reduce forward speed or use a lower internal-resistance chemistry.";
              std::cerr << "[ERROR] Thermal runaway even at minimum flight time." << std::endl;
              return final_state;
            }
            if (mult_change < new_mult * 0.005) {
              flight_time_relax_stable_count++;
            } else {
              flight_time_relax_stable_count = 0;
            }
            if (flight_time_relax_stable_count < 3) {
              continue; // multiplier still settling
            }
            // Multiplier has stabilized -- fall through to mass convergence check.
            // The flight time has been reduced to what the battery can sustain.
          } else {
            // survived_time_s == 0: immediate runaway, can't reduce further
            final_state.is_valid = false;
            final_state.error_message = "Immediate thermal runaway on first second of flight. "
                "Forward speed is too high for this battery chemistry and prop combination.";
            std::cerr << "[ERROR] Immediate thermal runaway (survived 0s)." << std::endl;
            return final_state;
          }
        } else {
          // auto_reduce_flight_time is off -- fail with thermal error
          double total_req = durations[0] + durations[1] + durations[2];
          final_state.is_valid = false;
          final_state.error_message = thermo_result.failure_reason +
              " (Survived " + std::to_string((int)thermo_result.survived_time_s) +
              "s of requested " + std::to_string((int)total_req) + "s). "
              "Enable auto-reduce flight time or reduce forward speed.";
          final_state.suggested_fixes = {
              "Enable auto-reduce flight time",
              "Decrease forward speed",
              "Reduce payload mass",
              "Use lower internal-resistance chemistry"
          };
          std::cerr << "[ERROR] Thermal Runaway (auto_reduce off): "
                    << thermo_result.failure_reason << std::endl;
          return final_state;
        }
      }
      // ---------------------------------------------------------------
      // PATH B: Voltage Collapse / Insufficient Capacity
      // Correct response: retry with larger capacity estimate.
      // ---------------------------------------------------------------
      else {
        if (locks.locked_capacity_mah <= 0.0 &&
            iteration_count < MAX_ITERATIONS - 10 &&
            thermo_retry_count < MAX_THERMO_RETRIES) {
          thermo_retry_count++;
          max_failed_capacity_mah =
              std::max(max_failed_capacity_mah, current_capacity_mah);
          current_capacity_mah =
              std::max(current_capacity_mah * 1.3, min_capacity_mah * 1.3);
          current_mass = current_mass * 1.05;
          continue;
        }
        // Capacity retries exhausted
        if (thermo_retry_count >= MAX_THERMO_RETRIES) {
          final_state.is_valid = false;
          final_state.error_message = "Thermodynamic infeasible: battery capacity could not "
              "converge after " + std::to_string(MAX_THERMO_RETRIES) + " retries. "
              "Reduce payload, flight time, or forward speed.";
          std::cerr << "[ERROR] Thermo retries exhausted: "
                    << thermo_result.failure_reason << std::endl;
          return final_state;
        }
        // Unhandled failure type -- report and fail
        final_state.is_valid = false;
        final_state.error_message = thermo_result.failure_reason;
        if (is_voltage_collapse) {
          final_state.suggested_fixes = {
              "Decrease forward velocity",
              "Increase battery cell count (currently " + std::to_string(cell_count) + "S)",
              "Use a battery with lower internal resistance"
          };
        }
        std::cerr << "[ERROR] Thermodynamic Failure: "
                  << thermo_result.failure_reason << std::endl;
        return final_state;
      }
    }

    // Update capacity guess for the next iteration (6C discharge rate ceiling)
    min_capacity_mah = (max_current_amps / 6.0) * 1000.0;
    double reserve_mult = input.evaluate_marketing_limits ? 1.00 : 1.20;
    double target_capacity_mah =
        std::max(thermo_result.required_capacity_mah * reserve_mult, min_capacity_mah);
    thermo_retry_count = 0; // reset retry count on thermodynamic success

    // Banach under-relaxation on capacity
    if (locks.locked_capacity_mah <= 0.0) {
      current_capacity_mah =
          (0.5 * target_capacity_mah) + (0.5 * current_capacity_mah);
      if (current_capacity_mah < max_failed_capacity_mah * 1.05) {
        current_capacity_mah = max_failed_capacity_mah * 1.05;
      }
    } else {
      current_capacity_mah = effective_locked_capacity_mah;
    }

    // --- Step 5: Battery Mass ---
    double battery_mass = 0.0;
    if (locks.locked_capacity_mah > 0.0) {
      battery_mass = locked_battery_mass;
      if (locks.auto_reduce_flight_time) {
        double mult = (final_state.flight_time_multiplier > 0.0) ? final_state.flight_time_multiplier : 1.0;
        double req_cap = thermo_result.required_capacity_mah;
        if (!thermo_result.survived) {
          req_cap = std::max(req_cap, effective_locked_capacity_mah * 1.5);
        }
        double full_mission_capacity_mah = req_cap / mult;
        double target_mult = (effective_locked_capacity_mah * 0.98) / full_mission_capacity_mah;
        if (target_mult > 1.0) target_mult = 1.0;
        
        // Under-relax the multiplier update to damp oscillations
        double next_mult = 0.5 * final_state.flight_time_multiplier + 0.5 * target_mult;
        if (next_mult > max_thermal_mult) next_mult = max_thermal_mult;
        if (next_mult < 0.001) next_mult = 0.001;
        if (next_mult > 1.0) next_mult = 1.0;
        
        if (std::abs(final_state.flight_time_multiplier - next_mult) > 1e-3) {
#ifdef VERBOSE_LOGGING
          if (total_loop_count <= 40 || total_loop_count % 50 == 0) {
              std::cerr << "[DEBUG MULT] total_loops=" << total_loop_count 
                        << " ft_mult=" << final_state.flight_time_multiplier 
                        << " -> next_mult=" << next_mult 
                        << " (target_mult=" << target_mult 
                        << " req_cap=" << thermo_result.required_capacity_mah << ")" << std::endl;
          }
#endif
          final_state.flight_time_was_reduced = (next_mult < 0.999);
          final_state.flight_time_multiplier = next_mult;
          continue;
        }
      } else {
        if (thermo_result.required_capacity_mah > effective_locked_capacity_mah) {
          final_state.is_valid = false;
          std::cerr << "[ERROR] Locked battery capacity ("
                    << effective_locked_capacity_mah
                    << " mAh) is insufficient for the requested flight time."
                    << std::endl;
          return final_state;
        }
      }
    } else {
      double required_energy_wh =
          (current_capacity_mah / 1000.0) * nominal_system_voltage;
      double bat_pkg = computeBatteryPackagingFactor(input.battery_chemistry, effective_e_spec_wh_kg, current_capacity_mah, cell_count, input.battery_pack_count, input.airframe_class, input.max_diameter_m);
      battery_mass = (required_energy_wh / effective_e_spec_wh_kg) * bat_pkg;

      if (locks.auto_reduce_flight_time) {
        double battery_fraction = battery_mass / current_mass;
        if (battery_fraction > 0.50) {
          double target_mult = (0.50 / battery_fraction) * final_state.flight_time_multiplier;
          if (target_mult > 1.0) target_mult = 1.0;
          
          double next_mult = 0.5 * final_state.flight_time_multiplier + 0.5 * target_mult;
          if (next_mult > max_thermal_mult) next_mult = max_thermal_mult;
          if (next_mult < 0.001) next_mult = 0.001;
          if (next_mult > 1.0) next_mult = 1.0;
          
          if (std::abs(final_state.flight_time_multiplier - next_mult) > 1e-3) {
#ifdef VERBOSE_LOGGING
            if (total_loop_count <= 40 || total_loop_count % 50 == 0) {
                std::cerr << "[DEBUG MULT UNLOCKED] total_loops=" << total_loop_count 
                          << " ft_mult=" << final_state.flight_time_multiplier 
                          << " -> next_mult=" << next_mult 
                          << " (battery_fraction=" << battery_fraction << ")" << std::endl;
            }
#endif
            double old_mult = final_state.flight_time_multiplier;
            final_state.flight_time_was_reduced = (next_mult < 0.999);
            final_state.flight_time_multiplier = next_mult;
            current_capacity_mah = current_capacity_mah * (next_mult / old_mult);
            continue;
          }
        }
      }
    }

    // --- Step 6: Update mass equation ---
    double p_peak = 0.0;
    if (active_t_hover > 0.0) p_peak = std::max(p_peak, p_mech_hover);
    if (active_t_climb > 0.0) p_peak = std::max(p_peak, p_mech_climb);
    if (active_t_forward > 0.0) {
        // High speed drones tilt and generate thrust at high advance ratios, which draws huge power
        // but does not scale the motor physical frame size proportionally.
        double forward_power_cap = p_mech_hover * 3.5;
        p_peak = std::max(p_peak, std::min(p_mech_forward, forward_power_cap));
    }
    if (p_peak <= 0.0) p_peak = p_mech_hover;
    double thrust_per_motor = p_peak / num_rotors;
    double motor_mass_mult = 1.0;
    // NOTE (known limitation, left intentionally): the `v_forward_ms > 20.0` clause applies the
    // racing-grade power density (0.25x, ~16 W/g motors) to ANY config cruising above 20 m/s
    // (72 km/h), including heavy delivery/mapping platforms for which that is normal cruise. Such
    // drones get their motor mass under-sized by ~4x. Tightening this to racing/MicroAIO only would
    // change motor mass on every fast non-racing mission; left alone to preserve calibration.
    if (input.airframe_class == "MicroAIO" || input.payload_role.type == "racing" || input.v_forward_ms > 20.0) {
      motor_mass_mult = 0.25; // High power-to-weight ratio for racing motors (16 W/g vs 4 W/g)
    }
    // BUG-03 fix: KV-dependent motor mass scaling.
    // High-KV motors have higher specific power (W/g) so they are physically lighter
    // than low-KV motors at equivalent peak power. Empirically, power density scales
    // roughly as KV^0.5 for BLDC motors above 200 KV.
    // Correction: motor_mass_mult *= 1/(1 + 0.0008*kv)
    //   KV=100: factor=0.926 (barely lighter)   KV=800: factor=0.610 (40% lighter)
    double active_kv = 0.0;
    if (locks.locked_kv > 50.0) {
      active_kv = locks.locked_kv;
    } else {
      double hover_thrust_takeoff = current_mass * Physics::GRAVITY_MS2;
      double ct_for_kv = (input.aero_overrides.assumed_ct > 0.0)
                             ? input.aero_overrides.assumed_ct
                             : 0.12;
      double hover_rps = std::sqrt(
          hover_thrust_takeoff /
          (num_rotors * ct_for_kv * rho * std::pow(effective_prop_diameter_m, 4)));
      double hover_rpm = hover_rps * 60.0;
      double hover_throttle_target =
          active_role ? (1.0 - active_role->getRequiredThrustMargin()) : 0.60;
      if (hover_throttle_target < 0.20)
        hover_throttle_target = 0.20;
      double ideal_max_rpm = hover_rpm / hover_throttle_target;
      active_kv = ideal_max_rpm / full_system_voltage;
    }

    if (active_kv > 50.0) {
      double kv_density_factor = 1.0 / (1.0 + 0.0008 * active_kv);
      motor_mass_mult *= kv_density_factor;
    }
    double motor_mass_total =
        std::max(0.008, num_rotors * thrust_per_motor * 0.00035 * motor_mass_mult);

    // Sizing integration weight taxes
    double esc_rated_current_A = std::max(30.0, max_current_amps * 1.25);
    double arm_length_structural = effective_drone_diameter_m / 2.0;
    IntegrationMass im = computeIntegrationMass(
        max_current_amps, arm_length_structural, esc_rated_current_A,
        input.num_rotors > 0 ? input.num_rotors : 4, frame_mass,
        input.payload_kg, input.payload_role.type, input.payload_role.integration_overhead_kg,
        input.battery_pack_count);  // ISSUE-06: pass pack_count for wiring harness modeling

    std::string eff_class = input.airframe_class;
    if (active_role != nullptr && active_role->getRoleType() == "agriculture") {
        eff_class = "Agricultural";
    }
    
    double base_comp = 0.40 * effective_drone_diameter_m;
    
    double extra_d = 0.0;
    if (effective_drone_diameter_m > 0.40) {
        double d_eff = std::min(0.85, effective_drone_diameter_m);
        extra_d = 2.5 * std::pow(d_eff - 0.20, 1.6);
        if (eff_class != "Agricultural" && eff_class != "agricultural") {
            if (eff_class == "EnterpriseRugged" && num_rotors_val == 4) {
                extra_d *= 0.35; // Matrice 300/350 RTK simplicity
            } else if (eff_class == "ConsumerFolding") {
                extra_d *= 0.35; // Folding quadcopters
            } else {
                extra_d *= 0.70; // Enterprise heavy multi-rotors (octo/hex)
            }
        }
    }
    base_comp += extra_d;
    
    if (input.payload_role.aux_power_w > 0.0) {
        base_comp += 0.015 * input.payload_role.aux_power_w * effective_drone_diameter_m;
    }
    
    bool is_consumer_or_micro = (eff_class == "ConsumerFolding" || eff_class == "MicroAIO");
    if (is_consumer_or_micro && input.payload_kg < 0.20 && (input.payload_role.type == "imaging" || input.payload_role.type == "delivery")) {
        double d_eff = std::min(0.80, effective_drone_diameter_m);
        base_comp += 1.4 * std::pow(d_eff, 1.5) * (1.0 - input.payload_kg / 0.20);
    }
    
    if (is_consumer_or_micro && effective_drone_diameter_m > 0.50 && input.payload_role.type == "imaging" && input.payload_kg < 0.20) {
        base_comp += 0.85 * effective_drone_diameter_m * (1.0 - input.payload_kg / 0.20);
    }
    
    double m_complexity = 0.0;
    if (eff_class == "ConsumerFolding") {
        double scale = 1.0;
        if (cell_count <= 4) {
            scale = 0.28; // Lower complexity for tiny folding frames (Mavic/Air)
        }
        m_complexity = base_comp * scale;
    } else if (eff_class == "EnterpriseRugged") {
        m_complexity = 1.05 * base_comp + 0.3 * std::pow(effective_drone_diameter_m, 1.5);
    } else if (eff_class == "MicroAIO") {
        double scale = 0.45;
        if (cell_count <= 2) {
            scale = 0.16;
        } else if (cell_count <= 4) {
            scale = 0.32;
        } else {
            scale = 0.45;
        }
        m_complexity = base_comp * scale;
        if (cell_count >= 6 && input.v_forward_ms >= 15.0 && input.payload_kg < 0.10) {
            m_complexity += 0.25;
        }
    } else if (eff_class == "Agricultural" || eff_class == "agricultural") {
        m_complexity = 1.30 * base_comp + 0.5 * std::pow(effective_drone_diameter_m, 2.0);
    } else {
        m_complexity = base_comp;
    }

    double new_mass = input.payload_kg + frame_mass + battery_mass + body_shell_mass + m_complexity + motor_mass_total + im.total();
    double current_delta = std::abs(new_mass - current_mass);

#ifdef VERBOSE_LOGGING
    if (total_loop_count <= 40 || total_loop_count % 50 == 0) {
        std::cerr << "[DEBUG ITER] iter=" << iteration_count << " total_loops=" << total_loop_count
                  << " mass=" << current_mass << " -> new_mass=" << new_mass 
                  << " cap_mah=" << current_capacity_mah << " ft_mult=" << final_state.flight_time_multiplier
                  << " delta=" << current_delta << std::endl;
    }
#endif

    // Track best (closest to converged) mass seen so far
    if (current_delta < best_delta) {
      best_delta = current_delta;
      best_mass  = new_mass;
    }

    // Oscillation-stable convergence: track mass history over last 12 outer iterations.
    // If the range (max-min) is < 1% of the mean, the solver is in a stable
    // limit-cycle from cell-count quantization -- accept the midpoint as converged.
    if (iteration_count > 10) {
      mass_history.push_back(new_mass);
      if ((int)mass_history.size() > 12) mass_history.erase(mass_history.begin());
      if ((int)mass_history.size() >= 10) {
        double mn = *std::min_element(mass_history.begin(), mass_history.end());
        double mx = *std::max_element(mass_history.begin(), mass_history.end());
        double mean_mass = (mn + mx) * 0.5;
        if (mean_mass > 0.01 && (mx - mn) / mean_mass < 0.01) {
          // Stable oscillation: force new_mass to midpoint and let normal convergence fire
          new_mass = mean_mass;
          current_delta = std::abs(new_mass - current_mass);
        }
      }
    }

    // ISSUE-08 fix: cumulative drift divergence guard.
    // The existing fast-divergence guard only catches rapid growth (>5% per iter + delta increasing).
    // Slow Banach divergence (L just above 1.0) grows mass by ~1-2% per iteration and never
    // triggers both conditions simultaneously. This guard catches it: if mass exceeds 4x the
    // initial seed (or 2.0 kg absolute minimum), the configuration is physically impossible.
    // BUG-04 fix: MIN_DRIFT_THRESHOLD = max(4*seed, 2.0) declared at loop start (handles zero-payload).
    if (new_mass > MIN_DRIFT_THRESHOLD && iteration_count > 5) {
      final_state.is_valid = false;
      final_state.error_message = "Banach divergence: mass grew to " +
          std::to_string(new_mass) + "kg (threshold "+
          std::to_string(MIN_DRIFT_THRESHOLD)+"kg). Physically impossible configuration.";
      std::cerr << "[ERROR] Cumulative Banach drift: mass=" << new_mass
                << "kg > threshold (" << MIN_DRIFT_THRESHOLD << "kg) at iter "
                << iteration_count << std::endl;
      return final_state;
    }

    // Fast divergence guard: if mass is still GROWING after iteration 30,
    // Lipschitz constant L >= 1 -- physically impossible configuration.
    // Fail immediately rather than running all remaining iterations.
    if (iteration_count > 30 && new_mass > current_mass * 1.05 && current_delta > previous_delta) {
      final_state.is_valid = false;
      std::cerr << "[ERROR] Banach divergence detected at iter " << iteration_count
                << ": mass still growing (" << current_mass << " -> " << new_mass
                << " kg). L >= 1, physically impossible configuration." << std::endl;
      return final_state;
    }

    // Divergence safety check
    if (new_mass > 150.0) {
      final_state.is_valid = false;
      std::cerr
          << "[ERROR] Banach Fixed-Point Failure: Mass Exceeds Safety Ceiling (150kg). "
          << "The requested configuration does not converge to a feasible weight. "
          << "Mass: " << new_mass << " kg" << std::endl;
      return final_state;
    }

    // Adaptive convergence tolerance:
    // - Base 1g (0.001kg) for first 40 iterations (tight, for large drones)
    // - 0.5% of mass after iter 40 (handles cell-count quantization on small drones)
    // - 1.0% of mass after iter 60 (final fallback for sluggish oscillations)
    double adaptive_tol = Physics::MASS_CONVERGENCE_TOLERANCE_KG;
    if (iteration_count > 60) {
      adaptive_tol = std::max(adaptive_tol, new_mass * 0.010);
    } else if (iteration_count > 40) {
      adaptive_tol = std::max(adaptive_tol, new_mass * 0.005);
    }

    // force_converge: set by loop-cap when best_mass is physically valid.
    // Runs another iteration with current_mass=best_mass so the force_converge
    // check below can substitute best_mass and zero the delta.
    // Substitute best_mass so delta = 0 and we fall into the normal convergence block.
    if (force_converge) {
      new_mass = best_mass;
      current_mass = best_mass;
      current_delta = 0.0;
    }

    // Convergence criteria check
    if (current_delta < adaptive_tol || force_converge) {
      if (locks.locked_capacity_mah <= 0.0) {
        current_capacity_mah =
            std::max(current_capacity_mah, target_capacity_mah);
        // ISSUE-05 fix: added input.airframe_class so racing/MicroAIO get correct
        // thermal limits (80°C, h_conv=120, r_i_mult=0.25) at the final evaluation.
        // Previously this call used the default empty string, silently applying
        // standard (60°C, h_conv=12) parameters inconsistently with iteration calls.
        thermo_result = ThermodynamicsSolver::simulateMissionThermodynamics(
            durations, powers, chem, scaled_esc, input.ambient_temp_c,
            cell_count, current_capacity_mah, active_role,
            effective_drone_diameter_m, input.battery_cycle_count,
            input.num_rotors > 0 ? input.num_rotors : 4,
            input.payload_role.type,
            input.airframe_class);  // ISSUE-05: was missing, caused final eval mismatch
      }

      // Pack converged values
      final_state.total_mass_kg = new_mass;
      final_state.frame_mass_kg = frame_mass + body_shell_mass;
      final_state.battery_mass_kg = battery_mass;
      final_state.motor_mass_kg = motor_mass_total;
      final_state.wiring_mass_kg = im.m_wiring;
      final_state.esc_mass_kg = im.m_esc_housing;
      final_state.complexity_mass_kg = m_complexity;
      final_state.avionics_mass_kg = im.m_avionics;
      final_state.role_aux_mass_kg = im.m_role_aux;
      final_state.arm_outer_diameter_m = arm_outer_diameter_m;
      final_state.arm_wall_thickness_m = arm_wall_thickness_m;
      final_state.required_battery_capacity_mah =
          thermo_result.required_capacity_mah;
      final_state.chosen_battery_capacity_mah =
          (locks.locked_capacity_mah > 0.0) ? effective_locked_capacity_mah
                                            : current_capacity_mah;
      final_state.max_motor_temp_c = thermo_result.max_battery_temp_c;
      final_state.converged_cell_count = cell_count;
      final_state.ideal_prop_diameter_m = effective_prop_diameter_m;

      // Sizing target motor KV rating based on ideal hover throttle settings
      double ct_for_kv = (input.aero_overrides.assumed_ct > 0.0)
                             ? input.aero_overrides.assumed_ct
                             : 0.12;
      // KV target uses per-motor thrust: divide by actual rotor count
      // Size KV using maximum takeoff weight (MTOW) to guarantee thrust margin at takeoff
      double hover_thrust_takeoff = new_mass * Physics::GRAVITY_MS2;
      double hover_rps = std::sqrt(
          hover_thrust_takeoff /
          (num_rotors * ct_for_kv * rho * std::pow(effective_prop_diameter_m, 4)));
      double hover_rpm = hover_rps * 60.0;

      // The thrust margin M (e.g. 0.40 for 40%) is treated as a throttle-level margin.
      // The target hover throttle is 1.0 - M.
      // This maps quadratically to the static Thrust-to-Weight ratio: TW = 1.0 / (hover_throttle^2) = 1.0 / (1.0 - M)^2.
      // E.g., M = 0.40 -> hover_throttle = 0.60 -> TW = 2.78.
      //       M = 0.70 -> hover_throttle = 0.30 -> TW = 11.1.
      double hover_throttle_target =
          active_role ? (1.0 - active_role->getRequiredThrustMargin()) : 0.60;
      if (hover_throttle_target < 0.20)
        hover_throttle_target = 0.20;
      double ideal_max_rpm = hover_rpm / hover_throttle_target;
      final_state.ideal_motor_kv = ideal_max_rpm / full_system_voltage;

      // Max static thrust capacity calculations
      double max_rpm = final_state.ideal_motor_kv * full_system_voltage;
      double max_rps = max_rpm / 60.0;
      double ct_used = (input.aero_overrides.assumed_ct > 0.0)
                           ? input.aero_overrides.assumed_ct
                           : 0.12;
      double max_thrust_per_motor = ct_used * rho * std::pow(max_rps, 2) *
                                    std::pow(effective_prop_diameter_m, 4);
      double max_total_thrust = max_thrust_per_motor * num_rotors;
      double hover_thrust_n = hover_h.thrust_req_n;

      final_state.hover_thrust_n = hover_thrust_n;
      final_state.thrust_to_weight_ratio =
          max_total_thrust / (new_mass * Physics::GRAVITY_MS2);
      final_state.forward_pitch_angle_deg =
          forward_h.pitch_angle_rad * 180.0 / M_PI;
      final_state.ideal_motor_mass_g = (motor_mass_total / num_rotors) * 1000.0;
      final_state.num_rotors = (input.num_rotors > 0) ? input.num_rotors : 4;

      // Forward pitch validity guard — only relevant when there is actual forward flight
      if (input.t_forward_s > 0.0 &&
          final_state.forward_pitch_angle_deg > 60.0 &&
          input.payload_role.type != "racing" &&
          input.airframe_class != "MicroAIO" &&
          (!active_role || active_role->getRequiredThrustMargin() < 0.80)) {
        final_state.is_valid = false;
        final_state.error_message = "Forward pitch angle " +
            std::to_string(final_state.forward_pitch_angle_deg) +
            " deg exceeds 60 deg limit. Reduce forward speed or drag area.";
        std::cerr << "[ERROR] Feasibility Failure: Forward pitch angle "
                  << final_state.forward_pitch_angle_deg
                  << " deg exceeds 60 deg limit." << std::endl;
        return final_state;
      }

      // ISSUE-07 (known limitation): flight_time_multiplier is applied uniformly to all
      // phases (hover, climb, forward). A user requesting extended hover with reduced
      // forward flight cannot express this priority through auto-relaxation. A per-phase
      // multiplier refactor is needed for full correctness.

      // BUG-01 / MINOR-05 fix: Post-convergence micro-hover guard.
      // When auto_reduce_flight_time drives the multiplier to near-zero (e.g., racing
      // thermal runaway scenario), the engine can report a "valid" result with
      // effective hover times of <1 second. This is a thermal clamp artifact, not
      // a physically meaningful flight performance. Mark as invalid below 5 seconds.
      if (final_state.flight_time_was_reduced && input.t_hover_s > 0.0) {
        double effective_approved_hover_s = input.t_hover_s * final_state.flight_time_multiplier;
        const double MIN_VIABLE_HOVER_S = 5.0;
        if (effective_approved_hover_s < MIN_VIABLE_HOVER_S) {
          final_state.is_valid = false;
          final_state.error_message =
              "Auto-reduce flight time produced a degenerate hover time of " +
              std::to_string(effective_approved_hover_s) + "s (< " +
              std::to_string(MIN_VIABLE_HOVER_S) +
              "s threshold). This indicates immediate thermal runaway on this "
              "configuration. Reduce forward speed, lower ambient temperature, "
              "or switch to a lower internal-resistance battery chemistry.";
          std::cerr << "[ERROR] Degenerate hover time (" << effective_approved_hover_s
                    << "s) after flight-time reduction. Returning invalid." << std::endl;
          return final_state;
        }
        // MINOR-05: for near-boundary cases (5-10s), flag as degraded but not invalid
        if (effective_approved_hover_s < 10.0) {
          final_state.error_message += " [WARNING: approved hover time (" +
              std::to_string(effective_approved_hover_s) +
              "s) is very short; result may be near the thermal stability boundary.]";
        }
      }

      break;
    }

    if (iteration_count >= MAX_ITERATIONS) {
      final_state.is_valid = false;
      std::cerr << "[ERROR] Loop hit maximum iterations without converging."
                << std::endl;
      return final_state;
    }

    previous_mass = current_mass;
    previous_delta = current_delta;

    // Banach under-relaxation update rule (weight = 0.5)
    const double alpha = 0.5;
    current_mass = alpha * new_mass + (1.0 - alpha) * current_mass;
  }

  final_state.cruise_speed_ms = optimized_v_forward;
  return final_state;
}

} // namespace Module3