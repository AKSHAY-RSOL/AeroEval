#include "../../include/module5_validation/SensitivityAnalysis.hpp"
#include "../../include/constants/PhysicalConstants.hpp"
#include "../../include/module3_physics/Aerodynamics.hpp"
#include "../../include/module3_physics/Thermodynamics.hpp"
#include "../../include/roles/IPayloadRole.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>


namespace Module5 {

/**
 * Validates the discrete hardware selections and generates operating envelopes.
 * 
 * Sizing & Validation Rationale:
 * Once the discrete optimizer selects COTS hardware, the continuous assumptions (e.g. ideal motor weight, 
 * steady-state efficiency) may deviate. This function runs a transient "Hardware-In-The-Loop" (HITL) 
 * re-simulation using the actual selected prop sizes, pitches, and motor KVs. 
 * If the discrete components cause battery thermal runaway or lack sufficient thrust capacity, 
 * the validation fails and recommends targeted fixes.
 * 
 * KV Shopping Range Formulation:
 * 1. KV_max: Increments test KV by +5.0 until propeller tips exceed the acoustic Mach barrier at full throttle.
 *    Capped to prevent thermal runaways if existing margins are low.
 * 2. KV_min: Determines the minimum KV required to achieve hover thrust plus a 30% control headroom margin:
 *    RPM_min = RPM_hover * 1.30
 *    KV_min = RPM_min / V_max.
 */
FinalValidationEnvelope ValidationEngine::validateAndGenerateEnvelope(
    const EngineData::ContinuousDroneState &ideal_state,
    const Module4::DiscreteHardware &selected_hardware,
    const EngineData::UserInput &input,
    const EngineData::BatteryChemistry &chem,
    const EngineData::ESCHardware &esc,
    const Module2::MissionEvaluator &kinematics,
    const EngineData::IPayloadRole *active_role, double rho) {
  FinalValidationEnvelope envelope;
  envelope.system_validated = false;

  int cell_count = ideal_state.converged_cell_count;

  // Convert selected propeller diameter from inches to metric radius
  double prop_radius_m = (selected_hardware.prop_diameter_in * 0.0254) / 2.0;
  double discrete_prop_area_m2 = M_PI * std::pow(prop_radius_m, 2);
  int nr_sa = (ideal_state.num_rotors > 0) ? ideal_state.num_rotors : 4;
  double total_prop_area_m2 = static_cast<double>(nr_sa) * discrete_prop_area_m2;

  double max_temp_c = 0.0;
  double final_capacity_mah = 0.0;

  // Run the COTS-based Hardware-In-The-Loop re-simulation
  bool passed_hitl = executeHardwareInTheLoop(
      total_prop_area_m2, ideal_state, input, chem, esc, kinematics,
      active_role, rho, cell_count, max_temp_c, final_capacity_mah);

  if (!passed_hitl) {
    envelope.validation_message =
        "CRITICAL ERROR: Discrete hardware selection caused thermodynamic or "
        "aerodynamic failure in re-validation. The COTS components cannot "
        "support this mission safely.";

    double ideal_d_inches = ideal_state.ideal_prop_diameter_m * 39.3701;
    // Suggest specific database adjustments if propellers are severely undersized
    if (selected_hardware.prop_diameter_in < ideal_d_inches * 0.85) {
      envelope.suggested_fixes.push_back(
          "The COTS database lacks propellers large enough for this payload. "
          "The ideal propeller is " +
          std::to_string((int)ideal_d_inches) +
          " inches, but the largest available is " +
          std::to_string((int)selected_hardware.prop_diameter_in) +
          " inches. Add larger propellers to your database.");
    } else {
      envelope.suggested_fixes.push_back(
          "Relax locked constraints, reduce flight time, or reduce payload "
          "mass.");
    }
    return envelope;
  }

  // Populate validated operational envelope parameters
  envelope.system_validated = true;
  envelope.validation_message =
      "Hardware successfully validated against physical limits.";
  envelope.recommended_prop_diameter_in = selected_hardware.prop_diameter_in;
  envelope.recommended_prop_pitch_in = selected_hardware.prop_pitch_in;
  envelope.motor_kv_target = selected_hardware.motor_kv;
  envelope.final_battery_capacity_mah = final_capacity_mah;
  
  double limit_temp = Physics::MAX_THERMAL_LIMIT_C;
  if (input.payload_role.type == "racing" || input.airframe_class == "MicroAIO") {
    limit_temp = 80.0; // Enforce elevated thermal ceiling for compact/high-power layouts
  }
  envelope.thermal_margin_c = limit_temp - max_temp_c;
  envelope.max_battery_temp_c = max_temp_c;
  
  // Recalculate discrete thrust-to-weight ratio using selected COTS components
  {
    double max_voltage = chem.v_full_cell * cell_count;
    double max_rpm = selected_hardware.motor_kv * max_voltage;
    double max_rps = max_rpm / 60.0;
    double ct_used = (input.aero_overrides.assumed_ct > 0.0)
                         ? input.aero_overrides.assumed_ct
                         : 0.08; // Enforce realistic Ct for COTS props (~0.08) like in MINLP
    double max_thrust_per_motor = ct_used * rho * std::pow(max_rps, 2) *
                                  std::pow(selected_hardware.prop_diameter_in * 0.0254, 4);
    double max_total_thrust = max_thrust_per_motor * nr_sa;
    envelope.thrust_to_weight_ratio = max_total_thrust / (ideal_state.total_mass_kg * Physics::GRAVITY_MS2);
  }
  envelope.forward_pitch_angle_deg = ideal_state.forward_pitch_angle_deg;
  envelope.converged_cell_count = cell_count;

  // --- Sizing Motor KV bounds (KV shopping range) ---
  // Solve KV_max boundary: Step KV up in increments of 5.0 until tip speeds hit the acoustic Mach barrier.
  double max_voltage = chem.v_full_cell * cell_count;
  double test_kv = selected_hardware.motor_kv;
  bool first_check = true;
  while (true) {
    double max_rpm = test_kv * max_voltage;
    double tip_speed = (max_rpm * M_PI / 30.0) * prop_radius_m;
    double speed_of_sound =
        std::sqrt(Physics::GAMMA_AIR * Physics::R_SPECIFIC_AIR *
                  (input.ambient_temp_c + 273.15));
    if ((tip_speed / speed_of_sound) > Physics::MAX_TIP_MACH) {
      if (first_check) {
        test_kv = selected_hardware.motor_kv;
      }
      break;
    }

    // Cap max KV if thermal margin is already extremely tight (preventing runaway selection)
    if (envelope.thermal_margin_c < 5.0 &&
        test_kv > (selected_hardware.motor_kv + 15.0))
      break;
    if (test_kv > selected_hardware.motor_kv + 200.0)
      break;

    test_kv += 5.0;
    first_check = false;
  }
  envelope.motor_kv_max = first_check ? selected_hardware.motor_kv : (test_kv - 5.0);
  if (envelope.motor_kv_max < selected_hardware.motor_kv) {
    envelope.motor_kv_max = selected_hardware.motor_kv;
  }

  // Solve KV_min boundary: Ensures hover RPM * 1.3 control headroom margin is fully achievable
  double hover_thrust_per_motor_n =
      kinematics.getHoverState(ideal_state.total_mass_kg).thrust_req_n / static_cast<double>(nr_sa);
  // BUGFIX: use the same realistic COTS Ct (~0.08) the thrust-to-weight validation uses above
  // (line ~104). Previously this defaulted to the optimistic 0.12, which produced an overly
  // low hover-RPM estimate and therefore a kv_min shopping bound the validation itself rejects.
  double assumed_ct = (input.aero_overrides.assumed_ct > 0.0)
                          ? input.aero_overrides.assumed_ct
                          : 0.08;
  double discrete_prop_diameter_m = selected_hardware.prop_diameter_in * 0.0254;

  double required_hover_rps =
      std::sqrt(hover_thrust_per_motor_n /
                (assumed_ct * rho * std::pow(discrete_prop_diameter_m, 4)));
  double required_hover_rpm = required_hover_rps * 60.0;
  double min_allowable_rpm = required_hover_rpm * 1.3;

  envelope.motor_kv_min = min_allowable_rpm / max_voltage;
  if (envelope.motor_kv_min < (selected_hardware.motor_kv - 100.0))
    envelope.motor_kv_min = selected_hardware.motor_kv - 100.0;

  return envelope;
}

/**
 * Runs a transient thermodynamic and aerodynamic simulation of the entire mission profile 
 * using the chosen discrete COTS components.
 * 
 * Simulation Flow:
 * 1. Mass Shedding Profile: Resolves discrete drops (e.g. delivery drop cargo) and continuous 
 *    shedding (e.g. agricultural spray rates) to split the vehicle mass into "heavy" (pre-drop) 
 *    and "light" (post-drop) operating states.
 * 2. COTS Power Calculations: Evaluates mechanical power requirements using the discrete 
 *    blade diameter for both mass cases.
 * 3. Dynamic Mixing: Interpolates hover, climb, and forward flight power demands based on 
 *    when the drop occurs (drop time ratio).
 * 4. Heat Transfer Solver: Invokes the transient cooling solver (Thermodynamics.cpp) to solve 
 *    battery core temperatures, internal resistance heating, and ESC thermal dissipation limits.
 */
bool ValidationEngine::executeHardwareInTheLoop(
    double discrete_area_m2,
    const EngineData::ContinuousDroneState &ideal_state,
    const EngineData::UserInput &input,
    const EngineData::BatteryChemistry &chem,
    const EngineData::ESCHardware &esc,
    const Module2::MissionEvaluator &kinematics,
    const EngineData::IPayloadRole *active_role, double rho, int cell_count,
    double &out_max_temp, double &out_final_capacity) {
  EngineData::MissionPhase hover =
      kinematics.getHoverState(ideal_state.total_mass_kg);
  EngineData::MissionPhase climb =
      kinematics.getClimbState(ideal_state.total_mass_kg);
  EngineData::MissionPhase forward =
      kinematics.getForwardState(ideal_state.total_mass_kg);

  int nr_sa = (ideal_state.num_rotors > 0) ? ideal_state.num_rotors : 4;

  // Resolve role payload mass drops
  double discrete_drop =
      active_role ? active_role->getDiscreteMassSheddingKg() : 0.0;
  if (discrete_drop > input.payload_kg) {
      discrete_drop = input.payload_kg;
  }
  double continuous_rate =
      active_role ? active_role->getContinuousMassSheddingRateKgPerS(ideal_state.flight_time_multiplier) : 0.0;

  double active_t_hover =
      hover.duration_s * ideal_state.flight_time_multiplier;
  double active_t_climb =
      climb.duration_s * ideal_state.flight_time_multiplier;
  double active_t_forward =
      forward.duration_s * ideal_state.flight_time_multiplier;
  double total_time_active =
      active_t_hover + active_t_climb + active_t_forward;

  double total_shed = continuous_rate * total_time_active;
  if (discrete_drop + total_shed > input.payload_kg) {
    total_shed = input.payload_kg - discrete_drop;
  }
  // Calculate average structural loading profiles
  double eff_continuous_drop = total_shed * 0.5;
  double m_heavy = ideal_state.total_mass_kg - eff_continuous_drop;
  double m_light = ideal_state.total_mass_kg - discrete_drop - eff_continuous_drop;

  EngineData::MissionPhase hover_h = kinematics.getHoverState(m_heavy);
  EngineData::MissionPhase climb_h = kinematics.getClimbState(m_heavy);
  EngineData::MissionPhase forward_h = kinematics.getForwardState(m_heavy);
  EngineData::MissionPhase hover_l = kinematics.getHoverState(m_light);
  EngineData::MissionPhase climb_l = kinematics.getClimbState(m_light);
  EngineData::MissionPhase forward_l = kinematics.getForwardState(m_light);

  // Compute mechanical power targets for heavy configuration
  double p_hover_h = Module3::AerodynamicsSolver::calculateMechanicalPower(
      hover_h, rho, discrete_area_m2, input.aero_overrides,
      ideal_state.arm_outer_diameter_m, input.structural_overrides.arm_config,
      nr_sa, input.coaxial_layout, input.payload_role.type, input.airframe_class);
  double p_climb_h = Module3::AerodynamicsSolver::calculateMechanicalPower(
      climb_h, rho, discrete_area_m2, input.aero_overrides,
      ideal_state.arm_outer_diameter_m, input.structural_overrides.arm_config,
      nr_sa, input.coaxial_layout, input.payload_role.type, input.airframe_class);
  double p_forward_h = Module3::AerodynamicsSolver::calculateMechanicalPower(
      forward_h, rho, discrete_area_m2, input.aero_overrides,
      ideal_state.arm_outer_diameter_m, input.structural_overrides.arm_config,
      nr_sa, input.coaxial_layout, input.payload_role.type, input.airframe_class);

  // Compute mechanical power targets for light configuration (post-drop)
  double p_hover_l = Module3::AerodynamicsSolver::calculateMechanicalPower(
      hover_l, rho, discrete_area_m2, input.aero_overrides,
      ideal_state.arm_outer_diameter_m, input.structural_overrides.arm_config,
      nr_sa, input.coaxial_layout, input.payload_role.type, input.airframe_class);
  double p_climb_l = Module3::AerodynamicsSolver::calculateMechanicalPower(
      climb_l, rho, discrete_area_m2, input.aero_overrides,
      ideal_state.arm_outer_diameter_m, input.structural_overrides.arm_config,
      nr_sa, input.coaxial_layout, input.payload_role.type, input.airframe_class);
  double p_forward_l = Module3::AerodynamicsSolver::calculateMechanicalPower(
      forward_l, rho, discrete_area_m2, input.aero_overrides,
      ideal_state.arm_outer_diameter_m, input.structural_overrides.arm_config,
      nr_sa, input.coaxial_layout, input.payload_role.type, input.airframe_class);

  // Mix powers according to drop timing schedule
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

  std::vector<double> durations = {
      hover.duration_s * ideal_state.flight_time_multiplier,
      climb.duration_s * ideal_state.flight_time_multiplier,
      forward.duration_s * ideal_state.flight_time_multiplier};
  std::vector<double> powers = {p_mech_hover, p_mech_climb, p_mech_forward};

  // Enforce ESC transient capacitance limits to match MassIteration.cpp scaling
  EngineData::ESCHardware scaled_esc = esc;
  double p_mech_max = 0.0;
  if (hover.duration_s > 0.0) p_mech_max = std::max(p_mech_max, p_mech_hover);
  if (climb.duration_s > 0.0) p_mech_max = std::max(p_mech_max, p_mech_climb);
  if (forward.duration_s > 0.0) p_mech_max = std::max(p_mech_max, p_mech_forward);
  if (p_mech_max <= 0.0) p_mech_max = p_mech_hover;
  double p_elec_max_hint = p_mech_max / 0.75;
  double nominal_system_voltage = chem.v_nom_cell * cell_count;
  double max_current_amps = p_elec_max_hint / nominal_system_voltage;

  if (max_current_amps < 20.0) {
    scaled_esc.c_oss_farads = 1.0e-9 * std::max(1.0, max_current_amps);
    scaled_esc.r_ds_on_ohms = 0.005;
  } else if (max_current_amps > 100.0) {
    scaled_esc.c_oss_farads = 1.0e-8 * (max_current_amps / 50.0);
    scaled_esc.r_ds_on_ohms = 0.0005;
  }

  // Run the core thermodynamic simulation
  Module3::ThermodynamicResult thermo =
      Module3::ThermodynamicsSolver::simulateMissionThermodynamics(
          durations, powers, chem, scaled_esc, input.ambient_temp_c, cell_count,
          ideal_state.chosen_battery_capacity_mah, active_role,
          input.max_diameter_m, input.battery_cycle_count,
          input.num_rotors > 0 ? input.num_rotors : 4,
          input.payload_role.type,
          input.airframe_class);
  out_max_temp = thermo.max_battery_temp_c;
  out_final_capacity = thermo.required_capacity_mah;

  bool survived = thermo.survived;
  // Apply a 0.5 C physical validation tolerance for boundary discrete offsets
  double limit_temp_over = Physics::MAX_THERMAL_LIMIT_C;
  if (input.payload_role.type == "racing" || input.airframe_class == "MicroAIO") {
    limit_temp_over = 80.0;
  }
  if (!survived &&
      thermo.failure_reason.find("Thermal Runaway") != std::string::npos &&
      thermo.max_battery_temp_c <= limit_temp_over + 0.5) {
    survived = true;
  }

  if (!survived) {
    std::cout << "[HITL THERMO FAILURE] Reason: " << thermo.failure_reason
              << std::endl;
  }
  return survived;
}

} // namespace Module5