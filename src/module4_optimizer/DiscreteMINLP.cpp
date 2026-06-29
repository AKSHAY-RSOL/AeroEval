#include "../../include/module4_optimizer/DiscreteMINLP.hpp"
#include "../../include/constants/PhysicalConstants.hpp"
#include "../../include/roles/IPayloadRole.hpp"
#include <cmath>
#include <iostream>
#include <limits>


namespace Module4 {

/**
 * Solves the Mixed-Integer Non-Linear Programming (MINLP) hardware selection problem.
 * 
 * Sizing & Optimization Rationale:
 * Core physics equations solve for "continuous" ideal parameters (e.g. ideal motor weight, 
 * ideal propeller diameter). Real projects must build on discrete Commercial Off-The-Shelf (COTS) 
 * hardware components. This solver scans a discrete combinatorial search space (cross product 
 * of available propeller diameters, pitches, and motor KV options) to find the configuration 
 * minimizing the objective penalty J-score.
 * 
 * Parameters:
 * - ideal_state: Continuous drone parameters output by the convergence iteration.
 * - locks: User locks restricting diameter, pitch, or motor KV to specific choices.
 * - hardware_database: COTS database vectors.
 */
DiscreteHardware DiscreteOptimizer::optimizeHardware(
    const EngineData::ContinuousDroneState &ideal_state,
    const EngineData::UserLocks &locks,
    const EngineData::COTS_Sets &hardware_database,
    const EngineData::BatteryChemistry &chem, double speed_of_sound_a,
    double rho, const EngineData::AerodynamicOverrides &aero_overrides,
    const EngineData::IPayloadRole *active_role) {
  DiscreteHardware best_hardware = {
      0.0, 0.0, 0.0, std::numeric_limits<double>::max(), false, "", {}};

  // Combinatorial nested search over all DB propeller diameters, pitches, and motor KVs
  for (double test_d : hardware_database.prop_diameters) {
    // Apply propeller diameter locks check (tolerance ±0.01 inch)
    if (locks.is_prop_locked &&
        std::abs(test_d - locks.locked_diameter_in) > 0.01)
      continue;

    for (double test_p : hardware_database.prop_pitches) {
      if (locks.is_prop_locked && locks.locked_pitch_in > 0.0 &&
          std::abs(test_p - locks.locked_pitch_in) > 0.01)
        continue;

      for (double test_kv : hardware_database.motor_kvs) {
        // Apply motor rating locks check
        if (locks.is_motor_locked && std::abs(test_kv - locks.locked_kv) > 0.5)
          continue;

        double current_j_score = calculateObjectiveFunction(
            test_d, test_p, test_kv, ideal_state, chem, speed_of_sound_a, rho,
            aero_overrides, active_role);

        // Track the candidate with the lowest J-score (least penalty deviation)
        if (current_j_score < best_hardware.j_score) {
          best_hardware.prop_diameter_in = test_d;
          best_hardware.prop_pitch_in = test_p;
          best_hardware.motor_kv = test_kv;
          best_hardware.j_score = current_j_score;
        }
      }
    }
  }

  // Check if optimal configuration is valid (infinite penalty implies failure)
  if (best_hardware.j_score >= Physics::PENALTY_INFINITY) {
    best_hardware.is_valid = false;
    best_hardware.error_message =
        "No safe discrete COTS components exist for the given constraints.";
    best_hardware.suggested_fixes = {"Relax propeller or motor locks",
                                     "Reduce payload mass",
                                     "Adjust mission parameters"};
    std::cerr << "[ERROR] MINLP Failure: No safe discrete COTS components "
                 "exist for this mission constraint."
              << std::endl;
  } else {
    best_hardware.is_valid = true;
    best_hardware.error_message.clear();
    best_hardware.suggested_fixes.clear();
  }
  return best_hardware;
}

/**
 * Calculates the composite J-score penalty representing the deviation of a candidate 
 * discrete hardware triplet from the continuous theoretical ideal design.
 * 
 * Penalty Formulations:
 * 1. Mach Limit Check: Rejects configurations where full-throttle tip speeds exceed acoustic limits.
 * 2. Propeller Diameter Deviation: Enforces standard penalties for deviating from ideal sizing. 
 *    Rejects oversized propellers that exceed maximum airframe limits (MINLP-02).
 * 3. Motor KV Rating Deviation: Penalizes deviations from the ideal back-EMF KV target.
 * 4. Pitch Ratio Adjustment: Matches target pitch-to-diameter ratios (MINLP-03).
 *    Heavy-lift / high disc-loading configurations demand higher pitch angles (ideal pitch ratio ~0.42)
 *    to prevent blade stalling under high thrust loads, while racing/agri props require lower pitch targets (~0.28).
 * 5. Control Authority & Hover Throttle (MINLP-01): Computes max static thrust based on a realistic 
 *    COTS thrust coefficient (Ct ≈ 0.08, rather than optimistic 0.12). Penalizes configurations 
 *    if the required hover thrust demands > 50-80% throttle (depending on active role agility requirements).
 * 6. Attitude Angle Limits: Penalizes configurations requiring forward cruise pitch tilt angles > 20 degrees.
 */
double DiscreteOptimizer::calculateObjectiveFunction(
    double test_d, double test_p, double test_kv,
    const EngineData::ContinuousDroneState &ideal_state,
    const EngineData::BatteryChemistry &chem, double a, double rho,
    const EngineData::AerodynamicOverrides &aero_overrides,
    const EngineData::IPayloadRole *active_role) {
  double j_score = 0.0;
  double ideal_d_inches = ideal_state.ideal_prop_diameter_m * 39.3701;
  int cell_count = ideal_state.converged_cell_count;

  // BUGFIX: guard against a degenerate ideal state. If the continuous solver produced a
  // zero/near-zero ideal diameter or KV, the deviation ratios below would divide by zero and
  // emit inf/nan J-scores that silently win the best_hardware comparison. Reject such candidates.
  if (ideal_d_inches < 1e-6 || ideal_state.ideal_motor_kv < 1e-6) {
    return Physics::PENALTY_INFINITY;
  }

  // 1. Acoustic Mach limit check
  if (!checkMachLimit(test_d, test_kv, chem, a, cell_count)) {
    return Physics::PENALTY_INFINITY;
  }

  // Restrict COTS under-sizing to prevent unphysical small propeller selection
  if (test_d < 0.50 * ideal_d_inches) {
    return Physics::PENALTY_INFINITY;
  }

  // --- MINLP-02 FIX: Restrict oversized propellers violating maximum diameter constraints ---
  if (test_d > ideal_d_inches + 0.01) {
    return Physics::PENALTY_INFINITY;
  }

  // 2. Propeller diameter deviation penalty
  double d_deviation = std::abs(test_d - ideal_d_inches) / ideal_d_inches;
  j_score += (d_deviation * Physics::WEIGHT_THRUST_MARGIN);

  // 3. Motor KV rating deviation penalty
  double kv_deviation = std::abs(test_kv - ideal_state.ideal_motor_kv) /
                        ideal_state.ideal_motor_kv;
  j_score += (kv_deviation * Physics::WEIGHT_KV_DEVIATION);

  // 4. Ideal pitch deviation penalty
  double ideal_pd_ratio = 0.333; // default hover
  
  // --- MINLP-03 FIX: Apply higher pitch ratio thresholds for high disc-loading drones ---
  int nr_minlp = (ideal_state.num_rotors > 0) ? ideal_state.num_rotors : 4;
  double test_d_meters_dl = test_d * 0.0254;
  double disc_area = nr_minlp * M_PI * std::pow(test_d_meters_dl / 2.0, 2);
  double disc_loading_kg_m2 = ideal_state.total_mass_kg / disc_area;
  if (disc_loading_kg_m2 > 15.0) {
    ideal_pd_ratio = 0.42; // Heavy lifts require higher pitch to sustain high load factors
  }

  if (active_role && active_role->getRequiredThrustMargin() > 0.60) {
    ideal_pd_ratio = 0.280; // racing
  } else if (aero_overrides.propeller_class == "AG" ||
             aero_overrides.propeller_class == "agricultural" ||
             aero_overrides.propeller_class == "AGRICULTURAL") {
    ideal_pd_ratio = 0.270; // agricultural (e.g. 1:3.7)
  }
  double ideal_pitch = test_d * ideal_pd_ratio;
  double pitch_deviation = std::abs(test_p - ideal_pitch) / ideal_pitch;
  j_score += (pitch_deviation * Physics::WEIGHT_CURRENT_EFFICIENCY);

  // Under-sizing diameter penalty scaling
  if (test_d < ideal_d_inches) {
    j_score += (d_deviation * Physics::WEIGHT_THRUST_MARGIN * 2.0);
  }

  // --- PENALTY: Control Authority Hover Thrust Margin (MINLP-01) ---
  double required_margin = active_role != nullptr ? active_role->getRequiredThrustMargin() : 0.40;
  double max_allowable_hover_throttle = 1.0 - required_margin;

  double max_voltage = chem.v_full_cell * cell_count;
  double max_rpm = test_kv * max_voltage;
  double test_d_meters = test_d * 0.0254;

  // T = Ct * rho * n^2 * D^4
  // --- MINLP-01 FIX: Enforce realistic Ct for COTS props (~0.08) rather than optimistic 0.12 ---
  double assumed_ct =
      (aero_overrides.assumed_ct > 0.0) ? aero_overrides.assumed_ct : 0.08;
  double max_rps = max_rpm / 60.0;
  double max_discrete_thrust_per_motor =
      assumed_ct * rho * std::pow(max_rps, 2) * std::pow(test_d_meters, 4);
  int nr_minlp_count = (ideal_state.num_rotors > 0) ? ideal_state.num_rotors : 4;
  double max_discrete_thrust_total = max_discrete_thrust_per_motor * static_cast<double>(nr_minlp_count);

  double hover_throttle_fraction =
      (ideal_state.total_mass_kg * Physics::GRAVITY_MS2) /
      max_discrete_thrust_total;

  if (hover_throttle_fraction > max_allowable_hover_throttle) {
    j_score +=
        ((hover_throttle_fraction - max_allowable_hover_throttle) * 1000.0);
  }

  // --- PENALTY: Forward pitch tilt angle limits ---
  double candidate_pitch_deg = ideal_state.forward_pitch_angle_deg;
  if (candidate_pitch_deg > 20.0) {
    double excess = candidate_pitch_deg - 20.0;
    j_score += excess * Physics::WEIGHT_KV_DEVIATION;
  }

  return j_score;
}

/**
 * Checks if the propeller blade tip speed under maximum battery voltage exceeds acoustic Mach limits.
 * Enforces a safety boundary to prevent propellers from entering transonic flow regimes, 
 * which cause massive compressibility drag rises and noise envelopes.
 */
bool DiscreteOptimizer::checkMachLimit(double test_d_in, double test_kv,
                                       const EngineData::BatteryChemistry &chem,
                                       double a, int cell_count) {
  double max_voltage = chem.v_full_cell * cell_count;
  double max_rpm = test_kv * max_voltage;
  double test_d_meters = test_d_in * 0.0254;
  double max_omega = max_rpm * (M_PI / 30.0);
  double max_tip_speed = max_omega * (test_d_meters / 2.0);
  return (max_tip_speed / a) <= Physics::MAX_TIP_MACH;
}

} // namespace Module4