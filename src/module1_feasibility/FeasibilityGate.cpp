#include "../../include/module1_feasibility/FeasibilityGate.hpp"
#include <algorithm>
#include <cmath>

namespace Module1 {

/**
 * FeasibilityGate checks initial vehicle constraints.
 * Ensures basic feasibility (payload compatibility, downwash speed, tip Mach limits, Ragone limits)
 * before starting expensive mass convergence loops.
 */
FeasibilityResult
FeasibilityGate::evaluateMission(const EngineData::UserInput &user_input,
                                 const EngineData::UserLocks &locks,
                                 const EngineData::BatteryChemistry &chemistry,
                                 const EngineData::IPayloadRole *active_role) {
  FeasibilityResult result;
  result.is_possible = true;
  result.fail_reason = "Mission is theoretically feasible.";

  // Resolve environmental density and speed of sound at local altitude using ICAO standard
  calculateAtmospherics(user_input.altitude_m, user_input.ambient_temp_c,
                        result.env_density_rho, result.env_speed_of_sound);

  std::string error_buffer;

  // Agility-Payload Compatibility
  if (active_role != nullptr && active_role->getRoleType() == "racing" && user_input.payload_kg > 0.5) {
    result.is_possible = false;
    result.fail_reason = "Racing configurations require high thrust margin, payload cannot exceed 0.5 kg.";
    return result;
  }

  // --- GATE 1: Rotor Disk Loading (Vortex Ring State Prevention) ---
  if (!checkStructuralAndDiskLoading(user_input, locks, result.env_density_rho,
                                     active_role, error_buffer)) {
    // C7: For agriculture, exceeding the downwash cap is a crop-damage/drift advisory,
    // not a physical VRS impossibility. Warn and continue rather than hard-failing.
    bool is_agriculture =
        (active_role != nullptr && active_role->getRoleType() == "agriculture");
    bool is_downwash_failure =
        error_buffer.find("Disk Loading Failure") != std::string::npos;
    if (is_agriculture && is_downwash_failure) {
      result.warnings.push_back(
          "Advisory: " + error_buffer +
          " (agriculture downwash cap is a crop-damage/drift guideline, not a hard "
          "limit; sizing proceeds. Increase diameter or rotor count to reduce spray drift.)");
    } else {
      result.is_possible = false;
      result.fail_reason = error_buffer;
      return result;
    }
  }

  // --- GATE 2: Propeller Tip Acoustic Mach Limits ---
  if (!checkAeroacousticMachLimit(user_input, locks, result.env_density_rho,
                                  result.env_speed_of_sound, error_buffer)) {
    result.is_possible = false;
    result.fail_reason = error_buffer;
    return result;
  }

  // --- GATE 3: Ragone specific energy limits (bypass if auto-reduce is enabled) ---
  if (!locks.auto_reduce_flight_time && !checkRagoneEnvelope(user_input, chemistry, result.env_density_rho,
                           error_buffer)) {
    result.is_possible = false;
    result.fail_reason = error_buffer;
    computeParetoFrontier(user_input, chemistry, result.env_density_rho,
                          result);
    return result;
  }

  return result;
}

/**
 * Calculates density (rho) and speed of sound (a) using the standard atmosphere model.
 * Handles troposphere and stratosphere pressure/temperature profiles.
 * TODO: verify temperature lapse behavior at negative ambient inputs.
 */
void FeasibilityGate::calculateAtmospherics(double alt_m, double temp_c,
                                            double &rho, double &a) {
  double temp_k = temp_c + 273.15;
  double pressure_pa = 0.0;
  double lapse_rate = 0.0065; // standard L = 6.5 K/km
  double exponent = Physics::GRAVITY_MS2 / (Physics::R_SPECIFIC_AIR * lapse_rate);

  if (alt_m <= 11000.0) {
    // Troposphere layer: temperature lapses linearly
    pressure_pa =
        Physics::P0_SEA_LEVEL_PA *
        std::pow(1.0 - (lapse_rate * alt_m / Physics::T0_SEA_LEVEL_K), exponent);
  } else if (alt_m <= 20000.0) {
    // Lower Stratosphere layer: isothermal zone
    double p11000 =
        Physics::P0_SEA_LEVEL_PA *
        std::pow(1.0 - (lapse_rate * 11000.0 / Physics::T0_SEA_LEVEL_K), exponent);
    pressure_pa = p11000 * std::exp(-Physics::GRAVITY_MS2 * (alt_m - 11000.0) /
                                    (Physics::R_SPECIFIC_AIR * 216.65));
  } else {
    // Upper Stratosphere layer: temperature starts rising slightly
    double p11000 =
        Physics::P0_SEA_LEVEL_PA *
        std::pow(1.0 - (lapse_rate * 11000.0 / Physics::T0_SEA_LEVEL_K), exponent);
    double p20000 = p11000 * std::exp(-Physics::GRAVITY_MS2 * (20000.0 - 11000.0) /
                                      (Physics::R_SPECIFIC_AIR * 216.65));
    double temp_ratio = 216.65 / (216.65 + 0.001 * (alt_m - 20000.0));
    pressure_pa = p20000 * std::pow(temp_ratio, 34.1632);
  }

  // blending field pressure and user temperature for density sizing
  rho = pressure_pa / (Physics::R_SPECIFIC_AIR * temp_k);
  a = std::sqrt(Physics::GAMMA_AIR * Physics::R_SPECIFIC_AIR * temp_k);
}

/**
 * Validates rotor disk loading to avoid downwash conditions that induce Vortex Ring State (VRS).
 */
bool FeasibilityGate::checkStructuralAndDiskLoading(
    const EngineData::UserInput &input, const EngineData::UserLocks &locks,
    double rho, const EngineData::IPayloadRole *active_role, std::string &err) {
  int nr_disk = input.num_rotors > 0 ? input.num_rotors : 4;
  int n_eff = input.coaxial_layout ? (nr_disk / 2) : nr_disk;
  
  // Enforce frame geometry constraints to calculate maximum possible propeller radius
  double overlap_ratio = M_SQRT1_2;
  if (n_eff >= 3) {
    overlap_ratio = 0.95 * std::sin(M_PI / n_eff);
  }
  double prop_radius_m = (locks.locked_diameter_in > 0.0)
                             ? (locks.locked_diameter_in * 0.0254) / 2.0
                             : (input.max_diameter_m * overlap_ratio / 2.0);
  double single_prop_area = M_PI * std::pow(prop_radius_m, 2);
  double total_prop_area = nr_disk * single_prop_area;

  // Minimum theoretical takeoff mass assuming structure weight floor
  double min_theoretical_mass =
      input.payload_kg / (1.0 - Physics::MIN_STRUCTURAL_FRACTION);
  double min_thrust_n = min_theoretical_mass * Physics::GRAVITY_MS2;

  // 1D Momentum theory induced downwash velocity calculation
  double induced_velocity =
      std::sqrt(min_thrust_n / (2.0 * rho * total_prop_area));
  double downwash_velocity = 2.0 * induced_velocity;
  double limit = active_role ? active_role->getMaxInducedVelocityMs()
                             : Physics::MAX_INDUCED_VELOCITY_MS;

  if (downwash_velocity > limit) {
    err = "Disk Loading Failure: Downwash velocity " +
          std::to_string(downwash_velocity) + " m/s exceeds limit " +
          std::to_string(limit) +
          " m/s. High risk of Vortex Ring State. Increase drone diameter.";
    return false;
  }
  return true;
}

/**
 * Validates that required propeller blade tip speeds do not violate acoustic limits.
 * Uses a minimum total takeoff weight basis to compute required tip speed.
 */
bool FeasibilityGate::checkAeroacousticMachLimit(
    const EngineData::UserInput &input, const EngineData::UserLocks &locks,
    double rho, double speed_of_sound, std::string &err) {
  int nr_mach = input.num_rotors > 0 ? input.num_rotors : 4;
  int n_eff = input.coaxial_layout ? (nr_mach / 2) : nr_mach;
  double overlap_ratio = M_SQRT1_2;
  if (n_eff >= 3) {
    overlap_ratio = 0.95 * std::sin(M_PI / n_eff);
  }
  double prop_radius_m = (locks.locked_diameter_in > 0.0)
                             ? (locks.locked_diameter_in * 0.0254) / 2.0
                             : (input.max_diameter_m * overlap_ratio / 2.0);
  double single_prop_area = M_PI * std::pow(prop_radius_m, 2);
  double total_prop_area = nr_mach * single_prop_area;

  // Use minimum theoretical total mass instead of raw payload
  double min_total_mass = input.payload_kg / (1.0 - Physics::MIN_STRUCTURAL_FRACTION);
  double min_thrust_n = min_total_mass * Physics::GRAVITY_MS2;
  double assumed_max_ct = 0.15; // Upper bound thrust coefficient
  double min_v_tip =
      std::sqrt(min_thrust_n / (assumed_max_ct * rho * total_prop_area));
  double mach_number = min_v_tip / speed_of_sound;

  if (mach_number > Physics::MAX_TIP_MACH) {
    err = "Aeroacoustic Failure: Required tip speed exceeds Mach " +
          std::to_string(Physics::MAX_TIP_MACH) +
          ". Propellers will enter supersonic drag compressibility.";
    return false;
  }
  return true;
}

/**
 * Validates battery energy capacity limits against the physical Ragone envelope.
 * Assumes a battery fraction limit of 45% to prevent structural sizing runaway.
 */
bool FeasibilityGate::checkRagoneEnvelope(
    const EngineData::UserInput &input,
    const EngineData::BatteryChemistry &chem, double rho, std::string &err) {
  int nr = input.num_rotors > 0 ? input.num_rotors : 4;
  int n_eff = input.coaxial_layout ? (nr / 2) : nr;
  double overlap_ratio = M_SQRT1_2;
  if (n_eff >= 3) {
    overlap_ratio = 0.95 * std::sin(M_PI / n_eff);
  }
  double prop_radius_m = input.max_diameter_m * overlap_ratio / 2.0;
  double total_prop_area = nr * M_PI * std::pow(prop_radius_m, 2);

  double min_mass = input.payload_kg / (1.0 - Physics::MIN_STRUCTURAL_FRACTION);
  double min_thrust = min_mass * Physics::GRAVITY_MS2;
  double ideal_hover_power_w =
      std::sqrt(std::pow(min_thrust, 3) / (2.0 * rho * total_prop_area));
  double total_time_hours =
      (input.t_hover_s + input.t_forward_s + input.t_climb_s) / 3600.0;
  double min_energy_req_wh = ideal_hover_power_w * total_time_hours;

  // 1. Solve for capacity energy mass: m_energy = Energy / Specific_Energy
  double m_batt_energy = min_energy_req_wh / chem.e_spec_max_wh_kg;
  
  // 2. Solve for peak climb power mass: m_power = Power / Specific_Power
  double min_climb_power_w = min_thrust * input.v_climb_ms;
  double m_batt_power = min_climb_power_w / chem.p_spec_max_w_kg;
  double required_batt_mass = std::max(m_batt_energy, m_batt_power);

  double theoretical_total_mass = (input.payload_kg + required_batt_mass) /
                                  (1.0 - Physics::MIN_STRUCTURAL_FRACTION);
  double batt_fraction = required_batt_mass / theoretical_total_mass;

  // 45% physical battery fraction limit
  if (batt_fraction > 0.45) {
    err = "Ragone Envelope Failure: The specific energy of " +
          chem.chemistry_name +
          " cannot sustain this payload for the requested time. Battery mass "
          "fraction (" +
          std::to_string(static_cast<int>(batt_fraction * 100)) +
          "%) exceeds the 45% physical limit for multicopters.";
    return false;
  }
  return true;
}

/**
 * Computes maximum theoretical flight time and maximum payload at the Pareto boundary.
 */
void FeasibilityGate::computeParetoFrontier(
    const EngineData::UserInput &input,
    const EngineData::BatteryChemistry &chem, double rho,
    FeasibilityResult &result) {
  int nr = input.num_rotors > 0 ? input.num_rotors : 4;
  int n_eff = input.coaxial_layout ? (nr / 2) : nr;
  double overlap_ratio = M_SQRT1_2;
  if (n_eff >= 3) {
    overlap_ratio = 0.95 * std::sin(M_PI / n_eff);
  }
  double prop_radius_m = input.max_diameter_m * overlap_ratio / 2.0;
  // scale disk area with actual rotor count
  double max_area = nr * M_PI * std::pow(prop_radius_m, 2);
  double min_mass = input.payload_kg / (1.0 - Physics::MIN_STRUCTURAL_FRACTION);
  double min_thrust = min_mass * Physics::GRAVITY_MS2;
  double ideal_power_w =
      std::sqrt(std::pow(min_thrust, 3) / (2.0 * rho * max_area));

  // Solve max theoretical flight duration assuming max battery mass limit
  double max_batt_mass = min_mass;
  double max_energy_wh = max_batt_mass * chem.e_spec_max_wh_kg;
  result.pareto_max_time_s = (max_energy_wh / ideal_power_w) * 3600.0;

  double total_time_h =
      (input.t_hover_s + input.t_forward_s + input.t_climb_s) / 3600.0;
  double assumed_max_batt_wh = 5.0 * chem.e_spec_max_wh_kg;
  double max_allowable_power = assumed_max_batt_wh / total_time_h;
  double max_total_mass =
      std::pow((std::pow(max_allowable_power, 2) * 2.0 * rho * max_area) /
                   std::pow(Physics::GRAVITY_MS2, 3),
               1.0 / 3.0);
  result.pareto_max_payload =
      max_total_mass * (1.0 - Physics::MIN_STRUCTURAL_FRACTION) - 5.0;
  if (result.pareto_max_payload < 0)
    result.pareto_max_payload = 0;
}

} // namespace Module1