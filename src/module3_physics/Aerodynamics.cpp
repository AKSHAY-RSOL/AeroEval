#include "../../include/module3_physics/Aerodynamics.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace Module3 {

/**
 * Calculates the total mechanical power required by the rotor propulsion system.
 * Accounts for rotor-arm interference, tip Reynolds number, forward speed,
 * and coaxial wake penalties.
 */
double AerodynamicsSolver::calculateMechanicalPower(
    const EngineData::MissionPhase &phase, double rho, double total_prop_area,
    const EngineData::AerodynamicOverrides &overrides, double arm_width_m,
    EngineData::ArmConfiguration arm_config, int num_rotors,
    bool coaxial_layout, const std::string &role_type, const std::string &airframe_class) {
  int nr = (num_rotors > 0) ? num_rotors : 4;
  double single_prop_area = total_prop_area / static_cast<double>(nr);
  double prop_diameter_m = 2.0 * std::sqrt(single_prop_area / M_PI);

  // Downwash arm blockage factor
  double k_interference = 1.0;
  if (overrides.figure_of_merit_mode != "isolated") {
    if (arm_width_m > 0.0 && prop_diameter_m > 0.0) {
      double blockage_ratio = arm_width_m / prop_diameter_m;
      if (arm_config == EngineData::ArmConfiguration::OFFSET) {
        k_interference = 1.0 + 0.6 * blockage_ratio;
      } else if (arm_config == EngineData::ArmConfiguration::FOLDING) {
        k_interference = 1.0;
      } else {
        k_interference = 1.0 + 1.5 * blockage_ratio; // UNDER_ROTOR (highest blockage penalty)
      }
      if (k_interference > 1.15)
        k_interference = 1.15; // physical clamp for maximum possible blockage penalty
    }
  }
  double corrected_thrust = phase.thrust_req_n * k_interference;

  // Estimate RPM and advance ratio J
  double ct_val = (overrides.assumed_ct > 0.0) ? overrides.assumed_ct : 0.12;
  double thrust_per_motor = corrected_thrust / static_cast<double>(nr);
  double t_val = std::max(0.1, thrust_per_motor);
  double rps_est =
      std::sqrt(t_val / (ct_val * rho * std::pow(prop_diameter_m, 4)));
  double J_est =
      (rps_est > 0.0) ? (phase.velocity_ms / (rps_est * prop_diameter_m)) : 0.0;

  double fom = -1.0;
  if (overrides.figure_of_merit_mode == "isolated") {
    fom = (overrides.figure_of_merit_isolated > 0.0)
              ? overrides.figure_of_merit_isolated
              : -1.0;
    if (fom <= 0.0 && overrides.figure_of_merit > 0.0) {
      fom = overrides.figure_of_merit;
    }
  } else {
    fom = (overrides.figure_of_merit > 0.0) ? overrides.figure_of_merit : -1.0;
  }
  double eta_prop = (overrides.propulsive_efficiency > 0.0)
                        ? overrides.propulsive_efficiency
                        : -1.0;

  // Scale Figure of Merit based on blade tip Reynolds number if not overridden
  if (fom <= 0.0) {
    double tip_speed = rps_est * M_PI * prop_diameter_m;
    double chord_ratio = 0.10;
    if (overrides.propeller_class == "TE" ||
        overrides.propeller_class == "thin_electric" ||
        overrides.propeller_class == "THIN_ELECTRIC") {
      chord_ratio = 0.07; // Thin Electric propellers typically have narrower blades
    } else if (overrides.propeller_class == "AG" ||
               overrides.propeller_class == "agricultural" ||
               overrides.propeller_class == "AGRICULTURAL") {
      chord_ratio = 0.13; // Agricultural props are wider to handle heavier thrust distributions
    }
    double chord_m = chord_ratio * prop_diameter_m; // mean chord ratio approximation
    double mu_air = 1.789e-5; // Dynamic viscosity of air (Pa*s) at standard sea level temperature
    double Re_tip = (rho * tip_speed * chord_m) / mu_air;

    // Bohorquez fit for small rotors
    double fom_max = 0.6707; // Baseline Slow Flyer class maximum figure of merit
    if (overrides.propeller_class == "TE" ||
        overrides.propeller_class == "thin_electric" ||
        overrides.propeller_class == "THIN_ELECTRIC") {
      fom_max = 0.6147;
    }
    if (overrides.figure_of_merit_mode != "isolated") {
      double penalty = 1.46;
      std::string target_class = overrides.aero_body_class.empty() ? airframe_class : overrides.aero_body_class;
      if (target_class == "research_exposed" || target_class == "MicroAIO") {
        penalty = 1.12;
      } else if (target_class == "ConsumerFolding") {
        penalty = 1.08;
      } else if (target_class == "commercial_compact") {
        penalty = 1.08;
      } else if (target_class == "commercial_bulky" || target_class == "Agricultural" || target_class == "agricultural") {
        penalty = 1.22;
      } else if (target_class == "EnterpriseRugged") {
        penalty = 1.10;
      } else {
        if (role_type == "racing") {
          penalty = 1.12;
        } else if (role_type == "agriculture") {
          penalty = 1.15;
        } else if (role_type == "imaging" || role_type == "mapping" || role_type == "inspection") {
          penalty = 1.08;
        }
      }
      fom_max = fom_max / penalty; // Apply structural installation blockage penalty
    }
    const double FoM_min = 0.38;
    const double Re_ref = 20000.0;

    fom = fom_max * (1.0 - std::exp(-Re_tip / Re_ref));
    fom = std::max(fom, FoM_min);
  }

  // Advance ratio correction for forward flight unloads
  if (eta_prop <= 0.0) {
    double J_clamped = std::max(0.0, std::min(J_est, 1.0));
    double eta_correction = 1.0 + 1.5 * J_clamped * (1.0 - J_clamped);
    eta_prop = fom * eta_correction;
    eta_prop = std::max(fom * 0.70, std::min(0.85, eta_prop));
  }

  double mechanical_power = 0.0;
  if (phase.velocity_ms == 0.0) {
    mechanical_power = solveHoverPower(corrected_thrust, rho, total_prop_area, fom);
  } else if (phase.pitch_angle_rad == 0.0 && phase.velocity_ms > 0.0) {
    mechanical_power = solveClimbPower(corrected_thrust, phase.velocity_ms, rho,
                           total_prop_area, fom);
  } else {
    mechanical_power = solveForwardPower(corrected_thrust, phase.velocity_ms,
                             phase.pitch_angle_rad, rho, total_prop_area,
                             eta_prop);
  }

  // Coaxial wake penalty
  if (coaxial_layout) {
    mechanical_power *= 1.16;
  }

  return mechanical_power;
}

// Hover power via 1D momentum theory
double AerodynamicsSolver::solveHoverPower(double thrust, double rho,
                                           double area, double fom) {
  double induced_velocity = std::sqrt(thrust / (2.0 * rho * area));
  double ideal_power = thrust * induced_velocity;
  return ideal_power / fom;
}

// Climb power with vertical inflow adjustments
double AerodynamicsSolver::solveClimbPower(double thrust, double v_climb,
                                           double rho, double area,
                                           double fom) {
  double v_i_hover = std::sqrt(thrust / (2.0 * rho * area));
  double half_vc = v_climb / 2.0;
  double induced_velocity =
      -half_vc + std::sqrt(std::pow(half_vc, 2) + std::pow(v_i_hover, 2));
  double ideal_power = thrust * (v_climb + induced_velocity);
  return ideal_power / fom;
}

// Forward power using Glauert momentum theory and propeller tilt angle
double AerodynamicsSolver::solveForwardPower(double thrust, double v_forward,
                                             double pitch_rad, double rho,
                                             double area,
                                             double eta_propulsive) {
  double induced_velocity =
      solveGlauertNewtonRaphson(thrust, v_forward, pitch_rad, rho, area);
  double ideal_power =
      thrust * (v_forward * std::sin(pitch_rad) + induced_velocity);

  return ideal_power / eta_propulsive;
}

/**
 * Solves Glauert's induced velocity equation using Newton-Raphson.
 * E7: hardened to degrade gracefully instead of throwing in the aero hot path.
 * On an ill-conditioned derivative or non-convergence it falls back to the
 * momentum-theory hover induced velocity (a physically valid upper bound),
 * rather than propagating an exception up through the mass loop.
 */
double AerodynamicsSolver::solveGlauertNewtonRaphson(double thrust,
                                                     double v_forward,
                                                     double pitch_rad,
                                                     double rho, double area) {
  double C = thrust / (2.0 * rho * area);
  double v_i = std::sqrt(std::max(0.0, C)); // Seed with hover induced velocity
  const double TOLERANCE = 1e-6;
  const int MAX_ITERATIONS = 100;

  double v_x = v_forward * std::cos(pitch_rad);

  for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
    double v_z = v_forward * std::sin(pitch_rad) + v_i;
    double denominator = std::sqrt(v_x * v_x + v_z * v_z);
    if (denominator < 1e-9) {
      // Degenerate geometry; fall back to hover induced velocity.
      return std::sqrt(std::max(0.0, C));
    }
    double f_x = v_i - (C / denominator);
    if (std::abs(f_x) < TOLERANCE) {
      return v_i; // Convergence achieved
    }
    double derivative_denominator = std::pow(denominator, 3);
    double f_prime_x = 1.0 + ((C * v_z) / derivative_denominator);
    if (std::abs(f_prime_x) < 1e-9) {
      // Ill-conditioned derivative: damp the step instead of throwing.
      v_i = 0.5 * (v_i + std::sqrt(std::max(0.0, C)));
      continue;
    }
    double step = f_x / f_prime_x;
    v_i = v_i - step;
    if (v_i < 0.0) v_i = 0.0; // induced velocity cannot be negative
  }
  // Non-convergence fallback: momentum-theory hover induced velocity.
  return std::sqrt(std::max(0.0, C));
}

} // namespace Module3