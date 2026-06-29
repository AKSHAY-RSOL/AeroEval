#include "../../include/module3_physics/Aerodynamics.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace Module3 {

/**
 * Calculates the total mechanical power required by the rotor propulsion system 
 * during a specific mission phase.
 * 
 * Physics & Sizing Methodology:
 * 1. Rotor-Arm Interference: Aerodynamic blockage from support arms under the high-velocity downwash
 *    induces a downward force (blockage). We apply an empirical thrust correction factor (k_interference) 
 *    based on the width-to-diameter ratio and configuration (under-rotor, offset, folding) to increase 
 *    the required thrust target: T_corrected = T_req * k_interference.
 * 2. Rotor Tip Reynolds Number Scaling: Figure of Merit (FoM) is scaled dynamically based on blade tip 
 *    operating Reynolds number (Re_tip). This represents physical scaling behavior where smaller rotors 
 *    operating at lower Re suffer higher viscous profile drag losses, following the Bohorquez-Pines empirical model.
 * 3. Propulsive Efficiency (eta_prop): Propellers unload in forward flight as the advance ratio (J) increases. 
 *    We model this using an advance-ratio-dependent interpolation peaking at J=0.5, representing the conversion 
 *    of rotational torque to forward translational thrust.
 * 4. Coaxial Layout Penalty: Coaxial configurations suffer aerodynamic slipstream losses due to the lower rotor 
 *    operating in the high-velocity contracted wake of the upper rotor. An empirical 16% power multiplier 
 *    (coaxial_layout efficiency penalty) is applied to model this aerodynamic penalty.
 */
double AerodynamicsSolver::calculateMechanicalPower(
    const EngineData::MissionPhase &phase, double rho, double total_prop_area,
    const EngineData::AerodynamicOverrides &overrides, double arm_width_m,
    EngineData::ArmConfiguration arm_config, int num_rotors,
    bool coaxial_layout, const std::string &role_type, const std::string &airframe_class) {
  int nr = (num_rotors > 0) ? num_rotors : 4;
  double single_prop_area = total_prop_area / static_cast<double>(nr);
  double prop_diameter_m = 2.0 * std::sqrt(single_prop_area / M_PI);

  // --- Rotor-Arm Interference Drag ---
  // Downwash impinging on the support arms creates downward drag (blockage),
  // requiring a thrust correction factor: T_corrected = T_req * k_interference
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

  // --- Reynolds-number dependent Figure of Merit & Propulsive Efficiency ---
  // 1. Estimate RPM from corrected thrust (per motor) to get advance ratio J
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

    // Bohorquez & Pines empirical fit for small-scale rotor viscous losses
    double fom_max = 0.6707; // Baseline Slow Flyer class maximum figure of merit
    if (overrides.propeller_class == "TE" ||
        overrides.propeller_class == "thin_electric" ||
        overrides.propeller_class == "THIN_ELECTRIC") {
      fom_max = 0.6147;
    }
    if (overrides.figure_of_merit_mode != "isolated") {
      double penalty = 1.46;
      if (airframe_class == "research_exposed" || airframe_class == "MicroAIO") {
        penalty = 1.12;
      } else if (airframe_class == "ConsumerFolding") {
        penalty = 1.08;
      } else if (airframe_class == "commercial_compact") {
        penalty = 1.08;
      } else if (airframe_class == "commercial_bulky" || airframe_class == "Agricultural" || airframe_class == "agricultural") {
        penalty = 1.22;
      } else if (airframe_class == "EnterpriseRugged") {
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

  // Forward propulsive efficiency: advance-ratio-dependent interpolation.
  // FoM is a hover-only metric (defined at J=0). In forward flight the propeller
  // unloads as speed increases, so propulsive efficiency first rises above FoM
  // then falls at high advance ratios. The correction factor:
  //   eta_correction = 1 + 1.5*J*(1-J)   [peaks at J=0.5 with +37.5%]
  // is derived from a simplified actuator-disk advance-ratio correction and is
  // clamped to [fom*0.7, 0.85] for physical plausibility.
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

  // Apply aerodynamic downwash/slipstream loss penalty for coaxial configurations.
  // Lower rotors ingest high-velocity slipstream from upper rotors, yielding reduced angle of attack
  // and increased induced losses. This results in a ~16% efficiency degradation.
  if (coaxial_layout) {
    mechanical_power *= 1.16;
  }

  return mechanical_power;
}

// Resolves hover power requirements: P = T * v_i / FoM
// Based on 1D Momentum Theory where ideal induced velocity is v_i = sqrt(T / (2 * rho * A)).
// Ideal Power P_ideal = T * v_i. Actual Power is P_ideal divided by the Figure of Merit (fom).
double AerodynamicsSolver::solveHoverPower(double thrust, double rho,
                                           double area, double fom) {
  double induced_velocity = std::sqrt(thrust / (2.0 * rho * area));
  double ideal_power = thrust * induced_velocity;
  return ideal_power / fom;
}

// Resolves climb power requirements incorporating vertical inflow adjustments.
// For vertical climb, the induced velocity v_i satisfies: v_i * (v_climb + v_i) = v_i_hover^2.
// Solving the quadratic equation yields: v_i = -v_climb/2 + sqrt((v_climb/2)^2 + v_i_hover^2).
// The total climb power is T * (v_climb + v_i) scaled by FoM.
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

// Resolves forward flight power requirements incorporating propeller tilt vectors.
// Glauert's momentum theory formulation is solved first to get the forward induced velocity,
// which is then combined with the forward climb component (v_forward * sin(pitch)) to resolve 
// the total ideal induced power, scaled by the propulsive efficiency (eta_propulsive).
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
 * Solves Glauert's implicit induced velocity equation in forward flight using Newton-Raphson.
 * 
 * Equation:
 *   f(v_i) = v_i - C / sqrt( (V*cos(alpha))^2 + (V*sin(alpha) + v_i)^2 ) = 0
 *   where C = T / (2 * rho * A)
 * 
 * Newton-Raphson update step:
 *   v_i_new = v_i - f(v_i) / f'(v_i)
 *   where the derivative with respect to v_i is:
 *   f'(v_i) = 1 + (C * (V*sin(alpha) + v_i)) / ((V*cos(alpha))^2 + (V*sin(alpha) + v_i)^2)^(1.5)
 */
double AerodynamicsSolver::solveGlauertNewtonRaphson(double thrust,
                                                     double v_forward,
                                                     double pitch_rad,
                                                     double rho, double area) {
  double C = thrust / (2.0 * rho * area);
  double v_i = std::sqrt(C); // Seed with the hover induced velocity as a close initial guess
  const double TOLERANCE = 1e-6;
  const int MAX_ITERATIONS = 100;
  
  double v_x = v_forward * std::cos(pitch_rad);
  
  for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
    double v_z = v_forward * std::sin(pitch_rad) + v_i;
    double denominator = std::sqrt(v_x * v_x + v_z * v_z);
    double f_x = v_i - (C / denominator);
    if (std::abs(f_x) < TOLERANCE) {
      return v_i; // Convergence achieved
    }
    double derivative_denominator = std::pow(denominator, 3);
    double f_prime_x = 1.0 + ((C * v_z) / derivative_denominator);
    if (f_prime_x == 0.0) {
      throw std::runtime_error(
          "Newton-Raphson Derivative hit zero. Asymptote reached.");
    }
    v_i = v_i - (f_x / f_prime_x);
  }
  throw std::runtime_error(
      "Glauert equation failed to converge within MAX_ITERATIONS.");
}

} // namespace Module3