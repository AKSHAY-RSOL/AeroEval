#ifndef FEASIBILITY_GATE_HPP
#define FEASIBILITY_GATE_HPP

#include "../constants/PhysicalConstants.hpp"
#include "../roles/IPayloadRole.hpp"
#include "../utils/DataStructures.hpp"
#include <string>
#include <tuple>


namespace Module1 {

// Output package summarizing findings from early feasibility audits.
struct FeasibilityResult {
  bool is_possible; // True if the mission satisfies basic physical parameters
  std::string fail_reason; // Summary of failure modes if is_possible is false
  std::vector<std::string>
      suggested_fixes;       // Remediation advice to bring design into envelope
  double env_density_rho;    // Air density at target altitude (kg/m^3)
  double env_speed_of_sound; // Local speed of sound (m/s)
  double pareto_max_time_s = -1.0; // Maximum theoretical flight time under current layout
  double pareto_max_payload = -1.0; // Maximum theoretical carrying payload weight (kg)
};

// Feasibility validator executing rapid early audits.
// Uses 1D kinematics, atmosphere models, and Ragone limits to check feasibility
// before running convergence solver.
class FeasibilityGate {
public:
  // Analyzes structural layouts, Mach speeds, and energy envelopes.
  static FeasibilityResult
  evaluateMission(const EngineData::UserInput &user_input,
                  const EngineData::UserLocks &locks,
                  const EngineData::BatteryChemistry &chemistry,
                  const EngineData::IPayloadRole *active_role = nullptr);

private:
  // Standard atmospheric layer pressure integration to find air density and
  // local sound speed.
  static void calculateAtmospherics(double altitude_m, double temp_c,
                                    double &out_rho, double &out_a);

  // Verifies structural center-of-gravity shifts and maximum rotor induced
  // downwash velocities.
  static bool
  checkStructuralAndDiskLoading(const EngineData::UserInput &input,
                                const EngineData::UserLocks &locks, double rho,
                                const EngineData::IPayloadRole *active_role,
                                std::string &error_msg);

  // Verifies propeller tip velocities under maximum motor RPM locks relative to
  // speed of sound.
  static bool checkAeroacousticMachLimit(const EngineData::UserInput &input,
                                         const EngineData::UserLocks &locks,
                                         double rho, double speed_of_sound,
                                         std::string &error_msg);

  // Uses a 1D hover power estimator to verify that structural/battery fractions
  // do not cause divergence.
  static bool checkRagoneEnvelope(const EngineData::UserInput &input,
                                  const EngineData::BatteryChemistry &chem,
                                  double rho, std::string &error_msg);

  // Iterates theoretical bounds to define the Pareto frontier on failure.
  static void computeParetoFrontier(const EngineData::UserInput &input,
                                    const EngineData::BatteryChemistry &chem,
                                    double rho, FeasibilityResult &result);
};

} // namespace Module1

#endif