#ifndef DISCRETE_MINLP_HPP
#define DISCRETE_MINLP_HPP

#include "../utils/DataStructures.hpp"
#include <vector>

namespace EngineData {
class IPayloadRole;
}

namespace Module4 {

// Selected discrete hardware specifications matched from COTS database.
struct DiscreteHardware {
  double prop_diameter_in;   // Selected propeller diameter (in)
  double prop_pitch_in;      // Selected propeller pitch (in)
  double motor_kv;           // Selected motor KV rating (RPM/V)
  double j_score;            // Convergence objective score (lower is better)
  bool is_valid = false;     // True if a safe discrete hardware match was found
  std::string error_message; // Diagnostics details if no valid match exists
  std::vector<std::string>
      suggested_fixes; // Repair hints if optimization fails
};

// MINLP solver selecting COTS components matching ideal target sizing
// calculations.
class DiscreteOptimizer {
public:
  // Solves combinatorial search over database options to minimize objective
  // J-score.
  static DiscreteHardware
  optimizeHardware(const EngineData::ContinuousDroneState &ideal_state,
                   const EngineData::UserLocks &locks,
                   const EngineData::COTS_Sets &hardware_database,
                   const EngineData::BatteryChemistry &chem,
                   double speed_of_sound_a, double rho,
                   const EngineData::AerodynamicOverrides &aero_overrides,
                   const EngineData::IPayloadRole *active_role = nullptr);

private:
  // Computes penalty score for a candidate hardware combo.
  static double calculateObjectiveFunction(
      double test_diameter_in, double test_pitch_in, double test_kv,
      const EngineData::ContinuousDroneState &ideal_state,
      const EngineData::BatteryChemistry &chem, double speed_of_sound_a,
      double rho, const EngineData::AerodynamicOverrides &aero_overrides,
      const EngineData::IPayloadRole *active_role = nullptr);

  // Verifies if full-throttle tip speeds remain below acoustic Mach limits.
  static bool checkMachLimit(double test_diameter_in, double test_kv,
                             const EngineData::BatteryChemistry &chem,
                             double speed_of_sound_a, int cell_count);
};

} // namespace Module4

#endif // DISCRETE_MINLP_HPP