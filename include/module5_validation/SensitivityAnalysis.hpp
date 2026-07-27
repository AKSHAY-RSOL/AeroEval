#ifndef SENSITIVITY_ANALYSIS_HPP
#define SENSITIVITY_ANALYSIS_HPP

#include "../utils/DataStructures.hpp"
#include "../module4_optimizer/DiscreteMINLP.hpp"
#include "../module2_kinematics/MissionProfiler.hpp"
#include <string>

namespace EngineData {
    class IPayloadRole;
}

namespace Module5 {

    // Final validated specs envelope package returned by validation suite.
    struct FinalValidationEnvelope {
        bool system_validated = false;               // True if COTS components passed the transient validation pass
        std::string validation_message;              // Diagnostics summary of re-simulation results
        std::vector<std::string> suggested_fixes;   // Actionable hints if validation checks fail
        double recommended_prop_diameter_in = 0.0;  // Recommended propeller diameter (in)
        double recommended_prop_pitch_in = 0.0;     // Recommended propeller pitch (in)
        double motor_kv_target = 0.0;                // Target motor KV rating (RPM/V)
        double motor_kv_min = 0.0;                   // Minimum acceptable motor KV limit
        double motor_kv_max = 0.0;                   // Maximum acceptable motor KV limit
        double final_battery_capacity_mah = 0.0;     // Sized battery capacity (mAh)
        double thermal_margin_c = 0.0;               // Safety margin below thermal runaway limit (C)
        double max_battery_temp_c = 0.0;             // Peak battery temperature reached during mission (C)
        double thrust_to_weight_ratio = 0.0;         // Static thrust-to-weight ratio capability
        double forward_pitch_angle_deg = 0.0;        // Target forward cruise pitch angle (deg)
        int    converged_cell_count = 6;             // Series cell count (S)
    };

    // Validation engine validating discrete COTS components.
    // Simulates dynamic mission stages under selected motor and propeller specs.
    class ValidationEngine {
    public:
        // Runs re-simulation checks to verify safety margins and computes motor KV limits.
        static FinalValidationEnvelope validateAndGenerateEnvelope(
            const EngineData::ContinuousDroneState& ideal_state,
            const Module4::DiscreteHardware& selected_hardware,
            const EngineData::UserInput& input,
            const EngineData::BatteryChemistry& chem,
            const EngineData::ESCHardware& esc,
            const Module2::MissionEvaluator& kinematics,
            const EngineData::IPayloadRole* active_role,
            double rho
        );

    private:
        // Solves transient validation loops using discrete propeller/motor KV.
        static bool executeHardwareInTheLoop(
            double discrete_area_m2,
            const EngineData::ContinuousDroneState& ideal_state,
            const EngineData::UserInput& input,
            const EngineData::BatteryChemistry& chem,
            const EngineData::ESCHardware& esc,
            const Module2::MissionEvaluator& kinematics,
            const EngineData::IPayloadRole* active_role,
            double rho,
            int cell_count,
            double& out_max_temp,
            double& out_final_capacity
        );
    };

}

#endif // SENSITIVITY_ANALYSIS_HPP