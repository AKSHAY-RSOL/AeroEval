#ifndef MASS_ITERATION_HPP
#define MASS_ITERATION_HPP

#include "../utils/DataStructures.hpp"
#include "../module2_kinematics/MissionProfiler.hpp"

namespace EngineData {
    class IPayloadRole;
}

namespace Module3 {

    // Solver loop resolving vehicle total takeoff mass.
    // Iteratively solves structures, electronics, aerodynamics, and battery weights.
    class MassIterationEngine {
    public:
        // Main mass convergence solver executing Banach fixed-point loops.
        static EngineData::ContinuousDroneState convergeDroneMass(
            const EngineData::UserInput& input,
            const EngineData::UserLocks& locks,
            const EngineData::BatteryChemistry& chem,
            const EngineData::ESCHardware& esc,
            const Module2::MissionEvaluator& kinematics_evaluator,
            const EngineData::IPayloadRole* active_role,
            double rho
        );
    };

}

#endif // MASS_ITERATION_HPP