#ifndef MISSION_PROFILER_HPP
#define MISSION_PROFILER_HPP

#include "../utils/DataStructures.hpp"
#include "../roles/IPayloadRole.hpp"
#include <vector>

namespace Module2 {

    // Evaluator resolving thrust requirements and pitch tilt vectors for each mission segment.
    class MissionEvaluator {
    private:
        double rho;                              // Air density (kg/m^3)
        double v_forward;                        // Target cruise flight speed (m/s)
        double v_climb;                          // Target vertical climb speed (m/s)
        double t_hover;                          // Total hover time (s)
        double t_climb;                          // Total climb time (s)
        double t_forward;                        // Total forward cruise time (s)

        const EngineData::IPayloadRole* active_role;           // Active payload behavior settings
        EngineData::AerodynamicOverrides aero_overrides;       // Aerodynamic overrides block

        // Solves body parasite drag forces based on frontal projection areas.
        double calculateParasiteDrag(double mass, double velocity, double drag_coefficient) const;

    public:
        // Constructor setting up the environment.
        MissionEvaluator(
            double air_density, 
            const EngineData::UserInput& input, 
            const EngineData::IPayloadRole* role = nullptr
        );

        // Resolves thrust demands during hover.
        EngineData::MissionPhase getHoverState(double current_mass) const;

        // Resolves thrust demands during climb.
        EngineData::MissionPhase getClimbState(double current_mass) const;

        // Resolves thrust and tilt angle vectors during forward cruise.
        EngineData::MissionPhase getForwardState(double current_mass) const;
    };

}

#endif