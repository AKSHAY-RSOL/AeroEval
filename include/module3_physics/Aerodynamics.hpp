#ifndef AERODYNAMICS_HPP
#define AERODYNAMICS_HPP

#include "../utils/DataStructures.hpp"

namespace Module3 {

    // Physics solver resolving propeller mechanical power requirements.
    // Models arm blockage, Reynolds-number scaled profile drag, and coaxial layout efficiency factors.
    class AerodynamicsSolver {
    public:
        // Main entry point for mechanical power calculation across all phases.
        static double calculateMechanicalPower(
            const EngineData::MissionPhase& phase,
            double rho,
            double total_prop_area,
            const EngineData::AerodynamicOverrides& overrides,
            double arm_width_m = 0.0,
            EngineData::ArmConfiguration arm_config = EngineData::ArmConfiguration::UNDER_ROTOR,
            int num_rotors = 4,
            bool coaxial_layout = false,
            const std::string &role_type = "",
            const std::string &airframe_class = ""
        );

    private:
        // Solves for hover power using 1D momentum theory and Figure of Merit.
        static double solveHoverPower(double thrust, double rho, double area, double fom);

        // Solves for vertical climb power including ascent velocity inflow adjustments.
        static double solveClimbPower(double thrust, double v_climb, double rho, double area, double fom);

        // Solves for forward flight power incorporating propeller tilt vectors.
        static double solveForwardPower(double thrust, double v_forward, double pitch_rad, double rho, double area, double eta_propulsive);

        // Solves Glauert's implicit induced velocity equation using a Newton-Raphson iteration loop.
        static double solveGlauertNewtonRaphson(double thrust, double v_forward, double pitch_rad, double rho, double area);
    };

}

#endif // AERODYNAMICS_HPP