#ifndef THERMODYNAMICS_HPP
#define THERMODYNAMICS_HPP

#include "../utils/DataStructures.hpp"
#include <vector>
#include <string>

namespace EngineData {
    class IPayloadRole;
}

namespace Module3 {

    // Output package summarizing transient thermodynamics simulation results.
    struct ThermodynamicResult {
        double required_capacity_mah = 0.0;  // Minimum battery capacity needed to survive the mission (mAh)
        double max_battery_temp_c = 0.0;     // Peak battery pack temperature reached (C)
        bool survived = false;               // True if battery survived without voltage collapse or overheating
        std::string failure_reason = "";     // Diagnostic details if survived is false
        double survived_time_s = 0.0;        // Total duration the simulation ran before failure or completion (s)
    };

    // Physics solver running transient battery thermal and discharge simulations.
    // Models cell internal resistances, ESC switching losses, and natural convection cooling.
    class ThermodynamicsSolver {
    public:
        // Runs transient mission simulation to verify battery capacity requirements and thermals.
        static ThermodynamicResult simulateMissionThermodynamics(
            const std::vector<double>& phase_durations_s,
            const std::vector<double>& phase_mechanical_powers_w,
            const EngineData::BatteryChemistry& chem,
            const EngineData::ESCHardware& esc,
            double ambient_temp_c,
            int cell_count,
            double total_capacity_mah,
            const EngineData::IPayloadRole* role = nullptr,
            double max_diameter_m = 1.0,
            int battery_cycle_count = 0,
            int num_rotors = 4,
            const std::string &role_type = "",
            const std::string &airframe_class = ""
        );

    private:
        static constexpr double PWM_FREQUENCY_HZ = 32000.0;     // ESC PWM switching frequency (Hz)
        static constexpr double CP_LITHIUM_J_KG_K = 1020.0;    // Lithium specific heat capacity (J/(kg·K))
        static constexpr double CELL_MASS_KG = 0.046;          // Reference mass of a single cell (kg)
        static constexpr double ETA_BASE  = 0.83;              // BLDC motor efficiency linear model base constant
        static constexpr double ETA_SLOPE = 0.08;              // BLDC motor efficiency slope factor

        // Computes ESC electrical efficiency as a function of throttle.
        static double getEscEfficiency(double throttle_fraction);

        // Computes brushless motor mechanical efficiency as a function of throttle.
        static double getMotorEfficiency(double throttle_fraction);

        // Evaluates open circuit voltage (Voc) dynamically using the Tremblay-Dessaint equation.
        static double getTremblayVoc(double soc, const EngineData::BatteryChemistry& chem, double c_rate = 1.0);

        // Solves for current (A) required to produce target electric power using quadratic formula.
        static double solveQuadraticCurrent(double p_req, double v_oc, double r_tot);
    };

}

#endif // THERMODYNAMICS_HPP