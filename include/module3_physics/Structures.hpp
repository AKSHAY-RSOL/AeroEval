#ifndef STRUCTURES_HPP
#define STRUCTURES_HPP

#include "../utils/DataStructures.hpp"
#include "../roles/IPayloadRole.hpp"

namespace Module3 {

    // Mechanical solver for carbon fiber airframe sizing.
    // Models arm bending stresses using Euler-Bernoulli cantilever beam formulas.
    class StructuresSolver {
    public:
        // Sizes structural arm tube dimensions and estimates frame mass.
        static double calculateFrameMass(
            double current_total_mass_kg,
            double max_diameter_m,
            double max_thrust_n,
            int num_rotors,
            const EngineData::IPayloadRole* active_role,
            const EngineData::StructuralOverrides& overrides,
            double* out_d_out_m = nullptr,
            double* out_t_wall_m = nullptr,
            bool coaxial_layout = false,
            const std::string& airframe_class = ""
        );
    };

}

#endif // STRUCTURES_HPP