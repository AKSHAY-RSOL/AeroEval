#ifndef PHYSICAL_CONSTANTS_HPP
#define PHYSICAL_CONSTANTS_HPP

namespace Physics {
    // Gravitational acceleration (standard sea level, m/s^2)
    constexpr double GRAVITY_MS2 = 9.80665;
    
    // Specific gas constant for dry air (J/(kg·K))
    constexpr double R_SPECIFIC_AIR = 287.058;
    
    // Heat capacity ratio (adiabatic index) of air
    constexpr double GAMMA_AIR = 1.4;
    
    // Standard sea level atmospheric pressure (Pa)
    constexpr double P0_SEA_LEVEL_PA = 101325.0;
    
    // Standard sea level temperature (Kelvin)
    constexpr double T0_SEA_LEVEL_K = 288.15;
    
    // Density of carbon fiber tubes used for structural frame sizing (kg/m^3)
    constexpr double RHO_CARBON_FIBER_KG_M3 = 1600.0;
    
    // Ultimate tensile/bending strength of standard carbon fiber tube (Pa)
    constexpr double YIELD_STRENGTH_CF_PA = 720e6;

    // Fatigue knockdown factor applied to ultimate strength (endurance-limit ratio for CFRP under cyclic rotor load)
    constexpr double FATIGUE_FACTOR_CF = 0.40;

    // Factor of safety on structural bending
    constexpr double FACTOR_OF_SAFETY_STRUCT = 1.5;

    // Fatigue-and-FoS-adjusted allowable bending stress (Pa): 720e6 * 0.40 / 1.5 = 192 MPa.
    // Derived transparently from ultimate strength rather than used raw (C3-b reconciliation).
    constexpr double ALLOWABLE_STRESS_CF_PA =
        YIELD_STRENGTH_CF_PA * FATIGUE_FACTOR_CF / FACTOR_OF_SAFETY_STRUCT;
    
    // Minimum structural mass ratio (structure mass / total mass) floor for vehicle iteration
    constexpr double MIN_STRUCTURAL_FRACTION = 0.10;
    
    // Sizing G-load multiplier applied during dynamic maneuvering load calculations
    constexpr double DYNAMIC_LOAD_FACTOR = 1.5;
    
    // Aeroacoustic limit: max propeller tip Mach number allowed before shockwaves degrade efficiency
    constexpr double MAX_TIP_MACH = 0.75;
    
    // Disk loading: maximum allowable induced air column velocity (downwash, m/s)
    constexpr double MAX_INDUCED_VELOCITY_MS = 25.0;
    
    // Thermal safety threshold for battery packs (Celsius) to prevent thermal runaway
    constexpr double MAX_THERMAL_LIMIT_C = 60.0;
    
    // Banach loop mass convergence threshold (kg)
    constexpr double MASS_CONVERGENCE_TOLERANCE_KG = 0.001;
    
    // Objective function tuning: weight penalty for thrust margin constraints mismatch
    constexpr double WEIGHT_THRUST_MARGIN = 100.0;
    
    // Objective function tuning: weight penalty for pitch-to-diameter ratio deviations
    constexpr double WEIGHT_CURRENT_EFFICIENCY = 50.0;
    
    // Objective function tuning: weight penalty for motor KV rating mismatch
    constexpr double WEIGHT_KV_DEVIATION = 10.0;
    
    // Standard high penalty boundary representing invalid/unfeasible states
    constexpr double PENALTY_INFINITY = 1e9; 
} 

#endif