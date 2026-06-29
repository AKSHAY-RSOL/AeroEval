#ifndef DATA_STRUCTURES_HPP
#define DATA_STRUCTURES_HPP

#include <vector>
#include <string>

namespace EngineData {

    // Arm positioning configurations relative to the rotor plane.
    enum class ArmConfiguration {
        UNDER_ROTOR,  // Structural arms run directly underneath the rotors (maximum blockage)
        OFFSET,       // Structural arms offset longitudinally/laterally
        FOLDING       // Arms fold back, minimizing frontal downwash blockage
    };

    // Specialized settings for payload roles.
    struct PayloadRoleConfig {
        std::string type;                       // Name of the role (imaging, delivery, agriculture, mapping, racing, inspection)
        double aux_power_w = 0.0;               // Auxiliary electrical power draw (W)
        double added_drag_m2 = 0.0;             // Added frontal parasite drag area (m^2)
        double cg_shift_m = 0.0;                // Center of Gravity offset (m)
        double delivery_drop_mass_kg = -1.0;    // Dropped cargo weight (kg)
        double delivery_drop_time_ratio = 0.5;  // Mission time fraction where cargo is released
        double spray_rate_kg_per_s = -1.0;      // Liquid payload depletion rate (kg/s)
        double racing_load_factor = -1.0;       // Max maneuvering load factor (G-load) override
        double ag_max_downwash_ms = -1.0;       // Downwash velocity limit (m/s) override
        double thrust_margin_override = -1.0;   // Control margin throttle override
        double drag_area_multiplier = -1.0;     // Frontal drag scaling factor override
        double integration_overhead_kg = -1.0;  // User override for gimbal/cargo integration overhead (kg)
    };

    // Overrides for default aerodynamic parameters.
    struct AerodynamicOverrides {
        double figure_of_merit = -1.0;          // Propeller static hover Figure of Merit override
        double propulsive_efficiency = -1.0;    // Forward cruise flight propulsive efficiency override
        double cd_horizontal = -1.0;            // Horizontal fuselage drag coefficient override
        double cd_vertical = -1.0;              // Vertical downwash drag coefficient override
        double area_scaling_factor = -1.0;      // Fuselage cross-sectional area scaling override
        double assumed_ct = -1.0;               // Propeller thrust coefficient Ct override
        std::string propeller_class = "SF";     // Target propeller class profile
        std::string figure_of_merit_mode = "installed"; // "installed" or "isolated"
        double figure_of_merit_isolated = -1.0; // Isolated lab-test Figure of Merit override
    };

    // Overrides for structural frame weight calculations.
    struct StructuralOverrides {
        double geometry_constant = -1.0;        // Carbon fiber tube wall bending ratio constant override
        double wall_thickness_ratio = -1.0;     // Tube inner/outer diameter ratio override
        double body_mass_multiplier = -1.0;     // Central hub weight scaling multiplier override
        double num_blades = -1.0;               // Propeller blade count override (default 2)
        ArmConfiguration arm_config = ArmConfiguration::UNDER_ROTOR; // Arm positioning layout
    };

    // Input parameters describing mission requirements.
    struct UserInput {
        double payload_kg;                      // Cargo weight (kg)
        double max_diameter_m;                  // Maximum vehicle diagonal motor-to-motor span (m)
        double v_forward_ms;                    // Forward cruise speed (m/s)
        double v_climb_ms;                      // Vertical climb rate (m/s)
        double t_climb_s;                       // Vertical climb duration (s)
        double t_forward_s;                     // Forward flight duration (s)
        double t_hover_s;                       // Hover duration (s)
        double altitude_m;                      // Cruise flight altitude (m)
        double ambient_temp_c;                  // Ambient air temperature (C)
        std::string battery_chemistry = "LiPo"; // Selected battery chemistry type
        int nominal_cell_count = 0;              // Target cell count (0 = auto-select)
        int battery_cycle_count = 0;             // Battery age cycle count
        int num_rotors = 4;                      // Target rotor count (default 4)
        int battery_pack_count = 1;              // Number of parallel battery packs (multiplies capacity and mass)
        double battery_specific_energy_wh_kg_override = -1.0; // User override for chemistry specific energy (Wh/kg); -1 = use database
        double fixed_airframe_mass_kg = -1.0;    // When > 0, bypasses structural solver and pins frame mass (anchored analysis)
        bool coaxial_layout = false;             // Coaxial configuration (e.g. X8)
        std::string airframe_class = "ConsumerFolding"; // Airframe classification
        double baseline_v_forward_ms = -1.0;    // Optional override for baseline forward speed; -1 = use v_forward_ms
        double baseline_t_forward_s = -1.0;     // Optional override for baseline forward flight time; -1 = use t_forward_s
        bool evaluate_marketing_limits = false;  // Bypass DoD limits & optimize cruise speed (V_md)
        PayloadRoleConfig payload_role;         // Payload configuration overrides
        AerodynamicOverrides aero_overrides;    // Aerodynamic parameter overrides
        StructuralOverrides structural_overrides; // Structural parameter overrides
    };

    // Electrochemical battery pack parameters.
    struct BatteryChemistry {
        std::string chemistry_name;             // Chemistry label
        double e_spec_max_wh_kg;                // Maximum specific energy density (Wh/kg)
        double p_spec_max_w_kg;                 // Peak specific power density (W/kg)
        double r_i_base_ohms;                   // Base internal resistance per cell (ohms)
        double v_full_cell;                     // Fully-charged cell terminal voltage (V)
        double v_nom_cell;                      // Nominal cell voltage (V)
        double v_exp_cell;                      // Exponential zone threshold voltage (V)
        double tremblay_exp_rate = 10.0;        // Tremblay model exponential decay rate constant
        double tremblay_polarization_coeff = 0.05; // Tremblay model polarization coefficient
    };

    // ESC hardware parameters.
    struct ESCHardware {
        double c_oss_farads;                    // Parasitic output capacitance (F)
        double r_ds_on_ohms;                    // Switching resistance (ohms)
    };

    // Single mission segment status.
    struct MissionPhase {
        double thrust_req_n;                    // Total thrust demand during segment (N)
        double velocity_ms;                     // Flight velocity (m/s)
        double duration_s;                      // Phase duration (s)
        double pitch_angle_rad;                 // Fuselage pitch angle relative to horizon (rad)
    };

    // Sizing results from continuous vehicle search.
    struct ContinuousDroneState {
        double total_mass_kg = 0.0;             // Total takeoff weight (kg)
        double frame_mass_kg = 0.0;             // Sized carbon-fiber frame weight (kg)
        double battery_mass_kg = 0.0;           // Sized battery weight (kg)
        double ideal_prop_diameter_m = 0.0;     // Target propeller diameter (m)
        double ideal_motor_kv = 0.0;            // Target motor KV rating (RPM/V)
        double required_battery_capacity_mah = 0.0; // Consumed battery capacity during mission (mAh)
        double chosen_battery_capacity_mah = 0.0; // Final battery capacity size (mAh)
        double max_motor_temp_c = 0.0;          // Peak battery temperature reached (C)
        double hover_thrust_n = 0.0;            // Sized hover thrust (N)
        double thrust_to_weight_ratio = 0.0;    // Thrust-to-weight ratio capacity
        double forward_pitch_angle_deg = 0.0;   // Fuselage tilt in forward flight (deg)
        double arm_outer_diameter_m = 0.0;      // Sized arm outer diameter (m)
        double arm_wall_thickness_m = 0.0;      // Sized arm wall thickness (m)
        double ideal_motor_mass_g = 0.0;        // Ideal motor mass (g)
        bool flight_time_was_reduced = false;   // True if goal relaxation cut flight time due to overheat
        double flight_time_multiplier = 1.0;    // Flight duration scaling factor
        int converged_cell_count = 6;           // Converged series voltage cell count (S)
        double cruise_speed_ms = 0.0;           // Solved/optimized cruise speed (m/s)
        int num_rotors = 4;                     // Rotor count passed through from UserInput
        bool is_valid = false;                  // True if mass convergence succeeded
        std::string error_message;              // Diagnostics summary on failure
        std::vector<std::string> suggested_fixes; // Sizing fixes on failure

        // Sized component mass breakdown
        double motor_mass_kg = 0.0;
        double wiring_mass_kg = 0.0;
        double esc_mass_kg = 0.0;
        double complexity_mass_kg = 0.0;
        double avionics_mass_kg = 0.0;
        double role_aux_mass_kg = 0.0;
    };

    // Database listing COTS hardware options.
    struct COTS_Sets {
        std::vector<double> prop_diameters;     // Available propeller diameters (in)
        std::vector<double> prop_pitches;       // Available propeller pitches (in)
        std::vector<double> motor_kvs;           // Available motor KV ratings (RPM/V)
    };

    // Hard parameter locks.
    struct UserLocks {
        bool is_battery_locked = false;         // True if battery capacity is frozen
        double locked_capacity_mah = 0.0;       // Locked capacity size (mAh)
        int locked_cell_count = 0;              // Locked cell count (S)
        bool is_prop_locked = false;            // True if propeller specs are frozen
        double locked_diameter_in = 0.0;        // Locked propeller diameter (in)
        double locked_pitch_in = 0.0;           // Locked propeller pitch (in)
        bool is_motor_locked = false;           // True if motor rating is frozen
        double locked_kv = 0.0;                 // Locked motor KV (RPM/V)
        bool auto_reduce_flight_time = true;    // True if flight time can relax on thermal runaway
    };

}

#endif // DATA_STRUCTURES_HPP