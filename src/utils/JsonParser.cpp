#include "../../include/utils/JsonParser.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <nlohmann/json.hpp> 

using json = nlohmann::json;

namespace Utils {

    /**
     * Parses and validates the main User Input JSON configuration file.
     * 
     * Parser & Validation Rationale:
     * - Nested Schema Translation: Supports legacy nested JSON formats (`drone_config` and `mission_profile` blocks) 
     *   as well as flat structures by mapping attributes dynamically.
     * - Parameter Boundary Verification: Ensures physical inputs (e.g. payload mass, max frame diameter, speeds) 
     *   stay within realistic engineering boundaries to avoid float overflows or unphysical states.
     * - Structural Class Translation: Maps structural class strings ("1", "2a", "2b", "3") to their matching 
     *   airframe classification strings (MicroAIO, ConsumerFolding, EnterpriseRugged, Agricultural).
     * - Overrides: Checks for specific battery energy capacity or airframe weight overrides.
     */
    EngineData::UserInput JsonParser::parseUserInput(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) throw std::runtime_error("Could not open User Input JSON: " + filepath);
        json j;
        file >> j; 
        EngineData::UserInput input;

        // Fallback checks for JSON nesting styles
        json drone_cfg = j.contains("drone_config") ? j["drone_config"] : j;
        json mission_prof = j.contains("mission_profile") ? j["mission_profile"] : j;

        // Payload Mass checks
        input.payload_kg = drone_cfg.value("payload_kg", j.value("payload_kg", 1.0));
        if (input.payload_kg < 0.0) throw std::runtime_error("Invalid input: payload_kg cannot be negative.");
        if (input.payload_kg > 100.0) throw std::runtime_error("Invalid input: payload_kg > 100 kg is out of scope.");

        // Maximum Frame Diameter checks
        input.max_diameter_m = drone_cfg.value("max_diameter_m", j.value("max_diameter_m", 1.0));
        if (input.max_diameter_m <= 0.05) throw std::runtime_error("Invalid input: max_diameter_m must be > 0.05 m.");
        if (input.max_diameter_m > 5.0) throw std::runtime_error("Invalid input: max_diameter_m > 5 m is out of scope.");

        // Parse forward velocity (handles string "auto" defaults)
        if (mission_prof.contains("v_forward_ms") && mission_prof["v_forward_ms"].is_string()) {
            std::string v_str = mission_prof["v_forward_ms"].get<std::string>();
            if (v_str == "auto") {
                input.v_forward_ms = 15.0;
            } else {
                try {
                    input.v_forward_ms = std::stod(v_str);
                } catch (...) {
                    input.v_forward_ms = 10.0;
                }
            }
        } else if (j.contains("v_forward_ms") && j["v_forward_ms"].is_string()) {
            std::string v_str = j["v_forward_ms"].get<std::string>();
            if (v_str == "auto") {
                input.v_forward_ms = 15.0;
            } else {
                try {
                    input.v_forward_ms = std::stod(v_str);
                } catch (...) {
                    input.v_forward_ms = 10.0;
                }
            }
        } else {
            input.v_forward_ms = mission_prof.value("v_forward_ms", j.value("v_forward_ms", 10.0));
        }
        if (input.v_forward_ms < 0.0) throw std::runtime_error("Invalid input: v_forward_ms cannot be negative.");
        if (input.v_forward_ms > 100.0) throw std::runtime_error("Invalid input: v_forward_ms > 100 m/s is out of scope.");

        input.baseline_v_forward_ms = mission_prof.value("baseline_v_forward_ms", j.value("baseline_v_forward_ms", -1.0));
        input.baseline_t_forward_s = mission_prof.value("baseline_t_forward_s", j.value("baseline_t_forward_s", -1.0));

        input.v_climb_ms = mission_prof.value("v_climb_ms", j.value("v_climb_ms", 2.0));
        if (input.v_climb_ms < 0.0) throw std::runtime_error("Invalid input: v_climb_ms cannot be negative.");

        input.t_climb_s = mission_prof.value("t_climb_s", j.value("t_climb_s", 30.0));
        if (input.t_climb_s < 0.0) throw std::runtime_error("Invalid input: t_climb_s cannot be negative.");

        input.t_forward_s = mission_prof.value("t_forward_s", j.value("t_forward_s", 300.0));
        if (input.t_forward_s < 0.0) throw std::runtime_error("Invalid input: t_forward_s cannot be negative.");

        input.t_hover_s = mission_prof.value("t_hover_s", j.value("t_hover_s", 60.0));
        if (input.t_hover_s < 0.0) throw std::runtime_error("Invalid input: t_hover_s cannot be negative.");

        double total_time = input.t_hover_s + input.t_forward_s + input.t_climb_s;
        if (total_time <= 0.0) throw std::runtime_error("Invalid input: total mission time (hover + forward + climb) must be > 0.");

        input.altitude_m = mission_prof.value("altitude_m", j.value("altitude_m", 100.0));
        if (input.altitude_m < 0.0) throw std::runtime_error("Invalid input: altitude_m cannot be negative.");
        if (input.altitude_m > 32000.0) throw std::runtime_error("Invalid input: altitude_m > 32000 m exceeds ISA model range.");

        input.ambient_temp_c = mission_prof.value("ambient_temp_c", j.value("ambient_temp_c", 25.0));
        if (input.ambient_temp_c < -60.0 || input.ambient_temp_c > 60.0)
            throw std::runtime_error("Invalid input: ambient_temp_c must be between -60 and 60 C.");

        // Sizing platform parameters
        input.battery_chemistry = drone_cfg.value("battery_chemistry", j.value("battery_chemistry", "LiPo"));
        input.nominal_cell_count = drone_cfg.value("nominal_cell_count", j.value("nominal_cell_count", 0)); 
        input.battery_cycle_count = drone_cfg.value("battery_cycle_count", j.value("battery_cycle_count", 0));
        input.num_rotors = drone_cfg.value("rotor_count", drone_cfg.value("num_rotors", j.value("rotor_count", j.value("num_rotors", 4))));
        input.coaxial_layout = drone_cfg.value("coaxial_layout", drone_cfg.value("coaxial", j.value("coaxial_layout", j.value("coaxial", false))));
        input.battery_pack_count = drone_cfg.value("battery_pack_count", j.value("battery_pack_count", 1));
        if (input.battery_pack_count < 1) input.battery_pack_count = 1;
        
        std::string structural_class = drone_cfg.value("drone_structural_class", j.value("drone_structural_class", ""));
        if (!structural_class.empty()) {
            if (structural_class == "1") {
                input.airframe_class = "MicroAIO";
            } else if (structural_class == "2a") {
                input.airframe_class = "ConsumerFolding";
            } else if (structural_class == "2b") {
                input.airframe_class = "EnterpriseRugged";
            } else if (structural_class == "3") {
                input.airframe_class = "Agricultural";
            }
        } else {
            input.airframe_class = drone_cfg.value("airframe_class", j.value("airframe_class", "ConsumerFolding"));
        }

        // Auto-promote to EnterpriseRugged if diameter indicates a commercial heavy-lift platform
        if (input.airframe_class == "ConsumerFolding" && input.max_diameter_m > 0.70) {
            input.airframe_class = "EnterpriseRugged";
        }
        
        // User-specified specific energy overrides the chemistry database (e.g. premium solid-state cells)
        double e_spec_user = drone_cfg.value("battery_specific_energy_wh_kg", 
                             drone_cfg.value("battery_specific_energy_wh_kg_override", 
                             j.value("battery_specific_energy_wh_kg", 
                             j.value("battery_specific_energy_wh_kg_override", -1.0))));
        input.battery_specific_energy_wh_kg_override = e_spec_user;
        
        // Anchored structural analysis: fixes frame mass to real empty weight, bypassing solver estimates
        input.fixed_airframe_mass_kg = drone_cfg.value("fixed_airframe_mass_kg", j.value("fixed_airframe_mass_kg", -1.0));
        input.evaluate_marketing_limits = j.value("evaluate_marketing_limits", drone_cfg.value("evaluate_marketing_limits", false));

        // Payload role parsing
        json payload_role_json;
        if (j.contains("payload_role")) {
            payload_role_json = j["payload_role"];
        } else if (drone_cfg.contains("payload_role")) {
            payload_role_json = drone_cfg["payload_role"];
        }
        
        if (!payload_role_json.is_null()) {
            input.payload_role.type = payload_role_json.value("type", "");
            input.payload_role.aux_power_w = payload_role_json.value("aux_power_w", 0.0);
            input.payload_role.added_drag_m2 = payload_role_json.value("added_drag_m2", 0.0);
            input.payload_role.cg_shift_m = payload_role_json.value("cg_shift_m", 0.0);
            
            // Advanced role-specific parameters
            input.payload_role.delivery_drop_mass_kg = payload_role_json.value("delivery_drop_mass_kg", -1.0);
            input.payload_role.delivery_drop_time_ratio = payload_role_json.value("delivery_drop_time_ratio", -1.0);
            input.payload_role.spray_rate_kg_per_s = payload_role_json.value("spray_rate_kg_per_s", -1.0);
            input.payload_role.racing_load_factor = payload_role_json.value("racing_load_factor", -1.0);
            input.payload_role.ag_max_downwash_ms = payload_role_json.value("ag_max_downwash_ms", -1.0);
            double tm_override = payload_role_json.value("thrust_margin_override", -1.0);
            if (tm_override > 1.0) {
                tm_override /= 100.0;
            }
            input.payload_role.thrust_margin_override = tm_override;
            input.payload_role.drag_area_multiplier = payload_role_json.value("drag_area_multiplier", -1.0);
            input.payload_role.integration_overhead_kg = payload_role_json.value("integration_overhead_kg", -1.0);
        }

        // Structural overrides parsing
        json struct_ovr_json;
        if (drone_cfg.contains("structural_overrides")) {
            struct_ovr_json = drone_cfg["structural_overrides"];
        } else if (j.contains("structural_overrides")) {
            struct_ovr_json = j["structural_overrides"];
        }
        
        if (!struct_ovr_json.is_null()) {
            input.structural_overrides.geometry_constant = struct_ovr_json.value("geometry_constant", -1.0);
            input.structural_overrides.wall_thickness_ratio = struct_ovr_json.value("wall_thickness_ratio", -1.0);
            input.structural_overrides.body_mass_multiplier = struct_ovr_json.value("body_mass_multiplier", -1.0);
            input.structural_overrides.num_blades = struct_ovr_json.value("num_blades", -1.0);
            
            std::string ac_str = struct_ovr_json.value("arm_config", "under_rotor");
            if (ac_str == "offset") {
                input.structural_overrides.arm_config = EngineData::ArmConfiguration::OFFSET;
            } else if (ac_str == "folding") {
                input.structural_overrides.arm_config = EngineData::ArmConfiguration::FOLDING;
            } else {
                input.structural_overrides.arm_config = EngineData::ArmConfiguration::UNDER_ROTOR;
            }
        }

        // Aerodynamic overrides parsing
        json aero_ovr_json;
        if (drone_cfg.contains("aerodynamic_overrides")) {
            aero_ovr_json = drone_cfg["aerodynamic_overrides"];
        } else if (j.contains("aerodynamic_overrides")) {
            aero_ovr_json = j["aerodynamic_overrides"];
        }
        
        input.aero_overrides.figure_of_merit = -1.0;
        input.aero_overrides.propulsive_efficiency = -1.0;
        input.aero_overrides.cd_horizontal = -1.0;
        input.aero_overrides.cd_vertical = -1.0;
        input.aero_overrides.area_scaling_factor = -1.0;
        input.aero_overrides.assumed_ct = -1.0;
        input.aero_overrides.propeller_class = "SF";
        input.aero_overrides.figure_of_merit_mode = "installed";
        input.aero_overrides.figure_of_merit_isolated = -1.0;

        if (!aero_ovr_json.is_null()) {
            input.aero_overrides.figure_of_merit = aero_ovr_json.value("figure_of_merit", -1.0);
            input.aero_overrides.propulsive_efficiency = aero_ovr_json.value("propulsive_efficiency", -1.0);
            input.aero_overrides.cd_horizontal = aero_ovr_json.value("cd_horizontal", -1.0);
            input.aero_overrides.cd_vertical = aero_ovr_json.value("cd_vertical", -1.0);
            input.aero_overrides.area_scaling_factor = aero_ovr_json.value("area_scaling_factor", -1.0);
            input.aero_overrides.assumed_ct = aero_ovr_json.value("assumed_ct", -1.0);
            input.aero_overrides.propeller_class = aero_ovr_json.value("propeller_class", "SF");
            input.aero_overrides.figure_of_merit_mode = aero_ovr_json.value("figure_of_merit_mode", "installed");
            input.aero_overrides.figure_of_merit_isolated = aero_ovr_json.value("figure_of_merit_isolated", -1.0);

            // Map aero_body_class -> area_scaling_factor (fuselage equivalent flat-plate CdA scaling).
            // The UI exposes aero_body_class as a body-streamlining selector; previously the engine
            // ignored it entirely. Only apply the preset when the user did not set an explicit
            // area_scaling_factor override (explicit numeric override always wins).
            if (input.aero_overrides.area_scaling_factor < 0.0 && aero_ovr_json.contains("aero_body_class")) {
                std::string body_class = aero_ovr_json.value("aero_body_class", "");
                if (body_class == "commercial_compact") {
                    input.aero_overrides.area_scaling_factor = 0.004; // streamlined folding (DJI Mavic class)
                } else if (body_class == "commercial_bulky") {
                    input.aero_overrides.area_scaling_factor = 0.012; // industrial platform (Matrice RTK class)
                } else if (body_class == "research_exposed") {
                    input.aero_overrides.area_scaling_factor = 0.05;  // exposed wiring / boxy DIY body
                }
            }
        }

        // Check fallback parameter paths for aerodynamic overrides nested inside structural definitions
        if (!struct_ovr_json.is_null()) {
            if (input.aero_overrides.figure_of_merit < 0.0 && struct_ovr_json.contains("figure_of_merit"))
                input.aero_overrides.figure_of_merit = struct_ovr_json.value("figure_of_merit", -1.0);
            if (input.aero_overrides.propulsive_efficiency < 0.0 && struct_ovr_json.contains("propulsive_efficiency"))
                input.aero_overrides.propulsive_efficiency = struct_ovr_json.value("propulsive_efficiency", -1.0);
            if (input.aero_overrides.cd_horizontal < 0.0 && struct_ovr_json.contains("cd_horizontal"))
                input.aero_overrides.cd_horizontal = struct_ovr_json.value("cd_horizontal", -1.0);
            if (input.aero_overrides.cd_vertical < 0.0 && struct_ovr_json.contains("cd_vertical"))
                input.aero_overrides.cd_vertical = struct_ovr_json.value("cd_vertical", -1.0);
            if (input.aero_overrides.area_scaling_factor < 0.0 && struct_ovr_json.contains("area_scaling_factor"))
                input.aero_overrides.area_scaling_factor = struct_ovr_json.value("area_scaling_factor", -1.0);
            if (input.aero_overrides.assumed_ct < 0.0 && struct_ovr_json.contains("assumed_ct"))
                input.aero_overrides.assumed_ct = struct_ovr_json.value("assumed_ct", -1.0);
        }
        
        return input;
    }

    /**
     * Parses parameter locks from configuration inputs.
     * Caches battery capacity locks, voltage (series cell) locks, COTS propeller locks, 
     * motor KV locks, and auto flight-time reduction strategies.
     */
    EngineData::UserLocks JsonParser::parseUserLocks(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) throw std::runtime_error("Could not open User Input JSON for locks: " + filepath);
        json j;
        file >> j; 
        EngineData::UserLocks locks;
        
        json locks_json;
        if (j.contains("locks")) {
            locks_json = j["locks"];
        } else if (j.contains("drone_config") && j["drone_config"].contains("locks")) {
            locks_json = j["drone_config"]["locks"];
        }
        
        if (!locks_json.is_null()) {
            locks.is_battery_locked = locks_json.value("is_battery_locked", false);
            if (locks.is_battery_locked) {
                locks.locked_capacity_mah = locks_json.value("locked_capacity_mah", 0.0);
                locks.locked_cell_count = locks_json.value("locked_cell_count", 0);
            } else {
                locks.locked_capacity_mah = 0.0;
                locks.locked_cell_count = 0;
            }

            // Check for system voltage locking if battery capacity is unlocked
            if (!locks.is_battery_locked && locks_json.value("is_voltage_locked", false)) {
                locks.locked_cell_count = locks_json.value("locked_series_cells", 0);
            }

            locks.is_prop_locked = locks_json.value("is_prop_locked", false);
            if (locks.is_prop_locked) {
                locks.locked_diameter_in = locks_json.value("locked_diameter_in", 0.0);
                locks.locked_pitch_in = locks_json.value("locked_pitch_in", 0.0);
            } else {
                locks.locked_diameter_in = 0.0;
                locks.locked_pitch_in = 0.0;
            }

            locks.is_motor_locked = locks_json.value("is_motor_locked", false);
            if (locks.is_motor_locked) {
                locks.locked_kv = locks_json.value("locked_kv", 0.0);
            } else {
                locks.locked_kv = 0.0;
            }

            locks.auto_reduce_flight_time = locks_json.value("auto_reduce_flight_time", true);
        }
        return locks;
    }

    /**
     * Searches and parses a specific chemistry profile from the battery chemistry database.
     * Extracts energy density (Wh/kg), discharge power density (W/kg), base internal resistance, 
     * and Tremblay dynamic cell parameters.
     */
    EngineData::BatteryChemistry JsonParser::parseChemistry(const std::string& filepath, const std::string& chem_name) {
        std::ifstream file(filepath);
        if (!file.is_open()) throw std::runtime_error("Could not open Chemistry JSON: " + filepath);
        json j;
        file >> j;
        for (const auto& item : j["chemistries"]) {
            if (item["name"] == chem_name) {
                EngineData::BatteryChemistry chem;
                chem.chemistry_name = item["name"];
                chem.e_spec_max_wh_kg = item["e_spec_max_wh_kg"];
                chem.p_spec_max_w_kg = item["p_spec_max_w_kg"];
                chem.r_i_base_ohms = item["r_i_base_ohms"];
                chem.v_full_cell = item["v_full_cell"];
                chem.v_nom_cell = item["v_nom_cell"];
                chem.v_exp_cell = item["v_exp_cell"];
                chem.tremblay_exp_rate = item.value("tremblay_exp_rate", 10.0);
                chem.tremblay_polarization_coeff = item.value("tremblay_polarization_coeff", 0.05);
                return chem;
            }
        }
        throw std::runtime_error("Chemistry profile not found in database: " + chem_name);
    }

    /**
     * Loads arrays of available COTS hardware options (propeller sizes, pitches, motor KVs).
     */
    EngineData::COTS_Sets JsonParser::parseCOTSDatabase(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) throw std::runtime_error("Could not open COTS JSON: " + filepath);
        json j;
        file >> j;
        EngineData::COTS_Sets cots;
        cots.prop_diameters = j["prop_diameters"].get<std::vector<double>>();
        cots.prop_pitches = j["prop_pitches"].get<std::vector<double>>();
        cots.motor_kvs = j["motor_kvs"].get<std::vector<double>>();
        return cots;
    }

    /**
     * Loads default hardware specs for ESC validation.
     */
    EngineData::ESCHardware JsonParser::parseESCHardware(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) throw std::runtime_error("Could not open ESC JSON: " + filepath);
        json j;
        file >> j;
        EngineData::ESCHardware esc;
        esc.c_oss_farads = j["c_oss_farads"];
        esc.r_ds_on_ohms = j["r_ds_on_ohms"];
        return esc;
    }

}