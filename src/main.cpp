#include "../include/module1_feasibility/FeasibilityGate.hpp"
#include "../include/module2_kinematics/MissionProfiler.hpp"
#include "../include/module3_physics/MassIteration.hpp"
#include "../include/module4_optimizer/DiscreteMINLP.hpp"
#include "../include/module5_validation/SensitivityAnalysis.hpp"
#include "../include/roles/IPayloadRole.hpp"
#include "../include/roles/TaxonomyRoles.hpp"
#include "../include/utils/JsonParser.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

/**
 * Utility function to format propeller sizing (Diameter x Pitch) as a string.
 * Truncates trailing decimals to output clean expressions (e.g., "15x5" or "10.5x4.5").
 */
static std::string format_prop(double d, double p) {
  std::string sd = (d == (int)d) ? std::to_string((int)d)
                                 : std::to_string(d).substr(0, 4);
  std::string sp = (p == (int)p) ? std::to_string((int)p)
                                 : std::to_string(p).substr(0, 4);
  if (sd.find('.') != std::string::npos) {
    while (sd.back() == '0')
      sd.pop_back();
    if (sd.back() == '.')
      sd.pop_back();
  }
  if (sp.find('.') != std::string::npos) {
    while (sp.back() == '0')
      sp.pop_back();
    if (sp.back() == '.')
      sp.pop_back();
  }
  return sd + "x" + sp;
}

namespace EngineData {
/**
 * Factory pattern creating decorator payload role objects based on user inputs.
 * 
 * Rationale:
 * Encapsulates role-specific behaviors (e.g., mass shedding schedules, continuous auxiliary power, 
 * aerodynamic drag area penalties, and target thrust margins) using polymorphic classes inheriting from IPayloadRole.
 */
std::unique_ptr<IPayloadRole> RoleFactory(const UserInput &input) {
  std::string t = input.payload_role.type;

  if (t == "delivery") {
    // Sizing dropped cargo mass values (defaults to 80% of payload if unspecified)
    double drop_mass = (input.payload_role.delivery_drop_mass_kg >= 0.0)
                           ? input.payload_role.delivery_drop_mass_kg
                           : input.payload_kg * 0.80;
    return std::make_unique<DeliveryRole>(input.payload_role, drop_mass);
  }
  if (t == "agriculture")
    return std::make_unique<AgricultureRole>(
        input.payload_role, input.payload_kg,
        input.t_forward_s + input.t_hover_s + input.t_climb_s);
  if (t == "mapping")
    return std::make_unique<MappingRole>(input.payload_role);
  if (t == "racing")
    return std::make_unique<RacingRole>(input.payload_role);
  if (t == "inspection")
    return std::make_unique<InspectionRole>(input.payload_role);
  // Default fallback to Standard Cinematography / Imaging
  return std::make_unique<CinematographyRole>(input.payload_role);
}
} // namespace EngineData

// Struct to store pipeline execution results for comparison tables and output json files
struct RunResult {
  bool success;
  std::string error_stage;
  std::string error_message;
  std::vector<std::string> suggested_fixes;
  double total_mass;
  double frame_mass;
  double battery_mass;
  double capacity_mah;
  double max_temp;
  double prop_diameter;
  double prop_pitch;
  double kv_target;
  double kv_min;
  double kv_max;
  double thermal_margin;
  double thrust_to_weight;
  double forward_pitch_deg;
  int cell_count;
  double arm_od_mm;
  double arm_wall_mm;
  bool flight_time_was_reduced = false;
  double flight_time_multiplier = 1.0;
  // Actual nominal cell voltage from the selected chemistry. Used for output voltage/energy
  // reporting instead of a hardcoded 3.7 V (which is wrong for Li-ion NMC/NCA, LiFePO4, etc.).
  double nominal_cell_voltage = 3.7;
  double approved_hover_s = 0.0;
  double approved_forward_s = 0.0;
  double approved_climb_s = 0.0;
  double discrete_j_score = 0.0;
  double ideal_kv = 0.0;
  double ideal_motor_mass_g = 0.0;
  double pareto_max_time_s = -1.0;
  double pareto_max_payload = -1.0;
  double physical_capacity_mah = 0.0;
  double cruise_speed_ms = 0.0;

  // detailed component masses
  double motor_mass_kg = 0.0;
  double wiring_mass_kg = 0.0;
  double esc_mass_kg = 0.0;
  double complexity_mass_kg = 0.0;
  double avionics_mass_kg = 0.0;
  double role_aux_mass_kg = 0.0;
};

/**
 * Orchestrates the execution of the five sizing and validation modules in sequence.
 * 
 * Pipeline Execution Stages:
 * 1. Module 1 (Feasibility Gate): Pre-checks disk loading (VRS), tip speed (Mach), and energy densities (Ragone).
 * 2. Module 2 (Kinematics Profiler): Sets up mission state evaluators to solve flight forces.
 * 3. Module 3 (Mass Iteration Engine): Runs the Banach fixed-point loop to solve converged empty weight.
 * 4. Module 4 (Discrete Optimizer): combinatorial search for optimal matching COTS props and motor KV.
 * 5. Module 5 (Sensitivity validation): Runs transient re-simulations of COTS selections to verify safety.
 */
RunResult runPipeline(const EngineData::UserInput &input,
                      const EngineData::UserLocks &locks,
                      const EngineData::BatteryChemistry &chem,
                      const EngineData::ESCHardware &esc,
                      const EngineData::COTS_Sets &cots,
                      const EngineData::IPayloadRole *active_role) {
  RunResult out{};
  out.success = false;

  // 1. Feasibility Check
  Module1::FeasibilityResult m1 = Module1::FeasibilityGate::evaluateMission(
      input, locks, chem, active_role);
  out.pareto_max_time_s = m1.pareto_max_time_s;
  out.pareto_max_payload = m1.pareto_max_payload;
  if (!m1.is_possible) {
    out.error_stage = "Module 1: Feasibility Gate";
    out.error_message = m1.fail_reason;
    return out;
  }

  // 2. Kinematics Profiling
  EngineData::UserInput adjusted_input = input;
  Module2::MissionEvaluator kinematics(m1.env_density_rho, adjusted_input, active_role);

  // 3. Mass Iteration Solver Loop
  EngineData::ContinuousDroneState ideal =
      Module3::MassIterationEngine::convergeDroneMass(
          input, locks, chem, esc, kinematics, active_role, m1.env_density_rho);
  if (!ideal.is_valid) {
    out.error_stage = "Module 3: Mass Iteration";
    out.error_message = ideal.error_message.empty()
                            ? "Mass iteration failed or goals relaxed too much."
                            : ideal.error_message;
    out.suggested_fixes = ideal.suggested_fixes;
    return out;
  }

  // Update kinematics if cruise speed was optimized
  if (input.evaluate_marketing_limits && ideal.cruise_speed_ms > 0.0) {
    adjusted_input.v_forward_ms = ideal.cruise_speed_ms;
    kinematics = Module2::MissionEvaluator(m1.env_density_rho, adjusted_input, active_role);
  }

  // 4. MINLP Discrete COTS Optimization Search
  Module4::DiscreteHardware discrete =
      Module4::DiscreteOptimizer::optimizeHardware(
          ideal, locks, cots, chem, m1.env_speed_of_sound, m1.env_density_rho,
          input.aero_overrides, active_role);
  if (!discrete.is_valid) {
    out.error_stage = "Module 4: Discrete Optimizer (MINLP)";
    out.error_message = discrete.error_message;
    out.suggested_fixes = discrete.suggested_fixes;
    return out;
  }

  // 5. Hardware-In-The-Loop (HITL) Validation Re-Simulation
  Module5::FinalValidationEnvelope final_env =
      Module5::ValidationEngine::validateAndGenerateEnvelope(
          ideal, discrete, input, chem, esc, kinematics, active_role,
          m1.env_density_rho);
  if (!final_env.system_validated) {
    out.error_stage = "Module 5: Hardware-In-The-Loop Validation";
    out.error_message = final_env.validation_message;
    out.suggested_fixes = final_env.suggested_fixes;
    return out;
  }

  // Pack output structure
  out.success = true;
  out.nominal_cell_voltage = chem.v_nom_cell;
  out.total_mass = ideal.total_mass_kg;
  out.frame_mass = ideal.frame_mass_kg;
  out.battery_mass = ideal.battery_mass_kg;
  out.motor_mass_kg = ideal.motor_mass_kg;
  out.wiring_mass_kg = ideal.wiring_mass_kg;
  out.esc_mass_kg = ideal.esc_mass_kg;
  out.complexity_mass_kg = ideal.complexity_mass_kg;
  out.avionics_mass_kg = ideal.avionics_mass_kg;
  out.role_aux_mass_kg = ideal.role_aux_mass_kg;
  out.capacity_mah = final_env.final_battery_capacity_mah;
  out.physical_capacity_mah = ideal.chosen_battery_capacity_mah;
  out.max_temp = final_env.system_validated ? final_env.max_battery_temp_c : ideal.max_motor_temp_c;
  out.prop_diameter = final_env.recommended_prop_diameter_in;
  out.prop_pitch = final_env.recommended_prop_pitch_in;
  out.kv_target = final_env.motor_kv_target;
  out.kv_min = final_env.motor_kv_min;
  out.kv_max = final_env.motor_kv_max;
  out.thermal_margin = final_env.thermal_margin_c;
  out.thrust_to_weight = final_env.thrust_to_weight_ratio;
  out.forward_pitch_deg = final_env.forward_pitch_angle_deg;
  out.cell_count = final_env.converged_cell_count;
  out.arm_od_mm = ideal.arm_outer_diameter_m * 1000.0;
  out.arm_wall_mm = ideal.arm_wall_thickness_m * 1000.0;
  out.flight_time_was_reduced = ideal.flight_time_was_reduced;
  out.flight_time_multiplier = ideal.flight_time_multiplier;
  out.cruise_speed_ms = (ideal.cruise_speed_ms > 0.0) ? ideal.cruise_speed_ms : adjusted_input.v_forward_ms;
  
  EngineData::MissionPhase hover = kinematics.getHoverState(ideal.total_mass_kg);
  EngineData::MissionPhase climb = kinematics.getClimbState(ideal.total_mass_kg);
  EngineData::MissionPhase forward = kinematics.getForwardState(ideal.total_mass_kg);
  
  out.approved_hover_s = final_env.system_validated ? (hover.duration_s * ideal.flight_time_multiplier) : 0.0;
  out.approved_forward_s = final_env.system_validated ? (forward.duration_s * ideal.flight_time_multiplier) : 0.0;
  out.approved_climb_s = final_env.system_validated ? (climb.duration_s * ideal.flight_time_multiplier) : 0.0;
  out.discrete_j_score = discrete.j_score;
  out.ideal_kv = ideal.ideal_motor_kv;
  out.ideal_motor_mass_g = ideal.ideal_motor_mass_g;

  return out;
}

/**
 * Writes computed comparison results back to the output sizing JSON file.
 * Serves as the database export interface loaded by the Next.js frontend actions.
 */
void writeSizingOutput(const std::string &filepath,
                       const EngineData::UserInput &input,
                       const EngineData::UserLocks &locks,
                       const RunResult &r1,
                       const RunResult &r2) {
  nlohmann::json j;
  j["success"] = r1.success && r2.success;

  // Compile failure trace reports if runs crashed or hit design boundaries
  nlohmann::json failures = nlohmann::json::array();
  if (!r1.success) {
    nlohmann::json f;
    f["run"] = "Dead Weight";
    f["stage"] = r1.error_stage;
    f["reason"] = r1.error_message;
    if (r1.pareto_max_time_s >= 0.0) {
      f["pareto_max_time_s"] = r1.pareto_max_time_s;
    }
    if (r1.pareto_max_payload >= 0.0) {
      f["pareto_max_payload"] = r1.pareto_max_payload;
    }
    failures.push_back(f);
  }
  if (!r2.success) {
    nlohmann::json f;
    f["run"] = "Custom Role";
    f["stage"] = r2.error_stage;
    f["reason"] = r2.error_message;
    if (r2.pareto_max_time_s >= 0.0) {
      f["pareto_max_time_s"] = r2.pareto_max_time_s;
    }
    if (r2.pareto_max_payload >= 0.0) {
      f["pareto_max_payload"] = r2.pareto_max_payload;
    }
    failures.push_back(f);
  }
  if (input.aero_overrides.figure_of_merit_isolated > 0.0 &&
      input.aero_overrides.figure_of_merit_mode != "isolated") {
    nlohmann::json f;
    f["run"] = "Input Warning";
    f["stage"] = "Aerodynamic Overrides";
    f["reason"] = "figure_of_merit_isolated is set, but figure_of_merit_mode is not set to 'isolated'. This isolated override will be silently ignored unless figure_of_merit_mode is set to 'isolated'.";
    failures.push_back(f);
  }
  j["failures"] = failures;

  // Helper lambda to structure metrics to matching JSON output format
  auto format_res_json = [&input](const RunResult &r) -> nlohmann::json {
    if (!r.success)
      return nullptr;

    nlohmann::json res;
    res["mass"] = r.total_mass;
    res["total_mass_kg"] = r.total_mass;
    res["frame"] = r.frame_mass;
    res["frame_mass_kg"] = r.frame_mass;
    res["battery"] = r.battery_mass;
    res["battery_mass_kg"] = r.battery_mass;
    res["motor_mass_kg"] = r.motor_mass_kg;
    res["motorMassKg"] = r.motor_mass_kg;
    res["wiring_mass_kg"] = r.wiring_mass_kg;
    res["wiringMassKg"] = r.wiring_mass_kg;
    res["esc_mass_kg"] = r.esc_mass_kg;
    res["escMassKg"] = r.esc_mass_kg;
    res["complexity_mass_kg"] = r.complexity_mass_kg;
    res["complexityMassKg"] = r.complexity_mass_kg;
    res["avionics_mass_kg"] = r.avionics_mass_kg;
    res["avionicsMassKg"] = r.avionics_mass_kg;
    res["role_aux_mass_kg"] = r.role_aux_mass_kg;
    res["roleAuxMassKg"] = r.role_aux_mass_kg;
    // BUGFIX: report energy/voltage using the chemistry's actual nominal cell voltage,
    // not a hardcoded 3.7 V (wrong for Li-ion NMC/NCA ~3.6, LiFePO4 ~3.2, etc.).
    res["energy"] = r.capacity_mah * (r.cell_count * r.nominal_cell_voltage) / 1000.0;
    res["battery_capacity_mah"] = r.capacity_mah;
    res["physicalCapacity"] = r.physical_capacity_mah;
    res["physical_capacity_mah"] = r.physical_capacity_mah;
    res["temp"] = r.max_temp;
    res["max_temp_c"] = r.max_temp;
    res["prop"] = format_prop(r.prop_diameter, r.prop_pitch);
    res["prop_dimensions"] = format_prop(r.prop_diameter, r.prop_pitch);
    res["targetKv"] = r.kv_target;
    res["motor_kv_target"] = r.kv_target;
    res["shopping"] =
        std::to_string((int)r.kv_min) + " - " + std::to_string((int)r.kv_max);
    res["motor_kv_shopping"] =
        std::to_string((int)r.kv_min) + " - " + std::to_string((int)r.kv_max);
    res["approvedHover"] = r.approved_hover_s;
    res["approved_hover_time_s"] = r.approved_hover_s;
    res["approvedForward"] = r.approved_forward_s;
    res["approved_forward_time_s"] = r.approved_forward_s;
    res["approvedClimb"] = r.approved_climb_s;
    res["approved_climb_time_s"] = r.approved_climb_s;
    res["cellCount"] = r.cell_count;
    res["cell_count"] = r.cell_count;
    res["thermalMargin"] = r.thermal_margin;
    res["thermal_margin_c"] = r.thermal_margin;
    res["thrustToWeight"] = r.thrust_to_weight;
    res["thrust_to_weight_ratio"] = r.thrust_to_weight;
    res["forwardPitch"] = r.forward_pitch_deg;
    res["forward_pitch_angle_deg"] = r.forward_pitch_deg;
    res["voltage"] = r.cell_count * r.nominal_cell_voltage;
    res["voltage_v"] = r.cell_count * r.nominal_cell_voltage;
    res["armOd"] = r.arm_od_mm;
    res["arm_outer_diameter_mm"] = r.arm_od_mm;
    res["armWall"] = r.arm_wall_mm;
    res["arm_wall_thickness_mm"] = r.arm_wall_mm;
    res["jScore"] = r.discrete_j_score;
    res["j_score"] = r.discrete_j_score;
    res["idealKv"] = r.ideal_kv;
    res["ideal_motor_kv"] = r.ideal_kv;
    res["idealMotorMassG"] = r.ideal_motor_mass_g;
    res["ideal_motor_mass_g"] = r.ideal_motor_mass_g;
    res["optimalSpeedMs"] = r.cruise_speed_ms;
    res["optimal_speed_ms"] = r.cruise_speed_ms;
    res["num_rotors"] = input.num_rotors > 0 ? input.num_rotors : 4;
    if (r.pareto_max_time_s >= 0.0) {
      res["pareto_max_time_s"] = r.pareto_max_time_s;
    }
    if (r.pareto_max_payload >= 0.0) {
      res["pareto_max_payload"] = r.pareto_max_payload;
    }
    return res;
  };

  j["baseline"] = format_res_json(r1);
  j["custom"] = format_res_json(r2);

  // Compile goal relaxation logs (warning trackers indicating when target runtimes were auto-reduced)
  if (r1.success && r1.flight_time_was_reduced) {
    nlohmann::json rel;
    rel["requestedHover"] = std::to_string((int)input.t_hover_s);
    rel["requestedForward"] = std::to_string((int)input.t_forward_s);
    rel["requestedClimb"] = std::to_string((int)input.t_climb_s);
    rel["isBatteryLocked"] = locks.is_battery_locked || (locks.locked_capacity_mah > 0.0);
    j["relaxation1"] = rel;
  } else {
    j["relaxation1"] = nullptr;
  }

  if (r2.success && r2.flight_time_was_reduced) {
    nlohmann::json rel;
    rel["requestedHover"] = std::to_string((int)input.t_hover_s);
    rel["requestedForward"] = std::to_string((int)input.t_forward_s);
    rel["requestedClimb"] = std::to_string((int)input.t_climb_s);
    rel["isBatteryLocked"] = locks.is_battery_locked || (locks.locked_capacity_mah > 0.0);
    j["relaxation2"] = rel;
  } else {
    j["relaxation2"] = nullptr;
  }

  std::ofstream file(filepath);
  if (file.is_open()) {
    file << j.dump(4);
  }
}

/**
 * Main application entry point orchestrating comparative sizing simulation runs.
 * 
 * Process Steps:
 * 1. Configuration Hydration: Parses active user input files, chemistry tables, and COTS databases.
 * 2. Run 1 (Baseline - Standard Hover Sizing): Runs pipeline without role-specific drag, power, 
 *    or mass dynamics ("Dead Weight").
 * 3. Run 2 (Custom Role - Physics-Driven Sizing): Invokes role-specific modifications 
 *    (e.g., active structural drops, agility margins, downwash offsets).
 * 4. Comparative Printouts: Outputs comparison table detailing weights, sizes, and templates.
 * 5. Exports results to JSON for frontend dashboard rendering.
 */
int main() {
  std::cout << "====================================================\n";
  std::cout << "  ACADEMIC DRONE SIZING & VALIDATION COMPARATIVE RUN  \n";
  std::cout << "====================================================\n\n";

  try {
    // Load data profiles
    EngineData::UserInput input =
        Utils::JsonParser::parseUserInput("data/user_input.json");
    EngineData::UserLocks locks =
        Utils::JsonParser::parseUserLocks("data/user_input.json");
    EngineData::BatteryChemistry chem = Utils::JsonParser::parseChemistry(
        "data/chemistry_profiles.json", input.battery_chemistry);
    EngineData::ESCHardware esc =
        Utils::JsonParser::parseESCHardware("data/esc_profiles.json");
    EngineData::COTS_Sets cots =
        Utils::JsonParser::parseCOTSDatabase("data/cots_database.json");

    const EngineData::IPayloadRole *baseline_role = nullptr;
    std::unique_ptr<EngineData::IPayloadRole> custom_role =
        EngineData::RoleFactory(input);

    std::cout << "[SIMULATING] Run 1: " << input.payload_kg
              << "kg 'Dead Weight' (Standard Hover Math)...\n";
    EngineData::UserInput baseline_input = input;
    if (input.baseline_v_forward_ms > 0.0) {
        baseline_input.v_forward_ms = input.baseline_v_forward_ms;
    }
    if (input.baseline_t_forward_s > 0.0) {
        baseline_input.t_forward_s = input.baseline_t_forward_s;
    }
    RunResult r1 = runPipeline(baseline_input, locks, chem, esc, cots, baseline_role);

    std::cout << "[SIMULATING] Run 2: " << input.payload_kg << "kg '"
              << input.payload_role.type
              << " Role' (Active Aero/Thermo/Torque)...\n\n";
    RunResult r2 =
        runPipeline(input, locks, chem, esc, cots, custom_role.get());

    if (!r1.success || !r2.success) {
      std::cerr << "\n====================================================\n";
      std::cerr << "                [PIPELINE FAILURE]                  \n";
      std::cerr << "====================================================\n";

      auto print_error = [](const std::string &run_name, const RunResult &r) {
        if (!r.success) {
          std::cerr << ">>> " << run_name << " FAILED\n";
          std::cerr << "    STAGE  : " << r.error_stage << "\n";
          std::cerr << "    REASON : " << r.error_message << "\n";
          if (!r.suggested_fixes.empty()) {
            std::cerr << "    SUGGESTED FIXES:\n";
            for (size_t i = 0; i < r.suggested_fixes.size(); ++i)
              std::cerr << "      " << (i + 1) << ". " << r.suggested_fixes[i]
                        << "\n";
          }
          std::cerr << "----------------------------------------------------\n";
        }
      };
      print_error("Run 1: Dead Weight", r1);
      print_error("Run 2: Custom Role", r2);
      writeSizingOutput("data/sizing_output.json", input, locks, r1, r2);
      return 1;
    }

    // Print goal relaxation warning triggers
    auto print_relaxation = [&](const std::string &run_name,
                                const RunResult &r) {
      if (r.flight_time_was_reduced) {
        std::cout << "\n[!] GOAL RELAXATION APPLIED TO " << run_name << "\n";
        std::cout
            << "    Thermal limit hit. Flight time was automatically scaled to "
            << std::fixed << std::setprecision(1)
            << (r.flight_time_multiplier * 100.0) << "% of requested.\n";
        std::cout << "    Approved Hover   : " << r.approved_hover_s
                  << " s  (requested: " << input.t_hover_s << " s)\n";
        std::cout << "    Approved Forward : " << r.approved_forward_s
                  << " s  (requested: " << input.t_forward_s << " s)\n";
        std::cout << "    Approved Climb   : " << r.approved_climb_s
                  << " s  (requested: " << input.t_climb_s << " s)\n";
      }
    };
    print_relaxation("Run 1 (Dead Weight)", r1);
    print_relaxation("Run 2 (Custom Role)", r2);

    // Print side-by-side analysis comparison table
    std::cout << "\n====================================================\n";
    std::cout << "          DETERMINISTIC COMPARISON ANALYSIS         \n";
    std::cout << "====================================================\n";
    std::cout << "Battery Chemistry : " << chem.chemistry_name << "\n";
    std::cout << std::left << std::setw(32) << "Parameter" << std::setw(20)
              << "Run 1: Dead Weight" << std::setw(20) << "Run 2: Custom Role"
              << "\n";
    std::cout << "----------------------------------------------------\n";
    std::cout << std::fixed << std::setprecision(2);

    auto row = [](const std::string &label, double v1, double v2) {
      std::cout << std::left << std::setw(32) << label << std::setw(20) << v1
                << std::setw(20) << v2 << "\n";
    };
    auto row_i = [](const std::string &label, int v1, int v2) {
      std::cout << std::left << std::setw(32) << label << std::setw(20) << v1
                << std::setw(20) << v2 << "\n";
    };
    auto row_s = [](const std::string &label, const std::string &v1,
                    const std::string &v2) {
      std::cout << std::left << std::setw(32) << label << std::setw(20) << v1
                << std::setw(20) << v2 << "\n";
    };

    row("Total Mass (kg):", r1.total_mass, r2.total_mass);
    row("Carbon Frame Mass (kg):", r1.frame_mass, r2.frame_mass);
    row("Battery Mass (kg):", r1.battery_mass, r2.battery_mass);
    row("Drained Energy (mAh):", r1.capacity_mah, r2.capacity_mah);
    row_i("Converged Cell Count (S):", r1.cell_count, r2.cell_count);
    row("Max Battery Temp (C):", r1.max_temp, r2.max_temp);
    row("Thermal Margin (C):", r1.thermal_margin, r2.thermal_margin);
    row("Arm Outer Diameter (mm):", r1.arm_od_mm, r2.arm_od_mm);
    row("Arm Wall Thickness (mm):", r1.arm_wall_mm, r2.arm_wall_mm);
    row("Thrust-to-Weight Ratio:", r1.thrust_to_weight, r2.thrust_to_weight);
    row("Forward Pitch Angle (deg):", r1.forward_pitch_deg,
        r2.forward_pitch_deg);
    row("Approved Hover Time (s):", r1.approved_hover_s, r2.approved_hover_s);
    row("Approved Forward Time (s):", r1.approved_forward_s,
        r2.approved_forward_s);
    row("Approved Climb Time (s):", r1.approved_climb_s, r2.approved_climb_s);
    row("MINLP Objective J-Score:", r1.discrete_j_score, r2.discrete_j_score);

    std::cout << "----------------------------------------------------\n";
    std::cout << "-> SELECTED COMPONENTS:\n";
    row_s("Propeller Size (in):", format_prop(r1.prop_diameter, r1.prop_pitch),
          format_prop(r2.prop_diameter, r2.prop_pitch));
    row("Selected Target KV:", r1.kv_target, r2.kv_target);
    row_s("Shopping Bounds (KV):",
          std::to_string((int)r1.kv_min) + " - " +
              std::to_string((int)r1.kv_max),
          std::to_string((int)r2.kv_min) + " - " +
              std::to_string((int)r2.kv_max));

    // Pitch control warning limits checks
    auto warn_pitch = [](const std::string &name, double deg) {
      if (deg > 45.0) {
        std::cout << "\n[!] WARNING (" << name
                  << "): Forward pitch angle = " << std::fixed
                  << std::setprecision(1) << deg
                  << " deg exceeds 45 deg warning threshold. Drone will be difficult to "
                     "control in forward flight.\n";
      }
    };
    warn_pitch("Run 1", r1.forward_pitch_deg);
    warn_pitch("Run 2", r2.forward_pitch_deg);

    std::cout << "====================================================\n";
    writeSizingOutput("data/sizing_output.json", input, locks, r1, r2);

  } catch (const std::exception &e) {
    std::cerr << "\n[FATAL SYSTEM CRASH] Exception: " << e.what() << "\n";
    nlohmann::json err_json;
    err_json["success"] = false;
    nlohmann::json failures = nlohmann::json::array();
    nlohmann::json f;
    f["run"] = "System";
    f["stage"] = "Fatal Crash";
    f["reason"] = e.what();
    failures.push_back(f);
    err_json["failures"] = failures;
    std::ofstream file("data/sizing_output.json");
    if (file.is_open())
      file << err_json.dump(4);
    return 1;
  }

  return 0;
}