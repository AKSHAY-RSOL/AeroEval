import json
import subprocess
import os
import re
import matplotlib.pyplot as plt
import numpy as np

# Base templates for the 3 classes
LIGHT_TEMPLATE = {
    "payload_kg": 0.2,
    "max_diameter_m": 0.35,
    "v_forward_ms": 15.0,
    "v_climb_ms": 3.0,
    "t_climb_s": 15.0,
    "t_forward_s": 180.0,
    "t_hover_s": 60.0,
    "altitude_m": 100.0,
    "ambient_temp_c": 25.0,
    "battery_chemistry": "LiPo",
    "nominal_cell_count": 3,
    "locks": {
        "is_battery_locked": False,
        "locked_capacity_mah": 1500,
        "locked_cell_count": 3,
        "is_prop_locked": False,
        "locked_diameter_in": 5,
        "locked_pitch_in": 3,
        "is_motor_locked": False,
        "locked_kv": 1200,
        "auto_reduce_flight_time": True
    },
    "payload_role": {
        "type": "racing",
        "aux_power_w": 5.0,
        "added_drag_m2": 0.005,
        "cg_shift_m": 0.0,
        "delivery_drop_mass_kg": -1,
        "delivery_drop_time_ratio": -1,
        "spray_rate_kg_per_s": -1,
        "racing_load_factor": 5.0,
        "ag_max_downwash_ms": -1,
        "thrust_margin_override": -1,
        "drag_area_multiplier": -1
    },
    "aerodynamic_overrides": {
        "figure_of_merit": -1,
        "propulsive_efficiency": -1,
        "cd_horizontal": -1,
        "cd_vertical": -1,
        "area_scaling_factor": -1,
        "assumed_ct": -1
    },
    "structural_overrides": {
        "geometry_constant": -1,
        "wall_thickness_ratio": -1,
        "body_mass_multiplier": -1
    }
}

MEDIUM_TEMPLATE = {
    "payload_kg": 1.5,
    "max_diameter_m": 1.0,
    "v_forward_ms": 10.0,
    "v_climb_ms": 2.0,
    "t_climb_s": 30.0,
    "t_forward_s": 600.0,
    "t_hover_s": 120.0,
    "altitude_m": 100.0,
    "ambient_temp_c": 25.0,
    "battery_chemistry": "LiPo",
    "nominal_cell_count": 0,
    "locks": {
        "is_battery_locked": False,
        "locked_capacity_mah": 5000,
        "locked_cell_count": 6,
        "is_prop_locked": False,
        "locked_diameter_in": 15,
        "locked_pitch_in": 5,
        "is_motor_locked": False,
        "locked_kv": 400,
        "auto_reduce_flight_time": True
    },
    "payload_role": {
        "type": "imaging",
        "aux_power_w": 10.0,
        "added_drag_m2": 0.02,
        "cg_shift_m": 0.0,
        "delivery_drop_mass_kg": -1,
        "delivery_drop_time_ratio": -1,
        "spray_rate_kg_per_s": -1,
        "racing_load_factor": -1,
        "ag_max_downwash_ms": -1,
        "thrust_margin_override": -1,
        "drag_area_multiplier": -1
    },
    "aerodynamic_overrides": {
        "figure_of_merit": -1,
        "propulsive_efficiency": -1,
        "cd_horizontal": -1,
        "cd_vertical": -1,
        "area_scaling_factor": -1,
        "assumed_ct": -1
    },
    "structural_overrides": {
        "geometry_constant": -1,
        "wall_thickness_ratio": -1,
        "body_mass_multiplier": -1
    }
}

HEAVY_TEMPLATE = {
    "payload_kg": 8.0,
    "max_diameter_m": 2.0,
    "v_forward_ms": 8.0,
    "v_climb_ms": 1.5,
    "t_climb_s": 45.0,
    "t_forward_s": 900.0,
    "t_hover_s": 240.0,
    "altitude_m": 100.0,
    "ambient_temp_c": 25.0,
    "battery_chemistry": "LiPo",
    "nominal_cell_count": 12,
    "locks": {
        "is_battery_locked": False,
        "locked_capacity_mah": 16000,
        "locked_cell_count": 12,
        "is_prop_locked": False,
        "locked_diameter_in": 30,
        "locked_pitch_in": 10,
        "is_motor_locked": False,
        "locked_kv": 100,
        "auto_reduce_flight_time": True
    },
    "payload_role": {
        "type": "delivery",
        "aux_power_w": 25.0,
        "added_drag_m2": 0.05,
        "cg_shift_m": 0.0,
        "delivery_drop_mass_kg": 6.4,
        "delivery_drop_time_ratio": 0.5,
        "spray_rate_kg_per_s": -1,
        "racing_load_factor": -1,
        "ag_max_downwash_ms": -1,
        "thrust_margin_override": -1,
        "drag_area_multiplier": -1
    },
    "aerodynamic_overrides": {
        "figure_of_merit": -1,
        "propulsive_efficiency": -1,
        "cd_horizontal": -1,
        "cd_vertical": -1,
        "area_scaling_factor": -1,
        "assumed_ct": -1
    },
    "structural_overrides": {
        "geometry_constant": -1,
        "wall_thickness_ratio": -1,
        "body_mass_multiplier": -1
    }
}

# Structural test template — 15 kg payload, 2 m diameter, inspection role (6 G).
# Chosen so that Euler-Bernoulli arm stress always exceeds the MIN_STRUCTURAL_FRACTION clamp,
# making geometry_constant, wall_thickness_ratio, body_mass_multiplier, cg_shift and
# racing_load_factor all visible in the output.  This replaces the old EXTREME_HEAVY_TEMPLATE
# which was failing (Banach divergence) for most structural sweep values.
STRUCTURAL_TEST_TEMPLATE = {
    "payload_kg": 15.0,
    "max_diameter_m": 2.5,
    "v_forward_ms": 8.0,
    "v_climb_ms": 1.5,
    "t_climb_s": 30.0,
    "t_forward_s": 300.0,
    "t_hover_s": 120.0,
    "altitude_m": 100.0,
    "ambient_temp_c": 25.0,
    "battery_chemistry": "LiPo",
    "nominal_cell_count": 0,
    "locks": {
        "is_battery_locked": False,
        "locked_capacity_mah": 10000,
        "locked_cell_count": 12,
        "is_prop_locked": False,
        "locked_diameter_in": 28,
        "locked_pitch_in": 9,
        "is_motor_locked": False,
        "locked_kv": 100,
        "auto_reduce_flight_time": True
    },
    "payload_role": {
        "type": "racing",
        "aux_power_w": 15.0,
        "added_drag_m2": 0.04,
        "cg_shift_m": 0.0,
        "delivery_drop_mass_kg": -1,
        "delivery_drop_time_ratio": -1,
        "spray_rate_kg_per_s": -1,
        "racing_load_factor": 6.0,
        "ag_max_downwash_ms": -1,
        "thrust_margin_override": -1,
        "drag_area_multiplier": -1
    },
    "aerodynamic_overrides": {
        "figure_of_merit": -1,
        "propulsive_efficiency": -1,
        "cd_horizontal": -1,
        "cd_vertical": -1,
        "area_scaling_factor": -1,
        "assumed_ct": -1
    },
    "structural_overrides": {
        "geometry_constant": -1,
        "wall_thickness_ratio": -1,
        "body_mass_multiplier": 3.0
    }
}

# Sensitivity baseline template (A verified 5.0 kg payload drone with active/unclamped structures)
SENSITIVITY_BASELINE = {
    "payload_kg": 5.0,
    "max_diameter_m": 1.2,
    "v_forward_ms": 10.0,
    "v_climb_ms": 2.0,
    "t_climb_s": 30.0,
    "t_forward_s": 300.0,
    "t_hover_s": 100.0,
    "altitude_m": 100.0,
    "ambient_temp_c": 25.0,
    "battery_chemistry": "LiPo",
    "nominal_cell_count": 0,
    "locks": {
        "is_battery_locked": False,
        "locked_capacity_mah": 5000,
        "locked_cell_count": 6,
        "is_prop_locked": False,
        "locked_diameter_in": 15,
        "locked_pitch_in": 5,
        "is_motor_locked": False,
        "locked_kv": 400,
        "auto_reduce_flight_time": True
    },
    "payload_role": {
        "type": "imaging",
        "aux_power_w": 15.0,
        "added_drag_m2": 0.03,
        "cg_shift_m": 0.05,
        "delivery_drop_mass_kg": -1,
        "delivery_drop_time_ratio": -1,
        "spray_rate_kg_per_s": -1,
        "racing_load_factor": -1,
        "ag_max_downwash_ms": -1,
        "thrust_margin_override": -1,
        "drag_area_multiplier": -1
    },
    "aerodynamic_overrides": {
        "figure_of_merit": 0.65,
        "propulsive_efficiency": 0.70,
        "cd_horizontal": 1.0,
        "cd_vertical": 1.2,
        "area_scaling_factor": -1.0,
        "assumed_ct": 0.12
    },
    "structural_overrides": {
        "geometry_constant": 0.3439,
        "wall_thickness_ratio": 0.90,
        "body_mass_multiplier": 1.5
    }
}

def parse_output(stdout, stderr):
    full_text = stdout + "\n" + stderr
    parsed = {}
    
    if "STAGE  :" in full_text:
        stage_match = re.search(r"STAGE\s*:\s*(.*)", full_text)
        reason_match = re.search(r"REASON\s*:\s*(.*)", full_text)
        parsed["error_stage"] = stage_match.group(1).strip() if stage_match else "Unknown Stage"
        parsed["error_message"] = reason_match.group(1).strip() if reason_match else "Unknown Error"
        parsed["success"] = False
        return parsed

    parsed["success"] = True

    patterns = {
        "total_mass": r"Total Mass \(kg\):\s+([\d\.]+)\s+([\d\.]+)",
        "frame_mass": r"Carbon Frame Mass \(kg\):\s+([\d\.]+)\s+([\d\.]+)",
        "battery_mass": r"Battery Mass \(kg\):\s+([\d\.]+)\s+([\d\.]+)",
        "drained_energy": r"Drained Energy \(mAh\):\s+([\d\.]+)\s+([\d\.]+)",
        "cell_count": r"Converged Cell Count \(S\):\s+(\d+)\s+(\d+)",
        "max_battery_temp": r"Max Battery Temp \(C\):\s+([\d\.]+)\s+([\d\.]+)",
        "thermal_margin": r"Thermal Margin \(C\):\s+([\d\.-]+)\s+([\d\.-]+)",
        "thrust_to_weight": r"Thrust-to-Weight Ratio:\s+([\d\.]+)\s+([\d\.]+)",
        "forward_pitch": r"Forward Pitch Angle \(deg\):\s+([\d\.-]+)\s+([\d\.-]+)",
        "j_score": r"MINLP Objective J-Score:\s+([\d\.]+)\s+([\d\.]+)",
        "prop_size": r"Propeller Size \(in\):\s+(\S+)\s+(\S+)",
        "target_kv": r"Selected Target KV:\s+([\d\.]+)\s+([\d\.]+)",
    }

    for key, pattern in patterns.items():
        match = re.search(pattern, full_text)
        if match:
            parsed[key] = float(match.group(2)) if key != "prop_size" else match.group(2)
        else:
            parsed[key] = None
    return parsed

def run_engine(config, exe_path):
    with open("data/user_input.json", "w") as f:
        json.dump(config, f, indent=4)
    p = subprocess.run([exe_path], capture_output=True, text=True)
    return parse_output(p.stdout, p.stderr)

def main():
    exe_path = "build/Release/DroneEngine.exe" if os.path.exists("build/Release/DroneEngine.exe") else "build/DroneEngine.exe"
    
    classes = {
        "Light": LIGHT_TEMPLATE,
        "Medium": MEDIUM_TEMPLATE,
        "Heavy": HEAVY_TEMPLATE
    }
    
    artifact_dir = "C:/Users/user/.gemini/antigravity/brain/2bb68460-98e1-4e53-a8d0-6970e18b569a"
    os.makedirs(artifact_dir, exist_ok=True)
    colors = {"Light": "#2c7bb6", "Medium": "#fdae61", "Heavy": "#d7191c", "ExtremeHeavy": "#762a83"}

    # -------------------------------------------------------------------------
    # Sweep 1: Core Physical Parameters
    # -------------------------------------------------------------------------
    print("Plotting Sweep 1: Core Physical Parameters...")
    payload_sweeps = {"Light": [0.05, 0.1, 0.2, 0.3, 0.4], "Medium": [0.5, 1.0, 1.5, 2.0, 3.0], "Heavy": [3.0, 5.0, 8.0, 11.0, 14.0]}
    payload_data = {c: {"x": [], "total_mass": [], "battery_mass": []} for c in classes}
    for cls_name, template in classes.items():
        for p_val in payload_sweeps[cls_name]:
            cfg = json.loads(json.dumps(template))
            cfg["payload_kg"] = p_val
            if cls_name == "Heavy":
                cfg["payload_role"]["delivery_drop_mass_kg"] = p_val * 0.8
            res = run_engine(cfg, exe_path)
            if res.get("success", False):
                payload_data[cls_name]["x"].append(p_val)
                payload_data[cls_name]["total_mass"].append(res["total_mass"])
                payload_data[cls_name]["battery_mass"].append(res["battery_mass"])

    plt.figure(figsize=(8, 5))
    for cls_name, data in payload_data.items():
        if data["x"]:
            plt.plot(data["x"], data["total_mass"], marker="o", linestyle="-", color=colors[cls_name], label=f"{cls_name} Total Mass")
            plt.plot(data["x"], data["battery_mass"], marker="x", linestyle="--", color=colors[cls_name], alpha=0.7, label=f"{cls_name} Battery Mass")
    plt.title("Payload Mass vs Drone Total & Battery Mass", fontsize=12, fontweight="bold")
    plt.xlabel("Payload Mass (kg)")
    plt.ylabel("Output Mass (kg)")
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{artifact_dir}/payload_vs_mass.png", dpi=150)
    plt.close()

    # -------------------------------------------------------------------------
    # Sweep 2: Mission Timings
    # -------------------------------------------------------------------------
    print("Plotting Sweep 2: Mission Timings vs Energy...")
    duration_multipliers = [0.5, 1.0, 1.5, 2.0]
    timings_data = {c: {"x": [], "energy": []} for c in classes}
    for cls_name, template in classes.items():
        for mult in duration_multipliers:
            cfg = json.loads(json.dumps(template))
            cfg["t_hover_s"] *= mult
            cfg["t_forward_s"] *= mult
            cfg["t_climb_s"] *= mult
            res = run_engine(cfg, exe_path)
            if res.get("success", False):
                timings_data[cls_name]["x"].append(mult * template["t_forward_s"])
                timings_data[cls_name]["energy"].append(res["drained_energy"])

    plt.figure(figsize=(8, 5))
    for cls_name, data in timings_data.items():
        if data["x"]:
            plt.plot(data["x"], data["energy"], marker="o", linestyle="-", color=colors[cls_name], label=cls_name)
    plt.title("Forward Duration vs Drained Battery Energy", fontsize=12, fontweight="bold")
    plt.xlabel("Forward Flight Duration (seconds)")
    plt.ylabel("Drained Energy (mAh)")
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{artifact_dir}/timings_vs_energy.png", dpi=150)
    plt.close()

    # -------------------------------------------------------------------------
    # Sweep 3: Climb Dynamics
    # -------------------------------------------------------------------------
    print("Plotting Sweep 3: Climb Dynamics...")
    climb_speeds = [1.0, 2.0, 3.0, 5.0, 8.0]
    climb_data = {c: {"x": [], "cell_count": [], "energy": []} for c in classes}
    for cls_name, template in classes.items():
        for vc in climb_speeds:
            cfg = json.loads(json.dumps(template))
            cfg["v_climb_ms"] = vc
            res = run_engine(cfg, exe_path)
            if res.get("success", False):
                climb_data[cls_name]["x"].append(vc)
                climb_data[cls_name]["cell_count"].append(res["cell_count"])
                climb_data[cls_name]["energy"].append(res["drained_energy"])

    fig, ax1 = plt.subplots(figsize=(8, 5))
    ax2 = ax1.twinx()
    for cls_name, data in climb_data.items():
        if data["x"]:
            ax1.plot(data["x"], data["cell_count"], marker="s", linestyle="-", color=colors[cls_name], label=f"{cls_name} Cells")
            ax2.plot(data["x"], data["energy"], marker="^", linestyle=":", color=colors[cls_name], alpha=0.7, label=f"{cls_name} Energy")
    ax1.set_xlabel("Climb Velocity (m/s)")
    ax1.set_ylabel("Converged Battery Cell Count (S)")
    ax2.set_ylabel("Drained Energy (mAh)")
    plt.title("Climb Velocity vs Cell Count & Energy", fontsize=12, fontweight="bold")
    ax1.grid(True, linestyle=":", alpha=0.6)
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1+h2, l1+l2, loc="upper left")
    plt.tight_layout()
    plt.savefig(f"{artifact_dir}/climb_dynamics.png", dpi=150)
    plt.close()

    # -------------------------------------------------------------------------
    # Sweep 4: Aerodynamic Overrides
    # -------------------------------------------------------------------------
    print("Plotting Sweep 4: Aerodynamic Overrides...")
    fom_sweep = [0.45, 0.55, 0.65, 0.75, 0.85]
    fom_data = {c: {"x": [], "energy": []} for c in classes}
    for cls_name, template in classes.items():
        for fom in fom_sweep:
            cfg = json.loads(json.dumps(template))
            cfg["aerodynamic_overrides"]["figure_of_merit"] = fom
            res = run_engine(cfg, exe_path)
            if res.get("success", False):
                fom_data[cls_name]["x"].append(fom)
                fom_data[cls_name]["energy"].append(res["drained_energy"])

    ct_sweep = [0.06, 0.09, 0.12, 0.15]
    ct_data = {c: {"x": [], "target_kv": []} for c in classes}
    for cls_name, template in classes.items():
        for ct in ct_sweep:
            cfg = json.loads(json.dumps(template))
            cfg["aerodynamic_overrides"]["assumed_ct"] = ct
            res = run_engine(cfg, exe_path)
            if res.get("success", False):
                ct_data[cls_name]["x"].append(ct)
                ct_data[cls_name]["target_kv"].append(res["target_kv"])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    for cls_name, data in fom_data.items():
        if data["x"]:
            ax1.plot(data["x"], data["energy"], marker="o", linestyle="-", color=colors[cls_name], label=cls_name)
    ax1.set_title("Figure of Merit vs Energy", fontsize=11, fontweight="bold")
    ax1.set_xlabel("Figure of Merit (FoM)")
    ax1.set_ylabel("Drained Energy (mAh)")
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend()

    for cls_name, data in ct_data.items():
        if data["x"]:
            ax2.plot(data["x"], data["target_kv"], marker="s", linestyle="-", color=colors[cls_name], label=cls_name)
    ax2.set_title("Assumed Ct vs Selected Target KV", fontsize=11, fontweight="bold")
    ax2.set_xlabel("Assumed Thrust Coefficient (Ct)")
    ax2.set_ylabel("Target KV (RPM/V)")
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend()
    plt.tight_layout()
    plt.savefig(f"{artifact_dir}/aerodynamic_overrides.png", dpi=150)
    plt.close()

    # -------------------------------------------------------------------------
    # Sweep 5: Structural Overrides
    # -------------------------------------------------------------------------
    print("Plotting Sweep 5: Structural Overrides...")
    geom_sweep = [0.15, 0.25, 0.3439, 0.45, 0.55]
    geom_res = []
    wall_sweep = [0.70, 0.78, 0.85, 0.90, 0.95]
    wall_res = []
    body_sweep = [1.5, 2.5, 3.5, 4.5, 5.5]
    body_res = []

    for val in geom_sweep:
        cfg = json.loads(json.dumps(STRUCTURAL_TEST_TEMPLATE))
        cfg["structural_overrides"]["geometry_constant"] = val
        res = run_engine(cfg, exe_path)
        if not res.get("success", False):
            print(f"  [WARN] geom_constant={val} failed")
        geom_res.append(res["frame_mass"] if res.get("success", False) else None)

    for val in wall_sweep:
        cfg = json.loads(json.dumps(STRUCTURAL_TEST_TEMPLATE))
        cfg["structural_overrides"]["wall_thickness_ratio"] = val
        res = run_engine(cfg, exe_path)
        if not res.get("success", False):
            print(f"  [WARN] wall_thickness_ratio={val} failed")
        wall_res.append(res["frame_mass"] if res.get("success", False) else None)

    for val in body_sweep:
        cfg = json.loads(json.dumps(STRUCTURAL_TEST_TEMPLATE))
        cfg["structural_overrides"]["body_mass_multiplier"] = val
        res = run_engine(cfg, exe_path)
        if not res.get("success", False):
            print(f"  [WARN] body_mass_multiplier={val} failed")
        body_res.append(res["frame_mass"] if res.get("success", False) else None)

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4.5))
    valid_g = [(x, y) for x, y in zip(geom_sweep, geom_res) if y is not None]
    if valid_g:
        ax1.plot([v[0] for v in valid_g], [v[1] for v in valid_g], marker="o", linestyle="-", color="#762a83")
    ax1.set_title("Geometry Constant vs Frame Mass (15 kg drone)", fontsize=10, fontweight="bold")
    ax1.set_xlabel("Geometry Constant")
    ax1.set_ylabel("Carbon Frame Mass (kg)")
    ax1.grid(True, linestyle=":", alpha=0.6)

    valid_w = [(x, y) for x, y in zip(wall_sweep, wall_res) if y is not None]
    if valid_w:
        ax2.plot([v[0] for v in valid_w], [v[1] for v in valid_w], marker="s", linestyle="-", color="#1b7837")
    ax2.set_title("Wall Thickness Ratio vs Frame Mass (15 kg drone)", fontsize=10, fontweight="bold")
    ax2.set_xlabel("Wall Thickness Ratio (d_in/d_out)")
    ax2.set_ylabel("Carbon Frame Mass (kg)")
    ax2.grid(True, linestyle=":", alpha=0.6)

    valid_b = [(x, y) for x, y in zip(body_sweep, body_res) if y is not None]
    if valid_b:
        ax3.plot([v[0] for v in valid_b], [v[1] for v in valid_b], marker="d", linestyle="-", color="#2166ac")
    ax3.set_title("Body Mass Multiplier vs Frame Mass (15 kg drone)", fontsize=10, fontweight="bold")
    ax3.set_xlabel("Body Mass Multiplier")
    ax3.set_ylabel("Carbon Frame Mass (kg)")
    ax3.grid(True, linestyle=":", alpha=0.6)

    plt.tight_layout()
    plt.savefig(f"{artifact_dir}/structural_overrides.png", dpi=150)
    plt.close()

    # -------------------------------------------------------------------------
    # Sweep 6: Role Specific Overrides
    # -------------------------------------------------------------------------
    print("Plotting Sweep 6: Role Specific Parameters...")
    # Use STRUCTURAL_TEST_TEMPLATE (15 kg, 6G racing, 2m diameter) — converges reliably.
    # d_y = (2.0/2) * sqrt(2)/2 = 0.707 m, so cg_shift up to 0.6 m is safe.
    cg_sweep = [0.0, 0.15, 0.30, 0.45, 0.60]
    cg_res = []
    for cg in cg_sweep:
        cfg = json.loads(json.dumps(STRUCTURAL_TEST_TEMPLATE))
        cfg["payload_role"]["cg_shift_m"] = cg
        res = run_engine(cfg, exe_path)
        if not res.get("success", False):
            print(f"  [WARN] cg_shift={cg} failed")
        cg_res.append(res["frame_mass"] if res.get("success", False) else None)

    lf_sweep = [1.5, 3.0, 6.0, 10.0, 14.0]
    lf_res = []
    for lf in lf_sweep:
        cfg = json.loads(json.dumps(STRUCTURAL_TEST_TEMPLATE))
        cfg["payload_role"]["racing_load_factor"] = lf
        res = run_engine(cfg, exe_path)
        if not res.get("success", False):
            print(f"  [WARN] racing_load_factor={lf} failed")
        lf_res.append(res["frame_mass"] if res.get("success", False) else None)

    dam_sweep = [1.0, 1.5, 2.0, 2.5]
    dam_res = []
    for dam in dam_sweep:
        cfg = json.loads(json.dumps(MEDIUM_TEMPLATE))
        cfg["payload_role"]["type"] = "mapping"
        cfg["payload_role"]["drag_area_multiplier"] = dam
        cfg["payload_role"]["added_drag_m2"] = 0.05
        res = run_engine(cfg, exe_path)
        dam_res.append(res["forward_pitch"] if res.get("success", False) else None)

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4.5))
    valid_cg = [(x, y) for x, y in zip(cg_sweep, cg_res) if y is not None]
    if valid_cg:
        ax1.plot([v[0] for v in valid_cg], [v[1] for v in valid_cg], marker="o", linestyle="-", color="#d73027")
    ax1.set_title("CG Shift vs Frame Mass (15 kg Drone, 6G)", fontsize=10, fontweight="bold")
    ax1.set_xlabel("Center of Gravity Shift (m)")
    ax1.set_ylabel("Carbon Frame Mass (kg)")
    ax1.grid(True, linestyle=":", alpha=0.6)

    valid_lf = [(x, y) for x, y in zip(lf_sweep, lf_res) if y is not None]
    if valid_lf:
        ax2.plot([v[0] for v in valid_lf], [v[1] for v in valid_lf], marker="s", linestyle="-", color="#f46d43")
    ax2.set_title("Load Factor (G-force) vs Frame Mass (15 kg Drone)", fontsize=10, fontweight="bold")
    ax2.set_xlabel("Dynamic Load Factor (G)")
    ax2.set_ylabel("Carbon Frame Mass (kg)")
    ax2.grid(True, linestyle=":", alpha=0.6)

    ax3.plot(dam_sweep, dam_res, marker="d", linestyle="-", color="#4575b4")
    ax3.set_title("Drag Area Multiplier vs Pitch (Mapping)", fontsize=10, fontweight="bold")
    ax3.set_xlabel("Drag Area Multiplier")
    ax3.set_ylabel("Forward Pitch Angle (deg)")
    ax3.grid(True, linestyle=":", alpha=0.6)

    plt.tight_layout()
    plt.savefig(f"{artifact_dir}/role_specific_overrides.png", dpi=150)
    plt.close()

    # -------------------------------------------------------------------------
    # Sweep 7: Battery Chemistry
    # -------------------------------------------------------------------------
    print("Plotting Sweep 7: Battery Chemistry...")
    chemistries = ["LiPo", "LiHV", "Li-ion NMC", "Li-ion NCA", "Solid State", "Li-S"]
    chem_mass = {c: [] for c in classes}
    chem_temp = {c: [] for c in classes}
    for cls_name, template in classes.items():
        for chem in chemistries:
            cfg = json.loads(json.dumps(template))
            cfg["battery_chemistry"] = chem
            res = run_engine(cfg, exe_path)
            if res.get("success", False):
                chem_mass[cls_name].append(res["battery_mass"])
                chem_temp[cls_name].append(res["max_battery_temp"])
            else:
                chem_mass[cls_name].append(0.0)
                chem_temp[cls_name].append(0.0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    x_indices = np.arange(len(chemistries))
    width = 0.25

    ax1.bar(x_indices - width, chem_mass["Light"], width, label="Light Class", color=colors["Light"])
    ax1.bar(x_indices, chem_mass["Medium"], width, label="Medium Class", color=colors["Medium"])
    ax1.bar(x_indices + width, chem_mass["Heavy"], width, label="Heavy Class", color=colors["Heavy"])
    ax1.set_title("Battery Chemistry vs Battery Mass", fontsize=11, fontweight="bold")
    ax1.set_xticks(x_indices)
    ax1.set_xticklabels(chemistries, rotation=20)
    ax1.set_ylabel("Battery Weight (kg)")
    ax1.grid(True, linestyle=":", alpha=0.4)
    ax1.legend()

    for cls_name, temps in chem_temp.items():
        ax2.plot(chemistries, temps, marker="o", linestyle="-", color=colors[cls_name], label=cls_name)
    ax2.set_title("Battery Chemistry vs Peak Operating Temp", fontsize=11, fontweight="bold")
    ax2.set_ylabel("Peak Temperature (°C)")
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend()
    plt.tight_layout()
    plt.savefig(f"{artifact_dir}/battery_chemistry.png", dpi=150)
    plt.close()

    # -------------------------------------------------------------------------
    # Sweep 8: Locks Effects
    # -------------------------------------------------------------------------
    print("Plotting Sweep 8: Locks Effects...")
    locked_diams = [20.0, 24.0, 26.0, 28.0, 30.0, 32.0, 36.0]
    j_scores = []
    prop_pitches = [6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]
    
    for diam, pitch in zip(locked_diams, prop_pitches):
        cfg = json.loads(json.dumps(MEDIUM_TEMPLATE))
        cfg["locks"]["is_prop_locked"] = True
        cfg["locks"]["locked_diameter_in"] = diam
        cfg["locks"]["locked_pitch_in"] = pitch
        res = run_engine(cfg, exe_path)
        j_scores.append(res["j_score"] if res.get("success", False) else None)

    plt.figure(figsize=(8, 5))
    plt.plot(locked_diams, j_scores, marker="o", color="#e7298a", linewidth=2, label="Locked Hardware Objective")
    plt.axvline(28.0, color="green", linestyle="--", label="Unlocked Optimum (28x9)")
    plt.title("Locked Propeller Diameter vs MINLP Objective J-Score", fontsize=12, fontweight="bold")
    plt.xlabel("Locked Propeller Diameter (inches)")
    plt.ylabel("Discrete Optimization J-Score")
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{artifact_dir}/locks_effects.png", dpi=150)
    plt.close()

    # -------------------------------------------------------------------------
    # Sweep 9: Sensitivity Heatmap Matrix
    # -------------------------------------------------------------------------
    print("Generating Sweep 9: 8x15 Sensitivity Heatmap Matrix...")
    inputs = {
        "payload_kg": ("payload_kg", 0.15),
        "max_diameter_m": ("max_diameter_m", 0.10),
        "v_forward_ms": ("v_forward_ms", 0.15),
        "v_climb_ms": ("v_climb_ms", 0.15),
        "t_hover_s": ("t_hover_s", 0.15),
        "t_forward_s": ("t_forward_s", 0.15),
        "t_climb_s": ("t_climb_s", 0.15),
        "altitude_m": ("altitude_m", 100.0),
        "ambient_temp_c": ("ambient_temp_c", 5.0),
        "figure_of_merit": ("aerodynamic_overrides.figure_of_merit", 0.05),
        # propulsive_efficiency: perturb DOWN (-0.10) so the COTS optimizer doesn't snap
        # to a different discrete component that accidentally raises energy (sign inversion).
        "propulsive_efficiency": ("aerodynamic_overrides.propulsive_efficiency", -0.10),
        "cd_horizontal": ("aerodynamic_overrides.cd_horizontal", 0.15),
        "cd_vertical": ("aerodynamic_overrides.cd_vertical", 0.15),
        "assumed_ct": ("aerodynamic_overrides.assumed_ct", 0.02),
        # body_mass_multiplier: perturb by +0.15 (relative) so new_val = 1.5 * 1.15 = 1.725
        "body_mass_multiplier": ("structural_overrides.body_mass_multiplier", 0.15)
    }

    outputs = [
        "total_mass",
        "frame_mass",
        "battery_mass",
        "drained_energy",
        "cell_count",
        "max_battery_temp",
        "forward_pitch",
        "target_kv"
    ]

    base_res = run_engine(SENSITIVITY_BASELINE, exe_path)
    if not base_res.get("success", False):
        print("ERROR: Sensitivity baseline failed. Sizing engine must converge.")
        return

    sensitivity_matrix = np.zeros((len(outputs), len(inputs)))

    for in_idx, (in_name, (in_path, pert)) in enumerate(inputs.items()):
        cfg = json.loads(json.dumps(SENSITIVITY_BASELINE))
        parts = in_path.split('.')
        base_val = cfg
        for part in parts[:-1]:
            base_val = base_val[part]
        
        orig_val = base_val[parts[-1]]
        
        is_relative = True
        if in_name in ["altitude_m", "ambient_temp_c"]:
            new_val = orig_val + pert
            is_relative = False
        else:
            new_val = orig_val * (1.0 + pert)
            
        base_val[parts[-1]] = new_val
        
        pert_res = run_engine(cfg, exe_path)
        
        for out_idx, out_name in enumerate(outputs):
            if not pert_res.get("success", False):
                sensitivity_matrix[out_idx, in_idx] = 0.0
                continue
                
            y_base = base_res[out_name]
            y_pert = pert_res[out_name]
            
            if y_base is None or y_pert is None:
                sensitivity_matrix[out_idx, in_idx] = 0.0
                continue
                
            dy = y_pert - y_base
            
            if is_relative:
                dx_relative = pert
                dy_relative = dy / y_base if y_base != 0.0 else dy
                sensitivity = dy_relative / dx_relative
            else:
                dy_relative = dy / y_base if y_base != 0.0 else dy
                sensitivity = dy_relative / pert
                
            sensitivity_matrix[out_idx, in_idx] = np.clip(sensitivity, -5.0, 5.0)

    plt.figure(figsize=(14, 8))
    im = plt.imshow(sensitivity_matrix, cmap="RdBu_r", aspect="auto", vmin=-2.0, vmax=2.0)
    cbar = plt.colorbar(im)
    cbar.set_label("Normalized Sensitivity Coefficient (Elasticity)", fontsize=11)

    plt.xticks(np.arange(len(inputs)), list(inputs.keys()), rotation=45, ha="right", fontsize=10)
    plt.yticks(np.arange(len(outputs)), [o.replace('_', ' ').title() for o in outputs], fontsize=10)
    plt.title("Sizing Sensitivity Heatmap Matrix: How Every Input Affects Every Output", fontsize=14, fontweight="bold", pad=20)

    for i in range(len(outputs)):
        for j in range(len(inputs)):
            val = sensitivity_matrix[i, j]
            text_color = "white" if abs(val) > 1.0 else "black"
            plt.text(j, i, f"{val:+.2f}", ha="center", va="center", color=text_color, fontweight="bold", fontsize=9)

    plt.tight_layout()
    plt.savefig(f"{artifact_dir}/sensitivity_heatmap.png", dpi=150)
    plt.close()

    # -------------------------------------------------------------------------
    # NEW FEATURE: Sweep 10 — Detailed Line Plots: "How One Input Affects All Outputs"
    # Generates a 5x3 subplot matrix where each cell represents a swept input
    # and plots the relative change of all 8 output parameters.
    # -------------------------------------------------------------------------
    print("Generating Sweep 10: 5x3 Line Plots Grid...")
    
    # Define sweep ranges for each of the 15 continuous inputs
    sweep_ranges = {
        "payload_kg": np.linspace(3.0, 7.0, 6),              # baseline 5.0
        "max_diameter_m": np.linspace(0.8, 1.6, 6),          # baseline 1.2
        "v_forward_ms": np.linspace(6.0, 14.0, 6),           # baseline 10.0
        # Wider range: 0.5 -> 5.0 m/s makes the steep power curve visible
        "v_climb_ms": np.linspace(0.5, 5.0, 6),              # baseline 2.0
        "t_hover_s": np.linspace(60.0, 140.0, 6),            # baseline 100.0
        "t_forward_s": np.linspace(180.0, 420.0, 6),         # baseline 300.0
        # Wider range: 10 -> 90 s makes climb-fraction effect visible
        "t_climb_s": np.linspace(10.0, 90.0, 6),             # baseline 30.0
        "altitude_m": np.linspace(0.0, 500.0, 6),            # baseline 100.0
        "ambient_temp_c": np.linspace(10.0, 40.0, 6),        # baseline 25.0
        "figure_of_merit": np.linspace(0.40, 0.85, 6),       # baseline 0.65
        "propulsive_efficiency": np.linspace(0.45, 0.90, 6), # baseline 0.70
        "cd_horizontal": np.linspace(0.5, 1.8, 6),           # baseline 1.0
        # Wider range: 0.5 -> 4.0 to make cd_vertical measurably visible
        "cd_vertical": np.linspace(0.5, 4.0, 6),             # baseline 1.2
        "assumed_ct": np.linspace(0.06, 0.18, 6),            # baseline 0.12
        # Body-mass-multiplier sweep in a range compatible with the 5 kg baseline
        "body_mass_multiplier": np.linspace(1.5, 4.5, 6)     # baseline 1.5
    }

    # Setup the large 5x3 figure
    fig, axes = plt.subplots(5, 3, figsize=(18, 22))
    axes = axes.flatten()

    output_colors = {
        "total_mass": "#e41a1c",
        "frame_mass": "#377eb8",
        "battery_mass": "#4daf4a",
        "drained_energy": "#984ea3",
        "cell_count": "#ff7f00",
        "max_battery_temp": "#ffff33",
        "forward_pitch": "#a65628",
        "target_kv": "#f781bf"
    }

    for in_idx, (in_name, sweep_vals) in enumerate(sweep_ranges.items()):
        ax = axes[in_idx]
        in_path = inputs[in_name][0]
        
        # Store results for plotting
        collected_runs = {out_name: [] for out_name in outputs}
        x_vals = []

        for val in sweep_vals:
            cfg = json.loads(json.dumps(SENSITIVITY_BASELINE))
            parts = in_path.split('.')
            base_val = cfg
            for part in parts[:-1]:
                base_val = base_val[part]
            
            base_val[parts[-1]] = float(val)
            
            res = run_engine(cfg, exe_path)
            if res.get("success", False):
                x_vals.append(val)
                for out_name in outputs:
                    collected_runs[out_name].append(res[out_name])
        
        # Plot normalized curves (relative to baseline value)
        for out_name in outputs:
            y_vals = collected_runs[out_name]
            y_base = base_res[out_name]
            
            if y_base is not None and len(y_vals) > 0:
                # Normalize relative to baseline
                normalized_y = [y / y_base if y_base != 0.0 else y for y in y_vals]
                ax.plot(x_vals, normalized_y, marker="o", linestyle="-", 
                        color=output_colors[out_name], label=out_name.replace('_', ' ').title())
        
        ax.axhline(1.0, color="black", linestyle="--", alpha=0.5)
        ax.set_title(f"Sweep: {in_name}", fontsize=12, fontweight="bold")
        ax.set_xlabel("Input Value", fontsize=10)
        ax.set_ylabel("Normalized Outputs (Ratio vs Baseline)", fontsize=10)
        ax.grid(True, linestyle=":", alpha=0.6)

    # Place a single legend for all subplots
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, fontsize=12, bbox_to_anchor=(0.5, 0.98))
    
    plt.suptitle("Normalized Sizing Sensitivity: How One Input Parameter Influences All Sizing Outputs", 
                 fontsize=18, fontweight="bold", y=0.995)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    grid_plot_path = f"{artifact_dir}/all_inputs_vs_outputs.png"
    plt.savefig(grid_plot_path, dpi=120)
    plt.close()
    print("5x3 line plots grid generated successfully!")

if __name__ == "__main__":
    main()
