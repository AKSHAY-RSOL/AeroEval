import json
import subprocess
import random
import os

def generate_realistic_case(seed):
    random.seed(seed)
    
    roles = ["imaging", "agriculture", "racing", "delivery", "mapping", "inspection"]
    role = random.choice(roles)
    
    # 1. Start with a realistic diameter category
    size_category = random.choice(["mini", "standard", "heavy"])
    
    if size_category == "mini":
        max_diameter = random.uniform(0.15, 0.3)
        payload = random.uniform(0.0, 0.3)
        target_capacity = random.choice([800, 1500, 2200])
        cell_count = random.choice([3, 4])
        v_forward = random.uniform(5, 15)
        t_forward = random.uniform(180, 400)
    elif size_category == "standard":
        max_diameter = random.uniform(0.4, 0.8)
        payload = random.uniform(0.5, 2.0)
        target_capacity = random.choice([4000, 6000, 8000])
        cell_count = random.choice([4, 6])
        v_forward = random.uniform(10, 20)
        t_forward = random.uniform(400, 900)
    else: # heavy
        max_diameter = random.uniform(1.0, 1.8)
        payload = random.uniform(3.0, 8.0)
        target_capacity = random.choice([12000, 16000, 22000])
        cell_count = random.choice([6, 8, 12])
        v_forward = random.uniform(8, 15)
        t_forward = random.uniform(600, 1200)

    # Agriculture constraints
    if role == "agriculture":
        size_category = "heavy"
        max_diameter = random.uniform(1.2, 2.0)
        payload = random.uniform(5.0, 12.0)
        v_forward = random.uniform(3, 8) # Ag drones fly slow
        target_capacity = 22000
        cell_count = 12

    # Racing constraints
    if role == "racing":
        size_category = "mini"
        max_diameter = random.uniform(0.12, 0.25)
        payload = 0.0 # Racers rarely carry payloads beyond themselves
        v_forward = random.uniform(20, 35)
        t_forward = random.uniform(60, 180) # Short flights

    locks = {
        "is_battery_locked": random.choice([True, False]),
        "locked_capacity_mah": target_capacity,
        "locked_cell_count": cell_count,
        "is_prop_locked": False, # Let engine pick prop
        "locked_diameter_in": 0.0,
        "locked_pitch_in": 0.0,
        "is_motor_locked": False, # Let engine pick motor
        "locked_kv": 0.0,
        "auto_reduce_flight_time": True # Be forgiving
    }

    payload_role = {
        "type": role,
        "aux_power_w": payload * 2.0, # Sensible aux power
        "added_drag_m2": payload * 0.005,
        "cg_shift_m": random.uniform(0, 0.02),
        "delivery_drop_mass_kg": payload * 0.8 if role == "delivery" else -1.0,
        "delivery_drop_time_ratio": 0.5,
        "spray_rate_kg_per_s": -1.0,
        "racing_load_factor": 5.0 if role == "racing" else -1.0,
        "ag_max_downwash_ms": 8.0 if role == "agriculture" else -1.0,
        "thrust_margin_override": -1.0
    }

    return {
        "payload_kg": payload,
        "max_diameter_m": max_diameter,
        "v_forward_ms": v_forward,
        "v_climb_ms": random.uniform(1, 4),
        "t_climb_s": random.uniform(10, 30),
        "t_forward_s": t_forward, 
        "t_hover_s": random.uniform(30, 120),
        "altitude_m": 0.0,
        "ambient_temp_c": 20.0,
        "locks": locks,
        "payload_role": payload_role
    }

def run_test():
    exe_path = "build/Release/DroneEngine.exe" if os.name == 'nt' else "build/DroneEngine"
    if not os.path.exists(exe_path):
        if os.path.exists("build/DroneEngine.exe"):
            exe_path = "build/DroneEngine.exe"
        else:
            print("ERROR: Executable not found.")
            return

    results = {
        "total": 0,
        "passed": 0,
        "failed": 0,
        "errors": {}
    }

    for i in range(40):
        case = generate_realistic_case(i + 100) # Offset seed
        
        with open("data/user_input.json", "w") as f:
            json.dump(case, f, indent=4)
            
        try:
            result = subprocess.run([exe_path], capture_output=True, text=True, timeout=5)
            
            results["total"] += 1
            if result.returncode == 0:
                results["passed"] += 1
            else:
                results["failed"] += 1
                
                output = result.stderr + result.stdout
                error_stage = "Unknown"
                
                for line in output.split('\\n'):
                    if "STAGE  :" in line:
                        error_stage = line.split(":", 1)[1].strip()
                        break
                        
                if error_stage not in results["errors"]:
                    results["errors"][error_stage] = 0
                results["errors"][error_stage] += 1
                
        except subprocess.TimeoutExpired:
            results["total"] += 1
            results["failed"] += 1
            results["errors"]["Timeout"] = results["errors"].get("Timeout", 0) + 1

    print("\\n=== REALISTIC TEST RESULTS ===")
    print(f"Total Cases Run: {results['total']}")
    print(f"Passed Validations: {results['passed']}")
    print(f"Failed Interceptions: {results['failed']}")
    
    print("\\n--- Failure Modes (Honest Analysis) ---")
    for stage, count in results["errors"].items():
        print(f"{stage}: {count} cases")

if __name__ == "__main__":
    print("Beginning 40-case realistic test...")
    run_test()
