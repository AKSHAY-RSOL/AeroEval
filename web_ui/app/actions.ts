'use server'

import { writeFile, readFile } from 'fs/promises';
import { exec } from 'child_process';
import { promisify } from 'util';
import path from 'path';
import { unlink } from 'fs/promises';
import { existsSync } from 'fs';

const execAsync = promisify(exec);
const ENGINE_TIMEOUT_MS = 10_000; // 10s hard limit -- engine normally runs in <50ms

/**
 * Next.js Server Action representing the primary gateway to the C++ Sizing Engine.
 * 
 * Flow Control:
 * 1. Sanitizes and parses browser form parameters.
 * 2. If in Mission Workspace (KMODE), executes a multi-pass sweep across cruise speeds 
 *    to maximize the optimization score (Range, Flight Time, or Payload).
 * 3. Writes parameters to `user_input.json`.
 * 4. Executes the native C++ compiled solver `DroneEngine.exe` asynchronously.
 * 5. Reads `sizing_output.json` back, formats the quantitative values, and returns them to the UI.
 *
 * @param formData - Raw HTML5 form data sent from the browser client.
 * @returns Formatted baseline and custom validation outputs, failures, or error payloads.
 */
export async function runEngine(formData: FormData) {
  try {
    // 1. Reconstruct the JSON input payload structure from raw FormData.
    // Set role-appropriate cruise speeds (m/s) as defaults.
    // Cinema drones orbit smoothly at 10 m/s; a flat 15 m/s default previously caused
    // structural or thermal runaway on smaller, high-resistance battery packs.
    const roleType = (formData.get('role_type') || 'imaging') as string;
    const roleCruiseSpeeds: Record<string, number> = {
      cinema: 10.0, imaging: 10.0,      // smooth orbit / steady approach
      delivery: 14.0, inspection: 10.0, // transit velocity vs. detail scanning
      agriculture: 5.0,                 // slow, steady spray passes for coverage
      mapping: 12.0, search: 12.0,      // grid-search coverage efficiency
      racing: 25.0,                     // wide-open throttle (WOT) high-drain flight
    };
    const autoSpeed = roleCruiseSpeeds[roleType] ?? 12.0;
    const v_forward_ms_raw = formData.get('v_forward_ms');
    const v_forward_ms = (v_forward_ms_raw === 'auto' || v_forward_ms_raw === 'Auto' || v_forward_ms_raw === '' || v_forward_ms_raw === null)
      ? autoSpeed
      : Number(v_forward_ms_raw || autoSpeed);

    // Build the canonical input schema matching C++ JSON parser configurations (nlohmann/json)
    const inputJson = {
      evaluate_marketing_limits: formData.get('evaluate_marketing_limits') === 'true',
      fixed_airframe_mass_kg: formData.get('fixed_airframe_mass_kg') ? Number(formData.get('fixed_airframe_mass_kg')) : -1.0,
      payload_kg: Number(formData.get('payload_kg') || 1.5),
      max_diameter_m: Number(formData.get('max_diameter_m') || 1.0),
      v_forward_ms: v_forward_ms,
      v_climb_ms: Number(formData.get('v_climb_ms') || 2.0),
      t_climb_s: Number(formData.get('t_climb_s') || 30.0),
      t_forward_s: Number(formData.get('t_forward_s') || 600.0),
      t_hover_s: Number(formData.get('t_hover_s') || 120.0),
      altitude_m: Number(formData.get('altitude_m') || 100.0),
      ambient_temp_c: Number(formData.get('ambient_temp_c') || 25.0),
      battery_cycle_count: Number(formData.get('battery_cycle_count') || 0),
      battery_chemistry: formData.get('battery_chemistry') || 'LiPo',
      nominal_cell_count: Number(formData.get('nominal_cell_count') || 0),
      rotor_count: Number(formData.get('rotor_count') || 4),
      // Coaxial is only meaningful for an X8 (8 motors on 4 arms). Match the client-side
      // rule (page.tsx) which restricts coaxial to rotor_count === 8 so direct server-action
      // calls cannot enable coaxial on a 4/6-rotor frame.
      coaxial_layout: (Number(formData.get('rotor_count') || 4) === 8) && formData.get('coaxial_layout') === 'true',
      airframe_class: formData.get('airframe_class') || 'ConsumerFolding',
      battery_pack_count: formData.get('battery_pack_count') ? Number(formData.get('battery_pack_count')) : 0,
      battery_specific_energy_wh_kg: formData.get('battery_specific_energy_wh_kg') ? Math.max(10.1, Number(formData.get('battery_specific_energy_wh_kg'))) : -1.0,
      locks: {
        is_battery_locked: formData.get('is_battery_locked') === 'true',
        locked_capacity_mah: Number(formData.get('locked_capacity_mah') || 5000.0),
        locked_cell_count: Number(formData.get('locked_cell_count') || 0),
        is_voltage_locked: formData.get('is_voltage_locked') === 'true',
        locked_series_cells: Number(formData.get('locked_series_cells') || 6.0),
        is_prop_locked: formData.get('is_prop_locked') === 'true',
        locked_diameter_in: Number(formData.get('locked_diameter_in') || 15.0),
        locked_pitch_in: Number(formData.get('locked_pitch_in') || 5.0),
        is_motor_locked: formData.get('is_motor_locked') === 'true',
        locked_kv: Number(formData.get('locked_kv') || 0.0),
        auto_reduce_flight_time: formData.get('auto_reduce_flight_time') === 'true'
      },
      payload_role: {
        type: formData.get('role_type') || 'imaging',
        aux_power_w: Number(formData.get('aux_power_w') || 10.0),
        added_drag_m2: Number(formData.get('added_drag_m2') || 0.02),
        cg_shift_m: Number(formData.get('cg_shift_m') || 0.0),
        
        // Advanced Overrides for structural/payload modeling
        delivery_drop_mass_kg: formData.get('delivery_drop_mass_kg') ? Number(formData.get('delivery_drop_mass_kg')) : -1.0,
        delivery_drop_time_ratio: formData.get('delivery_drop_time_ratio') ? Number(formData.get('delivery_drop_time_ratio')) : -1.0,
        spray_rate_kg_per_s: formData.get('spray_rate_kg_per_s') ? Number(formData.get('spray_rate_kg_per_s')) : -1.0,
        racing_load_factor: formData.get('racing_load_factor') ? Number(formData.get('racing_load_factor')) : -1.0,
        ag_max_downwash_ms: formData.get('ag_max_downwash_ms') ? Number(formData.get('ag_max_downwash_ms')) : -1.0,
        thrust_margin_override: formData.get('thrust_margin_override') ? Number(formData.get('thrust_margin_override')) : -1.0,
        drag_area_multiplier: formData.get('drag_area_multiplier') ? Number(formData.get('drag_area_multiplier')) : -1.0,
        integration_overhead_kg: formData.get('integration_overhead_kg') ? Number(formData.get('integration_overhead_kg')) : -1.0
      },
      aerodynamic_overrides: {
        figure_of_merit: formData.get('figure_of_merit') ? Number(formData.get('figure_of_merit')) : -1.0,
        propulsive_efficiency: formData.get('propulsive_efficiency') ? Number(formData.get('propulsive_efficiency')) : -1.0,
        cd_horizontal: formData.get('cd_horizontal') ? Number(formData.get('cd_horizontal')) : -1.0,
        cd_vertical: formData.get('cd_vertical') ? Number(formData.get('cd_vertical')) : -1.0,
        area_scaling_factor: formData.get('area_scaling_factor') ? Number(formData.get('area_scaling_factor')) : -1.0,
        assumed_ct: formData.get('assumed_ct') ? Number(formData.get('assumed_ct')) : -1.0,
        aero_body_class: formData.get('aero_body_class') || 'research_exposed',
        propeller_class: 'SF',
        figure_of_merit_mode: formData.get('figure_of_merit_mode') || 'installed',
        figure_of_merit_isolated: formData.get('figure_of_merit_isolated') ? Number(formData.get('figure_of_merit_isolated')) : -1.0
      },
      structural_overrides: {
        geometry_constant: formData.get('geometry_constant') ? Number(formData.get('geometry_constant')) : -1.0,
        wall_thickness_ratio: formData.get('wall_thickness_ratio') ? Number(formData.get('wall_thickness_ratio')) : -1.0,
        body_mass_multiplier: formData.get('body_mass_multiplier') ? Number(formData.get('body_mass_multiplier')) : -1.0,
        arm_config: formData.get('arm_config') || 'under_rotor'
      }
    };

    const isKmode = formData.get('is_kmode') === 'true';
    const priority = formData.get('optimization_priority') || 'flight_time';
    const useRangeOverride = formData.get('use_range_override') === 'true';
    const range_km = formData.get('range_km') ? Number(formData.get('range_km')) : 5.0;

    // Establish absolute path metrics relative to Next.js server working directory
    const jsonPath = path.join(process.cwd(), '..', 'data', 'user_input.json');
    
    // Resolve the compiled binary path dynamically across OS configurations (Windows, Linux, macOS)
    const isWindows = process.platform === 'win32';
    const binaryName = isWindows ? 'DroneEngine.exe' : 'DroneEngine';
    const candidates = [
      path.join(process.cwd(), '..', 'build', binaryName),
      path.join(process.cwd(), '..', 'build', 'Release', binaryName),
      path.join(process.cwd(), '..', 'build', 'Debug', binaryName),
      path.join(process.cwd(), '..', binaryName),
    ];
    let exePath = candidates[0];
    for (const candidate of candidates) {
      if (existsSync(candidate)) {
        exePath = candidate;
        break;
      }
    }
    
    const engineCwd = path.join(process.cwd(), '..');
    const outputPath = path.join(process.cwd(), '..', 'data', 'sizing_output.json');

    const executeEngineRun = async (v: number) => {
      const runInput = { 
        ...inputJson, 
        v_forward_ms: v,
        baseline_v_forward_ms: inputJson.v_forward_ms,
        baseline_t_forward_s: inputJson.t_forward_s
      };
      if (useRangeOverride) {
        runInput.t_forward_s = Math.round((range_km * 1000) / v);
      }
      
      await writeFile(jsonPath, JSON.stringify(runInput, null, 4));
      
      try { await unlink(outputPath); } catch (_) {}
      
      try {
        const result = await execAsync(`"${exePath}"`, {
          cwd: engineCwd,
          timeout: ENGINE_TIMEOUT_MS,
          maxBuffer: 1 * 1024 * 1024,
          killSignal: 'SIGKILL'
        });
        const fileData = await readFile(outputPath, 'utf-8');
        return { success: true, data: JSON.parse(fileData), stdout: result.stdout, stderr: result.stderr };
      } catch (error: any) {
        return { success: false, data: null, stdout: error.stdout || '', stderr: error.stderr || error.message };
      }
    };

    let outputJson: any = null;
    let best_v: number | null = null;
    let outStr = '';

    if (isKmode) {
      // Mission Workspace Optimization Sweep:
      // The native solver calculates drone characteristics for a specific input speed.
      // To find the optimal cruise speed (e.g. to maximize range or flight time),
      // we run a two-pass sweep over candidate forward speeds.
      let bestScore = -Infinity;
      
      // Pass 1 (Coarse Sweep): Sample speeds from 2 m/s to 25 m/s to locate the general peak region.
      const coarseSpeeds = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 25];
      let bestRes: any = null;
      
      for (const v of coarseSpeeds) {
        const res = await executeEngineRun(v);
        if (!res || !res.success || !res.data || !res.data.success) continue;
        
        const cust = res.data.custom || {};
        const t_fwd = cust.approved_forward_time_s || 0;
        const t_hvr = cust.approved_hover_time_s || 0;
        const t_clb = cust.approved_climb_time_s || 0;
        const mtow = cust.mass || 0;
        
        const endurance_min = (t_fwd + t_hvr + t_clb) / 60.0;
        const range_km_achieved = (v * t_fwd) / 1000.0;
        
        let score = 0;
        if (priority === 'range') {
          score = range_km_achieved;
        } else if (priority === 'flight_time') {
          // Objective function: Maximize flight time, penalized slightly by weight (MTOW)
          // to favor lighter designs when endurance is comparable.
          // Penalty weight 0.01: 1 kg extra mass costs ~0.01 min of score.
          score = endurance_min - (mtow * 0.01);
        } else if (priority === 'system_mass') {
          // Minimum System Mass: same airframe/chemistry envelope as endurance, but the
          // score weights mass far more heavily (0.5 min/kg vs 0.01) so the sweep trades
          // endurance for the lightest viable build instead of the longest-flying one.
          score = endurance_min - (mtow * 0.5);
        } else { // priority === 'payload'
          // For payload priority, we minimize MTOW (maximize negative mass) to find the most weight-efficient sizing
          score = -mtow;
        }
        
        if (score > bestScore) {
          bestScore = score;
          best_v = v;
          bestRes = res;
        }
      }
      
      // Pass 2 (Fine Sweep): Zoom in around the coarse peak with 0.5 m/s resolution to find the local optimum.
      if (best_v !== null) {
        const fineSpeeds: number[] = [];
        const start_v = Math.max(2.0, best_v - 2.0);
        const end_v = Math.min(25.0, best_v + 2.0);
        
        let v_test = start_v;
        while (v_test <= end_v) {
          if (Math.abs(v_test - best_v) > 1e-5) {
            fineSpeeds.push(Number(v_test.toFixed(2)));
          }
          v_test += 0.5;
        }
        
        for (const v of fineSpeeds) {
          const res = await executeEngineRun(v);
          if (!res || !res.success || !res.data || !res.data.success) continue;
          
          const cust = res.data.custom || {};
          const t_fwd = cust.approved_forward_time_s || 0;
          const t_hvr = cust.approved_hover_time_s || 0;
          const t_clb = cust.approved_climb_time_s || 0;
          const mtow = cust.mass || 0;
          
          const endurance_min = (t_fwd + t_hvr + t_clb) / 60.0;
          const range_km_achieved = (v * t_fwd) / 1000.0;
          
          let score = 0;
          if (priority === 'range') {
            score = range_km_achieved;
          } else if (priority === 'flight_time') {
            score = endurance_min - (mtow * 0.01);
          } else if (priority === 'system_mass') {
            // Minimum System Mass: weight mass heavily so the sweep favors the lightest build.
            score = endurance_min - (mtow * 0.5);
          } else { // payload
            score = -mtow;
          }
          
          if (score > bestScore) {
            bestScore = score;
            best_v = v;
            bestRes = res;
          }
        }
      }
      
      if (best_v !== null && bestRes) {
        // UI-KMODE-03 fix: Utilize the pre-evaluated optimal result stored in bestRes directly
        // rather than spawning a redundant execution thread of the native solver.
        outputJson = bestRes.data;
        outStr = bestRes.stdout + '\n' + bestRes.stderr;
      }
      
      if (!outputJson || !outputJson.success) {
        return {
          success: false,
          failures: [{ run: 'System', stage: 'Sizing Sweep', reason: 'Failed to converge on any cruising speed in the sweep.' }],
          rawOutput: ''
        };
      }
      
      outputJson.custom.optimalSpeedMs = best_v;
      outputJson.baseline.optimalSpeedMs = best_v;
      
    } else {
      await writeFile(jsonPath, JSON.stringify(inputJson, null, 4));
      try { await unlink(outputPath); } catch (_) {}
      
      let stdout = '';
      let stderr = '';
      
      try {
        const result = await execAsync(`"${exePath}"`, {
          cwd: engineCwd,
          timeout: ENGINE_TIMEOUT_MS,
          maxBuffer: 1 * 1024 * 1024,
          killSignal: 'SIGKILL'
        });
        stdout = result.stdout;
        stderr = result.stderr;
      } catch (error: any) {
        stdout = error.stdout || '';
        stderr = error.stderr || error.message || 'Unknown execution error';
      }
      
      outStr = stdout + '\n' + stderr;
      
      try {
        const fileData = await readFile(outputPath, 'utf-8');
        outputJson = JSON.parse(fileData);
      } catch (e: any) {
        return {
          success: false,
          failures: [{ run: 'System', stage: 'JSON Read/Parse', reason: `Failed to load sizing_output.json: ${e.message}` }],
          rawOutput: outStr
        };
      }
    }

    const formatValue = (val: any, decimals: number = 2) => {
      if (val === undefined || val === null) return 'N/A';
      if (typeof val === 'number') {
        return val.toFixed(decimals);
      }
      return String(val);
    };

    const formatRes = (res: any) => {
      if (!res) return null;
      return {
        mass: formatValue(res.mass),
        frame: formatValue(res.frame),
        battery: formatValue(res.battery),
        energy: formatValue(res.energy, 2),
        temp: formatValue(res.temp),
        prop: String(res.prop),
        targetKv: formatValue(res.targetKv),
        shopping: String(res.shopping),
        approvedHover: formatValue(res.approvedHover),
        approvedForward: formatValue(res.approvedForward),
        approvedClimb: formatValue(res.approvedClimb),
        cellCount: String(res.cellCount),
        thermalMargin: formatValue(res.thermalMargin),
        thrustToWeight: formatValue(res.thrustToWeight),
        forwardPitch: formatValue(res.forwardPitch),
        voltage: formatValue(res.voltage),
        armOd: formatValue(res.armOd),
        armWall: formatValue(res.armWall),
        jScore: formatValue(res.jScore),
        idealKv: formatValue(res.idealKv),
        idealMotorMassG: formatValue(res.idealMotorMassG),
        optimalSpeedMs: formatValue(res.optimalSpeedMs)
      };
    };

    return {
      success: outputJson.success,
      baseline: formatRes(outputJson.baseline),
      custom: formatRes(outputJson.custom),
      failures: outputJson.failures || [],
      relaxation1: outputJson.relaxation1,
      relaxation2: outputJson.relaxation2,
      rawOutput: outStr
    };

  } catch (error: any) {
    return {
      success: false,
      error: error.message
    };
  }
}

export async function getDatabases() {
  try {
    const cotsPath = path.join(process.cwd(), '..', 'data', 'cots_database.json');
    const chemPath = path.join(process.cwd(), '..', 'data', 'chemistry_profiles.json');
    const escPath = path.join(process.cwd(), '..', 'data', 'esc_profiles.json');

    const cotsData = JSON.parse(await readFile(cotsPath, 'utf-8'));
    const chemData = JSON.parse(await readFile(chemPath, 'utf-8'));
    const escData = JSON.parse(await readFile(escPath, 'utf-8'));

    return {
      success: true,
      cots: cotsData,
      chemistry: chemData,
      esc: escData
    };
  } catch (error: any) {
    return {
      success: false,
      error: error.message
    };
  }
}

export async function updateDatabases(cots: any, chemistry: any, esc: any) {
  try {
    const cotsPath = path.join(process.cwd(), '..', 'data', 'cots_database.json');
    const chemPath = path.join(process.cwd(), '..', 'data', 'chemistry_profiles.json');
    const escPath = path.join(process.cwd(), '..', 'data', 'esc_profiles.json');

    if (cots) await writeFile(cotsPath, JSON.stringify(cots, null, 4));
    if (chemistry) await writeFile(chemPath, JSON.stringify(chemistry, null, 4));
    if (esc) await writeFile(escPath, JSON.stringify(esc, null, 4));

    return { success: true };
  } catch (error: any) {
    return {
      success: false,
      error: error.message
    };
  }
}

