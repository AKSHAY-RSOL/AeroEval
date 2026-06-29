'use client';

import { useState, useRef, useEffect } from 'react';
import { runEngine, getDatabases, updateDatabases } from './actions';

const MOTOR_DATABASE = [
  { name: 'T-Motor MN4014', kv: 380, mass: 152 },
  { name: 'Sunnysky V4004', kv: 400, mass: 148 },
  { name: 'DJI E800 3508', kv: 350, mass: 118 },
  { name: 'T-Motor U8 Lite', kv: 150, mass: 250 },
  { name: 'BrotherHobby 2806.5', kv: 1300, mass: 48 },
  { name: 'Emax ECO II 2207', kv: 1700, mass: 33 },
  { name: 'T-Motor F60 Pro V', kv: 2020, mass: 32 },
  { name: 'Xing 2207', kv: 2750, mass: 31 },
  { name: 'T-Motor U10 II', kv: 100, mass: 420 },
  { name: 'Sunnysky X2212', kv: 980, mass: 56 },
  { name: 'Sunnysky X3520', kv: 520, mass: 205 },
  { name: 'DJI 2212', kv: 920, mass: 53 },
  { name: 'T-Motor MN5008', kv: 170, mass: 280 },
  { name: 'KDE 7215XF', kv: 135, mass: 640 },
  { name: 'KDE 5215XF', kv: 435, mass: 290 },
  { name: 'T-Motor MN6007', kv: 160, mass: 180 },
  { name: 'T-Motor MN3508', kv: 700, mass: 82 },
  { name: 'Gartt ML3508', kv: 415, mass: 90 },
  { name: 'Sunnysky V2806', kv: 650, mass: 52 }
];

// Inputs to the Mission Workspace (KMODE) requirements -> physical parameters translation.
interface KmodeInputs {
  payload: number;
  transport: string;
  mission: string;
  priority: string;
  useCustomWheelbase: boolean;
  customWheelbaseMm: number;
  useRangeOverride: boolean;
  kmodeRangeKm: number;
  kmodeHoverTimeS: number;
  kmodeClimbAltitudeM: number;
}

// Resolved engine parameters produced from KMODE high-level requirements.
interface KmodeParams {
  rotor_count: number;
  coaxial_layout: boolean;
  airframe_class: string;
  role_type: string;
  battery_chemistry: string;
  battery_specific_energy_wh_kg: number;
  max_diameter_m: number;
  aux_power_w: number;
  v_forward: number;
  t_hover: number;
  t_forward: number;
  t_climb: number;
  altitude: number;
}

// Single source of truth for the Mission Workspace requirements -> drone parameter mapping.
// Previously this logic was duplicated across the mode-sync effect, the share-link builder,
// and the submit handler; any change had to be made in all of them or they silently drifted.
function translateKmodeToParams(k: KmodeInputs): KmodeParams {
  const { payload, transport, mission, priority } = k;

  const transportCategory = k.useCustomWheelbase
    ? (k.customWheelbaseMm <= 350 ? 'backpack' : k.customWheelbaseMm <= 650 ? 'trunk' : 'industrial')
    : transport;

  // Rotor count / coaxial sizing from payload + transport envelope
  let rotor_count = 4;
  let coaxial_layout = false;
  if (payload > 5.0) {
    rotor_count = 8;
    coaxial_layout = true;
  } else if (payload > 1.5 || transportCategory === 'trunk') {
    rotor_count = 6;
    coaxial_layout = false;
  }

  // Mission type -> airframe class + payload role
  let airframe_class = 'ConsumerFolding';
  let role_type = 'imaging';
  if (mission === 'cinema') {
    airframe_class = 'ConsumerFolding'; role_type = 'imaging';
  } else if (mission === 'delivery') {
    airframe_class = 'EnterpriseRugged'; role_type = 'delivery';
  } else if (mission === 'inspection') {
    airframe_class = 'EnterpriseRugged'; role_type = 'inspection';
  } else if (mission === 'agriculture') {
    airframe_class = 'Agricultural'; role_type = 'agriculture';
  } else if (mission === 'racing') {
    airframe_class = 'ConsumerFolding'; role_type = 'racing';
  } else if (mission === 'mapping') {
    airframe_class = 'ConsumerFolding'; role_type = 'mapping';
  }

  // Priority overrides for airframe class
  if (priority === 'flight_time' || priority === 'range' || priority === 'system_mass') {
    airframe_class = 'ConsumerFolding';
  } else if (priority === 'payload') {
    if (mission !== 'agriculture') {
      airframe_class = 'EnterpriseRugged';
    }
  }

  // Battery chemistry selection. All four priorities currently take the premium branch.
  let battery_chemistry = 'LiPo';
  let battery_specific_energy_wh_kg = 180;
  if (priority === 'flight_time' || priority === 'range' || priority === 'system_mass' || priority === 'payload') {
    if (payload <= 1.5) {
      battery_chemistry = 'Li-ion NMC';
      battery_specific_energy_wh_kg = 240;
    } else {
      battery_chemistry = 'LiPo_premium';
      battery_specific_energy_wh_kg = 250;
    }
  } else {
    battery_chemistry = 'LiPo';
    battery_specific_energy_wh_kg = 145;
  }

  const max_diameter_m = k.useCustomWheelbase
    ? k.customWheelbaseMm / 1000.0
    : (transport === 'backpack' ? 0.35 : transport === 'trunk' ? 0.65 : 1.20);

  let aux_power_w = 10;
  if (mission === 'cinema') aux_power_w = 5;
  else if (mission === 'agriculture') aux_power_w = 25;

  const roleCruiseSpeeds: Record<string, number> = {
    cinema: 10.0, imaging: 10.0, delivery: 14.0, inspection: 10.0,
    agriculture: 5.0, mapping: 12.0, racing: 25.0
  };
  const roleSpeed = roleCruiseSpeeds[mission] || 12.0;
  let v_forward = roleSpeed;
  if (priority === 'range') {
    v_forward = 15.0;
  } else if (priority === 'flight_time' || priority === 'system_mass') {
    v_forward = Math.min(roleSpeed, 8.0);
  } else if (priority === 'payload') {
    v_forward = Math.min(roleSpeed, 12.0);
  }

  const t_hover = k.useRangeOverride ? k.kmodeHoverTimeS : 120;
  const t_forward = k.useRangeOverride ? Math.round((k.kmodeRangeKm * 1000) / v_forward) : 600;
  const t_climb = k.useRangeOverride ? Math.round(k.kmodeClimbAltitudeM / 2.0) : 30;
  const altitude = k.useRangeOverride ? k.kmodeClimbAltitudeM : 0.0;

  return {
    rotor_count, coaxial_layout, airframe_class, role_type,
    battery_chemistry, battery_specific_energy_wh_kg, max_diameter_m,
    aux_power_w, v_forward, t_hover, t_forward, t_climb, altitude
  };
}

function DatabaseCrosshair({ idealKv, idealMotorMassG, selectedKv }: { idealKv: number, idealMotorMassG: number, selectedKv?: number }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [hovered, setHovered] = useState<any>(null);
  const [tooltipPos, setTooltipPos] = useState({ x: 0, y: 0 });

  // Compute dynamic scale boundaries based on catalog motor deviations
  const getScaleConfig = () => {
    if (!idealKv || isNaN(idealKv) || !idealMotorMassG || isNaN(idealMotorMassG)) {
      return { maxDiff: 35, scale: 3.7 };
    }
    const maxKvDiff = Math.max(...MOTOR_DATABASE.map(m => Math.abs(((m.kv - idealKv) / idealKv) * 100)));
    const maxMassDiff = Math.max(...MOTOR_DATABASE.map(m => Math.abs(((m.mass - idealMotorMassG) / idealMotorMassG) * 100)));
    const rawMax = Math.max(maxKvDiff, maxMassDiff);
    const maxDiff = rawMax > 0 && isFinite(rawMax) ? Math.max(35, Math.ceil(rawMax / 10) * 10) : 35;
    const scale = 130 / maxDiff; // 130px represents maxDiff% deviation
    return { maxDiff, scale };
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // High-DPI screen scaling
    const dpr = window.devicePixelRatio || 1;
    canvas.width = 300 * dpr;
    canvas.height = 300 * dpr;
    ctx.scale(dpr, dpr);

    ctx.clearRect(0, 0, 300, 300);

    if (!idealKv || isNaN(idealKv) || !idealMotorMassG || isNaN(idealMotorMassG)) {
      ctx.fillStyle = '#6b7280';
      ctx.font = '500 10px SFMono-Regular, Consolas, monospace';
      ctx.textAlign = 'center';
      ctx.fillText('NO SOLVER DATA YET', 150, 150);
      return;
    }

    const { maxDiff, scale } = getScaleConfig();

    // Precise dotted grid lines
    ctx.strokeStyle = '#e5e7eb';
    ctx.lineWidth = 0.5;
    ctx.setLineDash([2, 3]);
    for (let i = -5; i <= 5; i++) {
      if (i === 0) continue;
      const offset = 150 + i * 25;
      
      // Vertical grid
      ctx.beginPath();
      ctx.moveTo(offset, 15);
      ctx.lineTo(offset, 285);
      ctx.stroke();

      // Horizontal grid
      ctx.beginPath();
      ctx.moveTo(15, offset);
      ctx.lineTo(285, offset);
      ctx.stroke();
    }
    ctx.setLineDash([]);

    // Centered axis lines
    ctx.strokeStyle = '#9ca3af';
    ctx.lineWidth = 0.75;
    ctx.beginPath(); ctx.moveTo(150, 15); ctx.lineTo(150, 285); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(15, 150); ctx.lineTo(285, 150); ctx.stroke();

    // Axis tick marks
    ctx.strokeStyle = '#4b5563';
    ctx.lineWidth = 1;
    for (let i = -5; i <= 5; i++) {
      if (i === 0) continue;
      const offset = 150 + i * 25;
      
      // Horizontal axis ticks
      ctx.beginPath();
      ctx.moveTo(offset, 147);
      ctx.lineTo(offset, 153);
      ctx.stroke();

      // Vertical axis ticks
      ctx.beginPath();
      ctx.moveTo(147, offset);
      ctx.lineTo(153, offset);
      ctx.stroke();
    }

    // Monospace technical labels with dynamic percentage scale bounds
    ctx.fillStyle = '#6b7280';
    ctx.font = '500 8px SFMono-Regular, Consolas, Menlo, monospace';
    ctx.textAlign = 'left';
    ctx.fillText(`+${Math.round(maxDiff)}% MASS`, 155, 25);
    ctx.fillText(`-${Math.round(maxDiff)}% MASS`, 155, 280);
    ctx.fillText(`-${Math.round(maxDiff)}% KV`, 20, 143);
    ctx.textAlign = 'right';
    ctx.fillText(`+${Math.round(maxDiff)}% KV`, 280, 143);

    // Identify selected motor closest to the selected KV rating
    const selectedMotor = selectedKv
      ? MOTOR_DATABASE.reduce((prev, curr) => Math.abs(curr.kv - selectedKv) < Math.abs(prev.kv - selectedKv) ? curr : prev)
      : null;

    // Plot database catalog motors
    MOTOR_DATABASE.forEach(m => {
      const kvDiff = ((m.kv - idealKv) / idealKv) * 100;
      const massDiff = ((m.mass - idealMotorMassG) / idealMotorMassG) * 100;

      const x = 150 + kvDiff * scale;
      const y = 150 - massDiff * scale;

      const isSelected = selectedMotor && m.name === selectedMotor.name && Math.abs(m.kv - selectedKv!) < 50;

      if (x >= 15 && x <= 285 && y >= 15 && y <= 285) {
        if (isSelected) {
          // Highlight selected motor
          ctx.fillStyle = '#3b82f6';
          ctx.strokeStyle = '#1d4ed8';
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          ctx.arc(x, y, 5, 0, 2 * Math.PI);
          ctx.fill();
          ctx.stroke();

          ctx.fillStyle = '#ffffff';
          ctx.beginPath();
          ctx.arc(x, y, 1.5, 0, 2 * Math.PI);
          ctx.fill();
        } else {
          // Standard catalog motor dot
          ctx.fillStyle = '#ffffff';
          ctx.strokeStyle = '#4b5563';
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.arc(x, y, 3, 0, 2 * Math.PI);
          ctx.fill();
          ctx.stroke();

          ctx.fillStyle = '#9ca3af';
          ctx.beginPath();
          ctx.arc(x, y, 0.75, 0, 2 * Math.PI);
          ctx.fill();
        }
      }
    });

    // Optimum mathematical target: CAD reticle
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 0.8;
    ctx.beginPath();
    ctx.moveTo(150 - 10, 150); ctx.lineTo(150 - 3, 150);
    ctx.moveTo(150 + 3, 150); ctx.lineTo(150 + 10, 150);
    ctx.moveTo(150, 150 - 10); ctx.lineTo(150, 150 - 3);
    ctx.moveTo(150, 150 + 3); ctx.lineTo(150, 150 + 10);
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(150, 150, 2.5, 0, 2 * Math.PI);
    ctx.stroke();

    // Hover highlighted ring
    if (hovered) {
      const hx = 150 + hovered.kvDiff * scale;
      const hy = 150 - hovered.massDiff * scale;
      ctx.strokeStyle = '#3b82f6';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(hx, hy, 7, 0, 2 * Math.PI);
      ctx.stroke();
    }

  }, [idealKv, idealMotorMassG, hovered, selectedKv]);

  const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;

    if (!idealKv || isNaN(idealKv) || !idealMotorMassG || isNaN(idealMotorMassG)) return;

    const { scale } = getScaleConfig();

    let found = null;
    for (const m of MOTOR_DATABASE) {
      const kvDiff = ((m.kv - idealKv) / idealKv) * 100;
      const massDiff = ((m.mass - idealMotorMassG) / idealMotorMassG) * 100;

      const x = 150 + kvDiff * scale;
      const y = 150 - massDiff * scale;

      if (Math.hypot(mx - x, my - y) < 8) {
        found = { ...m, kvDiff, massDiff, x, y };
        break;
      }
    }

    if (found) {
      setHovered(found);
      setTooltipPos({ x: found.x + 10, y: found.y - 45 });
    } else {
      setHovered(null);
    }
  };

  return (
    <div style={{ position: 'relative', textAlign: 'center', fontFamily: 'Inter, sans-serif' }}>
      <h4 style={{ fontSize: '0.85rem', fontWeight: 600, marginBottom: '0.25rem', color: '#1f2937' }}>COTS Component Offset</h4>
      <p style={{ fontSize: '0.7rem', color: '#6b7280', marginBottom: '0.75rem' }}>Optimal: {Math.round(idealKv)} KV, {Math.round(idealMotorMassG)}g per motor</p>
      <canvas
        ref={canvasRef}
        style={{ 
          width: '300px', 
          height: '300px', 
          border: '1px solid #e5e7eb', 
          borderRadius: '4px', 
          background: '#fafafa', 
          cursor: 'crosshair'
        }}
        onMouseMove={handleMouseMove}
        onMouseLeave={() => setHovered(null)}
      />
      {hovered && (
        <div style={{
          position: 'absolute',
          left: `${tooltipPos.x}px`,
          top: `${tooltipPos.y}px`,
          background: 'rgba(17, 24, 39, 0.95)',
          color: '#ffffff',
          padding: '0.4rem 0.6rem',
          borderRadius: '4px',
          fontSize: '0.7rem',
          pointerEvents: 'none',
          zIndex: 10,
          textAlign: 'left',
          boxShadow: '0 2px 8px rgba(0, 0, 0, 0.15)',
          border: '1px solid #374151',
          lineHeight: 1.3,
          backdropFilter: 'blur(4px)',
          fontFamily: 'SFMono-Regular, Consolas, monospace'
        }}>
          <strong style={{ display: 'block', marginBottom: '0.15rem', color: '#93c5fd' }}>{hovered.name}</strong>
          KV: {hovered.kv} ({hovered.kvDiff > 0 ? '+' : ''}{hovered.kvDiff.toFixed(1)}%)<br/>
          Mass: {hovered.mass}g ({hovered.massDiff > 0 ? '+' : ''}{hovered.massDiff.toFixed(1)}%)
        </div>
      )}
    </div>
  );
}

function WhatIfExplorer({ totalMass, batteryMass, hoverTime, batterySpecificEnergy, nominalVoltage }: { totalMass: number, batteryMass: number, hoverTime: number, batterySpecificEnergy: number, nominalVoltage: number }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [battScale, setBattScale] = useState<number>(1.0);

  const dryMass = totalMass - batteryMass;
  const currentCapacityWh = batteryMass * batterySpecificEnergy;
  const hoverTimeHours = hoverTime / 3600;
  const dischargeEfficiency = 0.82;
  const hoverPowerChosen = hoverTimeHours > 0 ? (currentCapacityWh * dischargeEfficiency) / hoverTimeHours : 150.0;

  const getSizingPoint = (scale: number) => {
    const capWh = currentCapacityWh * scale;
    const bMass = capWh / batterySpecificEnergy;
    const tMass = dryMass + bMass;
    const hoverPower = hoverPowerChosen * Math.pow(tMass / totalMass, 1.5);
    const timeHours = (capWh * dischargeEfficiency) / hoverPower;
    const timeMins = timeHours * 60;
    return { timeMins, tMass, bMass, capMah: (capWh / nominalVoltage) * 1000 };
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Retina support scaling
    const dpr = window.devicePixelRatio || 1;
    canvas.width = 300 * dpr;
    canvas.height = 300 * dpr;
    ctx.scale(dpr, dpr);

    ctx.clearRect(0, 0, 300, 300);

    const points: any[] = [];
    let maxTime = 0;
    let maxTimeCapScale = 1.0;
    for (let s = 0.1; s <= 3.0; s += 0.05) {
      const pt = getSizingPoint(s);
      points.push({ scale: s, ...pt });
      if (pt.timeMins > maxTime) {
        maxTime = pt.timeMins;
        maxTimeCapScale = s;
      }
    }

    // Origin: (40, 250). Top-right plot bound: (280, 30)
    const mapX = (scale: number) => 40 + ((scale - 0.1) / 2.9) * 235;
    const mapY = (tMins: number) => 250 - (tMins / (maxTime * 1.25)) * 215;

    // Faint engineering horizontal grid lines
    ctx.strokeStyle = '#f3f4f6';
    ctx.lineWidth = 0.5;
    ctx.setLineDash([2, 3]);
    const stepMin = maxTime > 60 ? 20 : maxTime > 30 ? 10 : 5;
    for (let t = stepMin; t < maxTime * 1.25; t += stepMin) {
      const gy = mapY(t);
      ctx.beginPath(); ctx.moveTo(40, gy); ctx.lineTo(280, gy); ctx.stroke();
    }
    ctx.setLineDash([]);

    // Plain minimalist axes (L-bracket)
    ctx.strokeStyle = '#9ca3af';
    ctx.lineWidth = 0.75;
    ctx.beginPath(); ctx.moveTo(40, 20); ctx.lineTo(40, 250); ctx.lineTo(285, 250); ctx.stroke();

    // Axis tick marks & numeric values
    ctx.strokeStyle = '#4b5563';
    ctx.lineWidth = 1;
    ctx.fillStyle = '#6b7280';
    ctx.font = '500 8px SFMono-Regular, Consolas, Menlo, monospace';

    // Y ticks (Time)
    ctx.textAlign = 'right';
    for (let t = stepMin; t < maxTime * 1.25; t += stepMin) {
      const gy = mapY(t);
      ctx.beginPath(); ctx.moveTo(36, gy); ctx.lineTo(40, gy); ctx.stroke();
      ctx.fillText(`${t}m`, 32, gy + 3);
    }

    // X ticks (Capacity Scale)
    ctx.textAlign = 'center';
    const xScales = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0];
    xScales.forEach(s => {
      const gx = mapX(s);
      ctx.beginPath(); ctx.moveTo(gx, 250); ctx.lineTo(gx, 254); ctx.stroke();
      ctx.fillText(`${s}x`, gx, 264);
    });

    // Axis labels
    ctx.fillStyle = '#4b5563';
    ctx.font = '500 8.5px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Battery capacity scale', 160, 282);
    ctx.save();
    ctx.translate(12, 135);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText('Sized flight time', 0, 0);
    ctx.restore();

    // Subtle area gradient fill under curve
    const gradient = ctx.createLinearGradient(0, 30, 0, 250);
    gradient.addColorStop(0, 'rgba(59, 130, 246, 0.05)');
    gradient.addColorStop(1, 'rgba(59, 130, 246, 0.0)');
    ctx.fillStyle = gradient;
    ctx.beginPath();
    points.forEach((pt, i) => {
      const cx = mapX(pt.scale);
      const cy = mapY(pt.timeMins);
      if (i === 0) ctx.moveTo(cx, 250);
      ctx.lineTo(cx, cy);
    });
    ctx.lineTo(mapX(points[points.length - 1].scale), 250);
    ctx.closePath();
    ctx.fill();

    // Sleek Indigo Curve Line
    ctx.strokeStyle = '#2563eb';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    points.forEach((pt, i) => {
      const cx = mapX(pt.scale);
      const cy = mapY(pt.timeMins);
      if (i === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    });
    ctx.stroke();

    // Active design sizing details
    const activeX = mapX(battScale);
    const activePt = getSizingPoint(battScale);
    const activeY = mapY(activePt.timeMins);

    // Active cursor crosshair projections (fine dashed gray lines)
    ctx.strokeStyle = '#9ca3af';
    ctx.lineWidth = 0.5;
    ctx.setLineDash([2, 2]);
    
    // Horizontal projection to Y-axis
    ctx.beginPath(); ctx.moveTo(40, activeY); ctx.lineTo(activeX, activeY); ctx.stroke();
    // Vertical projection to X-axis
    ctx.beginPath(); ctx.moveTo(activeX, activeY); ctx.lineTo(activeX, 250); ctx.stroke();
    ctx.setLineDash([]);

    // Optimal peak efficiency marker (emerald-500 target reticle)
    const peakPt = getSizingPoint(maxTimeCapScale);
    const px = mapX(maxTimeCapScale);
    const py = mapY(peakPt.timeMins);
    ctx.strokeStyle = '#10b981';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(px, py, 3, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(px - 5, py); ctx.lineTo(px + 5, py);
    ctx.moveTo(px, py - 5); ctx.lineTo(px, py + 5);
    ctx.stroke();

    // Active selection marker (neat red dot with outline ring)
    ctx.strokeStyle = '#ef4444';
    ctx.fillStyle = '#ef4444';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(activeX, activeY, 1.5, 0, 2 * Math.PI);
    ctx.fill();
    ctx.beginPath();
    ctx.arc(activeX, activeY, 4.5, 0, 2 * Math.PI);
    ctx.stroke();

    // Floating text label near pointer
    ctx.fillStyle = '#ef4444';
    ctx.font = 'bold 8.5px SFMono-Regular, Consolas, monospace';
    ctx.textAlign = activeX > 200 ? 'right' : 'left';
    ctx.fillText(
      `${activePt.timeMins.toFixed(1)} min`, 
      activeX + (activeX > 200 ? -8 : 8), 
      activeY - 5
    );

  }, [totalMass, batteryMass, hoverTime, batterySpecificEnergy, nominalVoltage, battScale]);

  const activePt = getSizingPoint(battScale);

  return (
    <div style={{ textAlign: 'center', fontFamily: 'Inter, sans-serif' }}>
      <h4 style={{ fontSize: '0.85rem', fontWeight: 600, marginBottom: '0.25rem', color: '#1f2937' }}>"What-If" Endurance Explorer</h4>
      <p style={{ fontSize: '0.7rem', color: '#6b7280', marginBottom: '0.75rem' }}>Active: {activePt.timeMins.toFixed(1)}m duration at {activePt.tMass.toFixed(2)}kg</p>
      <canvas
        ref={canvasRef}
        style={{ 
          width: '300px', 
          height: '300px', 
          border: '1px solid #e5e7eb', 
          borderRadius: '4px', 
          background: '#fafafa'
        }}
      />
      <div style={{ display: 'flex', flexDirection: 'column', width: '280px', margin: '0.75rem auto 0', gap: '0.4rem' }}>
        <input 
          type="range" 
          min="0.1" 
          max="3.0" 
          step="0.05" 
          value={battScale} 
          onChange={(e) => setBattScale(parseFloat(e.target.value))}
          style={{ width: '100%', accentColor: '#2563eb', cursor: 'ew-resize' }}
        />
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.7rem', color: '#4b5563', fontWeight: 500 }}>
          <span>Nameplate Capacity: {Math.round(activePt.capMah)} mAh</span>
          <span>Sized Mass: {activePt.tMass.toFixed(2)} kg</span>
        </div>
      </div>
    </div>
  );
}

interface RationaleProps {
  payload_kg: number;
  transport_size: string;
  custom_wheelbase: boolean;
  custom_wheelbase_mm: number;
  mission_type: string;
  optimization_priority: string;
  use_range_override?: boolean;
  range_km?: number;
  hover_time_s?: number;
  climb_altitude_m?: number;
  optimal_speed_ms?: number;
}

function RationaleRenderer({
  payload_kg,
  transport_size,
  custom_wheelbase,
  custom_wheelbase_mm,
  mission_type,
  optimization_priority,
  use_range_override = false,
  range_km = 5.0,
  hover_time_s = 120,
  climb_altitude_m = 100,
  optimal_speed_ms
}: RationaleProps) {
  // Translate transport input to actual wheelbase limit
  const wheelbase_m = custom_wheelbase 
    ? custom_wheelbase_mm / 1000.0 
    : (transport_size === 'backpack' ? 0.35 : transport_size === 'trunk' ? 0.65 : 1.20);
  
  // Decide category for logic
  const transportCategory = custom_wheelbase
    ? (custom_wheelbase_mm <= 350 ? 'backpack' : custom_wheelbase_mm <= 650 ? 'trunk' : 'industrial')
    : transport_size;

  const getRotorRationale = () => {
    if (payload_kg > 5.0) {
      return `Sized as an Octocopter in Coaxial X8 layout (8 motors on 4 arms). Lifting a heavy ${payload_kg.toFixed(1)} kg payload within the ${(wheelbase_m).toFixed(2)}m transport limit requires maximum thrust density. Standard quadcopter propellers would exceed your size limit; coaxial stacking doubles thrust without expanding the footprint.`;
    }
    if (payload_kg > 1.5 || transportCategory === 'trunk') {
      const reason = payload_kg > 1.5 
        ? `carrying a medium payload of ${payload_kg.toFixed(1)} kg`
        : `operating within a medium ${(wheelbase_m).toFixed(2)}m transport limit`;
      return `Sized as a Hexacopter (6 motors) because ${reason} benefits from motor-out safety redundancy. If a motor fails, the remaining 5 can stabilize and land the drone safely, protecting your mission equipment.`;
    }
    return `Sized as a simple Quadcopter (4 motors). This is the most aerodynamically and mechanically efficient configuration for a light ${payload_kg.toFixed(1)} kg payload, minimizing structural empty weight and mechanical complexity.`;
  };

  const getBatteryRationale = () => {
    const roleCruiseSpeeds: Record<string, number> = {
      cinema: 10.0,
      imaging: 10.0,
      delivery: 14.0,
      inspection: 10.0,
      agriculture: 5.0,
      mapping: 12.0,
      racing: 25.0
    };
    const roleSpeed = roleCruiseSpeeds[mission_type] || 12.0;
    let v_forward = optimal_speed_ms !== undefined && optimal_speed_ms > 0 ? optimal_speed_ms : roleSpeed;
    if (optimal_speed_ms === undefined || optimal_speed_ms <= 0) {
      if (optimization_priority === 'range') {
        v_forward = 15.0;
      } else if (optimization_priority === 'flight_time' || optimization_priority === 'system_mass') {
        v_forward = Math.min(roleSpeed, 8.0);
      } else if (optimization_priority === 'payload') {
        v_forward = Math.min(roleSpeed, 12.0);
      }
    }

    let suffix = "";
    if (use_range_override) {
      const cruise_s = Math.round((range_km * 1000) / v_forward);
      const climb_s = Math.round(climb_altitude_m / 2.0);
      const total_s = cruise_s + climb_s + hover_time_s;
      suffix = ` This is optimized to meet your custom mission profile: ${range_km.toFixed(1)} km range (${cruise_s}s cruise at ${v_forward.toFixed(1)} m/s), ${climb_altitude_m}m climb altitude (${climb_s}s at 2 m/s), and ${hover_time_s}s hover (Total: ${total_s}s / ${(total_s/60.0).toFixed(1)} min).`;
    }

    // Determine the active battery chemistry
    let chemName = "Standard LiPo pouches (145 Wh/kg)";
    let reason = "to minimize build cost. While heavier than premium options, they provide standard C-rate discharge capacity at a lower price point.";
    
    const isPremium = (optimization_priority === 'flight_time' || optimization_priority === 'range' || optimization_priority === 'payload' || optimization_priority === 'system_mass');
    
    if (isPremium) {
      if (payload_kg <= 1.5) {
        chemName = "Lithium-Ion NMC chemistry (240 Wh/kg)";
        reason = "because your lightweight payload allows utilizing high-density cylindrical cells, maximizing specific energy capacity and flight duration.";
      } else {
        chemName = "LiPo Premium chemistry (250 Wh/kg)";
        reason = "because heavy-lift configurations draw high current where standard Li-ion cells would experience severe voltage sag and thermal runaway. It also provides the highest specific energy density among high-discharge cells, minimizing battery dead-weight.";
      }
    }

    return `Selected ${chemName} ${reason}${suffix}`;
  };

  const getMissionRationale = () => {
    if (mission_type === 'cinema') {
      return "Configured with ConsumerFolding airframe (1.05x structural mass penalty for folding mechanisms) and configured for Cinema & Photography payload role (5W camera stabilization gimbal power draw, low drag integration).";
    }
    if (mission_type === 'delivery') {
      return "Configured with EnterpriseRugged airframe (1.15x structural mass penalty for IP-sealed, drop-tested frame) and configured for Package Delivery payload role (10W cargo release mechanism power draw, medium aerodynamic drag).";
    }
    if (mission_type === 'agriculture') {
      return "Configured with Agricultural airframe (1.10x structural mass penalty for heavy spray gears/liquid tank) and configured for Agricultural Spraying payload role (25W high-flow pump power draw, high aerodynamic drag).";
    }
    if (mission_type === 'inspection') {
      return "Configured with EnterpriseRugged airframe (1.15x structural mass penalty for rugged IP-sealed chassis) and configured for Industrial Inspection payload role (10W sensor package power draw, medium aerodynamic drag).";
    }
    return "";
  };

  return (
    <div className="panel" style={{
      background: 'var(--surface)',
      border: '1px solid var(--border)',
      borderRadius: '6px',
      padding: '1.5rem',
      display: 'flex',
      flexDirection: 'column',
      gap: '1rem',
      height: 'fit-content'
    }}>
      <h3 style={{
        fontSize: '0.9rem',
        fontWeight: 700,
        textTransform: 'uppercase',
        letterSpacing: '0.05em',
        color: 'var(--accent)',
        margin: 0,
        borderBottom: '1px solid var(--border)',
        paddingBottom: '0.5rem'
      }}>
        System Design Decisions (Dynamic Rationale)
      </h3>
      <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem', fontSize: '0.85rem', lineHeight: '1.4' }}>
        <p style={{ margin: 0, color: 'var(--text-main)' }}>
          <strong style={{ color: 'var(--accent)' }}>Rotor Configuration:</strong><br />
          <span style={{ color: 'var(--text-muted)' }}>{getRotorRationale()}</span>
        </p>
        <p style={{ margin: 0, color: 'var(--text-main)' }}>
          <strong style={{ color: 'var(--accent)' }}>Power Plant:</strong><br />
          <span style={{ color: 'var(--text-muted)' }}>{getBatteryRationale()}</span>
        </p>
        <p style={{ margin: 0, color: 'var(--text-main)' }}>
          <strong style={{ color: 'var(--accent)' }}>Mission & Airframe:</strong><br />
          <span style={{ color: 'var(--text-muted)' }}>{getMissionRationale()}</span>
        </p>
      </div>
    </div>
  );
}

function ThermalHeatmap({ rotorCount, batteryTemp, armDiameter, coaxialLayout }: { rotorCount: number, batteryTemp: number, armDiameter: number, coaxialLayout: boolean }) {
  const armCount = coaxialLayout ? rotorCount / 2 : rotorCount;
  const arms: number[] = [];
  for (let i = 0; i < armCount; i++) {
    arms.push((i * 360) / armCount);
  }

  const getTempColor = (temp: number) => {
    if (temp < 40.0) return '#40c057'; // Soft Green
    if (temp < 55.0) return '#ffa94d'; // Soft Amber/Orange
    return '#ff6b6b'; // Crimson Red
  };

  const battColor = getTempColor(batteryTemp);
  const motorColor = getTempColor(batteryTemp * 1.25);
  const armThickness = Math.max(2.5, Math.min(8, armDiameter * 0.25));

  return (
    <div style={{ textAlign: 'center', fontFamily: 'Inter, sans-serif' }}>
      <h4 style={{ fontSize: '0.85rem', fontWeight: 600, marginBottom: '0.25rem', color: '#1f2937' }}>Thermal & Structural Schematic</h4>
      <p style={{ fontSize: '0.7rem', color: '#6b7280', marginBottom: '0.75rem' }}>Battery: {batteryTemp.toFixed(1)} °C · Motors: {(batteryTemp*1.25).toFixed(1)} °C</p>
      
      <div style={{ 
        width: '300px', 
        height: '300px', 
        border: '1px solid #e5e7eb', 
        borderRadius: '4px', 
        background: '#fafafa', 
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        margin: '0 auto',
        position: 'relative'
      }}>
        <svg width="260" height="260" viewBox="0 0 100 100" style={{ overflow: 'visible' }}>
          {/* Faint swept propeller disks */}
          {arms.map((angle, idx) => {
            const rad = (angle * Math.PI) / 180;
            const mx = 50 + 38 * Math.cos(rad);
            const my = 50 + 38 * Math.sin(rad);
            return (
              <g key={`prop-${idx}`} transform={`rotate(${angle}, ${mx}, ${my})`}>
                {/* Swept path outer limit */}
                <ellipse 
                  cx={mx} 
                  cy={my} 
                  rx="15" 
                  ry="3.2" 
                  fill="none" 
                  stroke="#9ca3af" 
                  strokeWidth="0.4" 
                  strokeDasharray="2,2"
                />
                {/* Thin halo contour */}
                <ellipse 
                  cx={mx} 
                  cy={my} 
                  rx="15.4" 
                  ry="3.5" 
                  fill="none" 
                  stroke="#e5e7eb" 
                  strokeWidth="0.25" 
                />
              </g>
            );
          })}

          {/* Structural arm carbon fiber tubes */}
          {arms.map((angle, idx) => {
            const rad = (angle * Math.PI) / 180;
            const x2 = 50 + 38 * Math.cos(rad);
            const y2 = 50 + 38 * Math.sin(rad);
            const armThicknessMax = armThickness;
            const innerThickness = Math.max(0.5, armThicknessMax - 1.2);
            return (
              <g key={`arm-${idx}`}>
                {/* CF Tube Walls */}
                <line 
                  x1="50" 
                  y1="50" 
                  x2={x2} 
                  y2={y2} 
                  stroke="#4b5563" 
                  strokeWidth={armThicknessMax} 
                  strokeLinecap="butt"
                />
                {/* Hollow core fill */}
                <line 
                  x1="50" 
                  y1="50" 
                  x2={x2} 
                  y2={y2} 
                  stroke="#fafafa" 
                  strokeWidth={innerThickness} 
                  strokeLinecap="butt"
                />
                {/* Technical dashed centerline (Cyan/Aqua CAD axis of symmetry) */}
                <line 
                  x1="50" 
                  y1="50" 
                  x2={x2} 
                  y2={y2} 
                  stroke="#06b6d4" 
                  strokeWidth="0.4" 
                  strokeDasharray="3,2"
                />
              </g>
            );
          })}

          {/* Central chassis deck hexagon (Machined plate with pocketing & bolts) */}
          <g>
            {/* Outer plate boundary */}
            <polygon 
              points="50,35 63,42.5 63,57.5 50,65 37,57.5 37,42.5" 
              fill="#f3f4f6" 
              stroke="#374151" 
              strokeWidth="0.8" 
            />
            {/* Inner pocket relief pattern */}
            <polygon 
              points="50,38 60,43.8 60,56.2 50,62 40,56.2 40,43.8" 
              fill="#fafafa" 
              stroke="#9ca3af" 
              strokeWidth="0.5" 
            />
            {/* Corner mounting bolt sockets */}
            <circle cx="50" cy="35" r="0.6" fill="#1f2937" />
            <circle cx="63" cy="42.5" r="0.6" fill="#1f2937" />
            <circle cx="63" cy="57.5" r="0.6" fill="#1f2937" />
            <circle cx="50" cy="65" r="0.6" fill="#1f2937" />
            <circle cx="37" cy="57.5" r="0.6" fill="#1f2937" />
            <circle cx="37" cy="42.5" r="0.6" fill="#1f2937" />
          </g>

          {/* Battery Core block with individual cells */}
          <g>
            <rect 
              x="44" 
              y="41" 
              width="12" 
              height="18" 
              rx="1"
              fill={battColor} 
              stroke="#1f2937" 
              strokeWidth="0.8"
              style={{ transition: 'fill 0.5s ease' }}
            />
            <line x1="44" y1="45" x2="56" y2="45" stroke="#1f2937" strokeWidth="0.4" />
            <line x1="44" y1="49" x2="56" y2="49" stroke="#1f2937" strokeWidth="0.4" />
            <line x1="44" y1="53" x2="56" y2="53" stroke="#1f2937" strokeWidth="0.4" />
            <line x1="44" y1="57" x2="56" y2="57" stroke="#1f2937" strokeWidth="0.4" />
            <text x="50" y="39" textAnchor="middle" fontSize="2.5" fill="#4b5563" fontWeight="bold" fontFamily="SFMono-Regular, Consolas, monospace">CORE BATT</text>
          </g>

          {/* Motors */}
          {arms.map((angle, idx) => {
            const rad = (angle * Math.PI) / 180;
            const mx = 50 + 38 * Math.cos(rad);
            const my = 50 + 38 * Math.sin(rad);
            return (
              <g key={`motor-${idx}`}>
                {/* Stator Outer housing */}
                <circle cx={mx} cy={my} r="4.2" fill={motorColor} stroke="#374151" strokeWidth="0.8" style={{ transition: 'fill 0.5s ease' }} />
                {/* Rotor Core assembly */}
                <circle cx={mx} cy={my} r="2.8" fill="none" stroke="#4b5563" strokeWidth="0.5" />
                {/* Motor Shaft spindle */}
                <circle cx={mx} cy={my} r="0.9" fill="#1f2937" stroke="#111827" strokeWidth="0.4" />
                {/* Coaxial upper motor outline projection */}
                {coaxialLayout && (
                  <circle cx={mx} cy={my} r="5.2" fill="none" stroke="#ef4444" strokeWidth="0.4" strokeDasharray="1,1" />
                )}
                {/* Mounting bolt point details */}
                <circle cx={mx - 1.8} cy={my - 1.8} r="0.25" fill="#374151" />
                <circle cx={mx + 1.8} cy={my - 1.8} r="0.25" fill="#374151" />
                <circle cx={mx - 1.8} cy={my + 1.8} r="0.25" fill="#374151" />
                <circle cx={mx + 1.8} cy={my + 1.8} r="0.25" fill="#374151" />
              </g>
            );
          })}

          {/* CAD Dimensioning Line for total motor spacing span */}
          <g>
            <line x1="12" y1="50" x2="12" y2="88" stroke="#9ca3af" strokeWidth="0.4" strokeDasharray="1,1" />
            <line x1="88" y1="50" x2="88" y2="88" stroke="#9ca3af" strokeWidth="0.4" strokeDasharray="1,1" />
            <line x1="12" y1="85" x2="88" y2="85" stroke="#4b5563" strokeWidth="0.5" />
            {/* Arrowheads */}
            <polygon points="12,85 15,83.5 15,86.5" fill="#4b5563" />
            <polygon points="88,85 85,83.5 85,86.5" fill="#4b5563" />
            {/* Value Overlay */}
            <rect x="34" y="83" width="32" height="4.5" fill="#fafafa" />
            <text 
              x="50" 
              y="86.2" 
              textAnchor="middle" 
              fontSize="2.8" 
              fill="#4b5563" 
              fontFamily="SFMono-Regular, Consolas, monospace"
              fontWeight="bold"
            >
              D_rotor = {(armDiameter * 24).toFixed(0)} mm
            </text>
          </g>

          {/* Sleek Monospace Legend in bottom left */}
          <g transform="translate(1, 80)">
            <rect x="0" y="0" width="38" height="18" fill="rgba(250,250,250,0.9)" rx="2" stroke="#e5e7eb" strokeWidth="0.5" />
            
            <circle cx="4" cy="4" r="1.5" fill="#3bc9db" />
            <text x="8" y="6.2" fontSize="2.4" fill="#4b5563" fontFamily="SFMono-Regular, Consolas, monospace">COLD (&lt;40°C)</text>
            
            <circle cx="4" cy="9" r="1.5" fill="#ffa94d" />
            <text x="8" y="11.2" fontSize="2.4" fill="#4b5563" fontFamily="SFMono-Regular, Consolas, monospace">WARM (40-55°C)</text>
            
            <circle cx="4" cy="14" r="1.5" fill="#ff6b6b" />
            <text x="8" y="16.2" fontSize="2.4" fill="#4b5563" fontFamily="SFMono-Regular, Consolas, monospace">HOT (&gt;55°C)</text>
          </g>
        </svg>
      </div>
      <div style={{ height: '1.25rem' }} />
    </div>
  );
}

const getFlightTimeDisplay = (approvedS: string, relaxation: any, phase: 'Hover' | 'Forward' | 'Climb') => {
  const appS = parseFloat(approvedS);
  if (isNaN(appS)) return 'N/A';
  const approvedMin = (appS / 60).toFixed(1);
  if (relaxation) {
    const reqS = parseFloat(phase === 'Hover' ? relaxation.requestedHover : phase === 'Forward' ? relaxation.requestedForward : relaxation.requestedClimb);
    if (!isNaN(reqS) && appS < reqS - 0.5) {
      const requestedMin = (reqS / 60).toFixed(1);
      if (relaxation.isBatteryLocked) {
        return `${approvedMin} min (Reduced from ${requestedMin} min due to locked battery capacity)`;
      }
      return `${approvedMin} min (Reduced from ${requestedMin} min due to thermal limits)`;
    }
  }
  return `${approvedMin} min`;
};

export default function Home() {
  const [loading, setLoading] = useState(false);
  const [result, setResult] = useState<any>(null);
  const [selectedRole, setSelectedRole] = useState('imaging');
  const [submittedParams, setSubmittedParams] = useState<any>(null);
  const [copied, setCopied] = useState(false);
  const [shareLinkError, setShareLinkError] = useState<string | null>(null);
  const isHydratingShareLink = useRef(false);

  // Database customizer states
  const [cotsDb, setCotsDb] = useState<any>(null);
  const [chemistryDb, setChemistryDb] = useState<any>(null);
  const [escDb, setEscDb] = useState<any>(null);
  const [dbStatus, setDbStatus] = useState<string | null>(null);
  const [activeDbTab, setActiveDbTab] = useState<'cots' | 'chemistry' | 'esc'>('cots');

  // Load databases on mount
  const fetchDbs = async () => {
    const res = await getDatabases();
    if (res.success) {
      setCotsDb(res.cots);
      setChemistryDb(res.chemistry);
      setEscDb(res.esc);
    } else {
      console.error('Failed to load databases:', res.error);
    }
  };

  useEffect(() => {
    fetchDbs();
  }, []);

  // Advanced workspace form states for rotor count and coaxial layout compatibility
  const [formRotorCount, setFormRotorCount] = useState("4");
  const [formCoaxial, setFormCoaxial] = useState(false);

  // Dual-mode state variables
  const [isKmode, setIsKmode] = useState(true);
  const [kmodePayload, setKmodePayload] = useState(1.5);

  const renderMassBreakdown = (res: any) => {
    if (!res) return null;
    const payload = submittedParams?.payload || (isKmode ? kmodePayload : 1.5);
    const total = parseFloat(res.mass) || 1.5;
    const frame = parseFloat(res.frame) || 0.0;
    const battery = parseFloat(res.battery) || 0.0;
    const other = Math.max(0, total - payload - frame - battery);
    
    const pct = (val: number) => total > 0 ? ((val / total) * 100).toFixed(1) : '0.0';
    
    return (
      <div style={{ marginTop: '0.5rem', marginBottom: '1.5rem', borderBottom: '1px dashed var(--border)', paddingBottom: '1rem' }}>
        <div className="result-row" style={{ fontWeight: 600, borderBottom: '1px solid var(--border)', paddingBottom: '0.5rem', marginBottom: '0.5rem' }}>
          <span className="result-label">Total Mass (MTOW)</span>
          <span className="result-value">{total.toFixed(2)} kg (100%)</span>
        </div>
        <div className="result-row" style={{ paddingLeft: '0.75rem', fontSize: '0.9rem' }}>
          <span className="result-label">↳ Payload Mass</span>
          <span className="result-value">{payload.toFixed(2)} kg ({pct(payload)}%)</span>
        </div>
        <div className="result-row" style={{ paddingLeft: '0.75rem', fontSize: '0.9rem' }}>
          <span className="result-label">↳ Carbon Frame</span>
          <span className="result-value">{frame.toFixed(2)} kg ({pct(frame)}%)</span>
        </div>
        <div className="result-row" style={{ paddingLeft: '0.75rem', fontSize: '0.9rem' }}>
          <span className="result-label">↳ Battery Mass</span>
          <span className="result-value">{battery.toFixed(2)} kg ({pct(battery)}%)</span>
        </div>
        <div className="result-row" style={{ paddingLeft: '0.75rem', fontSize: '0.9rem', marginBottom: res.motor_mass_kg !== undefined ? '0.25rem' : 0, paddingBottom: 0 }}>
          <span className="result-label">↳ Propulsion & System</span>
          <span className="result-value">{other.toFixed(2)} kg ({pct(other)}%)</span>
        </div>
        {res.motor_mass_kg !== undefined && (
          <div style={{ paddingLeft: '1.5rem', fontSize: '0.8rem', color: '#4b5563', display: 'flex', flexDirection: 'column', gap: '4px', borderLeft: '2px solid var(--border)', marginLeft: '1rem', marginTop: '0.1rem', marginBottom: '0.25rem' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between' }}>
              <span>• Motors</span>
              <span>{res.motor_mass_kg.toFixed(3)} kg</span>
            </div>
            <div style={{ display: 'flex', justifyContent: 'space-between' }}>
              <span>• Wiring & Cabling</span>
              <span>{res.wiring_mass_kg.toFixed(3)} kg</span>
            </div>
            <div style={{ display: 'flex', justifyContent: 'space-between' }}>
              <span>• ESC Housing & Power</span>
              <span>{res.esc_mass_kg.toFixed(3)} kg</span>
            </div>
            <div style={{ display: 'flex', justifyContent: 'space-between' }}>
              <span>• Structural Complexity</span>
              <span>{res.complexity_mass_kg.toFixed(3)} kg</span>
            </div>
            <div style={{ display: 'flex', justifyContent: 'space-between' }}>
              <span>• Avionics & Sensors</span>
              <span>{res.avionics_mass_kg.toFixed(3)} kg</span>
            </div>
            {res.role_aux_mass_kg > 0.0 && (
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span>• Role-Specific Aux Mounts</span>
                <span>{res.role_aux_mass_kg.toFixed(3)} kg</span>
              </div>
            )}
          </div>
        )}
      </div>
    );
  };
  const [kmodeTransport, setKmodeTransport] = useState('trunk');
  const [useCustomWheelbase, setUseCustomWheelbase] = useState(false);
  const [customWheelbaseMm, setCustomWheelbaseMm] = useState(650);
  const [kmodeMission, setKmodeMission] = useState('cinema');
  const [kmodePriority, setKmodePriority] = useState('flight_time');
  const [useRangeOverride, setUseRangeOverride] = useState(false);
  const [kmodeRangeKm, setKmodeRangeKm] = useState(5.0);
  const [kmodeHoverTimeS, setKmodeHoverTimeS] = useState(120);
  const [kmodeClimbAltitudeM, setKmodeClimbAltitudeM] = useState(100);

  const handleModeToggle = (toKmode: boolean) => {
    setIsKmode(toKmode);
  };

  // Senior Developer Comment: 
  // This synchronization side-effect maps high-level user mission requirements (Mission Workspace) 
  // into physical/technical drone design parameters (Physics Workspace). 
  // It reactively calculates rotor configuration, optimal battery chemistry, motor speed, and sizing envelopes 
  // in real-time as the user adjusts KMODE requirements, ensuring both that the UI results card 
  // and the hidden form submission inputs stay fully unified.
  useEffect(() => {
    if (isKmode) {
      if (isHydratingShareLink.current) {
        return;
      }
      // Graduating to Advanced Workspace: Compile Kmode inputs into technical variables
      const payload = kmodePayload;
      const p = translateKmodeToParams({
        payload: kmodePayload,
        transport: kmodeTransport,
        mission: kmodeMission,
        priority: kmodePriority,
        useCustomWheelbase,
        customWheelbaseMm,
        useRangeOverride,
        kmodeRangeKm,
        kmodeHoverTimeS,
        kmodeClimbAltitudeM
      });
      const {
        rotor_count, coaxial_layout, airframe_class, role_type,
        battery_chemistry, battery_specific_energy_wh_kg, max_diameter_m,
        aux_power_w, v_forward, t_hover, t_forward, t_climb, altitude
      } = p;

      // Update related React states first
      setSelectedRole(role_type);
      setFormRotorCount(String(rotor_count));
      setFormCoaxial(coaxial_layout);

      // Hydrate DOM inputs immediately
      const form = document.getElementById('engine-form') as HTMLFormElement | null;
      if (form) {
        const updates: Record<string, string | boolean> = {
          payload_kg: String(payload),
          max_diameter_m: String(max_diameter_m),
          rotor_count: String(rotor_count),
          coaxial_layout: coaxial_layout,
          airframe_class: airframe_class,
          battery_chemistry: battery_chemistry,
          battery_specific_energy_wh_kg: String(battery_specific_energy_wh_kg),
          v_forward_ms: String(v_forward),
          altitude_m: String(altitude),
          ambient_temp_c: '20.0',
          t_hover_s: String(t_hover),
          t_forward_s: String(t_forward),
          t_climb_s: String(t_climb),
          aux_power_w: String(aux_power_w)
        };

        Object.keys(updates).forEach(key => {
          const input = form.elements.namedItem(key) as HTMLInputElement | HTMLSelectElement | null;
          if (input) {
            if (input.type === 'checkbox') {
              (input as HTMLInputElement).checked = updates[key] === true || updates[key] === 'true';
            } else {
              input.value = String(updates[key]);
            }
          }
        });
      }
    }
  }, [
    isKmode,
    kmodePayload,
    kmodeTransport,
    kmodeMission,
    kmodePriority,
    useCustomWheelbase,
    customWheelbaseMm,
    useRangeOverride,
    kmodeHoverTimeS,
    kmodeRangeKm,
    kmodeClimbAltitudeM
  ]);

  const submitDecoded = async (data: any) => {
    setLoading(true);
    setResult(null);
    
    const rc = parseInt(data.rotor_count) || 4;
    const sanitizedCoaxial = (rc === 8) && (data.coaxial_layout === 'true' || data.coaxial_layout === true);
    const formData = new FormData();
    Object.keys(data).forEach(key => {
      if (key === 'coaxial_layout') {
        formData.append(key, String(sanitizedCoaxial));
      } else {
        formData.append(key, String(data[key]));
      }
    });
    
    const chemistry = data.battery_chemistry || 'LiPo_premium';
    const specEnergyRaw = data.battery_specific_energy_wh_kg;
    let specEnergy = parseFloat(specEnergyRaw);
    if (isNaN(specEnergy) || specEnergy <= 0) {
      const chemistrySpecEnergies: any = {
        LiPo: 180,
        LiPo_standard: 180,
        LiPo_highvoltage: 220,
        LiPo_premium: 250,
        LiHV: 195,
        'Li-ion NMC': 240,
        'Li-ion NCA': 260,
        'Solid State': 400,
        'Li-S': 500
      };
      specEnergy = chemistrySpecEnergies[chemistry] || 180;
    }
    
    setSubmittedParams({
      specEnergy,
      rotorCount: rc,
      coaxialLayout: sanitizedCoaxial,
      payload: parseFloat(data.payload_kg) || 1.5
    });
    
    const res = await runEngine(formData);
    setResult(res);
    setLoading(false);
  };

  useEffect(() => {
    const query = window.location.search;
    const params = new URLSearchParams(query);
    const buildData = params.get('build');
    if (buildData) {
      try {
        isHydratingShareLink.current = true;
        const decoded = JSON.parse(decodeURIComponent(escape(window.atob(buildData))));
        
        if (!decoded || typeof decoded !== 'object') {
          throw new Error("Parsed build parameter is not a valid object");
        }
        
        const hasRequiredKeys = ['payload_kg', 'max_diameter_m', 'rotor_count'].every(k => k in decoded);
        if (!hasRequiredKeys) {
          throw new Error("Missing key configuration fields in build parameter");
        }
        
        // Auto switch to Advanced mode when loading a custom build query link
        setIsKmode(false);

        if (decoded.role_type) {
          setSelectedRole(decoded.role_type);
        }
        const rc = decoded.rotor_count ? parseInt(String(decoded.rotor_count)) : 4;
        setFormRotorCount(String(rc));
        const coax = decoded.coaxial_layout === true || decoded.coaxial_layout === 'true';
        const sanitizedCoaxial = rc === 8 && coax;
        setFormCoaxial(sanitizedCoaxial);
        
        // Wait slightly for the role custom inputs to mount before populating them
        setTimeout(() => {
          const form = document.getElementById('engine-form') as HTMLFormElement | null;
          if (form) {
            Object.keys(decoded).forEach(key => {
              const input = form.elements.namedItem(key) as HTMLInputElement | HTMLSelectElement | null;
              if (input) {
                if (input.type === 'checkbox') {
                  if (key === 'coaxial_layout') {
                    (input as HTMLInputElement).checked = sanitizedCoaxial;
                  } else {
                    (input as HTMLInputElement).checked = decoded[key] === true || decoded[key] === 'true';
                  }
                } else {
                  input.value = String(decoded[key]);
                }
              }
            });
          }
        }, 80);
        
        submitDecoded(decoded);
      } catch (e: any) {
        console.error("Malformed shareable URL. Safely falling back to baseline parameters:", e);
        setShareLinkError("Malformed or truncated shareable build URL. Safely loaded baseline parameters.");
      }
    }
  }, []);

  const copyShareableLink = () => {
    let obj: any = {};
    if (isKmode) {
      // Requirements-to-Parameter translation matrix
      const p = translateKmodeToParams({
        payload: kmodePayload,
        transport: kmodeTransport,
        mission: kmodeMission,
        priority: kmodePriority,
        useCustomWheelbase,
        customWheelbaseMm,
        useRangeOverride,
        kmodeRangeKm,
        kmodeHoverTimeS,
        kmodeClimbAltitudeM
      });

      obj = {
        payload_kg: kmodePayload,
        max_diameter_m: p.max_diameter_m,
        added_drag_m2: 0.02,
        cg_shift_m: 0.0,
        rotor_count: p.rotor_count,
        coaxial_layout: p.coaxial_layout,
        airframe_class: p.airframe_class,
        role_type: p.role_type,
        v_forward_ms: p.v_forward,
        t_hover_s: p.t_hover,
        t_forward_s: p.t_forward,
        t_climb_s: p.t_climb,
        altitude_m: p.altitude,
        battery_chemistry: p.battery_chemistry,
        battery_specific_energy_wh_kg: p.battery_specific_energy_wh_kg,
        auto_reduce_flight_time: true,
        aux_power_w: p.aux_power_w
      };
    } else {
      const form = document.getElementById('engine-form') as HTMLFormElement | null;
      if (!form) return;
      
      const formData = new FormData(form);
      formData.forEach((value, key) => {
        obj[key] = value;
      });
      
      const inputs = form.querySelectorAll('input, select');
      inputs.forEach((input: any) => {
        if (input.name) {
          if (input.type === 'checkbox') {
            obj[input.name] = input.checked;
          } else {
            obj[input.name] = input.value;
          }
        }
      });
      obj['role_type'] = selectedRole;
    }
    
    try {
      const str = window.btoa(unescape(encodeURIComponent(JSON.stringify(obj))));
      const shareUrl = `${window.location.origin}${window.location.pathname}?build=${str}`;
      navigator.clipboard.writeText(shareUrl);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (e) {
      console.error(e);
    }
  };

  const handleSubmit = async (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    setLoading(true);
    setResult(null);
    
    let submissionFormData: FormData;
    let specEnergy = 180.0;
    let rotorCount = 4;
    let coaxialLayout = false;

    let payloadVal = 1.5;
    if (isKmode) {
      payloadVal = kmodePayload;
      const payload = kmodePayload;
      const priority = kmodePriority;
      const p = translateKmodeToParams({
        payload: kmodePayload,
        transport: kmodeTransport,
        mission: kmodeMission,
        priority: kmodePriority,
        useCustomWheelbase,
        customWheelbaseMm,
        useRangeOverride,
        kmodeRangeKm,
        kmodeHoverTimeS,
        kmodeClimbAltitudeM
      });
      rotorCount = p.rotor_count;
      coaxialLayout = p.coaxial_layout;
      specEnergy = p.battery_specific_energy_wh_kg;

      submissionFormData = new FormData();
      submissionFormData.append('payload_kg', String(payload));
      submissionFormData.append('max_diameter_m', String(p.max_diameter_m));
      submissionFormData.append('added_drag_m2', '0.02');
      submissionFormData.append('cg_shift_m', '0.0');
      submissionFormData.append('rotor_count', String(rotorCount));
      submissionFormData.append('coaxial_layout', String(coaxialLayout));
      submissionFormData.append('airframe_class', p.airframe_class);
      submissionFormData.append('role_type', p.role_type);
      submissionFormData.append('v_forward_ms', String(p.v_forward));
      submissionFormData.append('is_kmode', 'true');
      submissionFormData.append('optimization_priority', priority);
      submissionFormData.append('t_hover_s', String(p.t_hover));
      submissionFormData.append('t_forward_s', String(p.t_forward));
      submissionFormData.append('t_climb_s', String(p.t_climb));
      submissionFormData.append('altitude_m', String(p.altitude));
      submissionFormData.append('battery_chemistry', p.battery_chemistry);
      submissionFormData.append('battery_specific_energy_wh_kg', String(specEnergy));
      submissionFormData.append('auto_reduce_flight_time', 'true');
      submissionFormData.append('aux_power_w', String(p.aux_power_w));
      submissionFormData.append('ambient_temp_c', '20.0');
      submissionFormData.append('v_climb_ms', '2.0');
      submissionFormData.append('battery_cycle_count', '0');
      submissionFormData.append('is_voltage_locked', 'false');
      submissionFormData.append('use_range_override', String(useRangeOverride));
      submissionFormData.append('range_km', String(kmodeRangeKm));
      submissionFormData.append('locked_series_cells', '6');
      submissionFormData.append('figure_of_merit_mode', 'installed');
      submissionFormData.append('figure_of_merit_isolated', '-1.0');
      submissionFormData.append('arm_config', 'under_rotor');
    } else {
      submissionFormData = new FormData(e.currentTarget);
      submissionFormData.append('role_type', selectedRole);
      payloadVal = parseFloat(submissionFormData.get('payload_kg') as string) || 1.5;

      const chemistry = submissionFormData.get('battery_chemistry') as string;
      const specEnergyRaw = submissionFormData.get('battery_specific_energy_wh_kg') as string;
      let parsedSpecEnergy = parseFloat(specEnergyRaw);
      if (isNaN(parsedSpecEnergy) || parsedSpecEnergy <= 0) {
        const chemistrySpecEnergies: any = {
          LiPo: 180,
          LiPo_standard: 180,
          LiPo_highvoltage: 220,
          LiPo_premium: 250,
          LiHV: 195,
          'Li-ion NMC': 240,
          'Li-ion NCA': 260,
          'Solid State': 400,
          'Li-S': 500
        };
        parsedSpecEnergy = chemistrySpecEnergies[chemistry] || 180;
      }
      specEnergy = parsedSpecEnergy;
      rotorCount = parseInt(submissionFormData.get('rotor_count') as string) || 4;
      coaxialLayout = (rotorCount === 8) && (submissionFormData.get('coaxial_layout') === 'true');
      submissionFormData.set('coaxial_layout', String(coaxialLayout));
    }
    
    setSubmittedParams({
      specEnergy,
      rotorCount,
      coaxialLayout,
      payload: payloadVal
    });
    
    const res = await runEngine(submissionFormData);
    setResult(res);
    setLoading(false);
  };

  return (
    <div className="dashboard">
      {/* Header */}
      <div className="panel" style={{ paddingBottom: '1rem' }}>
        <h1 className="brand-title">AeroEval</h1>
        <p style={{ color: 'var(--text-muted)' }}>Configure payload constraints and mission parameters to simulate fixed-point mass convergence.</p>
      </div>

      {shareLinkError && (
        <div style={{
          background: '#f8d7da',
          color: '#842029',
          border: '1px solid #f5c2c7',
          padding: '1rem 1.5rem',
          borderRadius: '6px',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          boxShadow: 'var(--shadow-sm)'
        }}>
          <span style={{ fontWeight: 500, fontSize: '0.9rem' }}>⚠ {shareLinkError}</span>
          <button 
            type="button"
            onClick={() => setShareLinkError(null)} 
            style={{ 
              background: 'transparent', 
              border: 'none', 
              color: '#842029', 
              cursor: 'pointer', 
              fontWeight: 'bold',
              fontSize: '1.2rem',
              lineHeight: 1
            }}
          >
            ×
          </button>
        </div>
      )}

      {/* Mode Switcher Banner */}
      <div style={{
        display: 'flex',
        background: 'var(--surface-hover)',
        border: '1px solid var(--border)',
        borderRadius: '6px',
        padding: '0.25rem',
        marginBottom: '2rem',
        alignSelf: 'center',
        width: 'fit-content',
        gap: '0.25rem',
        margin: '0 auto 2rem'
      }}>
        <button
          type="button"
          onClick={() => handleModeToggle(true)}
          style={{
            padding: '0.5rem 1.25rem',
            fontSize: '0.85rem',
            fontWeight: 600,
            borderRadius: '4px',
            border: 'none',
            background: isKmode ? 'var(--surface)' : 'transparent',
            color: isKmode ? 'var(--accent)' : 'var(--text-muted)',
            cursor: 'pointer',
            boxShadow: isKmode ? 'var(--shadow-sm)' : 'none',
            transition: 'all 0.15s ease-in-out'
          }}
        >
          Mission Workspace
        </button>
        <button
          type="button"
          onClick={() => handleModeToggle(false)}
          style={{
            padding: '0.5rem 1.25rem',
            fontSize: '0.85rem',
            fontWeight: 600,
            borderRadius: '4px',
            border: 'none',
            background: !isKmode ? 'var(--surface)' : 'transparent',
            color: !isKmode ? 'var(--accent)' : 'var(--text-muted)',
            cursor: 'pointer',
            boxShadow: !isKmode ? 'var(--shadow-sm)' : 'none',
            transition: 'all 0.15s ease-in-out'
          }}
        >
          Physics Workspace
        </button>
      </div>

      <form id="engine-form" onSubmit={handleSubmit}>
        {isKmode ? (
          <div style={{
            display: 'grid',
            gridTemplateColumns: 'repeat(auto-fit, minmax(320px, 1fr))',
            gap: '2rem',
            maxWidth: '1200px',
            margin: '0 auto'
          }}>
            <div className="panel" style={{ display: 'flex', flexDirection: 'column', gap: '2rem' }}>
              <h2 className="section" style={{ marginBottom: '1rem', borderBottom: '1px dashed var(--border)', paddingBottom: '1rem' }}>Mission Requirements</h2>
              
              {/* 1. Payload */}
              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                  <label style={{ fontSize: '0.9rem', fontWeight: 600, color: 'var(--text-main)' }}>What is your payload mass?</label>
                  <span style={{ fontSize: '1rem', fontWeight: 700, color: 'var(--accent)', fontFamily: 'SFMono-Regular, Consolas, monospace' }}>
                    {kmodePayload.toFixed(2)} kg
                  </span>
                </div>
                <input
                  type="range"
                  name="payload_kg"
                  min="0.1"
                  max="25.0"
                  step="0.05"
                  value={kmodePayload}
                  onChange={(e) => setKmodePayload(parseFloat(e.target.value))}
                  style={{ width: '100%', accentColor: 'var(--accent)', cursor: 'ew-resize' }}
                />

                {/* Quick Presets */}
                <div style={{ display: 'flex', flexWrap: 'wrap', gap: '0.5rem', marginTop: '0.25rem' }}>
                  {[
                    { name: 'GoPro', weight: 0.15 },
                    { name: 'Thermal Gimbal', weight: 0.8 },
                    { name: 'DSLR Camera', weight: 1.5 },
                    { name: '10L Spray Tank', weight: 10.0 }
                  ].map(preset => (
                    <button
                      key={preset.name}
                      type="button"
                      onClick={() => setKmodePayload(preset.weight)}
                      style={{
                        padding: '0.35rem 0.75rem',
                        fontSize: '0.75rem',
                        fontWeight: 500,
                        borderRadius: '20px',
                        border: '1px solid var(--border)',
                        background: kmodePayload === preset.weight ? 'var(--accent)' : 'var(--surface)',
                        color: kmodePayload === preset.weight ? '#ffffff' : 'var(--text-muted)',
                        cursor: 'pointer',
                        transition: 'all 0.15s ease'
                      }}
                      onMouseEnter={(e) => {
                        if (kmodePayload !== preset.weight) {
                          e.currentTarget.style.background = 'var(--surface-hover)';
                        }
                      }}
                      onMouseLeave={(e) => {
                        if (kmodePayload !== preset.weight) {
                          e.currentTarget.style.background = 'var(--surface)';
                        }
                      }}
                    >
                      {preset.name} ({preset.weight}kg)
                    </button>
                  ))}
                </div>
              </div>

              {/* 2. Transport Size */}
              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
                <label style={{ fontSize: '0.9rem', fontWeight: 600, color: 'var(--text-main)' }}>What is your transport / size limit?</label>
                <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: '0.5rem' }}>
                  {[
                    { id: 'backpack', name: 'Backpack', desc: 'Wheelbase ≤ 0.35m' },
                    { id: 'trunk', name: 'Car Trunk', desc: 'Wheelbase ≤ 0.65m' },
                    { id: 'industrial', name: 'Industrial', desc: 'Wheelbase ≤ 1.20m' }
                  ].map(item => (
                    <button
                      key={item.id}
                      type="button"
                      onClick={() => {
                        setKmodeTransport(item.id);
                        if (useCustomWheelbase) {
                          if (item.id === 'backpack') setCustomWheelbaseMm(350);
                          else if (item.id === 'trunk') setCustomWheelbaseMm(650);
                          else if (item.id === 'industrial') setCustomWheelbaseMm(1200);
                        }
                      }}
                      style={{
                        display: 'flex',
                        flexDirection: 'column',
                        alignItems: 'center',
                        padding: '0.75rem 0.5rem',
                        borderRadius: '6px',
                        border: kmodeTransport === item.id ? '2px solid var(--accent)' : '1px solid var(--border)',
                        background: 'var(--surface)',
                        color: kmodeTransport === item.id ? 'var(--accent)' : 'var(--text-main)',
                        cursor: 'pointer',
                        transition: 'all 0.15s ease',
                        textAlign: 'center'
                      }}
                    >
                      <span style={{ fontSize: '0.85rem', fontWeight: 600 }}>{item.name}</span>
                      <span style={{ fontSize: '0.65rem', color: 'var(--text-muted)', marginTop: '0.25rem' }}>{item.desc}</span>
                    </button>
                  ))}
                </div>

                {/* Custom Wheelbase Slider Option */}
                <div style={{ marginTop: '0.5rem', display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                  <label style={{ display: 'inline-flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer', fontSize: '0.8rem', color: 'var(--text-muted)' }}>
                    <input
                      type="checkbox"
                      checked={useCustomWheelbase}
                      onChange={(e) => {
                        setUseCustomWheelbase(e.target.checked);
                        if (e.target.checked) {
                          if (kmodeTransport === 'backpack') setCustomWheelbaseMm(350);
                          else if (kmodeTransport === 'trunk') setCustomWheelbaseMm(650);
                          else if (kmodeTransport === 'industrial') setCustomWheelbaseMm(1200);
                        }
                      }}
                      style={{ accentColor: 'var(--accent)' }}
                    />
                    <span>Enable Custom Size Override</span>
                  </label>
                  
                  {useCustomWheelbase && (
                    <div style={{
                      display: 'flex',
                      flexDirection: 'column',
                      gap: '0.5rem',
                      padding: '0.75rem',
                      background: 'var(--surface-hover)',
                      borderRadius: '6px',
                      border: '1px solid var(--border)',
                      marginTop: '0.25rem'
                    }}>
                      <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.8rem', fontWeight: 500 }}>
                        <span style={{ color: 'var(--text-muted)' }}>Wheelbase Limit:</span>
                        <span style={{ color: 'var(--accent)', fontWeight: 600 }}>
                          {customWheelbaseMm} mm ({(customWheelbaseMm / 1000).toFixed(2)} m)
                        </span>
                      </div>
                      <input
                        type="range"
                        min="200"
                        max="2000"
                        step="10"
                        value={customWheelbaseMm}
                        onChange={(e) => setCustomWheelbaseMm(parseInt(e.target.value))}
                        style={{ width: '100%', accentColor: 'var(--accent)', cursor: 'ew-resize' }}
                      />
                    </div>
                  )}
                </div>
              </div>

              {/* 3. Primary Mission Type */}
              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
                <label style={{ fontSize: '0.9rem', fontWeight: 600, color: 'var(--text-main)' }}>What is your primary mission type?</label>
                <select
                  value={kmodeMission}
                  onChange={(e) => setKmodeMission(e.target.value)}
                  style={{
                    width: '100%',
                    padding: '0.75rem',
                    borderRadius: '6px',
                    border: '1px solid var(--border)',
                    background: 'var(--surface)',
                    color: 'var(--text-main)',
                    fontSize: '0.9rem',
                    fontWeight: 500,
                    outline: 'none',
                    cursor: 'pointer'
                  }}
                >
                  <option value="cinema">Cinema & Photography</option>
                  <option value="delivery">Package Delivery</option>
                  <option value="agriculture">Agricultural Spraying</option>
                  <option value="inspection">Industrial Inspection</option>
                </select>
              </div>

              {/* Custom Mission Profile (Range & Hover) Override */}
              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem', borderTop: '1px dashed var(--border)', paddingTop: '1rem' }}>
                <label style={{ display: 'inline-flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer', fontSize: '0.9rem', fontWeight: 600, color: 'var(--text-main)' }}>
                  <input
                    type="checkbox"
                    checked={useRangeOverride}
                    onChange={(e) => setUseRangeOverride(e.target.checked)}
                    style={{ accentColor: 'var(--accent)' }}
                  />
                  <span>Enable Range & Hover Time Override</span>
                </label>

                {useRangeOverride && (
                  <div style={{
                    display: 'flex',
                    flexDirection: 'column',
                    gap: '1rem',
                    padding: '1rem',
                    background: 'var(--surface-hover)',
                    borderRadius: '6px',
                    border: '1px solid var(--border)'
                  }}>
                    {/* Range Slider */}
                    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.35rem' }}>
                      <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.8rem', fontWeight: 500 }}>
                        <span style={{ color: 'var(--text-muted)' }}>Mission Range:</span>
                        <span style={{ color: 'var(--accent)', fontWeight: 600 }}>{kmodeRangeKm.toFixed(1)} km</span>
                      </div>
                      <input
                        type="range"
                        min="0.5"
                        max="30.0"
                        step="0.5"
                        value={kmodeRangeKm}
                        onChange={(e) => setKmodeRangeKm(parseFloat(e.target.value))}
                        style={{ width: '100%', accentColor: 'var(--accent)', cursor: 'ew-resize' }}
                      />
                    </div>

                    {/* Hover Time Slider */}
                    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.35rem' }}>
                      <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.8rem', fontWeight: 500 }}>
                        <span style={{ color: 'var(--text-muted)' }}>Additional Hover Time:</span>
                        <span style={{ color: 'var(--accent)', fontWeight: 600 }}>{kmodeHoverTimeS} s ({Math.floor(kmodeHoverTimeS / 60)} min {kmodeHoverTimeS % 60} s)</span>
                      </div>
                      <input
                        type="range"
                        min="0"
                        max="1200"
                        step="10"
                        value={kmodeHoverTimeS}
                        onChange={(e) => setKmodeHoverTimeS(parseInt(e.target.value))}
                        style={{ width: '100%', accentColor: 'var(--accent)', cursor: 'ew-resize' }}
                      />
                    </div>

                    {/* Climb Altitude Slider */}
                    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.35rem' }}>
                      <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.8rem', fontWeight: 500 }}>
                        <span style={{ color: 'var(--text-muted)' }}>Climb Altitude:</span>
                        <span style={{ color: 'var(--accent)', fontWeight: 600 }}>{kmodeClimbAltitudeM} m</span>
                      </div>
                      <input
                        type="range"
                        min="10"
                        max="300"
                        step="10"
                        value={kmodeClimbAltitudeM}
                        onChange={(e) => setKmodeClimbAltitudeM(parseInt(e.target.value))}
                        style={{ width: '100%', accentColor: 'var(--accent)', cursor: 'ew-resize' }}
                      />
                    </div>

                    {/* Calculated Summary */}
                    <div style={{
                      marginTop: '0.5rem',
                      padding: '0.75rem',
                      background: 'var(--surface)',
                      border: '1px solid var(--border)',
                      borderRadius: '4px',
                      fontSize: '0.75rem',
                      color: 'var(--text-muted)',
                      display: 'flex',
                      flexDirection: 'column',
                      gap: '0.25rem'
                    }}>
                      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                        <span>Cruise Speed (multirotor default):</span>
                        <span style={{ fontWeight: 600, color: 'var(--text-main)' }}>15.0 m/s (54.0 km/h)</span>
                      </div>
                      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                        <span>Calculated Cruise Time:</span>
                        <span style={{ fontWeight: 600, color: 'var(--text-main)' }}>{Math.round((kmodeRangeKm * 1000) / 15.0)} s ({(kmodeRangeKm * 1000 / 15.0 / 60.0).toFixed(1)} min)</span>
                      </div>
                      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                        <span>Calculated Climb Time (at 2 m/s):</span>
                        <span style={{ fontWeight: 600, color: 'var(--text-main)' }}>{Math.round(kmodeClimbAltitudeM / 2.0)} s ({kmodeClimbAltitudeM / 2.0} s)</span>
                      </div>
                      <div style={{ display: 'flex', justifyContent: 'space-between', borderTop: '1px dashed var(--border)', paddingTop: '0.25rem', marginTop: '0.25rem', fontWeight: 600 }}>
                        <span style={{ color: 'var(--accent)' }}>Total Sizing Flight Time:</span>
                        <span style={{ color: 'var(--accent)' }}>
                          {Math.round((kmodeRangeKm * 1000) / 15.0) + Math.round(kmodeClimbAltitudeM / 2.0) + kmodeHoverTimeS} s ({((Math.round((kmodeRangeKm * 1000) / 15.0) + Math.round(kmodeClimbAltitudeM / 2.0) + kmodeHoverTimeS) / 60.0).toFixed(1)} min)
                        </span>
                      </div>
                    </div>
                  </div>
                )}
              </div>

              {/* 4. Optimization Priority */}
              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
                <label style={{ fontSize: '0.9rem', fontWeight: 600, color: 'var(--text-main)' }}>Optimization Priority</label>
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.5rem' }}>
                  {[
                    { id: 'flight_time', name: 'Maximum Endurance', desc: 'Optimize chemistry & capacity for hover duration' },
                    { id: 'range', name: 'Maximum Range', desc: 'Optimize cruising speed & specific energy for distance' },
                    { id: 'payload', name: 'Maximum Payload', desc: 'Maximize lift capacity with high-power battery chemistry' },
                    { id: 'system_mass', name: 'Minimum System Mass', desc: 'Minimize empty weight with lightweight frame classes' }
                  ].map(item => (
                    <button
                      key={item.id}
                      type="button"
                      onClick={() => setKmodePriority(item.id)}
                      style={{
                        display: 'flex',
                        flexDirection: 'column',
                        alignItems: 'center',
                        padding: '0.75rem 0.5rem',
                        borderRadius: '6px',
                        border: kmodePriority === item.id ? '2px solid var(--accent)' : '1px solid var(--border)',
                        background: 'var(--surface)',
                        color: kmodePriority === item.id ? 'var(--accent)' : 'var(--text-main)',
                        cursor: 'pointer',
                        transition: 'all 0.15s ease',
                        textAlign: 'center'
                      }}
                    >
                      <span style={{ fontSize: '0.85rem', fontWeight: 600 }}>{item.name}</span>
                      <span style={{ fontSize: '0.65rem', color: 'var(--text-muted)', marginTop: '0.25rem' }}>{item.desc}</span>
                    </button>
                  ))}
                </div>
              </div>
            </div>

            <RationaleRenderer
              payload_kg={kmodePayload}
              transport_size={kmodeTransport}
              custom_wheelbase={useCustomWheelbase}
              custom_wheelbase_mm={customWheelbaseMm}
              mission_type={kmodeMission}
              optimization_priority={kmodePriority}
              use_range_override={useRangeOverride}
              range_km={kmodeRangeKm}
              hover_time_s={kmodeHoverTimeS}
              climb_altitude_m={kmodeClimbAltitudeM}
              optimal_speed_ms={result?.custom?.optimalSpeedMs ? Number(result.custom.optimalSpeedMs) : undefined}
            />
          </div>
        ) : (
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(300px, 1fr))', gap: '2rem' }}>
          
          {/* Section 1: Physical Bounds */}
          <div className="panel">
            <h2 className="section">1. Physical Bounds</h2>
            <div className="form-grid">
              <div className="form-group">
                <label>Payload Mass (kg)</label>
                <input type="number" name="payload_kg" step="0.1" defaultValue="1.5" required />
              </div>
              <div className="form-group">
                <label>Max Diameter (m)</label>
                <input type="number" name="max_diameter_m" step="0.01" defaultValue="1.0" required />
              </div>
              <div className="form-group">
                <label>Added Drag Area (m²)</label>
                <input type="number" name="added_drag_m2" step="0.01" defaultValue="0.02" />
              </div>
              <div className="form-group">
                <label>CG Shift (m)</label>
                <input type="number" name="cg_shift_m" step="0.01" defaultValue="0.0" />
              </div>
              <div className="form-group">
                <label>Fixed Airframe Mass (kg)</label>
                <input type="number" name="fixed_airframe_mass_kg" step="0.1" placeholder="Auto" />
              </div>
              <div className="form-group">
                <label>Rotor Count</label>
                <select
                  name="rotor_count"
                  value={formRotorCount}
                  onChange={(e) => {
                    const count = e.target.value;
                    setFormRotorCount(count);
                    if (count === "4" || count === "6") {
                      setFormCoaxial(false);
                    }
                  }}
                  style={{ padding: '0.5rem', borderRadius: '4px', border: '1px solid var(--border)', background: 'var(--bg-color)', color: 'var(--text-color)' }}
                >
                  <option value="4">Quadcopter (4)</option>
                  <option value="6">Hexacopter (6)</option>
                  <option value="8">Octocopter (8)</option>
                </select>
              </div>
              <div className="form-group" style={{ gridColumn: 'span 2' }}>
                <label className="toggle-wrapper" style={{ margin: 0, opacity: (formRotorCount === "4" || formRotorCount === "6") ? 0.5 : 1 }}>
                  <span>Coaxial Layout (2 motors per arm)</span>
                  <input
                    type="checkbox"
                    name="coaxial_layout"
                    value="true"
                    checked={formRotorCount === "8" && formCoaxial}
                    disabled={formRotorCount === "4" || formRotorCount === "6"}
                    onChange={(e) => setFormCoaxial(e.target.checked)}
                  />
                </label>
              </div>
              <div className="form-group">
                <label>Airframe Class</label>
                <select name="airframe_class" defaultValue="ConsumerFolding" style={{ padding: '0.5rem', borderRadius: '4px', border: '1px solid var(--border)', background: 'var(--bg-color)', color: 'var(--text-color)' }}>
                  <option value="ConsumerFolding">Consumer Folding (Mavic/Air)</option>
                  <option value="MicroAIO">Micro AIO (Tiny FPV)</option>
                  <option value="EnterpriseRugged">Enterprise Rugged (M300/Alta X)</option>
                  <option value="Agricultural">Agricultural (Agras)</option>
                </select>
              </div>
            </div>
          </div>

          {/* Section 2: Mission Profile */}
          <div className="panel">
            <h2 className="section">2. Mission Profile</h2>
            <div className="form-grid">
              <div className="form-group">
                <label>Forward Velocity (m/s)</label>
                <input type="text" name="v_forward_ms" placeholder="Auto or 10.0" defaultValue="10.0" required />
              </div>
              <div className="form-group">
                <label>Hover Time (s)</label>
                <input type="number" name="t_hover_s" step="10" defaultValue="120" required />
              </div>
              <div className="form-group">
                <label>Forward Flight Time (s)</label>
                <input type="number" name="t_forward_s" step="10" defaultValue="600" required />
              </div>
              <div className="form-group">
                <label>Climb Velocity (m/s)</label>
                <input type="number" name="v_climb_ms" step="0.1" min="0" placeholder="Auto (2.0)" />
              </div>
              <div className="form-group">
                <label>Climb Time (s)</label>
                <input type="number" name="t_climb_s" step="1" min="0" placeholder="Auto (30)" />
              </div>
              <div className="form-group">
                <label>Operating Altitude (m)</label>
                <input type="number" name="altitude_m" step="1" min="0" max="32000" placeholder="Auto (100)" />
              </div>
              <div className="form-group">
                <label>Ambient Temp (°C)</label>
                <input type="number" name="ambient_temp_c" step="0.1" min="-60" max="60" placeholder="Auto (25.0)" />
              </div>
              <div className="form-group">
                <label>Auxiliary Power (W)</label>
                <input type="number" name="aux_power_w" step="1" defaultValue="10" />
              </div>
            </div>
          </div>

          {/* Section 3: Hardware Locks */}
          <div className="panel">
            <h2 className="section">3. Hardware Constraints</h2>
            <div className="form-grid">
              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                <label>Battery Chemistry</label>
                <select name="battery_chemistry" defaultValue="LiPo_premium" style={{ padding: '0.5rem', borderRadius: '4px', border: '1px solid var(--border)', background: 'var(--bg-color)', color: 'var(--text-color)' }}>
                  <option value="LiPo">LiPo (Lithium Polymer - 180 Wh/kg)</option>
                  <option value="LiPo_standard">LiPo Standard (180 Wh/kg)</option>
                  <option value="LiPo_highvoltage">LiPo High Voltage (220 Wh/kg)</option>
                  <option value="LiPo_premium">LiPo Premium (Gens Ace/Tattu - 250 Wh/kg)</option>
                  <option value="LiHV">LiHV (High Voltage - 195 Wh/kg)</option>
                  <option value="Li-ion NMC">Li-ion NMC (240 Wh/kg)</option>
                  <option value="Li-ion NCA">Li-ion NCA (260 Wh/kg)</option>
                  <option value="Solid State">Solid State (Next Gen - 400 Wh/kg)</option>
                  <option value="Li-S">Li-S (Lithium Sulfur - 500 Wh/kg)</option>
                </select>
              </div>

              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                <label>Nominal Cell Count (S)</label>
                <input type="number" name="nominal_cell_count" step="1" defaultValue="0" placeholder="0 = Auto-Solve" />
              </div>

              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                <label>Battery Pack Count</label>
                <input type="number" name="battery_pack_count" step="1" defaultValue="0" placeholder="0 = Auto-Select" />
              </div>

              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                <label>Battery Specific Energy (Wh/kg)</label>
                <input type="number" name="battery_specific_energy_wh_kg" step="any" min="10.1" defaultValue="" placeholder="Auto (Chemistry Spec)" />
              </div>

              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                <label>Battery Cycle Count</label>
                <input type="number" name="battery_cycle_count" step="1" min="0" defaultValue="0" placeholder="0 = New Pack" />
              </div>

              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                <label className="toggle-wrapper">
                  <span>Lock Battery Capacity & Cells</span>
                  <input type="checkbox" name="is_battery_locked" value="true" />
                </label>
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.5rem' }}>
                  <input type="number" name="locked_capacity_mah" step="100" defaultValue="5000" placeholder="Capacity (mAh)" />
                  <input type="number" name="locked_cell_count" step="1" defaultValue="6" placeholder="Cells (S)" />
                </div>
              </div>

              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                <label className="toggle-wrapper">
                  <span>Lock System Voltage</span>
                  <input type="checkbox" name="is_voltage_locked" value="true" />
                </label>
                <input type="number" name="locked_series_cells" step="1" defaultValue="6" placeholder="Series Cells (S)" />
              </div>


              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                <label className="toggle-wrapper">
                  <span>Lock Propeller</span>
                  <input type="checkbox" name="is_prop_locked" value="true" />
                </label>
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.5rem' }}>
                  <input type="number" name="locked_diameter_in" step="0.5" defaultValue="15.0" placeholder="Dia (in)" />
                  <input type="number" name="locked_pitch_in" step="0.5" defaultValue="5.0" placeholder="Pitch (in)" />
                </div>
              </div>

              <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                <label className="toggle-wrapper">
                  <span>Lock Motor</span>
                  <input type="checkbox" name="is_motor_locked" value="true" />
                </label>
                <input type="number" name="locked_kv" step="50" defaultValue="400" placeholder="Target KV" />
              </div>
              
              <div style={{ gridColumn: '1 / -1' }}>
                <label className="toggle-wrapper" style={{background: 'var(--bg-color)'}}>
                  <span style={{fontWeight: 600}}>Auto-Reduce Flight Time on Thermal Overload</span>
                  <input type="checkbox" name="auto_reduce_flight_time" value="true" defaultChecked />
                </label>
              </div>

              <div style={{ gridColumn: '1 / -1', marginTop: '0.5rem' }}>
                <label className="toggle-wrapper" style={{background: 'var(--bg-color)', borderColor: 'var(--accent)'}}>
                  <span style={{fontWeight: 600}}>Marketing Spec Simulation (DoD Bypass & V_md Optimization)</span>
                  <input type="checkbox" name="evaluate_marketing_limits" value="true" />
                </label>
              </div>
            </div>
          </div>

          {/* Section 4: Taxonomy */}
          <div className="panel">
            <h2 className="section">4. Taxonomy & Overrides</h2>
            
            <div className="role-grid">
              {['imaging', 'delivery', 'agriculture', 'racing', 'mapping', 'inspection'].map(role => (
                <div 
                  key={role} 
                  className={`role-card ${selectedRole === role ? 'selected' : ''}`}
                  onClick={() => setSelectedRole(role)}
                >
                  <div style={{textTransform: 'capitalize'}}>{role}</div>
                </div>
              ))}
            </div>

            <div style={{ background: 'var(--bg-color)', padding: '1.5rem', borderRadius: '4px', border: '1px solid var(--border)' }}>
              <h4 style={{ color: 'var(--text-muted)', marginBottom: '1rem', fontSize: '0.9rem', textTransform: 'uppercase' }}>Advanced Math Overrides</h4>
              <div className="form-grid" style={{ marginBottom: 0 }}>
                <div className="form-group">
                  <label>Thrust Margin</label>
                  <input type="number" name="thrust_margin_override" step="0.01" placeholder="Auto" />
                </div>
                <div className="form-group">
                  <label>Integration Overhead (kg)</label>
                  <input type="number" name="integration_overhead_kg" step="0.1" placeholder="Auto" />
                </div>

                {selectedRole === 'delivery' && (
                  <>
                    <div className="form-group">
                      <label>Drop Mass (kg)</label>
                      <input type="number" name="delivery_drop_mass_kg" step="0.01" placeholder="Auto (80%)" />
                    </div>
                    <div className="form-group">
                      <label>Drop Time Ratio</label>
                      <input type="number" name="delivery_drop_time_ratio" step="0.01" placeholder="Auto (0.5)" />
                    </div>
                  </>
                )}

                {selectedRole === 'agriculture' && (
                  <>
                    <div className="form-group">
                      <label>Spray Rate (kg/s)</label>
                      <input type="number" name="spray_rate_kg_per_s" step="0.001" placeholder="Auto" />
                    </div>
                    <div className="form-group">
                      <label>Downwash Limit (m/s)</label>
                      <input type="number" name="ag_max_downwash_ms" step="0.1" placeholder="Auto (5.0)" />
                    </div>
                  </>
                )}

                {selectedRole === 'racing' && (
                  <div className="form-group">
                    <label>Structural G-Limit</label>
                    <input type="number" name="racing_load_factor" step="0.1" placeholder="Auto (10.0)" />
                  </div>
                )}

                {selectedRole === 'mapping' && (
                  <div className="form-group">
                    <label>Drag Area Multiplier</label>
                    <input type="number" name="drag_area_multiplier" step="0.05" placeholder="Auto (1.5)" min="0.1" max="5.0" />
                  </div>
                )}
              </div>
            </div>

            <div style={{ background: 'var(--bg-color)', padding: '1.5rem', borderRadius: '4px', border: '1px solid var(--border)', marginTop: '1rem' }}>
              <h4 style={{ color: 'var(--text-muted)', marginBottom: '1rem', fontSize: '0.9rem', textTransform: 'uppercase' }}>Advanced Scientific Overrides (Physics Defaults)</h4>
              <p style={{fontSize: '0.85rem', color: 'var(--text-muted)', marginBottom: '1rem'}}>Leave empty for Auto (fallback to academic defaults).</p>
              <div className="form-grid" style={{ marginBottom: 0 }}>
                <div className="form-group">
                  <label>Hover Figure of Merit</label>
                  <input type="number" name="figure_of_merit" step="0.01" placeholder="Auto (0.65)" />
                </div>
                <div className="form-group" style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                  <label>Figure of Merit Mode</label>
                  <select name="figure_of_merit_mode" defaultValue="installed" style={{ padding: '0.5rem', borderRadius: '4px', border: '1px solid var(--border)', background: 'var(--bg-color)', color: 'var(--text-color)' }}>
                    <option value="installed">Installed (Aero drag coupled)</option>
                    <option value="isolated">Isolated (Ideal rotor)</option>
                  </select>
                </div>
                <div className="form-group">
                  <label>Isolated FoM Override</label>
                  <input type="number" name="figure_of_merit_isolated" step="0.01" placeholder="Auto (e.g. 0.70)" />
                </div>
                <div className="form-group">
                  <label>Propulsive Efficiency</label>
                  <input type="number" name="propulsive_efficiency" step="0.01" placeholder="Auto (0.75)" />
                </div>
                <div className="form-group">
                  <label>Cd Horizontal (Forward)</label>
                  <input type="number" name="cd_horizontal" step="0.01" placeholder="Auto (1.1)" />
                </div>
                <div className="form-group">
                  <label>Cd Vertical (Climb)</label>
                  <input type="number" name="cd_vertical" step="0.01" placeholder="Auto (1.4)" />
                </div>
                <div className="form-group">
                  <label>Area Scaling Factor</label>
                  <input type="number" name="area_scaling_factor" step="0.001" placeholder="Auto (0.05)" />
                </div>
                <div className="form-group" style={{ gridColumn: '1 / -1', display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                  <label>Aerodynamic Body Class</label>
                  <select name="aero_body_class" defaultValue="research_exposed" style={{ padding: '0.5rem', borderRadius: '4px', border: '1px solid var(--border)', background: 'var(--bg-color)', color: 'var(--text-color)' }}>
                    <option value="research_exposed">Research / DIY (Exposed wiring, boxy body) [CdA scaling = 0.05]</option>
                    <option value="commercial_compact">Commercial Compact (Streamlined folding DJI Mavic class) [CdA scaling = 0.004]</option>
                    <option value="commercial_bulky">Commercial Bulky (Matrice RTK, industrial platform) [CdA scaling = 0.012]</option>
                  </select>
                </div>
                <div className="form-group">
                  <label>Assumed Thrust Coeff (Ct)</label>
                  <input type="number" name="assumed_ct" step="0.001" placeholder="Auto (0.12)" />
                </div>

                <div className="form-group">
                  <label>CF Tube Geometry Const</label>
                  <input type="number" name="geometry_constant" step="0.0001" placeholder="Auto (0.3439)" />
                </div>
                <div className="form-group">
                  <label>CF Wall Thickness Ratio</label>
                  <input type="number" name="wall_thickness_ratio" step="0.01" placeholder="Auto (0.9)" />
                </div>
                <div className="form-group">
                  <label>Body Mass Multiplier</label>
                  <input type="number" name="body_mass_multiplier" step="0.1" placeholder="Auto (1.5)" />
                </div>
                <div className="form-group" style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
                  <label>Arm Configuration</label>
                  <select name="arm_config" defaultValue="under_rotor" style={{ padding: '0.5rem', borderRadius: '4px', border: '1px solid var(--border)', background: 'var(--bg-color)', color: 'var(--text-color)' }}>
                    <option value="under_rotor">Under Rotor (Default)</option>
                    <option value="offset">Offset</option>
                    <option value="folding">Folding</option>
                  </select>
                </div>
              </div>
            </div>

            {/* Section 7: Database Customizer & COTS Library */}
            <div className="panel" style={{ gridColumn: '1 / -1', marginTop: '2rem' }}>
              <h2 className="section" style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                <span>7. Database Customizer & COTS Library</span>
                <span style={{ fontSize: '0.85rem', fontWeight: 'normal', color: 'var(--text-muted)' }}>
                  Modify validation boundaries and component libraries
                </span>
              </h2>

              <div style={{ display: 'flex', gap: '0.5rem', marginBottom: '1.5rem', borderBottom: '1px solid var(--border)', paddingBottom: '0.5rem' }}>
                <button
                  type="button"
                  onClick={() => setActiveDbTab('cots')}
                  className={`tab-btn ${activeDbTab === 'cots' ? 'active' : ''}`}
                  style={{
                    padding: '0.5rem 1rem',
                    borderRadius: '4px',
                    border: 'none',
                    background: activeDbTab === 'cots' ? 'var(--surface-hover)' : 'transparent',
                    color: activeDbTab === 'cots' ? 'var(--accent)' : 'var(--text-muted)',
                    cursor: 'pointer',
                    fontWeight: 600,
                    fontSize: '0.85rem'
                  }}
                >
                  COTS Library
                </button>
                <button
                  type="button"
                  onClick={() => setActiveDbTab('chemistry')}
                  className={`tab-btn ${activeDbTab === 'chemistry' ? 'active' : ''}`}
                  style={{
                    padding: '0.5rem 1rem',
                    borderRadius: '4px',
                    border: 'none',
                    background: activeDbTab === 'chemistry' ? 'var(--surface-hover)' : 'transparent',
                    color: activeDbTab === 'chemistry' ? 'var(--accent)' : 'var(--text-muted)',
                    cursor: 'pointer',
                    fontWeight: 600,
                    fontSize: '0.85rem'
                  }}
                >
                  Battery Chemistry
                </button>
                <button
                  type="button"
                  onClick={() => setActiveDbTab('esc')}
                  className={`tab-btn ${activeDbTab === 'esc' ? 'active' : ''}`}
                  style={{
                    padding: '0.5rem 1rem',
                    borderRadius: '4px',
                    border: 'none',
                    background: activeDbTab === 'esc' ? 'var(--surface-hover)' : 'transparent',
                    color: activeDbTab === 'esc' ? 'var(--accent)' : 'var(--text-muted)',
                    cursor: 'pointer',
                    fontWeight: 600,
                    fontSize: '0.85rem'
                  }}
                >
                  ESC Profile
                </button>
              </div>

              {activeDbTab === 'cots' && cotsDb && (
                <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))', gap: '2rem' }}>
                  {/* Prop Diameters */}
                  <div>
                    <h4 style={{ marginBottom: '1rem', color: 'var(--text-main)' }}>Propeller Diameters (inches)</h4>
                    <div style={{ maxHeight: '250px', overflowY: 'auto', border: '1px solid var(--border)', borderRadius: '4px', padding: '0.5rem', background: 'var(--bg-color)', marginBottom: '1rem' }}>
                      {cotsDb.prop_diameters.map((d: number, idx: number) => (
                        <div key={`diam-${idx}`} style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '0.25rem 0.5rem', borderBottom: '1px solid var(--border)' }}>
                          <span style={{ fontSize: '0.85rem', fontFamily: 'monospace' }}>{d.toFixed(1)}"</span>
                          <button
                            type="button"
                            onClick={() => {
                              const newDiams = [...cotsDb.prop_diameters];
                              newDiams.splice(idx, 1);
                              setCotsDb({ ...cotsDb, prop_diameters: newDiams });
                            }}
                            style={{ background: 'transparent', border: 'none', color: '#ff4d4d', cursor: 'pointer', fontSize: '0.75rem' }}
                          >
                            Remove
                          </button>
                        </div>
                      ))}
                    </div>
                    <div style={{ display: 'flex', gap: '0.5rem' }}>
                      <input
                        type="number"
                        id="new-diam"
                        step="0.1"
                        placeholder="Add Diameter (in)"
                        style={{ flex: 1, padding: '0.4rem', fontSize: '0.85rem', borderRadius: '4px', border: '1px solid var(--border)', background: 'var(--bg-color)', color: 'var(--text-color)' }}
                        onKeyDown={(e) => {
                          if (e.key === 'Enter') {
                            e.preventDefault();
                            const val = parseFloat((e.target as HTMLInputElement).value);
                            if (!isNaN(val) && !cotsDb.prop_diameters.includes(val)) {
                              setCotsDb({ ...cotsDb, prop_diameters: [...cotsDb.prop_diameters, val].sort((a,b)=>a-b) });
                              (e.target as HTMLInputElement).value = '';
                            }
                          }
                        }}
                      />
                      <button
                        type="button"
                        onClick={() => {
                          const input = document.getElementById('new-diam') as HTMLInputElement;
                          const val = parseFloat(input.value);
                          if (!isNaN(val) && !cotsDb.prop_diameters.includes(val)) {
                            setCotsDb({ ...cotsDb, prop_diameters: [...cotsDb.prop_diameters, val].sort((a,b)=>a-b) });
                            input.value = '';
                          }
                        }}
                        style={{ padding: '0.4rem 0.8rem', background: 'var(--accent)', border: 'none', color: '#fff', borderRadius: '4px', cursor: 'pointer', fontSize: '0.85rem' }}
                      >
                        Add
                      </button>
                    </div>
                  </div>

                  {/* Prop Pitches */}
                  <div>
                    <h4 style={{ marginBottom: '1rem', color: 'var(--text-main)' }}>Propeller Pitches (inches)</h4>
                    <div style={{ maxHeight: '250px', overflowY: 'auto', border: '1px solid var(--border)', borderRadius: '4px', padding: '0.5rem', background: 'var(--bg-color)', marginBottom: '1rem' }}>
                      {cotsDb.prop_pitches.map((p: number, idx: number) => (
                        <div key={`pitch-${idx}`} style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '0.25rem 0.5rem', borderBottom: '1px solid var(--border)' }}>
                          <span style={{ fontSize: '0.85rem', fontFamily: 'monospace' }}>{p.toFixed(1)}"</span>
                          <button
                            type="button"
                            onClick={() => {
                              const newPitches = [...cotsDb.prop_pitches];
                              newPitches.splice(idx, 1);
                              setCotsDb({ ...cotsDb, prop_pitches: newPitches });
                            }}
                            style={{ background: 'transparent', border: 'none', color: '#ff4d4d', cursor: 'pointer', fontSize: '0.75rem' }}
                          >
                            Remove
                          </button>
                        </div>
                      ))}
                    </div>
                    <div style={{ display: 'flex', gap: '0.5rem' }}>
                      <input
                        type="number"
                        id="new-pitch"
                        step="0.1"
                        placeholder="Add Pitch (in)"
                        style={{ flex: 1, padding: '0.4rem', fontSize: '0.85rem', borderRadius: '4px', border: '1px solid var(--border)', background: 'var(--bg-color)', color: 'var(--text-color)' }}
                        onKeyDown={(e) => {
                          if (e.key === 'Enter') {
                            e.preventDefault();
                            const val = parseFloat((e.target as HTMLInputElement).value);
                            if (!isNaN(val) && !cotsDb.prop_pitches.includes(val)) {
                              setCotsDb({ ...cotsDb, prop_pitches: [...cotsDb.prop_pitches, val].sort((a,b)=>a-b) });
                              (e.target as HTMLInputElement).value = '';
                            }
                          }
                        }}
                      />
                      <button
                        type="button"
                        onClick={() => {
                          const input = document.getElementById('new-pitch') as HTMLInputElement;
                          const val = parseFloat(input.value);
                          if (!isNaN(val) && !cotsDb.prop_pitches.includes(val)) {
                            setCotsDb({ ...cotsDb, prop_pitches: [...cotsDb.prop_pitches, val].sort((a,b)=>a-b) });
                            input.value = '';
                          }
                        }}
                        style={{ padding: '0.4rem 0.8rem', background: 'var(--accent)', border: 'none', color: '#fff', borderRadius: '4px', cursor: 'pointer', fontSize: '0.85rem' }}
                      >
                        Add
                      </button>
                    </div>
                  </div>

                  {/* Motor KVs */}
                  <div>
                    <h4 style={{ marginBottom: '1rem', color: 'var(--text-main)' }}>Motor KV Offerings (RPM/V)</h4>
                    <div style={{ maxHeight: '250px', overflowY: 'auto', border: '1px solid var(--border)', borderRadius: '4px', padding: '0.5rem', background: 'var(--bg-color)', marginBottom: '1rem' }}>
                      {cotsDb.motor_kvs.map((k: number, idx: number) => (
                        <div key={`kv-${idx}`} style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '0.25rem 0.5rem', borderBottom: '1px solid var(--border)' }}>
                          <span style={{ fontSize: '0.85rem', fontFamily: 'monospace' }}>{k.toFixed(0)} KV</span>
                          <button
                            type="button"
                            onClick={() => {
                              const newKvs = [...cotsDb.motor_kvs];
                              newKvs.splice(idx, 1);
                              setCotsDb({ ...cotsDb, motor_kvs: newKvs });
                            }}
                            style={{ background: 'transparent', border: 'none', color: '#ff4d4d', cursor: 'pointer', fontSize: '0.75rem' }}
                          >
                            Remove
                          </button>
                        </div>
                      ))}
                    </div>
                    <div style={{ display: 'flex', gap: '0.5rem' }}>
                      <input
                        type="number"
                        id="new-kv"
                        step="10"
                        placeholder="Add KV"
                        style={{ flex: 1, padding: '0.4rem', fontSize: '0.85rem', borderRadius: '4px', border: '1px solid var(--border)', background: 'var(--bg-color)', color: 'var(--text-color)' }}
                        onKeyDown={(e) => {
                          if (e.key === 'Enter') {
                            e.preventDefault();
                            const val = parseFloat((e.target as HTMLInputElement).value);
                            if (!isNaN(val) && !cotsDb.motor_kvs.includes(val)) {
                              setCotsDb({ ...cotsDb, motor_kvs: [...cotsDb.motor_kvs, val].sort((a,b)=>a-b) });
                              (e.target as HTMLInputElement).value = '';
                            }
                          }
                        }}
                      />
                      <button
                        type="button"
                        onClick={() => {
                          const input = document.getElementById('new-kv') as HTMLInputElement;
                          const val = parseFloat(input.value);
                          if (!isNaN(val) && !cotsDb.motor_kvs.includes(val)) {
                            setCotsDb({ ...cotsDb, motor_kvs: [...cotsDb.motor_kvs, val].sort((a,b)=>a-b) });
                            input.value = '';
                          }
                        }}
                        style={{ padding: '0.4rem 0.8rem', background: 'var(--accent)', border: 'none', color: '#fff', borderRadius: '4px', cursor: 'pointer', fontSize: '0.85rem' }}
                      >
                        Add
                      </button>
                    </div>
                  </div>
                </div>
              )}

              {activeDbTab === 'chemistry' && chemistryDb && (
                <div>
                  <h4 style={{ marginBottom: '1rem', color: 'var(--text-main)' }}>Battery Chemistry Profiles</h4>
                  <div style={{ overflowX: 'auto', marginBottom: '1.5rem' }}>
                    <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.85rem' }}>
                      <thead>
                        <tr style={{ borderBottom: '2px solid var(--border)', textAlign: 'left', color: 'var(--text-muted)' }}>
                          <th style={{ padding: '0.5rem' }}>Name</th>
                          <th style={{ padding: '0.5rem' }}>E Spec Max (Wh/kg)</th>
                          <th style={{ padding: '0.5rem' }}>P Spec Max (W/kg)</th>
                          <th style={{ padding: '0.5rem' }}>Ri Base (Ω)</th>
                          <th style={{ padding: '0.5rem' }}>V Full (V)</th>
                          <th style={{ padding: '0.5rem' }}>V Nom (V)</th>
                          <th style={{ padding: '0.5rem' }}>V Exp (V)</th>
                          <th style={{ padding: '0.5rem' }}>Tremblay Exp Rate</th>
                          <th style={{ padding: '0.5rem' }}>Tremblay Polariz Coeff</th>
                          <th style={{ padding: '0.5rem' }}>Actions</th>
                        </tr>
                      </thead>
                      <tbody>
                        {chemistryDb.chemistries.map((chem: any, idx: number) => (
                          <tr key={`chem-${idx}`} style={{ borderBottom: '1px solid var(--border)' }}>
                            <td style={{ padding: '0.5rem' }}>
                              <input
                                type="text"
                                value={chem.name}
                                onChange={(e) => {
                                  const newList = [...chemistryDb.chemistries];
                                  newList[idx].name = e.target.value;
                                  setChemistryDb({ chemistries: newList });
                                }}
                                style={{ width: '100%', padding: '0.25rem', border: '1px solid transparent', background: 'transparent', color: 'var(--text-color)' }}
                                onFocus={(e) => { e.target.style.borderColor = 'var(--border)'; e.target.style.background = 'var(--bg-color)'; }}
                                onBlur={(e) => { e.target.style.borderColor = 'transparent'; e.target.style.background = 'transparent'; }}
                              />
                            </td>
                            <td style={{ padding: '0.5rem' }}>
                              <input
                                type="number"
                                step="1"
                                value={chem.e_spec_max_wh_kg}
                                onChange={(e) => {
                                  const newList = [...chemistryDb.chemistries];
                                  newList[idx].e_spec_max_wh_kg = parseFloat(e.target.value) || 0;
                                  setChemistryDb({ chemistries: newList });
                                }}
                                style={{ width: '60px', padding: '0.25rem', border: '1px solid transparent', background: 'transparent', color: 'var(--text-color)' }}
                                onFocus={(e) => { e.target.style.borderColor = 'var(--border)'; e.target.style.background = 'var(--bg-color)'; }}
                                onBlur={(e) => { e.target.style.borderColor = 'transparent'; e.target.style.background = 'transparent'; }}
                              />
                            </td>
                            <td style={{ padding: '0.5rem' }}>
                              <input
                                type="number"
                                step="10"
                                value={chem.p_spec_max_w_kg}
                                onChange={(e) => {
                                  const newList = [...chemistryDb.chemistries];
                                  newList[idx].p_spec_max_w_kg = parseFloat(e.target.value) || 0;
                                  setChemistryDb({ chemistries: newList });
                                }}
                                style={{ width: '70px', padding: '0.25rem', border: '1px solid transparent', background: 'transparent', color: 'var(--text-color)' }}
                                onFocus={(e) => { e.target.style.borderColor = 'var(--border)'; e.target.style.background = 'var(--bg-color)'; }}
                                onBlur={(e) => { e.target.style.borderColor = 'transparent'; e.target.style.background = 'transparent'; }}
                              />
                            </td>
                            <td style={{ padding: '0.5rem' }}>
                              <input
                                type="number"
                                step="0.001"
                                value={chem.r_i_base_ohms}
                                onChange={(e) => {
                                  const newList = [...chemistryDb.chemistries];
                                  newList[idx].r_i_base_ohms = parseFloat(e.target.value) || 0;
                                  setChemistryDb({ chemistries: newList });
                                }}
                                style={{ width: '70px', padding: '0.25rem', border: '1px solid transparent', background: 'transparent', color: 'var(--text-color)' }}
                                onFocus={(e) => { e.target.style.borderColor = 'var(--border)'; e.target.style.background = 'var(--bg-color)'; }}
                                onBlur={(e) => { e.target.style.borderColor = 'transparent'; e.target.style.background = 'transparent'; }}
                              />
                            </td>
                            <td style={{ padding: '0.5rem' }}>
                              <input
                                type="number"
                                step="0.05"
                                value={chem.v_full_cell}
                                onChange={(e) => {
                                  const newList = [...chemistryDb.chemistries];
                                  newList[idx].v_full_cell = parseFloat(e.target.value) || 0;
                                  setChemistryDb({ chemistries: newList });
                                }}
                                style={{ width: '60px', padding: '0.25rem', border: '1px solid transparent', background: 'transparent', color: 'var(--text-color)' }}
                                onFocus={(e) => { e.target.style.borderColor = 'var(--border)'; e.target.style.background = 'var(--bg-color)'; }}
                                onBlur={(e) => { e.target.style.borderColor = 'transparent'; e.target.style.background = 'transparent'; }}
                              />
                            </td>
                            <td style={{ padding: '0.5rem' }}>
                              <input
                                type="number"
                                step="0.05"
                                value={chem.v_nom_cell}
                                onChange={(e) => {
                                  const newList = [...chemistryDb.chemistries];
                                  newList[idx].v_nom_cell = parseFloat(e.target.value) || 0;
                                  setChemistryDb({ chemistries: newList });
                                }}
                                style={{ width: '60px', padding: '0.25rem', border: '1px solid transparent', background: 'transparent', color: 'var(--text-color)' }}
                                onFocus={(e) => { e.target.style.borderColor = 'var(--border)'; e.target.style.background = 'var(--bg-color)'; }}
                                onBlur={(e) => { e.target.style.borderColor = 'transparent'; e.target.style.background = 'transparent'; }}
                              />
                            </td>
                            <td style={{ padding: '0.5rem' }}>
                              <input
                                type="number"
                                step="0.05"
                                value={chem.v_exp_cell}
                                onChange={(e) => {
                                  const newList = [...chemistryDb.chemistries];
                                  newList[idx].v_exp_cell = parseFloat(e.target.value) || 0;
                                  setChemistryDb({ chemistries: newList });
                                }}
                                style={{ width: '60px', padding: '0.25rem', border: '1px solid transparent', background: 'transparent', color: 'var(--text-color)' }}
                                onFocus={(e) => { e.target.style.borderColor = 'var(--border)'; e.target.style.background = 'var(--bg-color)'; }}
                                onBlur={(e) => { e.target.style.borderColor = 'transparent'; e.target.style.background = 'transparent'; }}
                              />
                            </td>
                            <td style={{ padding: '0.5rem' }}>
                              <input
                                type="number"
                                step="0.5"
                                value={chem.tremblay_exp_rate}
                                onChange={(e) => {
                                  const newList = [...chemistryDb.chemistries];
                                  newList[idx].tremblay_exp_rate = parseFloat(e.target.value) || 0;
                                  setChemistryDb({ chemistries: newList });
                                }}
                                style={{ width: '60px', padding: '0.25rem', border: '1px solid transparent', background: 'transparent', color: 'var(--text-color)' }}
                                onFocus={(e) => { e.target.style.borderColor = 'var(--border)'; e.target.style.background = 'var(--bg-color)'; }}
                                onBlur={(e) => { e.target.style.borderColor = 'transparent'; e.target.style.background = 'transparent'; }}
                              />
                            </td>
                            <td style={{ padding: '0.5rem' }}>
                              <input
                                type="number"
                                step="0.001"
                                value={chem.tremblay_polarization_coeff}
                                onChange={(e) => {
                                  const newList = [...chemistryDb.chemistries];
                                  newList[idx].tremblay_polarization_coeff = parseFloat(e.target.value) || 0;
                                  setChemistryDb({ chemistries: newList });
                                }}
                                style={{ width: '60px', padding: '0.25rem', border: '1px solid transparent', background: 'transparent', color: 'var(--text-color)' }}
                                onFocus={(e) => { e.target.style.borderColor = 'var(--border)'; e.target.style.background = 'var(--bg-color)'; }}
                                onBlur={(e) => { e.target.style.borderColor = 'transparent'; e.target.style.background = 'transparent'; }}
                              />
                            </td>
                            <td style={{ padding: '0.5rem' }}>
                              <button
                                type="button"
                                onClick={() => {
                                  const newList = [...chemistryDb.chemistries];
                                  newList.splice(idx, 1);
                                  setChemistryDb({ chemistries: newList });
                                }}
                                style={{ background: 'transparent', border: 'none', color: '#ff4d4d', cursor: 'pointer', fontSize: '0.85rem' }}
                              >
                                Delete
                              </button>
                            </td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>

                  {/* Add Chemistry Form */}
                  <div style={{ background: 'var(--surface)', padding: '1rem', borderRadius: '4px', border: '1px solid var(--border)' }}>
                    <h5 style={{ marginBottom: '0.75rem', color: 'var(--text-main)', fontSize: '0.9rem' }}>Add New Battery Chemistry Profile</h5>
                    <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(130px, 1fr))', gap: '1rem', marginBottom: '1rem' }}>
                      <div className="form-group" style={{ margin: 0 }}>
                        <label style={{ fontSize: '0.75rem' }}>Name</label>
                        <input type="text" id="new-chem-name" placeholder="Solid State V2" style={{ padding: '0.4rem', fontSize: '0.8rem' }} />
                      </div>
                      <div className="form-group" style={{ margin: 0 }}>
                        <label style={{ fontSize: '0.75rem' }}>E Max (Wh/kg)</label>
                        <input type="number" id="new-chem-e" placeholder="450" style={{ padding: '0.4rem', fontSize: '0.8rem' }} />
                      </div>
                      <div className="form-group" style={{ margin: 0 }}>
                        <label style={{ fontSize: '0.75rem' }}>P Max (W/kg)</label>
                        <input type="number" id="new-chem-p" placeholder="1000" style={{ padding: '0.4rem', fontSize: '0.8rem' }} />
                      </div>
                      <div className="form-group" style={{ margin: 0 }}>
                        <label style={{ fontSize: '0.75rem' }}>Ri Base (Ω)</label>
                        <input type="number" step="0.001" id="new-chem-ri" placeholder="0.04" style={{ padding: '0.4rem', fontSize: '0.8rem' }} />
                      </div>
                      <div className="form-group" style={{ margin: 0 }}>
                        <label style={{ fontSize: '0.75rem' }}>V Full (V)</label>
                        <input type="number" step="0.05" id="new-chem-vfull" placeholder="4.2" style={{ padding: '0.4rem', fontSize: '0.8rem' }} />
                      </div>
                      <div className="form-group" style={{ margin: 0 }}>
                        <label style={{ fontSize: '0.75rem' }}>V Nom (V)</label>
                        <input type="number" step="0.05" id="new-chem-vnom" placeholder="3.6" style={{ padding: '0.4rem', fontSize: '0.8rem' }} />
                      </div>
                      <div className="form-group" style={{ margin: 0 }}>
                        <label style={{ fontSize: '0.75rem' }}>V Exp (V)</label>
                        <input type="number" step="0.05" id="new-chem-vexp" placeholder="3.85" style={{ padding: '0.4rem', fontSize: '0.8rem' }} />
                      </div>
                    </div>
                    <button
                      type="button"
                      onClick={() => {
                        const nameEl = document.getElementById('new-chem-name') as HTMLInputElement;
                        const eEl = document.getElementById('new-chem-e') as HTMLInputElement;
                        const pEl = document.getElementById('new-chem-p') as HTMLInputElement;
                        const riEl = document.getElementById('new-chem-ri') as HTMLInputElement;
                        const vfullEl = document.getElementById('new-chem-vfull') as HTMLInputElement;
                        const vnomEl = document.getElementById('new-chem-vnom') as HTMLInputElement;
                        const vexpEl = document.getElementById('new-chem-vexp') as HTMLInputElement;

                        if (nameEl.value) {
                          const newChem = {
                            name: nameEl.value,
                            e_spec_max_wh_kg: parseFloat(eEl.value) || 200.0,
                            p_spec_max_w_kg: parseFloat(pEl.value) || 3000.0,
                            r_i_base_ohms: parseFloat(riEl.value) || 0.005,
                            v_full_cell: parseFloat(vfullEl.value) || 4.2,
                            v_nom_cell: parseFloat(vnomEl.value) || 3.7,
                            v_exp_cell: parseFloat(vexpEl.value) || 3.9,
                            tremblay_exp_rate: 6.0,
                            tremblay_polarization_coeff: 0.045
                          };
                          setChemistryDb({ chemistries: [...chemistryDb.chemistries, newChem] });
                          nameEl.value = '';
                          eEl.value = '';
                          pEl.value = '';
                          riEl.value = '';
                          vfullEl.value = '';
                          vnomEl.value = '';
                          vexpEl.value = '';
                        }
                      }}
                      style={{ padding: '0.4rem 1rem', background: 'var(--accent)', border: 'none', color: '#fff', borderRadius: '4px', cursor: 'pointer', fontSize: '0.85rem' }}
                    >
                      Add Chemistry Profile
                    </button>
                  </div>
                </div>
              )}

              {activeDbTab === 'esc' && escDb && (
                <div>
                  <h4 style={{ marginBottom: '1rem', color: 'var(--text-main)' }}>Electronic Speed Controller (ESC) Specifications</h4>
                  <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(200px, 1fr))', gap: '2rem', marginBottom: '1.5rem' }}>
                    <div className="form-group">
                      <label>Output Capacitance C_oss (Farads)</label>
                      <input
                        type="number"
                        step="1e-11"
                        value={escDb.c_oss_farads}
                        onChange={(e) => {
                          setEscDb({ ...escDb, c_oss_farads: parseFloat(e.target.value) || 0 });
                        }}
                        style={{ padding: '0.5rem', borderRadius: '4px', border: '1px solid var(--border)', background: 'var(--bg-color)', color: 'var(--text-color)', width: '100%' }}
                      />
                      <span style={{ fontSize: '0.75rem', color: 'var(--text-muted)' }}>Typical: 1.5nF (1.5e-9 F) to 5.0nF</span>
                    </div>
                    <div className="form-group">
                      <label>MOSFET On-Resistance R_ds_on (Ohms)</label>
                      <input
                        type="number"
                        step="0.0001"
                        value={escDb.r_ds_on_ohms}
                        onChange={(e) => {
                          setEscDb({ ...escDb, r_ds_on_ohms: parseFloat(e.target.value) || 0 });
                        }}
                        style={{ padding: '0.5rem', borderRadius: '4px', border: '1px solid var(--border)', background: 'var(--bg-color)', color: 'var(--text-color)', width: '100%' }}
                      />
                      <span style={{ fontSize: '0.75rem', color: 'var(--text-muted)' }}>Typical: 0.001 Ω (1 mΩ) to 0.005 Ω</span>
                    </div>
                  </div>
                </div>
              )}

              <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginTop: '1.5rem', paddingTop: '1.5rem', borderTop: '1px solid var(--border)' }}>
                <button
                  type="button"
                  onClick={async () => {
                    setDbStatus('Saving database modifications...');
                    const res = await updateDatabases(cotsDb, chemistryDb, escDb);
                    if (res.success) {
                      setDbStatus('Databases successfully saved!');
                      fetchDbs(); // reload
                      setTimeout(() => setDbStatus(null), 3000);
                    } else {
                      setDbStatus(`Error saving databases: ${res.error}`);
                      setTimeout(() => setDbStatus(null), 5000);
                    }
                  }}
                  className="btn-primary"
                  style={{ margin: 0, padding: '0.5rem 1.25rem', fontSize: '0.85rem' }}
                >
                  SAVE DATABASE CHANGES
                </button>
                {dbStatus && (
                  <span style={{ fontSize: '0.85rem', fontWeight: 600, color: dbStatus.includes('Error') ? '#ff4d4d' : 'var(--accent)' }}>
                    {dbStatus}
                  </span>
                )}
              </div>
            </div>
          </div>
        </div>
      )}

        <div style={{ display: 'flex', justifyContent: 'center', gap: '1rem', marginTop: '2rem' }}>
          <button type="submit" className="btn-primary" style={{ margin: 0 }} disabled={loading}>
            {loading ? <span className="loader"></span> : 'EXECUTE VALIDATION ENGINE'}
          </button>
          <button 
            type="button" 
            onClick={copyShareableLink} 
            className="btn-secondary" 
            style={{
              padding: '0.75rem 1.5rem',
              fontSize: '0.9rem',
              fontWeight: '500',
              borderRadius: '4px',
              border: '1px solid var(--border)',
              background: 'transparent',
              color: 'var(--text-main)',
              cursor: 'pointer',
              transition: 'all 0.15s ease-in-out',
              minWidth: '200px',
              display: 'inline-flex',
              alignItems: 'center',
              justifyContent: 'center',
              gap: '0.5rem'
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.background = 'var(--surface-hover)';
              e.currentTarget.style.borderColor = 'var(--text-muted)';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.background = 'transparent';
              e.currentTarget.style.borderColor = 'var(--border)';
            }}
          >
            {copied ? (
              <>
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round" style={{ color: 'var(--success)' }}>
                  <polyline points="20 6 9 17 4 12" />
                </svg>
                <span style={{ color: 'var(--success)' }}>LINK COPIED</span>
              </>
            ) : (
              <>
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <path d="M10 13a5 5 0 0 0 7.54.54l3-3a5 5 0 0 0-7.07-7.07l-1.72 1.71" />
                  <path d="M14 11a5 5 0 0 0-7.54-.54l-3 3a5 5 0 0 0 7.07 7.07l1.71-1.71" />
                </svg>
                <span>SHARE BUILD LINK</span>
              </>
            )}
          </button>
        </div>
      </form>

      {/* Main Results Dashboard */}
      {result && (
        <div className="results-container">
          <div className="panel">
            <div style={{display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '2rem'}}>
              <h2 style={{marginBottom: 0}}>Simulation Output</h2>
              {result.success ? 
                <span style={{background: '#d1e7dd', color: '#0f5132', padding: '0.4rem 1rem', borderRadius: '4px', fontWeight: 600, fontSize: '0.9rem'}}>SUCCESS</span> : 
                <span style={{background: '#f8d7da', color: '#842029', padding: '0.4rem 1rem', borderRadius: '4px', fontWeight: 600, fontSize: '0.9rem'}}>PIPELINE FAILED</span>
              }
            </div>

            {/* Goal Relaxation Warnings */}
            {(result.relaxation1 || result.relaxation2) && (
              <div style={{background: '#fff3cd', border: '1px solid #ffc107', borderRadius: '4px', padding: '1rem 1.25rem', marginBottom: '1.5rem'}}>
                <div style={{fontWeight: 700, color: '#856404', marginBottom: '0.5rem'}}>⚠ Auto Goal Relaxation Applied</div>
                <p style={{color: '#856404', fontSize: '0.9rem', marginBottom: '0.5rem'}}>The requested flight time exceeded the thermal limit. The engine automatically scaled it down to stay below 60 °C. <strong>Sized battery and mass reflect the REDUCED mission, not your requested one.</strong></p>
                {result.relaxation1 && (
                  <div style={{fontSize: '0.85rem', color: '#533f03', marginTop: '0.5rem'}}>
                    <strong>Baseline:</strong> Hover {result.relaxation1.approvedHover}s (req: {result.relaxation1.requestedHover}s) · Forward {result.relaxation1.approvedForward}s (req: {result.relaxation1.requestedForward}s) · Climb {result.relaxation1.approvedClimb}s (req: {result.relaxation1.requestedClimb}s)
                  </div>
                )}
                {result.relaxation2 && (
                  <div style={{fontSize: '0.85rem', color: '#533f03', marginTop: '0.25rem'}}>
                    <strong>Custom Role:</strong> Hover {result.relaxation2.approvedHover}s (req: {result.relaxation2.requestedHover}s) · Forward {result.relaxation2.approvedForward}s (req: {result.relaxation2.requestedForward}s) · Climb {result.relaxation2.approvedClimb}s (req: {result.relaxation2.requestedClimb}s)
                  </div>
                )}
              </div>
            )}

            {result.success && result.baseline && result.custom && (
              <>
                {isKmode && (
                  <div style={{ marginBottom: '2rem' }}>
                    <RationaleRenderer
                      payload_kg={kmodePayload}
                      transport_size={kmodeTransport}
                      custom_wheelbase={useCustomWheelbase}
                      custom_wheelbase_mm={customWheelbaseMm}
                      mission_type={kmodeMission}
                      optimization_priority={kmodePriority}
                      use_range_override={useRangeOverride}
                      range_km={kmodeRangeKm}
                      hover_time_s={kmodeHoverTimeS}
                      climb_altitude_m={kmodeClimbAltitudeM}
                      optimal_speed_ms={result.custom?.optimalSpeedMs ? Number(result.custom.optimalSpeedMs) : undefined}
                    />
                  </div>
                )}
                <div className="comparison-grid">
                <div className="result-card baseline">
                  <h3 style={{color: '#6c757d'}}>Standard Hover Math (Baseline)</h3>
                  
                  {renderMassBreakdown(result.baseline)}
                  <div className="result-row">
                    <span className="result-label">Drained Energy</span>
                    <span className="result-value">{result.baseline.energy} Wh</span>
                  </div>
                  <div className="result-row" style={result.relaxation1 ? { background: '#fff3cd', padding: '0.25rem 0.5rem', borderRadius: '4px', border: '1px solid #ffeeba' } : {}}>
                    <span className="result-label" style={{ fontWeight: result.relaxation1 ? '600' : 'normal' }}>Sized Hover Time</span>
                    <span className="result-value" style={{ fontWeight: result.relaxation1 ? '600' : 'normal' }}>
                      {getFlightTimeDisplay(result.baseline.approvedHover, result.relaxation1, 'Hover')}
                    </span>
                  </div>
                  <div className="result-row" style={result.relaxation1 ? { background: '#fff3cd', padding: '0.25rem 0.5rem', borderRadius: '4px', border: '1px solid #ffeeba' } : {}}>
                    <span className="result-label" style={{ fontWeight: result.relaxation1 ? '600' : 'normal' }}>Sized Forward Time</span>
                    <span className="result-value" style={{ fontWeight: result.relaxation1 ? '600' : 'normal' }}>
                      {getFlightTimeDisplay(result.baseline.approvedForward, result.relaxation1, 'Forward')}
                    </span>
                  </div>
                  <div className="result-row" style={result.relaxation1 ? { background: '#fff3cd', padding: '0.25rem 0.5rem', borderRadius: '4px', border: '1px solid #ffeeba' } : {}}>
                    <span className="result-label" style={{ fontWeight: result.relaxation1 ? '600' : 'normal' }}>Sized Climb Time</span>
                    <span className="result-value" style={{ fontWeight: result.relaxation1 ? '600' : 'normal' }}>
                      {getFlightTimeDisplay(result.baseline.approvedClimb, result.relaxation1, 'Climb')}
                    </span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Cell Count (S)</span>
                    <span className="result-value">{result.baseline.cellCount} S</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Nominal Voltage</span>
                    <span className="result-value">{result.baseline.voltage} V</span>
                  </div>
                  {result.baseline.optimalSpeedMs && result.baseline.optimalSpeedMs !== 'N/A' && (
                    <div className="result-row" style={{ background: '#eef2ff', padding: '0.35rem 0.5rem', borderRadius: '4px', border: '1px solid var(--accent)', marginBottom: '0.5rem' }}>
                      <span className="result-label" style={{ fontWeight: '600', color: 'var(--accent)' }}>Optimized Cruise Speed</span>
                      <span className="result-value" style={{ fontWeight: '600', color: 'var(--accent)' }}>{result.baseline.optimalSpeedMs} m/s</span>
                    </div>
                  )}
                  <div className="result-row">
                    <span className="result-label">Battery Temp</span>
                    <span className="result-value" style={{color: parseFloat(result.baseline.temp) > 60 ? 'var(--error)' : 'inherit'}}>{result.baseline.temp} °C</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Thermal Margin</span>
                    <span className="result-value">{result.baseline.thermalMargin} °C</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Arm Diameter</span>
                    <span className="result-value">{result.baseline.armOd} mm</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Arm Wall Thickness</span>
                    <span className="result-value">{result.baseline.armWall} mm</span>
                  </div>
                  <div className="result-row" style={{marginTop: '1rem', paddingTop: '1rem', borderTop: '2px dashed var(--border)'}}>
                    <span className="result-label">Thrust-to-Weight</span>
                    <span className="result-value">{result.baseline.thrustToWeight} : 1</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Forward Pitch</span>
                    <span className="result-value" style={{color: parseFloat(result.baseline.forwardPitch) > 30 ? 'var(--error)' : 'inherit'}}>{result.baseline.forwardPitch}°</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label" title="MINLP Objective J-Score measures the deviation of COTS components from the continuous sizing optimum combined with physical constraint penalties. A lower score is better; a score > 5 suggests component mismatches or constraint relaxations. Hover over the Model Documentation at the bottom of the page to read more.">Objective J-Score 🛈</span>
                    <span className="result-value">{result.baseline.jScore}</span>
                  </div>
                  <div className="result-row" style={{marginTop: '1rem', paddingTop: '1rem', borderTop: '2px dashed var(--border)'}}>
                    <span className="result-label">Target Propeller</span>
                    <span className="result-value">{result.baseline.prop} in</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Target KV</span>
                    <span className="result-value">{result.baseline.targetKv} KV</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">KV Shopping Range</span>
                    <span className="result-value">{result.baseline.shopping} KV</span>
                  </div>
                  <div className="result-row" style={{marginTop: '1rem', paddingTop: '1rem', borderTop: '2px dashed var(--border)'}}>
                    <span className="result-label">Approved Hover</span>
                    <span className="result-value">{result.baseline.approvedHover} s</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Approved Forward</span>
                    <span className="result-value">{result.baseline.approvedForward} s</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Approved Climb</span>
                    <span className="result-value">{result.baseline.approvedClimb} s</span>
                  </div>
                </div>
 
                <div className="result-card custom">
                  <h3 style={{color: 'var(--accent)'}}>Active Taxonomy Math ({selectedRole})</h3>
                  
                  {renderMassBreakdown(result.custom)}
                  <div className="result-row">
                    <span className="result-label">Drained Energy</span>
                    <span className="result-value">{result.custom.energy} Wh</span>
                  </div>
                  <div className="result-row" style={result.relaxation2 ? { background: '#fff3cd', padding: '0.25rem 0.5rem', borderRadius: '4px', border: '1px solid #ffeeba' } : {}}>
                    <span className="result-label" style={{ fontWeight: result.relaxation2 ? '600' : 'normal' }}>Sized Hover Time</span>
                    <span className="result-value" style={{ fontWeight: result.relaxation2 ? '600' : 'normal' }}>
                      {getFlightTimeDisplay(result.custom.approvedHover, result.relaxation2, 'Hover')}
                    </span>
                  </div>
                  <div className="result-row" style={result.relaxation2 ? { background: '#fff3cd', padding: '0.25rem 0.5rem', borderRadius: '4px', border: '1px solid #ffeeba' } : {}}>
                    <span className="result-label" style={{ fontWeight: result.relaxation2 ? '600' : 'normal' }}>Sized Forward Time</span>
                    <span className="result-value" style={{ fontWeight: result.relaxation2 ? '600' : 'normal' }}>
                      {getFlightTimeDisplay(result.custom.approvedForward, result.relaxation2, 'Forward')}
                    </span>
                  </div>
                  <div className="result-row" style={result.relaxation2 ? { background: '#fff3cd', padding: '0.25rem 0.5rem', borderRadius: '4px', border: '1px solid #ffeeba' } : {}}>
                    <span className="result-label" style={{ fontWeight: result.relaxation2 ? '600' : 'normal' }}>Sized Climb Time</span>
                    <span className="result-value" style={{ fontWeight: result.relaxation2 ? '600' : 'normal' }}>
                      {getFlightTimeDisplay(result.custom.approvedClimb, result.relaxation2, 'Climb')}
                    </span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Cell Count (S)</span>
                    <span className="result-value">{result.custom.cellCount} S</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Nominal Voltage</span>
                    <span className="result-value">{result.custom.voltage} V</span>
                  </div>
                  {result.custom.optimalSpeedMs && result.custom.optimalSpeedMs !== 'N/A' && (
                    <div className="result-row" style={{ background: '#eef2ff', padding: '0.35rem 0.5rem', borderRadius: '4px', border: '1px solid var(--accent)', marginBottom: '0.5rem' }}>
                      <span className="result-label" style={{ fontWeight: '600', color: 'var(--accent)' }}>Optimized Cruise Speed</span>
                      <span className="result-value" style={{ fontWeight: '600', color: 'var(--accent)' }}>{result.custom.optimalSpeedMs} m/s</span>
                    </div>
                  )}
                  <div className="result-row">
                    <span className="result-label">Battery Temp</span>
                    <span className="result-value" style={{color: parseFloat(result.custom.temp) > 60 ? 'var(--error)' : 'inherit'}}>{result.custom.temp} °C</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Thermal Margin</span>
                    <span className="result-value">{result.custom.thermalMargin} °C</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Arm Diameter</span>
                    <span className="result-value">{result.custom.armOd} mm</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Arm Wall Thickness</span>
                    <span className="result-value">{result.custom.armWall} mm</span>
                  </div>
                  <div className="result-row" style={{marginTop: '1rem', paddingTop: '1rem', borderTop: '2px dashed var(--border)'}}>
                    <span className="result-label">Thrust-to-Weight</span>
                    <span className="result-value">{result.custom.thrustToWeight} : 1</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Forward Pitch</span>
                    <span className="result-value" style={{color: parseFloat(result.custom.forwardPitch) > 30 ? 'var(--error)' : 'inherit'}}>{result.custom.forwardPitch}°</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label" title="MINLP Objective J-Score measures the deviation of COTS components from the continuous sizing optimum combined with physical constraint penalties. A lower score is better; a score > 5 suggests component mismatches or constraint relaxations. Hover over the Model Documentation at the bottom of the page to read more.">Objective J-Score 🛈</span>
                    <span className="result-value">{result.custom.jScore}</span>
                  </div>
                  <div className="result-row" style={{marginTop: '1rem', paddingTop: '1rem', borderTop: '2px dashed var(--border)'}}>
                    <span className="result-label">Target Propeller</span>
                    <span className="result-value">{result.custom.prop} in</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Target KV</span>
                    <span className="result-value">{result.custom.targetKv} KV</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">KV Shopping Range</span>
                    <span className="result-value">{result.custom.shopping} KV</span>
                  </div>
                  <div className="result-row" style={{marginTop: '1rem', paddingTop: '1rem', borderTop: '2px dashed var(--border)'}}>
                    <span className="result-label">Approved Hover</span>
                    <span className="result-value">{result.custom.approvedHover} s</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Approved Forward</span>
                    <span className="result-value">{result.custom.approvedForward} s</span>
                  </div>
                  <div className="result-row">
                    <span className="result-label">Approved Climb</span>
                    <span className="result-value">{result.custom.approvedClimb} s</span>
                  </div>
                </div>
              </div>

              {/* Visual Engineering Diagnostics Grid */}
              <div style={{
                display: 'grid',
                gridTemplateColumns: 'repeat(auto-fit, minmax(300px, 1fr))',
                gap: '2rem',
                marginTop: '2rem',
                paddingTop: '2rem',
                borderTop: '2px solid var(--border)'
              }}>
                <div className="panel" style={{ display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
                  <DatabaseCrosshair 
                    idealKv={parseFloat(result.custom.idealKv) || 400} 
                    idealMotorMassG={parseFloat(result.custom.idealMotorMassG) || 100} 
                  />
                </div>
                <div className="panel" style={{ display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
                  <WhatIfExplorer 
                    totalMass={parseFloat(result.custom.mass) || 1.5} 
                    batteryMass={parseFloat(result.custom.battery) || 0.5} 
                    hoverTime={parseFloat(result.custom.approvedHover) || 600} 
                    batterySpecificEnergy={submittedParams?.specEnergy || 250.0} 
                    nominalVoltage={parseFloat(result.custom.voltage) || 22.2} 
                  />
                </div>
                <div className="panel" style={{ display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
                  <ThermalHeatmap 
                    rotorCount={submittedParams?.rotorCount || 4} 
                    batteryTemp={parseFloat(result.custom.temp) || 25.0} 
                    armDiameter={parseFloat(result.custom.armOd) || 20.0} 
                    coaxialLayout={submittedParams?.coaxialLayout || false} 
                  />
                </div>
              </div>
            </>
          )}

            {/* Display Failures & Warnings */}
            {result.failures && result.failures.length > 0 && (
              <div style={{marginTop: '2rem'}}>
                <h3 style={{color: '#721c24'}}>Validation Warnings & Terminations</h3>
                {result.failures.map((f: any, i: number) => {
                  const isWarning = f.run && (f.run.includes("Warning") || f.run.includes("warning"));
                  return (
                    <div key={i} className={isWarning ? "warning-box" : "error-box"} style={isWarning ? { borderLeft: '4px solid #ffc107', background: '#fff3cd', color: '#856404', padding: '1rem', borderRadius: '4px', marginBottom: '1rem' } : {}}>
                      <div className="error-title" style={isWarning ? { fontWeight: 'bold', fontSize: '1.05rem', color: '#856404' } : {}}>{f.run === "Input Warning" ? "⚠️ Input Configuration Warning" : f.run + " Intercepted"}</div>
                      <div style={{fontSize: '0.9rem', marginBottom: '0.5rem', color: isWarning ? '#664d03' : '#495057'}}><strong>Stage:</strong> {f.stage}</div>
                      <div style={{fontSize: '0.95rem'}}><strong>{isWarning ? "Description:" : "Mathematical Reason:"}</strong> {f.reason}</div>
                    </div>
                  );
                })}
              </div>
            )}
          </div>


          {/* Raw Console Log Output for advanced debugging */}
          <div className="panel" style={{marginTop: '2rem'}}>
            <h3 style={{fontSize: '1.1rem', color: 'var(--text-muted)'}}>Terminal Log</h3>
            <pre style={{
              background: '#212529', 
              padding: '1.5rem', 
              borderRadius: '4px', 
              overflowX: 'auto',
              fontSize: '0.85rem',
              color: '#f8f9fa',
              fontFamily: 'SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace',
              lineHeight: 1.4
            }}>
              {result.rawOutput}
            </pre>
          </div>
        </div>
      )}
    </div>
  );
}