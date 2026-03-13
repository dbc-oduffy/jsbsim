# JSBSim Propulsion — Implementation Deep Dive

**Compiled:** 2026-03-11
**Purpose:** Technical notes for porting/adapting JSBSim propulsion patterns to DroneSim
**Audience:** Research agents, architecture review

---

## 1. Motor Model: FGBrushLessDCMotor Key Algorithm

### 3-Constant Equation Set
```
Given: Throttle [0–1], Voltage [V], Atmosphere (ρ, alt)

1. Effective Voltage:  V_eff = MaxVolts × Throttle
                       (assumes motor voltage proportional to throttle)

2. No-load Torque:     τ₀ = I₀ × Kv_τ
                       (proportional to no-load current and back-EMF constant)

3. Motor RPM:          RPM = Kv × V_eff × (1 − efficiency_loss)
                       (empirical; Kv in [RPM/V], simplified model)

4. Total Current:      I_total = I₀ + τ / K_τ
                       (no-load + load-dependent; K_τ from Kv)

5. Resistive Loss:     P_loss = I_total² × Rm
                       (I²R heating in coil)

6. Output Power:       P_out [W] = P_in − P_loss − P_friction
                       (simplified; friction subsumed into I₀)

7. Horsepower:         HP = P_out / 745.7  [hp]
                       (returned as GetPowerAvailable() × hptoftlbssec)
```

### Limitations vs. Real BLDC
- **No back-EMF saturation:** Assumes linear Kv regardless of speed/load
- **No temperature effects:** Rm and Kv assumed constant
- **No commutation losses:** Ideal ESC assumed
- **Algebraic only:** No shaft dynamics; RPM directly from throttle

### Suitable For
- Quick prototyping
- Real-time UAV simulation (low compute cost)
- Cases where Kv, Rm, I₀ are well-characterized from datasheet/testing

### Not Suitable For
- Transient dynamics (motor spinup, failsafe braking)
- Temperature-dependent performance
- Detailed efficiency mapping
- Back-EMF saturation at high speed

---

## 2. Propeller Model: FGPropeller Core Loop

### Thrust & Power Computation
```
INPUT: Engine Power Available [ft⋅lbf/s], Atmosphere

STATE: RPM, Pitch [deg]

COMPUTE:
  1. Angular velocity: ω [rad/s] = RPM × (2π/60)

  2. Advance ratio: J = V_airspeed / (n × D)
     where: n = rev/sec = RPM/60
            D = propeller diameter [ft]
            V_airspeed = aircraft true airspeed [ft/s]

  3. Lookup coefficients:
     Ct = C_THRUST_TABLE(J)        [dimensionless]
     Cp = C_POWER_TABLE(J)         [dimensionless]

  4. Apply Mach corrections (if tables provided):
     M_tip = helical tip Mach = (2πrn + V_parallel) / a_sound
     Ct_corrected = Ct × CT_MACH_TABLE(M_tip)
     Cp_corrected = Cp × CP_MACH_TABLE(M_tip)

  5. Thrust calculation:
     T = Ct_corrected × ρ × n² × D⁴  [lbs]
     (normalized by ρ₀ = 0.002377 slugs/ft³)

  6. Power required:
     P_req = Cp_corrected × ρ × n³ × D⁵  [ft⋅lbf/s]

  7. Torque balance (integrator):
     MotorTorque [ft⋅lbf] = PowerAvailable / ω
     PropDragTorque = P_req / ω
     ExcessTorque = MotorTorque − PropDragTorque

     dω/dt = ExcessTorque / Ixx
     RPM_new = RPM_old + (dω/dt) × dt × (60 / 2π)

OUTPUT: Thrust [lbs], Torque [ft⋅lbf], updated RPM, Moments (P-factor + gyro)
```

### P-Factor (Asymmetric Thrust Effect)
```
When yaw rate (r) ≠ 0, prop disk is tilted relative to airflow:
  - Right side of disk (at CW rotation) moves faster through air
  - Left side moves slower
  - Differential thrust creates roll moment

P_factor_moment = -P_factor_coef × Thrust × yaw_rate

(Sign convention: positive yaw → negative roll for CW prop)
```

### Gyroscopic Moment
```
Rotor angular momentum: h = Ixx × ω

Precession about body axes (yaw/roll):
  - If body rolls (p ≠ 0) with spinning prop: precesses in yaw
  - If body yaws (r ≠ 0) with spinning prop: precesses in roll

Moment_gyro = h × (p × k + r × j)  [vector cross product]

(version="1.1" in XML: correct sign; earlier versions had sign error for backward compat)
```

### Constant-Speed Governor (Optional)
```
if ConstantSpeed_mode enabled:
  target_RPM = NominalRPM  [e.g., 2000 RPM]

  if RPM < target_RPM − hysteresis:
    pitch = MinPitch  (lower pitch → higher RPM)
  elif RPM > target_RPM + hysteresis:
    pitch = MaxPitch  (higher pitch → lower RPM)
  else:
    pitch = current_pitch  (hold)

  Apply pitch limits: pitch = clamp(pitch, MinPitch, MaxPitch)
```

### Reverse Thrust
```
ReversePitch [deg]: pitch angle when fully reversed
Reverse_coef [0–1]: interpolation between MinPitch and ReversePitch

Pitch_actual = MinPitch + Reverse_coef × (ReversePitch − MinPitch)
```

### Advance Ratio Interpretation
```
J = 0:      Hovering; maximum Ct, moderate Cp
J = 0.1–0.3: Typical cruise for UAVs at moderate throttle
J = 0.5–0.7: High-speed flight; Ct → 0, Cp increasing (prop operates at low efficiency)
J > 0.7:    Windmilling; negative or near-zero Ct (prop drags)
```

---

## 3. Rotor Model: FGRotor Core Loop

### Rotor State Dynamics (Simplified)
```
INPUT: Collective [rad], Lateral [rad], Longitudinal [rad], Engine Power

STATE: RPM, a0 (coning), a1s (pitch flap), b1s (roll flap)

COMPUTE:
  1. Advance ratio: μ = V_forward / (Ω × R)

  2. Inflow ratio: λ = v_induced / (Ω × R)
     (v_induced initialized from momentum theory; updated by lag)

  3. Disk angle of attack (estimated from control inputs + dynamics):
     θ₀ = collective + small_angle_approximations(a1s, b1s)

  4. Thrust coefficient:
     C_T = (thrust) / (ρ × (Ω × R)² × π × R²)
     (from blade element theory)

  5. Blade flapping (differential equations):
     da0/dt ~ (thrust_coeff − λ) / LockNumber
     da1s/dt ~ (lateral_control − mu_related_term) / (tau + LockNumber)
     db1s/dt ~ (longitudinal_control − mu_related_term) / (tau + LockNumber)

     (Time constant τ = InflowLag; LockNumber from blade parameters)

  6. Inflow lag update:
     dv_induced/dt = (v_induced_equilibrium − v_induced) / InflowLag

  7. Torque balance:
     MotorTorque = PowerAvailable / ω
     PropDragTorque = Power_required / ω
     dω/dt = (MotorTorque − PropDragTorque − GearLoss) / (GearMoment + PolarMoment)

OUTPUT: Thrust [lbs], Torque [ft⋅lbf], Flapping angles, Induced velocity,
        Roll/Pitch moments (from blade asymmetry)
```

### Ground Effect Approximation
```
When near ground (height_AGL < 2–3 × rotor_diameter):

  Ground_effect_factor = exp(−GroundEffectExp × (height_AGL + GroundEffectShift))

  v_induced_in_GE = v_induced_without_GE × GroundEffectScaleNorm × Ground_effect_factor

  (GroundEffectScaleNorm allows runtime modulation; e.g., scaled to 0 on landing pads)

  Result: v_induced → 0 as ground is approached (reduced downwash loss near surface)
```

### Control Mapping (Main vs. Tail Rotor)
```
MAIN rotor (ControlMap = eMainCtrl):
  CollectiveCtrl ← propulsion/engine[0]/collective-ctrl-rad
  LateralCtrl ← propulsion/engine[0]/lateral-ctrl-rad
  LongitudinalCtrl ← propulsion/engine[0]/longitudinal-ctrl-rad

TAIL rotor (ControlMap = eTailCtrl):
  CollectiveCtrl ← propulsion/engine[x]/antitorque-ctrl-rad
  (LateralCtrl, LongitudinalCtrl ignored)

TANDEM rotor:
  Similar to main, but on rear fuselage (often controlled as secondary main)
```

### RPM Governance
```
if ExternalRPM ≥ 0:  # Linked to another rotor or external source
  RPM = GetExternalRPM() / SourceGearRatio
  EngineRPM = RPM × GearRatio

elif ExternalRPM = −1:  # Manually controlled via property
  RPM = property:propulsion/engine[x]/x-rpm-dict
  EngineRPM = RPM × GearRatio

else:  # Normal: integrated from torque balance (see above)
  (integrator computes dω/dt)
```

---

## 4. Force/Moment Transformation: FGForce Details

### Transform Matrix Computation
```
Native (wind or local) forces: [Fx, Fy, Fz]_native
Native moments:               [Mx, My, Mz]_native

Transform to body:
  [F_body] = Transform_Matrix × [F_native]
  [M_body] = Transform_Matrix × [M_native]

Moment arm calculation (if force doesn't act at CG):
  M_arm = (CG − ForceLocation) × F_body
  M_total = M_native + M_arm
```

### Transform Type: tCustom (Vectorable Thrust)
```
Used when force vector is NOT aligned with body/wind/local axes.
Example: Tilted motor arm on multirotor.

SetAnglesToBody(roll, pitch, yaw):  # Angles relative to body frame
  Builds rotation matrix:
    T = Rz(yaw) × Ry(pitch) × Rx(roll)

  Then: F_body = T × F_native

Typical usage for quadrotor:
  (Motor mounted on arm, arm rotated 90° for vertical thrust)
  SetAnglesToBody(roll=0, pitch=90°, yaw=0)
```

### Moment Arm Offset (P-Factor Example)
```
Propeller disk is at location [x_prop, y_prop, z_prop].
CG is at [0, 0, 0] in body frame.

SetLocation(x_prop, y_prop, z_prop)
SetActingLocation(x_act, y_act, z_act)  # May differ if modeling off-center thrust

Moment = (CG − ActingLocation) × F_body
       = [−x_act, −y_act, −z_act] × [Fx, Fy, Fz]
```

---

## 5. Tank Model: Fuel Management Integration

### Priority-Based Feed Sequence
```
Tanks sorted by Priority (1 = highest).

When engine requests fuel:
  for tank in sorted_tanks:
    if tank.Selected (Priority > 0):
      fuel_available = tank.Drain(fuel_requested)
      if fuel_available < fuel_requested:
        request_remaining = fuel_requested − fuel_available
        continue to next tank
      else:
        break  (tank supplied all requested fuel)

After all tanks drain:
  if TotalFuelRemaining <= 0:
    for engine in engines:
      engine.SetStarved(true)
```

### Unusual Features (Relevant for Future Expansion)
```
1. Standpipe:
   tank.Standpipe = amount [lbs] that cannot be dumped.
   tank.Drain() will not drain below Standpipe level.
   (Useful for emergency reserve modeling)

2. Unusable Volume:
   tank.UnusableVol = volume [gal] that cannot be burned.
   (Distinct from Standpipe; modeled as tank geometry constraint)

3. External Flow:
   tank.SetExternalFlow(rate [lbs/sec])
   Positive = inbound (refueling), negative = outbound (dumping)

4. Temperature Dynamics (Optional):
   If InitialTemperature is set in XML:
     Area = f(Capacity)  [estimated from tank geometry]
     dT/dt = (T_external − T_fuel) × HeatTransfer / (mass × SpecificHeat)
   (Useful for high-altitude / endurance flights; not critical for UAVs)
```

---

## 6. Property Tree Interface (Runtime Observability)

### Engine Properties (Read/Write)
```
propulsion/engine[i]/
  throttle-cmd                [0–1] write: pilot command
  throttle-pos                [0–1] read: actual position (filtered)
  mixture-cmd                 [0–1] write: pilot command (piston engines)
  mixture-pos                 [0–1] read: actual position
  prop-pitch-cmd              [deg] write: variable-pitch command
  prop-pitch-actual           [deg] read: current pitch
  prop-advance-cmd            [0–1] write: advance ratio command
  starter-cmd                 [bool] write: starter engagement
  magnetos-cmd                [int] write: magneto setting (piston)
  cutoff-cmd                  [bool] write: engine shutoff

  running                     [bool] read: engine running state
  starved                     [bool] read: fuel starvation
  cranking                    [bool] read: starter active
  fuel-flow-rate-pph          [lbs/hr] read: instantaneous flow
  fuel-flow-rate-gph          [gal/hr] read: instantaneous flow (alt units)
  fuel-used                   [lbs] read: cumulative
  fuel-density                [lbs/gal] read/write: used for GPH conversion

  (Thruster properties)
  x-rpm                       [RPM] read: rotor/propeller RPM
  thrust                      [lbs] read: current thrust output
  power-required              [hp] read: power absorbed (propellers/rotors)
  reverser-angle              [rad] write: reverser positioning

  (BLDC/Electric specific)
  x-voltage                   [V] read: effective voltage
  x-current                   [A] read: total current draw
  x-power-in                  [W] read: input power
```

### Tank Properties (Read/Write)
```
propulsion/tank[i]/
  contents-lbs                [lbs] read/write: current contents
  contents-gal                [gal] read/write: current contents (alt units)
  capacity-lbs                [lbs] read: max capacity
  capacity-gal                [gal] read: max capacity (alt units)
  percent-full                [%] read: fill level (0–100)
  temperature-c               [°C] read: fuel temperature
  temperature-f              [°F] read: fuel temperature (alt units)
  selected                    [bool] read: whether tank is in feed sequence
  priority                    [int] write: feed priority (1 = first)
  density                     [lbs/gal] write: fuel density
  external-flow-rate-pps      [lbs/sec] write: external drain/refuel rate
```

---

## 7. Key Integration Points for DroneSim Porting

### 1. Motor Model Adaptation
```
DroneSim has native DGBrushLessDCMotor with more physics:
  - Temperature-dependent Kv and Rm
  - ESC throttle filtering and startup transients
  - Battery discharge model (voltage sag)
  - Commutation phase alignment

JSBSim's FGBrushLessDCMotor is simpler (algebraic, no thermal).

Option A: Use JSBSim's FGBrushLessDCMotor as baseline, enhance with thermal/battery
Option B: Adapt DroneSim's motor model to JSBSim's XML config format
Option C: Implement hybrid (use DroneSim for high-fidelity cases, JSBSim for real-time)
```

### 2. Propeller Model Integration
```
JSBSim's FGPropeller provides:
  ✓ Table-driven Ct/Cp vs. advance ratio (industry standard)
  ✓ Mach corrections (important at high altitude/speed)
  ✓ P-factor and gyroscopic effects
  ✓ Constant-speed governor (not needed for DroneSim quadrotors, but nice for fixed-wing)

DroneSim's propeller model:
  ✓ Simpler analytic model (Ct_hover, effective_pitch)
  ✗ No Mach effects (OK for low-speed UAVs)
  ✓ Rotor-specific (disk loading model)

Recommendation: Use JSBSim's FGPropeller for fixed-wing, adapt for rotor disks.
```

### 3. Rotor Model (Quadrotors)
```
JSBSim's FGRotor models helicopters (full 6-DOF flapping, cyclic/collective controls).

DroneSim's rotor model:
  ✓ Simpler momentum theory (thrust ∝ Ct × ρ × Ω²)
  ✗ No flapping dynamics (acceptable; quadrotors have rigid hubs)
  ✓ Ground effect approximation (same formula as JSBSim)
  ✗ No cyclic control (N/A for quadrotors)

Recommendation: Keep DroneSim's simpler model for quadrotors; JSBSim's FGRotor overkill.
Use JSBSim's ground effect formula as-is.
```

### 4. Force/Moment Transformation
```
JSBSim's FGForce is a general-purpose utility (excellent design).

DroneSim could reuse this with minimal adaptation:
  - tCustom transform for tilted motor arms
  - Moment-arm calculations for offset thrust
  - P-factor modeling (if needed for fixed-wing)

Recommendation: Adopt FGForce logic into DroneSim's propulsion component
(either as code pattern or direct integration if C++ boundary allows).
```

### 5. Tank/Fuel Management
```
JSBSim's FGTank is comprehensive (priority feeds, thermal, grains, etc.).

DroneSim electric quadrotors:
  - No fuel (battery instead)
  - Could model battery as "tank" (SOC = percent full, discharge = drain)
  - Priority-based cell balancing (future feature?)

Recommendation: Don't port tanks unless adding fixed-wing hybrid-electric in future.
Model battery separately (orthogonal domain).
```

---

## 8. Configuration File Patterns

### Minimal BLDC Motor Definition
```xml
<!-- Example: DJI E305 from F450 -->
<brushless_dc_motor name="DJI E305">
  <velocityconstant> 960 </velocityconstant>      <!-- Kv [RPM/V] -->
  <coilresistance> 0.117 </coilresistance>        <!-- Rm [Ω] -->
  <noloadcurrent> 0.45 </noloadcurrent>           <!-- I₀ [A] -->
  <maxvolts> 14.63 </maxvolts>                    <!-- V_max [V] -->
</brushless_dc_motor>
```

**How to infer missing parameters:**
- `Kv = 960 RPM/V` → Max RPM ≈ 960 × 14.63 ≈ 14,045 RPM @ nominal voltage
- `Rm = 0.117 Ω` → At rated current (e.g., 5A), I²R loss ≈ 3W
- `I₀ = 0.45 A` → Friction/windage draw; scales with RPM
- `MaxVolts = 14.63 V` → Safe operating voltage (e.g., 4S LiPo nominal = 14.8V, safe limit)

### Minimal Propeller Definition
```xml
<!-- Example: DJI 9450 from F450 -->
<propeller name="DJI 9450" version="1.1">
  <ixx unit="KG*M2"> 6.05e-05 </ixx>
  <diameter unit="IN"> 9.4 </diameter>
  <numblades> 2 </numblades>
  <constspeed> 0 </constspeed>

  <table name="C_THRUST" type="internal">
    <tableData>
      0.000   0.1288
      0.100   0.1207
      0.200   0.1053
      ...
      0.700   0.0064
    </tableData>
  </table>

  <table name="C_POWER" type="internal">
    <tableData>
      0.000   0.0666
      0.100   0.0595
      0.200   0.0531
      ...
      0.700   0.0099
    </tableData>
  </table>
</propeller>
```

**How to generate/interpolate tables from APC datasheets:**
- APC publishes thrust vs. RPM tables at fixed airspeeds
- Convert to advance ratio: J = V / (n × D)
- Normalize thrust/power to dimensionless coefficients: Ct, Cp
- Create lookup table: J → (Ct, Cp)
- Use JSBSim's FGTable interpolator for intermediate J values

---

## 9. Debugging & Validation Checklist

### Motor Sanity Checks
```
[ ] RPM stays within reasonable range (0 to Kv × MaxVolts)
[ ] Current draw increases with throttle and load
[ ] No-load current visible at idle (baseline)
[ ] Power output = V × I, accounting for losses
[ ] Motor doesn't exceed MaxVolts (ESC protection)
```

### Propeller Sanity Checks
```
[ ] At J=0 (hover): Ct ≈ 0.1–0.15, Cp ≈ 0.05–0.07 (typical)
[ ] As J increases: Ct decreases monotonically
[ ] At high J: Ct → 0 or negative (prop stalls/windmills)
[ ] P-factor moment changes sign with yaw rate sign
[ ] Gyroscopic effect magnitude ~ Ixx × ω × body_rate
[ ] Thrust & power units consistent (lbs, ft⋅lbf/s, hp)
```

### Rotor Sanity Checks
```
[ ] Collective increases thrust smoothly
[ ] Flapping angles (a0, a1s, b1s) stay within ±45° (typical)
[ ] Inflow ratio λ ≈ 0.05–0.15 in hover (typical)
[ ] Ground effect reduces power required as height → 0
[ ] RPM stabilizes at equilibrium (motor torque ≈ prop drag torque)
```

### System Integration Checks
```
[ ] Total thrust from 4 motors sums correctly
[ ] Moment arms computed from motor locations
[ ] Fuel feed sequence respects tank priority
[ ] Engine starvation flag raised when fuel depleted
[ ] Property tree reflects all state (visible in replay/logging)
```

---

## 10. Performance & Scalability Notes

### Computational Cost
- **FGBrushLessDCMotor**: O(1) algebraic per motor per frame (negligible)
- **FGPropeller**: O(log N) per prop (table lookups); ~2–3 microseconds on modern CPU
- **FGRotor**: O(1) per rotor (iterative flapping solver); ~5–10 microseconds
- **FGTank**: O(1) per tank (drain/fill); negligible
- **FGForce**: O(1) per force (matrix multiply); negligible

**Total for F450 quad:** ~4 motors + 4 props + 1 fuel tank ≈ 20–30 microseconds @ 1000 Hz = **2–3% CPU per frame** (very efficient)

### Numerical Stability
- **RPM integration**: Use semi-implicit Euler (symplectic) to avoid instability at low time steps
- **Flapping angles** (rotor): Small-angle approximation valid for helicopter; quadrotors with rigid hubs don't flap significantly
- **Inflow lag**: Time constant 0.1–0.2 s; ensure dt << InflowLag
- **Pitch control saturation**: Clamp Pitch to [MinPitch, MaxPitch] every frame

### Scalability (Multi-Rotor vs. Helicopter)
```
Quadrotor (4 motors):
  - Use FGBrushLessDCMotor + simplified FGPropeller
  - No cyclic control (collective only)
  - Rotor speeds are independent
  - Scaling: linear in number of rotors

Helicopter (1 main + 1 tail):
  - Use FGBrushLessDCMotor + full FGRotor
  - Cyclic/collective/antitorque control
  - Tail rotor RPM linked to main (gear-driven)
  - Scaling: constant per helicopter (not linear in parts)

Hybrid Fixed-Wing:
  - Wing: Aerodynamics subsystem (separate model)
  - Fuselage motors (pusher/puller): FGBrushLessDCMotor + FGPropeller
  - Vertical thrusters: FGBrushLessDCMotor + FGPropeller or FGRotor
  - Tank: FGTank (fuel for ICE or hybrid battery)
```

---

## Summary: JSBSim Propulsion Design Philosophy

1. **Separation of Concerns:**
   - **FGEngine**: Throttle input → Power output (agnostic to motor type)
   - **FGThruster**: Power input → Thrust + Torque output
   - **FGForce**: Force → Moment transformation (orthogonal)
   - **FGTank**: Fuel accounting (orthogonal)

2. **Extensibility via Inheritance:**
   - New motor types: subclass FGEngine, implement Calculate()
   - New thrusters: subclass FGThruster, implement Calculate(power)
   - New forces: subclass FGForce, override SetTransformType()

3. **Configuration-Driven:**
   - No hardcoded parameters; all tuning via XML
   - Tables interpolated at runtime; easy to swap propeller definitions
   - Property tree enables dynamic reconfiguration

4. **Physics Fidelity Spectrum:**
   - **Low:** FGElectric (linear power)
   - **Medium:** FGBrushLessDCMotor (3-constant BLDC) + FGPropeller (table-driven)
   - **High:** FGRotor (blade element + flapping dynamics)
   - Choose fidelity to match simulation goals (real-time vs. accuracy)

