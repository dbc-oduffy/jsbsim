# JSBSim Propulsion Domain File Inventory
## Electric Brushless + Propeller Systems

**Compiled:** 2026-03-11
**Purpose:** Foundation document for JSBSim propulsion architecture research
**Scope:** Electric brushless DC motor systems + propeller aerodynamics + engine/thruster hierarchy

---

## 1. FGPropulsion (System Manager)
**Location:** `src/models/FGPropulsion.h` / `.cpp`
**Line Count:** ~380 header + 906 implementation = 1,286 total

### Key Classes
- **FGPropulsion** — Container/manager for all engines and tanks in the aircraft

### Public API (Core Methods)
- `Run(bool Holding) → bool` — Main update loop; executes all engines
- `InitModel() → bool` — Initialization routine
- `Load(Element* el) → bool` — XML config loader for `<propulsion>` section
- `GetNumEngines() → size_t` — Engine count
- `GetEngine(unsigned int index) → FGEngine*` — Engine accessor
- `GetNumTanks() → size_t` — Tank count
- `GetTank(unsigned int index) → FGTank*` — Tank accessor
- `GetSteadyState() → bool` — Loops engines until steady-state (trimming)
- `InitRunning(int n)` / `SetEngineRunning(int index)` — Engine startup
- `GetForces() → const FGColumnVector3&` — Total propulsive forces
- `GetMoments() → const FGColumnVector3&` — Total propulsive moments
- `Transfer(int src, int tgt, double amount) → double` — Fuel transfer between tanks
- `DoRefuel(double time_slice)` / `DumpFuel(double time_slice)` — Fuel management
- `GetTanksMoment() → const FGColumnVector3&` — Tank inertia moment
- `GetTanksWeight() → double` — Total tank mass

### Private Members
- `Engines: std::vector<std::shared_ptr<FGEngine>>` — Engine instances
- `Tanks: std::vector<std::shared_ptr<FGTank>>` — Tank instances
- `vForces: FGColumnVector3` — Aggregated forces (body frame)
- `vMoments: FGColumnVector3` — Aggregated moments
- `ActiveEngine: int` — Currently selected engine index
- `FuelFreeze: bool` — Freeze fuel consumption (for testing)
- `TotalFuelQuantity, TotalOxidizerQuantity: double` — Global inventory
- `DumpRate, RefuelRate: double` — Fuel management rates

### Configuration Format
```xml
<propulsion>
  <engine file="{string}">
    ... (see FGEngine) ...
  </engine>
  <tank type="{FUEL | OXIDIZER}">
    ... (see FGTank) ...
  </tank>
  <dump-rate unit="{LBS/MIN | KG/MIN}"> {number} </dump-rate>
  <refuel-rate unit="{LBS/MIN | KG/MIN}"> {number} </refuel-rate>
</propulsion>
```

### Physics/Constants
- None directly; delegates to engines and tanks

---

## 2. FGBrushLessDCMotor (BLDC Electric Motor)
**Location:** `src/models/propulsion/FGBrushLessDCMotor.h` / `.cpp`
**Line Count:** ~107 header + 233 implementation = 340 total
**Inherits from:** FGEngine

### Key Classes
- **FGBrushLessDCMotor** — Permanent-magnet synchronous motor model based on "3 constant motor equations"

### Model Type & Physics
**3-Constant Motor Model:**
- Kv (speed constant) [RPM/Volt] — converts voltage to angular velocity
- Rm (coil resistance) [Ohms] — internal winding loss
- I₀ (no-load current) [Amperes] — friction/windage draw

**Equations (implied from constants):**
- RPM = Kv × V_effective
- I_torque = (I_total - I₀)  [effective current after no-load draw]
- Torque ∝ I_torque
- Power = I × V, efficiency = (useful power) / (total input power)

### Public API
- Constructor: `FGBrushLessDCMotor(FGFDMExec* exec, Element* el, int engine_number, FGEngine::Inputs& input)`
- `Calculate() → void` — Core computation: throttle → RPM/Current/Power
- `GetPowerAvailable() → double` — Returns HP × hptoftlbssec (foot-lbs/sec)
- `CalcFuelNeed() → double` — Returns 0 (no fuel; battery-powered)
- `GetEngineLabels(const std::string& delimiter) → std::string`
- `GetEngineValues(const std::string& delimiter) → std::string`

### Private Members
- `ZeroTorqueCurrent: double` — No-load current [A]
- `CoilResistance: double` — Winding resistance [Ω]
- `PowerWatts: double` — Max power rating [W]
- `MaxVolts: double` — Battery nominal voltage [V]
- `Kv: double` — Speed constant [RPM/V]
- `HP: double` — Output power [hp]
- `Current: double` — Total current draw [A]

### Configuration Format
```xml
<brushless_dc_motor>
  <maxvolts units="VOLTS"> {number} </maxvolts>
  <velocityconstant units="RPM/V"> {number} </velocityconstant>
  <coilresistance units="OHMS"> {number} </coilresistance>
  <noloadcurrent units="AMPERES"> {number} </noloadcurrent>
</brushless_dc_motor>
```

### Physics Constants (Code)
```cpp
constexpr double NMtoftpound = 1.3558;          // N⋅m → ft⋅lbf
constexpr double hptowatts = 745.7;             // hp → W
constexpr double WattperRPMtoftpound = 60 / (2 * M_PI * NMtoftpound);
```

---

## 3. FGElectric (Basic Electric Motor — Legacy)
**Location:** `src/models/propulsion/FGElectric.h` / `.cpp`
**Line Count:** ~98 header + 187 implementation = 285 total
**Inherits from:** FGEngine

### Model Type & Physics
**Linear throttle-power model:**
- Throttle (0–1) linearly maps to 0–MaxPower
- No battery model; infinite power available
- No internal losses; simple input → output

### Public API
- Constructor: `FGElectric(FGFDMExec* exec, Element* el, int engine_number, FGEngine::Inputs& input)`
- `Calculate() → void` — Throttle → HP output
- `GetPowerAvailable() → double` — Returns HP × hptoftlbssec
- `getRPM() → double` — Returns RPM
- `GetEngineLabels(const std::string& delimiter) → std::string`
- `GetEngineValues(const std::string& delimiter) → std::string`

### Private Members
- `PowerWatts: double` — Max power [W]
- `RPM: double` — Motor revolutions per minute
- `HP: double` — Output [hp]
- `hptowatts: double` — Conversion constant (745.7)

### Configuration Format
```xml
<electric>
  <power unit="{WATTS | HORSEPOWER}"> {number} </power>
</electric>
```

### Notes
- Simplified compared to FGBrushLessDCMotor
- No battery model, no internal losses modeled
- Used for rapid prototyping or basic electric aircraft

---

## 4. FGPropeller (Fixed/Variable-Pitch Propeller)
**Location:** `src/models/propulsion/FGPropeller.h` / `.cpp`
**Line Count:** ~347 header + 475 implementation = 822 total
**Inherits from:** FGThruster

### Key Physics Model
**Advance Ratio & Thrust/Power Coefficients:**
- Advance ratio: `J = Vt / (n × D)` — where Vt = airspeed, n = rps, D = diameter
- Thrust: `T = Ct × ρ × n² × D⁴` — from lookup tables indexed by J
- Power: `P = Cp × ρ × n³ × D⁵` — from lookup tables indexed by J
- Gyroscopic moment due to propeller spin + p-factor (asymmetric thrust at yaw)

**Optional Mach corrections:**
- CT_MACH table: scales Ct based on helical tip Mach number
- CP_MACH table: scales Cp based on helical tip Mach number

### Public API (Key Methods)
- Constructor: `FGPropeller(FGFDMExec* exec, Element* el, int num = 0)`
- `Calculate(double EnginePower) → double` — Integrates torque balance; returns thrust [lbs]
- `SetRPM(double rpm)` / `GetRPM() → double` — Direct RPM access
- `SetEngineRPM(double rpm)` — RPM via engine gear ratio
- `SetPitch(double pitch)` — Blade pitch angle [deg]
- `SetAdvance(double advance)` — Pitch command as %: 0.0 = min, 1.0 = max
- `SetConstantSpeed(int mode)` — Enable/disable constant-speed prop governor
- `SetSense(double s)` — Rotation sense: +1 = CW (from rear), -1 = CCW
- `SetPFactor(double pf)` / `GetPFactor() → FGColumnVector3` — P-factor moment
- `SetCtFactor(double ctf)` / `GetCtFactor() → double` — Ct multiplier (scaling)
- `SetCpFactor(double cpf)` / `GetCpFactor() → double` — Cp multiplier
- `IsVPitch() → bool` — True if variable-pitch (MaxPitch ≠ MinPitch)
- `GetPowerRequired() → double` — Absorbed power at current conditions
- `GetDiameter() → double`, `GetIxx() → double` — Geometry accessors
- `GetThrustCoefficient() → double`, `GetHelicalTipMach() → double` — State accessors
- `SetReverse(bool r)` / `GetReverse() → bool` — Reverse thrust
- `SetFeather(bool f)` / `GetFeather() → bool` — Feathered (zero thrust)
- `GetCThrustTable() → FGTable*`, `GetCPowerTable() → FGTable*` — Coefficient tables
- `GetCtMachTable() → FGTable*`, `GetCpMachTable() → FGTable*` — Mach correction tables

### Private Members (Physics State)
- `numBlades: int`
- `J: double` — Advance ratio
- `RPM: double` — Current rotation rate
- `Ixx: double` — Rotational inertia (moment) [slug⋅ft²]
- `Diameter: double` — Propeller disk diameter [in]
- `MaxPitch, MinPitch: double` — Blade pitch limits [deg]
- `MinRPM, MaxRPM: double` — RPM operating range (for const-speed mode)
- `Pitch: double` — Current blade pitch [deg]
- `P_Factor: double` — P-factor moment coefficient
- `Sense, Sense_multiplier: double` — Rotation direction
- `Advance: double` — Pitch command [0–1]
- `ExcessTorque: double` — Remaining torque after inertial acceleration
- `HelicalTipMach: double` — Mach number at blade tips
- `Vinduced: double` — Induced velocity at disk
- `D4, D5: double` — Pre-computed diameter powers (D⁴, D⁵)
- `vTorque: FGColumnVector3` — Torque applied to aircraft
- `cThrust, cPower: FGTable*` — Ct(J), Cp(J) lookup tables
- `CtMach, CpMach: FGTable*` — Mach correction factors (optional)
- `CtFactor, CpFactor: double` — Coefficient multipliers
- `ConstantSpeed: int` — 0 = manual pitch, nonzero = const-speed mode
- `ReversePitch, Reverse_coef, Reversed: bool`
- `Feathered: bool`

### Configuration Format
```xml
<propeller name="{string}" version="{string}">
  <ixx> {number} </ixx>
  <diameter unit="IN"> {number} </diameter>
  <numblades> {number} </numblades>
  <gearratio> {number} </gearratio>
  <minpitch> {number} </minpitch>
  <maxpitch> {number} </maxpitch>
  <minrpm> {number} </minrpm>
  <maxrpm> {number} </maxrpm>
  <constspeed> {0 | 1} </constspeed>
  <reversepitch> {number} </reversepitch>
  <ct_factor> {number} </ct_factor>
  <cp_factor> {number} </cp_factor>

  <table name="C_THRUST" type="internal">
    <tableData> {advance_ratio  ct_coeff ...} </tableData>
  </table>

  <table name="C_POWER" type="internal">
    <tableData> {advance_ratio  cp_coeff ...} </tableData>
  </table>

  <table name="CT_MACH" type="internal">
    <tableData> {mach  ct_factor ...} </tableData>
  </table>

  <table name="CP_MACH" type="internal">
    <tableData> {mach  cp_factor ...} </tableData>
  </table>
</propeller>
```

### Key Physics References
- Barnes W. McCormick, "Aerodynamics, Aeronautics, and Flight Mechanics", 1979
- Hartman & Biermann, "Aerodynamic Characteristics of Full Scale Propellers", NACA TN-640
- Various NACA Technical Notes

### Version Attribute
- `version="1.1"` (or higher): Correct gyroscopic moment sign
- Absent or `1.0`: Legacy (incorrect) gyroscopic moment sign for backward compatibility

---

## 5. FGRotor (Helicopter Rotor Model)
**Location:** `src/models/propulsion/FGRotor.h` / `.cpp`
**Line Count:** ~433 header + 905 implementation = 1,338 total
**Inherits from:** FGThruster

### Model Type & Physics
**Momentum Theory + Blade Element:**
- Rotor disk thrust model with collective/cyclic control
- Blade flapping dynamics (coning a₀, longitudinal a₁ₛ, lateral b₁ₛ)
- Inflow lag (rotor response time constant)
- Ground effect approximation: exp(−groundeffectexp × (height + groundeffectshift))
- Transmission (gear, brake, clutch) integration via FGTransmission class

**Key Angles/Ratios:**
- Advance ratio: `μ = V_forward / (Ω × R)` (tip speed ratio)
- Inflow ratio: `λ = v_induced / (Ω × R)` (normalized induced velocity)
- Induced inflow ratio: `ν = induced_velocity / (Ω × R)`

**Control Inputs (via property tree):**
- `propulsion/engine[x]/collective-ctrl-rad` [rad]
- `propulsion/engine[x]/lateral-ctrl-rad` [rad]
- `propulsion/engine[x]/longitudinal-ctrl-rad` [rad]
- `propulsion/engine[x]/antitorque-ctrl-rad` (tail rotor) [rad]

### Public API (Key Methods)
- Constructor: `FGRotor(FGFDMExec *exec, Element* rotor_element, int num)`
- `Calculate(double EnginePower) → double` — Main integrator; returns scalar thrust
- `GetRPM() → double` / `SetRPM(double rpm)`
- `GetEngineRPM() → double` / `SetEngineRPM(double rpm)`
- `GetGearRatio() → double`
- `GetThrust() → double` — Scalar rotor thrust [lbs]
- `GetPowerRequired() → double` — Power absorbed at current state [hp]
- **Blade/Flow State Accessors:**
  - `GetA0() → double` — Coning angle [rad]
  - `GetA1() → double` — Longitudinal flapping (shaft coords)
  - `GetB1() → double` — Lateral flapping (shaft coords)
  - `GetLambda() → double` — Inflow ratio
  - `GetMu() → double` — Tip-speed ratio
  - `GetNu() → double` — Induced inflow ratio
  - `GetVi() → double` — Induced velocity [ft/s]
  - `GetCT() → double` — Thrust coefficient
  - `GetTorque() → double` — Reaction torque [ft⋅lbf]
- **Downwash Angles:**
  - `GetThetaDW() → double` — Pitch downwash [rad]
  - `GetPhiDW() → double` — Roll downwash [rad]
- **Ground Effect:**
  - `GetGroundEffectScaleNorm() → double` / `SetGroundEffectScaleNorm(double g)`
- **Control Input Accessors:**
  - `GetCollectiveCtrl(), SetCollectiveCtrl(double c)` [rad]
  - `GetLateralCtrl(), SetLateralCtrl(double c)` [rad]
  - `GetLongitudinalCtrl(), SetLongitudinalCtrl(double c)` [rad]

### Private Members (Configuration)
- `Radius: double` — Rotor disk radius [ft]
- `BladeNum: int` — Number of blades
- `BladeChord: double` — Blade chord length [ft]
- `LiftCurveSlope: double` — Section lift curve slope [1/rad]
- `BladeTwist: double` — Twist from root to tip [rad]
- `HingeOffset: double` — Flapping hinge offset [ft]
- `BladeFlappingMoment: double` — Flapping moment of inertia [slug⋅ft²]
- `BladeMassMoment: double` — Blade mass moment [slug⋅ft]
- `PolarMoment: double` — Rotor disk polar moment [slug⋅ft²]
- `InflowLag: double` — Inflow time constant [s] (typical 0.1–0.2 s)
- `TipLossB: double` — Tip-loss factor (0.95–1.0)

### Private Members (Ground Effect)
- `GroundEffectExp: double` — Ground effect exponent (0.04–0.1)
- `GroundEffectShift: double` — Height adjustment [ft]
- `GroundEffectScaleNorm: double` — Runtime scaling (0–1)

### Private Members (Dynamic State)
- `RPM: double` — Current rotor rotation rate
- `Omega: double` — Angular velocity [rad/s]
- `a0, a1s, b1s: double` — Flapping angles [rad]
- `C_T: double` — Thrust coefficient
- `lambda, mu, nu: double` — Flow ratios
- `v_induced: double` — Induced velocity [ft/s]
- `Torque: double` — Reaction torque [ft⋅lbf]
- `theta_downwash, phi_downwash: double` — Downwash angles [rad]

### Private Members (Transmission)
- `Transmission: FGTransmission*` — Gear/brake/clutch integration
- `EngineRPM: double` — Engine-side RPM
- `MaxBrakePower: double` — Rotor brake max power [hp]
- `GearLoss: double` — Friction loss [hp]
- `GearMoment: double` — Transmission inertia [slug⋅ft²]

### Configuration Format
```xml
<rotor name="{string}">
  <diameter unit="{LENGTH}"> {number} </diameter>
  <numblades> {number} </numblades>
  <gearratio> {number} </gearratio>
  <nominalrpm> {number} </nominalrpm>
  <minrpm> {number} </minrpm>
  <maxrpm> {number} </maxrpm>
  <chord unit="{LENGTH}"> {number} </chord>
  <liftcurveslope> {number} </liftcurveslope>
  <twist unit="{ANGLE}"> {number} </twist>
  <hingeoffset unit="{LENGTH}"> {number} </hingeoffset>
  <flappingmoment unit="{MOMENT}"> {number} </flappingmoment>
  <massmoment unit="SLUG*FT"> {number} </massmoment>
  <polarmoment unit="{MOMENT}"> {number} </polarmoment>
  <inflowlag> {number} </inflowlag>
  <tiplossfactor> {number} </tiplossfactor>
  <maxbrakepower unit="{POWER}"> {number} </maxbrakepower>
  <gearloss unit="{POWER}"> {number} </gearloss>
  <gearmoment unit="{MOMENT}"> {number} </gearmoment>
  <controlmap> {MAIN | TAIL | TANDEM} </controlmap>
  <ExternalRPM> {number} </ExternalRPM>
  <groundeffectexp> {number} </groundeffectexp>
  <groundeffectshift unit="{LENGTH}"> {number} </groundeffectshift>
</rotor>
```

### Key Physics References
- Shaughnessy et al., "Development and Validation of a Piloted Simulation of a Helicopter", NASA TP-1285
- Bailey, "Simplified Theoretical Method of Determining Rotor Characteristics", NACA Rep. 716
- Amer, "Theory of Helicopter Damping", NACA TN-2136
- Talbot & Corliss, "Mathematical Force and Moment Model", NASA TM-73,254
- Gessow & Amer, "Introduction to Physical Aspects of Helicopter Stability", NACA TN-1982

---

## 6. FGEngine (Base Engine Class)
**Location:** `src/models/propulsion/FGEngine.h` / `.cpp`
**Line Count:** ~230 header + 294 implementation = 524 total
**Inherits from:** FGModelFunctions (abstract)

### Model Type & Physics
**Abstract base class** for all engine types:
- Contains common fuel management, state tracking, property binding
- Subclasses: FGElectric, FGBrushLessDCMotor, FGPiston, FGTurbine, FGTurboprop, FGRocket

### Engine Input Struct
```cpp
struct Inputs {
  double Pressure, PressureRatio, Temperature, Density, DensityRatio;
  double Soundspeed, TotalPressure, TAT_c;
  double Vt, Vc, qbar, alpha, beta, H_agl;
  FGColumnVector3 AeroUVW, AeroPQR, PQRi;
  std::vector<double> ThrottleCmd, MixtureCmd, ThrottlePos, MixturePos, PropAdvance;
  std::vector<bool> PropFeather;
  double TotalDeltaT;
};
```

### Public API (Virtual Methods)
- `Calculate() = 0` — Abstract; must be overridden by subclass
- `GetType() → EngineType` — Returns engine type enum
- `GetName() → const std::string&`
- `GetThrottle[Min|Max]() → double`
- `GetStarter() → bool` — Is starter engaged?
- `GetRunning() → bool` — Engine running?
- `GetCranking() → bool` — Engine cranking (starting)?
- `GetStarved() → bool` — Fuel starvation?
- `SetStarved(bool)` / `SetStarved()` — Set starvation state
- `SetRunning(bool)`, `SetName(const std::string&)`
- `SetFuelFreeze(bool f)`, `SetFuelDensity(double d)`
- `GetFuelFlow_gph() / GetFuelFlow_pph() → double` — Fuel flow [gal/hr or lb/hr]
- `GetFuelFlowRate() → double` — Fuel flow [lb/sec]
- `GetFuelFlowRateGPH() → double` — Fuel flow [gal/hr]
- `GetFuelUsedLbs() → double` — Cumulative fuel consumed [lbs]
- `GetPowerAvailable() → double` — Available power (0 for base class)
- `CalcFuelNeed() → double` — Computed fuel requirement [lbs]
- `CalcOxidizerNeed() → double` — Oxidizer requirement [lbs] (0 for air-breathing)
- `GetBodyForces() → const FGColumnVector3&` — Force vector
- `GetMoments() → const FGColumnVector3&` — Moment vector
- `ResetToIC() → void` — Reset to initial conditions
- `GetThruster() → FGThruster*` — Attached thruster accessor
- `GetSourceTank(unsigned int i) → unsigned int` — Fuel feed tank index
- `GetNumSourceTanks() → size_t`
- `LoadThruster(FGFDMExec* exec, Element *el)` — Create/load thruster
- `GetEngineLabels/Values(const std::string& delimiter) → std::string`

### Protected Members
- `Name: std::string`
- `EngineNumber: const int` — Index in engine array
- `Type: EngineType` — Enum: etUnknown, etRocket, etPiston, etTurbine, etTurboprop, etElectric
- `SLFuelFlowMax: double` — Max fuel flow at sea level [lb/hr]
- `MaxThrottle, MinThrottle: double` — Throttle limits [0–1]
- `FuelExpended: double` — Total fuel used [lbs]
- `FuelFlowRate: double` — Instantaneous flow [lb/sec]
- `PctPower: double` — Power percentage (0–100)
- `Starter, Starved, Running, Cranking: bool` — State flags
- `FuelFreeze: bool`
- `FuelFlow_gph, FuelFlow_pph: double` — Flow rate variants
- `FuelUsedLbs: double` — Cumulative [lbs]
- `FuelDensity: double` — Fuel density [lbs/gal]
- `Thruster: FGThruster*` — Attached thruster object
- `SourceTanks: std::vector<int>` — Indices of fuel feed tanks

### Configuration Format (Generic)
```xml
<engine file="{string}">
  <feed> {tank_index} </feed>
  ... (more feed tank indices) ...
  <thruster file="{string}">
    <location unit="{IN | M}">
      <x> {number} </x>
      <y> {number} </y>
      <z> {number} </z>
    </location>
    <orient unit="{RAD | DEG}">
      <roll> {number} </roll>
      <pitch> {number} </pitch>
      <yaw> {number} </yaw>
    </orient>
  </thruster>
</engine>
```

---

## 7. FGThruster (Base Thruster Class)
**Location:** `src/models/propulsion/FGThruster.h` / `.cpp`
**Line Count:** ~137 header + 216 implementation = 353 total
**Inherits from:** FGForce

### Model Type & Physics
**Abstract base class** for thrust producers:
- Subclasses: FGPropeller, FGRotor, FGNozzle, FGDirect
- Manages force/moment transformation and reverser angle

**Reverser Angle Physics:**
- Final thrust = cos(reverser_angle) × unmodified_thrust
- Angle in radians: 0 = full forward, π/2 = no thrust, π = full reverse

### Public API
- Constructor: `FGThruster(FGFDMExec *exec, Element *el, int num)`
- `Calculate(double tt) → double` — Core: input power/thrust → output thrust
  - Default impl: `Thrust = cos(ReverserAngle) × tt`
- `SetName(std::string name)`
- `GetName() → std::string`
- `GetType() → eType` — Enum: ttNozzle, ttRotor, ttPropeller, ttDirect
- `GetThrust() → double` — Current thrust [lbs]
- `SetRPM(double rpm)` / `GetRPM() → double` — Virtual stubs
- `SetEngineRPM(double rpm)` / `GetEngineRPM() → double` — Virtual stubs
- `GetPowerRequired() → double` — Virtual; returns 0 by default
- `SetReverserAngle(double angle)` / `GetReverserAngle() → double`
- `GetGearRatio() → double` — Gear reduction ratio
- `ResetToIC()` — Reset to initial conditions
- `GetThrusterLabels/Values(int id, const std::string& delim) → std::string`

### Thruster Input Struct
```cpp
struct Inputs {
  double TotalDeltaT, H_agl;
  FGColumnVector3 PQRi, AeroPQR, AeroUVW;
  double Density, Pressure, Soundspeed;
  double Alpha, Beta, Vt;
};
```

### Protected Members
- `Type: eType` — Thruster subtype
- `Name: std::string`
- `Thrust: double` — Current output [lbs]
- `PowerRequired: double` — Power absorbed [hp]
- `GearRatio: double` — Engine-to-thruster speed ratio
- `ThrustCoeff: double` — Thrust coefficient (if applicable)
- `ReverserAngle: double` — Reverser positioning [rad]
- `EngineNum: int` — Associated engine index

---

## 8. FGForce (Force/Moment Transformation Utility)
**Location:** `src/models/propulsion/FGForce.h` / `.cpp`
**Line Count:** ~325 header + 203 implementation = 528 total
**Inherits from:** FGJSBBase (utility class, not model)

### Model Type & Physics
**Coordinate system transformation utility:**
- Converts forces from native (native, wind, local) to body frame
- Computes moments due to offset from CG
- Supports custom transform matrices for arbitrary angles

**Transform Types:**
- `tNone` — No angular transform
- `tWindBody` — Wind-to-body (uses JSBSim built-in matrix)
- `tLocalBody` — Local (ECEF)-to-body
- `tInertialBody` — Inertial-to-body
- `tCustom` — User-supplied pitch/roll/yaw angles

### Public API
- Constructor: `FGForce(FGFDMExec *exec)`
- `SetTransformType(TransformType tt)` / `GetTransformType() → TransformType`
- `SetLocation(double x, double y, double z)` — Force application point (structural coords)
- `SetActingLocation(double x, double y, double z)` — Offset location (e.g., P-factor)
- `GetLocation() → const FGColumnVector3&`, `GetActingLocation() → const FGColumnVector3&`
- `SetAnglesToBody(double roll, double pitch, double yaw)` — Custom transform angles
- `GetAnglesToBody() → const FGColumnVector3&`
- `SetPitch(double)`, `GetPitch() → double`
- `SetYaw(double)`, `GetYaw() → double`
- `UpdateCustomTransformMatrix()` — Recalculate custom matrix
- `GetBodyForces() → const FGColumnVector3&` — Compute and return body-frame forces
- `GetMoments() → const FGColumnVector3&` — Moments (computed with GetBodyForces)
- `Transform() → const FGMatrix33&` — Return current transform matrix

### Protected Members
- `fdmex: FGFDMExec*`
- `MassBalance: std::shared_ptr<FGMassBalance>` — CG location
- `vFn: FGColumnVector3` — Native force vector
- `vMn: FGColumnVector3` — Native moment vector
- `vOrient: FGColumnVector3` — Angles for custom transform (roll, pitch, yaw)
- `ttype: TransformType` — Current transform mode
- `vXYZn: FGColumnVector3` — Force location (structural coords)
- `vActingXYZn: FGColumnVector3` — Offset location
- `mT: FGMatrix33` — Transform matrix

---

## 9. FGTank (Fuel/Oxidizer Tank)
**Location:** `src/models/propulsion/FGTank.h` / `.cpp`
**Line Count:** ~379 header + 555 implementation = 934 total

### Model Type & Physics
**Fuel tank with optional temperature dynamics:**
- Tank contents tracking [lbs], percentage fill
- Fuel temperature calculation (if initial temp specified)
  - Assumes wing-tank geometry: h × 4h × 10h (volume = 40h³)
  - Surface area: 40h² (upper/lower surface)
  - Heat capacity: 900 J/(lbm⋅K) for jet fuel
  - Heat transfer: 1.115 W/(sq ft⋅K)
- Fuel drain location (separate from fill location)
- Standpipe (non-dumpable contents)
- Unusable fuel (can't be burned)
- Priority-based feed sequence

**Tank Types:**
- `ttFUEL` — Jet/avgas/ethanol fuel
- `ttOXIDIZER` — Oxidizer (for rockets)

**Grain Types (for rocket propellant):**
- `gtCYLINDRICAL` — Cylindrical grain
- `gtENDBURNING` — End-burn grain
- `gtFUNCTION` — Function-based custom grain

### Public API (Key Methods)
- Constructor: `FGTank(FGFDMExec* exec, Element* el, int tank_number)`
- `Drain(double used) → double` — Remove fuel [lbs]; returns remaining contents
- `Calculate(double dt, double TempC) → double` — Update temperature; returns temp [°C]
- `Fill(double amount) → double` — Add fuel [lbs]; returns new contents
- `SetContents(double amount)` — Set exact contents [lbs]
- `SetContentsGallons(double gallons)` — Set contents [gal]
- `SetTemperature(double temp)` — Set fuel temp [°C]
- `SetStandpipe(double amount)` — Set non-dumpable level [lbs]
- `SetSelected(bool sel)` — Enable/disable this tank for fuel feed
- `GetType() → int` — Returns tank type enum
- `GetName() → const std::string&`
- `GetSelected() → bool` — Is this tank selected for feed?
- `GetPctFull() → double` — Fill level [0–100 %]
- `GetCapacity() → double` — Max contents [lbs]
- `GetCapacityGallons() → double` — Max contents [gal]
- `GetContents() → double` — Current contents [lbs]
- `GetContentsGallons() → double` — Current contents [gal]
- `GetTemperature_degC() → double` — Fuel temp [°C]
- `GetTemperature() → double` — Fuel temp [°F]
- `GetUnusable() → double` — Non-burnable contents [lbs]
- `GetUnusableVolume() → double` — Non-burnable volume [gal]
- `SetUnusableVolume(double volume)` — Set non-burnable [gal]
- `GetStandpipe() → double` — Non-dumpable contents [lbs]
- `GetPriority() → int` / `SetPriority(int p)` — Feed sequence
- `GetDensity() → double` / `SetDensity(double d)` — Fuel density [lbs/gal]
- `GetExternalFlow() → double` / `SetExternalFlow(double f)` — External flow rate [lbs/sec]
- `GetXYZ() → FGColumnVector3` / `GetXYZ(int idx) → double` — Tank location
- `GetLocationX/Y/Z()` / `SetLocationX/Y/Z(double)` — Location accessors
- `GetIxx/Iyy/Izz() → double` — Moments of inertia [slug⋅ft²]
- `GetGrainType() → GrainType` — Propellant grain type (if applicable)
- `ResetToIC()` — Reset to initial conditions
- `ProcessFuelName(const std::string& name) → double` — Lookup fuel density by type name

### Fuel Types (Named Density Lookups)
AVGAS, JET-A, JET-A1, JET-B, JP-1, JP-2, JP-3, JP-4, JP-5, JP-6, JP-7, JP-8, JP-8+100, RP-1, T-1, ETHANOL, HYDRAZINE, F-34, F-35, F-40, F-44, AVTAG, AVCAT

### Private Members
- `Type: TankType` — Tank type (FUEL or OXIDIZER)
- `grainType: GrainType` — Propellant grain type
- `TankNumber: int` — Tank index (zero-based)
- `Name: std::string` — Tank identifier
- `vXYZ, vXYZ_drain: FGColumnVector3` — Tank location and drain location [in]
- `Capacity, Contents, InitialContents: double` — Contents [lbs]
- `UnusableVol: double` — Non-burnable volume [gal]
- `Radius, InnerRadius: double` — Tank geometry [in]
- `Length: double` — Tank length (for cylindrical grain) [in]
- `Volume: double` — Tank volume [cu in]
- `Density: double` — Fuel density [lbs/gal]
- `Ixx, Iyy, Izz: double` — Inertia tensor [slug⋅ft²]
- `InertiaFactor: double` — Slosh inertia scaling [0–1]
- `PctFull: double` — Fill percentage [0–100]
- `Area: double` — Effective surface area [sq in]
- `Temperature, InitialTemperature: double` — Fuel temp [°C]
- `Standpipe, InitialStandpipe: double` — Non-dumpable contents [lbs]
- `ExternalFlow: double` — Ext. input/output flow rate [lbs/sec]
- `Selected: bool` — Is this tank selected for feed?
- `Priority, InitialPriority: int` — Feed priority (1 = highest)

### Configuration Format
```xml
<tank type="{FUEL | OXIDIZER}" name="{string}">
  <location unit="{FT | M | IN}">
    <x> {number} </x>
    <y> {number} </y>
    <z> {number} </z>
  </location>
  <drain_location unit="{FT | M | IN}">
    <x> {number} </x>
    <y> {number} </y>
    <z> {number} </z>
  </drain_location>
  <radius unit="{IN | FT | M}"> {number} </radius>
  <capacity unit="{LBS | KG}"> {number} </capacity>
  <inertia_factor> {number:0-1} </inertia_factor>
  <contents unit="{LBS | KG}"> {number} </contents>
  <temperature> {number} </temperature>
  <standpipe unit="{LBS | KG}"> {number} </standpipe>
  <unusable unit="{GAL | LTR | M3 | IN3 | FT3 | CC}"> {number} </unusable>
  <priority> {integer} </priority>
  <density unit="{KG/L | LBS/GAL}"> {number} </density>
  <type> {string} </type>
  <grain_config type="{CYLINDRICAL | ENDBURNING | FUNCTION}">
    <length unit="{IN | FT | M}"> {number} </length>
    <bore_diameter unit="{IN | FT | M}"> {number} </bore_diameter>
    [<ixx>, <iyy>, <izz> with {function}]
  </grain_config>
</tank>
```

---

## 10. Example Configuration: F450 Quadrotor (DJI Phantom-Class)
**Files:**
- `aircraft/F450/Propulsion.xml` — 72 lines
- `engine/DJI_E305.xml` — 8 lines
- `engine/DJI_9450.xml` — 78 lines

### Propulsion Configuration Structure
**4 Electric Motor + Propeller Units:**
- 2 motors with CW sense (+1.0), 2 with CCW sense (−1.0)
- All pitched at 90° (thrust vertical in body frame)
- Arranged in X-quadrotor pattern (232 mm diagonal wheelbase)

```xml
<propulsion>
  <engine file="DJI_E305" name="front right">
    <thruster file="DJI_9450">
      <location unit="M"> <x>-0.1651</x> <y>0.1651</y> <z>0.025</z> </location>
      <orient unit="DEG"> <roll>0</roll> <pitch>90</pitch> <yaw>0</yaw> </orient>
      <sense>1.0</sense>
      <p_factor>0.0</p_factor>
    </thruster>
  </engine>
  <!-- ... 3 more engines ... -->
</propulsion>
```

### DJI E305 Brushless DC Motor (`DJI_E305.xml`)
```xml
<brushless_dc_motor name="DJI E305">
  <velocityconstant> 960 </velocityconstant>      <!-- [RPM/V] -->
  <coilresistance> 0.117 </coilresistance>        <!-- [Ohms] -->
  <noloadcurrent> 0.45 </noloadcurrent>           <!-- [Amperes] -->
  <maxvolts> 14.63 </maxvolts>                    <!-- [V] nominal battery -->
</brushless_dc_motor>
```

**Implied Physics:**
- Max RPM ≈ 960 × 14.63 ≈ 14,045 RPM
- Coil loss: I²Rm ≈ I² × 0.117 W
- No-load (friction/windage) current: 0.45 A baseline

### DJI 9450 Propeller (`DJI_9450.xml`)
- **Diameter:** 9.4 inches (≈ 9.4" × 4.5" pitch designation)
- **Blades:** 2
- **Inertia:** 6.05 × 10⁻⁵ kg⋅m² (≈ 0.000605 slug⋅ft²)
- **Mode:** Fixed-pitch (const_speed = 0)
- **Tables:** C_THRUST and C_POWER indexed by advance ratio J
  - J ranges 0.0–0.729 (low advance ratio; UAV props operate at high thrust coefficient, low speed ratio)
  - Ct ranges 0.129 → ~0.0 (maximum at J=0)
  - Cp ranges 0.067 → ~0.006 (maximum near J=0)
- **Version:** 1.1 (correct gyroscopic moment sign)

**Physical Interpretation:**
- At J=0 (hovering): Ct ≈ 0.129, Cp ≈ 0.067 → high thrust, moderate power
- As J increases (forward flight): both Ct and Cp decrease (propeller stalls/windmills)
- Typical hover point: J ~ 0.1, Ct ~ 0.125, Cp ~ 0.065

---

## 11. Data Flow & Integration Points

### Engine → Thruster → Force/Moment Pipeline
```
FGPropulsion::Run()
  ├─ for each Engine:
  │   ├─ Engine::Calculate()  (reads Inputs from FDMExec atmosphere/dynamics)
  │   │   └─ FGBrushLessDCMotor::Calculate()  (throttle → RPM, Current, Power)
  │   ├─ Engine::GetPowerAvailable()  (returns HP in ft⋅lbf/s)
  │   └─ Thruster = Engine::GetThruster()
  │       ├─ if FGPropeller:
  │       │   └─ FGPropeller::Calculate(PowerAvailable)
  │       │       ├─ Compute J (advance ratio)
  │       │       ├─ Lookup Ct, Cp from tables (interpolated by J)
  │       │       ├─ Apply Mach corrections (optional CT_MACH, CP_MACH)
  │       │       ├─ T = Ct × ρ × n² × D⁴
  │       │       ├─ Integrate torque balance (motor torque - prop drag)
  │       │       ├─ dω/dt = (MotorTorque - PropDragTorque) / Ixx
  │       │       └─ Return Thrust, Torque
  │       │
  │       └─ if FGRotor:
  │           └─ FGRotor::Calculate(PowerAvailable)
  │               ├─ Compute blade flapping (a0, a1s, b1s)
  │               ├─ Compute inflow ratio (λ, ν)
  │               ├─ T = C_T × ρ × Ω² × R²
  │               └─ Return Thrust, Torque, Moments
  │
  ├─ Thruster::GetBodyForces() & GetMoments()
  │   └─ FGForce::GetBodyForces() [transform via FGForce::Transform()]
  │   └─ Moments = vFn × (CG - ForceLocation) + NativeMoments
  │
  └─ vForces += BodyForces
      vMoments += Moments
```

### Fuel Management Loop
```
FGPropulsion::Run()
  └─ for each Engine:
      ├─ Engine::CalcFuelNeed()  (based on throttle/power level)
      └─ FGPropulsion::ConsumeFuel(Engine)
          └─ for each SourceTank[i]:
              └─ FGTank::Drain(FuelRequired)
                  ├─ Contents -= used
                  ├─ PctFull = Contents / Capacity × 100
                  └─ if Contents <= Standpipe: Engine::SetStarved()
```

### Property Tree Interface (Runtime Tuning)
```
propulsion/
  engine[i]/
    throttle-cmd              [0–1] pilot input
    mixture-cmd               [0–1] pilot input
    starter-cmd               [bool]
    magnetos-cmd              [int]
    cutoff-cmd                [bool]
    prop-pitch-cmd            [deg] variable-pitch propeller
    prop-feather-cmd          [bool] feather propeller
    running                   [bool] state output
    starved                   [bool] fuel starvation flag
    fuel-flow-rate-pph        [lbs/hr] output
    fuel-used                 [lbs] cumulative output

  tank[i]/
    contents-lbs              [lbs] settable
    contents-gal              [gal] settable
    capacity-lbs              [lbs] read-only
    percent-full              [%] read-only
    temperature-c             [°C] read-only
```

---

## 12. Key Constants & Conversion Factors

**Unit Conversions (Code Constants):**
- 1 N⋅m = 1.3558 ft⋅lbf → `NMtoftpound = 1.3558`
- 1 hp = 745.7 W → `hptowatts = 745.7`
- 1 W / (1 RPM × 1 N⋅m) = 60 / (2π × 1.3558) → `WattperRPMtoftpound`

**Physics Constants:**
- Standard atmosphere density (SL): ρ₀ = 0.002377 slugs/ft³
- Speed of sound (SL): a₀ = 661.5 knots ≈ 1116 ft/s
- Gravity: g = 32.174 ft/s²

**JSBSim Structural Coordinate System:**
- Origin: CG of aircraft
- X-axis: +forward (along fuselage)
- Y-axis: +right wing
- Z-axis: +down
- Units: inches (structural), feet or meters (locations)

---

## 13. Summary Table: File Roles & Complexity

| File | Lines | Role | Complexity |
|------|-------|------|------------|
| **FGPropulsion** | 1,286 | System manager; aggregates engines & tanks | High (orchestration) |
| **FGBrushLessDCMotor** | 340 | BLDC motor model (3-constant equations) | Low (algebraic) |
| **FGElectric** | 285 | Basic linear electric motor (legacy) | Low (trivial) |
| **FGPropeller** | 822 | Fixed/variable-pitch propeller aerodynamics | **Very High** (thrust/power tables, torque integration, Mach effects, gyro) |
| **FGRotor** | 1,338 | Helicopter rotor (flapping dynamics, inflow, transmission) | **Very High** (blade element, momentum theory, 6-DOF flapping) |
| **FGEngine** | 524 | Abstract base engine class | Medium (property binding, state mgmt) |
| **FGThruster** | 353 | Abstract base thruster class | Low (force/moment wrapper) |
| **FGForce** | 528 | Coordinate transform utility | High (matrix math, moment arm) |
| **FGTank** | 934 | Fuel tank model (contents, temperature, priority) | Medium (thermal dynamics optional) |
| **F450 Propulsion.xml** | 72 | Example quad config (4× motor+prop) | — |
| **DJI_E305.xml** | 8 | Brushless motor spec (example) | — |
| **DJI_9450.xml** | 78 | Propeller tables (example) | — |

---

## Key Takeaways for Research

1. **Propeller aerodynamics dominate complexity:** FGPropeller handles advance ratio, coefficient tables, Mach effects, and torque integration. For DroneSim, this is a critical component.

2. **BLDC motor model is simple:** Only 3 parameters (Kv, Rm, I₀) govern behavior. No internal dynamics; equations are algebraic per simulation step.

3. **Rotor model is elaborate:** FGRotor models flapping, inflow lag, cyclic control, transmission, and ground effect. Significantly more complex than propellers.

4. **Fuel/tank system is sophisticated:** Despite not being used for electric UAVs, JSBSim's tank model includes temperature dynamics, priority-based feeding, external flows, and grain-based mass moment calculations.

5. **Force/moment transformation is orthogonal:** FGForce is a reusable utility handling coordinate conversions and moment-arm calculations. Both propellers and rotors inherit from FGThruster which uses FGForce.

6. **Electric-specific simplifications:**
   - No fuel flow in FGBrushLessDCMotor or FGElectric (CalcFuelNeed() returns 0)
   - No battery discharge model; infinite power assumed
   - BLDC model more physically accurate than legacy FGElectric

7. **XML-driven configuration:** All models are configured via external XML (propulsion definitions, engine specs, propeller tables). This enables data-driven tuning without recompilation.

