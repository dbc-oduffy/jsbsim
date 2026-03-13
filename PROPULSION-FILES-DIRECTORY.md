# JSBSim Propulsion Domain — File Directory & Dependencies

**Purpose:** Quick reference for file organization and header/implementation pair locations.

---

## 1. Source Tree Organization

```
src/models/
├── FGPropulsion.h           [System Manager — Container for engines & tanks]
├── FGPropulsion.cpp         [Implementation: ~906 lines]
│
└── propulsion/              [Propulsion subsystem components]
    ├── FGEngine.h           [Abstract base class for all engines]
    ├── FGEngine.cpp         [Implementation: ~294 lines]
    │
    ├── FGBrushLessDCMotor.h [BLDC motor model (3-constant equations)]
    ├── FGBrushLessDCMotor.cpp [Implementation: ~233 lines]
    │
    ├── FGElectric.h         [Basic electric motor (legacy, linear throttle model)]
    ├── FGElectric.cpp       [Implementation: ~187 lines]
    │
    ├── FGThruster.h         [Abstract base class for all thrusters]
    ├── FGThruster.cpp       [Implementation: ~216 lines]
    │
    ├── FGPropeller.h        [Fixed/variable-pitch propeller (table-driven)]
    ├── FGPropeller.cpp      [Implementation: ~475 lines]
    │
    ├── FGRotor.h            [Helicopter rotor (blade element + flapping)]
    ├── FGRotor.cpp          [Implementation: ~905 lines]
    │
    ├── FGForce.h            [Force/moment transformation utility (non-model)]
    ├── FGForce.cpp          [Implementation: ~203 lines]
    │
    ├── FGTank.h             [Fuel/oxidizer tank model]
    ├── FGTank.cpp           [Implementation: ~555 lines]
    │
    ├── FGTransmission.h     [Transmission (gear, brake, clutch) — rotor support]
    ├── FGTransmission.cpp   [Not examined in this inventory]
    │
    └── ... (other thruster types: FGNozzle, FGDirect, etc.)
```

---

## 2. Configuration/Data Files

```
engine/                     [Engine & motor definitions]
├── DJI_E305.xml            [Brushless DC motor (960 RPM/V, 0.117 Ω, 14.63 V)]
├── DJI_9450.xml            [Propeller definition (9.4" diam, 2 blades, Ct/Cp tables)]
│
└── ... (other engines: piston, turbine, turboprop, rocket, etc.)

aircraft/F450/              [Example: DJI Phantom-class quadrotor]
├── Propulsion.xml          [4× electric motor + propeller configuration]
├── F450.xml                [Main aircraft definition]
│
└── ... (other components: flight dynamics, aerodynamics, landing gear, etc.)
```

---

## 3. Dependency Graph (Simplified)

```
FGPropulsion (system)
  ├→ FGEngine (abstract)
  │    ├→ FGBrushLessDCMotor (concrete)
  │    ├→ FGElectric (concrete)
  │    ├→ FGPiston (concrete, not examined)
  │    ├→ FGTurbine (concrete, not examined)
  │    ├→ FGTurboprop (concrete, not examined)
  │    └→ FGRocket (concrete, not examined)
  │
  ├→ FGThruster (abstract, inherits from FGForce)
  │    ├→ FGPropeller (concrete)
  │    ├→ FGRotor (concrete, contains FGTransmission)
  │    ├→ FGNozzle (concrete, not examined)
  │    └→ FGDirect (concrete, not examined)
  │
  ├→ FGForce (utility, not a model)
  │    └→ [coordinate transform, moment-arm calculation]
  │
  └→ FGTank (storage, independent)
       └→ [fuel accounting, temperature, priority feed]

Each FGThruster is attached to exactly one FGEngine.
Each FGEngine lists 0+ FGTanks as fuel sources.
Each FGThruster uses FGForce for transformation (single instance per thruster).
```

---

## 4. Header Dependency Chain (Critical Paths)

```
FGBrushLessDCMotor.h
  ├→ FGEngine.h
  │    ├→ FGModelFunctions.h
  │    ├→ FGColumnVector3.h [math]
  │    └→ FGFDMExec.h [forward decl]
  └→ Element.h [XML parsing]

FGPropeller.h
  ├→ FGThruster.h
  │    ├→ FGForce.h
  │    │    ├→ FGJSBBase.h
  │    │    ├→ FGMatrix33.h [math]
  │    │    ├→ FGMassBalance.h [CG accessor]
  │    │    └→ FGColumnVector3.h [math]
  │    └→ FGColumnVector3.h [math]
  └→ FGTable.h [interpolation]

FGRotor.h
  ├→ FGThruster.h [same as above]
  ├→ FGTransmission.h [gear/brake/clutch]
  └→ FGColumnVector3.h, FGMatrix33.h [math]

FGTank.h
  ├→ FGJSBBase.h
  ├→ FGColumnVector3.h [math]
  └→ FGFunction.h [optional, for grain dynamics]
```

---

## 5. File Size Summary

| File | Type | Lines | Size (est.) | Role |
|------|------|-------|-------------|------|
| FGPropulsion.h | header | 212 | 7 KB | System manager interface |
| FGPropulsion.cpp | impl | 906 | 32 KB | System manager logic |
| FGEngine.h | header | 230 | 8 KB | Engine base class interface |
| FGEngine.cpp | impl | 294 | 11 KB | Engine base class logic |
| FGBrushLessDCMotor.h | header | 107 | 4 KB | BLDC interface |
| FGBrushLessDCMotor.cpp | impl | 233 | 8 KB | BLDC algebra |
| FGElectric.h | header | 98 | 4 KB | Basic electric interface |
| FGElectric.cpp | impl | 187 | 7 KB | Basic electric algebra |
| FGThruster.h | header | 137 | 5 KB | Thruster base interface |
| FGThruster.cpp | impl | 216 | 9 KB | Thruster base logic |
| FGPropeller.h | header | 347 | 12 KB | Propeller interface |
| FGPropeller.cpp | impl | 475 | 18 KB | Propeller integration & tables |
| FGRotor.h | header | 433 | 15 KB | Rotor interface |
| FGRotor.cpp | impl | 905 | 33 KB | Rotor flapping & dynamics |
| FGForce.h | header | 325 | 11 KB | Force utility interface |
| FGForce.cpp | impl | 203 | 8 KB | Force transform logic |
| FGTank.h | header | 379 | 13 KB | Tank interface |
| FGTank.cpp | impl | 555 | 20 KB | Tank accounting & thermal |
| **TOTALS** | **18 files** | **~5900** | **~220 KB** | **Full propulsion system** |

---

## 6. Configuration File Examples

### Minimal Motor Definition (DJI E305)
```xml
Location: engine/DJI_E305.xml
Size: 8 lines, <1 KB

<brushless_dc_motor name="DJI E305">
  <velocityconstant> 960 </velocityconstant>
  <coilresistance> 0.117 </coilresistance>
  <noloadcurrent> 0.45 </noloadcurrent>
  <maxvolts> 14.63 </maxvolts>
</brushless_dc_motor>
```

### Minimal Propeller Definition (DJI 9450)
```xml
Location: engine/DJI_9450.xml
Size: 78 lines, ~4 KB

<propeller name="DJI 9450" version="1.1">
  <ixx unit="KG*M2"> 6.05e-05 </ixx>
  <diameter unit="IN"> 9.4 </diameter>
  <numblades> 2 </numblades>
  <constspeed> 0 </constspeed>

  <table name="C_THRUST" type="internal">
    <tableData>
      {30 rows: advance_ratio Ct_coefficient}
    </tableData>
  </table>

  <table name="C_POWER" type="internal">
    <tableData>
      {30 rows: advance_ratio Cp_coefficient}
    </tableData>
  </table>
</propeller>
```

### Aircraft Propulsion Configuration (F450)
```xml
Location: aircraft/F450/Propulsion.xml
Size: 72 lines, ~3 KB

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

---

## 7. Key File Relationships

### FGPropulsion ↔ FGEngine
- FGPropulsion contains vector<FGEngine>
- Each FGEngine has one attached FGThruster
- FGPropulsion calls Engine::Calculate() on each
- FGPropulsion calls Engine::CalcFuelNeed() for tank consumption

### FGEngine ↔ FGThruster
- FGEngine creates FGThruster via LoadThruster()
- FGEngine::GetPowerAvailable() feeds into Thruster::Calculate()
- Thruster::GetThrust() feeds back to FGPropulsion

### FGThruster ↔ FGForce
- FGThruster inherits from FGForce
- Force location set from XML <location>
- Transform type set from XML <orient>
- FGForce::GetBodyForces() computes moment arm effects

### FGPropulsion ↔ FGTank
- FGEngine lists source tank indices
- FGPropulsion::ConsumeFuel() drains tanks
- FGPropulsion tracks total fuel quantity

---

## 8. XML Parsing Flow

```
FGPropulsion::Load(Element* propulsion_el)
  ├─ for each <engine> child:
  │   └─ FGEngine::Load(engine_el)
  │       ├─ Parse <feed> tank indices
  │       ├─ Construct engine (FGBrushLessDCMotor, FGElectric, etc.)
  │       ├─ Engine::LoadThruster(thruster_el)
  │       │   ├─ Read <location>, <orient>, <sense>, <p_factor>
  │       │   ├─ Load thruster from file (FGPropeller, FGRotor, etc.)
  │       │   └─ Construct thruster with loaded config
  │       └─ Store engine in vector
  │
  └─ for each <tank> child:
      └─ FGTank(tank_el)
          ├─ Parse <location>, <capacity>, <contents>, etc.
          ├─ Set fuel type (density lookup)
          └─ Store tank in vector
```

---

## 9. Test & Validation Files (Not Included in Inventory)

```
test/
├── FGPropulsionTest.cpp     [Propulsion system integration tests]
├── FGEngineTest.cpp         [Engine base class tests]
├── FGPropellerTest.cpp      [Propeller aerodynamics tests]
├── FGRotorTest.cpp          [Rotor dynamics tests]
├── FGTankTest.cpp           [Fuel management tests]
└── ... (other subsystem tests)
```

---

## 10. Reference Documentation (Within Codebase)

```
doc/
├── propulsion.html          [Doxygen HTML: propulsion system overview]
├── html/
│   ├── classFGPropulsion.html
│   ├── classFGEngine.html
│   ├── classFGBrushLessDCMotor.html
│   ├── classFGPropeller.html
│   ├── classFGRotor.html
│   ├── classFGTank.html
│   └── ... (auto-generated from Doxygen comments in headers)
└── README.propulsion        [Propulsion subsystem README, if exists]
```

---

## 11. Property Tree Bindings (Runtime Interface)

All propulsion properties are bound via FGPropertyManager in:
```
FGPropulsion::bind()          [Main propulsion properties]
FGEngine::bind()              [Per-engine properties]
FGThruster::bind()            [Per-thruster properties]
FGTank::bind()                [Per-tank properties]
```

Properties are registered with full path hierarchy:
```
propulsion/
  ├─ engine[i]/{throttle-cmd, running, fuel-flow-rate-pph, ...}
  ├─ tank[i]/{contents-lbs, percent-full, temperature-c, ...}
  └─ ... (aggregate stats: num-engines, total-fuel-quantity, ...)
```

---

## Summary: Navigation Guide

**To understand BLDC motors:**
1. Read `FGBrushLessDCMotor.h` (104 lines — very short)
2. Review `DJI_E305.xml` (8 lines — example config)
3. See PROPULSION-IMPLEMENTATION-NOTES.md § 1 (algorithm walkthrough)

**To understand propellers:**
1. Read `FGPropeller.h` (347 lines — detailed)
2. Review `DJI_9450.xml` (78 lines — example with full tables)
3. See PROPULSION-INVENTORY.md § 4 (API & physics)
4. See PROPULSION-IMPLEMENTATION-NOTES.md § 2 (algorithm walkthrough)

**To understand rotors (helicopters):**
1. Read `FGRotor.h` (433 lines — detailed)
2. See PROPULSION-INVENTORY.md § 5 (API & physics)
3. See PROPULSION-IMPLEMENTATION-NOTES.md § 3 (algorithm walkthrough)

**To understand system integration:**
1. Read `FGPropulsion.h` (212 lines — high-level)
2. Review `aircraft/F450/Propulsion.xml` (72 lines — example)
3. See PROPULSION-INVENTORY.md § 11 (data flow)

**To port/adapt for DroneSim:**
1. See PROPULSION-IMPLEMENTATION-NOTES.md § 7 (integration points & recommendations)
2. Review code patterns in PROPULSION-IMPLEMENTATION-NOTES.md § 8–9 (config/validation)

