# JSBSim Propulsion Domain — File Inventory Index

**Compiled:** 2026-03-11
**Status:** COMPLETE
**Entry Point:** You are here.

---

## What Is This?

A comprehensive file inventory and technical reference for the **JSBSim propulsion system**, focusing on **electric brushless motors + propellers** (quadrotor/multirotor relevant).

This inventory was created to support DroneSim research and potential JSBSim architecture adaptation.

---

## Documents Provided

### 1. **PROPULSION-INVENTORY.md** (885 lines)
**Detailed file-by-file breakdown with API reference.**

Contains:
- **11 source files** (headers + implementations) with:
  - Line counts and key classes
  - Complete public API (methods, accessors, mutators)
  - Configuration file format (XML schema)
  - Physics constants and key equations
  - Key algorithmic patterns (thrust calculation, torque integration, etc.)

- **3 example configuration files** (DJI motors & propellers)
  - Full F450 quadrotor setup
  - Motor specs (Kv, Rm, I₀, MaxVolts)
  - Propeller tables (advance ratio → Ct/Cp coefficients)

- **Data flow & integration points** (engine → thruster → force/moment pipeline)

- **Summary table** of file roles and complexity

**Use This When:**
- You need exact API signatures
- You're designing an integration with DroneSim
- You want to understand the public interface
- You need config file format reference

---

### 2. **PROPULSION-IMPLEMENTATION-NOTES.md** (625 lines)
**Technical deep-dive into algorithms and physics models.**

Contains:
- **Motor Model (§1):** 3-constant BLDC equations, limitations, suitable use cases
- **Propeller Model (§2):** Advance ratio, Ct/Cp tables, P-factor, gyroscopic effects, constant-speed governor
- **Rotor Model (§3):** Momentum theory, blade flapping, inflow lag, ground effect
- **Force/Moment Transformation (§4):** Coordinate system transforms, moment-arm calculation
- **Tank Model (§5):** Priority-based feed, standpipe, unusable volume, thermal dynamics
- **Property Tree Interface (§6):** Runtime observability, tunable parameters
- **DroneSim Integration Recommendations (§7):** Where to adapt vs. reuse
- **Config File Patterns (§8):** How to write/generate XML definitions
- **Debugging & Validation Checklist (§9):** Sanity checks for each component
- **Performance & Scalability (§10):** Computational cost, numerical stability, multi-rotor vs. helicopter

**Use This When:**
- You're implementing an algorithm
- You need to understand the physics
- You're debugging a simulation
- You're adapting code to DroneSim
- You want to generate propeller tables from APC datasheets

---

### 3. **PROPULSION-FILES-DIRECTORY.md** (342 lines)
**Navigation guide and file organization reference.**

Contains:
- **Source tree layout** (where files are located)
- **Dependency graph** (which files depend on which)
- **Header dependency chain** (critical paths for #include)
- **File size summary** (lines of code per file)
- **Configuration file examples** (quick reference)
- **File relationships** (FGPropulsion ↔ FGEngine ↔ FGThruster, etc.)
- **XML parsing flow** (how configs are loaded)
- **Test & validation files** (where tests live)
- **Navigation guide** (which documents to read for each topic)

**Use This When:**
- You need to find a specific file
- You're tracing a dependency
- You want to understand the module structure
- You need a quick reference for file locations

---

## Quick Start: Which Document Should I Read?

### "I want to port BLDC motor code to DroneSim"
→ Read **PROPULSION-INVENTORY.md § 2** (FGBrushLessDCMotor) + **IMPLEMENTATION-NOTES.md § 1**

### "I need to understand propeller aerodynamics"
→ Read **PROPULSION-INVENTORY.md § 4** (FGPropeller) + **IMPLEMENTATION-NOTES.md § 2**

### "I'm integrating JSBSim propulsion with DroneSim"
→ Read **IMPLEMENTATION-NOTES.md § 7** (Integration Points) + **PROPULSION-INVENTORY.md § 11** (Data Flow)

### "I need to generate propeller tables from APC datasheets"
→ Read **IMPLEMENTATION-NOTES.md § 8** (Config Patterns) + **PROPULSION-INVENTORY.md § 10** (Example: DJI_9450.xml)

### "I'm debugging a propeller model"
→ Read **IMPLEMENTATION-NOTES.md § 9** (Validation Checklist) + **PROPULSION-INVENTORY.md § 4** (Physics)

### "I need to find where FGPropeller is located"
→ Read **PROPULSION-FILES-DIRECTORY.md § 1** (Source Tree)

### "I need to understand the full propulsion system architecture"
→ Start with **PROPULSION-INVENTORY.md § 1** (FGPropulsion), then § 11 (Data Flow)

---

## Key Findings: Summary

### 1. Motor Model (FGBrushLessDCMotor)
- **Simplicity:** 3 parameters (Kv, Rm, I₀) fully specify the motor
- **Physics:** Algebraic only; no transients, thermal, or saturation
- **Code:** ~340 lines (very compact)
- **Suitable for:** Real-time UAV simulation, quick prototyping
- **Config:** 8-line XML with 4 parameters

### 2. Propeller Model (FGPropeller)
- **Complexity:** Table-driven Ct(J) and Cp(J) with optional Mach corrections
- **Physics:** Momentum theory + blade element; includes P-factor and gyroscopic effects
- **Code:** ~822 lines (moderate complexity)
- **Tables:** ~30–50 advance ratio (J) points → interpolated at runtime
- **Example:** DJI 9450: 9.4" diameter, 2-blade, ~0.0001 kg⋅m² inertia
- **Suitable for:** Multi-rotor, fixed-wing, any propeller-driven aircraft

### 3. Rotor Model (FGRotor)
- **Complexity:** Full 6-DOF flapping, cyclic/collective control, transmission
- **Physics:** Momentum theory + blade element with flapping dynamics
- **Code:** ~1,338 lines (high complexity, most complex in propulsion system)
- **Suitable for:** Helicopters with dynamic control, not quadrotors
- **Note:** Overkill for rigid-hub quadrotors; simpler models sufficient

### 4. System Architecture
- **Separation of Concerns:** Engine (power) → Thruster (force) → Force (transform)
- **Extensibility:** Inheritance-based; easy to add new engine/thruster types
- **Configuration:** All tuning via XML; no hardcoded parameters
- **Property Tree:** Runtime observability and dynamic reconfiguration

### 5. Performance & Scalability
- **CPU Cost:** ~20–30 microseconds for F450 quad @ 1000 Hz (2–3% per frame)
- **Numerical Stability:** Use semi-implicit Euler for RPM integration
- **Scaling:** Linear in number of motors; rotor complexity independent of scale

---

## File Statistics

| Document | Lines | Size | Density |
|----------|-------|------|---------|
| PROPULSION-INVENTORY.md | 885 | 39 KB | High detail; reference manual |
| PROPULSION-IMPLEMENTATION-NOTES.md | 625 | 22 KB | Medium density; algorithm guide |
| PROPULSION-FILES-DIRECTORY.md | 342 | 14 KB | Low density; navigation |
| **Total** | **1,852** | **75 KB** | **Complete reference set** |

---

## How to Use These Documents

### Scenario 1: You're new to JSBSim propulsion
**Read in order:**
1. PROPULSION-FILES-DIRECTORY.md § 7 (Dependency Graph)
2. PROPULSION-INVENTORY.md § 1 (FGPropulsion overview)
3. PROPULSION-INVENTORY.md § 11 (Data Flow)
4. IMPLEMENTATION-NOTES.md § 10 (Philosophy)

### Scenario 2: You're porting code to DroneSim
**Read in order:**
1. PROPULSION-INVENTORY.md § 2 (FGBrushLessDCMotor) or § 4 (FGPropeller)
2. IMPLEMENTATION-NOTES.md § 1–4 (Physics of that component)
3. IMPLEMENTATION-NOTES.md § 7 (Integration Points for DroneSim)
4. IMPLEMENTATION-NOTES.md § 8 (Config Patterns)

### Scenario 3: You're debugging a simulation
**Read in order:**
1. IMPLEMENTATION-NOTES.md § 9 (Validation Checklist)
2. PROPULSION-INVENTORY.md (relevant section; e.g., § 2, § 4, § 5)
3. IMPLEMENTATION-NOTES.md (relevant section with equations)

### Scenario 4: You're generating new propeller tables
**Read in order:**
1. PROPULSION-INVENTORY.md § 10 (Example: DJI_9450.xml)
2. IMPLEMENTATION-NOTES.md § 8 (Config Patterns)
3. IMPLEMENTATION-NOTES.md § 2 (Advance Ratio Interpretation)

---

## Related Files in JSBSim Repository

**You should also examine (not included in this inventory):**
- `src/models/propulsion/FGTransmission.h/.cpp` — Gear/brake/clutch (used by FGRotor)
- `src/models/FGAircraft.h` — Main aircraft integrator (loads propulsion system)
- `src/models/FGAtmosphere.h` — Atmosphere interface (provides ρ, a, etc. to engines)
- `src/models/FGMassBalance.h` — CG location (used by FGForce for moments)
- `aircraft/*/Propulsion.xml` — Example configs for various aircraft types

---

## Next Steps (For Research)

### If pursuing DroneSim integration:
1. **Phase 1:** Compare DroneSim's motor/propeller models with JSBSim's (this inventory)
2. **Phase 2:** Identify gaps and porting opportunities (see IMPLEMENTATION-NOTES.md § 7)
3. **Phase 3:** Prototype adapters (XSD config format, motor model wrapper, etc.)
4. **Phase 4:** Validate on test cases (DJI F450 quad, etc.)

### If pursuing JSBSim deep-dive:
1. **Phase 1:** Understand FGPropulsion & FGEngine architecture (this inventory)
2. **Phase 2:** Study FGPropeller in detail (tables, P-factor, gyroscopic effects)
3. **Phase 3:** Examine FGRotor for helicopter dynamics
4. **Phase 4:** Review test suite (`test/FGPropeller*`, etc.) for validation patterns

### If pursuing propeller characterization:
1. **Phase 1:** Understand Ct/Cp table format (PROPULSION-INVENTORY.md § 10)
2. **Phase 2:** Locate APC datasheets (DJI 9450 example: 9.4" × 4.5")
3. **Phase 3:** Convert thrust/power vs. RPM tables to advance ratio tables
4. **Phase 4:** Generate JSBSim XML with interpolated Ct/Cp values

---

## Document Maintenance

**Last Updated:** 2026-03-11
**Scope:** JSBSim propulsion system (all thruster types, focus on BLDC + propeller)
**Completeness:** Covers 11 primary source files + 3 example configs
**Known Gaps:**
- FGTransmission not analyzed (referenced by FGRotor, but secondary)
- Piston/turbine engines not analyzed (focus on electric)
- Test suite not inventoried (see PROPULSION-FILES-DIRECTORY.md § 9)

---

## Contact & Questions

For questions about this inventory:
- Consult the relevant document section (see index above)
- Cross-reference with JSBSim source code (files listed in PROPULSION-FILES-DIRECTORY.md § 1)
- Consult JSBSim documentation: http://www.jsbsim.org/

---

## License & Attribution

**JSBSim** is licensed under the GNU Lesser General Public License (LGPL).

This inventory is a **reference compilation** of publicly available JSBSim source code and is provided for research and educational purposes under the same LGPL terms.

---

**End of Index.**

Start reading: [PROPULSION-INVENTORY.md](./PROPULSION-INVENTORY.md) or [Quick Navigation Guide](./PROPULSION-FILES-DIRECTORY.md)

