# JSBSim Integration & Propagation Domain — File Inventory

**Purpose:** Complete mapping of JSBSim's core equations of motion (EOM), state propagation, and numerical integration layer for deep research phase.

**Mapping Date:** 2025-03-11
**Repository:** E:\dev\jsbsim (JSBSim main branch)

---

## 1. FGPropagate.h/.cpp — State Propagation Model

**File Paths:**
- Header: `E:\dev\jsbsim\src\models\FGPropagate.h` (689 lines)
- Implementation: `E:\dev\jsbsim\src\models\FGPropagate.cpp`

### Purpose
Models the equations of motion (EOM) and integration/propagation of the vehicle state given forces and moments. Manages state variables, selectable integrators, and coordinate frame transformations between ECI, ECEF, local (NED), and body frames.

### Key Classes/Structs

**VehicleState struct** (lines 100–139)
- `FGLocation vLocation` — Vehicle position in ECEF frame (ft)
- `FGColumnVector3 vUVW` — Body-frame velocity relative to ECEF (ft/sec)
- `FGColumnVector3 vPQR` — Body angular rates relative to ECEF (rad/sec)
- `FGColumnVector3 vPQRi` — Body angular rates relative to ECI (rad/sec)
- `FGQuaternion qAttitudeLocal` — Body→Local (NED) rotation quaternion
- `FGQuaternion qAttitudeECI` — Body→ECI rotation quaternion
- `FGQuaternion vQtrndot` — Quaternion time derivative
- `FGColumnVector3 vInertialVelocity` — Velocity in ECI frame
- `FGColumnVector3 vInertialPosition` — Position in ECI frame
- Deques for derivative history: `dqPQRidot`, `dqUVWidot`, `dqInertialVelocity`, `dqQtrndot`

**Inputs struct** (lines 610–618)
- `FGColumnVector3 vPQRidot` — Angular acceleration (ECI frame)
- `FGColumnVector3 vUVWidot` — Translational acceleration (ECI frame)
- `FGColumnVector3 vOmegaPlanet` — Earth rotation rate (rad/sec)
- `double SemiMajor`, `SemiMinor` — Earth ellipsoid parameters
- `double GM` — Gravitational parameter
- `double DeltaT` — Time step

### Key Methods

#### Initialization & State Management
- `bool InitModel(void)` (line 161) — Initializes Propagate module, called by FGFDMExec
- `void InitializeDerivatives()` (line 163) — Resets derivative deques for multi-step integrators
- `void SetVState(const VehicleState& vstate)` (line 525) — Replace entire state vector
- `bool Run(bool Holding)` (line 172) — Main propagation loop

#### State Access — Position
- `double GetAltitudeASL(void)` (line 330) — Altitude above sea level (ft)
- `double GetLatitude(void)` (line 450), `GetLongitude(void)` (line 449)
- `const FGLocation& GetLocation(void)` (line 460) — Full ECEF position
- `const FGColumnVector3& GetInertialPosition(void)` (line 313) — ECI position

#### State Access — Velocity
- `const FGColumnVector3& GetUVW(void)` (line 197) — Body-frame velocity (ft/sec)
- `const FGColumnVector3& GetVel(void)` (line 185) — Local (NED) velocity (ft/sec)
- `const FGColumnVector3& GetInertialVelocity(void)` (line 308) — ECI velocity
- `double GetInertialVelocityMagnitude(void)` (line 300)

#### State Access — Attitude & Angular Rates
- `const FGColumnVector3& GetEuler(void)` (line 253) — Euler angles (rad): [phi, theta, psi]
- `const FGQuaternion& GetQuaternion(void)` (line 539) — Attitude quaternion (Local→Body)
- `const FGColumnVector3& GetPQR(void)` (line 211) — Angular rates rel. ECEF (rad/sec)
- `const FGColumnVector3& GetPQRi(void)` (line 225) — Angular rates rel. ECI (rad/sec)
- `const FGQuaternion& GetQuaterniondot(void)` (line 236) — Quaternion time derivative

#### Coordinate Frame Transformations
- `const FGMatrix33& GetTl2b(void)` (line 467) — Local→Body transformation matrix
- `const FGMatrix33& GetTb2l(void)` (line 473) — Body→Local
- `const FGMatrix33& GetTec2b(void)` (line 477) — ECEF→Body
- `const FGMatrix33& GetTb2ec(void)` (line 481) — Body→ECEF
- `const FGMatrix33& GetTi2b(void)` (line 485) — ECI→Body
- `const FGMatrix33& GetTb2i(void)` (line 489) — Body→ECI
- `const FGMatrix33& GetTec2l(void)` (line 505) — ECEF→Local
- `const FGMatrix33& GetTl2ec(void)` (line 511) — Local→ECEF
- `const FGMatrix33& GetTec2i(void)` (line 494) — ECEF→ECI
- `const FGMatrix33& GetTi2ec(void)` (line 499) — ECI→ECEF
- `const FGMatrix33& GetTl2i(void)` (line 516) — Local→ECI
- `const FGMatrix33& GetTi2l(void)` (line 521) — ECI→Local

#### State Setters
- `void SetUVW(unsigned int i, double val)` (line 552) — Set body velocity component, recalc inertial
- `void SetPQR(unsigned int i, double val)` (line 547) — Set angular rate, recalc inertial
- `void SetLatitude(double lat)`, `SetLongitude(double lon)` (lines 565, 559)
- `void SetAltitudeASL(double altASL)` (line 577)
- `void SetEarthPositionAngle(double EPA)` (line 532) — Earth rotation angle (rad)

#### Earth & Orbit Parameters
- `double GetEarthPositionAngle(void)` (line 431) — EPA in radians
- `const VehicleState& GetVState(void)` (line 523) — Raw state struct access
- `void ComputeOrbitalParameters(void)` (line 681) — Orbital elements (h, eccentricity, true anomaly, etc.)

#### Integrator Configuration
- Enum `eIntegrateType` (line 155):
  - `eNone` (0) — Freeze state
  - `eRectEuler` (1) — Rectangular Euler
  - `eTrapezoidal` (2)
  - `eAdamsBashforth2` (3), `eAdamsBashforth3` (4), `eAdamsBashforth4` (5)
  - `eBuss1`, `eBuss2`, `eLocalLinearization`, `eAdamsBashforth5`
- Properties to select integrators:
  - `simulation/integrator/rate/rotational`
  - `simulation/integrator/rate/translational`
  - `simulation/integrator/position/rotational`
  - `simulation/integrator/position/translational`

#### Internal Integration Overloads
- `void Integrate(FGColumnVector3& Integrand, FGColumnVector3& Val, std::deque<FGColumnVector3>& ValDot, double dt, eIntegrateType type)` (lines 666–670)
- `void Integrate(FGQuaternion& Integrand, FGQuaternion& Val, std::deque<FGQuaternion>& ValDot, double dt, eIntegrateType type)` (lines 672–676)

### Private Members

**Transformation Matrices** (lines 628–639)
- `FGMatrix33 Tec2b, Tb2ec, Tl2b, Tb2l, Tl2ec, Tec2l, Tec2i, Ti2ec, Ti2b, Tb2i, Ti2l, Tl2i`

**Orbital Elements** (lines 642–651)
- `double h` — Specific angular momentum
- `double Inclination, RightAscension, Eccentricity, PerigeeArgument, TrueAnomaly`
- `double ApoapsisRadius, PeriapsisRadius, OrbitalPeriod`

**Integrator State** (lines 657–660)
- `eIntegrateType integrator_rotational_rate, integrator_translational_rate`
- `eIntegrateType integrator_rotational_position, integrator_translational_position`

**Private Methods**
- `void CalculateInertialVelocity(void)` (line 662) — Convert body→ECI velocity
- `void CalculateUVW(void)` (line 663) — Convert ECI→body velocity
- `void CalculateQuatdot(void)` (line 664) — Compute quaternion time derivative
- `void UpdateLocationMatrices(void)` (line 678) — Recompute ECEF↔Local transforms
- `void UpdateBodyMatrices(void)` (line 679) — Recompute quaternion-based transforms

---

## 2. FGAccelerations.h/.cpp — Acceleration Computation

**File Paths:**
- Header: `E:\dev\jsbsim\src\models\FGAccelerations.h` (386 lines)
- Implementation: `E:\dev\jsbsim\src\models\FGAccelerations.cpp`

### Purpose
Calculates linear and angular accelerations from all applied forces and moments. Handles gravity (standard and WGS84), gravitational torque, ground friction, and accounts for rotating reference frames.

### Key Classes

**Inputs struct** (lines 320–363)
- `FGMatrix33 J` — Inertia matrix (body frame)
- `FGMatrix33 Jinv` — Inverse inertia matrix
- `FGMatrix33 Ti2b, Tb2i` — ECI↔Body transformations
- `FGMatrix33 Tec2b, Tec2i` — ECEF↔Body, ECEF↔ECI transformations
- `FGColumnVector3 Moment` — Total moments (excluding friction/gravity)
- `FGColumnVector3 GroundMoment` — Ground contact moments
- `FGColumnVector3 Force` — Total forces (excluding friction/gravity)
- `FGColumnVector3 GroundForce` — Ground contact forces
- `FGColumnVector3 vGravAccel` — Gravity vector (ECEF frame)
- `FGColumnVector3 vPQRi, vPQR` — Angular velocities (ECI and ECEF frames)
- `FGColumnVector3 vUVW` — Body-frame velocity
- `FGColumnVector3 vInertialPosition` — ECI position
- `FGColumnVector3 vOmegaPlanet` — Earth rotation vector (ECI)
- `FGColumnVector3 TerrainVelocity, TerrainAngularVel` — Terrain motion
- `double DeltaT` — Time step
- `double Mass` — Body mass
- `std::vector<LagrangeMultiplier*> *MultipliersList` — Friction constraint multipliers

### Key Methods

#### Initialization & Execution
- `bool InitModel(void) override` (line 107)
- `bool Run(bool Holding) override` (line 116) — Compute accelerations for current state
- `void InitializeDerivatives(void)` (line 312) — Reset derivative caches

#### Translational Acceleration Access
- `const FGColumnVector3& GetUVWdot(void)` (line 130) — Acceleration in body frame (ft/sec²)
  - Includes Coriolis, centripetal, and gravity effects
- `const FGColumnVector3& GetUVWidot(void)` (line 146) — Acceleration in ECI frame (ft/sec²)
  - Excludes Coriolis/centripetal (inertial frame)
- `double GetUVWdot(int idx)` (line 191) — Component accessor
- `const FGColumnVector3& GetBodyAccel(void)` (line 205) — Acceleration from applied forces only (no gravity)

#### Angular Acceleration Access
- `const FGColumnVector3& GetPQRdot(void)` (line 162) — Angular acceleration (rad/sec²), ECEF frame
- `const FGColumnVector3& GetPQRidot(void)` (line 177) — Angular acceleration (rad/sec²), ECI frame
- `double GetPQRdot(int axis)` (line 235) — Component accessor

#### Force & Moment Accessors
- `FGColumnVector3 GetForces(void)` (line 264) — Total forces = applied + friction (excluding gravity)
- `double GetForces(int idx)` (line 263) — Component
- `FGColumnVector3 GetMoments(void)` (line 250) — Total moments = applied + friction + gravitational torque
- `double GetMoments(int idx)` (line 249) — Component
- `FGColumnVector3 GetGroundForces(void)` (line 292) — Ground reaction forces only
- `FGColumnVector3 GetGroundMoments(void)` (line 278) — Ground reaction moments only
- `FGColumnVector3 GetWeight(void)` (line 306) — Weight force = Mass × g (body frame)
- `double GetWeight(int idx)` (line 305) — Component
- `double GetGravAccelMagnitude(void)` (line 207) — Magnitude of gravity

#### Configuration
- `void SetHoldDown(bool hd)` (line 318) — Enable "hold-down" mode (e.g., rocket on pad)

### Private Members

**Computed Accelerations** (lines 367–371)
- `FGColumnVector3 vPQRdot` — Angular accel. (ECEF frame)
- `FGColumnVector3 vPQRidot` — Angular accel. (ECI frame)
- `FGColumnVector3 vUVWdot` — Linear accel. (body frame, with Coriolis)
- `FGColumnVector3 vUVWidot` — Linear accel. (ECI frame, inertial)
- `FGColumnVector3 vBodyAccel` — Acceleration from forces only (no gravity)
- `FGColumnVector3 vFrictionForces, vFrictionMoments` — Ground friction

**Configuration Flags** (line 373)
- `bool gravTorque` — Enable/disable gravitational torque calculation

### Private Methods

- `void CalculatePQRdot(void)` (line 375) — Euler's rotational equation: I·α = M - ω×(I·ω)
- `void CalculateUVWdot(void)` (line 376) — Newton's second law: F = m·a (accounting for frame rotation)
- `void CalculateFrictionForces(double dt)` (line 378) — Ground friction from Lagrange multipliers

### Properties (from header documentation)
- `simulation/gravity-model` — 0: spherical, 1: WGS84 (default)
- `simulation/gravitational-torque` — Enable/disable gravitational torque (default: disabled)

---

## 3. FGQuaternion.h/.cpp — Quaternion Representation

**File Paths:**
- Header: `E:\dev\jsbsim\src\math\FGQuaternion.h` (569 lines)
- Implementation: `E:\dev\jsbsim\src\math\FGQuaternion.cpp`

### Purpose
Represents 3D rotations using unit quaternions. Manages conversions between quaternion, Euler angles, and transformation matrices. Uses lazy caching to avoid recomputation.

### Key Constructors

- `FGQuaternion()` (line 90) — Identity rotation (1, 0, 0, 0)
- `FGQuaternion(const FGQuaternion& q)` (line 98) — Copy constructor
- `FGQuaternion(double phi, double tht, double psi)` (line 105) — From Euler angles (radians)
  - Order: Yaw-Pitch-Roll (Z-Y-X or 3-2-1)
- `FGQuaternion(FGColumnVector3 vOrient)` (line 110) — From Euler vector
- `FGQuaternion(int idx, double angle)` (line 117) — Single Euler angle
  - `idx=1` (ePhi) → roll, `idx=2` (eTht) → pitch, `idx=3` (ePsi) → yaw
- `FGQuaternion(double angle, const FGColumnVector3& axis)` (line 152) — Axis-angle representation
- `FGQuaternion(const FGMatrix33& m)` (line 171) — From rotation matrix

### Key Methods

#### Quaternion Operations
- `FGQuaternion GetQDot(const FGColumnVector3& PQR)` (line 183) — Quaternion time derivative
  - Implements: q̇ = 0.5 * q ⊗ [0, p, q, r]
  - Used in propagation loop
- `FGQuaternion operator*(const FGQuaternion& q)` (line 406) — Quaternion multiplication (composition of rotations)
- `FGQuaternion operator+(const FGQuaternion& q)` (line 389) — Addition (for integration)
- `FGQuaternion operator-(const FGQuaternion& q)` (line 397) — Subtraction
- `FGQuaternion operator*=(double scalar)` (line 370) — Scalar multiplication
- `FGQuaternion Inverse(void)` (line 436) — Inverse (conjugate for unit quaternions)
- `FGQuaternion Conjugate(void)` (line 450) — Conjugate (swap sign of vector part)
- `double Magnitude(void)` (line 460) — Quaternion norm
- `double SqrMagnitude(void)` (line 466) — Squared norm
- `void Normalize(void)` (line 476) — Normalize to unit quaternion

#### Attitude Representation
- `const FGColumnVector3& GetEuler(void)` (line 199) — Euler angles [phi, theta, psi] (rad)
- `const FGMatrix33& GetT(void)` (line 188) — Transformation matrix (body→local)
- `const FGMatrix33& GetTInv(void)` (line 193) — Inverse transformation (local→body)
- `double GetEuler(int i)` (line 210) — Single Euler angle
- `double GetSinEuler(int i)` (line 237), `GetCosEuler(int i)` (line 245) — Cached trig values

#### Comparison
- `bool operator==(const FGQuaternion& q)` (line 335) — Exact equality
- `bool operator!=(const FGQuaternion& q)` (line 343) — Inequality

#### I/O
- `std::string Dump(const std::string& delimiter)` (line 482) — String representation

### Private Members (Caching)

- `double data[4]` (line 508) — Master: [q0, q1, q2, q3] = [scalar, vector]
- `mutable bool mCacheValid` (line 516) — Cache validity flag
- `mutable FGMatrix33 mT, mTInv` (line 519) — Cached transformation matrices
- `mutable FGColumnVector3 mEulerAngles` (line 523) — Cached Euler angles
- `mutable FGColumnVector3 mEulerSines, mEulerCosines` (line 526) — Cached trig values

### Private Methods

- `void ComputeDerived(void)` (line 502) — Lazy evaluation: checks cache validity, calls `ComputeDerivedUnconditional()` if stale
- `void ComputeDerivedUnconditional(void)` (line 494) — Recomputes all cached values unconditionally
- `void InitializeFromEulerAngles(double phi, double tht, double psi)` (line 529)

### Global Functions

- `FGQuaternion QExp(const FGColumnVector3& omega)` (line 548) — Quaternion exponential
  - Used in some integration schemes
  - q = exp(ω) = [cos(|ω|), sin(|ω|)·ω/|ω|]

---

## 4. FGRungeKutta.h/.cpp — ODE Integration Methods

**File Paths:**
- Header: `E:\dev\jsbsim\src\math\FGRungeKutta.h` (182 lines)
- Implementation: `E:\dev\jsbsim\src\math\FGRungeKutta.cpp`

### Purpose
Provides classical and adaptive Runge-Kutta integrators for solving ordinary differential equations. Used as fallback/alternative to multi-step Adams-Bashforth methods.

### Key Classes

**FGRungeKuttaProblem** (lines 66–69)
- Abstract base for ODE systems
- Virtual method: `virtual double pFunc(double x, double y) = 0`
- Represents: dy/dx = f(x, y)

**FGRungeKutta** (lines 78–123)
- Abstract base class for RK integrators
- Public interface:
  - `int init(double x_start, double x_end, int intervals = 4)` (line 84)
  - `double evolve(double y_0, FGRungeKuttaProblem *pf)` (line 86) — Single integration step
  - `double getXEnd()` (line 88), `getError()` (line 89)
  - `int getStatus()` (line 91), `getIterations()` (line 92)
  - `void clearStatus()` (line 93), `setTrace(bool t)` (line 94)
- Status enum (line 82):
  - `eNoError`, `eMathError`, `eFaultyInit`, `eEvolve`, `eUnknown`
- Protected members:
  - `double h` — Step size
  - `double h05` — h*0.5 (half-width)
  - `double err` — Estimated error

**FGRK4** (lines 133–137)
- Classical 4th-order Runge-Kutta
- Private: `double approximate(double x, double y)` (line 136) — RK4 step: y' = y + (k1 + 2k2 + 2k3 + k4)/6
- 4 evaluations per step
- 4th-order local truncation error O(h⁵)

**FGRKFehlberg** (lines 156–176)
- Runge-Kutta-Fehlberg (RKF45) — Semi-adaptive method
- Public:
  - `double getEpsilon()` (line 161), `setEpsilon(double e)` (line 163)
  - `int getShrinkAvail()` (line 162), `setShrinkAvail(int s)` (line 164)
- Features:
  - Uses 6 function evaluations per step
  - Compares 5th-order and 4th-order estimates to detect error
  - Adapts interval (shrinks only, not expands) if error threshold exceeded
  - `shrink_avail` limits number of shrink attempts per interval
  - Default `epsilon = 1e-12` (error tolerance)
- Static coefficient arrays: `A2[], A3[], A4[], A5[], A6[], B[], Bs[], C[]`

### Key Behavior

**Integration Loop (pseudo-pseudocode)**
```
For each interval [x0, x_end]:
  For each sub-interval with step h:
    k_i = pFunc(x + c_i*h, y + sum(a_ij*k_j))
    y_next = y + h * sum(b_i*k_i)
    If RKF: compare with alternate estimate, shrink if error > epsilon
  Advance to next interval
```

---

## 5. FGLocation.h/.cpp — Geographic Position

**File Paths:**
- Header: `E:\dev\jsbsim\src\math\FGLocation.h` (551 lines)
- Implementation: `E:\dev\jsbsim\src\math\FGLocation.cpp`

### Purpose
Represents geographic location as ECEF Cartesian coordinates with efficient conversion to/from geodetic (lon/lat) and local (NED) frames. Uses WGS84 ellipsoid model.

### Key Constructors

- `FGLocation(void)` (line 155) — Default at origin
- `FGLocation(double lon, double lat, double radius)` (line 162) — From geodetic + radius (GEOCENTRIC lat)
- `FGLocation(const FGColumnVector3& lv)` (line 168) — From ECEF Cartesian vector
- `FGLocation(const FGLocation& l)` (line 171) — Copy

### Key Methods

#### Position Setters
- `void SetLongitude(double longitude)` (line 184) — Lon in rad, preserves lat/radius
- `void SetLatitude(double latitude)` (line 197) — GEOCENTRIC lat in rad
- `void SetRadius(double radius)` (line 209) — Distance from Earth center (ft)
- `void SetPosition(double lon, double lat, double radius)` (line 215) — All three
- `void SetPositionGeodetic(double lon, double lat, double height)` (line 221) — GEODETIC lat + altitude above ellipsoid

#### Position Getters
- `double GetLongitude()` (line 234) — Lon in rad [-π, π]
- `double GetLatitude()` (line 252) — GEOCENTRIC lat in rad
- `double GetGeodLatitudeRad()` (line 258), `GetGeodLatitudeDeg()` (line 273) — GEODETIC lat
- `double GetRadius()` (line 291) — Distance from Earth center (ft)
- `double GetGeodAltitude()` (line 279) — Altitude above WGS84 ellipsoid
- `double GetLongitudeDeg()`, `GetLatitudeDeg()` (lines 458–459) — Degrees

#### Ellipsoid Configuration
- `void SetEllipse(double semimajor, double semiminor)` (line 228) — Set WGS84 parameters (or custom planet)
  - Default: WGS84 Earth (a ≈ 20925646 ft, c ≈ 20855486 ft)

#### Trig Helpers
- `double GetSinLongitude()` (line 243), `GetCosLongitude()` (line 246)

#### Coordinate Conversions
- `FGLocation LocalToLocation(const FGColumnVector3& lvec)` (line 326) — Local (NED) offset → ECEF location
- `FGColumnVector3 LocationToLocal(const FGColumnVector3& ecvec)` (line 336) — ECEF vector → Local offset
- `double GetDistanceTo(double target_lon, double target_lat)` (line 309) — Geodetic distance between two locations (ft)
- `double GetHeadingTo(double target_lon, double target_lat)` (line 318) — Bearing to target (rad)

#### Transformation Matrices
- `const FGMatrix33& GetTl2ec(void)` (line 296) — Local→ECEF matrix (NED→ECEF)
- `const FGMatrix33& GetTec2l(void)` (line 301) — ECEF→Local (ECEF→NED)

#### Vector-Like Operators (for time-stepping)
- `double operator()(unsigned int idx)` (line 347) — Read ECEF component (1-based)
- `double& operator()(unsigned int idx)` (line 354) — Write ECEF component
- `const FGLocation& operator=(const FGColumnVector3& v)` (line 384) — Set from vector
- `FGLocation& operator=(const FGLocation& l)` (line 397) — Copy assignment
- `const FGLocation& operator+=(const FGLocation& l)` (line 413) — Add ECEF vectors
- `const FGLocation& operator-=(const FGLocation& l)` (line 423) — Subtract
- `const FGLocation& operator*=(double scalar)` (line 433), `operator/=(double scalar)` (line 443)
- `FGLocation operator+(const FGLocation& l)` (line 450) — Addition
- `FGLocation operator-(const FGLocation& l)` (line 459) — Subtraction
- `FGLocation operator*(double scalar)` (line 469) — Scalar multiplication
- Cast: `operator const FGColumnVector3&()` (line 476) — Implicit cast to ECEF vector

#### Comparison
- `bool operator==(const FGLocation& l)` (line 401) — Equality
- `bool operator!=(const FGLocation& l)` (line 407) — Inequality

#### I/O
- `double Entry(unsigned int idx)` (line 364), `double& Entry(unsigned int idx)` (line 374) — Alternative access

### Private Members (Cached)

- `FGColumnVector3 mECLoc` (line 504) — Master: Cartesian ECEF position (ft)
- `mutable double mLon, mLat, mRadius` (lines 507–509) — GEOCENTRIC lon/lat/radius
- `mutable double mGeodLat, GeodeticAltitude` (lines 510–511) — GEODETIC lat/altitude
- `mutable FGMatrix33 mTl2ec, mTec2l` (lines 514–515) — NED↔ECEF transformation matrices
- `mutable bool mCacheValid` (line 530) — Cache validity flag
- `bool mEllipseSet` (line 533) — Whether `SetEllipse()` has been called

**WGS84 Parameters** (lines 518–522)
- `double a` — Semimajor axis (ft)
- `double e2` — Eccentricity squared
- `double c, ec, ec2` — Derived ellipsoid parameters

### Private Methods

- `void ComputeDerived(void)` (line 490) — Lazy evaluation
- `void ComputeDerivedUnconditional(void)` (line 484) — Recompute all cached values

### Key Design Pattern

**Cartesian-as-Master:**
- ECEF (X, Y, Z) is the master representation (linear, singularity-free)
- Lon/lat/radius derived on-demand and cached
- Avoids singularities at poles and discontinuity at ±π longitude
- Enables stable numerical integration (all components have similar rate of change)

---

## 6. FGColumnVector3.h/.cpp — 3D Vector Math

**File Paths:**
- Header: `E:\dev\jsbsim\src\math\FGColumnVector3.h` (partial read: lines 1–200)
- Implementation: `E:\dev\jsbsim\src\math\FGColumnVector3.cpp`

### Purpose
Implements 3D column vector (1×3 matrix) with standard linear algebra operations. Used for all vector quantities (forces, velocities, angular rates, positions).

### Key Constructors

- `FGColumnVector3(void)` (line 70) — Zero vector
- `FGColumnVector3(const double X, const double Y, const double Z)` (line 77) — From components
- `FGColumnVector3(const FGColumnVector3& v)` (line 86) — Copy

### Key Methods

#### Element Access
- `double operator()(const unsigned int idx)` (line 100) — Read 1-based index
- `double& operator()(const unsigned int idx)` (line 107) — Write 1-based index
- `double Entry(const unsigned int idx)` (line 117), `double& Entry(...)` (line 127) — Alternative

#### Arithmetic Operations
- `FGColumnVector3 operator*(const double scalar)` (line 171) — Scalar multiplication
- `FGColumnVector3 operator/(const double scalar)` (line 179) — Scalar division
- `FGColumnVector3 operator*(const FGColumnVector3& V)` (line 186) — Cross product (lines 187–190)
  - v1 × v2 = [y1·z2 - z1·y2, z1·x2 - x1·z2, x1·y2 - y1·x2]
- `FGColumnVector3 operator+(const FGColumnVector3& B)` (line 193) — Vector addition
- `FGColumnVector3 operator-(const FGColumnVector3& B)` (line 199) — Vector subtraction

#### Norms & Magnitude
- `double Magnitude()` (implied from context) — Euclidean norm
- (See implementation file for full list)

#### I/O
- `std::string Dump(const std::string& delimeter)` (line 132) — String representation

#### Comparison
- `bool operator==(const FGColumnVector3& b)` (line 158) — Exact equality
- `bool operator!=(const FGColumnVector3& b)` (line 165) — Inequality

#### Assignment
- `FGColumnVector3& operator=(const FGColumnVector3& b)` (line 137) — Copy assignment
- `FGColumnVector3& operator=(std::initializer_list<double> lv)` (line 147) — From initializer list {x, y, z}

---

## 7. FGMatrix33.h/.cpp — 3×3 Matrix Math

**File Paths:**
- Header: `E:\dev\jsbsim\src\math\FGMatrix33.h` (partial read: lines 1–200)
- Implementation: `E:\dev\jsbsim\src\math\FGMatrix33.cpp`

### Purpose
Implements 3×3 matrix for rotation/transformation operations. Storage is column-major (column 0: data[0,1,2], column 1: data[3,4,5], column 2: data[6,7,8]).

### Key Constructors

- `FGMatrix33(void)` (line 82) — Zero matrix
- `FGMatrix33(const FGMatrix33& M)` (line 90) — Copy
- `FGMatrix33(const double m11, ..., const double m33)` (line 117) — From 9 elements (row-major input, column-major storage)

### Key Methods

#### Element Access
- `double operator()(unsigned int row, unsigned int col)` (line 155) — Read
- `double& operator()(unsigned int row, unsigned int col)` (line 168) — Write
  - 1-based indexing: internally converts to `data[(col-1)*3 + (row-1)]`
- `double Entry(unsigned int row, unsigned int col)` (line 185), reference version (line 189)

#### Matrix Operations
- (See implementation file)

#### Transformation
- Implicit use: `Tb2l * vBody` transforms body vector to local frame

#### I/O
- `std::string Dump(const std::string& delimeter)` (line 139) — String output
- `std::string Dump(const std::string& delimiter, const std::string& prefix)` (line 146) — With indentation

#### Constants
- `enum { eRows = 3, eColumns = 3 }` (lines 73–76)

### Storage Layout

```
data[0..8] in column-major order:
  [m11  m12  m13]   [0  3  6]
  [m21  m22  m23] = [1  4  7]
  [m31  m32  m33]   [2  5  8]
```

---

## 8. FGFDMExec.h/.cpp — Executive & Model Scheduling

**File Paths:**
- Header: `E:\dev\jsbsim\src\FGFDMExec.h` (partial read: lines 1–250)
- Implementation: `E:\dev\jsbsim\src\FGFDMExec.cpp` (partial read: lines 1–150)

### Purpose
Central executive orchestrating all model execution in proper sequence. Manages initialization, model loading, state integration loop, and property binding.

### Key Enums

**eModels** (lines 225–241) — Model execution order (CRITICAL)

```cpp
ePropagate = 0,          // Step 0: Integrate state (position, velocity, attitude)
eInput,                  // Step 1: Read control inputs
eInertial,               // Step 2: Compute inertial properties (Earth rotation, gravity)
eAtmosphere,             // Step 3: Atmosphere model
eWinds,                  // Step 4: Wind model
eSystems,                // Step 5: FCS + flight control laws
eMassBalance,            // Step 6: CG & inertia (needed before forces)
eAuxiliary,              // Step 7: Auxiliary computations (Mach, alpha, beta, etc.)
ePropulsion,             // Step 8: Engine thrust & torque
eAerodynamics,           // Step 9: Aerodynamic forces & moments
eGroundReactions,        // Step 10: Ground contact forces
eExternalReactions,      // Step 11: External forces (custom)
eBuoyantForces,          // Step 12: Buoyancy
eAircraft,               // Step 13: Sum forces/moments from all sources
eAccelerations,          // Step 14: Compute accelerations (a = F/m, α = I⁻¹·M)
eOutput,                 // Step 15: Log/display outputs
eNumStandardModels       // Total count
```

**Execution Dependency Notes** (from header, lines 219–224):
1. **FCS → MassBalance** — FCS can modify inertia via point-mass properties
2. **MassBalance → (Propulsion, Aero, Ground, External, Buoyant)** — All force/moment models need current CG
3. **All Forces → Aircraft** — Sum forces/moments
4. **Aircraft → Accelerations** — Compute a and α from summed F and M

### Key Methods

#### Initialization & Execution
- `FGFDMExec(FGPropertyManager* root = nullptr, std::shared_ptr<unsigned int> fdmctr = nullptr)` (line 211)
- `~FGFDMExec()` (line 214)
- `bool Run(void)` (line 248) — Execute one time step (runs all models in eModels order)
- Constructor details (from .cpp):
  - Allocates all model objects
  - Sets default `dT = 1.0/120.0` (120 Hz)
  - Initializes property tree
  - Manages multi-FDM child instances

#### Configuration
- `RootDir` — Root aircraft directory
- `AircraftPath`, `EnginePath`, `SystemsPath` — Asset paths
- `Frame` — Frame counter (increments each Run())
- `sim_time` — Absolute simulation time
- `dT` — Time step (seconds)
- `modelLoaded` — Model initialization flag
- `Terminate` — Stop simulation flag
- `HoldDown` — Hold-down mode (rocket on pad)
- `IncrementThenHolding` — Increment dT once, then hold
- `TimeStepsUntilHold` — Countdown for hold

#### Properties
- `simulator/do_trim` (write-only) — Trigger trim: 0=longitudinal, 1=full, 2=ground, 3=pullup, 4=custom, 5=turn

#### Child FDM Management
- Multi-FDM support for multi-vehicle simulations
- Each child has own state and position relative to parent
- `childData` struct stores child exec + relative location

### Private Members (Constructor Init)

```cpp
Frame = 0
disperse = 0                           // Dispersions flag (from JSBSIM_DISPERSE env var)
RandomSeed, RandomGenerator            // RNG for Monte Carlo
FDMctr (shared_ptr)                    // Child FDM counter
IdFDM                                  // This FDM's ID
modelLoaded = false
IsChild = false
holding = false
Terminate = false
HoldDown = false
IncrementThenHolding = false
TimeStepsUntilHold = -1
sim_time = 0.0
dT = 1.0/120.0                        // Default: 120 Hz
AircraftPath = "aircraft"
EnginePath = "engine"
SystemsPath = "systems"
debug_lvl (from JSBSIM_DEBUG env var)
```

### Model References

Executive creates/manages:
- `Propagate` (FGPropagate) — State integration
- `Input` (FGInput) — Control inputs
- `Inertial` (FGInertial) — Earth properties
- `Atmosphere` (FGAtmosphere) — Atmospheric model
- `Winds` (FGWinds) — Wind field
- `FCS` (FGFCS) — Flight control system
- `MassBalance` (FGMassBalance) — Mass and inertia
- `Auxiliary` (FGAuxiliary) — Auxiliary calcs (Mach, angles, airspeed)
- `Propulsion` (FGPropulsion) — Engines/rotors
- `Aerodynamics` (FGAerodynamics) — Aerodynamic forces
- `GroundReactions` (FGGroundReactions) — Landing gear
- `ExternalReactions` (FGExternalReactions) — External forces
- `BuoyantForces` (FGBuoyantForces) — Buoyancy (for lighter-than-air)
- `Aircraft` (FGAircraft) — Force/moment summation
- `Accelerations` (FGAccelerations) — Acceleration computation
- `Output` (FGOutput) — Data logging

---

## Summary: Integration & Propagation Architecture

### State Representation

**Master State Variables** (in FGPropagate::VehicleState):
- Position: `vLocation` (ECEF Cartesian, ft)
- Velocity: `vUVW` (body-frame velocity, ft/sec)
- Attitude: `qAttitudeLocal` (quaternion, body→NED)
- Angular Rate: `vPQR` (body-frame rates rel. ECEF, rad/sec)

**Inertial Frame Counterparts** (computed from above):
- `vInertialPosition` (ECI Cartesian)
- `vInertialVelocity` (ECI velocity)
- `qAttitudeECI` (body→ECI quaternion)
- `vPQRi` (body-frame rates rel. ECI)

### Integration Pipeline

1. **FGPropagate::Run()** — Main loop
   - Reads `Accelerations.GetUVWidot()` (ECI accel)
   - Reads `Accelerations.GetPQRidot()` (ECI angular accel)
   - Selects integrator (Euler, Trapezoidal, Adams-Bashforth 2-4, etc.)
   - Integrates: vUVW, vPQR, vLocation, qAttitude
   - Updates all transformation matrices

2. **FGAccelerations::Run()** — Force/moment balance
   - Reads applied forces/moments from all subsystems
   - Transforms gravity to body frame
   - Computes: UVWdot = (F + F_gravity) / m - ω × V
   - Computes: PQRdot = I⁻¹ · (M - ω × I·ω)
   - Handles ground friction via Lagrange multipliers

3. **Coordinate Frames**
   - **ECI** — Inertial (non-rotating, stars fixed)
   - **ECEF** — Earth-centered, Earth-fixed (rotates with planet)
   - **Local/NED** — North-East-Down (local horizontal, Z down)
   - **Body** — Aircraft body frame (X forward, Z down, right-hand)

4. **Integrator Selection**
   - Property-driven (can be changed at runtime)
   - Default: Trapezoidal for position, Adams-Bashforth 2 for rates
   - Multi-step methods use derivative history (deques in VehicleState)

5. **Key Algorithms**
   - **Quaternion Integration**: q_dot = 0.5 * q ⊗ [0, p, q, r]
   - **Rotating Earth**: Accounts for ω_planet (Earth rotation vector in ECI)
   - **Coriolis & Centripetal**: Naturally emerge from ECEF frame dynamics
   - **Gravity**: WGS84 or spherical, optionally with gravitational torque

### Execution Order (Per Frame)

```
1. Propagate.Run()              // Integrate state from previous accelerations
2. Input.Run()                  // Read control inputs
3. Inertial.Run()              // Earth properties (gravity, rotation)
4. Atmosphere.Run()            // Air density, temperature, pressure
5. Winds.Run()                 // Wind field at current location
6. Systems.Run()               // Flight control laws
7. MassBalance.Run()           // Update CG, moment of inertia
8. Auxiliary.Run()             // Mach, alpha, beta, airspeed, etc.
9. Propulsion.Run()            // Thrust, torque from engines
10. Aerodynamics.Run()          // Lift, drag, pitching moment
11. GroundReactions.Run()       // Landing gear forces (if in contact)
12. ExternalReactions.Run()     // Custom external forces
13. BuoyantForces.Run()         // Buoyancy forces
14. Aircraft.Run()              // Sum all forces → F, sum all moments → M
15. Accelerations.Run()         // Compute a = F/m, α = I⁻¹·M
16. Output.Run()                // Log state to file/network
```

→ Next frame: Loop back to step 1 with new accelerations

---

## Key Files & Lines Summary

| File | Role | Key Components |
|------|------|-----------------|
| `FGPropagate.h` (689 L) | State integration | VehicleState, integrator selection, all frame transforms |
| `FGAccelerations.h` (386 L) | Force/moment → accel | Inputs struct, PQRdot/UVWdot computation, gravity |
| `FGQuaternion.h` (569 L) | Rotation representation | Quaternion ops, Euler conversion, caching |
| `FGRungeKutta.h` (182 L) | ODE solvers | FGRK4, FGRKFehlberg (fallback integrators) |
| `FGLocation.h` (551 L) | Geographic coordinates | ECEF/NED conversion, WGS84 ellipsoid |
| `FGColumnVector3.h` (partial) | 3D vectors | Cross product, scalar ops, magnitude |
| `FGMatrix33.h` (partial) | 3×3 transforms | Rotation matrices, 1-based indexing |
| `FGFDMExec.h` (partial) | Executive | Model scheduling (eModels enum), initialization |

---

## Research Implications

### For DroneSim Integration:

1. **Multi-step integrators** — Adams-Bashforth 2-4 used by default (better stability than RK4 for stiff systems)
2. **Quaternion caching** — Lazy evaluation of Euler angles and transforms (avoid recomputation)
3. **ECEF-as-master** — Location stored as Cartesian (numerical stability)
4. **Rotating Earth handling** — ω_planet explicitly included in dynamics
5. **Frame separation** — ECI for translation (inertial), Body for rotation (natural for aerodynamics)
6. **Gravity model** — Switchable between spherical and WGS84 (with optional gravitational torque)
7. **Execution ordering** — Critical: FCS before MassBalance, all forces before Accelerations

