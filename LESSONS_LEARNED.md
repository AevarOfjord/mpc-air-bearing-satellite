# Lessons Learned

This document captures the complete development journey, technical breakthroughs, and lessons learned from building a functional satellite thruster control system with Model Predictive Control and includes failed approaches, design evolution, and critical insights.

---

## Table of Contents

- [Development Timeline & Major Milestones](#development-timeline--major-milestones)
- [Hardware Design & Implementation](#hardware-design--implementation)
- [Software Control Algorithm Development](#software-control-algorithm-development)
- [Critical Breakthroughs (The Big Three)](#critical-breakthroughs-the-big-three)
- [Recommendations for Future Work](#recommendations-for-future-work)
- [Conclusion](#conclusion)

---

## Development Timeline & Major Milestones

### Phase 1: Conceptualization & Initial Design
- Literature review of MPC for satellite control
- Initial hardware design concepts (multiple thruster configurations explored)
- Initial budget planning and component sourcing

### Phase 2: Hardware Prototyping
- First satellite structure built with aluminum extrusion
- Initial power system using 5x LiPo batteries (later abandoned)
- First thruster configuration testing
- Acrylic sheet integration
- **Major Issue**: Power system unreliability

### Phase 3: MPC Software Development
- Gurobi solver integration
- Initial MPC formulation (linearized dynamics)
- Physics-based simulation development
- **Major Issue**: Solver timing violations, inadequate model accuracy

### Phase 4: Simulation Refinement
- Physics model validation and tuning
- Initial cost weight optimization
- Simulation vs theoretical behavior validation
- **Achievement**: Working simulation environment

### Phase 5: First Hardware Integration Attempts
- WiFi communication implementation (FAILED - too slow)
- Initial thruster calibration using vertical test stand (INADEQUATE)
- First closed-loop hardware tests (UNSTABLE - multiple reasons)
- **Critical Realization**: Model accuracy and communication latency are showstoppers

### Phase 6: The Breakthrough Period
- **Breakthrough 1**: ESP-NOW communication protocol (<10ms latency)
- **Breakthrough 2**: In-situ thruster calibration bench
- **Breakthrough 3**: Gurobi MIPGap tuning for real-time performance
- First successful stable hardware control achieved
- Iterative testing and refinement

### Phase 7: Validation & Documentation
- Simulation vs. hardware validation
- Performance characterization across mission types
- Documentation and thesis preparation
- **Achievement**: Robust, repeatable hardware control system

**Total Development Time**: ~8 months from concept to validated system

---

## Hardware Design & Implementation

### Power System Evolution

#### Initial Design (Failed)
**Configuration**: 5x 3.7V LiPo batteries
- 3x batteries in series → ~11.1V for solenoids
- 1x battery through boost converter → 5V for relays
- 1x battery → direct power for microcontroller and sensors

**Why It Failed**:
1. **Unreliable voltage regulation**: Battery discharge curves meant voltage dropped during operation
2. **Insufficient current capacity**: Simultaneous thruster firing caused voltage sag
3. **Complex power distribution**: Multiple conversion stages introduced failure points
4. **Poor modularity**: Battery replacement required top panel to be taken off the satellite structure
5. **Safety concerns**: No battery management system, risk of over-discharge

**Quantitative Issues**:
- Voltage drop during 4+ thruster operation
- Battery life: ~15 minutes of continuous operation
- Replacement time: 15 minutes to swap all batteries and recalibrate mass distribution

#### Final Design (Successful)
**Configuration**: 2x Milwaukee M12 batteries (12V)

**Battery 1** (Solenoid Power):
- Direct 12V connection to all 8 solenoids
- Dedicated battery ensures consistent voltage under load
- Current capacity: 20A continuous (adequate for 8 solenoids @ 0.5A each)

**Battery 2** (System Power) with dual regulation:
- **Line 1**: 3.9V regulator → XIAO ESP32-S3, distance sensors and other devices
- **Line 2**: 5V regulator → 8x relay circuits for solenoid control

**Why It Succeeded**:
1. **Voltage stability**: 12V nominal voltage stays 11.8-12.2V throughout discharge
2. **Ample current capacity**: Each M12 battery can deliver 20A continuous
3. **Modularity**: Hot-swappable batteries
4. **Commercial reliability**: Milwaukee M12 system has built-in protection circuits
5. **Extended runtime**: 6.0Ah capacity provides ~45 minutes of operation

**Quantitative Improvements**:
- Voltage stability
- Runtime: 45 minutes (vs. 15 minutes)
- Swap time: 10 seconds per battery

**Design Lesson**: **Overdesign power capacity for prototypes**. Power issues create failures that are difficult to diagnose. Commercial battery systems (Milwaukee, DeWalt, etc.) provide reliability and modularity worth the extra cost and weight.

---

### Structural Design Evolution

#### Material Selection Philosophy
The structural design evolved through three iterations to arrive at optimal material usage:

**Final Material Categories**:
1. **Aluminum extrusion** (20mm × 20mm T-slot) - Satellite chassis frame
2. **Aluminum sheet** (3mm thickness) - High-load bearing surfaces
3. **3D printed PLA+** - Custom geometry brackets and enclosures

#### Evolution of Structural Design

**Iteration 1: Mixed Materials (Problematic)**
- Aluminum extrusion frame
- Acrylic side panels
- Acrylic sheets for mounting components within the satellite structure
- Aluminum fasteners throughout

**Problems with Iteration 1**:
- Acrylic panels **too brittle**: Cracked during modification for side panels and could not be water cut
- Drilling precise holes is less accurate than 3d printing holes

**Iteration 2: Aluminum + Acrylic (Better but Still Issues)**
- Replaced acrylic mounting sheets with 3D printed PLA+ inside the satellite structure
- Kept acrylic side panels
- Added 3D printed camera mounts

**Remaining Issues**:
- Acrylic side panels still difficult to modify to fit cameras and distance sensors

**Iteration 3: Aluminum + 3D Printed Only (Final)**
- Aluminum extrusion: Structural frame (290mm × 290mm × 290mm)
- Aluminum sheet: Air bearing mounting plates (supports full 23kg) and air cylinder mounting plate
- 3D printed PLA+: All panels, mounts and some brackets and fasteners

**Why Final Design Works**:
1. **Modularity**: 3D printed panels easy to modify, reprint, iterate, while the acrylic sheets were too brittle to be water cut and had to be cut using less precise methods
2. **Precision**: Instead of having to drill holes into the acrylic sheet using mounts to make holes as precise as possible, holes could be 3D printed into the 3D printed sheet instead with more accuracy 
3. **Rapid iteration**: Design change → print overnight → test next day
4. **Integration flexibility**: Heated inserts in 3D prints provide reliable threading instead of using nuts

#### Internal Mounting

**Initial Approach**: Acrylic sheet mounting plates with drilled holes and nuts to tighten everything down
- **Issues**: Modification difficult and less accurate

**Final Approach**: 3D printed mounting plates with brass heated inserts
- **Process**: Print part holes → insert brass threaded inserts
- **Accuracy**: The 3D printed holes ended up being much more accurate than the drilled holes and also provided more secure mounting using the heated inserts rather than tightening things down using nuts

---

### Satellite Geometry & Thruster Configuration

#### Design Exploration Phase
Multiple thruster configurations were explored before selecting the final square arrangement. Each configuration was evaluated using rational analysis since the simulation software was still in development at that time.
- Control authority (ability to generate forces/torques in all directions)
- Computational complexity (impact on MPC solve time)
- Fuel efficiency

#### Configurations Considered

**Option 1: Square with 8 thrusters (SELECTED)**
- Geometry: Square
- Thruster arrangement: 2 thrusters per face, 4 faces
- Orientation: Opposite faces have mirrored thruster pairs

**Option 2: Triangle with 6 thrusters**
- Geometry: Equilateral triangle
- Thruster arrangement: 2 thrusters per face, 3 faces
- Advantage: Lower complexity (6 thrusters vs. 8)
- Disadvantage: Was not sure how good it would handle sharp rotations since it could not produce pure torque like the square shape

**Option 3: Hexagon with 6 thrusters**
- Geometry: Hexagon
- Thruster arrangement: 1 thruster per face, 6 faces
- Advantage: Lower complexity (6 thrusters vs. 8)
- Disadvantage: Not sure about the torque generation

**Option 4: Hexagon with 12 thrusters**
- Geometry: Hexagon
- Thruster arrangement: 2 thrusters per face, 6 faces
- Advantage: More thrusters could improve precise movements
- Disadvantage: Higher complexity for the MPC

**Option 5: Octagon with 8 thrusters**
- Geometry: Regular octagon
- Thruster arrangement: 1 thruster per face
- Advantage: More thrusters could improve precise movements
- Disadvantage: Uncertain about torque generation capability

#### Why Square Configuration Was Selected

**Decision Factors**:

1. **Adequate Torque Generation**
   - Largest moment arm of all shapes considered
   - Can produce pure torque

2. **Computational complexity**
   - Fewer thrusters mean less computational difficulty with fewer binary variables


**Quantitative Justification**:

| Configuration | Thrusters | Pure torque | Binary vars/step | Total MPC vars (N=12) |
|---------------|-----------|------------------|------------------|-----------------------|
| Square | 8 | Yes | 8 | 96 |
| Triangle | 6 | No | 6 | 72 |
| Hexagon | 6 | No | 6 | 72 |
| Hexagon | 12 | Yes | 12 | 144 |
| Octagon | 8 | No | 8 | 96 |

**Frame Dimensions Rationale**:
- **290mm × 290mm**: Driven by moment arm and sizes of components and devices that need to fit within the structure
- Smaller frame (e.g., 200mm): Insufficient moment arm for responsive attitude control and might be hard to fit all components and devices
- Larger frame (e.g., 400mm): Unnecessarily large, increased moment arm reduces agility
- **Trade-off point**: 290mm provides sufficient torque and also enough room to fit everything within the structure

#### Future Research Opportunities
Now that simulation is validated against hardware, exploring alternative geometries could yield insights:
- **Triangle (6 thrusters)**: Can it produce the pure torque needed for sharp corners 
- **Hexagon (12 thrusters)**: Can the MPC handle this level of complexity
- **Minimum spacing study**: How small can the moment arm be before it affects performance?
- **Fuel efficiency comparison**: Which geometry minimizes thruster usage for typical missions?

---

### Communication System: The Latency Breakthrough

Communication latency was identified as a **critical bottleneck** for this closed-loop hardware control. This section documents the evolution from inadequate WiFi to the successful ESP-NOW solution.

#### The Communication Requirement

**MPC Control Loop Architecture**:
1. **Ground Station** (Desktop running Python):
   - Receives current state from motion capture (OptiTrack Motive)
   - Computes optimal control using Gurobi MPC solver
   - Sends thruster commands to satellite

2. **Satellite Onboard Controller** (XIAO ESP32-S3):
   - Receives thruster commands via wireless link to computer
   - Actuates solenoid valves (8 thrusters)


#### Iteration 1: WiFi Communication (FAILED)

**Implementation**:
- Protocol: WiFi 802.11n (2.4GHz)
- Communication: TCP sockets
- Microcontroller: XIAO ESP32-S3 (chosen for WiFi capability)
- Network: Dedicated WiFi router for satellite communication

**Measured Performance**:
- **Latency**: Estimated around 200ms
- **Jitter**: High, noticeable variation
- **Packet loss**: 2-5% under ideal conditions, 10-15% with interference
- **Throughput**: 1-2 Mbps more than needed, but latency unacceptable

**Why WiFi Failed**:
1. **Protocol overhead**: TCP handshake, acknowledgment, retransmission add latency

**Observed Hardware Behavior with WiFi**:
- Satellite motion **unstable and oscillatory**
- Control commands arrived too late → thruster firing based on stale state
- MPC predictions invalid because state has changed by time command executes
- System could not stabilize even at single waypoint

#### Iteration 2: ESP-NOW Protocol (SUCCESS)

**What is ESP-NOW?**
- Espressif's protocol for ESP32/ESP8266 devices
- **Connectionless** peer-to-peer communication (no WiFi router needed)
- Operates on WiFi hardware but bypasses WiFi protocol stack
- Maximum 250 bytes per packet, up to 250kbps data rate

**Implementation**:
- Direct ESP32-to-ESP32 communication
- Ground station: ESP32 connected to laptop via USB serial (Gateway)
- Satellite: XIAO ESP32-S3 onboard controller
- Packet structure: 8 bytes (1 byte per thruster command)

**Measured Performance**:
- **Latency**: around 10ms
- **Jitter**: Very low, not noticeable
- **Throughput**: Adequate for 8-byte commands at 16.67Hz

**Why ESP-NOW Succeeded**:
1. **Minimal protocol overhead**: No TCP handshake, no acknowledgment
2. **Direct hardware-to-hardware**: Bypasses operating system network stack
3. **Optimized for low latency**: Designed for IoT sensor applications
4. **Deterministic timing**: Connectionless protocol has predictable latency

**Quantitative Comparison**:

| Metric | WiFi (Failed) | ESP-NOW (Success) |
|--------|---------------|-------------------|
| Typical latency | 200ms | 10ms |
| Jitter (std dev) | High | Low |
| Reliability | Unacceptable | Excellent |

**Impact on Control Performance**:
- With WiFi: **Unstable**, oscillatory, unable to hold position
- With ESP-NOW: **Stable**, precise waypoint tracking, successful at holding position

**Hardware Behavior Comparison**:
```
WiFi:
- Command sent at t=0ms    → Satellite at position X₀
- Command arrives at t=120ms → Satellite now at position X₀ + drift
- MPC predicted for X₀, but satellite is now at X₀ + drift → WRONG CONTROL

ESP-NOW:
- Command sent at t=0ms    → Satellite at position X₀
- Command arrives at t=5ms → Satellite at position X₀ + minimal drift
- MPC prediction still valid → CORRECT CONTROL
```

**The Breakthrough Moment**:
After struggling with WiFi instability and trying different ways of sending the signal through a dedicated router and so on, switching to ESP-NOW resulted in **immediate stable control signals** with no noticeable jitter and latency delay.

**Advisor Credit**: Dr. Poonawala, one of the advisors for this project, suggested using ESP-NOW after observing WiFi latency issues. This recommendation was **important** to project success.

**Lesson for Future Work**: **Communication latency is non-negotiable for MPC**. If control loop requires low latency communication, standard WiFi/Bluetooth are inadequate. ESP-NOW (for ESP32) or similar ultra-low-latency protocols are essential. Don't assume "WiFi is fast enough" without measuring actual latency under realistic conditions.

---

### Center of Mass Balancing

Achieving precise center of mass (COM) alignment with the geometric center is **critical** for the satellite dynamics. Any COM offset creates unintended torques during linear thrust, requiring corrective control.

#### Design Strategy for Symmetric Mass Distribution

**Principle**: Design all components in **symmetric pairs** or **centered placement**

**Symmetric Component Placement**:
- **4 identical faces**: Each face has same thruster configuration, same electronics mounting
- **Thrusters**: 2 per face × 4 faces = 8 total, all at identical radial distance from center
- **Electronics**: Sensors and devices distributed symmetrically

**Heavy Component Strategy**:
- **Air cylinder** (heaviest: ~8kg): Mounted **exactly** at geometric center, aligned with vertical axis

#### Handling Asymmetric Components

**Problem**: Pressure regulator (~800g) must mount to air cylinder, extends ~10cm from center

**Solution**: Air bearing placement adjustability

**Design Implementation**:
- Base platform has **three radial grooves** at 120° intervals
- Each air bearing slides along its groove (adjustable radial offset)
- Grooves allow COM correction by shifting support points

**Balancing Procedure**:
1. Assemble satellite with all components
2. Place the satellite on precision scales where each air bearing rests on a separate scale, measure total mass (23kg measured)
3. Adjust air bearing positions radially to compensate for imbalance
4. Iterate until all scales are within 50g of each other
5. Add small sliding weights on the structure to account for minimal remaining error

**Achieved Performance**:
- **Total mass**: 23.0kg
- **COM error**: 20g mass equivalent
- **Percentage error**: 0.09% of total mass

**Design Lesson**: **Adjustability is more important than perfect initial placement**. The radial air bearing slides allowed for post-assembly COM correction without rebuilding the structure. Plan for adjustment mechanisms in your design rather than assuming perfect manufacturing/assembly.

---

### Thruster Calibration: The Critical Hardware Breakthrough

This was **the most important hardware breakthrough** of the entire project. Accurate thruster characterization is **non-negotiable** for model-based control like MPC.

#### The Core Problem: Model-Reality Mismatch

**MPC Fundamentals**:
- MPC is a **model-based** controller
- Solver predicts future states using satellite dynamics model
- Model includes thrust forces: F = f(command)
- **If model is wrong, predictions are wrong → control fails**

**Initial Situation**:
- Simulation: MPC controller works perfectly, satellite tracks trajectories
- Hardware: MPC controller **completely unstable**, satellite uncontrollable
- **Root cause**: Simulated results did not match real satellite behavior. The simulated results would do what was expected and stabilize at a target, but the hardware would never reach the target. Error was caused by using estimated thrust forces based on pressure and velocity calculations rather than actual measured values

#### Iteration 1: Vertical Test Stand (INADEQUATE)

**Setup**:
- 3D printed bracket holds thruster **vertical**
- Thruster fires **downward** onto digital scale
- Scale measures force
- Record force generated

**Procedure**:
1. Mount thruster in vertical bracket (requires removing thruster from satellite)
2. Connect air supply with regulator set to 40 PSI
3. Fire thruster
4. Record scale reading over several seconds
5. Use the mean values of the stable force period

**Problems with Vertical Test Stand**:

1. **Measurement accuracy limited**:
   - Kitchen scale: ±1g accuracy (±0.01N force error)
   - Systematic errors (mounting, airflow) are larger than measurement noise

2. **Cannot test in actual flight configuration**:
   - Must remove thruster from satellite
   - Impossible to verify all 8 thrusters without disassembly

**Hardware Performance with Vertical Calibration**:
- MPC controller **unstable**
- Satellite either over-responds (too much thrust assumed) or under-responds (too little thrust)
- No amount of MPC tuning could achieve stability
- **Conclusion**: Model inaccuracy exceeded what controller could compensate for

#### Iteration 2: In-Situ Calibration Bench (BREAKTHROUGH)

**Design Philosophy**: **Measure thrusters in their exact flight configuration**

**Key Insight**: To get accurate thrust measurements, the thruster must be:
1. Mounted exactly as it will be in flight (same mounting hardware, same position)
2. Measured with higher accuracy than kitchen scale (±0.02g vs ±1g)

**Calibration Bench Design**:

**Mechanical Structure** (3D printed):
1. **Main bracket**: Replaces satellite side panel, uses same mounting holes
2. **Load cell mount**: Attaches 100g load cell to the bracket

**Thrust Measurement Path**:
1. Thruster fires horizontally
2. Remove 3D printed nozzle, replace with **push-to-connect fitting**
3. Silicone tubing connects thruster to a **replica nozzle** with same internal geometry as the nozzle normally mounted to the thruster
4. Replica nozzle mounted to **100g load cell** (±0.02g = ±0.0002N accuracy)
5. Tubing looped and secured to prevent sagging/tension on load cell (confirmed with multiple tests where the load cell value would return to original value before testing)
6. Load cell measures force transmitted to the replica nozzle through the silicone tube

**Critical Design Features**:

1. **Thruster never moved**: Stays in exact flight mounting location
   - Same air supply plumbing
   - Same power supply as in flight

2. **Identical nozzle geometry**: Replica nozzle has identical internal flow path
   - 3D printed with same printer, same settings as the regular nozzles attached to each solenoid
   - Only difference: Mounting for load cell instead of direct satellite mount using threads
   - Preserves airflow characteristics

3. **High-precision load cell**:
   - 100g capacity
   - ±0.02g accuracy (±0.0002N)
   - **More accurate** than kitchen scale
   - Calibrated against known precision weights before use

**Calibration Procedure**:

1. Calibrate load cell
2. Mount bracket to satellite (replaces one side panel)
3. Remove thruster nozzle, install push-to-connect fitting
4. Connect silicone tubing from thruster to replica nozzle on load cell
5. Secure tubing to prevent sagging/pulling on load cell
6. Verify load cell reads zero with thruster off
7. For each thruster:
   - Fire thruster for 10 seconds
   - Record load cell reading
   - Rest 5 seconds
   - Repeat 25 times
8. Repeat for all 8 thrusters twice

#### The Breakthrough: Hardware Finally Works

**Before In-Situ Calibration**:
- MPC controller completely unstable
- Satellite spins unpredictably
- Cannot hold even a single waypoint
- **Control system does not work at all**

**After In-Situ Calibration**:
- MPC controller stable on first test
- Satellite holds waypoints with <5cm error
- Successful shape-following missions
- **Control system works reliably**


#### Why Accurate Calibration is Non-Negotiable for MPC

**MPC Prediction Horizon** (N = 12 steps × 0.06s = 0.72s future):
```
MPC predicts: "If I fire thruster 3 at for 3 steps, satellite will move X and rotate Y"
```

**With Inaccurate Model** (Thrust = 0.85N assumed, actually 0.92N = 8% error):
```
After 0.9s:
- Predicted position: (1.00m, 0.50m)
- Actual position: (1.08m, 0.54m)  ← 8% error propagated
- Position error: 9.4cm
```

**With Accurate Model** (Thrust = 0.92N, correct):
```
After 0.9s:
- Predicted position: (1.00m, 0.50m)
- Actual position: (1.02m, 0.51m)  ← Only sensor noise
- Position error: 2.2cm
```

**MPC Requires Accurate Models**:
- **Model-based** means predictions must match reality
- Errors compound over prediction horizon
- 10% thrust error → 10% position error → wrong control → instability
- **No amount of tuning can fix bad model**

#### Lessons for Model-Based Control

1. **Measure in flight configuration**

2. **Individual component characterization**: Don't assume components are identical
   - The 8 "identical" solenoid valves varied in performance
   - MPC performance improved dramatically when accounting for individual differences

3. **Spend time on system calibration**:
   - Thruster bench: 2 hours to design, with total cost of $30 for components
   - Result: Project went from non-functional to functional
   - **Best time investment in entire project**

4. **Prediction horizon**:
   - Longer MPC horizon → more stringent accuracy requirement
   - Shorter horizon might give less optimal control but is much more forgiving, better suited for non-perfect environments like this lab

**Recommendation**: For any model-based controller (MPC, LQR, etc.), **invest heavily in accurate system identification**. Spend time on calibration before tuning control gains. Bad model + good controller = failure. Good model + mediocre controller = success.

---

## Software Control Algorithm Development

### MPC Formulation Evolution

The MPC formulation went through several iterations before arriving at the final working implementation. This section documents the mathematical evolution and design decisions.

#### State Space Representation

**Final State Vector** (6 states):
```
x = [x_pos, y_pos, theta, x_vel, y_vel, omega]^T

Where:
- x_pos, y_pos: Position in workspace (meters)
- theta: Orientation angle (radians)
- x_vel, y_vel: Linear velocity (m/s)
- omega: Angular velocity (rad/s)
```

**Control Input Vector** (8 binary inputs):
```
u = [u1, u2, u3, u4, u5, u6, u7, u8]^T

Where:
- ui ∈ {0, 1}: Binary on/off for thruster i
- Mixed-Integer Programming (MIP) problem
```

**Why Binary Thrusters?**:
- **Hardware constraint**: Solenoid valves are on/off (no proportional control)
- **Cost**: Proportional valves $200+ each vs. binary solenoid ~$12
- **Complexity**: Binary control simpler mechanically and more reliable
- **MPC challenge**: Binary variables → MIP (harder than continuous optimization)

#### Dynamics Model: Linearization Strategy

**True Nonlinear Dynamics**:
```
ẋ = v_x cos(θ) - v_y sin(θ)
ẏ = v_x sin(θ) + v_y cos(θ)
θ̇ = ω
v̇_x = (1/m) Σ F_i cos(α_i + θ)
v̇_y = (1/m) Σ F_i sin(α_i + θ)
ω̇ = (1/I) Σ F_i × r_i

Where:
- m: Satellite mass (23kg)
- I: Moment of inertia (0.32 kg⋅m²)
- F_i: Thrust force from thruster i
- α_i: Thruster mounting angle
- r_i: Moment arm for thruster i
```

**Linearization Approach**:

MPC requires linear dynamics for efficient solution. We linearize around **current state** each time step (successive linearization).

**Iteration 1**: Global Linearization (FAILED)
- Linearize around θ = 0
- Assumption: Small angle deviations
- **Problem**: Breaks down for large rotations (> 30°)
- **Result**: MPC unstable during turns

**Iteration 2**: Local Linearization (IMPROVED)
- Linearize around current state (x_k, u_k) each time step
- Compute Jacobian A_k, B_k at current state
- **Problem**: Linearization error for multi-step prediction
- **Result**: Stable but suboptimal

**Iteration 3**: Successive Linearization with Warm Start (FINAL)
- Linearize around current state
- Use previous solution as warm start for next solve
- Update linearization each time step (receding horizon)
- **Result**: Stable and near-optimal

**Linearized Dynamics** (computed each time step):
```
x_{k+1} = A_k x_k + B_k u_k

Where:
A_k = I + Δt × ∂f/∂x |_{x_k, u_k}
B_k = Δt × ∂f/∂u |_{x_k, u_k}

Δt = CONTROL_DT
```

#### Cost Function Evolution

**Objective**: Minimize tracking error and control effort

**Iteration 1**: Quadratic Cost (Standard)
```
J = Σ (x_k - x_ref)^T Q (x_k - x_ref) + u_k^T R u_k
```
- **Problem**: Binary u_k makes this nonlinear in binary variables
- Gurobi requires quadratic objective in **continuous** variables

**Iteration 2**: Linearized Cost (Workaround)
```
J = Σ ||x_k - x_ref||_Q² + Σ r_i u_i

Where:
- ||x - x_ref||_Q²: Quadratic state error weighted by Q matrix
- r_i u_i: Linear control cost (works with binary u_i)
```
- **Problem**: Doesn't penalize thruster switching (chattering)

**Iteration 3**: Cost with Switching Penalty (FINAL)
```
J = Σ_{k=0}^{N-1} [ (x_k - x_ref)^T Q (x_k - x_ref) + Σ_i (r_thrust u_{i,k} + r_switch |u_{i,k} - u_{i,k-1}|) ]

Where:
Q = diag([Q_pos, Q_pos, Q_angle, Q_vel, Q_vel, Q_omega])
r_thrust: Cost per thruster activation
r_switch: Cost per thruster state change (on→off or off→on)
```

**Cost Weight Tuning** (Final Values):
```python
Q_POSITION = 1000.0       # Position error penalty (m⁻²)
Q_VELOCITY = 1750.0       # Velocity error penalty (s²/m²)
Q_ANGLE = 1000.0          # Angle error penalty (rad⁻²)
Q_ANGULAR_VELOCITY = 1500.0  # Angular velocity penalty (s²/rad²)
R_THRUST = 1.0            # Thrust usage cost
R_SWITCH = 0.0            # Switching cost (disabled in final tuning)
```

**Tuning Process**:
1. Start with Q_position = 1, all others = 0 → learns to reach position
2. Add Q_velocity → learns to arrive smoothly
3. Add Q_angle → learns to orient correctly
4. Add R_thrust → learns fuel efficiency
5. Add R_switch → eliminates thruster chattering
6. Iterate to balance performance vs. fuel usage

#### Constraints

**State Constraints**:
```python
# Position bounds (6m × 6m workspace)
-3.0 ≤ x_pos ≤ 3.0
-3.0 ≤ y_pos ≤ 3.0

# Velocity limits (safety)
-0.25 ≤ v_x ≤ 0.25  (m/s)
-0.25 ≤ v_y ≤ 0.25
-π/2 ≤ ω ≤ π/2      (rad/s)

# Angle: unconstrained (can spin freely)
```

**Control Constraints**:
```python
u_i ∈ {0, 1}  ∀ i ∈ [1, 8]  # Binary thruster on/off
```

#### Horizon Length Selection

**Key Parameter**: `MPC_PREDICTION_HORIZON = N`

**Trade-offs**:
- **Longer horizon** (N large):
  - (+) Better long-term planning
  - (+) Smoother trajectories
  - (−) More decision variables → longer solve time
  - (−) Linearization error accumulates

- **Shorter horizon** (N small):
  - (+) Faster solves
  - (+) More accurate linearization (less future prediction)
  - (+) Better suited for non-perfect environment like labs
  - (−) Myopic behavior
  - (−) Poor anticipation of future obstacles

**Tested Values**:
| N | Solve Time | Performance | Selected? |
|---|------------|-------------|-----------|
| 5 | 3-8ms | Poor | No |
| 10 | 5-20ms | Adequate | No |
| 12 | 5-40ms | Good | **Yes** |
| 20 | 10-50ms | Adequate | No, does not do well with unpredicted external forces |
| 25 | 20-60ms | Poor | No (exceeds deadline) |

**Selected**: N = 12 (0.72 seconds lookahead)
- Adequate performance
- Meets real-time deadline (<50ms)
- Good balance

**Control Horizon**: `MPC_CONTROL_HORIZON = M = 12`
- Only first M steps have independent control
- Last (N - M) steps use terminal constraint or repeated control


#### Gurobi Solver Configuration: The Performance Breakthrough

This was the **second critical software breakthrough** (after accurate thruster calibration).

**The Problem**: Solver Timing Violations

**Initial Performance** (Default Gurobi settings):
- Solve time: 50-180ms
- Time budget: 50ms (MPC_SOLVER_TIME_LIMIT)
- **Result**: Multiple solves exceeded deadline so control intervals had to be extended

**Root Cause**: Didn't understand the solver parameters

I could formulate the MPC problem correctly mathematically, but didn't understand **how Gurobi solves MIP problems** or **what parameters control the solve strategy**.

**Key Realization**: **MPC doesn't need optimal solutions, just good feasible solutions quickly**

**The MPC Advantage**:
- Receding horizon: Next time step replans anyway
- 1% suboptimal solution arriving in 10ms >> optimal solution arriving in 60ms
- **Speed more important than optimality** for real-time control

**Solver Parameter Tuning** (The Breakthrough):

```python
# Real-time MPC settings (FINAL)
self.model.setParam("MIPGap", 1e-2)              # 1% optimality gap
self.model.setParam("MIPFocus", 1)               # Focus on finding solutions
self.model.setParam("Heuristics", 0.01)          # Minimal heuristics (1% time)
self.model.setParam("Cuts", 1)                   # Conservative cuts
self.model.setParam("Presolve", 2)               # Aggressive presolve
self.model.setParam("Threads", 1)                # Single-threaded
self.model.setParam("ImproveStartTime", 0.6*TimeLimit)  # Improve after 60% time
self.model.setParam("TimeLimit", 0.050)          # Hard 50ms deadline
```

**Detailed Parameter Analysis**:

**1. MIPGap = 1e-2 (THE MOST CRITICAL SETTING)**

What it does:
- Gurobi stops when: (Upper Bound - Lower Bound) / Upper Bound ≤ MIPGap
- Upper bound: Best feasible solution found
- Lower bound: Proven lower limit on objective value
- **1% gap = solution within 1% of proven optimal**

Why it helps:
- Default MIPGap = 1e-4 (0.01%) → Gurobi proves very tight optimality → slow
- Relaxed to 1e-2 (1%) → Gurobi stops much earlier → fast
- For MPC: 1% suboptimal is acceptable in practice

Performance impact:
- **Before** (MIPGap = 1e-4): Mean solve time 85ms
- **After** (MIPGap = 1e-2): Mean solve time 5ms

**Selected**: 1e-2 (1%) - best balance

**2. MIPFocus = 1 (Focus on Feasibility)**

What it does:
- Controls Gurobi's search strategy
- Options:
  - 0 (Default): Balanced
  - 1: Emphasize **finding feasible solutions**
  - 2: Emphasize **proving optimality**
  - 3: Focus on **improving best bound**

Why it helps:
- MIPFocus=1 tells Gurobi: "Find solutions fast, don't spend time proving they're optimal"
- Complements MIPGap setting
- Gurobi adjusts heuristics and branching to find solutions quickly

**3. Heuristics = 0.01 (Minimal Heuristics)**

What it does:
- Controls time spent on heuristic algorithms (fast approximations)
- Range: 0.0 (none) to 1.0 (lots)
- 0.01 = spend only 1% of time on heuristics

Why it helps:
- For small problems (N=12, 8 thrusters = 96 binary variables), heuristics have overhead
- Branch-and-cut is more efficient for structured MPC problems
- Heuristics more valuable for large, unstructured problems (1000+ variables)

**4. Cuts = 1 (Conservative Cuts)**

What it does:
- Controls cutting plane generation (linear inequalities that tighten LP relaxation)
- Options:
  - 0: No cuts
  - 1: Conservative (few cuts)
  - 2: Moderate (default)
  - 3: Aggressive (many cuts)

Why it helps:
- Cuts help prune branches, but generation takes time
- Conservative cuts: Fast generation, reasonable pruning
- For real-time: Prefer fast solve over tight bounds

Performance impact:
- Cuts=2 (default): Slightly slower
- Cuts=1: Marginally faster (5%)
- Cuts=0: Faster solve, worse solution quality

**5. Presolve = 2 (Aggressive Presolve)**

What it does:
- Preprocessing before solving (removes redundant variables/constraints)
- Options:
  - 0: No presolve
  - 1: Conservative
  - 2: Aggressive

Why it helps:
- MPC formulation has redundant constraints (dynamics linearization creates dependencies)
- Aggressive presolve removes redundancy → smaller problem → faster solve


**6. Threads = 1 (Single-Threaded)**

What it does:
- Controls number of parallel threads
- Default: -1 (auto, often 4-8 threads)

Why it helps:
- Small problems (96 binary variables) solve faster single-threaded
- Multi-threading overhead:
  - Thread creation/synchronization
  - Cache contention between threads
  - Load balancing overhead

**For large problems (1000+ variables)**: Multi-threading helps
**For small MPC problems**: Single-threaded is faster

**7. ImproveStartTime = 0.6 × TimeLimit (Timing Strategy)**

What it does:
- First 60% of time: Focus on finding feasible solutions
- Last 40% of time: Improve solution quality

Why it helps:
- Ensures Gurobi finds **some** solution quickly
- Without this: Might spend all 50ms on one hard branch, return nothing
- With this: Find solution in first 30ms, polish it for last 20ms

**8. TimeLimit = 0.050 (Hard Deadline)**

What it does:
- Gurobi stops **immediately** when time limit reached
- Returns best solution found (or error if no solution)

Why it helps:
- Real-time guarantee: Solver never blocks beyond deadline
- MPC can handle suboptimal solution; cannot handle missed deadline

**Lesson**: **Learn your solver's parameters**. The difference between default Gurobi and tuned Gurobi was the difference between 180ms control intervals and 60ms control intervals. Mathematical formulation is only half the battle; solver configuration is equally important for real-time applications.

#### Warm Start Strategy

**Concept**: Use previous solution as starting point for next solve

**Implementation**:
```python
# After solve at time k:
u_prev = u_optimal[k]

# At time k+1:
# Shift previous solution as initial guess
u_init = [u_prev[1], u_prev[2], ..., u_prev[N-1], u_prev[N-1]]
model.setAttr("Start", u_vars, u_init)
```

**Why it helps**:
- Successive MPC problems are similar (small state change)
- Previous solution likely close to new optimal
- Gurobi starts branch-and-bound from good initial point → faster convergence

**Caveat**: Large disturbances or target changes → warm start less helpful

### Software Architecture Decisions

#### Modular Configuration System

**Problem**: Parameters scattered across code, hard to tune

**Solution**: Centralized `config/` package with type-safe dataclasses

**Structure**:
```
config/
├── physics.py       # Mass, inertia, thruster forces
├── timing.py        # Control rates, stabilization times
├── mpc_params.py    # Horizons, weights, solver settings
├── constants.py     # Ports, paths, UI settings
├── mission_state.py # Runtime mission parameters
└── obstacles.py     # Obstacle definitions
```

**Benefits**:
1. **Single source of truth**: All parameters in one place
2. **Type safety**: Dataclasses catch type errors at import
3. **Documentation**: Parameters documented with units
4. **Validation**: Automatic sanity checking
5. **Easy tuning**: Change one file, entire system updates

**Example**:
```python
# config/mpc_params.py
MPC_PREDICTION_HORIZON = 12  # Time steps (0.72s lookahead)
MPC_CONTROL_HORIZON = 12     # Control steps
Q_POSITION = 1000.0          # Position error penalty (m⁻²)

# Anywhere in code:
from config import SatelliteConfig
N = SatelliteConfig.MPC_PREDICTION_HORIZON
```

**Lesson**: **Invest in configuration infrastructure early**. Tuning MPC requires changing many parameters frequently. Centralized config paid off immediately.

#### Simulation-Hardware Parity

**Design Principle**: **Same MPC code for simulation and hardware**

**Implementation**:
- `mpc.py`: Core MPC solver (used by both)
- `model.py`: Satellite physics (used by both)
- `simulation.py`: Physics-based simulation + MPC
- `real.py`: Hardware interface + MPC (same MPC code)

**Benefits**:
1. **Validation**: Simulation predicts hardware behavior
2. **Safety**: Test in simulation before hardware
3. **Debugging**: If simulation works but hardware doesn't → hardware issue (not control algorithm)
4. **Development speed**: Develop control without hardware access

**Critical Insight**: When hardware failed initially, **simulation worked perfectly**. This immediately indicated: MPC algorithm is correct, problem is in hardware characterization (thrust calibration) or communication (latency).

**Without simulation-hardware parity**: Would have suspected MPC algorithm bug, wasted time debugging correct code.

**Lesson**: **Build high-fidelity simulation before hardware**. Simulation is not just for testing; it's a **debugging tool** that isolates software vs. hardware issues.

---

## Critical Breakthroughs (The Big Three)

Three breakthroughs were **absolutely essential** for system functionality. Without any one of them, the system would not work.

### Breakthrough 1: In-Situ Thruster Calibration

**Problem**: Hardware completely unstable despite perfect simulation

**Root Cause**: Thrust model inaccuracy

**Solution**: In-situ calibration bench measuring thrusters in flight configuration

**Impact**:
- Before: System unable to stabilize and reach targets
- After: System stable and stabilizes on targets in first try without oscillations
- **Enabled**: Entire project (without this, system doesn't work at all)


### Breakthrough 2: ESP-NOW Communication Protocol

**Problem**: WiFi latency violated MPC assumptions, caused instability

**Root Cause**: Standard WiFi protocol overhead incompatible with MPC timing requirements

**Solution**: Switched to ESP-NOW (ultra-low-latency peer-to-peer protocol)

**Impact**:
- Before: MPC predictions stale by time commands arrive → unstable
- After: Commands arrive within 10ms → predictions valid → stable
- **Enabled**: Closed-loop hardware control


### Breakthrough 3: Gurobi MIPGap Tuning

**Problem**: MPC solver too slow

**Root Cause**: Default Gurobi settings prioritize optimality over speed

**Solution**: Relaxed MIPGap to 1e-2 (1%), focused solver on finding feasible solutions fast

**Impact**:
- Before: 180ms control intervals → less precise control → instability
- After: 60ms control intervals → precise control → stable
- **Enabled**: Real-time MPC in hardware

### Why These Three Together

**All three required**:
1. **Accurate model** (thrust calibration) → MPC predictions valid
2. **Low latency** (ESP-NOW) → Predictions still valid when commands arrive
3. **Fast solver** (Gurobi tuning) → Commands computed in shorter amount of time for more precise control

**Without any one**:
- No thrust calibration → Predictions wrong → Control fails
- No ESP-NOW → Commands arrive too late → Control fails
- No solver tuning → Command intervals larger → less precise control

**Sequential Discovery**:
- Fixed communication first (ESP-NOW) → Still unstable
- Fixed thrust calibration next → Much better, but long control intervals
- Fixed solver tuning last → Fully stable system with more precise control

**Lesson**: **Complex systems require multiple components working correctly**. Debugging is process of elimination, fixing bottlenecks sequentially until system works.

---

## Recommendations for Future Work

Based on lessons learned, here are recommendations for future researchers extending this work.

### Hardware Improvements

**1. Proportional Thrusters**
- **Current**: Binary on/off solenoids
- **Proposed**: Proportional valves with variable flow rate
- **Benefits**:
  - Finer control authority
  - Reduced chattering
  - Better fuel efficiency
- **Challenges**:
  - Cost more and might be less reliable
  - MPC formulation changes (continuous control instead of binary)
  - Characterization more complex

**2. Onboard MPC Computation**
- **Current**: Ground-station MPC, wireless command uplink
- **Proposed**: Embedded MPC on satellite microcontroller
- **Benefits**:
  - No communication latency
  - More realistic for actual satellite
  - Simpler system architecture
- **Challenges**:
  - Embedded solver needed (Gurobi won't run on ESP32)
  - Limited computational power (ESP32 much slower than laptop)
  - May need simpler MPC formulation (shorter horizon)

**3. Redundant Thrusters**
- **Current**: 8 thrusters, no redundancy
- **Proposed**: 12 thrusters (50% redundancy)
- **Benefits**:
  - Fault tolerance (can lose 4 thrusters and remain controllable)
  - Over-actuated system enables fuel optimization
- **Challenges**:
  - Increased complexity
  - MPC problem size larger (12 binary variables vs. 8)
  - More weight, more cost

**4. Improved Air Bearing Design**
- **Current**: 3 air bearings, manual balancing
- **Proposed**: Active balancing system
- **Implementation**: Motorized air bearing positioning
- **Benefits**:
  - Automatic COM correction
  - Adaptive to payload changes
  - Easier to balance
- **Challenges**:
  - Complexity, cost, weight

### Software/Control Improvements

**1. Nonlinear MPC**
- **Current**: Successive linearization
- **Proposed**: Full nonlinear optimization
- **Benefits**:
  - More accurate predictions (no linearization error)
  - Better performance for large rotations
- **Challenges**:
  - Nonlinear MIP extremely hard to solve
  - Unlikely to meet real-time deadline with current solvers
  - Requires advanced solver

**2. Multi-Agent Coordination**
- **Current**: Single satellite
- **Proposed**: Multiple satellites with coordinated MPC
- **Implementation**: Distributed MPC with communication
- **Benefits**:
  - Formation flying
  - Cooperative missions
  - Collision avoidance
- **Challenges**:
  - Communication between satellites
  - Coupled optimization problem
  - Scalability

---

## Conclusion

This project successfully demonstrated **Model Predictive Control for satellite thruster control** using a hardware air-bearing testbed. The journey from concept to functional system required overcoming significant challenges in hardware characterization, communication latency, and real-time optimization.

**Three critical breakthroughs** enabled system functionality:
1. In-situ thruster calibration for accurate model
2. ESP-NOW communication for low latency
3. Gurobi solver tuning for real-time performance

**Key Lessons**:
- **Model accuracy is non-negotiable** for model-based control
- **Communication latency is a hard constraint** for distributed MPC
- **Solver configuration is as important as mathematical formulation** for real-time optimization
- **Build high-fidelity simulation first** before hardware
- **Document failures and iterations** (most learning comes from what didn't work)


**Final Recommendation**: If you're building a similar system, **read this document first**. Learn from my failures. The breakthroughs documented here represent months of debugging—understanding them will save you significant time and effort.
