# Chapter 4.2.2 — State Representation

The simulation and control software represent the air-bearing satellite with a six-dimensional state vector that tracks planar position, translational velocity, yaw, and yaw rate. Expressed in the world (OptiTrack) reference frame, the continuous-time state is

$$
\mathbf{x}(t)=
\begin{bmatrix}
p_x(t) \\
p_y(t) \\
v_x(t) \\
v_y(t) \\
\theta(t) \\
\omega(t)
\end{bmatrix},
$$

where:

- $p_x(t), p_y(t)$ are the satellite’s x–y coordinates in meters.
- $v_x(t), v_y(t)$ are linear velocities in meters per second (derived from the same world-frame axes).
- $\theta(t)$ is the in-plane yaw angle in radians, measured counter-clockwise from the positive x-axis.
- $\omega(t)$ is the angular velocity about the vertical axis in radians per second.

This state completely specifies the vehicle’s pose and motion on the air table. Given applied forces and torques, it allows the continuous dynamics to be propagated and provides the information required by the predictive controller.

This representation fully captures the planar configuration of the satellite at every instant and, when coupled with known forces and torques, permits forward propagation. For bookkeeping convenience the numerical integrator treats the state as `[p_x, p_y, v_x, v_y, \theta, \omega]^T`, whereas the MPC derivation reorders it to `[p_x, p_y, \theta, v_x, v_y, \omega]^T` so that position and orientation precede velocities. Both permutations contain identical physical content; the difference merely reflects organizational preferences in subsequent derivations.

# Chapter 4.2.3 — Coordinate Frame Management

# Chapter 4.2.3 — Coordinate Frame Management

Each thruster is rigidly attached to the body, so its force direction is naturally expressed in the body-fixed frame. The equations of motion, however, are integrated in the inertial frame. Consequently, every thruster force must be rotated into world coordinates before it contributes to the translational dynamics. Let $\theta$ denote the current yaw angle. The body-to-world rotation used throughout the derivation is

$$
\mathbf{R}(\theta) =
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix},
$$
which we denote as Eq. (4.2). Given a thruster magnitude $F_i$ and body-frame unit direction $\hat{\mathbf{d}}_i$, the body-frame force is

$$
\mathbf{F}^{\text{body}}_i = F_i \hat{\mathbf{d}}_i, \tag{4.3}
$$
and its world-frame counterpart is

$$
\mathbf{F}^{\text{world}}_i = \mathbf{R}(\theta)\,\mathbf{F}^{\text{body}}_i. \tag{4.4}
$$
The transformation therefore proceeds as follows for each active thruster:

1. Retrieve its calibrated magnitude and body-frame unit direction.
2. Form $\mathbf{F}_i^{\text{body}}$ using Eq. (4.3).
3. Rotate into world coordinates using Eq. (4.4).
4. Sum all $\mathbf{F}^{\text{world}}_i$ to obtain the net force and torque acting on the satellite during that integration step.

Because $\theta$ evolves continuously, this transformation is recomputed at every integration step. Accurate handling of the rotation is essential to keep simulated motion aligned with the vehicle’s actual orientation.

# Chapter 4.2.4 — Force and Torque Computation

All eight thrusters contribute simultaneously to translation and rotation. Each thruster command is modeled as a binary decision variable $u_i \in \{0,1\}$ (on/off), multiplied by its calibrated magnitude $F_i$ and body-frame unit direction $\hat{\mathbf{d}}_i$, then rotated into world coordinates. The total external force applied to the satellite is therefore

$$
\mathbf{F}_{\text{net}} = \sum_{i=1}^{8} u_i \left[\mathbf{R}(\theta)\,(F_i \hat{\mathbf{d}}_i)\right], \tag{4.5}
$$

The torque about the true center of mass (COM) is computed in the body frame before rotation:

$$
\tau_{\text{net}} = \sum_{i=1}^{8} u_i \left(r_{x,i}\,F_{y,i} - r_{y,i}\,F_{x,i}\right), \tag{4.6}
$$

where $(r_{x,i}, r_{y,i})$ is thruster $i$’s position relative to the measured COM and $(F_{x,i}, F_{y,i})$ are the body-frame components of $F_i \hat{\mathbf{d}}_i$. Using the COM—not the geometric center—ensures that a “pure” translation command does not unintentionally create rotation.

# Chapter 4.2.5 — Integration of Equations of Motion

State propagation is performed with explicit Euler integration using a timestep of $\Delta t = 5$ ms. Translational motion follows Newton’s second law:

$$
\mathbf{a}(t) = \frac{\mathbf{F}_{\text{net}}(t)}{m}, \tag{4.7}
$$
$$
\mathbf{v}(t+\Delta t) = \mathbf{v}(t) + \mathbf{a}(t)\,\Delta t, \tag{4.8}
$$
$$
\mathbf{p}(t+\Delta t) = \mathbf{p}(t) + \mathbf{v}(t)\,\Delta t, \tag{4.9}
$$

with total mass $m = 23.09$ kg. Rotational motion is integrated in the same fashion using the planar moment of inertia $I = 0.324$ kg·m²:

$$
\alpha(t) = \frac{\tau_{\text{net}}(t)}{I}, \tag{4.10}
$$
$$
\omega(t+\Delta t) = \omega(t) + \alpha(t)\,\Delta t, \qquad
\theta(t+\Delta t) = \theta(t) + \omega(t)\,\Delta t. \tag{4.11}
$$

After every step, $\theta$ is wrapped to the interval $[-\pi, \pi]$ via

$$
\theta_{\text{normalized}} = \operatorname{atan2}(\sin\theta, \cos\theta), \tag{4.12}
$$

which ensures angular errors are always measured along the shorter rotation direction. The same wrapping operation is used in the Chapter 5 controller to prevent unnecessary long rotations.

# Chapter 4.3.2 — Idealized and Realistic Physics Modes

The digital twin exposes two physics modes to support controller development and hardware rehearsal.

## Idealized Mode

Used for algorithm bring-up, this mode removes most non-idealities:

- **No damping.** Linear and rotational drag terms are disabled.
- **Instant thrusters.** Commands immediately reach full magnitude; no valve delay or ramp.
- **Noise-free sensing.** States passed to the controller are identical to ground truth.
- **No disturbances.** Random forces/torques are disabled.

Idealized runs disable aerodynamic drag, measurement noise, thruster valve dynamics, and exogenous disturbances, providing a clean environment for controller tuning.

## Realistic Mode

Realistic runs incorporate the phenomena observed on the air-bearing testbed:

- **Linear drag:** $\mathbf{F}_{\text{drag}} = -b_{\text{linear}}\,\mathbf{v}$ with $b_{\text{linear}} = 1.8$ N · s/m.
- **Rotational drag:** $\tau_{\text{drag}} = -b_{\text{rotational}}\,\omega$ with $b_{\text{rotational}} = 0.3$ N · m · s/rad.
- **Measurement noise:** Small Gaussian perturbations added to position, velocity, angle, and angular rate to mimic telemetry errors.
- **Thruster dynamics:** Valve open/close delays of 40 ms and ramp-up time of 10 ms, producing finite rise and fall times.
- **Random disturbances:** Low-level force/torque jitter (0.4 N and 0.1 N·m standard deviation, respectively) to approximate air currents and floor irregularities.

These additions make the simulator closely mimic hardware so missions can be evaluated under realistic conditions before deployment.

# Chapter 4.4 — Software Architecture and Implementation

The simulation stack and the real-time hardware controller share the same software modules. This section summarizes the pieces that keep them aligned.

## 4.4.1 Central Configuration Layer

All global parameters are maintained in a single configuration layer that exposes physical properties (mass, planar inertia, thruster geometry), controller settings (prediction/control horizons, solver budget, state and velocity bounds, cost weights), mission metadata (mission modes, waypoint spacing, timing thresholds), and timing constants (control-loop period, stabilization windows). Using a shared configuration ensures that both simulation and hardware tests operate with identical assumptions.

## 4.4.2 Model Predictive Controller

The MPC described in Chapter 5 executes every 60 ms. Each cycle it reorders the current state, linearizes the dynamics about the latest attitude, updates the reference trajectory, and refreshes the quadratic cost, including the adaptive velocity weighting near the goal. A persistent Gurobi MIQP model with binary thruster variables is then updated, warm-started from the previous solution, and solved within a 50 ms time limit. The output consists of the 8-element thruster command vector and diagnostic information such as solver status and solve time. Identical logic runs in simulation and hardware, guaranteeing comparable behavior across platforms.

## 4.4.3 Mission Layer

Mission generation provides the desired trajectory for the controller. Supported behaviors include point-to-point moves, waypoint sequences, and contour or profile following derived from DXF outlines. For profile-following missions, the path is imported, offset by the specified standoff distance, and parameterized by the desired traversal speed to produce the time-varying reference $\mathbf{x}_{\text{ref}}(t)$. Each control cycle retrieves the appropriate target pose (and velocity, when needed), which the MPC then tracks.

## 4.4.4 Simulation Control Loop

Closed-loop missions are rehearsed entirely in software through two coupled loops: a 5 ms physics loop that applies the rigid-body dynamics (force/torque accumulation, Euler integration, and angle wrapping) and a 60 ms control loop that invokes the MPC. Each control interval executes the following sequence:

1. Acquire the current simulated state from the physics loop.
2. Request the current mission reference.
3. Pass state and reference to the MPC and solve for the next thruster command.
4. Apply the command to the simulated plant, including valve delays and ramp-up when realistic physics are enabled, and advance the physics loop accordingly.
5. Log the state/reference histories, thruster commands, and solver statistics.

The environment can operate in either the idealized or realistic mode described in Section 4.3.2. All data are written to a structured log format shared with the hardware controller, allowing identical plots and animations to be generated for both simulated and physical runs.

## 4.4.5 Hardware Control Loop

The real-time controller mirrors the same sequence while interfacing with the physical satellite. Each 60 ms control cycle:

1. Acquires the current pose and velocity estimate from the external motion-capture system.
2. Requests the current mission reference.
3. Passes the measured state and reference to the MPC and solves for the next thruster command.
4. Receives the 8-bit command vector from the solver.
5. Checks workspace, linear-velocity, and angular-rate limits for safety.
6. Sends the validated command to the solenoid valve controller and applies it to the hardware.
7. Logs the full cycle (state, reference, command, and solver timing) in the same format used by the simulation.

Because both controllers call the same MPC with identical parameters, the only difference between a simulated run and a hardware run is the plant being controlled—numerically integrated dynamics versus the air-bearing system. This symmetry enables the direct comparison presented in Chapter 6.

## 4.4.6 Visualization and Data Products

After each run—both simulation and hardware—data at every control step are saved to a structured CSV file. These records are then used to automatically generate plots and animations of the mission that just concluded (trajectory traces, position and heading error histories, velocity magnitude plots, per-thruster duty cycles, cumulative thrust usage, solver time histories, and MP4 animations of the satellite with commanded thrusters highlighted). When desired, the same tools can overlay two runs, create a statistics table that compares the missions side by side, and provide overlaid plots and animations.

The logging framework standardizes timestamps and ensures that state, reference, control, and solver metrics are recorded every cycle, allowing the visualization tools to operate identically on simulated and real data. This shared tooling is what enables the cross-run comparisons presented in Chapter 6.

## 4.4.7 Summary of Software Flow

Collectively, the software layers operate as follows:

- The mission layer defines **what to do**, producing target poses/velocities over time.
- The MPC decides **how to do it**, selecting thruster sequences that satisfy constraints.
- The simulation environment or the hardware controller applies the commands—either to the virtual physics model or to the physical solenoid valves.
- The shared configuration keeps both paths on identical parameters, and the visualization/logging tools convert results into plots and animations for analysis.

This structure lets every mission be rehearsed and tuned in simulation before touching hardware, and it enables one-to-one comparisons between predicted and observed behavior (Chapter 6).

# Chapter 5.2.2 — Prediction and Control Horizons

The MPC formulation uses two distinct horizons that serve complementary roles while sharing the same discretization interval $\Delta t = 0.06$ s (the 16.67 Hz control rate of the hardware).

**Prediction horizon ($N$).** This horizon determines how many future state transitions are propagated inside the optimizer. The implementation uses $N = 15$ steps, corresponding to a 0.9 s prediction window. The longer window allows the controller to anticipate how current actions influence future position and orientation, smooth out trajectory planning, and enforce state constraints further into the future. Increasing $N$ generally improves optimality because more future information is incorporated, but it also enlarges the state and constraint sets. Conversely, a shorter $N$ reduces computational burden and can react faster to the random external forces that arise in a non-ideal laboratory environment, though it risks overly short-sighted decisions. The 15-step value was selected as the best compromise between lookahead, robustness to disturbances, and solve time in testing.

**Control horizon ($M$).** This horizon specifies how many control inputs are explicitly optimized. The implementation uses $M = 12$, so the first 12 steps (0.72 s) of the prediction horizon have independent thruster decisions, while inputs for steps 13–15 are held equal to the value from step 12. Limiting $M$ provides most of the responsiveness benefits of a long prediction horizon while reducing the number of binary variables (and therefore the size of the MIQP). The near-term commands within $M$ dominate the system’s response, so optimizing a multi-step sequence—even though only the first move is applied—yields a realistic predicted trajectory that respects future constraints and disturbances. Fixing the tail at the last control value then keeps the computational burden manageable without sacrificing the quality of the first control decision.

**Timestep $\Delta t$.** Both horizons share the same discretization interval. Selecting $\Delta t = 0.06$ s balances several factors: it is fast enough to capture the satellite’s dynamics and react to disturbances, yet slow enough that the solver can reliably meet the 50 ms time limit. Empirical testing showed that this cadence provides good control performance while maintaining computational feasibility on the target hardware.

# Chapter 5.3 — System Dynamics Model

## 5.3.1 State-Space Representation

For the MPC we use a discrete-time linear state-space model describing how the satellite evolves between control intervals. The state vector is

$$
\mathbf{x}_k =
\begin{bmatrix}
p_{x,k} \\
p_{y,k} \\
\theta_k \\
v_{x,k} \\
v_{y,k} \\
\omega_k
\end{bmatrix},
$$

where $p_x, p_y$ are world-frame positions, $\theta$ is yaw (radians), $v_x, v_y$ are world-frame velocities, and $\omega$ is angular velocity (rad/s). The control vector contains the eight binary thruster commands,

$$
\mathbf{u}_k =
\begin{bmatrix}
u_{1,k} \\
u_{2,k} \\
\vdots \\
u_{8,k}
\end{bmatrix}, \qquad u_{i,k} \in \{0,1\},
$$

and the dynamics follow the linearized state-space form

$$
\mathbf{x}_{k+1} = \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k.
$$

Subscript $k$ highlights that $\mathbf{A}_k$ and $\mathbf{B}_k$ are regenerated every cycle by linearizing the nonlinear dynamics about the latest pose.

## 5.3.2 Linearization of Nonlinear Dynamics

The dynamics are nonlinear because each thruster’s body-frame force must be rotated through the current yaw angle before it contributes to the translational equations, and these forces couple with the torque balance through the COM offset. Solving the full nonlinear problem with binary controls at every control instant would be too slow for real-time operation, so we approximate the dynamics with a first-order expansion around the current operating point. This linearization captures how small changes in state and input influence the next state and provides a manageable mixed-integer quadratic program while remaining accurate over the short prediction window.

### State Transition Matrix

The discrete-time state transition matrix implements forward Euler in the absence of control inputs:

$$
\mathbf{A}_k =
\begin{bmatrix}
1 & 0 & 0 & \Delta t & 0 & 0 \\
0 & 1 & 0 & 0 & \Delta t & 0 \\
0 & 0 & 1 & 0 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix},
$$
with $\Delta t = 0.06$ s. Positions integrate velocities, and yaw integrates angular velocity.

### Control Input Matrix

For each thruster $i$, the body-frame force is $ \mathbf{F}^{\text{body}}_i = F_i \hat{\mathbf{d}}_i$. Rotating into world coordinates uses

$$
\mathbf{F}^{\text{world}}_i = \mathbf{R}(\theta_k)\,\mathbf{F}^{\text{body}}_i, \qquad
\mathbf{R}(\theta_k) = \begin{bmatrix}\cos\theta_k & -\sin\theta_k \\ \sin\theta_k & \cos\theta_k\end{bmatrix},
$$
and the torque about the COM is $\tau_i = r_{x,i} F^{\text{body}}_{i,y} - r_{y,i} F^{\text{body}}_{i,x}$. The torque expression uses the body-frame components because the lever arm $[r_{x,i}, r_{y,i}]$ is also defined in the body frame; evaluating the cross product in a single coordinate frame ensures consistency. The resulting input matrix is

$$
\mathbf{B}_k =
\begin{bmatrix}
\mathbf{0}_{3\times8} \\
\frac{\Delta t}{m}\,[f^{\text{world}}_{1x} \cdots f^{\text{world}}_{8x}] \\
\frac{\Delta t}{m}\,[f^{\text{world}}_{1y} \cdots f^{\text{world}}_{8y}] \\
\frac{\Delta t}{I}\,[\tau_1 \cdots \tau_8]
\end{bmatrix},
$$

### Linearization Strategy

- Linearization occurs once per control step using the current $\mathbf{x}_k$; the same $A_k, B_k$ are used across the entire 15-step prediction horizon.
- Successive linearization keeps the MIQP compact while capturing the dominant effect of attitude-dependent thrust directions.

## 5.3.3 Physical Parameters

The linearized model uses the calibrated parameters summarized in Table 3. All values originate from the air-bearing mass and inertia measurements described in Chapter 3.

| Parameter      | Symbol | Value (SI)                 | Description                                      |
| -------------- | ------ | -------------------------- | ------------------------------------------------ |
| Mass           | $m$    | 23.090 kg                  | Total satellite mass (sum of three air bearings) |
| Moment of inertia | $I$ | 0.324 kg·m²               | Planar inertia about the vertical axis           |
| Satellite size | —      | 0.290 m                    | Side length of the square chassis                |
| COM offset     | —      | (0.00556 m, 0.00078 m)     | Offset from geometric center                     |
| Time step      | $\Delta t$ | 0.06 s                 | MPC discretization interval                      |

Thruster geometry and forces (Table 4):

| Thruster | Position $(r_x,r_y)$ [m] | Direction $(d_x,d_y)$ | Force $F_i$ [N] |
| -------- | ----------------------- | ---------------------- | --------------- |
| 1        | (0.145, 0.060)          | (−1, 0)                | 0.441           |
| 2        | (0.145, −0.060)         | (−1, 0)                | 0.431           |
| 3        | (0.060, −0.145)         | (0, 1)                 | 0.428           |
| 4        | (−0.060, −0.145)        | (0, 1)                 | 0.438           |
| 5        | (−0.145, −0.060)        | (1, 0)                 | 0.469           |
| 6        | (−0.145, 0.060)         | (1, 0)                 | 0.447           |
| 7        | (−0.060, 0.145)         | (0, −1)                | 0.467           |
| 8        | (0.060, 0.145)          | (0, −1)                | 0.484           |

Small differences in the fourth decimal place reflect the most recent thrust calibrations. These numbers feed directly into the $B_k$ construction discussed above.

# Chapter 5.4 — Cost Function Design

The MPC objective balances three goals: track the reference trajectory, minimize control effort, and limit thruster switching. The tracking term dominates.

## 5.4.1 Quadratic Tracking Cost

The primary objective penalizes deviation from the desired trajectory:

$$
J_{\text{tracking}} = \sum_{k=1}^{N} (\mathbf{x}_k - \mathbf{x}_{\text{ref}})^T \mathbf{Q} (\mathbf{x}_k - \mathbf{x}_{\text{ref}}),
$$

with diagonal weights

$$
\mathbf{Q} = \operatorname{diag}(Q_{p_x}, Q_{p_y}, Q_\theta, Q_{v_x}, Q_{v_y}, Q_\omega).
$$

The present tuning employs

- $Q_{p_x} = Q_{p_y} = 1000$ — prioritize position accuracy.
- $Q_\theta = 1000$ — enforce heading alignment.
- $Q_{v_x} = Q_{v_y} = 1750$ — damp translational motion.
- $Q_\omega = 1500$ — limit spin rates.

### Adaptive Velocity Damping

Near the target, velocity weights are boosted to suppress overshoot. When the position error is below 0.25 m and the speed exceeds 0.03 m/s, the controller scales $Q_{v_x}$ and $Q_{v_y}$ up to $\min(3 Q_{v_x}, 1000)$:

$$
Q^{\text{adaptive}}_{v_x} = Q^{\text{adaptive}}_{v_y} =
\begin{cases}
\min(3 Q_{v_x}, 1000) & \text{if } \lVert \mathbf{p}_k - \mathbf{p}_{\text{ref}} \rVert < 0.25 \text{ m and } \lVert \mathbf{v}_k \rVert > 0.03 \text{ m/s}, \\
Q_{v_x} & \text{otherwise}.
\end{cases}
$$

This strategy improves settling performance without slowing the long-distance approach.

## 5.4.2 Control Effort Cost

To discourage unnecessary thruster firings and conserve air supply, we penalize the control effort:

$$
J_{\text{control}} = \sum_{k=0}^{M-1} \mathbf{u}_k^T \mathbf{R} \mathbf{u}_k,
$$

where $\mathbf{R} = R_{\text{thrust}} \mathbf{I}$ with $R_{\text{thrust}} = 1.0$. Because the inputs are binary, $u_i^2 = u_i$, so this term effectively counts the number of active thrusters each step. The relatively small weight ensures tracking accuracy remains the dominant objective, but it still nudges the optimizer toward solutions that achieve the goal with fewer firings when multiple options exist.

## 5.4.3 Switching Penalty

Rapid on/off chatter accelerates valve wear, so the MPC framework supports an optional switching penalty:

$$
J_{\text{switch}} = R_{\text{switch}} \sum_{k=0}^{M-1} \sum_{i=1}^{8} |u_{k,i} - u_{k-1,i}|.
$$

Currently $R_{\text{switch}} = 0$ (disabled) because hardware testing has not shown excessive chattering. However, the MILP already contains the auxiliary binary variables needed to linearize the absolute value. If future tests reveal a need to smooth transitions, setting $R_{\text{switch}}$ to a small positive value would encourage the optimizer to keep thruster states consistent across consecutive time steps.

## 5.4.4 Combined Objective Function

The full cost is the sum

$$
J_{\text{total}} = J_{\text{tracking}} + J_{\text{control}} + J_{\text{switch}},
$$

with $R_{\text{switch}} = 0$ in the current deployment. The weighting was tuned iteratively in simulation to achieve:

- Fast initial response to large position errors.
- Smooth, well-damped approach trajectories.
- Minimal overshoot and quick settling.
- Reasonable propellant usage (avoid unnecessary thruster firings).

This multi-objective formulation balances accuracy against resource consumption while keeping the option open to penalize switching if hardware data justifies it.

# Chapter 5.5 — Constraints

## 5.5.1 Dynamics Constraints

The optimizer enforces the linearized dynamics across the horizon:

$$
\mathbf{x}_{k+1} = \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k \quad \text{for } k = 0, \ldots, N-1,
$$

with initial condition $\mathbf{x}_0 = \mathbf{x}_{\text{measured}}$. These equality constraints couple the decision variables to the physics so that only dynamically feasible trajectories are considered. Because $\mathbf{A}_k$ and $\mathbf{B}_k$ are refreshed each solve, the approximation remains accurate over the short prediction window.

## 5.5.2 Actuator Constraints

Solenoid valves are either fully open or closed, so each control variable is constrained to be binary:

$$
u_{k,i} \in \{0,1\} \quad \text{for } i = 1,\ldots,8,\; k = 0,\ldots,M-1.
$$

This turns the problem into a Mixed-Integer Linear Program (MILP). Gurobi handles the discrete nature via branch-and-bound, ensuring the resulting commands map directly to real thruster states without relaxation to continuous thrust.

## 5.5.3 State Constraints

Physical safety limits constrain the predicted states throughout the horizon:

- **Position bounds:** $-3.0 \le p_x, p_y \le 3.0$ m (6 m × 6 m test area, ensuring OptiTrack visibility).
- **Velocity bounds:** $-0.25 \le v_x, v_y \le 0.25$ m/s; this cap allows brisk maneuvers but protects hardware.
- **Angular-rate bound:** $- \pi/2 \le \omega \le \pi/2$ rad/s (±90°/s) to avoid excessive spin.
- **Angle bound:** $-2\pi \le \theta \le 2\pi$ rad; wrap-around logic keeps the optimizer from taking unnecessarily long rotation paths.

All constraints are linear inequalities, making them compatible with the MILP formulation and solvable by Gurobi.

# Chapter 5.6 — Complete MILP Formulation

## 5.6.1 Summary

**Decision variables**

- Predicted states $\mathbf{x}_k$ for $k = 0,\ldots,N$ (continuous, 6-D).
- Control inputs $\mathbf{u}_k$ for $k = 0,\ldots,M-1$ (binary, 8-D).
- Switching indicators $s_{k,i}$ (binary) if the switching penalty is enabled.

**Objective**

$$
\min J_{\text{total}} = \sum_{k=1}^{N} (\mathbf{x}_k - \mathbf{x}_{\text{ref}})^T \mathbf{Q} (\mathbf{x}_k - \mathbf{x}_{\text{ref}}) + \sum_{k=0}^{M-1} \mathbf{u}_k^T \mathbf{R} \mathbf{u}_k + R_{\text{switch}} \sum_{k=0}^{M-1} \sum_{i=1}^{8} s_{k,i}.
$$

**Constraints**

1. Initial condition: $\mathbf{x}_0 = \mathbf{x}_{\text{measured}}$.
2. Dynamics: $\mathbf{x}_{k+1} = \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k$ for $k = 0,\ldots,N-1$.
3. Binary actuation: $u_{k,i} \in \{0,1\}$ for $i = 1,\ldots,8$, $k = 0,\ldots,M-1$.
4. Switching linearization (if $R_{\text{switch}} > 0$):
   $$
   s_{k,i} \ge u_{k,i} - u_{k-1,i}, \qquad
   s_{k,i} \ge u_{k-1,i} - u_{k,i}.
   $$
5. State bounds: $\mathbf{x}_{\min} \le \mathbf{x}_k \le \mathbf{x}_{\max}$ for $k = 1,\ldots,N$.

This MILP encapsulates the linearized dynamics, hard actuator limits, and safety bounds while solving for optimal binary thruster sequences each control cycle.

## 5.6.2 Problem Size

With the nominal horizons ($N = 15$, $M = 12$):

- Continuous states: $(N+1) \times 6 = 16 \times 6 = 96$ variables.
- Binary controls: $M \times 8 = 12 \times 8 = 96$ variables.
- Binary switching indicators (if enabled): another $96$.

Constraints:

- Initial condition: 6 equalities.
- Dynamics: $N \times 6 = 15 \times 6 = 90$ equalities.
- State bounds: $N \times 12 = 15 \times 12 = 180$ inequalities (upper/lower bounds).
- Switching linearization (if enabled): $M \times 8 \times 2 = 192$ inequalities.

Totals: 96 continuous + 96 binary variables with 96 equality + 180 inequality constraints (276 total). If switching penalties are active, binaries rise to 192 and inequalities to 372. The linear growth with $N$ and $M$ keeps the model solvable within the 50 ms time budget.

# Chapter 5.7 — Solver Details

## 5.7.1 Gurobi Optimizer Configuration

The controller uses Gurobi 12.0.2. Key parameters are:

- **TimeLimit = 0.05 s:** ensures a solution before the next 60 ms control tick; Gurobi returns the best found solution if time expires.
- **MIPGap = 1%:** allows termination within 1% of optimality, trading a sliver of performance for faster solves.
- **MIPFocus = 1:** prioritizes finding feasible solutions quickly over proving optimality.
- **Heuristics = 0.01:** caps heuristic effort at 1% of solve time, minimizing overhead on this small model.
- **Cuts = 1:** enables moderate cutting planes to tighten LP relaxations without excessive cost.
- **Presolve = 2:** aggressive presolve removes redundant variables/constraints before branch-and-bound.
- **Threads = 1:** single-threaded execution yields more predictable latency than multi-threaded runs on this problem size.
- **ImproveStartTime = 0.6 × TimeLimit:** after 60% of the time budget, the solver pivots from finding better solutions to improving optimality proofs.

These settings were tuned empirically to balance solution quality with consistent real-time performance; in practice solve times remain well below the 50 ms limit.

## 5.7.2 Warm Starting Strategy

Gurobi supports warm starts, so the MPC reuses the previous solution shifted forward in time:

- **State trajectory:** $\mathbf{x}^{\text{init}}_j = \mathbf{x}^{\text{prev}}_{j+1}$ for $j = 0,\ldots,N-1$, with $\mathbf{x}^{\text{init}}_N = \mathbf{x}^{\text{prev}}_N$.
- **Control sequence:** $\mathbf{u}^{\text{init}}_j = \mathbf{u}^{\text{prev}}_{j+1}$ for $j = 0,\ldots,M-2$, and $\mathbf{u}^{\text{init}}_{M-1} = \mathbf{u}^{\text{prev}}_{M-1}$.

Providing this initial guess typically places Gurobi close to the new optimum, reduces branch-and-bound effort, and smooths the thruster commands between control cycles.

# Chapter 5.8 — Implementation Architecture

## 5.8.1 Persistent Model Approach

`SatelliteMPCOptimized` builds the MILP once at startup and reuses it each cycle, updating only the numerical data. This avoids recreating variables and constraints every 60 ms.

**Model construction (startup):**

1. Create decision variables (states, controls, optional switching indicators).
2. Add structural constraints (dynamics, bounds, switching).
3. Define the objective expression.
4. Configure solver parameters (Section 5.7).

**Per-cycle updates:**

1. Set the initial condition $\mathbf{x}_0 = \mathbf{x}_{\text{measured}}$.
2. Refresh $\mathbf{A}_k, \mathbf{B}_k$ with the latest linearization.
3. Update the reference trajectory $\mathbf{x}_{\text{ref}}$.
4. Apply adaptive velocity weights if inside the damping zone.
5. Apply warm-start values from the previous solution.

This persistent approach eliminates the overhead of rebuilding the model, keeping each control iteration within the real-time budget.

## 5.8.2 Angle Wraparound Handling

Angles repeat every $2\pi$ radians, so a target heading of −170° is equivalent to +190°. Without preprocessing, the optimizer might choose the long way around. Before each solve we adjust the target angle to fall within ±π of the current angle:

$$
\theta_{\text{ref}}^{\text{adjusted}} = \theta_{\text{current}} + \operatorname{wrap}(\theta_{\text{ref}} - \theta_{\text{current}}),
$$

where $\operatorname{wrap}(\alpha) = \operatorname{atan2}(\sin\alpha, \cos\alpha)$ returns an angle in $[-\pi, \pi]$. This guarantees the MPC commands the shortest rotation needed to align with the desired heading.
