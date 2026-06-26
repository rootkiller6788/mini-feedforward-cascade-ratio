# Knowledge Graph -- mini-gain-scheduled-pid

## L1 -- Definitions (Complete)

| # | Knowledge Point | Location |
|---|----------------|----------|
| 1 | Scheduling variable types (11) | gain_schedule_core.h |
| 2 | Interpolation methods (7) | gain_schedule_core.h |
| 3 | PID controller forms (6) | gain_schedule_core.h |
| 4 | Operating region types (8) | gain_schedule_core.h |
| 5 | PID gain set struct | gain_schedule_core.h |
| 6 | Schedule table entry (1D) | gain_schedule_core.h |
| 7 | Gain schedule table (1D) | gain_schedule_core.h |
| 8 | 2D schedule table | gain_schedule_core.h |
| 9 | Gain-scheduled PID state | gain_schedule_core.h |
| 10 | Design configuration | gain_schedule_core.h |
| 11 | RLS estimator (adaptive) | gain_schedule_adaptive.h |
| 12 | Fuzzy logic schedule | gain_schedule_adaptive.h |
| 13 | Performance metrics (IAE/ISE/ITAE) | gain_schedule_adaptive.h |
| 14 | Process info struct | gain_schedule_design.h |

## L2 -- Core Concepts (Complete)

| # | Concept | Location |
|---|---------|----------|
| 1 | Gain scheduling principle | gain_schedule_core.h |
| 2 | Frozen-parameter design | gain_schedule_design.c |
| 3 | Online gain interpolation | gain_schedule_interp.c |
| 4 | Scheduling variable selection | gain_schedule_core.h |
| 5 | Anti-windup with scheduled gains | gain_schedule_pid.c |
| 6 | Bumpless gain transitions | gain_schedule_pid.c |
| 7 | Multi-model blending | gain_schedule_adaptive.c |
| 8 | Performance-based adaptation | gain_schedule_adaptive.c |
| 9 | Fuzzy logic gain adjustment | gain_schedule_adaptive.c |
| 10 | Gaussian RBF interpolation | gain_schedule_interp.c |

## L3 -- Engineering Structures (Complete)

| # | Structure | Location |
|---|-----------|----------|
| 1 | 1D gain schedule table | gain_schedule_core.c |
| 2 | 2D bilinear interpolation grid | gain_schedule_interp.c |
| 3 | PID state machine with scheduling | gain_schedule_pid.c |
| 4 | Spline moment computation (Thomas) | gain_schedule_interp.c |
| 5 | Fritsch-Carlson monotone cubic | gain_schedule_interp.c |
| 6 | RLS covariance matrix update | gain_schedule_adaptive.c |
| 7 | Fuzzy inference engine | gain_schedule_adaptive.c |
| 8 | Binary search bracket finding | gain_schedule_core.c |

## L4 -- Engineering Standards (Complete)

| # | Standard | Location |
|---|----------|----------|
| 1 | Ziegler-Nichols PID/PI (1942) | gain_schedule_design.c |
| 2 | Tyreus-Luyben PID/PI (1992) | gain_schedule_design.c |
| 3 | Cohen-Coon PID/PI (1953) | gain_schedule_design.c |
| 4 | IMC-based PID tuning (1989) | gain_schedule_design.c |
| 5 | SIMC method (Skogestad 2003) | gain_schedule_design.c |
| 6 | AMIGO PID/PI (Astrom 2004) | gain_schedule_design.c |
| 7 | Routh-Hurwitz stability criterion | gain_schedule_stability.c |
| 8 | Frozen-time stability (Shamma 1990) | gain_schedule_stability.c |
| 9 | Slow-variation condition (Desoer 1969) | gain_schedule_stability.c |
| 10 | Lean formal proofs (4 theorems) | gain_schedule_formal.lean |

## L5 -- Algorithms/Methods (Complete)

| # | Algorithm | Location |
|---|-----------|----------|
| 1 | Nearest-neighbor interpolation | gain_schedule_interp.c |
| 2 | Linear interpolation | gain_schedule_interp.c |
| 3 | Cubic Hermite interpolation (monotone) | gain_schedule_interp.c |
| 4 | Cubic spline (natural boundary) | gain_schedule_interp.c |
| 5 | Lagrange polynomial interpolation | gain_schedule_interp.c |
| 6 | Akima spline (oscillation-free) | gain_schedule_interp.c |
| 7 | Gaussian RBF interpolation | gain_schedule_interp.c |
| 8 | Frozen-parameter design (6 rules) | gain_schedule_design.c |
| 9 | Schedule grid refinement | gain_schedule_design.c |
| 10 | Schedule smoothing (moving average) | gain_schedule_design.c |
| 11 | RLS parameter estimation | gain_schedule_adaptive.c |
| 12 | Fuzzy inference (Mamdani-type) | gain_schedule_adaptive.c |
| 13 | Gradient-based gain adaptation | gain_schedule_adaptive.c |
| 14 | Gaussian membership weighting | gain_schedule_adaptive.c |
| 15 | Model blending (parallel bank) | gain_schedule_adaptive.c |
| 16 | Frequency-domain margin computation | gain_schedule_stability.c |
| 17 | Lyapunov condition number check | gain_schedule_stability.c |

## L6 -- Canonical Problems (Complete)

| # | Problem | Location |
|---|---------|----------|
| 1 | Temperature control (nonlinear gain) | example_temp_control.c |
| 2 | Servo positioning (velocity-dependent) | example_servo_position.c |
| 3 | pH neutralization (titration curve) | example_ph_control.c |
| 4 | Heat exchanger with varying load | example_temp_control.c |
| 5 | Motion control with friction | example_servo_position.c |

## L7 -- Industrial Applications (Partial+)

| # | Application | Location |
|---|-------------|----------|
| 1 | Heat exchanger temperature (energy) | example_temp_control.c |
| 2 | CNC/robot servo control (mfg) | example_servo_position.c |
| 3 | Chemical pH neutralization (process) | example_ph_control.c |
| 4 | DC motor velocity scheduling | gain_schedule_core.h |
| 5 | Flow-dependent PID (Toyota) | gain_schedule_design.c |

## L8 -- Advanced Topics (Partial+)

| # | Topic | Location |
|---|-------|----------|
| 1 | Recursive least squares (RLS) | gain_schedule_adaptive.c |
| 2 | Fuzzy logic gain scheduling | gain_schedule_adaptive.c |
| 3 | Lyapunov stability analysis | gain_schedule_stability.c |
| 4 | Multi-model blending | gain_schedule_adaptive.c |
| 5 | Gradient-based adaptation | gain_schedule_adaptive.c |
| 6 | Monte Carlo validation (SPSA) | gain_schedule_adaptive.c |
| 7 | Gaussian RBF interpolation | gain_schedule_interp.c |

## L9 -- Industry Frontiers (Partial)

| # | Frontier | Location |
|---|----------|----------|
| 1 | Adaptive gain scheduling | gain_schedule_adaptive.c |
| 2 | Fuzzy-neural hybrid scheduling | gain_schedule_adaptive.h (documented) |
| 3 | Digital twin-based scheduling | gain_schedule_formal.lean (documented) |
