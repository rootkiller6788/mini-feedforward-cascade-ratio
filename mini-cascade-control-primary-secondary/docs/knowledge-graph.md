# Knowledge Graph — mini-cascade-control-primary-secondary

## L1: Definitions (Complete)

| Item | Definition | Location |
|------|-----------|----------|
| Cascade Control | Multi-loop strategy: primary output → secondary setpoint | cascade_types.h |
| Primary Loop | Outer (master) loop; slow dynamics; setpoint tracking | cascade_types.h |
| Secondary Loop | Inner (slave) loop; fast dynamics; disturbance rejection | cascade_types.h |
| Cascade Mode | OFF/AUTO/MANUAL/CASCADE/REMOTE_SP/INITIALIZE/FAILSAFE | cascade_types.h |
| PID Form | PARALLEL/IDEAL/SERIES/DERIV_ON_PV | cascade_types.h |
| Anti-Windup | CLAMPING/BACK_CALC/CONDITIONAL/EXTERNAL_RESET | cascade_types.h |
| FOPDT Model | First-Order Plus Dead Time: K/(tau*s+1)*exp(-theta*s) | cascade_types.h |
| SOPDT Model | Second-Order Plus Dead Time | cascade_types.h |
| Integrating Model | K/s * exp(-theta*s), typical for level control | cascade_types.h |
| Feedforward Mode | OFF/STATIC/LEAD_LAG/DEADTIME/RATIO/TABLE | feedforward_control.h |
| Ratio Control | Maintains ratio R = wild_flow / controlled_flow | ratio_control.h |
| Override Selector | Max/min/median selection among competing controllers | override_selector.h |

## L2: Core Concepts (Complete)

| Concept | Description | Location |
|---------|-------------|----------|
| Cascade Hierarchy | Primary PID → Secondary SP → Secondary PID → Actuator | cascade_control.c |
| Bumpless Transfer | Manual↔Auto↔Cascade transitions without output bumps | cascade_control.c |
| Update Ratio | Secondary loop 3-10x faster than primary | cascade_types.h |
| Setpoint Tracking | Primary follows secondary PV when not in cascade | cascade_control.c |
| Anti-Windup Cascade | Primary I frozen when secondary output saturates | cascade_pid.c |
| Feedforward Principle | u_ff = -Gd/Gp * d for measured disturbance rejection | feedforward_control.c |
| FF+FB Architecture | Total output = u_feedback + u_feedforward | feedforward_control.c |
| Ratio Blending | Blending two streams to maintain composition ratio | ratio_control.c |
| Override Logic | Safety-critical override of normal control | override_selector.c |
| Loop Decoupling | RGA-based decoupling for interacting 2x2 systems | cascade_control.c |

## L3: Engineering Structures (Complete)

| Structure | Description | Location |
|-----------|-------------|----------|
| Positional PID | u(k) = Kp*e(k) + Ki*sum(e) + Kd*(e(k)-e(k-1))/Ts | cascade_pid.c |
| Velocity PID | du(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + ... | cascade_pid.c |
| Tustin Discretization | Bilinear transform for lead-lag filter | feedforward_control.c |
| Ring Buffer Delay | Deadtime implementation via circular buffer | feedforward_control.c |
| Split Range Output | Single CO → two actuator ranges (heat/cool) | cascade_control.c |
| Gain Schedule Table | [op_point, Kc, Ti, Td] x N breakpoints | cascade_control.c |
| Scan Cycle | Primary Ts_pri = N * Ts_sec (update ratio) | cascade_control.c |

## L4: Engineering Laws (Complete)

| Law/Standard | Statement | Location |
|--------------|-----------|----------|
| Nyquist Criterion | N = Z - P; stable iff no -1 encirclements | cascade_stability.c |
| Gain Margin | GM = -20*log10(|L(jw_pc)|) > 6 dB target | cascade_stability.c |
| Phase Margin | PM = 180 + arg(L(jw_c)) > 30 deg target | cascade_stability.c |
| Sensitivity Peak | Ms = max_w |1/(1+L(jw))| < 2.0 target | cascade_stability.c |
| Modulus Margin | min_w |1+L(jw)| = 1/Ms > 0.5 target | cascade_stability.c |
| Ziegler-Nichols | Kc=0.6*Ku, Ti=Pu/2, Td=Pu/8 (PID) | cascade_tuning.c |
| Cohen-Coon | Kc=(1/K)*(tau/theta)*(4/3+theta/(4*tau)) | cascade_tuning.c |
| SIMC Rules | Kc=tau/(K*(tau_c+theta)), Ti=min(tau,4*(tau_c+theta)) | cascade_tuning.c |
| Lambda Tuning | Kc=tau/(K*(lambda+theta)), Ti=tau | cascade_tuning.c |
| RGA (2x2) | lambda_11 = 1/(1-K12*K21/(K11*K22)) | cascade_control.c |

## L5: Algorithms (Complete)

| Algorithm | Description | Location |
|-----------|-------------|----------|
| Cascade PID Update | Sequential primary→secondary execution | cascade_pid.c |
| Anti-Windup Clamping | Freeze integrator when output saturates | cascade_pid.c |
| Anti-Windup Back-Calc | I += (Ts/Tt)*(u_sat-u_unsat) | cascade_pid.c |
| Sequential Tuning | Secondary first, then primary with closed inner loop | cascade_tuning.c |
| Ms-Constrained Tuning | Nelder-Mead optimization with Ms penalty | cascade_tuning.c |
| Phase Margin Tuning | Binary search for gain at specified PM | cascade_tuning.c |
| Monte Carlo Robustness | Parameter uncertainty sampling for probabilistic stability | cascade_advanced.c |
| Lyapunov Quadratic | Common P matrix for LPV cascade stability | cascade_advanced.c |
| Smith Predictor | Deadtime compensation in cascade primary | cascade_advanced.c |
| RLS Online ID | Recursive least squares for adaptive cascade | cascade_advanced.c |
| Gain Schedule Interpo | Linear interpolation between breakpoints | cascade_control.c |
| RGA Decoupling 2x2 | Compute decouplers d12, d21 from steady-state gains | cascade_control.c |
| Split Range Compute | Partition CO into heat/cool or coarse/fine ranges | cascade_control.c |
| Stiction Index | Horch (1999) method: detect valve stiction from PV-OP | cascade_advanced.c |

## L6: Canonical Problems (Complete)

| Problem | Description | Location |
|---------|-------------|----------|
| Temp-Flow Cascade | Reactor temp (primary) / jacket temp (secondary) | example_temp_cascade.c |
| Level-Flow Cascade | Tank level (primary) / outlet flow (secondary) | example_level_flow_cascade.c |
| Ratio Blending | Two-stream blending with ratio control | example_ratio_blending.c |
| Boiler Drum Level | Drum level / feedwater flow cascade | cascade_advanced.h template |

## L7: Industrial Applications (Partial+)

| Application | Industry | Reference |
|-------------|----------|-----------|
| Boiler Drum Level | Power Generation | cascade_advanced.c template |
| Heat Exchanger Temp | Oil & Gas, Chemical | cascade_advanced.c template |
| Distillation Column | Petrochemical | cascade_advanced.c template |
| Reactor Temperature | Pharma, Chemical | cascade_advanced.c template |
| ISA-101 HMI | Process Automation (documented) | L7 reference |

## L8: Advanced Topics (Partial+)

| Topic | Description | Location |
|-------|-------------|----------|
| Adaptive Cascade | RLS online identification + SIMC auto-tuning | cascade_advanced.c |
| Gain Scheduling | Parameter interpolation across operating range | cascade_control.c |
| Monte Carlo Analysis | Probabilistic stability under model uncertainty | cascade_advanced.c |
| Lyapunov Stability | Quadratic Lyapunov for time-varying cascade | cascade_advanced.c |
| Smith Predictor | Deadtime-dominant primary loop compensation | cascade_advanced.c |

## L9: Research Frontiers (Partial)

| Topic | Status |
|-------|--------|
| IT/OT Integration for Cascade | Documented |
| Autonomous Loop Tuning | Partial (RLS + SIMC) |
| Industrial 5G Cascade | Documented |
| Digital Twin Cascade | Documented |
