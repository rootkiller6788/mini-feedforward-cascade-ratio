# mini-cascade-control-primary-secondary

Cascade Control — Primary/Secondary Loop Configuration

## Module Status: COMPLETE

- L1-L6: Complete
- L7: Partial+ (4 industrial application templates)
- L8: Partial+ (adaptive cascade, Monte Carlo, Lyapunov, Smith predictor)
- L9: Partial (documented)

## Overview

This module implements comprehensive cascade control for industrial process automation.
Cascade control uses a hierarchical two-loop architecture where the primary (outer)
controller output serves as the setpoint for the secondary (inner) controller.

## Nine-Level Knowledge Coverage

| Level | Name | Status | Key Items |
|-------|------|--------|-----------|
| L1 | Definitions | Complete | Cascade modes, PID forms, FOPDT/SOPDT models |
| L2 | Core Concepts | Complete | Cascade hierarchy, bumpless transfer, FF+FB |
| L3 | Engineering Structures | Complete | Positional/velocity PID, Tustin discretization |
| L4 | Engineering Laws | Complete | Nyquist criterion, ZN/CC/SIMC/Lambda tuning |
| L5 | Algorithms/Methods | Complete | Anti-windup, sequential tuning, RLS |
| L6 | Canonical Problems | Complete | Temp cascade, level-flow, ratio blending |
| L7 | Industrial Applications | Partial+ | Boiler, HX, distillation, reactor templates |
| L8 | Advanced Topics | Partial+ | Adaptive, Monte Carlo, Lyapunov, Smith |
| L9 | Industry Frontiers | Partial | IT/OT, autonomous tuning (documented) |

## Core Definitions

- Cascade Control: Two-loop hierarchy; primary output sets secondary setpoint
- Primary Loop: Outer (master) loop for slow dynamics (temperature, level)
- Secondary Loop: Inner (slave) loop for fast dynamics (flow, pressure)
- Update Ratio: Secondary executes N times per primary update (N = 3-10)
- Bumpless Transfer: Smooth Manual-Auto-Cascade transitions without output jumps
- Anti-Windup: Prevent integrator accumulation when actuator saturates

## Core Theorems

1. Nyquist Stability: Closed-loop stable iff L(jw) does not encircle -1
2. SIMC Tuning: Kc = tau/(K*(tau_c+theta)), Ti = min(tau,4*(tau_c+theta))
3. Ziegler-Nichols PID: Kc = 0.6*Ku, Ti = Pu/2, Td = Pu/8
4. Cohen-Coon PID: Kc = (1/K)*(tau/theta)*(4/3+theta/(4*tau))
5. Lambda Tuning: Kc = tau/(K*(lambda+theta)), Ti = tau
6. RGA (2x2): lambda11 = 1/(1 - K12*K21/(K11*K22))
7. Ideal Feedforward: Gff(s) = -Gd(s)/Gp(s)
8. Bode Margins: GM > 6 dB, PM > 30 deg, Ms < 2.0

## Core Algorithms

1. Cascade PID — Positional/velocity form with anti-windup and bumpless transfer
2. Sequential Tuning — Secondary first, then primary with closed inner loop
3. Ms-Constrained Tuning — Nelder-Mead optimization with sensitivity constraint
4. Nyquist Analysis — Frequency sweep with interpolation for crossover detection
5. RGA Decoupling — Steady-state 2x2 relative gain array and decoupler design
6. Lead-Lag Feedforward — Tustin-discretized dynamic disturbance compensation
7. RLS Adaptive — Recursive least squares online identification for auto-tuning

## Canonical Problems

1. Temperature Cascade: Reactor temp (primary) / jacket temp (secondary)
2. Level-Flow Cascade: Tank level (primary) / outlet flow (secondary)
3. Ratio Blending: Two-stream blending with ratio control
4. Boiler Drum Level: Drum level (primary) / feedwater flow (secondary)

## Curriculum Mapping

| School | Course | Coverage |
|--------|--------|----------|
| MIT | 6.302 / 2.171 | Nyquist stability, cascade transfer functions |
| Stanford | ENGR205 | Cascade architecture, feedforward design |
| Berkeley | ME233 / EE C128 | PID discretization, digital cascade |
| CMU | 24-677 | RGA decoupling, MIMO cascade |
| Georgia Tech | ECE 6550 / AE 6530 | Nonlinear cascade, Monte Carlo |
| Purdue | ME 575 | Boiler drum, heat exchanger cascade |
| RWTH Aachen | ICS | IEC 61131-3 cascade blocks |
| Tsinghua | Process Control | Distillation, reactor cascade |
| ISA/IEC | ISA/ISE 61131/61508 | PID blocks, safety override |

## File Structure

```
mini-cascade-control-primary-secondary/
  Makefile              — make test / make examples
  README.md             — This file
  include/ (9 headers)
  src/ (7 C + 1 Lean)
  tests/ (3 test files)
  examples/ (3 examples)
  docs/ (5 knowledge docs)
```

## Build

```bash
make          # Build all test and example binaries
make test     # Run all tests (189 pass)
make examples # Run all examples
make clean    # Remove build artifacts
```

## Test Results

- Cascade Control Tests: 69 passed, 0 failed
- Cascade Tuning Tests: 64 passed, 0 failed
- Feedforward/Ratio/Override Tests: 56 passed, 0 failed
- **ALL 189 TESTS PASSED**

## References

- Seborg, Edgar, Mellichamp (2016) — Process Dynamics and Control
- Astrom & Hagglund (1995) — PID Controllers: Theory, Design, and Tuning
- Skogestad (2003) — Simple analytic rules for PID controller tuning
- Ziegler & Nichols (1942) — Optimum settings for automatic controllers
- Cohen & Coon (1953) — Theoretical consideration of retarded control
- Brosilow & Joseph (2002) — Techniques of Model-Based Control
- Smith (1957) — Closer control of loops with dead time
- IEC 61131-3 — Programming Industrial Automation Systems

---

**COMPLETE** | Lines: include/ + src/ = 6956 | L1-L6 Complete | L7-L9 Partial+ | 189 tests pass
