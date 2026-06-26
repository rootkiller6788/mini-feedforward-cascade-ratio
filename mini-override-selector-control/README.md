# mini-override-selector-control

Override/Selector Control Engineering — Complete Implementation

## Module Status: COMPLETE

- **L1-L6**: Complete
- **L7**: Partial+ (7 industrial applications)
- **L8**: Partial+ (6 advanced topics)
- **L9**: Partial (documented, not implemented — per SKILL.md 6.1)

## Overview

This module implements the complete override/selector control methodology, including high-select, low-select, and median-select algorithms, constraint evaluation with approach factors, PID tracking with external reset (IEC 61131-3), valve position control (VPC), compressor anti-surge protection, and triple-redundant voting schemes (2oo3/3oo3/1oo3).

Override (chao chi kong zhi) / selector control is a critical industrial control strategy where multiple controllers compete, and a selector chooses which controller output to use based on process constraints.

## Quick Start

`
make test        # Run all tests
make examples    # Build 4 end-to-end examples
make count       # Show line counts
make check       # Safety scan (anti-filler detection)
make clean       # Clean build artifacts
`

## Nine-Layer Knowledge Coverage

| Level | Name | Status | Highlights |
|-------|------|--------|------------|
| L1 | Definitions | Complete | 17 structs/enums, Lean formalizations |
| L2 | Core Concepts | Complete | HS/LS/MS, auctioneering, hysteresis, VPC, tracking |
| L3 | Engineering Structures | Complete | Selector FB, PID discretization, voting schemes |
| L4 | Engineering Laws | Complete | ISA-5.1, IEC 61131-3, IEC 61508/61511, ISA-18.2 |
| L5 | Algorithms | Complete | 24 algorithms: selectors, PID, constraints, surge |
| L6 | Canonical Problems | Complete | Compressor, Reactor, Boiler, Furnace (4 examples) |
| L7 | Industrial Applications | Partial+ | Oil & Gas, Power, Chemical, Refining, Aerospace |
| L8 | Advanced Topics | Partial+ | Confidence estimation, drift detection, prediction |
| L9 | Research Frontiers | Partial | AI, Digital Twin, RL (documented) |

## Core Definitions

### Selector Types

- **High-Select (HS)**: Choose maximum output. Anti-surge recycle valve.
- **Low-Select (LS)**: Choose minimum output. Valve position limit, max temp.
- **Median-Select (MS)**: Median of redundant measurements (auctioneering).

### Constraint Approach Factor

`
af_hi = (value - (hi_limit - margin)) / margin
af_lo = ((lo_limit + margin) - value) / margin
approach_factor = max(af_hi, af_lo)
`

- AF < 0: Safe (far from limit)
- 0 <= AF < 1: Margin zone (approaching)
- AF >= 1: Violated

### PID External Reset Tracking (IEC 61131-3)

`
I = tracking_value - Kc*(b*sp - pv) - D
`

## Core Algorithms

1. **Selector Algorithms** (5 types): High, Low, Median, Auctioneer, Weighted
2. **Hysteresis-Enabled Selectors**: Prevents chatter with configurable band
3. **Override PID**: Positional + velocity forms, 3 anti-windup methods
4. **Constraint Management**: Approach factor, violation detection, prediction
5. **Valve Position Control (VPC)**: Auto-tuned, split-range, deadband
6. **Compressor Surge Protection**: Surge line, margin, detection, PI control
7. **Signal Voting**: 2oo3, 3oo3, 1oo3 with drift/frozen detection

## Classical Problems

| Problem | File |
|---------|------|
| Compressor Surge Protection | examples/example_compressor_surge.c |
| Reactor Temperature Override | examples/example_reactor_override.c |
| Boiler Double Override (HS+LS) | examples/example_boiler_override.c |
| Furnace Tube Skin Temperature | examples/example_furnace_constraint.c |

## Nine-School Course Mapping

| School | Course | Coverage |
|--------|--------|----------|
| MIT | 6.302 Feedback Systems | Multivariable constraint handling |
| Stanford | ENGR205 Process Control | Industrial constraint control |
| Berkeley | ME233 Advanced Control | Coupled multivariable override |
| CMU | 24-677 Adv Ctrl Systems | System override architecture |
| Georgia Tech | ECE 6550 Nonlinear | Surge detection |
| Purdue | ME 575 Industrial Control | Override/selector methodology |
| RWTH Aachen | ICS | IEC 61131-3 FBs, VPC |
| Tsinghua | Process Control | chao chi kong zhi, xuan ze kong zhi |
| ISA/IEC | Standards | PID tracking, constraint management |

## Lean 4 Formalization

src/override_core.lean contains (513 lines, all theorems proven without `sorry`):
- Inductive types: SelectorType, OverrideMode, ConstraintType, Priority
- Structures: PIDParams, ConstraintDef, ControllerState, OverrideSystemState
- Functions: selectorHigh, selectorLow, selectorMedian, approachFactor (Float utilities)
- Theorems (L4, proven on Nat/Int per SKILL.md §4.3):
  - `selector_high_singleton`, `selector_high_ge_each`, `selector_low_le_each`
  - `selector_high_ge_low` (high ≥ low for nonempty lists)
  - `vote2oo3_all_equal`, `vote2oo3_sorted_picks_middle`, `vote2oo3_majority`
  - `emergency_is_highest`, `diagnostic_is_lowest`, `priority_order_transitive`
  - `hysteresis_prevents_chatter_int` (on ℤ)
  - `validActive_implies_enabled_and_not_faulted`
  - `fault_reachable_from_any`, `no_direct_disabled_to_override` (mode transitions)
- Concrete verifications via `#eval`/`native_decide`:
  - Constraint approach factor on example temperature constraint
  - Rate-limited output examples on concrete Int values

## File Structure

`
mini-override-selector-control/
  Makefile (make test/examples/count/check/clean)
  README.md (this file)
  include/ (6 headers, 1,806 lines)
  src/ (8 C files, 3,483 lines)
  src/ (1 Lean file, 513 lines)
  tests/ (2 test files, 22 tests passing)
  examples/ (4 end-to-end examples)
  demos/ (interactive demo)
  benches/ (micro-benchmarks)
  docs/ (5 documentation files)
`

## Line Counts

| Component | Lines |
|-----------|-------|
| include/ (6 headers) | 1,806 |
| src/ (8 C files) | 3,483 |
| src/ (1 Lean file) | 513 |
| **Total include/+src/** | **5,802** |

## References

1. Shinskey, F.G. (1996). Process Control Systems (4th ed.). McGraw-Hill. Ch. 9.
2. Liptak, B.G. (2006). Instrument Engineers' Handbook (4th ed.), Vol. 2.
3. Astrom & Hagglund (1995). PID Controllers (2nd ed.). ISA.
4. IEC 61131-3 (2013). PID Function Block with External Reset.
5. Nisenfeld & Seemann (1981). Centrifugal Compressors. ISA.
6. Luyben, W.L. (2007). Chemical Reactor Design and Control. Wiley.
7. Dukelow, S.G. (1991). The Control of Boilers (2nd ed.). ISA.
8. Boyce, M.P. (2012). Gas Turbine Engineering Handbook (4th ed.).
9. Seborg, Edgar & Mellichamp (2016). Process Dynamics and Control (4th ed.).
10. ISA-18.2 (2016). Management of Alarm Systems.
