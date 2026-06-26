# Course Tree - Split-Range Control Prerequisites

## Dependency Graph

mini-pid-control-engineering
  -> mini-feedforward-cascade-ratio
    -> mini-split-range-control-heat-cool  (THIS MODULE)
      -> mini-dcs-architecture-application
      -> mini-safety-instrumented-system
      -> mini-advanced-process-control-apc

## Prerequisites

- mini-pid-control-engineering: PID forms, anti-windup, ZN tuning
- mini-feedforward-cascade-ratio: Cascade architecture, external reset
- mini-industrial-measurement-actuator: PV scaling, valve fundamentals

## Dependents

- mini-safety-instrumented-system: Emergency cooling SIF, PST, SIL
- mini-dcs-architecture-application: Split-range function block, HMI
- mini-advanced-process-control-apc: Adaptive gain, MCMC optimization

## Knowledge Progression

Phase 1 (L1-L3): Definitions, PID forms, valve characteristics
Phase 2 (L4-L5): ISA standards, tuning methods, auto-tuning
Phase 3 (L6-L7): Reactor/pH/pressure control applications
Phase 4 (L8-L9): Adaptive control, MCMC, digital twin

## Recommended Reading

1. Seborg et al. (2016) Ch. 16 - Split-range control
2. Myke King (2016) Ch. 9 - Reactor control
3. Astrom & Hagglund (1995) Ch. 3 - PID implementation
4. ISA-75.01.01 - Flow equations for control valves
5. Fogler (2016) Ch. 12 - CSTR thermal effects
6. Morari & Zafiriou (1989) Ch. 3 - IMC tuning
7. Khalil (2002) Ch. 9 - Slowly varying systems
