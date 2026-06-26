# Course Tree -- mini-gain-scheduled-pid

## Prerequisites

This module depends on:
- **mini-pid-structure-p-i-pi-pd-pid** (L1: PID forms and definitions)
- **mini-sampling-rate-discretization** (L3: discrete-time PID)
- **mini-ziegler-nichols-tuning** (L4: ZN tuning rules)
- **mini-anti-windup-bumpless-transfer** (L3: anti-windup methods)
- **mini-cohen-coon-imc-tuning** (L5: IMC/SIMC design)
- **mini-pid-robustness-stability-margin** (L4: stability analysis)

## Knowledge Dependencies

```
L1: PID Definitions
 ����> L2: Gain scheduling concept
 ��    ����> L3: Schedule table structures
 ��    ��    ����> L4: Tuning rules (ZN, CC, IMC, SIMC, AMIGO)
 ��    ��    ��    ����> L5: Frozen-parameter design algorithm
 ��    ��    ��    ��    ����> L6: Temperature/pH/servo examples
 ��    ��    ��    ��    ����> L7: Industrial applications
 ��    ��    ��    ��    ����> L8: Adaptive/fuzzy/RLS extensions
 ��    ��    ��    ����> L4: Stability analysis (Routh-Hurwitz, Lyapunov)
 ��    ��    ��         ����> L5: Spectral abscissa, margin computation
 ��    ��    ����> L5: Interpolation algorithms (6 methods)
 ��    ��         ����> L6: Grid refinement, schedule smoothing
 ��    ����> L3: PID controller with gain scheduling
 ��         ����> L5: Gain smoothing and rate limiting
 ��         ����> L6: Anti-windup integration
 ����> L4: Lean formal proofs
      ����> L8: Formal stability guarantees
```

## Downstream Dependencies

Modules that depend on this:
- **mini-static-dynamic-feedforward** (L6: combined feedforward + gain scheduling)
- **mini-multi-loop-pid-decoupling** (L5: scheduled decoupling gains)
- **mini-model-free-pid-optimization** (L8: adaptive optimization)
- **mini-advanced-process-control-apc** (L8: hierarchical scheduling)
- **mini-siemens-plc-engineering** (L7: TIA Portal implementation)
