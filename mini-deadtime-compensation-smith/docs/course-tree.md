# Course Dependency Tree — Smith Predictor

## Prerequisites (Must Know First)
1. **PID Control Fundamentals** (module 1: mini-pid-control-engineering)
   - P, I, D terms, parallel form, digital implementation
2. **Advanced PID Tuning** (module 2: mini-advanced-pid-tuning)
   - IMC, SIMC, Ziegler-Nichols tuning rules
3. **Feedforward/Cascade/Ratio Control** (module 3 parent: mini-feedforward-cascade-ratio)
   - Structural control design, multi-loop architectures

## This Module Provides
- Dead-time compensation theory (Smith 1957)
- Process model identification (step test, relay, RLS)
- Digital implementation of time delays (circular buffer)
- Robustness analysis (sensitivity, margins, Monte Carlo)
- Adaptive dead-time compensation (RLS + auto-redesign)

## Modules That Depend On This
- **Module 7**: mini-dcs-architecture-application — Smith predictor as DCS function block
- **Module 12**: mini-advanced-process-control-apc — dead-time handling in APC
- **Module 13**: mini-industrial-mpc-implementation — delay in Model Predictive Control
- **Module 14**: mini-soft-sensor-inferential — inferential control with delay compensation
- **Module 18**: mini-industrial-ai-control-fusion — AI-based adaptive dead-time compensation

## Concept Dependency Graph
```
PID Basics --> Advanced Tuning --> Feedforward/Cascade
                                        |
                                        v
                              SMITH PREDICTOR (this module)
                                        |
                    +-------------------+-------------------+
                    v                   v                   v
              DCS/SCADA            APC / MPC           AI Control
              (module 7)          (module 12,13)       (module 18)
```
