# Course Tree — mini-static-dynamic-feedforward

## Prerequisite Dependency Tree

```
PID Control Engineering (mini-pid-control-engineering)
├── L2: Feedback control basics
├── L3: PID discrete forms (positional/incremental)
├── L4: Ziegler-Nichols, IMC tuning
└── L5: Anti-windup, bumpless transfer
        │
        ▼
Static-Dynamic Feedforward (this module)
├── L1: FF definitions, transfer functions
├── L2: Static vs dynamic FF, combined FF+FB
├── L3: Discretization (ZOH, Tustin, Euler)
├── L4: Perfect FF condition, Pade approximation
├── L5: FOPDT/SOPDT identification, pattern search
├── L6: Heat exchanger, distillation, pH, motor examples
├── L7: Gain scheduling, industrial applications
├── L8: NMP, Kalman, robustness
└── L9: ILC, digital-twin concepts
        │
        ▼
Advanced Process Control (mini-advanced-process-control-apc)
├── Model Predictive Control (DMC/GPC)
├── MIMO decoupling
├── Soft sensors
└── Real-time optimization
```

## Internal Module Dependencies

```
feedforward_defs.h         (L1 base types)
    │
    ├── feedforward_static.h   (L2 static FF)
    │       └── feedforward_static.c
    │
    ├── feedforward_dynamic.h  (L3/L5 dynamic FF)
    │       └── feedforward_dynamic.c
    │
    ├── feedforward_models.h   (L3/L4/L5 models)
    │       └── feedforward_models.c
    │
    ├── feedforward_combined.h (L2/L5/L6 combined)
    │       └── feedforward_combined.c
    │
    └── feedforward_advanced.h (L7/L8/L9 advanced)
            └── feedforward_advanced.c

Additional source files:
    feedforward_tuning.c       (L4/L5 design + tuning)
    feedforward_applications.c (L6/L7 industrial cases)
```