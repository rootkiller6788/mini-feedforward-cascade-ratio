# Course Tree — mini-ratio-control-gas-liquid

## Prerequisite Knowledge Dependencies

```
                          ┌─────────────────┐
                          │  L1: Ratio Defs │
                          └────────┬────────┘
                                   │
                    ┌──────────────┼──────────────┐
                    ▼              ▼              ▼
           ┌───────────┐  ┌───────────┐  ┌───────────┐
           │L2: Ratio   │  │L2: Master-│  │L2: Ratio  │
           │  Station   │  │  Slave    │  │  Error    │
           └─────┬─────┘  └─────┬─────┘  └─────┬─────┘
                 │              │              │
      ┌──────────┼──────────────┼──────────────┼──────────┐
      ▼          ▼              ▼              ▼          ▼
┌─────────┐ ┌─────────┐ ┌───────────┐ ┌─────────┐ ┌─────────┐
│L3: Lead-│ │L3: Flow │ │L4: Ideal  │ │L4: Henry│ │L5: Cross│
│  Lag    │ │ Filter  │ │  Gas Law  │ │   Law   │ │ -Limit  │
└────┬────┘ └────┬────┘ └─────┬─────┘ └────┬────┘ └────┬────┘
     │           │            │            │           │
     └───────────┼────────────┼────────────┼───────────┘
                 │            │            │
                 ▼            ▼            ▼
         ┌─────────────────────────────────────┐
         │      L5: Ratio Trim Controller      │
         │      L5: Feedforward Design         │
         └─────────────────┬───────────────────┘
                           │
         ┌─────────────────┼───────────────────┐
         ▼                 ▼                   ▼
┌─────────────────┐ ┌──────────────┐ ┌─────────────────┐
│L6: Boiler AFR   │ │L6: CSTR      │ │L6: Blending     │
│  Combustion     │ │  Reactor     │ │  Optimization   │
└────────┬────────┘ └──────┬───────┘ └────────┬────────┘
         │                 │                   │
         └─────────────────┼───────────────────┘
                           │
                           ▼
         ┌─────────────────────────────────────┐
         │     L7: Industrial Applications     │
         │  Siemens PCS7, Honeywell TDC3000,   │
         │  ABB Gas Flow, Toyota Production    │
         └─────────────────┬───────────────────┘
                           │
                           ▼
         ┌─────────────────────────────────────┐
         │     L8: Advanced Topics             │
         │  RLS Identification, Adaptive Trim, │
         │  Blend Optimization, Oscillation    │
         │  Detection, Lyapunov Stability      │
         └─────────────────┬───────────────────┘
                           │
                           ▼
         ┌─────────────────────────────────────┐
         │     L9: Research Frontiers          │
         │  AI Self-Optimization, Digital Twin,│
         │  IT/OT Fusion, Autonomous L4        │
         └─────────────────────────────────────┘
```

## Learning Pathway

1. **Start with L1** — Understand what a ratio is (R = F_slave / F_master)
2. **L2 Core** — Ratio station, master-slave architecture, error computation
3. **L3 Structures** — Filters (EWMA, Butterworth), compensators (lead-lag), deadtime
4. **L4 Laws** — Ideal gas law, Henry's Law, conservation of mass, cross-limiting safety
5. **L5 Algorithms** — Cross-limiting, ratio trim PI, feedforward design, optimization
6. **L6 Problems** — Apply to boiler, reactor, blender, absorber, pipeline, separator
7. **L7 Industry** — Map to Siemens, Honeywell, ABB, Toyota implementations
8. **L8 Advanced** — Adaptive identification, gain scheduling, economic optimization
9. **L9 Frontiers** — AI/ML, digital twin, autonomous operations
