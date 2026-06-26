# Mini Feedforward Cascade Ratio Control

A collection of **from-scratch, zero-dependency C implementations** of advanced industrial process control strategies: feedforward, cascade, ratio, split-range, override/selector, dead-time compensation, MIMO decoupling, and gain-scheduled PID. Each module bridges control theory textbooks and real-world industrial practice by translating mathematical design procedures into runnable C code.

## Sub-Modules

| Sub-Module | Topics | Key Courses |
|------------|--------|-------------|
| [mini-cascade-control-primary-secondary](mini-cascade-control-primary-secondary/) | Cascade PID control, inner/outer loop tuning (ZN, Cohen-Coon, SIMC, Lambda), bumpless transfer, Nyquist stability margins, feedforward compensation | MIT 6.241J, MIT 2.151 |
| [mini-deadtime-compensation-smith](mini-deadtime-compensation-smith/) | Smith Predictor (FOPDT/SOPDT), delay-free prediction, model mismatch correction, adaptive Smith, sensitivity/robustness analysis, Monte Carlo verification | MIT 6.241J, Åström–Hägglund |
| [mini-decoupling-mimo-process](mini-decoupling-mimo-process/) | Static/dynamic/inverted/SVD decoupling, RGA interaction analysis, Niederlinski index, condition number, Wood-Berry model, robust stability (mu analysis) | MIT 6.241J, MIT 2.151, Skogestad |
| [mini-gain-scheduled-pid](mini-gain-scheduled-pid/) | Gain scheduling via scheduling variables, interpolation (nearest/linear/Hermite/spline/RBF), PID forms (ideal/series/ISA/2-DOF), stability analysis, adaptive extensions | MIT 6.241J, MIT 16.30 |
| [mini-override-selector-control](mini-override-selector-control/) | Override/selector control, constraint management, auctioneering, valve position control (VPC), PID with anti-windup tracking, diagnostics | MIT 6.241J, ISA-5.1 |
| [mini-ratio-control-gas-liquid](mini-ratio-control-gas-liquid/) | Master-slave ratio tracking, cross-limiting combustion safety, gas/liquid flow compensation, blend optimization, adaptive ratio trim (RLS) | MIT 6.241J, MIT 10.450 |
| [mini-split-range-control-heat-cool](mini-split-range-control-heat-cool/) | Split-range mapping (heat/cool), valve characterization, advanced auto-tuning, adaptive gain, reactor safety sequencing | MIT 6.241J, MIT 10.450 |
| [mini-static-dynamic-feedforward](mini-static-dynamic-feedforward/) | Static/dynamic feedforward, lead-lag compensation, combined FF+FB, iterative learning control (ILC), Kalman filter disturbance estimation, NMP factorization | MIT 6.241J, MIT 2.151 |

## Design Philosophy

- **Zero external dependencies** — pure C (C99/C11), only `libc` and `libm`
- **Self-contained modules** — each directory has its own `Makefile`, `include/`, `src/`, `examples/`, `demos/`, `tests/`
- **Theory-to-code mapping** — every module includes `docs/` with course-alignment notes, translating textbook block diagrams into runnable control loops
- **Industrial-grade structures** — mirror real-world DCS/PLC control function blocks: PID forms, process models, sensor/actuator interfaces

## Building

Each module is standalone. Navigate to a module directory and run:

```bash
cd mini-cascade-control-primary-secondary
make all    # build everything
make test   # run tests
```

Requires **GCC** and **GNU Make**.

## Project Structure

```
mini-feedforward-cascade-ratio/
├── mini-cascade-control-primary-secondary/   # Cascade PID with primary/secondary loops
├── mini-deadtime-compensation-smith/          # Smith Predictor for dead-time processes
├── mini-decoupling-mimo-process/              # MIMO decoupling and interaction analysis
├── mini-gain-scheduled-pid/                   # Gain-scheduled PID across operating regions
├── mini-override-selector-control/            # Override/selector constraint handling
├── mini-ratio-control-gas-liquid/             # Ratio control with cross-limiting safety
├── mini-split-range-control-heat-cool/        # Split-range valve sequencing
└── mini-static-dynamic-feedforward/           # Static and dynamic feedforward compensation
```

## License

MIT
