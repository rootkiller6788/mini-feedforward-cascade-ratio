# Course Tree — MIMO Decoupling Control

## Prerequisites
- mini-pid-control-engineering (PID fundamentals)
- mini-feedforward-cascade-ratio (feedforward/ratio concepts)

## Core Dependencies
```
Matrix Algebra (Gaussian elimination, eigenvalues)
    ├── RGA Computation (Bristol 1966)
    │       └── Pairing Selection & Integrity Check
    ├── SVD Decomposition (Golub-Reinsch 1970)
    │       └── Static/Dynamic SVD Decoupler
    └── Matrix Inversion (LU decomposition)
            └── Static Decoupler D = K^{-1}

Transfer Functions (FOPDT, SOPDT models)
    ├── MIMO Model Construction
    ├── Frequency Response Evaluation
    └── Dynamic Decoupler Design
            ├── Ideal Decoupler D(s) = G^{-1}(s)·diag(G)
            ├── Simplified Decoupler (with realizability)
            └── Partial Decoupler (band-limited)

State-Space Theory (Kalman 1960)
    ├── Controllability/Observability Tests
    ├── Tustin Discretization
    └── Lyapunov Stability (Bartels-Stewart)

Control Theory
    ├── Niederlinski Index (1971)
    ├── Routh-Hurwitz Criterion (1877)
    ├── BLT Tuning (Luyben 1986)
    ├── IMC-Based Inverted Decoupling (Garrido 2012)
    └── Structured SVD (mu) Analysis (Doyle 1982)
```

## Postrequisites
- mini-advanced-process-control-apc (MPC for MIMO)
- mini-industrial-mpc-implementation (industrial MPC)
- mini-complex-control-theory (nonlinear MIMO control)
