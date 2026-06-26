# Knowledge Graph — MIMO Decoupling Control

## L1: Definitions (Complete)

| ID | Term | Definition | C Struct / Lean Def |
|----|------|-----------|---------------------|
| L1.1 | MIMO Transfer Function Matrix | G(s) ∈ ℂ^{p×m}: Y(s) = G(s)U(s) | MIMOModel, MIMOTransferFunction |
| L1.2 | Steady-State Gain Matrix | K = G(0) ∈ ℝ^{p×m} | mimo_model_steady_state_gain() |
| L1.3 | RGA (Relative Gain Array) | λ_{ij} = K_{ij}·(K^{-1})_{ji} | RGAMatrix |
| L1.4 | Niederlinski Index | NI = det(K)/∏K_{ii} | mimo_niederlinski_index() |
| L1.5 | Condition Number | κ(K) = σ_max/σ_min | mimo_condition_number() |
| L1.6 | Decoupler Matrix | D(s) such that G(s)D(s) ≈ diag | Decoupler, DecouplerElement |
| L1.7 | SVD Decomposition | K = UΣV^T | SVDDecoupler |
| L1.8 | State-Space Model | ẋ=Ax+Bu, y=Cx+Du | MIMOStateSpace |
| L1.9 | McMillan Degree | Minimal state dimension | mimo_model_mcmillan_degree() |
| L1.10 | Dynamic RGA | DRGA(ω) = G(jω).∗(G(jω)^{-1})^T | DynamicRGA |

## L2: Core Concepts (Complete)

| ID | Concept | Implementation |
|----|---------|---------------|
| L2.1 | Interaction Analysis | mimo_interaction.c (RGA, NI, CN) |
| L2.2 | Static Decoupling | mimo_static_decoupling.c (D=K^{-1}) |
| L2.3 | Dynamic Decoupling | mimo_dynamic_decoupling.c (ideal/simplified/partial) |
| L2.4 | Inverted Decoupling | mimo_inverted_decoupling.c (feedforward structure) |
| L2.5 | SVD-Based Decoupling | mimo_svd_decoupling.c (principal directions) |
| L2.6 | Input-Output Pairing | mimo_enumerate_pairings() (Bristol rules) |
| L2.7 | Decentralized vs Centralized | mimo_control.c (decentralized PI, BLT) |
| L2.8 | Robustness Analysis | mimo_robustness.c (Monte Carlo, mu analysis) |

## L3: Engineering Structures (Complete)

| ID | Structure | Implementation |
|----|-----------|---------------|
| L3.1 | Transfer Function to State-Space | mimo_model_to_state_space() |
| L3.2 | Tustin Bilinear Discretization | mimo_ss_c2d_tustin() |
| L3.3 | Discrete-Time Decoupler | mimo_decoupler_c2d() |
| L3.4 | Algebraic Loop Resolution | mimo_inverted_decoupler_step() (fixed-point) |
| L3.5 | FIR Filter Implementation | mimo_dynamic_decoupler_step() states |

## L4: Engineering Laws/Theorems (Complete)

| ID | Theorem/Law | Reference | Verification |
|----|------------|-----------|-------------|
| L4.1 | Bristol RGA Theorem | Bristol (1966) | mimo_rga_compute() + test |
| L4.2 | Niederlinski Stability | Niederlinski (1971) | test_niederlinski |
| L4.3 | Routh-Hurwitz Criterion | Routh (1877) | mimo_routh_hurwitz() + test |
| L4.4 | Kalman Controllability | Kalman (1960) | mimo_ss_is_controllable() |
| L4.5 | Kalman Observability | Kalman (1960) | mimo_ss_is_observable() |
| L4.6 | Lyapunov Stability | Lyapunov (1892) | mimo_lyapunov_stability() |
| L4.7 | Integrity Theorem | Grosdidier et al. (1985) | mimo_check_integrity() |
| L4.8 | SVD Eckart-Young | Eckart-Young (1936) | mimo_svd_decompose() |

## L5: Algorithms/Methods (Complete)

| ID | Algorithm | Implementation |
|----|-----------|---------------|
| L5.1 | Static Decoupler D=K^{-1} | mimo_static_decoupler_design() |
| L5.2 | Ideal Dynamic Decoupler | mimo_ideal_dynamic_decoupler() |
| L5.3 | Simplified Dynamic Decoupler | mimo_simplified_dynamic_decoupler() |
| L5.4 | Partial (Band-Limited) Decoupler | mimo_partial_dynamic_decoupler() |
| L5.5 | Inverted Decoupler Design | mimo_inverted_decoupler_design() |
| L5.6 | IMC-Based Inverted Decoupling | mimo_inverted_imc_decoupler() |
| L5.7 | SVD Static Decoupler | mimo_svd_static_decoupler() |
| L5.8 | Moore-Penrose Pseudoinverse | mimo_static_decoupler_pseudoinv() |
| L5.9 | Pairing Enumeration (BFS) | mimo_enumerate_pairings() |
| L5.10 | BLT Tuning (Luyben 1986) | mimo_control.c |
| L5.11 | Monte Carlo Robustness | mimo_robustness.c |
| L5.12 | Pole Finding (Companion Matrix) | mimo_find_poles() |

## L6: Canonical Problems (Complete)

| ID | Problem | Example |
|----|---------|---------|
| L6.1 | Wood-Berry Distillation Column | example_distillation_column.c |
| L6.2 | CSTR Reactor Decoupling | example_reactor_decoupling.c |
| L6.3 | Heat Exchanger Network (3x3) | example_heat_exchanger.c |

## L7: Industrial Applications (Complete)

| ID | Application | Coverage |
|----|------------|----------|
| L7.1 | Distillation Column Top/Bottom Control | Wood-Berry model, full decoupling |
| L7.2 | Chemical Reactor Temperature Control | CSTR with inverted decoupling |
| L7.3 | Heat Exchanger Network | 3x3 coupling analysis |

## L8: Advanced Topics (Complete)

| ID | Topic | Implementation |
|----|-------|---------------|
| L8.1 | Structured Singular Value (mu) | mimo_mu_analysis() |
| L8.2 | Lyapunov Stability Verification | mimo_lyapunov_stability() (Bartels-Stewart) |
| L8.3 | Principal Gains Alignment | mimo_principal_gains_alignment() |
| L8.4 | Robustness to Gain Uncertainty | mimo_static_sensitivity() |
| L8.5 | Monte Carlo Uncertainty Analysis | mimo_robustness.c |

## L9: Research Frontiers (Partial)

| ID | Topic | Status |
|----|-------|--------|
| L9.1 | Data-Driven Decoupling | Documented, not implemented |
| L9.2 | Adaptive Decoupling | Documented, not implemented |
| L9.3 | Nonlinear Decoupling | Documented, not implemented |
