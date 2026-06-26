# Knowledge Graph — mini-static-dynamic-feedforward

## L1: Definitions (Complete)

| # | Term | C Definition | Description |
|---|------|-------------|-------------|
| 1 | Feedforward Control | `feedforward_t` | Open-loop compensation using measured disturbances |
| 2 | Static Feedforward | `ff_mode_t::FF_MODE_STATIC` | Steady-state gain-based compensation |
| 3 | Dynamic Feedforward | `ff_mode_t::FF_MODE_DYNAMIC` | Transient compensation via lead-lag |
| 4 | Combined FF+FB | `ff_mode_t::FF_MODE_COMBINED` | Static + dynamic + PID feedback |
| 5 | Transfer Function | `tf_t` | Rational function s-domain model |
| 6 | FOPDT Model | `fopdt_t` | First-Order Plus Dead Time |
| 7 | SOPDT Model | `sopdt_t` | Second-Order Plus Dead Time |
| 8 | IPDT Model | `ipdt_t` | Integrating Plus Dead Time |
| 9 | Lead-Lag | `lead_lag_t` | G(s)=K(T_lead*s+1)/(T_lag*s+1) |
| 10 | Discrete TF | `tf_discrete_t` | Pulse transfer function G(z) |
| 11 | Disturbance Model | `dist_model_t` | Gd(s) = Kd*e^(-theta*s)/(tau*s+1) |
| 12 | Signal Quality | `signal_status_t` | ISA-18.2 alarm management compatible |
| 13 | Performance Metrics | `ff_performance_t` | ISE, variance reduction, settling time |
| 14 | Gain Schedule | `ff_gain_schedule_t` | Operating-point-dependent FF gain |
| 15 | Kalman Filter | `ff_kalman_dist_t` | State estimation for unmeasured disturbances |
| 16 | Iterative Learning | `ff_ilc_t` | Batch-to-batch feedforward improvement |

## L2: Core Concepts (Complete)

| # | Concept | Implementation |
|---|---------|---------------|
| 1 | Static FF law: u_ff = Kff * d + bias | `ff_static_step()` |
| 2 | Ideal FF gain: Kff = -Kd/Kp | `ff_static_gain_fopdt()` |
| 3 | Dynamic FF: Gff(s) = -Gd(s)/Gp(s) | `ff_dynamic_design_fopdt()` |
| 4 | Combined FF+FB: u = u_fb + u_ff | `feedforward_step_with_feedback()` |
| 5 | Disturbance Rejection Ratio | `ff_static_rejection_ratio()` |
| 6 | Gain mismatch residual | `ff_static_mismatch_residual()` |
| 7 | Causality condition | `ff_dynamic_is_causal()` |
| 8 | Bumpless transfer | `feedforward_bumpless_transfer()` |
| 9 | Mode management | `feedforward_set_mode()` |

## L3: Engineering Structures (Complete)

| # | Structure | Implementation |
|---|-----------|---------------|
| 1 | Continuous TF representation | `tf_t` struct |
| 2 | Discrete TF (pulse TF) | `tf_discrete_t` struct |
| 3 | Lead-lag Tustin discretization | `lead_lag_step()` |
| 4 | 2nd-order lead-lag | `lead_lag2_step()` |
| 5 | ZOH discretization | `fopdt_to_discrete_zoh()` |
| 6 | Tustin discretization | `fopdt_to_discrete_tustin()` |
| 7 | Backward Euler discretization | `fopdt_to_discrete_euler()` |
| 8 | Circular buffer for discrete TF | `dtf_step()` |
| 9 | Signal quality validation | `disturbance_validate()` |

## L4: Engineering Laws (Complete)

| # | Law/Theorem | Code Verification |
|---|------------|------------------|
| 1 | Perfect FF condition: Gff = -Gd/Gp | `ff_dynamic_design_fopdt()` |
| 2 | Steady-state rejection: e_ss = (Kd+Kp*Kff)*d | `ff_static_mismatch_residual()` |
| 3 | FOPDT step response: y(t) = Kp*u*(1-e^(-(t-theta)/tau)) | `fopdt_step_response()` (tested) |
| 4 | SOPDT step response (analytical) | `sopdt_step_response()` (tested) |
| 5 | Pade approximation: e^(-ts) ~ (1-ts/2)/(1+ts/2) | `pade_first_order()` |
| 6 | Frequency response: G(jw) = K*e^(-jwt)*N(jw)/D(jw) | `tf_frequency_response()` |
| 7 | R-squared model fit | `model_r_squared()` |
| 8 | Internal Model Principle (IMC tuning) | `ff_direct_synthesis_fopdt()` |

## L5: Algorithms/Methods (Complete)

| # | Algorithm | Implementation |
|---|-----------|---------------|
| 1 | FOPDT identification (Sundaresan-Krishnaswamy) | `fopdt_identify_step()` |
| 2 | Two-point identification (Astrom-Hagglund) | `fopdt_identify_two_point()` |
| 3 | Area method identification | `fopdt_identify_area()` |
| 4 | SOPDT Smith identification | `sopdt_identify_step()` |
| 5 | Direct synthesis FF design | `ff_direct_synthesis_fopdt()` |
| 6 | Pattern search optimization (Hooke-Jeeves) | `ff_tune_pattern_search()` |
| 7 | Bilinear lead-lag discretization | `lead_lag_step()` |
| 8 | SOPDT FF design | `ff_dynamic_design_sopdt()` |
| 9 | NMP factorization | `ff_factor_minimum_phase()` |
| 10 | Kalman filter (2-state) | `ff_kalman_dist_step()` |
| 11 | ILC P-type update with Q-filter | `ff_ilc_update()` |

## L6: Canonical Problems (Complete)

| # | Problem | Example |
|---|---------|---------|
| 1 | Heat exchanger temperature control | `example_heat_exchanger.c` |
| 2 | Distillation column composition control | `example_distillation.c` |
| 3 | pH neutralization (gain-scheduled) | `example_ph_neutralization.c` |
| 4 | DC motor speed control (Toyota assembly) | `example_motor.c` |
| 5 | Boiler drum level (inverse response) | `feedforward_applications.c::boiler_level_ff_*` |

## L7: Industrial Applications (Complete)

| # | Application | Implementation |
|---|------------|---------------|
| 1 | Heat exchanger (Seborg et al. case study) | `heat_exchanger_ff_*()` |
| 2 | Distillation column (Skogestad model) | `distillation_ff_*()` |
| 3 | Gain-scheduled pH (CSTR neutralization) | `ph_ff_*()` |
| 4 | Boiler drum level (Fukushima/power plant) | `boiler_level_ff_*()` |
| 5 | Toyota DC motor servo assembly | `dc_motor_ff_*()` |

## L8: Advanced Topics (Complete)

| # | Topic | Implementation |
|---|-------|---------------|
| 1 | Non-minimum-phase FF | `ff_is_non_minimum_phase()`, `ff_factor_minimum_phase()` |
| 2 | Kalman disturbance estimation | `ff_kalman_dist_*()` (2-state KF) |
| 3 | Feedforward with actuator limits | `feedforward_with_limits()` |
| 4 | Robustness gain margin analysis | `ff_robustness_gain_margin()` |
| 5 | Disturbance sensitivity analysis | `ff_disturbance_sensitivity()` |
| 6 | Pattern search tuning (Hooke-Jeeves) | `ff_tune_pattern_search()` |

## L9: Industry Frontiers (Partial)

| # | Topic | Status |
|---|-------|--------|
| 1 | Iterative Learning Control (ILC) | Implemented: `ff_ilc_*()` |
| 2 | Adaptive feedforward (gain scheduling) | Implemented: `ff_gain_schedule_*()` |
| 3 | Digital twin for FF optimization | Documented only |
| 4 | Edge AI disturbance prediction | Documented only |
| 5 | IT/OT convergence for FF data pipelines | Documented only |