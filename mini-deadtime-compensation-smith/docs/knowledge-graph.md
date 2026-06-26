# Knowledge Graph — Smith Predictor Dead-Time Compensation

## L1: Definitions
- **Smith Predictor**: Control structure that separates dead time from process dynamics (Smith 1957). Implemented in smith_types.h and smith_predictor.c.
- **FOPDT Model**: First-Order Plus Dead Time: G(s)=K*exp(-theta*s)/(tau*s+1). smith_fopdt_model_t struct.
- **SOPDT Model**: Second-Order Plus Dead Time. smith_sopdt_model_t with overdamped/underdamped variants.
- **Relative Dead Time**: theta/tau — determines whether Smith predictor is needed (<0.1: PI sufficient, >1.0: Smith essential).
- **Delay-Free Model**: Gp(s) — the rational part without dead time, used for primary controller design.
- **Prediction Error**: ep = y_measured - yp_delayed — real-time model accuracy indicator.
- **Smith Feedback**: y_fb = yp_model + (y_measured - yp_delayed) — the key structural insight.
- **Sampling Period Ts**: Digital implementation step size, must satisfy Ts <= theta/5 for adequate resolution.
- **Discretization Methods**: Forward/Backward Euler, Tustin, ZOH, FOH — 5 methods in smith_types.h enum.
- **Robustness Filter**: F_r(s)=1/(Tr*s+1) — Normey-Rico et al. (1997) filter on prediction error path.

## L2: Core Concepts
- **Dead-Time Compensation**: Structural separation of delay from dynamics — the Smith predictor's fundamental contribution.
- **Internal Model Principle**: Controller contains an explicit process model running in parallel.
- **Prediction Feedback**: The primary controller sees yp_model (delay-free estimate), not the delayed measurement.
- **Model Mismatch Correction**: ep = y - ym compensates for model inaccuracy and unmeasured disturbances.
- **Disturbance Rejection**: Filtered Smith predictor (SMITH_VARIANT_FILTERED) improves load disturbance response.
- **Two-Degree-of-Freedom (2-DOF)**: Independent tuning of setpoint response (b parameter) and disturbance rejection.
- **Setpoint Weighting**: b in [0,1] reduces proportional kick on setpoint changes.
- **Bumpless Transfer**: Integrator tracks manual output in MANUAL mode for smooth AUTO transition.

## L3: Engineering Structures
- **Circular Delay Buffer**: Ring buffer O(d) memory, O(1) push/pop implementing z^(-d). delay_buffer_alloc/push/free.
- **5 Discretization Methods**: Forward Euler, Backward Euler, Tustin (default), ZOH, FOH — stability/clarity trade-offs.
- **Fractional Delay Interpolation**: Linear interpolation for non-integer theta/Ts ratios. delay_buffer_push with delay_frac.
- **Back-Calculation Anti-Windup**: Tracking time Tt=sqrt(Ti*Td), integrator correction when saturated.
- **Rate Limiting**: |u(k)-u(k-1)| <= rate_limit * Ts — actuator protection.
- **Derivative on Measurement**: Avoids derivative kick from setpoint changes.
- **Filtered Derivative (Tustin)**: D(s)=s*Kp*Td/(1+s*Td/N), discretized for digital implementation.
- **Conditional Integration**: Integrator frozen when actuator saturates (alternative anti-windup strategy).

## L4: Engineering Laws / Theorems
- **Smith Theorem (1957)**: If Gp_model*exp(-theta_model*s) == Gp_true*exp(-theta_true*s), the closed-loop characteristic equation is 1+C(s)*Gp(s)=0 (delay-free!). Formalized in smith_formal.lean.
- **IMC PI Theorem (Rivera-Morari-Skogestad 1986)**: IMC filter f(s)=1/(lambda*s+1) yields C(s)=PI with Kc=tau/(K*lambda), Ti=tau. smith_tune_imc_pi().
- **SIMC Tuning (Skogestad 2003)**: Kc=tau/(K*Tc), Ti=min(tau,4*Tc). smith_tune_simc_pi().
- **Routh-Hurwitz Stability**: FOPDT+PI stable iff Kp*K > 0, Ti*tau > 0. fopdt_pi_stability_condition in Lean.
- **Jury Stability Test**: Discrete 2nd-order: |trace| < 1+det, |det| < 1. juryStable2 in Lean, smith_robustness_lyapunov_stable() in C.
- **Small-Gain Theorem (Zames 1966)**: Robust stability if |T(jw)| < 1/delta_K for all w. smith_robustness_gain_uncertainty().
- **Bode Integral Theorem**: Sensitivity peak Ms >= 1 always. smith_robustness_peak_sensitivity().
- **Normey-Rico Filter Rule (1997)**: Tr >= theta/2 for FOPDT Smith predictor robustness. smith_tune_robustness_filter().

## L5: Algorithms/Methods
- **Two-Point FOPDT Identification (Hoopes)**: tau=1.5*(t63-t28), theta=t63-tau. smith_identify_step_fopdt().
- **Area Method FOPDT**: tau = integral of (1-y_norm)dt. Fallback when two-point fails.
- **SOPDT Identification**: Overdamped (Harriott 1964 t35/t85) and underdamped (overshoot zeta/omega_n). smith_identify_step_sopdt().
- **Relay Feedback (Astrom-Hagglund 1984)**: Ku=4d/(pi*a), Tu from oscillation. smith_identify_relay_fopdt().
- **Recursive Least Squares (RLS)**: Online ARX parameter estimation with exponential forgetting. smith_rls_init/update/to_fopdt().
- **CUSUM Change Detection**: Cumulative sum for detecting model parameter shifts. smith_adaptive_detect_change().
- **ISE-Optimal PI**: Golden-section search minimizing J=int e^2 dt. smith_tune_optimal_ise_pi().
- **MIT Rule Adaptation**: Gradient descent on J=0.5*e^2. smith_adaptive_gradient_Kp().
- **Lyapunov-Based Adaptation**: V=e^2+(Kp-Kp*)^2/gamma with V_dot<=0. smith_adaptive_lyapunov_update().
- **Gain Scheduling for Time-Varying Delay**: Kc(theta)=Kc_nom*theta_nom/theta. smith_tune_gain_schedule() (declared).

## L6: Canonical Problems
- **Heat Exchanger Temperature Control**: 30s transport delay, tau=60s. examples/example_heat_exchanger.c — SIMC-tuned Smith predictor.
- **Distillation Composition Control**: 60s analyzer delay, inverse response (K<0). examples/example_distillation.c — IMC-tuned with sign inversion.
- **Pipeline Flow Control**: 250s transport delay, tau=5s, ratio=50. examples/example_flow_control.c — extreme dead-time case.

## L7: Industrial Applications
- **Modbus RTU/TCP**: Holding register mapping for PLC/SCADA communication. smith_predictor_map_modbus(), write_modbus().
- **OPC UA (IEC 62541)**: Node ID mapping with namespace. smith_predictor_map_opcua().
- **PLC Integration Patterns**: Function block for Siemens TIA Portal PID_Compact, Rockwell PIDE.
- **ISA-101 HMI**: Predictor output and mismatch indicators for operator displays.

## L8: Advanced Topics
- **Adaptive Smith Predictor**: Online RLS + automatic controller redesign. smith_adaptive_step/redesign().
- **Monte Carlo Robustness Verification**: Random parameter sampling within uncertainty bounds. smith_robustness_monte_carlo().
- **Structured Singular Value (mu) Concept**: Combined gain+delay uncertainty check. smith_robustness_combined().
- **Model-Reference Adaptive System (MRAS)**: Reference model 1/(T_ref*s+1). smith_adaptive_mras_step().
- **Lyapunov Discrete Stability**: Jury test on discretized closed-loop. smith_robustness_lyapunov_stable().

## L9: Research Frontiers (Documented Only)
- Reinforcement Learning for auto-tuning Smith predictor parameters
- Gaussian Process models for nonlinear dead-time compensation
- Neural network-based prediction of time-varying delays
- Digital twin integration for real-time model updating
- 5G-enabled remote Smith predictor with ultra-low-latency wireless control
