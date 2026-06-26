# Knowledge Graph — mini-ratio-control-gas-liquid

## L1: Definitions ✅ Complete

| ID | Term | Definition | C Type | Lean Formalization |
|----|------|-----------|--------|--------------------|
| L1.1 | Ratio | R = F_slave / F_master | ratio_compute_basic() | Ratio structure |
| L1.2 | Ratio Setpoint | Target ratio R_sp | ratio_config_t.ratio_setpoint | RatioSetpoint structure |
| L1.3 | Master Flow | Wild/uncontrolled stream | ratio_control_state_t.master | Flow structure |
| L1.4 | Slave Flow | Controlled stream | ratio_control_state_t.slave | Flow structure |
| L1.5 | Wild Stream | Uncontrolled feed stream | ratio_mode_t.WILD_STREAM | — |
| L1.6 | Cross-Limiting | Safety selector logic for combustion | cross_limiting_t | CrossLimit structure |
| L1.7 | Stoichiometric AFR | Theoretical air-fuel ratio for complete combustion | stoichiometric_afr_get() | afr_stoich_natural_gas |
| L1.8 | Lambda (λ) | Excess air ratio λ = AFR/stoich | lambda_from_afr() | lambda function |
| L1.9 | Henry's Law | C_eq = kH * P_gas | henry_equilibrium() | HenryConstant structure |
| L1.10 | Gas-Liquid Ratio | GLR = Q_gas / Q_liquid | gl_reactor_steady_gas_flow() | gl_stoich_ratio |
| L1.11 | Void Fraction | α = Q_gas / (Q_gas + Q_liquid) | two_phase_void_fraction() | — |
| L1.12 | Blending Ratio | F_i / F_total = x_i | blending_ratio_setpoints() | blend_normalized |

## L2: Core Concepts ✅ Complete

| ID | Concept | Implementation |
|----|---------|---------------|
| L2.1 | Ratio Station | ratio_station() — feedforward multiplier |
| L2.2 | Master-Slave Architecture | ratio_control_state_t + ratio_compute_slave_setpoint() |
| L2.3 | Ratio Trim Feedback | ratio_trim_controller_t + ratio_trim_step() |
| L2.4 | Wild Stream Feedforward | ratio_feedforward_correction() |
| L2.5 | Cascade Ratio Control | cascade_ratio_adjust() + cascade_anti_windup_check() |
| L2.6 | Ratio Error | ratio_error_compute() + ratio_error_percent() |
| L2.7 | Ratio Safety Bounds | ratio_set_limits() + ratio_check_safety() |
| L2.8 | Cross-Limiting Safety | cross_limit_air_leads/fuel_leads/double() |

## L3: Engineering Structures ✅ Complete

| ID | Structure | Implementation |
|----|-----------|---------------|
| L3.1 | Lead-Lag Compensator | lead_lag_compensator_t + Tustin discretization |
| L3.2 | Butterworth Filter | butterworth_filter_t + bilinear transform |
| L3.3 | Deadtime Delay Buffer | deadtime_delay_t + circular buffer |
| L3.4 | Rate Limiter | rate_limit_apply() + ratio_ramp_compute() |
| L3.5 | Cross-Limiting Selector Chain | High/Low select logic in ratio_cross_limiting.c |
| L3.6 | Flow Unit Converter | flow_unit_convert() — mass/volume/molar/normalized |
| L3.7 | EWMA Flow Filter | flow_ewma_filter() — first-order exponential |

## L4: Engineering Laws ✅ Complete

| ID | Law/Theorem | C Implementation | Lean Proof |
|----|------------|-----------------|-------------|
| L4.1 | Ideal Gas Law: ρ = PM/RT | gas_density_ideal() | — |
| L4.2 | Henry's Law | henry_equilibrium() | henry_monotone theorem |
| L4.3 | Conservation of Mass | mass_transfer_rate() | ratio_scale_invariance |
| L4.4 | Souders-Brown Equation | souders_brown_velocity() | — |
| L4.5 | Lockhart-Martinelli | lockhart_martinelli_multiplier() | — |
| L4.6 | Siegert Efficiency Formula | combustion_efficiency_compute() | — |
| L4.7 | Ratio Clamp Idempotence | ratio_clamp() | clamp_idempotent theorem |
| L4.8 | Feedforward Cancellation | feedforward_design() | feedforward_cancellation theorem |
| L4.9 | Ratio Positivity | — | ratio_positivity |
| L4.10 | Cross-Limiting Safety | cross_limit_safe definition | cross_limit_implies_lean theorem |

## L5: Algorithms/Methods ✅ Complete

| ID | Algorithm | Implementation |
|----|-----------|---------------|
| L5.1 | Air-Leads Cross-Limiting | cross_limit_air_leads() |
| L5.2 | Fuel-Leads Cross-Limiting | cross_limit_fuel_leads() |
| L5.3 | Double Cross-Limiting | cross_limit_double() |
| L5.4 | Ratio Trim PI Control | ratio_trim_step() with anti-windup |
| L5.5 | Feedforward Design (FOPDT) | feedforward_design() |
| L5.6 | RLS Online Identification | rls_update() |
| L5.7 | Blend Cost Optimization (LP) | blend_optimizer_solve() |
| L5.8 | Dynamic Compensation | lead_lag_step() |
| L5.9 | Ratio Gain Scheduling | adaptive_trim_gain_schedule() |
| L5.10 | Oscillation Detection (ACF) | ratio_detect_oscillation() |

## L6: Canonical Problems ✅ Complete

| ID | Problem | Implementation |
|----|---------|---------------|
| L6.1 | Boiler Air-Fuel Ratio | example_air_fuel_ratio.c + cross-limiting |
| L6.2 | Gas-Liquid CSTR Reactor | example_gas_liquid_reactor.c + gl_reactor_dynamic_step() |
| L6.3 | Multi-Component Blending | example_blending_ratio.c + blend_optimizer |
| L6.4 | Gas Absorption Column | absorber_ntu() + absorber_dynamic_step() |
| L6.5 | Two-Phase Pipeline Flow | two_phase_flow_regime() + lockhart_martinelli_multiplier() |
| L6.6 | Gas-Liquid Separator | souders_brown_velocity() + separation_efficiency() |

## L7: Industrial Applications ✅ Partial+

| ID | Application | Implementation |
|----|-------------|---------------|
| L7.1 | Siemens PCS7 Combustion Control | Cross-limiting double mode (ISA-77.41.01 compatible) |
| L7.2 | Honeywell TDC3000 Blending | blend_optimizer with cost minimization |
| L7.3 | ABB Gas Flow Computer | gas_flow_normal_to_actual() + density compensation |
| L7.4 | Chemical GTL Process Control | gl_reactor_steady_gas_flow() + Henry's Law model |

## L8: Advanced Topics ✅ Partial+

| ID | Topic | Implementation |
|----|-------|---------------|
| L8.1 | RLS Adaptive Identification | rls_init/update/get_theta/reset |
| L8.2 | Adaptive Trim Gain Scheduling | adaptive_trim_gain_schedule() |
| L8.3 | Blend Cost Optimization | blend_ncomp_solve() (greedy heuristic) |
| L8.4 | Real-Time Density Compensation | density_compensate_gas/liquid() |
| L8.5 | Loop Oscillation Detection | ratio_detect_oscillation() via ACF |
| L8.6 | Process Change Detection | adaptive_detect_process_change() |

## L9: Research Frontiers ✅ Partial

| ID | Topic | Status |
|----|-------|--------|
| L9.1 | AI-based Ratio Self-Optimization | Documented concept; RLS provides foundation |
| L9.2 | Digital Twin for Gas-Liquid Systems | Dynamic models (CSTR, absorber) serve as twin core |
| L9.3 | IT/OT Fusion for Blending | Ratio performance metrics enable cloud analytics |
| L9.4 | Autonomous Combustion Optimization | Cross-limiting + adaptive trim = L3 autonomy |

---

**Knowledge Coverage Summary:**
- L1 Definitions: 12 items ✅
- L2 Core Concepts: 8 items ✅
- L3 Engineering Structures: 7 items ✅
- L4 Engineering Laws: 10 items ✅
- L5 Algorithms/Methods: 10 items ✅
- L6 Canonical Problems: 6 items ✅
- L7 Industrial Applications: 4 items ✅
- L8 Advanced Topics: 6 items ✅
- L9 Research Frontiers: 4 items (documented) ✅
