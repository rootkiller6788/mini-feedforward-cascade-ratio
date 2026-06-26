# Knowledge Graph - Split-Range Control Heat/Cool

## L1: Definitions (Complete - 10 typedefs/structs)

| # | Concept | C Definition | Lean Definition |
|---|---------|-------------|-----------------|
| 1 | Split-range mode | split_range_mode_t enum | SplitScheme structure |
| 2 | Valve action direction | split_range_action_t enum | isReverse : Bool field |
| 3 | Valve characteristic | split_range_valve_char_t enum | Nat-based mapping |
| 4 | Controller output (CO) | SPLIT_CO_MIN/MAX constants | ControllerOutput structure |
| 5 | Valve position | SPLIT_VALVE_MIN/MAX constants | ValvePosition structure |
| 6 | Split point | split_point field | splitPoint : Nat field |
| 7 | Deadband / Overlap | deadband_width / overlap_width | deadband : Nat field |
| 8 | PID parameters | split_range_pid_params_t struct | -- |
| 9 | Reactor model (CSTR) | split_range_reactor_model_t struct | ReactorParams structure |
| 10 | Health status (NAMUR) | split_range_health_t enum | -- |

## L2: Core Concepts (Complete - 10 functions)

| # | Concept | Implementation |
|---|---------|---------------|
| 1 | Split-range distribution | split_distribute_output() |
| 2 | Deadband transition | split_apply_deadband_overlap() |
| 3 | Bumpless transfer | split_pid_mode_transition() |
| 4 | Channel position mapping | split_compute_channel_position() |
| 5 | Anti-windup (conditional) | split_pid_incremental() |
| 6 | Anti-windup (back-calc) | split_pid_positional() |
| 7 | External reset (cascade) | split_pid_external_reset() |
| 8 | Valve char linearization | split_valve_characteristic_inverse() |
| 9 | Slew rate limiting | split_slew_rate_limit() |
| 10 | Hysteresis compensation | split_hysteresis_compensate() |

## L3: Engineering Structures (Complete - 8 items)

| # | Concept | Implementation |
|---|---------|---------------|
| 1 | Velocity-form PID | split_pid_incremental() |
| 2 | Positional-form PID | split_pid_positional() |
| 3 | PID state structure | split_range_pid_state_t |
| 4 | Split scheme config | split_range_scheme_t |
| 5 | Valve channel config | split_range_channel_t |
| 6 | Installed valve char | split_valve_installed_characteristic() |
| 7 | User-defined valve table | split_valve_build_user_table() |
| 8 | Binary search lookup | split_valve_user_table_lookup() |

## L4: Engineering Laws/Standards (Complete - 10 items)

| # | Law / Standard | Implementation |
|---|---------------|---------------|
| 1 | ISA-75.01 Valve Sizing | split_valve_size_isa() |
| 2 | ISA-75.01 Pressure Drop Ratio | split_valve_pressure_drop_ratio() |
| 3 | ISA-75.11 Rangeability | split_valve_rangeability() |
| 4 | IEC 60534 Installed Char | split_valve_installed_characteristic() |
| 5 | ISA-96.02.01 Partial Stroke Test | split_valve_partial_stroke_test() |
| 6 | NAMUR NE107 Health States | split_range_health_t enum |
| 7 | Ziegler-Nichols Tuning | split_pid_zn_tuning() |
| 8 | ISA-84.00.01/IEC 61511 SIF | split_reactor_emergency_cooling() |
| 9 | Semenov Runaway Criterion | semenovRunaway (Lean) |
| 10 | Pade Approximation | split_pid_closed_loop_poles() |

## L5: Algorithms/Methods (Complete - 8 algorithms)

| # | Algorithm | Implementation |
|---|-----------|---------------|
| 1 | Relay feedback auto-tuning | split_autotune_step() |
| 2 | IMC-PID tuning | split_imc_tuning() |
| 3 | Golden-section search | split_optimize_split_point() |
| 4 | Cubic pole solver (Cardano) | split_pid_closed_loop_poles() |
| 5 | Choudhury stiction model | split_valve_stiction_model() |
| 6 | Cross-coupling analysis | split_cross_coupling_analysis() |
| 7 | MCMC calibration | split_monte_carlo_calibration() |
| 8 | Cohen-Coon PID tuning | split_pid_zn_tuning() method=2 |

## L6: Canonical Problems (Complete - 3 examples + 2 safety)

| # | Problem | Example |
|---|---------|---------|
| 1 | Jacketed CSTR temperature | example_reactor_heat_cool.c |
| 2 | pH neutralization | example_ph_control.c |
| 3 | Reactor pressure vent/inert | example_pressure_split.c |
| 4 | Thermal runaway detection | split_reactor_runaway_detect() |
| 5 | Emergency cooling response | split_reactor_emergency_cooling() |

## L7: Industrial Applications (Complete - 3 applications)

| # | Application | Implementation |
|---|-------------|---------------|
| 1 | Pharma batch reactor temp | split_control_create_reactor() |
| 2 | Water treatment pH control | split_control_create_ph() |
| 3 | Reactor inert blanketing | split_control_create_pressure() |

## L8: Advanced Topics (Partial - 4/6 implemented)

| # | Topic | Implementation |
|---|-------|---------------|
| 1 | Lyapunov adaptive gain scheduling | split_adaptive_update_gain() |
| 2 | MCMC parameter calibration | split_monte_carlo_calibration() |
| 3 | Asymmetric relay auto-tuning | split_autotune_init() |
| 4 | Stiction detection | split_valve_stiction_model() |
| 5 | Bayesian optimization | Documented only |
| 6 | Reinforcement learning control | Documented only |

## L9: Industry Frontiers (Partial)

| # | Topic | Status |
|---|-------|--------|
| 1 | Digital twin consistency | Formal statement in Lean |
| 2 | Autonomous split optimization | Golden-section search implemented |
| 3 | IT/OT integrated analytics | Documented |

## L4: Lean Formalization Theorems

| # | Theorem | Statement |
|---|---------|-----------|
| 1 | Clamp idempotence | clamp_idempotent |
| 2 | Clamp lower bound | clamp_lower_bound |
| 3 | Valve position bounded | valve_position_bounded |
| 4 | Bumpless velocity form | bumpless_velocity_form |
| 5 | Deadband exclusion | deadband_exclusion |
| 6 | Complement symmetry | split_symmetry |
| 7 | Emergency cooling sufficient | emergency_cooling_sufficient |
| 8 | Adaptive gain bounded | adaptive_gain_bounded |
| 9 | MCMC acceptance bound | mcmc_acceptance_ratio_bound |
| 10 | Split scheme coverage | coverage_gap_free |
