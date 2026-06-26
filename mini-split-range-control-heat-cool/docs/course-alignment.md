# Course Alignment - mini-split-range-control-heat-cool

## MIT 6.302 - Feedback Systems
- Actuator saturation: split_slew_rate_limit()
- Anti-windup: split_pid_incremental(), split_pid_positional()
- Bumpless transfer: split_pid_mode_transition()
- Nonlinear stability: split_adaptive_update_gain()

## Stanford ENGR205 - Process Control
- Split-range strategies: split_init_heat_cool_scheme()
- PID velocity form: split_pid_incremental()
- IMC tuning: split_imc_tuning()
- CSTR control: example_reactor_heat_cool.c

## Purdue ME575 - Industrial Control
- Valve sequencing: split_compute_channel_position()
- Valve engineering: split_valve_size_isa() (ISA-75.01)
- Reactor safety: split_reactor_runaway_detect()
- Split-point opt: split_optimize_split_point()

## RWTH Aachen - Industrial Control Systems
- Stellgerate: split_range_channel_t
- Regelkreis: split_pid_control_cycle()
- Sicherheit: split_reactor_emergency_cooling()

## Tsinghua - Process Control Engineering
- Split-range: split_init_heat_cool_scheme()
- Reactor temp: example_reactor_heat_cool.c
- pH control: example_ph_control.c

## CMU 24-677 - Advanced Control
- Adaptive gain: split_adaptive_update_gain()
- Relay auto-tuning: split_autotune_step()
- MCMC calibration: split_monte_carlo_calibration()

## Georgia Tech ECE 6550 - Nonlinear Control
- Stiction: split_valve_stiction_model()
- Hysteresis: split_hysteresis_compensate()

## ISA/IEC Standards
- ISA-75.01.01: split_valve_size_isa()
- ISA-75.11.01: split_valve_rangeability()
- ISA-96.02.01: split_valve_partial_stroke_test()
- ISA-84/IEC 61511: split_reactor_emergency_cooling()
- NAMUR NE107: split_range_health_t
