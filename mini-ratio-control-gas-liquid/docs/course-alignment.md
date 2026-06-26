# Course Alignment — mini-ratio-control-gas-liquid

## Nine-School Curriculum Mapping

### MIT — 6.302 Feedback Systems
- Feedforward control design → feedforward_design(), lead_lag_init()
- Disturbance rejection → ratio_feedforward_correction()
- 2-DOF control structures → ratio_trim_step() (outer loop)

### Stanford — ENGR205 Process Control
- Ratio control fundamentals → ratio_core.c (all functions)
- Cross-limiting combustion control → ratio_cross_limiting.c
- Blending control → blending_ratio_setpoints(), blend_optimizer_solve()

### Berkeley — ME233 Advanced Control / EE C128 Mechatronics
- Dynamic compensation → lead_lag_step(), butterworth_step()
- Digital implementation of compensators → Tustin discretization in lead_lag_init()
- Sensor signal conditioning → flow_ewma_filter(), ratio_filter_master()

### CMU — 24-677 Advanced Control Systems
- System identification → rls_update() (online RLS)
- Adaptive control → adaptive_trim_gain_schedule()
- Nonlinear process models → gl_reactor_dynamic_step()

### Georgia Tech — ECE 6550 / AE 6530
- Optimal estimation (RLS) → rls_update()
- Combustion dynamics → combustion_efficiency_compute()
- Two-phase flow regime detection → two_phase_flow_regime()

### Purdue — ME 575 Industrial Control
- Boiler combustion control → example_air_fuel_ratio.c
- Master-slave ratio architecture → ratio_control_init() through ratio_compute_slave_setpoint()
- ISA-77.41.01 boiler controls → cross_limit_double()

### RWTH Aachen — Industrial Control Systems
- IEC 61131-3 function blocks → ratio_station(), ratio_clamp() (standard blocks)
- Power plant automation → cross-limiting double mode
- Gas flow computation → gas_flow_normal_to_actual(), density_compensate_gas()

### Tsinghua — Process Control Engineering
- Ratio control design → ratio_config_t through ratio_control_state_t
- Chemical reactor control → gl_reactor_steady_gas_flow()
- Industrial blending → example_blending_ratio.c

### ISA/IEC Standards
- ISA-5.1 Instrumentation symbols → ratio_mode_t enumeration
- ISA-77.41.01 Boiler combustion → cross_limit_mode_t + cross_limit_double()
- ISA-88 Batch control (blending) → blending_ratio_validate()
- IEC 61131-3 PLC programming → modular function design pattern
- ISO 5024 Gas reference conditions → gas_flow_normalize()
