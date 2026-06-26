/**
 * @file example_pressure_split.c
 * @brief Example: Pressure Control with Vent/Inert Split-Range
 *
 * Demonstrates reactor headspace pressure control using a split-range
 * scheme with vent valve and inert gas makeup valve.  This is a
 * common configuration in batch reactors where pressure must be
 * maintained within a narrow band.
 *
 * Reference:
 *   Seborg et al. (2016) Process Dynamics and Control, Ch. 13
 *   ISA-77.44.01 — Fossil Fuel Plant Steam Temperature Controls
 *
 * Compile: gcc -I../include -o example_pressure_split example_pressure_split.c ../src/*.c -lm
 */

#include <stdio.h>
#include <math.h>
#include "split_range_control.h"

/**
 * Simplified pressure dynamics model.
 * dP/dt = (gas_in - gas_out + reaction_gas) * R*T / V
 *
 * where gas_in from inert valve, gas_out through vent valve,
 * reaction_gas from chemical reaction (if any).
 */
static double simulate_pressure(double current_pressure,
                                double vent_flow, double inert_flow,
                                double reaction_rate, double dt) {
    double gas_in = inert_flow * 0.05;   /* bar/s at full flow */
    double gas_out = vent_flow * 0.08;   /* bar/s at full flow */
    double react = reaction_rate * 0.02; /* bar/s from reaction */

    double dpdt = gas_in - gas_out + react;
    double new_p = current_pressure + dpdt * dt;

    if (new_p < 0.0) new_p = 0.0;
    return new_p;
}

int main(void) {
    printf("============================================\n");
    printf("  Example: Reactor Pressure Control\n");
    printf("  Split-Range Vent/Inert Gas Scheme\n");
    printf("============================================\n\n");

    /* Create pressure controller */
    split_range_controller_t ctrl = split_control_create_pressure();

    ctrl.pid_params.kc = 2.0;
    ctrl.pid_params.ti = 20.0;
    ctrl.pid_params.td = 3.0;
    ctrl.pv_context.setpoint = 2.5; /* 2.5 barg */
    split_pid_reset_state(&ctrl.pid_state);
    ctrl.enabled = true;

    double pressure = 1.0; /* Start at 1.0 barg (below SP) */
    double reaction_gas_gen = 2.0; /* steady gas generation from reaction */
    double dt = 0.2;
    double sim_time = 120.0;

    printf("Initial pressure: %.1f barg, SP: %.1f barg\n", pressure, ctrl.pv_context.setpoint);
    printf("Channels: Vent(CO=0->50%, rev), Inert(CO=50->100%, dir)\n");
    printf("Deadband: %.1f%%\n", ctrl.scheme.deadband_width);
    printf("\nTime(s)  Press    SP     CO(%)   Vent(%) Inert(%)  Gas Gen\n");
    printf("-------  ------   ----   -----   ------- --------  -------\n");

    for (double t = 0.0; t <= sim_time; t += dt) {
        /* At t=60s, reduce gas generation (reaction slowing down) */
        if (t > 60.0) reaction_gas_gen = 0.5;

        /* At t=90s, spike gas generation (exotherm causing more gas) */
        if (t >= 90.0 && t < 100.0) reaction_gas_gen = 5.0;

        ctrl.pv_context.process_variable = pressure;
        ctrl.pv_context.pv_filtered = pressure;

        /* Feedforward: use reaction gas generation as feedforward signal */
        ctrl.pv_context.feedforward_signal = (reaction_gas_gen - 2.0) * 20.0;
        ctrl.pv_context.feedforward_enabled = true;

        split_control_execute(&ctrl, dt);

        /* Simulate pressure response */
        double vent_frac = ctrl.split_outputs[0] / 100.0;
        double inert_frac = ctrl.split_outputs[1] / 100.0;
        pressure = simulate_pressure(pressure, vent_frac, inert_frac,
                                     reaction_gas_gen, dt);

        if ((int)(t * 5) % 20 == 0) {
            printf("%7.1f  %6.2f  %5.1f  %6.1f  %7.1f  %7.1f  %6.1f\n",
                   t, pressure, ctrl.pv_context.setpoint,
                   ctrl.controller_output,
                   ctrl.split_outputs[0], ctrl.split_outputs[1],
                   reaction_gas_gen);
        }
    }

    /* Summary */
    split_range_performance_t perf;
    split_control_evaluate(&ctrl, &perf);
    printf("\n--- Performance ---\n");
    printf("Final pressure:     %.2f barg\n", pressure);
    printf("Steady-state error: %.3f barg\n", perf.steady_state_error);
    printf("Valve reversal count (vent):  %.0f\n", perf.valve_reversal_count[0]);
    printf("Valve reversal count (inert): %.0f\n", perf.valve_reversal_count[1]);

    /* Scheme validation */
    int valid = split_control_validate_scheme(&ctrl.scheme);
    printf("Scheme validation: %s\n", (valid == 0) ? "PASS" : "FAIL");

    printf("\nExample complete.\n");
    return 0;
}
