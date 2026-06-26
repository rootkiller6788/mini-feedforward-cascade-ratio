/**
 * @file example_ph_control.c
 * @brief Example: pH Neutralization with Acid/Base Split-Range Control
 *
 * Demonstrates pH control using a split-range acid/base scheme.
 * pH control is notoriously difficult due to the highly nonlinear
 * titration curve.  This example shows how split-range overlap
 * helps smooth the transition near the neutral point.
 *
 * Reference:
 *   McMillan (2005) "pH Measurement and Control"
 *   Seborg et al. (2016) Process Dynamics and Control, Ch. 16
 *
 * Compile: gcc -I../include -o example_ph_control example_ph_control.c ../src/*.c -lm
 */

#include <stdio.h>
#include <math.h>
#include "split_range_control.h"

/**
 * Simplified pH process model: titration curve simulation.
 *
 * The pH of a weak acid/base mixture is modeled by the charge balance
 * and equilibrium relations.  For simplicity, we use a sigmoid function
 * that approximates the steep titration curve near pH 7.
 */
static double simulate_ph(double current_ph, double acid_flow, double base_flow,
                          double dt) {
    /* Acid addition decreases pH; base addition increases pH */
    double acid_effect = -0.1 * acid_flow * dt;
    double base_effect = 0.1 * base_flow * dt;

    /* Buffer effect: pH changes more slowly near pH 7 (high buffer capacity) */
    double buffer_gain = 0.5 + 4.5 / (1.0 + exp(-(current_ph - 7.0) * 2.0));

    double dph = (acid_effect + base_effect) / buffer_gain;
    return current_ph + dph;
}

int main(void) {
    printf("============================================\n");
    printf("  Example: pH Neutralization Control\n");
    printf("  Split-Range Acid/Base Scheme\n");
    printf("============================================\n\n");

    /* Create pH controller (factory settings include overlap) */
    split_range_controller_t ctrl = split_control_create_ph();

    /* Adjust PID for this specific process */
    ctrl.pid_params.kc = 3.0;
    ctrl.pid_params.ti = 200.0;
    ctrl.pid_params.td = 40.0;
    ctrl.pid_params.sample_time_sec = 0.5;

    /* Setpoint: neutralize to pH 7.0 */
    ctrl.pv_context.setpoint = 7.0;
    split_pid_reset_state(&ctrl.pid_state);
    ctrl.enabled = true;

    /* Initial conditions: acidic solution at pH 4.0 */
    double ph = 4.0;
    double dt = 0.5;
    double sim_time = 300.0;

    printf("Initial pH: %.1f, Setpoint: %.1f\n", ph, ctrl.pv_context.setpoint);
    printf("Overlap width: %.1f%% (for smooth acid/base transition)\n",
           ctrl.scheme.overlap_width);
    printf("PID: Kc=%.2f, Ti=%.0fs, Td=%.0fs\n",
           ctrl.pid_params.kc, ctrl.pid_params.ti, ctrl.pid_params.td);
    printf("\nTime(s)  pH      SP    CO(%)  Acid(%) Base(%%)  Overlap?\n");
    printf("-------  ----    ---   -----  ------- -------  --------\n");

    for (double t = 0.0; t <= sim_time; t += dt) {
        /* Disturbance: at t=150s, add acid spike to test rejection */
        if (t >= 150.0 && t < 160.0) {
            ph -= 0.2 * dt; /* acid disturbance: pH drops */
        }

        ctrl.pv_context.process_variable = ph;
        ctrl.pv_context.pv_filtered = ph;
        split_control_execute(&ctrl, dt);

        /* Simulate pH response */
        double acid_valve = ctrl.split_outputs[0] / 100.0;
        double base_valve = ctrl.split_outputs[1] / 100.0;
        ph = simulate_ph(ph, acid_valve, base_valve, dt);

        /* Check if both valves are overlapping */
        bool overlap = (ctrl.split_outputs[0] > 1.0 && ctrl.split_outputs[1] > 1.0);

        if ((int)t % 20 == 0) {
            printf("%7.0f  %5.2f  %4.1f  %5.1f  %7.1f  %7.1f  %s\n",
                   t, ph, ctrl.pv_context.setpoint,
                   ctrl.controller_output,
                   ctrl.split_outputs[0], ctrl.split_outputs[1],
                   overlap ? "YES" : "no");
        }
    }

    /* Performance evaluation */
    split_range_performance_t perf;
    split_control_evaluate(&ctrl, &perf);
    printf("\n--- Performance ---\n");
    printf("Final pH:           %.2f\n", ph);
    printf("Steady-state error: %.3f pH\n", perf.steady_state_error);
    printf("Overshoot:          %.1f%%\n", perf.overshoot_pct);
    printf("Cross-coupling:     %.4f\n", perf.cross_coupling_index);

    /* Cross-coupling analysis */
    double cc = split_cross_coupling_analysis(&ctrl);
    if (cc > 0.01) {
        printf("Note: Overlap region active (%.1f%% coupling) — "
               "this is expected for pH control\n", cc * 100.0);
    }

    printf("\nExample complete.\n");
    return 0;
}
