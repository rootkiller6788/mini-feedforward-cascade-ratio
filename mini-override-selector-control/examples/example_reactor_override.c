/**
 * @file example_reactor_override.c
 * @brief L6 Example: Reactor Temperature Override with Cooling Water Constraint
 *
 * An exothermic CSTR reactor has a primary temperature controller. A
 * low-select override limits the cooling water valve to prevent thermal
 * shock. Demonstrates the classic reactor temperature override pattern.
 *
 * Reference:
 *   Luyben, W.L. (2007). Chemical Reactor Design and Control. Wiley. Ch. 8.
 *
 * Knowledge Coverage: L6 — Reactor temperature override
 * Course: Purdue ME 575, MIT 6.302, Tsinghua process control
 */

#include <stdio.h>
#include <math.h>
#include "override_core.h"
#include "override_pid.h"

int main(void) {
    printf("=== Reactor Temperature Override Control ===\n\n");

    override_controller_t temp_ctrl, valve_ctrl;
    override_controller_init(&temp_ctrl, 0, "TIC-101",
                             "Reactor Temperature", PRIORITY_PRIMARY);
    override_controller_init(&valve_ctrl, 1, "VPC-101",
                             "Valve Position Limit", PRIORITY_CONSTRAINT);

    temp_ctrl.params.Kc = 2.0;
    temp_ctrl.params.Ti = 120.0;
    temp_ctrl.params.Td = 15.0;
    temp_ctrl.params.u_min = 0.0;
    temp_ctrl.params.u_max = 100.0;
    temp_ctrl.params.tracking_gain = 1.0;
    temp_ctrl.active = 1;

    valve_ctrl.params.Kc = 0.5;
    valve_ctrl.params.Ti = 60.0;
    valve_ctrl.params.Td = 0.0;
    valve_ctrl.params.u_min = 0.0;
    valve_ctrl.params.u_max = 80.0;
    valve_ctrl.active = 0;

    printf("Controllers:\n");
    printf("  [0] Temperature (Primary): Kc=%.1f Ti=%.1f\n",
           temp_ctrl.params.Kc, temp_ctrl.params.Ti);
    printf("  [1] Valve Position (Override): Kc=%.1f Ti=%.1f Max=%.1f%%\n\n",
           valve_ctrl.params.Kc, valve_ctrl.params.Ti, valve_ctrl.params.u_max);

    double reactor_temp = 85.0;
    double temp_setpoint = 80.0;
    double dt = 1.0;

    printf("Time(s)  Temp     SP       T-Out    V-Limit  Selected  Mode\n");
    printf("-------  -------  -------  -------  -------  -------  -----------\n");

    for (int t = 0; t <= 30; t += 1) {
        double t_out = override_pid_update(&temp_ctrl, temp_setpoint,
                                           reactor_temp, dt);
        valve_ctrl.pv = t_out;
        valve_ctrl.active = 1;
        double v_out = override_pid_update(&valve_ctrl, 80.0, t_out, dt);

        double selected = (t_out < v_out) ? t_out : v_out;
        const char *mode = (selected == v_out) ? "OVERRIDE" : "PRIMARY";

        printf("%7d  %7.1f  %7.1f  %7.1f  %7.1f  %7.1f  %s\n",
               t, reactor_temp, temp_setpoint, t_out, v_out, selected, mode);

        /* Simulate reactor cooling */
        if (t < 15) reactor_temp -= 0.3 * (selected / 100.0) * (reactor_temp - 20.0) / 10.0;
        else reactor_temp += 0.5;
    }

    printf("\nLow-select override clamps cooling demand to protect the system.\n");
    printf("\n=== Example Complete ===\n");
    return 0;
}
