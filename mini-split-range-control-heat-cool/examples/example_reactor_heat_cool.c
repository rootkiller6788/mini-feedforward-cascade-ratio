/**
 * @file example_reactor_heat_cool.c
 * @brief Example: Reactor Temperature Control with Heat/Cool Split-Range
 *
 * Demonstrates a complete reactor temperature control loop using
 * split-range heating and cooling.  Simulates a jacketed CSTR
 * with PID control and split-range valve distribution.
 *
 * This is a canonical L6 problem from:
 *   - MIT 6.302: Jacketed reactor temperature control
 *   - Stanford ENGR205: CSTR thermal management
 *   - Purdue ME575: Reactor unit operations
 *   - Tsinghua: 反应器温度控制
 *
 * Reference: Fogler (2016) Elements of Chemical Reaction Engineering
 *            Myke King (2016) Process Control: A Practical Approach
 *
 * Compile: gcc -I../include -o example_reactor_heat_cool example_reactor_heat_cool.c ../src/*.c -lm
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "split_range_control.h"

int main(void) {
    printf("============================================\n");
    printf("  Example: Reactor Temperature Control\n");
    printf("  Split-Range Heat/Cool — Jacketed CSTR\n");
    printf("============================================\n\n");

    /* Create reactor temperature controller */
    split_range_controller_t ctrl = split_control_create_reactor();

    /* Initialize reactor model (200L pilot reactor) */
    split_range_reactor_model_t reactor = {0};
    reactor.V = 200.0;       /* liters */
    reactor.rho = 1000.0;    /* kg/m3 */
    reactor.Cp = 4200.0;     /* J/(kg*K) */
    reactor.U_times_A = 5000.0; /* W/K heat transfer */
    reactor.delta_Hr = -50000.0; /* J/mol (exothermic) */
    reactor.k0 = 1.0e8;      /* 1/s pre-exponential */
    reactor.Ea = 70000.0;    /* J/mol activation energy */
    reactor.R_gas = 8.314;   /* J/(mol*K) */
    reactor.CA = 100.0;      /* mol/m3 reactant concentration */
    reactor.F = 0.01;        /* m3/s feed flow */
    reactor.Tin = 25.0;      /* degC feed temperature */
    reactor.T = 25.0;        /* degC initial reactor temperature */
    reactor.T_ambient = 20.0; /* degC ambient */
    reactor.Tj_min = 5.0;
    reactor.Tj_max = 150.0;
    reactor.Q_heater_max = 100000.0; /* W */
    reactor.Q_cooler_max = 80000.0;  /* W */
    reactor.inflection_temp = 60.0;

    /* Set control parameters */
    ctrl.pv_context.setpoint = 80.0; /* Target: 80 degC */
    ctrl.pid_params.kc = 3.0;
    ctrl.pid_params.ti = 90.0;
    ctrl.pid_params.td = 20.0;
    ctrl.pid_params.sample_time_sec = 1.0;
    split_pid_reset_state(&ctrl.pid_state);
    ctrl.enabled = true;

    double dt = 1.0; /* 1 second sample time */
    double sim_time = 600.0; /* 10 minutes simulation */

    printf("Reactor: %.0f L CSTR, SP=%.0f degC, T_init=%.0f degC\n",
           reactor.V, ctrl.pv_context.setpoint, reactor.T);
    printf("PID: Kc=%.2f, Ti=%.0fs, Td=%.0fs, Ts=%.0fs\n",
           ctrl.pid_params.kc, ctrl.pid_params.ti,
           ctrl.pid_params.td, ctrl.pid_params.sample_time_sec);
    printf("\nTime(s)  Temp(C)   SP(C)    CO(%)   Heat(%)  Cool(%%)  Runaway?\n");
    printf("-------  -------   -----    -----   -------  -------  --------\n");

    for (double t = 0.0; t <= sim_time; t += dt) {
        /* Update PV in controller */
        ctrl.pv_context.process_variable = reactor.T;
        ctrl.pv_context.pv_filtered = reactor.T;

        /* Execute control cycle */
        split_control_execute(&ctrl, dt);

        /* Check for runaway */
        reactor.has_runaway_risk = (reactor.T > reactor.inflection_temp);
        bool runaway = split_reactor_runaway_detect(&reactor, dt);

        if (runaway) {
            printf("*** THERMAL RUNAWAY DETECTED at t=%.0fs ***\n", t);
            split_reactor_emergency_cooling(&ctrl);
            printf("Emergency cooling activated!\n");
            break;
        }

        /* Simulate reactor response */
        double q_heat = ctrl.split_outputs[0] / 100.0; /* heating fraction */
        double q_cool = ctrl.split_outputs[1] / 100.0; /* cooling fraction */
        split_reactor_simulate_step(&reactor, q_heat, q_cool, dt);

        /* Print status every 30 seconds */
        if ((int)t % 30 == 0) {
            printf("%7.0f  %7.1f   %5.0f    %5.1f   %7.1f  %7.1f  %s\n",
                   t, reactor.T, ctrl.pv_context.setpoint,
                   ctrl.controller_output,
                   ctrl.split_outputs[0], ctrl.split_outputs[1],
                   runaway ? "YES!!!" : "no");
        }
    }

    /* Evaluate control performance */
    split_range_performance_t perf;
    split_control_evaluate(&ctrl, &perf);

    printf("\n--- Performance Summary ---\n");
    printf("Final temperature:  %.1f degC\n", reactor.T);
    printf("Setpoint:           %.1f degC\n", ctrl.pv_context.setpoint);
    printf("Steady-state error: %.2f degC\n", perf.steady_state_error);
    printf("Overshoot:          %.1f%%\n", perf.overshoot_pct);
    printf("Cross-coupling:     %.4f (%.2f%% energy waste)\n",
           perf.cross_coupling_index, perf.cross_coupling_index * 100.0);
    printf("Split efficiency:   %.2f%%\n", perf.split_efficiency_index * 100.0);
    printf("Heating energy:     %.1f kW-s\n", perf.energy_consumption_heating);
    printf("Cooling energy:     %.1f kW-s\n", perf.energy_consumption_cooling);
    printf("Valve travel (heat): %.1f%%\n", perf.total_valve_travel[0]);
    printf("Valve travel (cool): %.1f%%\n", perf.total_valve_travel[1]);

    /* Optimize split point for energy efficiency */
    printf("\n--- Split Point Optimization ---\n");
    double dummy_cost(double co, double sp, double aux) {
        (void)aux;
        return fabs(co - sp) * 10.0; /* simple penalty for deviation */
    }
    double optimal_sp = split_optimize_split_point(&ctrl, dummy_cost, 0.5);
    printf("Current split point: %.1f%%\n", ctrl.scheme.split_point);
    printf("Optimal split point: %.1f%%\n", optimal_sp);

    printf("\nExample complete.\n");
    return 0;
}
