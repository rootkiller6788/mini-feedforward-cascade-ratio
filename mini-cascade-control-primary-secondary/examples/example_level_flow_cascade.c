/**
 * @file example_level_flow_cascade.c
 * @brief L6 Example: Level-Flow Cascade — Tank Surge Control
 *
 * Demonstrates a level-flow cascade control:
 *   Primary (LC): Tank level → flow setpoint
 *   Secondary (FC): Outlet flow → control valve
 *
 * The level process is integrating (1/(A*s)), making it prone to
 * overshoot with single-loop PI. Cascade with a fast flow loop
 * provides superior level regulation and flow disturbance rejection.
 *
 * Reference: Seborg et al. (2016), Section 16.4.1
 * Curriculum: Purdue ME575, RWTH Aachen ICS
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "cascade_types.h"
#include "cascade_pid.h"
#include "cascade_control.h"
#include "cascade_tuning.h"

static double tank_level(double inflow, double outflow, double *level,
                          double area, double dt)
{
    /* dh/dt = (Qin - Qout) / A */
    *level += (inflow - outflow) / area * dt;
    if (*level < 0.0) *level = 0.0;
    if (*level > 100.0) *level = 100.0;
    return *level;
}

static double flow_process(double valve_pos, double *flow,
                            double max_flow, double dt)
{
    /* Fast flow loop: tau*dQ/dt + Q = max_flow * valve_pos/100 */
    double tau = 2.0;
    double dQ = (max_flow * valve_pos / 100.0 - *flow) / tau;
    *flow += dQ * dt;
    if (*flow < 0.0) *flow = 0.0;
    return *flow;
}

int main(void)
{
    printf("============================================================\n");
    printf("  Level-Flow Cascade Control: Tank Surge Control\n");
    printf("  Primary  (slow):  Tank level (integrating)\n");
    printf("  Secondary (fast): Outlet flow (τ ≈ 2 s)\n");
    printf("============================================================\n\n");

    /* Process parameters */
    double tank_area = 10.0;    /* m² */
    double max_flow = 50.0;     /* m³/h */
    double inflow = 20.0;       /* m³/h (constant feed) */

    /* Configure cascade */
    cascade_config_t cascade;
    cascade_fopdt_model_t pri = {.K=1.0, .tau=100.0, .theta=0.0, .type=CASCADE_MODEL_FOPDT};
    cascade_fopdt_model_t sec = {.K=0.5, .tau=2.0, .theta=1.0, .type=CASCADE_MODEL_FOPDT};
    cascade_pair_init(&cascade, &pri, &sec);

    /* Tune for level-flow */
    cascade.primary.params.kp = 0.5;
    cascade.primary.params.ti = 200.0;
    cascade.primary.params.beta = 0.5;
    cascade.secondary.params.kp = 2.0;
    cascade.secondary.params.ti = 5.0;

    cascade.primary_sp = 50.0;  /* Target 50% level */
    cascade.primary_sp_min = 0.0;
    cascade.primary_sp_max = 100.0;
    cascade.secondary_sp_min = 0.0;
    cascade.secondary_sp_max = max_flow;

    /* Initial conditions */
    double level = 30.0;   /* Start below setpoint */
    double flow = 20.0;    /* Steady state flow */
    cascade.secondary_sp = flow;
    cascade.secondary_pv = flow;
    cascade.primary_pv = level;

    cascade_set_mode(&cascade, CASCADE_MODE_CASCADE);

    printf("Time[s]  Level[%%]  Flow[m³/h]  SP2  MV  Mode\n");
    printf("------  --------  ----------  ---  ---  -------\n");

    double total_time = 600.0;
    double dt = 0.2;

    for (double t = 0.0; t < total_time; t += dt) {
        /* Secondary flow loop */
        cascade.secondary_pv = flow;
        cascade_execute_secondary(&cascade);
        double valve_cmd = cascade.secondary_co;
        flow = flow_process(valve_cmd, &flow, max_flow, dt);

        /* Primary level loop (runs every 5 dt) */
        if ((int)(t / dt) % 5 == 0) {
            cascade.primary_pv = level;
            cascade_execute_primary(&cascade);
        }

        /* Tank level dynamics */
        tank_level(inflow, flow, &level, tank_area, dt);

        if ((int)t % 50 == 0) {
            printf("%6.0f  %8.1f  %10.1f  %3.0f  %3.0f  %s\n",
                   t, level, flow,
                   cascade.secondary_sp, cascade.secondary_co,
                   cascade.mode == CASCADE_MODE_CASCADE ? "CASCADE" : "AUTO");
        }
    }

    /* Inflow disturbance */
    printf("\n--- Feed flow disturbance (+50%%) at t=600s ---\n");
    inflow = 30.0;

    for (double t = 0.0; t < 300.0; t += dt) {
        cascade.secondary_pv = flow;
        cascade_execute_secondary(&cascade);
        flow = flow_process(cascade.secondary_co, &flow, max_flow, dt);

        if ((int)(t / dt) % 5 == 0) {
            cascade.primary_pv = level;
            cascade_execute_primary(&cascade);
        }

        tank_level(inflow, flow, &level, tank_area, dt);

        if ((int)t % 50 == 0) {
            printf("%6.0f  %8.1f  %10.1f  %3.0f  %3.0f  %s\n",
                   600 + t, level, flow,
                   cascade.secondary_sp, cascade.secondary_co, "CASCADE");
        }
    }

    printf("\nFinal level: %.1f%% (target: 50.0%%)\n", level);
    printf("Level error: %.1f%%\n", fabs(level - 50.0));
    printf("Performance: %s\n", fabs(level - 50.0) < 3.0 ? "EXCELLENT" : "ACCEPTABLE");

    return 0;
}
