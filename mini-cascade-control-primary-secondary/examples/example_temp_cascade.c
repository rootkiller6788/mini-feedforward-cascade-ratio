/**
 * @file example_temp_cascade.c
 * @brief L6 Example: Temperature Cascade — Jacket + Reactor Control
 *
 * Demonstrates a classic temperature cascade control:
 *   Primary (TC): Reactor temperature → jacket temperature SP
 *   Secondary (TC): Jacket temperature → heating/cooling valve
 *
 * Process models:
 *   Jacket:  G_p2(s) = 1.5 / (2s + 1) * exp(-0.5s)  [°C/%]
 *   Reactor: G_p1(s) = 1.0 / (60s + 1) * exp(-5s)   [°C/°C]
 *
 * This example simulates a setpoint change from 25°C to 80°C and
 * demonstrates how the cascade handles disturbances to the jacket
 * temperature that would take much longer to correct in a single loop.
 *
 * Curriculum: MIT 6.302, Stanford ENGR205, Purdue ME575
 * Reference: Seborg et al. (2016), Section 16.4
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "cascade_types.h"
#include "cascade_pid.h"
#include "cascade_control.h"
#include "cascade_tuning.h"

static double process_jacket(double u, double *Tj, double T_amb, double dt)
{
    /* Simple 1st-order jacket model: tau*dTj/dt + Tj = K*u + T_amb */
    double K = 1.5, tau = 2.0;
    double dTj = (K * u + T_amb - *Tj) / tau;
    *Tj += dTj * dt;
    return *Tj;
}

static double process_reactor(double Tj, double *Tr, double Q_rxn, double dt)
{
    /* Simple 1st-order reactor model with heat from jacket */
    double K = 1.0, tau = 60.0;
    double dTr = (K * Tj + Q_rxn - *Tr) / tau;
    *Tr += dTr * dt;
    return *Tr;
}

int main(void)
{
    printf("============================================================\n");
    printf("  Temperature Cascade Control: Jacket + Reactor\n");
    printf("  Primary  (slow):  Reactor temperature  τ ≈ 60 s\n");
    printf("  Secondary (fast): Jacket temperature   τ ≈ 2 s\n");
    printf("============================================================\n\n");

    /* Configure cascade pair */
    cascade_config_t cascade;
    cascade_fopdt_model_t pri_model = {.K=1.0, .tau=60.0, .theta=5.0, .type=CASCADE_MODEL_FOPDT};
    cascade_fopdt_model_t sec_model = {.K=1.5, .tau=2.0, .theta=0.5, .type=CASCADE_MODEL_FOPDT};
    cascade_pair_init(&cascade, &pri_model, &sec_model);

    /* Tune with SIMC */
    cascade_tuning_result_t tune;
    cascade_tune_sequential(&sec_model, &pri_model, 2, &tune);
    printf("Tuning method: %s\n", tune.method_name);
    printf("Primary PID:   Kp=%.3f, Ti=%.1f s, Td=%.1f s\n",
           tune.primary_params.kp, tune.primary_params.ti, tune.primary_params.td);
    printf("Secondary PID: Kp=%.3f, Ti=%.1f s\n",
           tune.secondary_params.kp, tune.secondary_params.ti);
    printf("Update ratio:  %.0f:1\n\n", tune.recommended_update_ratio);

    /* Apply tuned parameters */
    cascade.primary.params = tune.primary_params;
    cascade.secondary.params = tune.secondary_params;
    cascade.primary.sample_time = 1.0;
    cascade.secondary.sample_time = 0.2;
    cascade.update_ratio = 5.0;
    cascade.secondary_sp_min = 0.0;
    cascade.secondary_sp_max = 150.0;
    cascade.primary_sp_min = 0.0;
    cascade.primary_sp_max = 100.0;

    /* Initial conditions */
    double Tr = 25.0;  /* Reactor temperature [°C] */
    double Tj = 25.0;  /* Jacket temperature [°C] */
    double T_amb = 25.0;
    double Q_rxn = 0.0;

    cascade.primary_sp = 80.0;  /* Target reactor temperature */
    cascade.primary_pv = Tr;
    cascade.secondary_pv = Tj;
    cascade.secondary_sp = Tj;

    cascade_set_mode(&cascade, CASCADE_MODE_CASCADE);

    printf("Time[s]  T_reactor  T_jacket  SP2    MV2    Mode\n");
    printf("------  ---------  --------  -----  -----  -------\n");

    double total_time = 300.0;
    double dt_primary = 1.0;
    double dt_secondary = 0.2;
    int steps = (int)(total_time / dt_primary);

    for (int k = 0; k < steps; k++) {
        /* Secondary loop runs 5x per primary update */
        for (int j = 0; j < 5; j++) {
            cascade.secondary_pv = Tj;
            cascade_execute_secondary(&cascade);
            double u = cascade.secondary_co;

            /* Apply to jacket process */
            Tj = process_jacket(u, &Tj, T_amb, dt_secondary);
        }

        /* Primary loop update */
        cascade.primary_pv = Tr;
        cascade_execute_primary(&cascade);

        /* Apply jacket temperature to reactor (via primary SP→secondary SP→jacket→reactor) */
        Tr = process_reactor(Tj, &Tr, Q_rxn, dt_primary);

        if (k % 10 == 0) {
            printf("%6.1f  %9.1f  %8.1f  %5.1f  %5.1f  %s\n",
                   k * dt_primary, Tr, Tj,
                   cascade.secondary_sp, cascade.secondary_co,
                   cascade.mode == CASCADE_MODE_CASCADE ? "CASCADE" : "AUTO");
        }
    }

    /* Simulate a jacket disturbance at t=200 */
    printf("\n--- Jacket cooling disturbance at t=200s ---\n");
    T_amb = 10.0;  /* Cold ambient temperature shock */

    for (int k = 0; k < 100; k++) {
        for (int j = 0; j < 5; j++) {
            cascade.secondary_pv = Tj;
            cascade_execute_secondary(&cascade);
            Tj = process_jacket(cascade.secondary_co, &Tj, T_amb, dt_secondary);
        }
        cascade.primary_pv = Tr;
        cascade_execute_primary(&cascade);
        Tr = process_reactor(Tj, &Tr, Q_rxn, dt_primary);

        if (k % 10 == 0) {
            printf("%6.1f  %9.1f  %8.1f  %5.1f  %5.1f  %s\n",
                   200 + k * dt_primary, Tr, Tj,
                   cascade.secondary_sp, cascade.secondary_co,
                   cascade.mode == CASCADE_MODE_CASCADE ? "CASCADE" : "AUTO");
        }
    }

    printf("\nFinal reactor temperature: %.1f °C (target: 80.0 °C)\n", Tr);
    double error = fabs(Tr - 80.0);
    printf("Steady-state error: %.1f °C\n", error);
    printf("Cascade performance: %s\n", error < 2.0 ? "EXCELLENT" : "ACCEPTABLE");

    return 0;
}
