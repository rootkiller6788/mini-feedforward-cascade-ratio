/**
 * @file demo_deadtime_visual.c
 * @brief Visual demonstration of dead-time effects and Smith predictor benefits.
 *
 * Shows side-by-side comparison of:
 *   1. Standard PI controller on delayed process
 *   2. Smith predictor on the same process
 *
 * Demonstrates how dead time limits achievable bandwidth and how the
 * Smith predictor recovers the delay-free performance.
 *
 * The demo outputs CSV-compatible data for plotting.
 */

#include "smith_predictor.h"
#include "smith_tuning.h"
#include <stdio.h>
#include <math.h>

int main(void)
{
    printf("\n╔══════════════════════════════════════════════════════════╗\n");
    printf("║  Dead-Time Effect Visualization                          ║\n");
    printf("║  Process: K=1, tau=10s, theta=5s (theta/tau=0.5)         ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n\n");

    double T_sim = 200.0;
    double Ts = 0.1;
    int n_steps = (int)(T_sim / Ts);

    double K = 1.0, tau = 10.0, theta = 5.0;

    /* Delay buffer for process simulation */
    int delay_steps = (int)(theta / Ts);
    double *delay_buf = (double *)calloc(delay_steps + 1, sizeof(double));
    int buf_idx = 0;

    double y = 0.0;     /* Process output */
    double y_state = 0.0; /* Delay-free process state */

    /* Setpoint profile: step up, disturbance, step down */
    double sp;

    /* === Standard PI (tuned for delay-free process — will oscillate) === */
    printf("Standard PI on delayed process:\n");
    printf("Time,Setpoint,PV,Output\n");

    double Kp_pi = 1.0, Ti_pi = 10.0;
    double integrator_pi = 0.0;
    y = 0.0; y_state = 0.0;

    for (int k = 0; k < n_steps; k++) {
        double t = k * Ts;
        if (t < 20.0) sp = 10.0;
        else if (t < 80.0) sp = 10.0;
        else if (t < 120.0) sp = 15.0;
        else sp = 10.0;

        /* PI control */
        double e = sp - y;
        double p = Kp_pi * e;
        integrator_pi += Kp_pi * Ts / Ti_pi * e;
        double u = p + integrator_pi;

        /* Process simulation */
        double dy = (K * u - y_state) / tau * Ts;
        y_state += dy;
        delay_buf[buf_idx % (delay_steps + 1)] = y_state;
        buf_idx++;
        int d = delay_steps;
        y = delay_buf[(buf_idx - d + delay_steps + 1) % (delay_steps + 1)];

        if (k % 100 == 0) {
            printf("%.1f,%.1f,%.2f,%.2f\n", t, sp, y, u);
        }
    }

    /* === Smith Predictor === */
    printf("\nSmith Predictor on same process:\n");
    printf("Time,Setpoint,PV,Output,Prediction,Mismatch\n");

    smith_predictor_t sp_smith;
    smith_predictor_init_fopdt(
        &sp_smith, K, tau, theta, Ts,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);

    /* IMC tuning: lambda = theta/2 */
    smith_process_model_t model;
    model.order = SMITH_MODEL_FOPDT;
    model.fopdt.K = K; model.fopdt.tau = tau; model.fopdt.theta = theta;
    model.K_delay_free = K; model.tau_delay_free = tau;

    double Kp_s, Ti_s;
    smith_tune_imc_pi(&model, theta / 2.0, &Kp_s, &Ti_s);
    smith_predictor_set_pi(&sp_smith, Kp_s, Ti_s, 1.0);
    smith_predictor_set_robustness_filter(&sp_smith, theta / 2.0);

    printf("Smith tuning: Kp=%.2f, Ti=%.1f\n\n", Kp_s, Ti_s);

    y = 0.0; y_state = 0.0;
    buf_idx = 0;
    memset(delay_buf, 0, (delay_steps + 1) * sizeof(double));

    for (int k = 0; k < n_steps; k++) {
        double t = k * Ts;
        if (t < 20.0) sp = 10.0;
        else if (t < 80.0) sp = 10.0;
        else if (t < 120.0) sp = 15.0;
        else sp = 10.0;

        double u = smith_predictor_step(&sp_smith, sp, y);

        /* Process simulation */
        double dy = (K * u - y_state) / tau * Ts;
        y_state += dy;
        delay_buf[buf_idx % (delay_steps + 1)] = y_state;
        buf_idx++;
        int d = delay_steps;
        y = delay_buf[(buf_idx - d + delay_steps + 1) % (delay_steps + 1)];

        if (k % 100 == 0) {
            printf("%.1f,%.1f,%.2f,%.2f,%.2f,%.3f\n",
                   t, sp, y, u,
                   smith_predictor_get_prediction(&sp_smith),
                   smith_predictor_get_mismatch(&sp_smith));
        }
    }

    printf("\n=== Demo Complete ===\n");
    printf("Output format: CSV, suitable for plotting with Python/matplotlib.\n");

    smith_predictor_destroy(&sp_smith);
    free(delay_buf);
    return 0;
}
