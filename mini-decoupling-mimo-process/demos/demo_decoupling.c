/**
 * @file demo_decoupling.c
 * @brief Interactive demonstration of MIMO decoupling control.
 *
 * Simulates a 2x2 Wood-Berry distillation column with and without
 * decoupling, showing the effect on setpoint tracking and disturbance
 * rejection through console-based visualization.
 */

#include <stdio.h>
#include <math.h>
#include "../include/mimo_model.h"
#include "../include/mimo_interaction.h"
#include "../include/mimo_static_decoupling.h"
#include "../include/mimo_dynamic_decoupling.h"

static void simulate_step(double K[4], double *u, double *y,
                           double dt, int n_steps) {
    printf("Time      y1(XD)    y2(XB)    u1(R)     u2(S)\n");
    printf("--------  --------  --------  --------  --------\n");

    /* Simple Euler integration for FOPDT dynamics (approximate) */
    double y1 = 0.0, y2 = 0.0;
    double tau1 = 16.7, tau2 = 14.4;
    double theta1 = 1.0, theta3 = 7.0;

    for (int k = 0; k < n_steps; k++) {
        double dy1 = (K[0]*u[0] + K[1]*u[1] - y1) / tau1;
        double dy2 = (K[2]*u[0] + K[3]*u[1] - y2) / tau2;
        y1 += dy1 * dt;
        y2 += dy2 * dt;
        if (k % 10 == 0) {
            printf("%8.2f  %8.4f  %8.4f  %8.4f  %8.4f\n",
                   k * dt, y1, y2, u[0], u[1]);
        }
    }
    y[0] = y1; y[1] = y2;
}

int main(void) {
    printf("===============================================================\n");
    printf("  MIMO Decoupling Control — Interactive Demonstration\n");
    printf("  Wood-Berry Distillation Column\n");
    printf("===============================================================\n\n");

    /* Create model */
    MIMOModel wb;
    mimo_model_init(&wb, 2, 2, "Wood-Berry Demo");

    mimo_model_set_fopdt(&wb, 0, 0, 12.8, 16.7, 1.0);
    mimo_model_set_fopdt(&wb, 0, 1, -18.9, 21.0, 3.0);
    mimo_model_set_fopdt(&wb, 1, 0, 6.6, 10.9, 7.0);
    mimo_model_set_fopdt(&wb, 1, 1, -19.4, 14.4, 3.0);

    /* RGA Analysis */
    double K[4];
    mimo_model_steady_state_gain(&wb, K);
    RGAMatrix rga;
    mimo_rga_compute(K, 2, &rga);

    printf("RGA Analysis:\n");
    printf("  RGA = [%7.4f  %7.4f]\n", rga.rga[0][0], rga.rga[0][1]);
    printf("        [%7.4f  %7.4f]\n", rga.rga[1][0], rga.rga[1][1]);
    printf("  Interpretation: %s\n\n", rga.interpretation);

    /* Design decoupler */
    StaticDecoupler sd;
    mimo_static_decoupler_design(&wb, &sd);

    printf("Static Decoupler D = K^{-1}:\n");
    printf("  D = [%7.4f  %7.4f]\n", sd.K_inv[0][0], sd.K_inv[0][1]);
    printf("      [%7.4f  %7.4f]\n\n", sd.K_inv[1][0], sd.K_inv[1][1]);

    /* Setpoint change: increase XD by +0.1 */
    printf("--- Scenario: Setpoint Change in Overhead Composition (+0.1) ---\n\n");

    /* Without decoupling */
    printf("WITHOUT decoupling (direct MV action):\n");
    double u_no_dec[] = { 0.1, 0.0 };
    double y_no_dec[2];
    simulate_step(K, u_no_dec, y_no_dec, 0.5, 200);
    printf("  Steady state: XD = %.4f, XB = %.4f\n", y_no_dec[0], y_no_dec[1]);
    printf("  Interaction: XB change = %.4f (should be 0 for perfect decoupling)\n\n",
           y_no_dec[1]);

    /* With decoupling */
    printf("WITH static decoupling (u = D * uc):\n");
    double uc_dec[] = { 0.1, 0.0 };
    double u_with_dec[2];
    mimo_static_decoupler_apply(&sd, uc_dec, u_with_dec);
    double y_with_dec[2];
    simulate_step(K, u_with_dec, y_with_dec, 0.5, 200);
    printf("  Controller output: uc = [%.4f, %.4f]\n", uc_dec[0], uc_dec[1]);
    printf("  Process input:     u  = [%.4f, %.4f]\n", u_with_dec[0], u_with_dec[1]);
    printf("  Steady state: XD = %.4f, XB = %.4f\n", y_with_dec[0], y_with_dec[1]);
    printf("  Residual interaction: XB = %.6f\n\n", y_with_dec[1]);

    /* Quantify improvement */
    double reduction = fabs((fabs(y_no_dec[1]) - fabs(y_with_dec[1]))
                            / fmax(fabs(y_no_dec[1]), 1e-6)) * 100.0;
    printf("Interaction reduction: %.1f%%\n", reduction);
    printf("Condition number of K: %.4f\n", sd.condition_number);

    /* Robustness */
    double sensitivity = mimo_static_sensitivity(&wb, &sd, 0.15);
    printf("Robustness (15%% gain error): residual = %.6f\n", sensitivity);

    printf("\n===============================================================\n");
    printf("  Demo complete.\n");
    printf("===============================================================\n");

    return 0;
}
