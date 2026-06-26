/**
 * @file example_chemical_reactor.c
 * @brief Chemical reactor temperature control with measurement delay.
 *
 * L6 Canonical Problem: CSTR jacket temperature control.
 *
 * In a Continuous Stirred Tank Reactor (CSTR), the temperature sensor
 * is often placed in the outlet pipe, introducing a measurement delay.
 * The Smith predictor compensates for this to enable tight temperature
 * control essential for product quality and safety.
 *
 * Model: SOPDT K=1.8°C/%, tau1=20s, tau2=5s, theta=10s
 *
 * This demonstrates the Smith predictor with PID control for SOPDT.
 *
 * Course: MIT 6.302, Stanford ENGR205, Purdue ME 575
 */

#include "smith_predictor.h"
#include "smith_tuning.h"
#include <stdio.h>
#include <math.h>

/* SOPDT reactor process simulator */
static double reactor_process(double u, double *y1, double *y2, double *buf, int bs, int *bi)
{
    double K = 1.8, tau1 = 20.0, tau2 = 5.0, theta = 10.0, Ts = 1.0;

    /* Cascade of two first-order systems */
    double dy1 = (K * u - *y1) / tau1 * Ts;
    *y1 += dy1;
    double dy2 = (*y1 - *y2) / tau2 * Ts;
    double y_fresh = *y2 + dy2;
    *y2 = y_fresh;

    /* Transport delay */
    int d = (int)(theta / Ts);
    buf[*bi % bs] = y_fresh;
    *bi = (*bi + 1) % bs;
    return buf[(*bi - d + bs) % bs];
}

int main(void)
{
    printf("\n╔══════════════════════════════════════════════════╗\n");
    printf("║  CSTR Reactor Temperature — Dead Time 10s        ║\n");
    printf("║  Smith Predictor with PID (SOPDT Model)           ║\n");
    printf("╚══════════════════════════════════════════════════╝\n\n");

    double Ts = 1.0;

    smith_predictor_t sp;
    smith_predictor_init_sopdt(
        &sp, 1.8, 20.0, 5.0, 0.0, 0.0, 10.0, Ts,
        SMITH_VARIANT_FILTERED, 0.0, 100.0);

    /* IMC-PID tuning for SOPDT */
    smith_process_model_t m;
    m.order = SMITH_MODEL_SOPDT;
    m.sopdt.K = 1.8; m.sopdt.tau1 = 20.0; m.sopdt.tau2 = 5.0; m.sopdt.theta = 10.0;
    double Kp, Ti, Td;
    smith_tune_imc_pid(&m, 5.0, &Kp, &Ti, &Td);
    smith_predictor_set_pid(&sp, Kp, Ti, Td, 10.0, 0.8, 0.0);

    /* Robustness filter */
    double Fr;
    smith_tune_robustness_filter(&m, 0.2, 2.0, &Fr);
    smith_predictor_set_robustness_filter(&sp, Fr);

    printf("Model: K=1.8, tau1=20s, tau2=5s, theta=10s\n");
    printf("PID: Kp=%.3f, Ti=%.1f, Td=%.1f, Fr=%.1f\n\n", Kp, Ti, Td, Fr);

    double y = 80.0, y1 = 80.0, y2 = 80.0;
    double buf[50] = {80.0};
    int buf_idx = 0;

    printf("Time(s)   SP(°C)    PV(°C)    CO(%%)    Mismatch\n");
    for (int k = 0; k < 600; k += 30) {
        double setpoint = (k < 300) ? 90.0 : 85.0;
        double u = smith_predictor_step(&sp, setpoint, y);
        y = reactor_process(u, &y1, &y2, buf, 50, &buf_idx);

        if (k % 60 == 0) {
            printf("%6.0f    %6.1f   %7.2f   %7.2f   %8.3f\n",
                   k * Ts, setpoint, y, u, smith_predictor_get_mismatch(&sp));
        }
    }

    printf("\nFinal temperature: %.2f°C (setpoint: 85.0°C)\n", y);
    printf("Prediction error:  %.3f°C\n", smith_predictor_get_mismatch(&sp));

    smith_predictor_destroy(&sp);
    return 0;
}
