#include <stdio.h>
#include <math.h>
#include "../include/feedforward_defs.h"
#include "../include/feedforward_dynamic.h"
#include "../include/feedforward_advanced.h"

int main(void) {
    printf("=== pH Neutralization with Gain-Scheduled Feedforward ===\n\n");

    double ph_points[] = {2.0, 3.5, 5.0, 6.0, 6.5, 7.0, 7.5, 8.0, 9.5, 11.0, 13.0};
    double Kff_points[] = {0.01, 0.02, 0.05, 0.15, 0.50, 1.00, 0.60, 0.20, 0.05, 0.02, 0.01};
    int n_pts = 11;

    ff_gain_schedule_t schedule;
    ff_gain_schedule_init(&schedule, ph_points, Kff_points, n_pts);

    printf("Gain Schedule (Kff vs pH):\n");
    printf("  pH      Kff\n");
    for (int i = 0; i < n_pts; i++) {
        printf("  %5.1f   %.3f\n", ph_points[i], Kff_points[i]);
    }

    printf("\n=== Interpolated Lookup Test ===\n");
    double test_ph[] = {3.0, 5.5, 6.3, 6.8, 7.0, 7.2, 7.8, 9.0, 12.0};
    for (int i = 0; i < 9; i++) {
        double Kff = ff_gain_schedule_lookup(&schedule, test_ph[i]);
        printf("  pH=%.1f -> Kff=%.4f\n", test_ph[i], Kff);
    }

    printf("\n=== Disturbance Rejection Simulation ===\n");
    printf("Scenario: Influent flow +5 L/min step at t=10s\n\n");

    double pH = 7.0, pH_sp = 7.0;
    double tau_mix = 30.0, Ts = 0.5, t_sim = 200.0;

    lead_lag_t ll;
    lead_lag_init(&ll, 1.0, 60.0, 30.0, Ts);

    double e_int = 0.0, Kc = 0.5, Ti = 120.0;

    printf("Time[s]    pH      u_FF     u_total   Error\n");
    printf("--------  ------  -------  --------  ------\n");

    for (double t = 0; t <= t_sim; t += Ts) {
        double d_flow = (t >= 10.0) ? 5.0 : 0.0;
        double Kff_sch = ff_gain_schedule_lookup(&schedule, pH);

        ll.K_ll = Kff_sch;
        double u_ff = lead_lag_step(&ll, d_flow);

        double error = pH_sp - pH;
        e_int += error * Ts;
        double u_fb = Kc * (error + e_int / Ti);
        double u_total = u_fb + u_ff;

        double K_eff = (Kff_sch > 0.001) ? Kff_sch : 0.001;
        double dy = (-u_total * 0.2 * K_eff) / tau_mix;
        pH += dy * Ts;

        if ((int)t % 40 == 0 || t < 2.0 || (t >= 9.5 && t <= 11.0)) {
            printf("%7.1f   %6.2f  %7.3f  %8.3f  %6.3f\n",
                   t, pH, u_ff, u_total, error);
        }
    }

    printf("\n=== Analysis ===\n");
    printf("pH control requires gain scheduling due to extreme nonlinearity.\n");
    printf("Kff at pH=5.0: %.3f (low gain, flat titration)\n",
           ff_gain_schedule_lookup(&schedule, 5.0));
    printf("Kff at pH=7.0: %.3f (high gain, steep titration)\n",
           ff_gain_schedule_lookup(&schedule, 7.0));
    printf("Without scheduling, FF would be too aggressive off-neutral or too weak at pH 7.\n");

    ff_gain_schedule_free(&schedule);
    printf("\nExample complete.\n");
    return 0;
}