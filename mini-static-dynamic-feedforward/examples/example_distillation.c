#include <stdio.h>
#include <math.h>
#include "../include/feedforward_defs.h"
#include "../include/feedforward_static.h"
#include "../include/feedforward_dynamic.h"
#include "../include/feedforward_models.h"
#include "../include/feedforward_combined.h"

int main(void) {
    printf("=== Distillation Column Feedforward Control Example ===\n\n");

    fopdt_t proc;
    fopdt_init(&proc, 0.05, 300.0, 120.0);
    dist_model_t dist;
    dist_model_init(&dist, 0.02, 240.0, 90.0);

    printf("Process (top comp -> reflux): Kp=%.3f, tau=%.0fs, theta=%.0fs\n",
           proc.Kp, proc.tau, proc.theta);
    printf("Disturbance (feed rate -> comp): Kd=%.3f, tau_d=%.0fs, theta_d=%.0fs\n",
           dist.Kd, dist.tau_d, dist.theta_d);

    double T_lead, T_lag, Kff;
    ff_dynamic_design_fopdt(&proc, &dist, ACTION_DIRECT, &T_lead, &T_lag, &Kff);
    printf("\nDynamic FF: Kff=%.4f, T_lead=%.0fs, T_lag=%.0fs\n", Kff, T_lead, T_lag);

    double extra_delay = ff_dynamic_required_delay(proc.theta, dist.theta_d);
    if (extra_delay > 0) {
        printf("Non-causal: adding %.0fs extra lag\n", extra_delay);
        T_lag += extra_delay;
    }

    feedforward_t ff_dyn;
    feedforward_configure_dynamic(&ff_dyn, Kff, T_lead, T_lag, 100.0, -500.0, 500.0,
                                  ACTION_DIRECT, 1.0);

    double lambda = proc.tau * 0.5;
    double Kc = proc.tau / (fabs(proc.Kp) * (lambda + proc.theta));
    double Ti = proc.tau;
    printf("PI: Kc=%.4f, Ti=%.0fs\n\n", Kc, Ti);

    double Ts = 2.0, t_sim = 2000.0;
    int n = (int)(t_sim / Ts) + 1;

    double y_fb = 0.0, y_ff = 0.0;
    double e_int_fb = 0.0, e_int_ff = 0.0;
    double ISE_fb = 0.0, ISE_ff = 0.0;
    double peak_fb = 0.0, peak_ff = 0.0;

    for (int k = 0; k < n; k++) {
        double t = k * Ts;
        double d = (t >= 100.0) ? 10.0 : 0.0;

        /* Feedback Only */
        double e_fb = 0.0 - y_fb;
        e_int_fb += e_fb * Ts;
        double u_fb = Kc * (e_fb + e_int_fb / Ti);
        double dy_fb = (proc.Kp * u_fb + dist.Kd * d - y_fb) / proc.tau;
        y_fb += dy_fb * Ts;
        ISE_fb += e_fb * e_fb * Ts;
        if (fabs(y_fb) > peak_fb) peak_fb = fabs(y_fb);

        /* Feedforward + Feedback */
        double e_ff = 0.0 - y_ff;
        e_int_ff += e_ff * Ts;
        double u_fb_ff = Kc * (e_ff + e_int_ff / Ti);
        double u_ff_dyn = feedforward_step(&ff_dyn, d);
        double u_total = u_fb_ff + u_ff_dyn;
        double dy_ff = (proc.Kp * u_total + dist.Kd * d - y_ff) / proc.tau;
        y_ff += dy_ff * Ts;
        ISE_ff += e_ff * e_ff * Ts;
        if (fabs(y_ff) > peak_ff) peak_ff = fabs(y_ff);
    }

    printf("=== Results ===\n");
    printf("                    FB Only       FF+FB        Improvement\n");
    printf("ISE:               %8.4f     %8.4f     %6.1f%%\n",
           ISE_fb, ISE_ff, (1.0 - ISE_ff/ISE_fb)*100.0);
    printf("Peak Deviation:    %8.4f     %8.4f     %6.1f%%\n",
           peak_fb, peak_ff, (1.0 - peak_ff/peak_fb)*100.0);

    printf("\n=== Analysis ===\n");
    printf("Column time constants are very long (tau=%.0fs).\n", proc.tau);
    printf("Feedforward reduces both transient and steady-state errors.\n");
    printf("Residual error from dead-time mismatch: theta_p=%.0f vs theta_d=%.0f.\n",
           proc.theta, dist.theta_d);

    printf("\nExample complete.\n");
    return 0;
}