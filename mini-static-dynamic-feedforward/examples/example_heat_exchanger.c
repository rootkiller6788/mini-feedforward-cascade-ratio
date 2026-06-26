#include <stdio.h>
#include <math.h>
#include "../include/feedforward_defs.h"
#include "../include/feedforward_static.h"
#include "../include/feedforward_dynamic.h"
#include "../include/feedforward_models.h"
#include "../include/feedforward_combined.h"

/**
 * @example example_heat_exchanger.c
 * @brief End-to-end heat exchanger temperature control with static+dynamic FF
 *
 * L6 Canonical Problem: Shell-and-tube heat exchanger outlet temperature control
 *
 * Process: Gp(s) = -0.8 * e^(-30s) / (120s + 1)   [C per %valve]
 * Disturbance: Gd(s) = 0.15 * e^(-15s) / (90s + 1)  [C per kg/s flow]
 *
 * Scenario: Inlet flow rate increases by 2 kg/s at t=50s.
 * Compare: (1) no FF, (2) static FF only, (3) static + dynamic FF
 */

int main(void) {
    printf("=== Heat Exchanger Feedforward Control Example ===\n\n");

    /* Process and disturbance models */
    fopdt_t proc;
    fopdt_init(&proc, -0.8, 120.0, 30.0);
    dist_model_t dist;
    dist_model_init(&dist, 0.15, 90.0, 15.0);

    printf("Process: Kp=%.2f, tau=%.0fs, theta=%.0fs\n", proc.Kp, proc.tau, proc.theta);
    printf("Disturbance: Kd=%.2f, tau_d=%.0fs, theta_d=%.0fs\n\n", dist.Kd, dist.tau_d, dist.theta_d);

    /* Design feedforward */
    double T_lead, T_lag, Kff;
    ff_dynamic_design_fopdt(&proc, &dist, ACTION_DIRECT, &T_lead, &T_lag, &Kff);
    printf("FF Design: Kff=%.4f, T_lead=%.1fs, T_lag=%.1fs\n", Kff, T_lead, T_lag);

    /* Causality check */
    int causal = ff_dynamic_is_causal(proc.theta, dist.theta_d);
    printf("Causality: %s (theta_p=%.0f, theta_d=%.0f)\n",
           causal ? "OK" : "NEEDS DELAY", proc.theta, dist.theta_d);
    if (!causal) {
        double extra = ff_dynamic_required_delay(proc.theta, dist.theta_d);
        printf("  Extra delay required: %.0fs\n", extra);
        T_lag += extra;
    }
    printf("\n");

    /* Configure FF controllers */
    feedforward_t ff_static_only, ff_dynamic;
    double Kff_static = -dist.Kd / proc.Kp;
    feedforward_configure_static(&ff_static_only, Kff_static, 50.0, 0.0, 100.0,
                                 ACTION_DIRECT, 0.1);
    feedforward_configure_dynamic(&ff_dynamic, Kff, T_lead, T_lag, 50.0, 0.0, 100.0,
                                  ACTION_DIRECT, 0.1);

    /* PI feedback tuning (IMC lambda = tau) */
    double lambda = proc.tau;
    double Kc = proc.tau / (fabs(proc.Kp) * (lambda + proc.theta));
    double Ti = proc.tau;
    printf("PI Tuning: Kc=%.4f, Ti=%.1fs\n\n", Kc, Ti);

    /* Simulation */
    double Ts = 0.5;
    double t_sim = 600.0;
    int n = (int)(t_sim / Ts) + 1;

    double y_none = 0.0, y_static = 0.0, y_dyn = 0.0;
    double e_int_none = 0.0, e_int_static = 0.0, e_int_dyn = 0.0;
    double ISE_none = 0.0, ISE_static = 0.0, ISE_dyn = 0.0;

    for (int k = 0; k < n; k++) {
        double t = k * Ts;

        /* Disturbance: step at t=50s */
        double d = (t >= 50.0) ? 2.0 : 0.0;

        /* -- No FF (feedback only) -- */
        double e_none = 0.0 - y_none;
        e_int_none += e_none * Ts;
        double u_none = Kc * (e_none + e_int_none / Ti);
        double dy_none = (proc.Kp * u_none + dist.Kd * d - y_none) / proc.tau;
        y_none += dy_none * Ts;
        ISE_none += e_none * e_none * Ts;

        /* -- Static FF only -- */
        double e_static = 0.0 - y_static;
        e_int_static += e_static * Ts;
        double u_fb_static = Kc * (e_static + e_int_static / Ti);
        double u_ff_static = ff_static_step(&ff_static_only, d);
        double u_total_static = u_fb_static + u_ff_static;
        double dy_static = (proc.Kp * u_total_static + dist.Kd * d - y_static) / proc.tau;
        y_static += dy_static * Ts;
        ISE_static += e_static * e_static * Ts;

        /* -- Dynamic FF -- */
        double e_dyn = 0.0 - y_dyn;
        e_int_dyn += e_dyn * Ts;
        double u_fb_dyn = Kc * (e_dyn + e_int_dyn / Ti);
        double u_ff_dyn = feedforward_step(&ff_dynamic, d);
        double u_total_dyn = u_fb_dyn + u_ff_dyn;
        double dy_dyn = (proc.Kp * u_total_dyn + dist.Kd * d - y_dyn) / proc.tau;
        y_dyn += dy_dyn * Ts;
        ISE_dyn += e_dyn * e_dyn * Ts;
    }

    printf("=== Results (ISE = Integral Squared Error) ===\n");
    printf("No FF:        ISE = %.4f  (baseline)\n", ISE_none);
    printf("Static FF:    ISE = %.4f  (%.1f%% reduction)\n",
           ISE_static, (1.0-ISE_static/ISE_none)*100.0);
    printf("Dynamic FF:   ISE = %.4f  (%.1f%% reduction)\n",
           ISE_dyn, (1.0-ISE_dyn/ISE_none)*100.0);

    printf("\n=== Key Takeaway ===\n");
    printf("Static FF compensates steady-state (%.1f%% reduction).\n",
           (1.0-ISE_static/ISE_none)*100.0);
    printf("Dynamic FF additionally compensates transients -> extra improvement.\n");

    printf("\nExample complete.\n");
    return 0;
}
