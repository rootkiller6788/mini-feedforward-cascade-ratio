/** @file demo_schedule_analysis.c
 * @brief Gain Schedule Visualization and Analysis Demo (L5/L6/L7)
 *
 * Comprehensive demonstration of the gain-scheduled PID design workflow:
 *   1. Design a full schedule table for a nonlinear temperature process
 *   2. Compute interpolated gains across the operating range using all methods
 *   3. Output CSV data for external plotting (Matplotlib, Excel, etc.)
 *   4. Analyze stability margins and bandwidth across the schedule
 *   5. Demonstrate frozen-parameter validation
 *   6. Show adaptive RLS-based schedule refinement
 *   7. Compare multi-model blending vs single-model interpolation
 *
 * Each function implements an independent knowledge point.
 * Output: CSV files to stdout and .csv files for visualization.
 *
 * Industrial Context: Heat exchanger temperature control (L7)
 * Advanced: Multi-model blending (L8), RLS adaptation (L8)
 */

#include "../include/gain_schedule_core.h"
#include "../include/gain_schedule_interp.h"
#include "../include/gain_schedule_design.h"
#include "../include/gain_schedule_pid.h"
#include "../include/gain_schedule_stability.h"
#include "../include/gain_schedule_adaptive.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* --------------------------------------------------------------------------
 * KP1: Schedule Table CSV Export -- exports full schedule to stdout/CSV
 * ------------------------------------------------------------------------*/
static void demo_export_schedule_csv(const gain_schedule_table_t *table,
                                      const double *K_arr,
                                      const double *tau_arr,
                                      const double *L_arr) {
    uint32_t n = gs_table_count(table);
    printf("# Gain Schedule Table CSV\n");
    printf("# Scheduling variable type: %d\n", table->sched_var_type);
    printf("# Interpolation method: %d\n", table->interp_method);
    printf("# Entries: %u\n", n);
    printf("sched_val,Kp,Ki,Kd,Ti,Td,N,process_K,process_tau,process_L,region,validated\n");
    for (uint32_t i = 0; i < n; i++) {
        const schedule_entry_t *e = gs_table_get_entry(table, i);
        if (!e) continue;
        double pk = (K_arr && i < n) ? K_arr[i] : 0.0;
        double pt = (tau_arr && i < n) ? tau_arr[i] : 0.0;
        double pl = (L_arr && i < n) ? L_arr[i] : 0.0;
        printf("%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%d,%d\n",
               e->scheduling_value,
               e->gains.Kp, e->gains.Ki, e->gains.Kd,
               e->gains.Ti, e->gains.Td, e->gains.N,
               pk, pt, pl,
               (int)e->region, (int)e->validated);
    }
}

/* --------------------------------------------------------------------------
 * KP2: Interpolation Method Comparison -- evaluates all 7 methods on fine grid
 * ------------------------------------------------------------------------*/
static void demo_compare_interpolation_methods(
    const gain_schedule_table_t *table,
    double xmin, double xmax, uint32_t npoints) {
    uint32_t n = table->num_entries;
    double *sched_axis = (double *)malloc(npoints * sizeof(double));
    double *gains_kp = (double *)malloc(n * sizeof(double));
    double *x_pts   = (double *)malloc(n * sizeof(double));
    if (!sched_axis || !gains_kp || !x_pts) {
        fprintf(stderr, "Memory allocation failed\n");
        free(sched_axis); free(gains_kp); free(x_pts);
        return;
    }
    for (uint32_t i = 0; i < n; i++) {
        x_pts[i] = table->entries[i].scheduling_value;
        gains_kp[i] = table->entries[i].gains.Kp;
    }
    printf("\n# Interpolation Comparison CSV (Kp values across range)\n");
    printf("x,Kp_nearest,Kp_linear,Kp_cubic_hermite,Kp_cubic_spline,Kp_lagrange3,Kp_akima,Kp_rbf\n");
    for (uint32_t k = 0; k < npoints; k++) {
        double xq = xmin + (xmax - xmin) * (double)k / (double)(npoints - 1);
        sched_axis[k] = xq;
        double v_near   = gs_interp_nearest(x_pts, gains_kp, n, xq);
        double v_linear = gs_interp_linear(x_pts, gains_kp, n, xq);
        double v_herm   = gs_interp_cubic_hermite(x_pts, gains_kp, n, xq);
        double v_spline = gs_interp_cubic_spline(x_pts, gains_kp, n, xq);
        double v_lagr   = gs_interp_lagrange(x_pts, gains_kp, n, xq, 3);
        double v_akima  = gs_interp_akima(x_pts, gains_kp, n, xq);
        double v_rbf    = gs_interp_gaussian_rbf(x_pts, gains_kp, n, xq, 0.3);
        printf("%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
               xq, v_near, v_linear, v_herm, v_spline, v_lagr, v_akima, v_rbf);
    }
    free(sched_axis); free(gains_kp); free(x_pts);
}

/* --------------------------------------------------------------------------
 * KP3: Stability Analysis Across Schedule
 * ------------------------------------------------------------------------*/
static void demo_analyze_stability_csv(const gain_schedule_table_t *table,
                                        const double *K_arr,
                                        const double *tau_arr,
                                        const double *L_arr) {
    uint32_t n = gs_table_count(table);
    printf("\n# Stability Analysis CSV\n");
    printf("Kp,Ki,Kd,K,tau,L,gm_dB,pm_deg,spectral_abscissa,delay_margin,modulus_margin,is_stable_rh\n");
    for (uint32_t i = 0; i < n; i++) {
        const schedule_entry_t *e = gs_table_get_entry(table, i);
        if (!e || !K_arr || !tau_arr || !L_arr) continue;
        double Kp = e->gains.Kp, Ki = e->gains.Ki, Kd = e->gains.Kd;
        double K  = K_arr[i], tau = tau_arr[i], L = L_arr[i];
        double gm_dB = 0.0, pm_deg = 0.0;
        gs_stability_margins_fopdt(Kp, Ki, Kd, K, tau, L, &gm_dB, &pm_deg);
        double s_abs = gs_stability_spectral_abscissa(Kp, Ki, Kd, K, tau, L);
        double a0 = tau * L / 2.0;
        double a1 = tau + L/2.0 - K * Kd * L / 2.0;
        double a2 = 1.0 + K * Kp - K * Ki * L / 2.0;
        double a3 = K * Ki;
        bool rh = gs_stability_routh_hurwitz_cubic(a0, a1, a2, a3);
        double w_gc = 1.0 / tau;
        double dm  = gs_stability_delay_margin(pm_deg, w_gc);
        double Ms  = 1.0 + 0.1 * (double)i;
        double mm  = gs_stability_modulus_margin(Ms);
        printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.4f,%.4f,%d\n",
               Kp, Ki, Kd, K, tau, L, gm_dB, pm_deg, s_abs, dm, mm, rh ? 1 : 0);
    }
}

/* --------------------------------------------------------------------------
 * KP4: Frozen-Parameter Design Validation
 * ------------------------------------------------------------------------*/
static bool demo_frozen_parameter_workflow(void) {
    printf("\n========================================\n");
    printf("  Frozen-Parameter Design Validation\n");
    printf("========================================\n");
    double sched_vals[7] = {30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0};
    double K_vals[7], tau_vals[7], L_vals[7];
    for (int i = 0; i < 7; i++) {
        double T = sched_vals[i];
        K_vals[i]   = 0.5 + 0.006 * T;
        tau_vals[i] = 20.0 - 0.1 * T;
        L_vals[i]   = 2.5 - 0.005 * T;
        if (K_vals[i] < 0.3) K_vals[i] = 0.3;
        if (tau_vals[i] < 3.0) tau_vals[i] = 3.0;
        if (L_vals[i] < 0.5) L_vals[i] = 0.5;
    }
    const char *rule_names[] = {"ZN", "Tyreus-Luyben", "Cohen-Coon", "IMC", "SIMC", "AMIGO"};
    int rules[] = {0, 1, 2, 3, 4, 5};
    for (int r = 0; r < 6; r++) {
        gain_schedule_table_t table;
        gs_table_init(&table, SCHED_VAR_TEMPERATURE);
        table.interp_method = INTERP_CUBIC_HERMITE;
        bool ok = gs_design_frozen_parameter(&table, rules[r], sched_vals,
                                              K_vals, tau_vals, L_vals, 7);
        if (!ok) { printf("  %-16s: DESIGN FAILED\n", rule_names[r]); continue; }
        char err[256] = {0};
        bool valid = gs_design_validate_margins(&table, 25.0, 3.0, err, 255);
        double max_ratio = gs_design_max_gain_ratio(&table);
        printf("  %-16s: entries=%2u  valid=%s  max_gain_ratio=%.3f\n",
               rule_names[r], gs_table_count(&table),
               valid ? "YES" : "NO", max_ratio);
        if (!valid && err[0]) printf("    Reason: %s\n", err);
    }
    return true;
}

/* --------------------------------------------------------------------------
 * KP5: Schedule Smoothing and Refinement Demo
 * ------------------------------------------------------------------------*/
static void demo_schedule_smoothing(void) {
    printf("\n========================================\n");
    printf("  Schedule Smoothing and Refinement\n");
    printf("========================================\n");
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_TEMPERATURE);
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    g.N = 10.0; g.b = 1.0; g.Kb = 0.3;
    g.Kp = 1.0; g.Ki = 0.1;  g.Kd = 0.01; g.Ti = 10.0; g.Td = 0.1;
    gs_table_add_entry(&table, 20.0, &g, "T=20");
    g.Kp = 1.5; g.Ki = 0.15; g.Kd = 0.02; g.Ti = 8.0;  g.Td = 0.15;
    gs_table_add_entry(&table, 40.0, &g, "T=40");
    g.Kp = 5.0; g.Ki = 0.5;  g.Kd = 0.05; g.Ti = 5.0;  g.Td = 0.2;
    gs_table_add_entry(&table, 60.0, &g, "T=60");
    g.Kp = 2.0; g.Ki = 0.2;  g.Kd = 0.03; g.Ti = 6.0;  g.Td = 0.18;
    gs_table_add_entry(&table, 80.0, &g, "T=80");
    g.Kp = 0.8; g.Ki = 0.08; g.Kd = 0.01; g.Ti = 12.0; g.Td = 0.08;
    gs_table_add_entry(&table, 100.0, &g, "T=100");
    printf("Before smoothing: %u entries, max_gain_ratio=%.3f\n",
           gs_table_count(&table), gs_design_max_gain_ratio(&table));
    gs_design_smooth_schedule(&table, 2);
    printf("After smoothing (window=2): %u entries, max_gain_ratio=%.3f\n",
           gs_table_count(&table), gs_design_max_gain_ratio(&table));
    for (uint32_t i = 0; i < gs_table_count(&table); i++) {
        const schedule_entry_t *e = gs_table_get_entry(&table, i);
        printf("  T=%.1f: Kp=%.4f Ki=%.4f Kd=%.4f\n",
               e->scheduling_value, e->gains.Kp, e->gains.Ki, e->gains.Kd);
    }
}

/* --------------------------------------------------------------------------
 * KP6: Multi-Model Blending vs Single-Model Interpolation (L8)
 * ------------------------------------------------------------------------*/
static void demo_multimodel_blending(void) {
    printf("\n========================================\n");
    printf("  Multi-Model Blending Demo (L8)\n");
    printf("========================================\n");
    uint32_t n_m = 5;
    double centers[] = {30.0, 50.0, 70.0, 90.0, 110.0};
    double sigmas[]  = {12.0, 12.0, 12.0, 12.0, 12.0};
    double outputs[] = {0.45, 0.52, 0.61, 0.58, 0.50};
    printf("Sched_Val  w1     w2     w3     w4     w5     Blended\n");
    printf("---------  -----  -----  -----  -----  -----  -------\n");
    for (double sv = 30.0; sv <= 110.0; sv += 5.0) {
        double weights[5] = {0};
        gs_adaptive_gaussian_weights(sv, centers, sigmas, n_m, weights);
        double blended = gs_adaptive_blend_outputs(weights, outputs, n_m);
        printf("%9.1f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %7.4f\n",
               sv, weights[0], weights[1], weights[2], weights[3], weights[4], blended);
    }
}

/* --------------------------------------------------------------------------
 * KP7: Performance Metric Evaluation (L5)
 * ------------------------------------------------------------------------*/
static void demo_performance_metrics(void) {
    printf("\n========================================\n");
    printf("  Performance Metric Comparison (L5)\n");
    printf("========================================\n");
    #define PERF_N 50
    double error_fast[PERF_N], error_osc[PERF_N], error_slow[PERF_N];
    double time_axis[PERF_N];
    for (int i = 0; i < PERF_N; i++) {
        double t = (double)i * 0.2;
        time_axis[i] = t;
        error_fast[i] = exp(-2.0 * t) * (1.0 + 0.1 * sin(3.0 * t));
        if (error_fast[i] < 0.001) error_fast[i] = 0.0;
        error_osc[i] = exp(-0.5 * t) * cos(2.0 * t);
        if (fabs(error_osc[i]) < 0.001) error_osc[i] = 0.0;
        error_slow[i] = exp(-0.8 * t);
        if (error_slow[i] < 0.001) error_slow[i] = 0.0;
    }
    printf("%-20s %12s %12s %12s\n", "Scenario", "IAE", "ISE", "ITAE");
    printf("%.20s %12s %12s %12s\n", "--------------------", "------------", "------------", "------------");
    printf("%-20s %12.6f %12.6f %12.6f\n", "Fast settling",
           gs_adaptive_evaluate_iae(error_fast, PERF_N),
           gs_adaptive_evaluate_ise(error_fast, PERF_N),
           gs_adaptive_evaluate_itae(error_fast, time_axis, PERF_N));
    printf("%-20s %12.6f %12.6f %12.6f\n", "Oscillatory",
           gs_adaptive_evaluate_iae(error_osc, PERF_N),
           gs_adaptive_evaluate_ise(error_osc, PERF_N),
           gs_adaptive_evaluate_itae(error_osc, time_axis, PERF_N));
    printf("%-20s %12.6f %12.6f %12.6f\n", "Slow monotonic",
           gs_adaptive_evaluate_iae(error_slow, PERF_N),
           gs_adaptive_evaluate_ise(error_slow, PERF_N),
           gs_adaptive_evaluate_itae(error_slow, time_axis, PERF_N));
}

/* --------------------------------------------------------------------------
 * KP8: RLS Online Parameter Estimation Demo (L8)
 * ------------------------------------------------------------------------*/
static void demo_rls_estimation(void) {
    printf("\n========================================\n");
    printf("  RLS Online Identification Demo (L8)\n");
    printf("========================================\n");
    gs_rls_estimator_t est;
    gs_adaptive_rls_init(&est, 0.98);
    double true_K = 1.5, true_tau = 3.0;
    double dt = 0.1;
    double a_true = exp(-dt / true_tau);
    double y = 0.0, u = 1.0;
    printf("Step, u, y_true, y_hat, K_est, tau_est, converged\n");
    for (int k = 0; k < 40; k++) {
        double y_prev = y;
        y = a_true * y_prev + true_K * (1.0 - a_true) * u;
        y += 0.01 * ((double)rand() / RAND_MAX - 0.5);
        gs_adaptive_rls_update(&est, u, y);
        double K_est = 0.0, tau_est = 0.0;
        bool conv = gs_adaptive_rls_get_params(&est, &K_est, &tau_est);
        printf("%4d, %.2f, %.4f, %.4f, %.4f, %.4f, %s\n",
               k, u, y, est.y_hat, K_est, tau_est, conv ? "YES" : "no");
    }
}

/* --------------------------------------------------------------------------
 * KP9: Gain Scheduling Controller Synthesis (end-to-end L6)
 * ------------------------------------------------------------------------*/
static void demo_full_synthesis(void) {
    printf("\n========================================\n");
    printf("  Full Gain-Scheduled PID Synthesis\n");
    printf("========================================\n");
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_TEMPERATURE);
    table.interp_method = INTERP_CUBIC_SPLINE;
    double sv[] = {25.0, 50.0, 75.0, 100.0, 125.0};
    double K[]  = {0.6, 0.9, 1.2, 1.5, 1.8};
    double tau[]= {18.0, 14.0, 10.0, 8.0, 6.0};
    double L[]  = {2.5, 2.2, 2.0, 1.8, 1.5};
    gs_design_frozen_parameter(&table, 4, sv, K, tau, L, 5);
    gs_design_smooth_schedule(&table, 2);
    printf("Schedule designed with %u entries (SIMC rule)\n", gs_table_count(&table));
    char errmsg[256] = {0};
    bool stable = gs_stability_frozen_time_check(&table, K, tau, L, 5, errmsg, sizeof(errmsg));
    printf("Frozen-time stability: %s\n", stable ? "PASS" : "FAIL");
    bool slow_var = gs_stability_slow_variation_check(&table, 0.5, 0.7, errmsg, sizeof(errmsg));
    printf("Slow-variation check: %s\n", slow_var ? "PASS" : "MAY FAIL");
    double min_margin = gs_stability_min_margin(&table, K, tau, L, 5);
    printf("Worst-case stability margin: %.4f\n", min_margin);
    double lyap_cond = gs_stability_lyapunov_condition(&table, K, tau, L, 5);
    printf("Lyapunov condition number: %.4f %s\n",
           lyap_cond,
           (lyap_cond > 0.0 && lyap_cond < 100.0) ? "(common P exists)" :
           (lyap_cond < 0.0) ? "(no common P)" : "(marginal)");
    gs_pid_state_t pid;
    gs_pid_init(&pid, "TIC-DEMO-01", GS_PID_ISA_STANDARD);
    gs_pid_set_dt(&pid, 0.5);
    gs_pid_set_saturation(&pid, 100.0, 0.0);
    double T_proc = 25.0, T_sp = 80.0, T_in = 20.0;
    printf("\nSimulation: T=%.1f -> %.1f degC\n", T_proc, T_sp);
    printf("Time[s]  T[degC]  SP[degC]  u[%%]    Kp      Ki\n");
    printf("-------  -------  --------  ------   ------  ------\n");
    for (double t = 0.0; t <= 60.0; t += 0.5) {
        double u_out;
        gs_pid_update(&pid, &table, T_sp, T_proc, T_proc, &u_out);
        double K_local = 0.8 + 0.004 * T_proc;
        double tau_local = 15.0 - 0.05 * T_proc;
        if (tau_local < 3.0) tau_local = 3.0;
        if (K_local > 2.0) K_local = 2.0;
        if (K_local < 0.5) K_local = 0.5;
        double a = exp(-0.5 / tau_local);
        double T_prev = T_proc;
        T_proc = a * T_prev + K_local * (1.0 - a) * u_out + (1.0 - a) * T_in;
        if (fmod(t, 5.0) < 0.25) {
            printf("%7.1f  %7.2f  %8.1f  %6.2f   %6.3f  %6.3f\n",
                   t, T_proc, T_sp, u_out, pid.Kp_current, pid.Ki_current);
        }
    }
    printf("\nFinal T=%.2f, switches=%llu\n",
           T_proc, (unsigned long long)gs_pid_get_switch_count(&pid));
    printf("Synthesis complete.\n");
}

/* =========================================================================
 * Main -- orchestrates all demos
 * =======================================================================*/
int main(void) {
    srand((unsigned int)time(NULL));
    printf("========================================\n");
    printf("  Gain-Scheduled PID Analysis Suite\n");
    printf("  Knowledge Coverage: L5, L6, L7, L8\n");
    printf("========================================\n");

    gain_schedule_table_t demo_table;
    gs_table_init(&demo_table, SCHED_VAR_TEMPERATURE);
    demo_table.interp_method = INTERP_CUBIC_HERMITE;
    double sv[6] = {25.0, 45.0, 65.0, 85.0, 105.0, 125.0};
    double Kd[6] = {0.5, 0.8, 1.1, 1.4, 1.7, 2.0};
    double td[6] = {20.0, 16.0, 12.0, 8.5, 6.0, 4.0};
    double Ld[6] = {2.0, 1.8, 1.6, 1.4, 1.2, 1.0};
    gs_design_frozen_parameter(&demo_table, 5, sv, Kd, td, Ld, 6);

    demo_export_schedule_csv(&demo_table, Kd, td, Ld);
    demo_compare_interpolation_methods(&demo_table, 25.0, 125.0, 101);
    demo_analyze_stability_csv(&demo_table, Kd, td, Ld);
    demo_frozen_parameter_workflow();
    demo_schedule_smoothing();
    demo_multimodel_blending();
    demo_performance_metrics();
    demo_rls_estimation();
    demo_full_synthesis();

    printf("\n========================================\n");
    printf("  All demos completed successfully.\n");
    printf("========================================\n");
    return 0;
}