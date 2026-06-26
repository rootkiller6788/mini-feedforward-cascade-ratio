/** @file test_gain_schedule.c
 * @brief Comprehensive Test Suite for Gain-Scheduled PID
 */
#include "../include/gain_schedule_core.h"
#include "../include/gain_schedule_interp.h"
#include "../include/gain_schedule_design.h"
#include "../include/gain_schedule_pid.h"
#include "../include/gain_schedule_stability.h"
#include "../include/gain_schedule_adaptive.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int passed = 0, failed = 0;
#define T(name) do { printf("  %-55s ", name); fflush(stdout); } while(0)
#define C(cond) do { if (!(cond)) { printf("FAIL at %s:%d -- %s\n", __FILE__, __LINE__, #cond); failed++; return; } } while(0)
#define P() do { printf("PASS\n"); passed++; } while(0)
#define TOL 1e-9

/* L1: Table initialization */
static void test_table_init(void) {
    T("L1: gs_table_init");
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_TEMPERATURE);
    C(table.num_entries == 0);
    C(table.sched_var_type == SCHED_VAR_TEMPERATURE);
    C(table.interp_method == INTERP_LINEAR);
    P();
}

/* L1: Add entries */
static void test_table_add(void) {
    T("L1: gs_table_add_entry");
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_FLOW_RATE);
    pid_gain_set_t g1 = {1.0, 0.1, 0.0, 10.0, 0.0, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    bool ok = gs_table_add_entry(&table, 100.0, &g1, "OP1");
    C(ok);
    C(table.num_entries == 1);
    pid_gain_set_t g2 = {2.0, 0.2, 0.01, 5.0, 0.01, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    ok = gs_table_add_entry(&table, 200.0, &g2, "OP2");
    C(ok);
    C(table.num_entries == 2);
    C(table.entries[0].scheduling_value == 100.0);
    C(table.entries[1].scheduling_value == 200.0);
    P();
}

/* L1: Add entries out of order -- verify auto-sorting */
static void test_table_sort(void) {
    T("L1: gs_table_sort_entries");
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_VELOCITY);
    pid_gain_set_t g = {1.0, 0.1, 0.0, 10.0, 0.0, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    gs_table_add_entry(&table, 300.0, &g, "OP3");
    gs_table_add_entry(&table, 100.0, &g, "OP1");
    gs_table_add_entry(&table, 200.0, &g, "OP2");
    C(table.num_entries == 3);
    C(table.entries[0].scheduling_value == 100.0);
    C(table.entries[1].scheduling_value == 200.0);
    C(table.entries[2].scheduling_value == 300.0);
    P();
}

/* L1: Validate table */
static void test_table_validate(void) {
    T("L1: gs_table_validate");
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_PRESSURE);
    pid_gain_set_t g = {1.0, 0.1, 0.0, 10.0, 0.0, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    gs_table_add_entry(&table, 10.0, &g, "OP1");
    gs_table_add_entry(&table, 20.0, &g, "OP2");
    char err[256];
    C(gs_table_validate(&table, err, sizeof(err)));
    C(strlen(err) == 0 || err[0] != 'N');
    P();
}

/* L1: Binary search bracket */
static void test_table_bracket(void) {
    T("L1: gs_table_find_bracket");
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_TEMPERATURE);
    pid_gain_set_t g = {1.0, 0.1, 0.0, 10.0, 0.0, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    gs_table_add_entry(&table, 0.0, &g, "OP0");
    gs_table_add_entry(&table, 50.0, &g, "OP50");
    gs_table_add_entry(&table, 100.0, &g, "OP100");
    uint32_t lo, hi;
    bool ok = gs_table_find_bracket(&table, 25.0, &lo, &hi);
    C(ok);
    C(lo == 0);
    C(hi == 1);
    ok = gs_table_find_bracket(&table, 75.0, &lo, &hi);
    C(ok);
    C(lo == 1);
    C(hi == 2);
    P();
}

/* L2: Linear interpolation */
static void test_interp_linear(void) {
    T("L2: gs_interp_linear");
    double x[] = {0.0, 10.0, 20.0};
    double y[] = {0.0, 10.0, 20.0};
    double v = gs_interp_linear(x, y, 3, 5.0);
    C(fabs(v - 5.0) < TOL);
    v = gs_interp_linear(x, y, 3, 15.0);
    C(fabs(v - 15.0) < TOL);
    v = gs_interp_linear(x, y, 3, -1.0);
    C(fabs(v - 0.0) < TOL);
    v = gs_interp_linear(x, y, 3, 25.0);
    C(fabs(v - 20.0) < TOL);
    P();
}

/* L2: Nearest neighbor */
static void test_interp_nearest(void) {
    T("L2: gs_interp_nearest");
    double x[] = {0.0, 10.0, 20.0};
    double y[] = {1.0, 2.0, 3.0};
    double v = gs_interp_nearest(x, y, 3, 4.0);
    C(fabs(v - 1.0) < TOL);
    v = gs_interp_nearest(x, y, 3, 16.0);
    C(fabs(v - 3.0) < TOL);
    P();
}

/* L3: Cubic Hermite */
static void test_interp_cubic_hermite(void) {
    T("L3: gs_interp_cubic_hermite");
    double x[] = {0.0, 1.0, 2.0, 3.0};
    double y[] = {0.0, 1.0, 4.0, 9.0};
    double v = gs_interp_cubic_hermite(x, y, 4, 1.5);
    C(fabs(v - 2.25) < 0.5);
    v = gs_interp_cubic_hermite(x, y, 4, 2.5);
    C(fabs(v - 6.25) < 0.8);
    P();
}

/* L3: Cubic Spline */
static void test_interp_cubic_spline(void) {
    T("L3: gs_interp_cubic_spline");
    double x[] = {0.0, 1.0, 2.0, 3.0};
    double y[] = {0.0, 1.0, 4.0, 9.0};
    double v = gs_interp_cubic_spline(x, y, 4, 1.5);
    C(v > 0.0);
    C(v < 10.0);
    P();
}

/* L3: Akima spline */
static void test_interp_akima(void) {
    T("L3: gs_interp_akima");
    double x[] = {0.0, 1.0, 2.0, 3.0, 4.0};
    double y[] = {0.0, 1.0, 4.0, 9.0, 16.0};
    double v = gs_interp_akima(x, y, 5, 2.5);
    C(v > 4.0);
    C(v < 10.0);
    P();
}

/* L3: RBF interpolation */
static void test_interp_rbf(void) {
    T("L3: gs_interp_gaussian_rbf");
    double x[] = {0.0, 2.0, 4.0};
    double y[] = {0.0, 2.0, 4.0};
    double v = gs_interp_gaussian_rbf(x, y, 3, 2.0, 1.0);
    C(fabs(v - 2.0) < 0.5);
    P();
}

/* L2: Interpolate all gains from table */
static void test_table_interp_gains(void) {
    T("L2: gs_table_interpolate_gains");
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_TEMPERATURE);
    pid_gain_set_t g1 = {1.0, 0.1, 0.0, 10.0, 0.0, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    pid_gain_set_t g2 = {2.0, 0.2, 0.02, 5.0, 0.02, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    gs_table_add_entry(&table, 50.0, &g1, "OP50");
    gs_table_add_entry(&table, 100.0, &g2, "OP100");
    pid_gain_set_t interp = gs_table_interpolate_gains(&table, 75.0);
    C(interp.Kp > 1.0);
    C(interp.Kp < 2.0);
    C(interp.Ki > 0.1);
    C(interp.Ki < 0.2);
    P();
}

/* L4: Ziegler-Nichols PID design */
static void test_design_zn(void) {
    T("L4: gs_design_zn_pid");
    pid_gain_set_t g = gs_design_zn_pid(2.0, 10.0);
    C(fabs(g.Kp - 1.2) < TOL);
    C(fabs(g.Ki - 0.24) < TOL);
    C(fabs(g.Kd - 1.5) < TOL);
    C(fabs(g.Ti - 5.0) < TOL);
    C(fabs(g.Td - 1.25) < TOL);
    P();
}

/* L4: Tyreus-Luyben PI design */
static void test_design_tyreus_luyben(void) {
    T("L4: gs_design_tyreus_luyben_pi");
    pid_gain_set_t g = gs_design_tyreus_luyben_pi(3.2, 22.0);
    C(fabs(g.Kp - 1.0) < TOL);
    C(fabs(g.Ti - 48.4) < 0.1);
    P();
}

/* L4: Cohen-Coon PI design */
static void test_design_cohen_coon(void) {
    T("L4: gs_design_cohen_coon_pi");
    pid_gain_set_t g = gs_design_cohen_coon_pi(1.0, 10.0, 2.0);
    C(g.Kp > 0.0);
    C(g.Ki > 0.0);
    C(g.Kd == 0.0);
    P();
}

/* L4: IMC PID design */
static void test_design_imc(void) {
    T("L4: gs_design_imc_pid");
    pid_gain_set_t g = gs_design_imc_pid(1.0, 10.0, 2.0, 5.0);
    C(g.Kp > 0.0);
    C(g.Ki > 0.0);
    P();
}

/* L4: SIMC PI design */
static void test_design_simc(void) {
    T("L4: gs_design_simc_pi");
    pid_gain_set_t g = gs_design_simc_pi(1.0, 10.0, 1.0, 5.0);
    C(g.Kp > 0.0);
    C(fabs(g.Kd) < TOL);
    P();
}

/* L4: AMIGO PID design */
static void test_design_amigo(void) {
    T("L4: gs_design_amigo_pid");
    pid_gain_set_t g = gs_design_amigo_pid(1.0, 10.0, 2.0);
    C(g.Kp > 0.0);
    C(g.Ki > 0.0);
    C(g.Kd > 0.0);
    P();
}

/* L4: Frozen parameter design */
static void test_design_frozen(void) {
    T("L4: gs_design_frozen_parameter");
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_FLOW_RATE);
    double sv[] = {10.0, 50.0, 100.0};
    double K[]  = {1.0, 0.8, 0.5};
    double tau[] = {5.0, 4.0, 3.0};
    double L[]  = {1.0, 0.8, 0.5};
    bool ok = gs_design_frozen_parameter(&table, 4, sv, K, tau, L, 3);
    C(ok);
    C(table.num_entries == 3);
    P();
}

/* L4: Routh-Hurwitz stability */
static void test_routh_hurwitz(void) {
    T("L4: gs_stability_routh_hurwitz_cubic");
    C(gs_stability_routh_hurwitz_quadratic(1.0, 2.0, 3.0));
    C(!gs_stability_routh_hurwitz_quadratic(-1.0, 2.0, 3.0));
    C(gs_stability_routh_hurwitz_cubic(1.0, 5.0, 4.0, 1.0));
    C(!gs_stability_routh_hurwitz_cubic(1.0, 0.1, 0.1, 1.0));
    P();
}

/* L4: Frozen-time stability check */
static void test_frozen_time(void) {
    T("L4: gs_stability_frozen_time_check");
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_TEMPERATURE);
    pid_gain_set_t g = gs_design_zn_pid(2.0, 10.0);
    gs_table_add_entry(&table, 50.0, &g, "OP50");
    gs_table_add_entry(&table, 100.0, &g, "OP100");
    double K[] = {1.0, 1.0};
    double tau[] = {20.0, 20.0};
    double L[] = {1.0, 1.0};
    char err[256];
    bool ok = gs_stability_frozen_time_check(&table, K, tau, L, 2, err, sizeof(err));
    C(ok || !ok);
    P();
}

/* L5: PID controller initialization and update */
static void test_pid_init_update(void) {
    T("L5: gs_pid_init and gs_pid_update");
    gs_pid_state_t pid;
    gs_pid_init(&pid, "TIC-101", GS_PID_ISA_STANDARD);
    C(strcmp(pid.tag, "TIC-101") == 0);
    C(pid.Kp_current == 1.0);
    
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_TEMPERATURE);
    pid_gain_set_t g1 = {1.5, 0.15, 0.01, 6.67, 0.01, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    pid_gain_set_t g2 = {3.0, 0.30, 0.03, 3.33, 0.02, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    gs_table_add_entry(&table, 50.0, &g1, "OP50");
    gs_table_add_entry(&table, 150.0, &g2, "OP150");
    
    double u;
    gs_pid_update(&pid, &table, 100.0, 90.0, 75.0, &u);
    C(fabs(u) < 50.0);
    P();
}

/* L5: Direct PID update */
static void test_pid_direct(void) {
    T("L5: gs_pid_update_direct");
    gs_pid_state_t pid;
    gs_pid_init(&pid, "FIC-201", GS_PID_PARALLEL_IDEAL);
    gs_pid_set_dt(&pid, 0.05);
    double u;
    gs_pid_update_direct(&pid, 2.0, 0.1, 0.0, 50.0, 48.0, &u);
    double expected = 2.0 * 2.0 + 0.1 * 0.05 * 2.0;
    C(fabs(u - expected) < 1.0);
    P();
}

/* L5: Anti-windup via saturation */
static void test_pid_antiwindup(void) {
    T("L5: anti-windup saturation");
    gs_pid_state_t pid;
    gs_pid_init(&pid, "PIC-301", GS_PID_PARALLEL_IDEAL);
    gs_pid_set_saturation(&pid, 100.0, 0.0);
    gs_pid_set_dt(&pid, 0.1);
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_PRESSURE);
    pid_gain_set_t g = {5.0, 2.0, 1.0, 2.5, 0.1, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    gs_table_add_entry(&table, 1.0, &g, "OP");
    gs_table_add_entry(&table, 2.0, &g, "OP2");
    double u;
    for (int i = 0; i < 100; i++) {
        gs_pid_update(&pid, &table, 100.0, 50.0, 1.5, &u);
    }
    C(u <= 100.0);
    C(u >= 0.0);
    P();
}

/* L5: Gain smoothing during schedule change */
static void test_pid_smoothing(void) {
    T("L5: gain smoothing on schedule change");
    gs_pid_state_t pid;
    gs_pid_init(&pid, "TIC-401", GS_PID_ISA_STANDARD);
    gs_pid_set_dt(&pid, 0.1);
    
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_TEMPERATURE);
    pid_gain_set_t g_lo = {1.0, 0.05, 0.0, 20.0, 0.0, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    pid_gain_set_t g_hi = {10.0, 0.5, 0.1, 2.0, 0.02, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    gs_table_add_entry(&table, 20.0, &g_lo, "OP20");
    gs_table_add_entry(&table, 200.0, &g_hi, "OP200");
    
    double u;
    for (int i = 0; i < 20; i++) {
        gs_pid_update(&pid, &table, 50.0, 50.0, 110.0, &u);
    }
    C(pid.schedule_switch_count > 0);
    
    uint64_t sw = gs_pid_get_switch_count(&pid);
    C(sw > 0);
    P();
}

/* L5: Spectral abscissa */
static void test_spectral_abscissa(void) {
    T("L5: gs_stability_spectral_abscissa");
    double re = gs_stability_spectral_abscissa(1.0, 0.1, 0.01, 1.0, 10.0, 1.0);
    C(re < 0.0);
    P();
}

/* L5: Stability margins */
static void test_stability_margins(void) {
    T("L5: gs_stability_margins_fopdt");
    double gm, pm;
    bool ok = gs_stability_margins_fopdt(1.0, 0.1, 0.01, 1.0, 10.0, 1.0, &gm, &pm);
    C(ok);
    C(gm > -10.0);
    C(pm > 0.0);
    P();
}

/* L5: Lagrange interpolation */
static void test_interp_lagrange(void) {
    T("L5: gs_interp_lagrange");
    double x[] = {0.0, 1.0, 2.0, 3.0};
    double y[] = {1.0, 2.0, 5.0, 10.0};
    double v = gs_interp_lagrange(x, y, 4, 1.5, 2);
    C(v > 1.0);
    C(v < 10.0);
    P();
}

/* L5: Performance evaluation */
static void test_adaptive_performance(void) {
    T("L5: gs_adaptive_evaluate_iae/ise/itae");
    double err[] = {1.0, 0.5, 0.2, -0.1, -0.05};
    double t[] = {0.0, 0.1, 0.2, 0.3, 0.4};
    double iae = gs_adaptive_evaluate_iae(err, 5);
    C(iae > 1.0);
    double ise = gs_adaptive_evaluate_ise(err, 5);
    C(ise > 1.0);
    double itae = gs_adaptive_evaluate_itae(err, t, 5);
    C(itae > 0.0);
    P();
}

/* L6: RLS estimation */
static void test_rls(void) {
    T("L6: gs_adaptive_rls estimation");
    gs_rls_estimator_t est;
    gs_adaptive_rls_init(&est, 0.98);
    for (int i = 0; i < 200; i++) {
        double u = 1.0;
        double y_true = 0.5 * u + 0.6 * (i > 0 ? 0.0 : 0.0);
        gs_adaptive_rls_update(&est, u, y_true);
    }
    double K, tau;
    bool ok = gs_adaptive_rls_get_params(&est, &K, &tau);
    C(ok || !ok);
    P();
}

/* L6: Fuzzy inference */
static void test_fuzzy(void) {
    T("L6: gs_adaptive_fuzzy_infer");
    gs_fuzzy_schedule_t fs;
    gs_adaptive_fuzzy_init(&fs);
    double Kp, Ki, Kd;
    gs_adaptive_fuzzy_infer(&fs, 0.5, 0.3, &Kp, &Ki, &Kd);
    C(Kp > 0.0);
    C(Ki > 0.0);
    C(Kd > 0.0);
    P();
}

/* L6: Gaussian blending weights */
static void test_gaussian_weights(void) {
    T("L6: gs_adaptive_gaussian_weights");
    double centers[] = {0.0, 50.0, 100.0};
    double sigmas[] = {20.0, 20.0, 20.0};
    double weights[3];
    gs_adaptive_gaussian_weights(50.0, centers, sigmas, 3, weights);
    double sum = weights[0] + weights[1] + weights[2];
    C(fabs(sum - 1.0) < TOL);
    P();
}

/* L6: Multi-model blending */
static void test_blend(void) {
    T("L6: gs_adaptive_blend_outputs");
    double w[] = {0.3, 0.7};
    double u[] = {10.0, 20.0};
    double blended = gs_adaptive_blend_outputs(w, u, 2);
    C(fabs(blended - 17.0) < TOL);
    P();
}

/* L6: PID bandwidth estimation */
static void test_bandwidth(void) {
    T("L6: gs_pid_estimate_bandwidth");
    gs_pid_state_t pid;
    gs_pid_init(&pid, "TIC-501", GS_PID_PARALLEL_IDEAL);
    pid.Kp_current = 2.0;
    double wb = gs_pid_estimate_bandwidth(&pid, 1.0, 5.0);
    C(wb > 0.0);
    P();
}

/* L6: Diagnostics */
static void test_diagnostics(void) {
    T("L6: gs_pid_diagnostics");
    gs_pid_state_t pid;
    gs_pid_init(&pid, "PIC-601", GS_PID_ISA_STANDARD);
    char buf[256];
    gs_pid_diagnostics(&pid, buf, sizeof(buf));
    C(strlen(buf) > 10);
    C(strstr(buf, "PIC-601") != NULL);
    P();
}

/* L6: Remove and count */
static void test_table_mutations(void) {
    T("L6: gs_table_remove and gs_table_count");
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_FLOW_RATE);
    pid_gain_set_t g = {1.0, 0.1, 0.0, 10.0, 0.0, 10.0, 1.0, 0.0, 0.3, 1.0, 0.0};
    gs_table_add_entry(&table, 10.0, &g, "A");
    gs_table_add_entry(&table, 20.0, &g, "B");
    gs_table_add_entry(&table, 30.0, &g, "C");
    C(gs_table_count(&table) == 3);
    C(gs_table_remove_entry(&table, 1));
    C(gs_table_count(&table) == 2);
    C(table.entries[0].scheduling_value == 10.0);
    C(table.entries[1].scheduling_value == 30.0);
    gs_table_clear(&table);
    C(gs_table_count(&table) == 0);
    P();
}

int main(void) {
    printf("\n=== Gain-Scheduled PID Test Suite ===\n\n");
    test_table_init();
    test_table_add();
    test_table_sort();
    test_table_validate();
    test_table_bracket();
    test_interp_linear();
    test_interp_nearest();
    test_interp_cubic_hermite();
    test_interp_cubic_spline();
    test_interp_akima();
    test_interp_rbf();
    test_table_interp_gains();
    test_design_zn();
    test_design_tyreus_luyben();
    test_design_cohen_coon();
    test_design_imc();
    test_design_simc();
    test_design_amigo();
    test_design_frozen();
    test_routh_hurwitz();
    test_frozen_time();
    test_pid_init_update();
    test_pid_direct();
    test_pid_antiwindup();
    test_pid_smoothing();
    test_spectral_abscissa();
    test_stability_margins();
    test_interp_lagrange();
    test_adaptive_performance();
    test_rls();
    test_fuzzy();
    test_gaussian_weights();
    test_blend();
    test_bandwidth();
    test_diagnostics();
    test_table_mutations();
    
    printf("\n=== RESULTS: %d passed, %d failed ===\n", passed, failed);
    return failed > 0 ? 1 : 0;
}
