/** @file bench_gain_schedule.c
 * @brief Performance Benchmarks for Gain-Scheduled PID (L5)
 *
 * Measures throughput of core operations:
 *   1. Interpolation method throughput (million interpolations/sec)
 *   2. PID update throughput (thousand controller updates/sec)
 *   3. Schedule table operation throughput
 *   4. Stability analysis throughput
 *   5. RLS estimation throughput
 *   6. Fuzzy inference throughput
 *   7. Memory footprint of data structures
 *
 * Each benchmark implements an independent knowledge point.
 * Uses clock() for portable timing with ms resolution.
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

/* Number of iterations for throughput benchmarks */
#define BENCH_LOOPS_INTERP  1000000
#define BENCH_LOOPS_PID      500000
#define BENCH_LOOPS_TABLE    100000
#define BENCH_LOOPS_STABLE   100000
#define BENCH_LOOPS_RLS      100000
#define BENCH_LOOPS_FUZZY    200000

static double elapsed_ms(clock_t start, clock_t end) {
    return 1000.0 * (double)(end - start) / (double)CLOCKS_PER_SEC;
}

/* --------------------------------------------------------------------------
 * KB1: Interpolation Throughput -- measure Minterp/sec for each method
 * ------------------------------------------------------------------------*/
static void bench_one_interp(const char *name, uint32_t n_pts,
                             const double *x_pts, const double *y_pts,
                             int method_tag) {
    double xq = 50.0;
    double sum = 0.0;
    clock_t t0 = clock();
    for (int i = 0; i < BENCH_LOOPS_INTERP; i++) {
        xq += 0.0001;
        if (xq > 110.0) xq = 0.0;
        double v = 0.0;
        switch (method_tag) {
            case 0: v = gs_interp_nearest(x_pts, y_pts, n_pts, xq); break;
            case 1: v = gs_interp_linear(x_pts, y_pts, n_pts, xq); break;
            case 2: v = gs_interp_cubic_hermite(x_pts, y_pts, n_pts, xq); break;
            case 3: v = gs_interp_cubic_spline(x_pts, y_pts, n_pts, xq); break;
            case 4: v = gs_interp_lagrange(x_pts, y_pts, n_pts, xq, 3); break;
            case 5: v = gs_interp_akima(x_pts, y_pts, n_pts, xq); break;
            case 6: v = gs_interp_gaussian_rbf(x_pts, y_pts, n_pts, xq, 0.3); break;
            case 7: v = gs_interp_dispatch(x_pts, y_pts, n_pts, xq,
                                           INTERP_LINEAR, 0.3); break;
            default: break;
        }
        sum += v;
    }
    clock_t t1 = clock();
    double ms = elapsed_ms(t0, t1);
    double mips = (double)BENCH_LOOPS_INTERP / (ms * 1000.0);
    printf("%-16s %10.2f %15d %15.2f\n", name, ms,
           BENCH_LOOPS_INTERP, mips);
    (void)sum;
}

static void bench_interpolation_throughput(void) {
    printf("\n========================================\n");
    printf("  Interpolation Throughput Benchmarks\n");
    printf("========================================\n");

    uint32_t n_pts = 12;
    double x_pts[12] = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0,
                         60.0, 70.0, 80.0, 90.0, 100.0, 110.0};
    double y_pts[12] = {1.0, 1.1, 1.3, 1.6, 2.0, 2.5,
                         3.1, 3.8, 4.6, 5.0, 5.2, 5.0};

    /* Precompute spline moments once */
    double M[12] = {0};
    gs_spline_compute_moments(x_pts, y_pts, n_pts, M);

    printf("%-16s %10s %15s %15s\n", "Method", "Time(ms)", "Interps", "Minterps/sec");
    printf("%-16s %10s %15s %15s\n", "----------------", "----------", "---------------", "---------------");

    bench_one_interp("Nearest",     n_pts, x_pts, y_pts, 0);
    bench_one_interp("Linear",      n_pts, x_pts, y_pts, 1);
    bench_one_interp("CubicHermite",n_pts, x_pts, y_pts, 2);
    bench_one_interp("CubicSpline", n_pts, x_pts, y_pts, 3);
    bench_one_interp("Lagrange(3)", n_pts, x_pts, y_pts, 4);
    bench_one_interp("Akima",       n_pts, x_pts, y_pts, 5);
    bench_one_interp("RBF(s=0.3)",  n_pts, x_pts, y_pts, 6);
    bench_one_interp("Dispatch",    n_pts, x_pts, y_pts, 7);
}

/* --------------------------------------------------------------------------
 * KB2: PID Update Throughput -- measure controller updates/sec
 * ------------------------------------------------------------------------*/
static void bench_pid_update_throughput(void) {
    printf("\n========================================\n");
    printf("  PID Controller Update Throughput\n");
    printf("========================================\n");

    /* Build a valid schedule table */
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_TEMPERATURE);

    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    g.N = 10.0; g.b = 1.0; g.Kb = 0.3;

    double sv[] = {20.0, 40.0, 60.0, 80.0, 100.0};
    for (int i = 0; i < 5; i++) {
        g.Kp = 1.0 + 0.01 * sv[i];
        g.Ki = 0.1 + 0.002 * sv[i];
        g.Kd = 0.01;
        g.Ti = 10.0;
        g.Td = 0.1;
        gs_table_add_entry(&table, sv[i], &g, NULL);
    }

    struct {
        const char *name;
        gs_pid_form_t form;
    } forms[] = {
        {"Parallel",   GS_PID_PARALLEL_IDEAL},
        {"Series",     GS_PID_SERIES_INTERACTING},
        {"ISA Std",    GS_PID_ISA_STANDARD},
        {"Academic",   GS_PID_ACADEMIC_PARALLEL},
    };
    uint32_t n_forms = sizeof(forms) / sizeof(forms[0]);

    printf("%-16s %10s %15s %15s\n", "PID Form", "Time(ms)", "Updates", "Kupdates/sec");
    printf("%-16s %10s %15s %15s\n", "----------------", "----------", "---------------", "---------------");

    for (uint32_t f = 0; f < n_forms; f++) {
        gs_pid_state_t pid;
        gs_pid_init(&pid, "BENCH", forms[f].form);
        gs_pid_set_dt(&pid, 0.01);
        gs_pid_set_saturation(&pid, 100.0, -100.0);

        double sp = 50.0, pv = 48.0, sched = 50.0, output = 0.0;
        double sum = 0.0;
        clock_t t0 = clock();
        for (int i = 0; i < BENCH_LOOPS_PID; i++) {
            pv += 0.001 * output;
            sched = pv;
            gs_pid_update(&pid, &table, sp, pv, sched, &output);
            sum += output;
        }
        clock_t t1 = clock();
        double ms = elapsed_ms(t0, t1);
        double kups = (double)BENCH_LOOPS_PID / (ms * 1000.0);
        printf("%-16s %10.2f %15d %15.2f\n",
               forms[f].name, ms, BENCH_LOOPS_PID, kups);
        (void)sum;
    }
}

/* --------------------------------------------------------------------------
 * KB3: Table Operation Throughput -- binary search, add, remove
 * ------------------------------------------------------------------------*/
static void bench_table_operations(void) {
    printf("\n========================================\n");
    printf("  Schedule Table Operation Throughput\n");
    printf("========================================\n");

    /* Build a table with 128 entries */
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_TEMPERATURE);
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    g.N = 10.0; g.b = 1.0; g.Kb = 0.3;
    for (int i = 0; i < 128; i++) {
        g.Kp = 1.0 + 0.01 * i;
        g.Ki = 0.1 + 0.002 * i;
        g.Kd = 0.005 * i;
        gs_table_add_entry(&table, (double)i, &g, NULL);
    }

    printf("%-24s %10s %15s %12s\n", "Operation", "Time(ms)", "Iterations", "Ops/sec");
    printf("%-24s %10s %15s %12s\n", "------------------------", "----------", "---------------", "------------");

    /* Binary search bracket finding */
    {
        uint32_t lo, hi;
        int count = 0;
        clock_t t0 = clock();
        for (int i = 0; i < BENCH_LOOPS_TABLE; i++) {
            double v = fmod(i * 1.5, 127.5);
            if (gs_table_find_bracket(&table, v, &lo, &hi)) count++;
        }
        clock_t t1 = clock();
        double ms = elapsed_ms(t0, t1);
        printf("%-24s %10.2f %15d %12.0f\n", "Binary bracket find",
               ms, BENCH_LOOPS_TABLE, (double)BENCH_LOOPS_TABLE / (ms * 0.001));
        (void)count;
    }

    /* Nearest entry search */
    {
        int count = 0;
        clock_t t0 = clock();
        for (int i = 0; i < BENCH_LOOPS_TABLE; i++) {
            double v = fmod(i * 1.5, 127.5);
            if (gs_table_find_nearest(&table, v) >= 0) count++;
        }
        clock_t t1 = clock();
        double ms = elapsed_ms(t0, t1);
        printf("%-24s %10.2f %15d %12.0f\n", "Nearest entry search",
               ms, BENCH_LOOPS_TABLE, (double)BENCH_LOOPS_TABLE / (ms * 0.001));
        (void)count;
    }

    /* Table entry access (O(1)) */
    {
        int count = 0;
        clock_t t0 = clock();
        for (int i = 0; i < BENCH_LOOPS_TABLE * 10; i++) {
            if (gs_table_get_entry(&table, (uint32_t)(i % 128))) count++;
        }
        clock_t t1 = clock();
        double ms = elapsed_ms(t0, t1);
        printf("%-24s %10.2f %15d %12.0f\n", "Entry access (O(1))",
               ms, BENCH_LOOPS_TABLE * 10,
               (double)(BENCH_LOOPS_TABLE * 10) / (ms * 0.001));
        (void)count;
    }
}

/* --------------------------------------------------------------------------
 * KB4: Stability Analysis Throughput
 * ------------------------------------------------------------------------*/
static void bench_stability_analysis(void) {
    printf("\n========================================\n");
    printf("  Stability Analysis Throughput\n");
    printf("========================================\n");

    printf("%-24s %10s %15s %12s\n", "Operation", "Time(ms)", "Iterations", "Ops/sec");
    printf("%-24s %10s %15s %12s\n", "------------------------", "----------", "---------------", "------------");

    /* Routh-Hurwitz cubic check */
    {
        int stable = 0;
        clock_t t0 = clock();
        for (int i = 0; i < BENCH_LOOPS_STABLE; i++) {
            double a0 = 1.0, a1 = 3.0, a2 = 3.0 + 0.001 * i, a3 = 1.0;
            if (gs_stability_routh_hurwitz_cubic(a0, a1, a2, a3)) stable++;
        }
        clock_t t1 = clock();
        double ms = elapsed_ms(t0, t1);
        printf("%-24s %10.2f %15d %12.0f\n", "Routh-Hurwitz cubic",
               ms, BENCH_LOOPS_STABLE, (double)BENCH_LOOPS_STABLE / (ms * 0.001));
        (void)stable;
    }

    /* Spectral abscissa */
    {
        double sum = 0.0;
        clock_t t0 = clock();
        for (int i = 0; i < BENCH_LOOPS_STABLE; i++) {
            double K = 1.0 + 0.001 * i, tau = 2.0, L = 0.5;
            sum += gs_stability_spectral_abscissa(2.0, 0.2, 0.05, K, tau, L);
        }
        clock_t t1 = clock();
        double ms = elapsed_ms(t0, t1);
        printf("%-24s %10.2f %15d %12.0f\n", "Spectral abscissa",
               ms, BENCH_LOOPS_STABLE, (double)BENCH_LOOPS_STABLE / (ms * 0.001));
        (void)sum;
    }

    /* Frequency-domain margins */
    {
        int ok = 0;
        clock_t t0 = clock();
        for (int i = 0; i < BENCH_LOOPS_STABLE; i++) {
            double gm, pm;
            if (gs_stability_margins_fopdt(2.0, 0.2, 0.05, 1.0, 2.0, 0.5,
                                            &gm, &pm)) ok++;
        }
        clock_t t1 = clock();
        double ms = elapsed_ms(t0, t1);
        printf("%-24s %10.2f %15d %12.0f\n", "Freq-domain margins",
               ms, BENCH_LOOPS_STABLE, (double)BENCH_LOOPS_STABLE / (ms * 0.001));
        (void)ok;
    }
}

/* --------------------------------------------------------------------------
 * KB5: RLS Estimation Throughput
 * ------------------------------------------------------------------------*/
static void bench_rls_estimation(void) {
    printf("\n========================================\n");
    printf("  RLS Estimation Throughput\n");
    printf("========================================\n");

    gs_rls_estimator_t est;
    gs_adaptive_rls_init(&est, 0.98);

    double y = 0.0, u = 1.0;
    double a = exp(-0.1 / 3.0), K = 1.5;

    clock_t t0 = clock();
    for (int i = 0; i < BENCH_LOOPS_RLS; i++) {
        y = a * y + K * (1.0 - a) * u;
        gs_adaptive_rls_update(&est, u, y);
        if (i % 50 == 0) u = 1.0 + 0.1 * sin(0.01 * i);
    }
    clock_t t1 = clock();
    double ms = elapsed_ms(t0, t1);

    double K_est, tau_est;
    gs_adaptive_rls_get_params(&est, &K_est, &tau_est);

    printf("Updates:     %d\n", BENCH_LOOPS_RLS);
    printf("Time:        %.2f ms\n", ms);
    printf("Throughput:  %.0f updates/sec\n",
           (double)BENCH_LOOPS_RLS / (ms * 0.001));
    printf("Final estimates: K=%.4f (true=1.5), tau=%.4f (true=3.0)\n",
           K_est, tau_est);
}

/* --------------------------------------------------------------------------
 * KB6: Fuzzy Inference Throughput
 * ------------------------------------------------------------------------*/
static void bench_fuzzy_inference(void) {
    printf("\n========================================\n");
    printf("  Fuzzy Inference Throughput\n");
    printf("========================================\n");

    gs_fuzzy_schedule_t fs;
    gs_adaptive_fuzzy_init(&fs);

    double sum_kp = 0.0, sum_ki = 0.0, sum_kd = 0.0;
    double e = 0.0;

    clock_t t0 = clock();
    for (int i = 0; i < BENCH_LOOPS_FUZZY; i++) {
        e = 0.5 * sin(0.02 * i);
        double de = 0.3 * cos(0.02 * i);
        double kp_adj, ki_adj, kd_adj;
        gs_adaptive_fuzzy_infer(&fs, e, de, &kp_adj, &ki_adj, &kd_adj);
        sum_kp += kp_adj; sum_ki += ki_adj; sum_kd += kd_adj;
    }
    clock_t t1 = clock();
    double ms = elapsed_ms(t0, t1);

    printf("Inferences:  %d\n", BENCH_LOOPS_FUZZY);
    printf("Time:        %.2f ms\n", ms);
    printf("Throughput:  %.0f inferences/sec\n",
           (double)BENCH_LOOPS_FUZZY / (ms * 0.001));
    (void)sum_kp; (void)sum_ki; (void)sum_kd;
}

/* --------------------------------------------------------------------------
 * KB7: Memory Footprint Analysis
 * ------------------------------------------------------------------------*/
static void bench_memory_footprint(void) {
    printf("\n========================================\n");
    printf("  Memory Footprint Analysis\n");
    printf("========================================\n");

    printf("%-32s %8s\n", "Data Structure", "Bytes");
    printf("%-32s %8s\n", "--------------------------------", "--------");

    printf("%-32s %8zu\n", "pid_gain_set_t", sizeof(pid_gain_set_t));
    printf("%-32s %8zu\n", "schedule_entry_t", sizeof(schedule_entry_t));
    printf("%-32s %8zu\n", "gain_schedule_table_t", sizeof(gain_schedule_table_t));
    printf("%-32s %8zu\n", "schedule_entry_2d_t", sizeof(schedule_entry_2d_t));
    printf("%-32s %8zu\n", "gain_schedule_table_2d_t", sizeof(gain_schedule_table_2d_t));
    printf("%-32s %8zu\n", "gs_pid_state_t", sizeof(gs_pid_state_t));
    printf("%-32s %8zu\n", "gs_design_config_t", sizeof(gs_design_config_t));
    printf("%-32s %8zu\n", "gs_process_info_t", sizeof(gs_process_info_t));
    printf("%-32s %8zu\n", "gs_rls_estimator_t", sizeof(gs_rls_estimator_t));
    printf("%-32s %8zu\n", "gs_adaptive_performance_t", sizeof(gs_adaptive_performance_t));
    printf("%-32s %8zu\n", "gs_fuzzy_schedule_t", sizeof(gs_fuzzy_schedule_t));

    printf("\nTotal for 128-entry 1D schedule: %zu bytes\n",
           sizeof(gain_schedule_table_t));
    printf("Total for 32x32 2D schedule:    %zu bytes\n",
           sizeof(gain_schedule_table_2d_t));
}

/* =========================================================================
 * Main
 * =======================================================================*/
int main(void) {
    printf("========================================\n");
    printf("  Gain-Scheduled PID Benchmarks\n");
    printf("  Knowledge Coverage: L5 (Algorithm Performance)\n");
    printf("========================================\n");

    bench_interpolation_throughput();
    bench_pid_update_throughput();
    bench_table_operations();
    bench_stability_analysis();
    bench_rls_estimation();
    bench_fuzzy_inference();
    bench_memory_footprint();

    printf("\n========================================\n");
    printf("  All benchmarks completed.\n");
    printf("========================================\n");
    return 0;
}