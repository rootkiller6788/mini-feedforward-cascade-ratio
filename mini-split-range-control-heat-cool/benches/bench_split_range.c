/**
 * @file bench_split_range.c
 * @brief Performance benchmarks for split-range control operations
 *
 * Benchmarks key operations to measure computational cost:
 *   - Split distribution (core hot path)
 *   - PID incremental computation
 *   - Valve characteristic lookups
 *   - Slew-rate limiting
 */

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include "split_range_control.h"

#define ITERATIONS 1000000

static double benchmark_split_distribute(void) {
    split_range_scheme_t scheme;
    split_init_heat_cool_scheme(&scheme);
    double positions[6];
    double sum = 0.0;

    clock_t start = clock();
    for (int i = 0; i < ITERATIONS; i++) {
        double co = (double)(i % 101);
        split_distribute_output(&scheme, co, positions);
        sum += positions[0] + positions[1];
    }
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("  split_distribute_output: %.2f ns/call (%.3f s for %d iters)\n",
           elapsed * 1e9 / ITERATIONS, elapsed, ITERATIONS);
    return elapsed;
}

static double benchmark_pid_incremental(void) {
    split_range_pid_params_t params;
    split_pid_init_params(&params, 2.0, 60.0, 15.0, 1.0);
    split_range_pid_state_t state;
    split_pid_reset_state(&state);
    double sum = 0.0;

    clock_t start = clock();
    for (int i = 0; i < ITERATIONS; i++) {
        double pv = 25.0 + (double)(i % 100);
        sum += split_pid_incremental(&params, &state, 80.0, pv);
    }
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("  split_pid_incremental:  %.2f ns/call (%.3f s for %d iters)\n",
           elapsed * 1e9 / ITERATIONS, elapsed, ITERATIONS);
    return elapsed;
}

static double benchmark_valve_characteristic(void) {
    double sum = 0.0;

    clock_t start = clock();
    for (int i = 0; i < ITERATIONS; i++) {
        double x = (double)(i % 101) / 100.0;
        sum += split_valve_characteristic_inverse(x, SPLIT_VALVE_EQUAL_PCT, 50.0);
        sum += split_valve_characteristic_forward(x, SPLIT_VALVE_MODIFIED_PARABOLIC, 50.0);
    }
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("  valve characteristics:   %.2f ns/call (%.3f s for %d iters)\n",
           elapsed * 1e9 / (ITERATIONS * 2), elapsed, ITERATIONS);
    return elapsed;
}

static double benchmark_slew_rate(void) {
    double sum = 0.0;

    clock_t start = clock();
    for (int i = 0; i < ITERATIONS; i++) {
        double cur = (double)(i % 101);
        double tgt = (double)((i + 10) % 101);
        sum += split_slew_rate_limit(cur, tgt, 25.0, 1.0);
    }
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("  split_slew_rate_limit:   %.2f ns/call (%.3f s for %d iters)\n",
           elapsed * 1e9 / ITERATIONS, elapsed, ITERATIONS);
    return elapsed;
}

static double benchmark_control_cycle(void) {
    split_range_controller_t ctrl = split_control_create_reactor();
    ctrl.enabled = true;
    ctrl.pv_context.setpoint = 80.0;
    ctrl.pv_context.process_variable = 25.0;
    ctrl.pid_params.sample_time_sec = 1.0;
    double sum = 0.0;

    clock_t start = clock();
    for (int i = 0; i < ITERATIONS / 10; i++) {
        split_control_execute(&ctrl, 1.0);
        sum += ctrl.controller_output;
    }
    clock_t end = clock();
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("  split_control_execute:   %.2f us/call (%.3f s for %d iters)\n",
           elapsed * 1e6 / (ITERATIONS / 10), elapsed, ITERATIONS / 10);
    return elapsed;
}

int main(void) {
    printf("============================================\n");
    printf("  Split-Range Control Benchmarks\n");
    printf("  Iterations per test: %d\n", ITERATIONS);
    printf("============================================\n\n");

    srand((unsigned int)time(NULL));

    double total = 0.0;
    total += benchmark_split_distribute();
    total += benchmark_pid_incremental();
    total += benchmark_valve_characteristic();
    total += benchmark_slew_rate();
    total += benchmark_control_cycle();

    printf("\n  Total benchmark time: %.3f s\n", total);
    printf("\nBench complete.\n");
    return 0;
}
