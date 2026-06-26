/**
 * @file bench_override.c
 * @brief Micro-benchmarks for override selector operations
 */
#include <stdio.h>
#include <time.h>
#include "override_core.h"
#include "override_selector.h"
#include "override_pid.h"
#define ITER 1000000

int main(void) {
    printf("=== Override Selector Benchmarks ===\n\n");
    clock_t start, end;
    double vals[5] = {1.0, 5.0, 3.0, 2.0, 4.0};
    int valid[5] = {1, 1, 1, 1, 1};
    int idx;

    start = clock();
    for (int i = 0; i < ITER; i++) selector_high(vals, valid, 5, &idx);
    end = clock();
    printf("High-select (5ch x %d): %.3f ms\n", ITER,
           1000.0 * (end - start) / CLOCKS_PER_SEC);

    start = clock();
    for (int i = 0; i < ITER; i++) selector_low(vals, valid, 5, &idx);
    end = clock();
    printf("Low-select  (5ch x %d): %.3f ms\n", ITER,
           1000.0 * (end - start) / CLOCKS_PER_SEC);

    start = clock();
    for (int i = 0; i < ITER; i++) selector_median(vals, valid, 5, &idx);
    end = clock();
    printf("Median-sel  (5ch x %d): %.3f ms\n", ITER,
           1000.0 * (end - start) / CLOCKS_PER_SEC);

    override_controller_t ctrl;
    override_pid_init(&ctrl, 2.0, 10.0, 0.5, 8.0, 0.1, 0.0, 100.0, 1.0);
    ctrl.active = 1;
    start = clock();
    for (int i = 0; i < ITER; i++) override_pid_update(&ctrl, 50.0, 48.0, 0.1);
    end = clock();
    printf("PID update  (x %d): %.3f ms\n", ITER,
           1000.0 * (end - start) / CLOCKS_PER_SEC);

    printf("\n=== Benchmarks Complete ===\n");
    return 0;
}
