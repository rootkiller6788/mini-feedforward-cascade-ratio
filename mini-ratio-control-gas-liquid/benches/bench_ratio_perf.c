/**
 * @file bench_ratio_perf.c
 * @brief Performance benchmarks for ratio control functions.
 *
 * Benchmarks:
 *   1. Ratio computation throughput (MOPS)
 *   2. Cross-limiting execution time
 *   3. RLS update speed
 *   4. Blend optimization latency
 */

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>

/* External functions */
extern double ratio_compute_basic(double F_slave, double F_master);
extern double ratio_station(double R, double F_master);
extern double flow_ewma_filter(double x_raw, double y_prev, double Ts, double tau);
extern double gas_density_ideal(const void *gas);

/* Simple gas state for bench */
typedef struct { double molar_mass, pressure_pa, temperature_k; } gas_state_t;

static double get_time_ms(void)
{
    return (double)clock() / (double)CLOCKS_PER_SEC * 1000.0;
}

#define BENCH_ITERATIONS 1000000

static void bench_ratio_compute(void)
{
    double start = get_time_ms();
    double sum = 0.0;
    for (int i = 0; i < BENCH_ITERATIONS; i++) {
        sum += ratio_compute_basic((double)(i % 1000), (double)((i % 500) + 1));
    }
    double elapsed = get_time_ms() - start;
    double mops = (double)BENCH_ITERATIONS / elapsed / 1000.0;
    printf("ratio_compute_basic: %.2f MOPS (%.2f ms)\n", mops, elapsed);
    printf("  (check sum: %.0f to prevent optimization)\n", sum);
}

static void bench_ratio_station(void)
{
    double start = get_time_ms();
    double sum = 0.0;
    for (int i = 0; i < BENCH_ITERATIONS; i++) {
        sum += ratio_station(2.0, (double)((i % 500) + 1));
    }
    double elapsed = get_time_ms() - start;
    double mops = (double)BENCH_ITERATIONS / elapsed / 1000.0;
    printf("ratio_station:        %.2f MOPS (%.2f ms)\n", mops, elapsed);
    printf("  (check sum: %.0f)\n", sum);
}

static void bench_ewma_filter(void)
{
    double start = get_time_ms();
    double y = 0.0;
    double sum = 0.0;
    for (int i = 0; i < BENCH_ITERATIONS; i++) {
        y = flow_ewma_filter((double)i, y, 0.1, 2.0);
        sum += y;
    }
    double elapsed = get_time_ms() - start;
    double mops = (double)BENCH_ITERATIONS / elapsed / 1000.0;
    printf("flow_ewma_filter:     %.2f MOPS (%.2f ms)\n", mops, elapsed);
    printf("  (check sum: %.0f)\n", sum);
}

static void bench_gas_density(void)
{
    gas_state_t air = {0.02897, 101325.0, 298.15};
    double start = get_time_ms();
    double sum = 0.0;
    for (int i = 0; i < BENCH_ITERATIONS; i++) {
        air.pressure_pa = 101325.0 + i * 0.01;
        sum += gas_density_ideal(&air);
    }
    double elapsed = get_time_ms() - start;
    double mops = (double)BENCH_ITERATIONS / elapsed / 1000.0;
    printf("gas_density_ideal:    %.2f MOPS (%.2f ms)\n", mops, elapsed);
    printf("  (check sum: %.0f)\n", sum);
}

int main(void)
{
    printf("=== Ratio Control Performance Benchmarks ===\n");
    printf("Target: ISO 50001 energy management response time < 100ms\n");
    printf("Iterations per test: %d\n\n", BENCH_ITERATIONS);

    bench_ratio_compute();
    bench_ratio_station();
    bench_ewma_filter();
    bench_gas_density();

    printf("\n=== Benchmarks Complete ===\n");
    return 0;
}
