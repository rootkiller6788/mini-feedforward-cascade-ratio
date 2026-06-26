/**
 * @file bench_predictor_performance.c
 * @brief Performance benchmarks for Smith predictor — throughput and latency.
 *
 * Measures:
 *   1. Single-step latency (ns per call)
 *   2. Throughput (million steps/second)
 *   3. Memory usage (bytes for various delay buffer sizes)
 *   4. Scaling with delay buffer length
 */

#include "smith_predictor.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
static double get_time_us(void) {
    LARGE_INTEGER freq, counter;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&counter);
    return (double)counter.QuadPart * 1e6 / (double)freq.QuadPart;
}
#else
#include <sys/time.h>
static double get_time_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec * 1e6 + (double)tv.tv_usec;
}
#endif

#define BENCH_ITERATIONS 1000000

static void bench_single_step(const char *label, double theta, double Ts)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, theta, Ts,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    smith_predictor_set_pi(&sp, 1.0, 10.0, 1.0);

    /* Warm-up */
    for (int i = 0; i < 1000; i++) {
        smith_predictor_step(&sp, 1.0, 0.0);
    }

    /* Benchmark */
    double t_start = get_time_us();
    for (int i = 0; i < BENCH_ITERATIONS; i++) {
        smith_predictor_step(&sp, 1.0, (double)(i % 100) / 100.0);
    }
    double t_end = get_time_us();

    double elapsed_us = t_end - t_start;
    double ns_per_step = elapsed_us * 1000.0 / BENCH_ITERATIONS;
    double steps_per_sec = BENCH_ITERATIONS / (elapsed_us / 1e6);

    size_t buf_bytes = sp.delay_buf.capacity * sizeof(double);
    size_t total_bytes = sizeof(smith_predictor_t) + buf_bytes;

    printf("%-30s  theta=%5.1fs  Ts=%5.2fs  d=%3zu  "
           "latency=%7.1f ns  throughput=%10.1f steps/s  memory=%6zu B\n",
           label, theta, Ts, sp.delay_buf.delay_int,
           ns_per_step, steps_per_sec, total_bytes);

    smith_predictor_destroy(&sp);
}

int main(void)
{
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  Smith Predictor Performance Benchmarks                      ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    printf("Benchmarking %d iterations per test.\n\n", BENCH_ITERATIONS);
    printf("%-30s  %-11s %-8s %-5s %-13s %-22s %-9s\n",
           "Test", "Delay", "Ts", "d", "Latency", "Throughput", "Memory");

    /* Short delays */
    bench_single_step("FOPDT, small delay", 0.5, 0.1);
    bench_single_step("FOPDT, medium delay", 5.0, 0.1);
    bench_single_step("FOPDT, large delay", 50.0, 0.1);

    /* Effect of sampling rate */
    bench_single_step("FOPDT, fast sampling", 5.0, 0.01);
    bench_single_step("FOPDT, normal sampling", 5.0, 0.1);
    bench_single_step("FOPDT, slow sampling", 5.0, 0.5);

    /* SOPDT */
    smith_predictor_t sp_sopdt;
    smith_predictor_init_sopdt(
        &sp_sopdt, 1.5, 8.0, 3.0, 0.0, 0.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    smith_predictor_set_pi(&sp_sopdt, 1.0, 8.0, 1.0);

    double t_start = get_time_us();
    for (int i = 0; i < BENCH_ITERATIONS; i++) {
        smith_predictor_step(&sp_sopdt, 1.0, (double)(i % 100) / 100.0);
    }
    double t_end = get_time_us();
    double ns_per = (t_end - t_start) * 1000.0 / BENCH_ITERATIONS;

    printf("%-30s  theta=%5.1fs  Ts=%5.2fs  d=%3zu  "
           "latency=%7.1f ns  throughput=%10.1f steps/s\n",
           "SOPDT, with delay",
           sp_sopdt.model.sopdt.theta, 0.1,
           sp_sopdt.delay_buf.delay_int,
           ns_per, BENCH_ITERATIONS / ((t_end - t_start) / 1e6));

    smith_predictor_destroy(&sp_sopdt);

    printf("\n=== Benchmark Complete ===\n");
    return 0;
}
