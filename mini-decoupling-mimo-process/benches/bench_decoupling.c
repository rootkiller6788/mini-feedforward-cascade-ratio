/**
 * @file bench_decoupling.c
 * @brief Performance benchmark for MIMO decoupling operations.
 *
 * Measures execution time of key operations:
 *   - RGA computation
 *   - Matrix inversion (for decoupler design)
 *   - SVD decomposition
 *   - Decoupler application (real-time step)
 */

#include <stdio.h>
#include <time.h>
#include <math.h>
#include "../include/mimo_model.h"
#include "../include/mimo_interaction.h"
#include "../include/mimo_static_decoupling.h"
#include "../include/mimo_svd_decoupling.h"

#define BENCH_ITERATIONS 10000
#define BENCH(name, stmt) do { \
    clock_t start = clock(); \
    for (int i = 0; i < BENCH_ITERATIONS; i++) { stmt; } \
    clock_t end = clock(); \
    double elapsed = (double)(end - start) / CLOCKS_PER_SEC; \
    printf("  %-35s %10.6f s (%8.2f us/op)\n", \
           name, elapsed, elapsed / BENCH_ITERATIONS * 1e6); \
} while(0)

int main(void) {
    printf("============================================\n");
    printf("  MIMO Decoupling — Performance Benchmarks\n");
    printf("  Iterations: %d\n", BENCH_ITERATIONS);
    printf("============================================\n\n");

    MIMOModel model;
    mimo_model_init(&model, 2, 2, "Bench");
    mimo_model_set_fopdt(&model, 0, 0, 12.8, 16.7, 1.0);
    mimo_model_set_fopdt(&model, 0, 1, -18.9, 21.0, 3.0);
    mimo_model_set_fopdt(&model, 1, 0, 6.6, 10.9, 7.0);
    mimo_model_set_fopdt(&model, 1, 1, -19.4, 14.4, 3.0);

    double K[4];
    mimo_model_steady_state_gain(&model, K);

    RGAMatrix rga;
    StaticDecoupler sd;
    SVDDecoupler svd;
    double uc[] = { 0.1, -0.05 };
    double up[2];

    printf("Matrix Operations:\n");
    BENCH("Steady-state gain extraction",
          mimo_model_steady_state_gain(&model, K));
    BENCH("RGA computation (2x2)",
          mimo_rga_compute(K, 2, &rga));
    BENCH("Niederlinski Index",
          mimo_niederlinski_index(K, 2, NULL));
    BENCH("Condition number",
          mimo_condition_number(K, 2));
    BENCH("Static decoupler design D=K^{-1}",
          mimo_static_decoupler_design(&model, &sd));
    BENCH("Static decoupler apply (2x2)",
          mimo_static_decoupler_apply(&sd, uc, up));
    BENCH("SVD decomposition (2x2)",
          mimo_svd_decompose(K, 2, &svd));
    BENCH("Interaction quotient",
          mimo_interaction_quotient(&rga));

    printf("\n============================================\n");
    return 0;
}
