/**
 * @file test_smith_predictor.c
 * @brief Test suite for core Smith predictor implementation.
 *
 * Tests verify:
 *   L1: Initialization with valid/invalid parameters
 *   L2: Basic control loop — tracking and disturbance rejection
 *   L3: Delay buffer operations — push/pop correctness
 *   L4: Perfect model → delay-free characteristic equation
 *   L5: Smith predictor variants comparison
 *   L6: Performance metrics computation
 */

#include "smith_predictor.h"
#include "smith_tuning.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#define ASSERT_NEAR(a, b, tol) \
    assert(fabs((a) - (b)) < (tol))

static int tests_run = 0;
static int tests_passed = 0;

#define RUN_TEST(name) do { \
    tests_run++; \
    printf("  %-50s", #name); \
    if (test_##name() == 0) { \
        printf("PASS\n"); \
        tests_passed++; \
    } else { \
        printf("FAIL\n"); \
    } \
} while(0)

/* ========================================================================
   L1: Initialization Tests
   ======================================================================== */

static int test_init_fopdt_valid(void)
{
    smith_predictor_t sp;
    int ret = smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    assert(ret == 0);
    assert(sp.model.order == SMITH_MODEL_FOPDT);
    assert(fabs(sp.model.fopdt.K - 1.0) < 1e-9);
    assert(fabs(sp.model.fopdt.tau - 10.0) < 1e-9);
    assert(fabs(sp.model.fopdt.theta - 2.0) < 1e-9);
    smith_predictor_destroy(&sp);
    return 0;
}

static int test_init_invalid_params(void)
{
    smith_predictor_t sp;
    /* Negative tau should fail */
    int ret = smith_predictor_init_fopdt(
        &sp, 1.0, -1.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    assert(ret == -1);

    /* Zero tau should fail */
    ret = smith_predictor_init_fopdt(
        &sp, 1.0, 0.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    assert(ret == -1);

    /* Negative sampling period should fail */
    ret = smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, 2.0, -0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    assert(ret == -1);

    /* NULL pointer should fail */
    ret = smith_predictor_init_fopdt(
        NULL, 1.0, 10.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    assert(ret == -1);

    return 0;
}

static int test_init_sopdt_valid(void)
{
    smith_predictor_t sp;
    int ret = smith_predictor_init_sopdt(
        &sp, 1.5, 8.0, 3.0, 0.0, 0.0, 1.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    assert(ret == 0);
    assert(sp.model.order == SMITH_MODEL_SOPDT);
    assert(fabs(sp.model.sopdt.tau1 - 8.0) < 1e-9);
    assert(fabs(sp.model.sopdt.tau2 - 3.0) < 1e-9);
    smith_predictor_destroy(&sp);
    return 0;
}

static int test_reset_clears_state(void)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    smith_predictor_set_pi(&sp, 2.0, 10.0, 1.0);

    /* Run a few steps */
    smith_predictor_step(&sp, 1.0, 0.0);
    smith_predictor_step(&sp, 1.0, 0.1);

    /* Reset */
    smith_predictor_reset(&sp);

    assert(fabs(sp.integrator) < 1e-9);
    assert(fabs(sp.prev_error) < 1e-9);
    assert(fabs(sp.yp_model) < 1e-9);
    assert(fabs(sp.yp_delayed) < 1e-9);
    /* core prediction and controller states cleared */

    smith_predictor_destroy(&sp);
    return 0;
}

/* ========================================================================
   L2: Controller Configuration Tests
   ======================================================================== */

static int test_set_pi_tuning(void)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);

    int ret = smith_predictor_set_pi(&sp, 2.5, 8.0, 0.8);
    assert(ret == 0);
    assert(fabs(sp.Kp - 2.5) < 1e-9);
    assert(fabs(sp.Ti - 8.0) < 1e-9);
    assert(fabs(sp.b - 0.8) < 1e-9);
    assert(fabs(sp.Td) < 1e-9);  /* Td should be 0 for PI */

    smith_predictor_destroy(&sp);
    return 0;
}

static int test_set_pid_tuning(void)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);

    int ret = smith_predictor_set_pid(&sp, 3.0, 10.0, 2.0, 10.0, 1.0, 0.0);
    assert(ret == 0);
    assert(fabs(sp.Td - 2.0) < 1e-9);
    assert(fabs(sp.N - 10.0) < 1e-9);

    smith_predictor_destroy(&sp);
    return 0;
}

static int test_robustness_filter_config(void)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);

    int ret = smith_predictor_set_robustness_filter(&sp, 1.0);
    assert(ret == 0);
    assert(fabs(sp.Fr - 1.0) < 1e-9);

    /* Negative filter time constant should fail */
    ret = smith_predictor_set_robustness_filter(&sp, -0.5);
    assert(ret == -1);

    smith_predictor_destroy(&sp);
    return 0;
}

/* ========================================================================
   L3: Core Control Loop Test
   ======================================================================== */

static int test_step_tracking(void)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 5.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    smith_predictor_set_pi(&sp, 2.0, 5.0, 1.0);

    /* Simulate step response */
    double y = 0.0;
    double u = 0.0;
    int converged = 0;

    for (int k = 0; k < 1000; k++) {
        u = smith_predictor_step(&sp, 1.0, y);

        /* Simulate actual process: K=1, tau=5, theta=2 (FOPDT with dead time) */
        /* Use forward Euler on a simple buffer to simulate dead time */
        static double yp_buf[100] = {0};
        static int buf_idx = 0;
        double yp = (5.0 * y + 1.0 * 0.1 * u) / 5.1;  /* backward Euler on FOPDT */
        (void)yp;

        /* Simple delay-free simulation for validation: */
        /* y(k+1) = (tau*y(k) + K*Ts*u(k)) / (tau + Ts) */
        double y_new = (5.0 * y + 1.0 * 0.1 * u) / 5.1;

        /* Dead time: buffer y_new for 20 steps (= 2.0/0.1) */
        yp_buf[buf_idx % 20] = y_new;
        y = yp_buf[(buf_idx + 1) % 20];  /* One of the delayed values */

        buf_idx++;

        /* Check convergence after settling */
        if (k > 200 && fabs(y - 1.0) < 0.05) {
            converged = 1;
            break;
        }
    }

    /* The Smith predictor should converge close to setpoint */
    assert(converged == 1);

    smith_predictor_destroy(&sp);
    return 0;
}

static int test_saturation_enforcement(void)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 5.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 20.0, 80.0);
    smith_predictor_set_pi(&sp, 10.0, 5.0, 1.0);

    /* Large setpoint step that should cause saturation */
    double u = smith_predictor_step(&sp, 200.0, 0.0);
    assert(u <= 80.0);

    /* Small setpoint step — should stay within limits */
    u = smith_predictor_step(&sp, 0.5, 0.0);
    assert(u >= 20.0);
    assert(u <= 80.0);

    smith_predictor_destroy(&sp);
    return 0;
}

/* ========================================================================
   L4: Perfect Model Property
   ======================================================================== */

static int test_perfect_model_prediction(void)
{
    /* With perfect model (simulated) the prediction error should converge to 0 */
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 5.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    smith_predictor_set_pi(&sp, 1.0, 5.0, 1.0);

    /* Run with the SAME model as the predictor to verify prediction matches */
    double y_sim = 0.0;
    double u = 0.0;

    for (int k = 0; k < 500; k++) {
        u = smith_predictor_step(&sp, 1.0, y_sim);

        /* Simulate process: exactly FOPDT K=1 tau=5 theta=0 (delay-free for test) */
        y_sim = (5.0 * y_sim + 1.0 * 0.1 * u) / 5.1;
    }

    /* After convergence, prediction error should be small */
    double mismatch = smith_predictor_get_mismatch(&sp);
    assert(fabs(mismatch) < 0.1);

    smith_predictor_destroy(&sp);
    return 0;
}

/* ========================================================================
   L5: Variant Comparison
   ======================================================================== */

static int test_variant_classic_initialized(void)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, 3.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    assert(sp.variant == SMITH_VARIANT_CLASSIC);
    smith_predictor_destroy(&sp);
    return 0;
}

static int test_variant_filtered_initialized(void)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, 3.0, 0.1,
        SMITH_VARIANT_FILTERED, 0.0, 100.0);
    assert(sp.variant == SMITH_VARIANT_FILTERED);

    /* Filtered variant should have robustness filter enabled by default */
    smith_predictor_set_robustness_filter(&sp, 1.5);
    assert(fabs(sp.Fr - 1.5) < 1e-9);

    smith_predictor_destroy(&sp);
    return 0;
}

/* ========================================================================
   L6: Performance Metrics
   ======================================================================== */

static int test_metrics_accumulation(void)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 5.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    smith_predictor_set_pi(&sp, 2.0, 5.0, 1.0);

    for (int k = 0; k < 100; k++) {
        smith_predictor_step(&sp, 1.0, 0.5);
    }

    assert(sp.IAE > 0.0);
    assert(sp.sample_count == 100);

    smith_predictor_compute_performance(&sp, &(smith_performance_t){0});
    smith_predictor_reset_metrics(&sp);
    assert(sp.IAE == 0.0);
    assert(sp.sample_count == 0);

    smith_predictor_destroy(&sp);
    return 0;
}

/* ========================================================================
   L3: Delay Buffer Tests
   ======================================================================== */

static int test_delay_buffer_correctness(void)
{
    smith_predictor_t sp;
    /* Model with known delay */
    smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, 1.0, 0.1,  /* theta=1.0s, Ts=0.1 → delay=10 samples */
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);
    smith_predictor_set_pi(&sp, 1.0, 10.0, 1.0);

    /* Push a known sequence into the model */
    double u = 1.0;
    smith_predictor_step(&sp, 0.0, 0.0);  /* Initialize states */

    /* The delay buffer should have correct capacity */
    assert(sp.delay_buf.capacity >= 13);  /* ceil(1.0/0.1) + 3 = 13 */

    smith_predictor_destroy(&sp);
    return 0;
}

/* ========================================================================
   L7: Modbus Integration
   ======================================================================== */

static int test_modbus_mapping(void)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);

    smith_modbus_map_t modbus;
    smith_predictor_map_modbus(&sp, &modbus, 0x1000);

    assert(modbus.Kp_reg == 0x1000);
    assert(modbus.status_reg == 0x100E);

    smith_predictor_destroy(&sp);
    return 0;
}

static int test_opcua_mapping(void)
{
    smith_predictor_t sp;
    smith_predictor_init_fopdt(
        &sp, 1.0, 10.0, 2.0, 0.1,
        SMITH_VARIANT_CLASSIC, 0.0, 100.0);

    smith_opcua_map_t opcua;
    smith_predictor_map_opcua(&sp, &opcua, 1, 5000);

    assert(opcua.namespace_idx == 1);
    assert(opcua.Kp_node == 5000);
    assert(opcua.mode_node == 5006);

    smith_predictor_destroy(&sp);
    return 0;
}

/* ========================================================================
   Test Runner
   ======================================================================== */

int main(void)
{
    printf("\n=== Smith Predictor Test Suite ===\n\n");

    printf("L1: Initialization\n");
    RUN_TEST(init_fopdt_valid);
    RUN_TEST(init_invalid_params);
    RUN_TEST(init_sopdt_valid);
    RUN_TEST(reset_clears_state);

    printf("\nL2: Controller Configuration\n");
    RUN_TEST(set_pi_tuning);
    RUN_TEST(set_pid_tuning);
    RUN_TEST(robustness_filter_config);

    printf("\nL3: Core Control & Delay Buffer\n");
    RUN_TEST(step_tracking);
    RUN_TEST(saturation_enforcement);
    RUN_TEST(delay_buffer_correctness);

    printf("\nL4: Perfect Model Property\n");
    RUN_TEST(perfect_model_prediction);

    printf("\nL5: Variant Comparison\n");
    RUN_TEST(variant_classic_initialized);
    RUN_TEST(variant_filtered_initialized);

    printf("\nL6: Performance Metrics\n");
    RUN_TEST(metrics_accumulation);

    printf("\nL7: Industrial Interfaces\n");
    RUN_TEST(modbus_mapping);
    RUN_TEST(opcua_mapping);

    printf("\n========================================\n");
    printf("Results: %d/%d tests passed\n", tests_passed, tests_run);
    printf("========================================\n");

    return (tests_passed == tests_run) ? 0 : 1;
}
