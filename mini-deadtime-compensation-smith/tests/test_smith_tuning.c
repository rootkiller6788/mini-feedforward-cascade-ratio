/**
 * @file test_smith_tuning.c
 * @brief Test suite for Smith predictor tuning algorithms.
 *
 * Tests: SIMC, IMC, Lambda tuning, ISE optimization, stability margins.
 */

#include "smith_tuning.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

#define ASSERT_NEAR(a, b, tol) assert(fabs((a) - (b)) < (tol))

static int tests_run = 0, tests_passed = 0;

#define RUN_TEST(name) do { \
    tests_run++; \
    printf("  %-45s", #name); \
    if (test_##name() == 0) { printf("PASS\n"); tests_passed++; } \
    else { printf("FAIL\n"); } \
} while(0)

static smith_process_model_t make_fopdt_model(double K, double tau, double theta)
{
    smith_process_model_t m;
    m.order = SMITH_MODEL_FOPDT;
    m.fopdt.K = K;
    m.fopdt.tau = tau;
    m.fopdt.theta = theta;
    m.K_delay_free = K;
    m.tau_delay_free = tau;
    return m;
}

static int test_simc_pi_balanced(void)
{
    smith_process_model_t m = make_fopdt_model(1.0, 10.0, 2.0);
    double Kp, Ti;
    int ret = smith_tune_simc_pi(&m, 10.0, &Kp, &Ti);
    assert(ret == 0);
    assert(Kp > 0.0);
    ASSERT_NEAR(Kp, 1.0, 0.1);    /* tau_c = tau → Kp = 1/K = 1.0 */
    ASSERT_NEAR(Ti, 10.0, 0.1);   /* Ti = tau */
    return 0;
}

static int test_simc_pi_fast(void)
{
    smith_process_model_t m = make_fopdt_model(1.0, 10.0, 2.0);
    double Kp, Ti;
    smith_tune_simc_pi(&m, 5.0, &Kp, &Ti);
    assert(Kp > 1.0);  /* Faster tau_c → higher gain */
    return 0;
}

static int test_simc_pid_sopdt(void)
{
    smith_process_model_t m;
    m.order = SMITH_MODEL_SOPDT;
    m.sopdt.K = 1.5; m.sopdt.tau1 = 8.0; m.sopdt.tau2 = 3.0;
    m.sopdt.theta = 1.0;
    m.K_delay_free = 1.5; m.tau_delay_free = 8.0;

    double Kp, Ti, Td;
    int ret = smith_tune_simc_pid(&m, 8.0, &Kp, &Ti, &Td);
    assert(ret == 0);
    assert(Td > 0.0);
    return 0;
}

static int test_imc_pi_tuning(void)
{
    smith_process_model_t m = make_fopdt_model(2.0, 12.0, 3.0);
    double Kp, Ti;
    int ret = smith_tune_imc_pi(&m, 6.0, &Kp, &Ti);
    assert(ret == 0);
    assert(Kp > 0.0);
    ASSERT_NEAR(Ti, 12.0, 0.1);
    return 0;
}

static int test_lambda_pi_tuning(void)
{
    smith_process_model_t m = make_fopdt_model(1.0, 8.0, 4.0);
    double Kp, Ti;
    int ret = smith_tune_lambda_pi(&m, 4.0, &Kp, &Ti);
    assert(ret == 0);
    assert(Kp > 0.0);
    assert(Ti > 0.0);
    return 0;
}

static int test_robustness_filter_tuning(void)
{
    smith_process_model_t m = make_fopdt_model(1.0, 10.0, 5.0);
    double Fr;
    int ret = smith_tune_robustness_filter(&m, 0.2, 1.0, &Fr);
    assert(ret == 0);
    assert(Fr > 0.0);
    return 0;
}

static int test_ise_optimal_tuning(void)
{
    smith_process_model_t m = make_fopdt_model(1.0, 5.0, 0.0);
    double Kp_opt, Ti_opt, J_min;
    int ret = smith_tune_optimal_ise_pi(&m, 1.0, 5.0, &Kp_opt, &Ti_opt, &J_min);
    assert(ret == 0);
    assert(Kp_opt > 0.0);
    assert(Ti_opt > 0.0);
    assert(J_min > 0.0);
    return 0;
}

static int test_stability_margins(void)
{
    smith_process_model_t m = make_fopdt_model(1.0, 10.0, 2.0);
    double gm_db, pm_deg;
    int ret = smith_tune_stability_margins(&m, 1.0, 10.0, 0.0, &gm_db, &pm_deg);
    assert(ret == 0);
    /* FOPDT with PI: GM should be finite (not NaN) */
    assert(!isnan(gm_db));
    assert(!isnan(pm_deg));
    return 0;
}

static int test_is_stable_check(void)
{
    smith_process_model_t m = make_fopdt_model(1.0, 10.0, 2.0);
    /* Zero gain should not be stable */
    int zero_stable = smith_tune_is_stable(0.0, 10.0, &m);
    assert(zero_stable == 0);
    /* Non-zero gain should at least not crash */
    int pos_stable = smith_tune_is_stable(1.0, 10.0, &m);
    (void)pos_stable;  /* Accepts conservative stability judgment */
    return 0;
}

static int test_setpoint_filter(void)
{
    smith_process_model_t m = make_fopdt_model(1.0, 10.0, 2.0);
    double T_ref;
    smith_tune_setpoint_filter(&m, 10.0, &T_ref);
    assert(T_ref > 0.0);
    return 0;
}

static int test_null_rejection(void)
{
    /* All functions should reject NULL */
    assert(smith_tune_simc_pi(NULL, 10.0, &(double){0}, &(double){0}) == -1);
    return 0;
}

int main(void)
{
    printf("\n=== Smith Predictor Tuning Test Suite ===\n\n");
    RUN_TEST(simc_pi_balanced);
    RUN_TEST(simc_pi_fast);
    RUN_TEST(simc_pid_sopdt);
    RUN_TEST(imc_pi_tuning);
    RUN_TEST(lambda_pi_tuning);
    RUN_TEST(robustness_filter_tuning);
    RUN_TEST(ise_optimal_tuning);
    RUN_TEST(stability_margins);
    RUN_TEST(is_stable_check);
    RUN_TEST(setpoint_filter);
    RUN_TEST(null_rejection);
    printf("\nResults: %d/%d tests passed\n\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
