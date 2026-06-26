/**
 * @file test_smith_robustness.c
 * @brief Tests for Smith predictor robustness analysis.
 */

#include "smith_robustness.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

#define ASSERT_NEAR(a, b, tol) assert(fabs((a) - (b)) < (tol))

static int tests_run = 0, tests_passed = 0;
#define RUN_TEST(n) do { tests_run++; printf("  %-45s", #n); \
    if (test_##n() == 0) { printf("PASS\n"); tests_passed++; } else { printf("FAIL\n"); } } while(0)

static smith_process_model_t make_fopdt(double K, double tau, double theta) {
    smith_process_model_t m;
    m.order = SMITH_MODEL_FOPDT;
    m.fopdt.K = K; m.fopdt.tau = tau; m.fopdt.theta = theta;
    m.K_delay_free = K; m.tau_delay_free = tau;
    return m;
}

static int test_sensitivity_at_low_freq(void) {
    smith_process_model_t m = make_fopdt(1.0, 10.0, 2.0);
    double S = smith_robustness_sensitivity(&m, 1.0, 10.0, 0.01);
    /* With integral action, |S(0)| ≈ 0 */
    assert(S < 1.0);
    return 0;
}

static int test_complementary_sensitivity(void) {
    smith_process_model_t m = make_fopdt(1.0, 10.0, 2.0);
    double T = smith_robustness_complementary_sensitivity(&m, 1.0, 10.0, 0.01);
    /* |T(0)| ≈ 1 for servo system */
    assert(T < 2.0);
    return 0;
}

static int test_peak_sensitivity_reasonable(void) {
    smith_process_model_t m = make_fopdt(1.0, 10.0, 0.0);
    double Ms = smith_robustness_peak_sensitivity(&m, 1.0, 10.0, 0.0, 0.01, 100.0, 100);
    /* Well-tuned PI on FOPDT → Ms should be reasonable */
    assert(Ms < 5.0);
    assert(Ms >= 1.0);
    return 0;
}

static int test_margins_finite(void) {
    smith_process_model_t m = make_fopdt(1.0, 10.0, 2.0);
    double gm, pm;
    int ret = smith_robustness_margins(&m, 1.0, 10.0, 0.0, &gm, &pm, NULL, NULL);
    assert(ret == 0);
    assert(!isnan(gm));
    assert(!isnan(pm));
    return 0;
}

static int test_delay_margin_positive(void) {
    smith_process_model_t m = make_fopdt(1.0, 10.0, 2.0);
    double dm = smith_robustness_delay_margin(&m, 1.0, 10.0, 0.0);
    assert(dm > 0.0);
    return 0;
}

static int test_gain_uncertainty_stable(void) {
    smith_process_model_t m = make_fopdt(1.0, 10.0, 2.0);
    /* Well-tuned, small uncertainty → should be robustly stable */
    int rs = smith_robustness_gain_uncertainty(&m, 1.0, 10.0, 0.1, 0.01, 100.0, 200);
    assert(rs == 1);
    return 0;
}

static int test_deadtime_uncertainty_small(void) {
    smith_process_model_t m = make_fopdt(1.0, 10.0, 2.0);
    int rs = smith_robustness_deadtime_uncertainty(&m, 1.0, 10.0, 0.1);
    /* Small dead-time uncertainty → should be stable */
    assert(rs == 1);
    return 0;
}

static int test_combined_robustness(void) {
    smith_process_model_t m = make_fopdt(1.0, 10.0, 2.0);
    int rs = smith_robustness_combined(&m, 1.0, 10.0, 0.05, 0.1, 0.01, 100.0, 200);
    assert(rs == 1);
    return 0;
}

static int test_monte_carlo_stability(void) {
    smith_process_model_t m = make_fopdt(1.0, 10.0, 2.0);
    int stable_cnt;
    double frac = smith_robustness_monte_carlo(&m, 1.0, 10.0, 0.1, 0.1, 0.5, 100, &stable_cnt);
    assert(frac >= 0.0 && frac <= 1.0);
    assert(stable_cnt >= 0 && stable_cnt <= 100);
    return 0;
}

static int test_lyapunov_stable(void) {
    smith_process_model_t m = make_fopdt(1.0, 10.0, 2.0);
    int ls = smith_robustness_lyapunov_stable(&m, 1.0, 10.0, 0.1);
    assert(ls == 1);  /* Well-tuned FOPDT with PI → Lyapunov stable */
    return 0;
}

static int test_nyquist_stable(void) {
    smith_process_model_t m = make_fopdt(1.0, 10.0, 2.0);
    int ns = smith_robustness_nyquist_criterion(&m, 1.0, 10.0, 500, NULL);
    assert(ns == 1);
    return 0;
}

int main(void) {
    printf("\n=== Smith Predictor Robustness Test Suite ===\n\n");
    RUN_TEST(sensitivity_at_low_freq);
    RUN_TEST(complementary_sensitivity);
    RUN_TEST(peak_sensitivity_reasonable);
    RUN_TEST(margins_finite);
    RUN_TEST(delay_margin_positive);
    RUN_TEST(gain_uncertainty_stable);
    RUN_TEST(deadtime_uncertainty_small);
    RUN_TEST(combined_robustness);
    RUN_TEST(monte_carlo_stability);
    RUN_TEST(lyapunov_stable);
    RUN_TEST(nyquist_stable);
    printf("\nResults: %d/%d tests passed\n\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
