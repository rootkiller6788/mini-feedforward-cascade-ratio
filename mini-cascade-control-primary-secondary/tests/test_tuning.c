/**
 * @file test_tuning.c
 * @brief Tests for cascade tuning algorithms (ZN, CC, SIMC, Lambda, PM, Ms)
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#include "cascade_types.h"
#include "cascade_tuning.h"
#include "cascade_stability.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int passed = 0, failed = 0;

#define TEST(desc, expr) do { \
    if (expr) { passed++; } \
    else { failed++; printf("FAIL: %s\n", desc); } \
} while(0)

#define TEST_FEQ(desc, a, b, tol) do { \
    if (fabs((a)-(b)) < (tol)) { passed++; } \
    else { failed++; printf("FAIL: %s (%.4f != %.4f)\n", desc, a, b); } \
} while(0)

static void test_zn_secondary(void)
{
    cascade_pid_params_t p;
    int rc = cascade_tune_zn_secondary(2.0, 10.0, 0.1, &p);
    TEST("ZN secondary success", rc == 0);
    TEST_FEQ("ZN secondary Kp", p.kp, 0.45*2.0, 0.01);
    TEST_FEQ("ZN secondary Ti", p.ti, 10.0/1.2, 0.01);
    TEST("ZN secondary Td=0", fabs(p.td) < 1e-9);

    /* Invalid inputs */
    rc = cascade_tune_zn_secondary(-1.0, 10.0, 0.1, &p);
    TEST("ZN rejects Ku<=0", rc != 0);
    rc = cascade_tune_zn_secondary(2.0, 10.0, 0.1, NULL);
    TEST("ZN rejects NULL result", rc != 0);
}

static void test_zn_primary(void)
{
    cascade_pid_params_t p;
    int rc = cascade_tune_zn_primary(2.0, 10.0, 0.1, &p);
    TEST("ZN primary success", rc == 0);
    TEST_FEQ("ZN primary Kp", p.kp, 0.6*2.0, 0.01);
    TEST_FEQ("ZN primary Ti", p.ti, 10.0/2.0, 0.01);
    TEST_FEQ("ZN primary Td", p.td, 10.0/8.0, 0.01);
}

static void test_cohen_coon(void)
{
    cascade_fopdt_model_t model;
    model.K = 1.0; model.tau = 10.0; model.theta = 2.0;
    model.type = CASCADE_MODEL_FOPDT;

    cascade_pid_params_t p;
    int rc = cascade_tune_cohen_coon_pi(&model, &p);
    TEST("CC PI success", rc == 0);
    TEST("CC PI Kp > 0", p.kp > 0.0);
    TEST("CC PI Ti > 0", p.ti > 0.0);

    rc = cascade_tune_cohen_coon_pid(&model, &p);
    TEST("CC PID success", rc == 0);
    TEST("CC PID Kp > 0", p.kp > 0.0);
    TEST("CC PID Td > 0", p.td > 0.0);

    /* Invalid model */
    model.K = 0.0;
    rc = cascade_tune_cohen_coon_pi(&model, &p);
    TEST("CC rejects K=0", rc != 0);

    rc = cascade_tune_cohen_coon_pi(NULL, &p);
    TEST("CC NULL model", rc != 0);
}

static void test_simc(void)
{
    cascade_fopdt_model_t m;
    m.K = 2.0; m.tau = 20.0; m.theta = 5.0;
    m.type = CASCADE_MODEL_FOPDT;

    cascade_pid_params_t p;
    int rc = cascade_tune_simc_secondary(&m, 5.0, &p);
    TEST("SIMC secondary success", rc == 0);
    TEST("SIMC Kp > 0", p.kp > 0.0);
    TEST("SIMC Ti > 0", p.ti > 0.0);

    rc = cascade_tune_simc_primary(&m, 15.0, &p);
    TEST("SIMC primary success", rc == 0);

    /* Invalid tau_c */
    rc = cascade_tune_simc_secondary(&m, -1.0, &p);
    TEST("SIMC rejects tau_c <= 0", rc != 0);

    rc = cascade_tune_simc_secondary(NULL, 5.0, &p);
    TEST("SIMC NULL model", rc != 0);
}

static void test_lambda(void)
{
    cascade_fopdt_model_t m;
    m.K = 1.0; m.tau = 10.0; m.theta = 2.0;
    m.type = CASCADE_MODEL_FOPDT;

    cascade_pid_params_t p;
    int rc = cascade_tune_lambda_pi(&m, 6.0, &p);
    TEST("Lambda PI success", rc == 0);
    TEST("Lambda PI Kp > 0", p.kp > 0.0);

    rc = cascade_tune_lambda_pid(&m, 6.0, &p);
    TEST("Lambda PID success", rc == 0);
    TEST("Lambda PID Td > 0", p.td > 0.0);
}

static void test_sequential(void)
{
    cascade_fopdt_model_t sec, pri;
    sec.K = 0.5; sec.tau = 3.0; sec.theta = 1.0;
    sec.type = CASCADE_MODEL_FOPDT;
    pri.K = 1.0; pri.tau = 60.0; pri.theta = 5.0;
    pri.type = CASCADE_MODEL_FOPDT;

    cascade_tuning_result_t result;

    for (int method = 0; method < 4; method++) {
        int rc = cascade_tune_sequential(&sec, &pri, method, &result);
        TEST("Sequential tuning success", rc == 0);
        TEST("Sequential pri Kp > 0", result.primary_params.kp > 0.0);
        TEST("Sequential sec Kp > 0", result.secondary_params.kp > 0.0);
        TEST("Sequential update ratio set",
             result.recommended_update_ratio >= CASCADE_UPDATE_RATIO_MIN);
    }

    /* Invalid inputs */
    int rc = cascade_tune_sequential(NULL, &pri, 0, &result);
    TEST("Sequential NULL sec model", rc != 0);
    rc = cascade_tune_sequential(&sec, &pri, 99, &result);
    TEST("Sequential invalid method", rc != 0);
}

static void test_phase_margin(void)
{
    cascade_fopdt_model_t m;
    m.K = 1.0; m.tau = 10.0; m.theta = 2.0;
    m.type = CASCADE_MODEL_FOPDT;

    cascade_pid_params_t p;
    int rc = cascade_tune_phase_margin(&m, 45.0, false, &p);
    TEST("PM PI tuning success", rc == 0);
    TEST("PM PI Kp > 0", p.kp > 0.0);

    rc = cascade_tune_phase_margin(&m, 60.0, true, &p);
    TEST("PM PID tuning success", rc == 0);

    /* Out of range PM */
    rc = cascade_tune_phase_margin(&m, 10.0, false, &p);
    TEST("PM rejects very low PM", rc != 0);
}

static void test_ms_constrained(void)
{
    cascade_fopdt_model_t m;
    m.K = 1.0; m.tau = 10.0; m.theta = 2.0;
    m.type = CASCADE_MODEL_FOPDT;

    cascade_pid_params_t p;
    int rc = cascade_tune_max_sensitivity(&m, 1.7, &p);
    TEST("Ms constrained success", rc == 0);
    TEST("Ms constrained Kp > 0", p.kp > 0.0);
}

static void test_verify_margins(void)
{
    cascade_pid_params_t sp, pp;
    sp.kp = 1.0; sp.ti = 10.0; sp.td = 0.0;
    pp.kp = 0.5; pp.ti = 60.0; pp.td = 5.0;

    cascade_system_model_t model;
    memset(&model, 0, sizeof(model));
    model.secondary_process.K = 0.5;
    model.secondary_process.tau = 3.0;
    model.secondary_process.theta = 1.0;
    model.primary_process.K = 1.0;
    model.primary_process.tau = 60.0;
    model.primary_process.theta = 5.0;

    cascade_stability_t stab;
    int rc = cascade_tuning_verify_margins(&pp, &sp, &model, &stab);
    /* For PI secondary with these params, should be stable */
    TEST("Verify margins returns int", rc >= -1 && rc <= 1);
}

static void test_compare_methods(void)
{
    cascade_fopdt_model_t m;
    m.K = 1.0; m.tau = 10.0; m.theta = 2.0;
    m.type = CASCADE_MODEL_FOPDT;

    int methods[4] = {0, 1, 2, 3};
    cascade_tuning_result_t results[4];
    int best = cascade_tuning_compare_methods(&m, 4, methods, results);
    TEST("Compare returns valid index", best >= 0 && best < 4);
}

/*-------------------------------------------------------------------------*/
/* Stability tests */
/*-------------------------------------------------------------------------*/

static void test_frequency_response(void)
{
    cascade_fopdt_model_t m;
    m.K = 1.0; m.tau = 10.0; m.theta = 2.0;

    double mag, phase;
    cascade_fopdt_frequency_response(&m, 0.1, &mag, &phase);
    TEST("FR FOPDT mag > 0", mag > 0.0);

    cascade_fopdt_frequency_response(&m, 0.0, &mag, &phase);
    TEST_FEQ("FR FOPDT DC gain = K", mag, 1.0, 1e-6);

    cascade_fopdt_frequency_response(NULL, 0.1, &mag, &phase);
    TEST("FR FOPDT NULL safe", 1);
}

static void test_pid_frequency_response(void)
{
    cascade_pid_params_t pid;
    memset(&pid, 0, sizeof(pid));
    pid.kp = 2.0; pid.ti = 10.0; pid.td = 1.0; pid.tf = 0.1;

    double mag, phase;
    cascade_pid_frequency_response(&pid, 0.1, 1.0, &mag, &phase);
    TEST("FR PID mag > 0", mag > 0.0);

    /* At ω=0, PID magnitude should be large (integral) */
    cascade_pid_frequency_response(&pid, 1e-12, 1.0, &mag, &phase);
    TEST("FR PID DC mag large", mag > 1.0);
}

static void test_cascade_stability(void)
{
    cascade_pid_params_t pp, sp;
    pp.kp = 0.5; pp.ti = 60.0; pp.td = 5.0; pp.tf = 0.5;
    sp.kp = 1.0; sp.ti = 10.0; sp.td = 0.0; sp.tf = 0.1;

    cascade_fopdt_model_t pri, sec;
    pri.K = 1.0; pri.tau = 60.0; pri.theta = 5.0;
    sec.K = 0.5; sec.tau = 3.0; sec.theta = 1.0;

    cascade_stability_t stab;
    int rc = cascade_compute_stability(&pp, &sp, &pri, &sec, 1.0, 50, &stab);
    TEST("Cascade stability computed", rc >= -1 && rc <= 1);

    /* Nyquist check */
    rc = cascade_check_nyquist_stability(&pp, &sp, &pri, &sec, 1.0, 50);
    TEST("Nyquist check runs", rc >= -1 && rc <= 0);

    /* Robustness */
    double ri = cascade_robustness_index(&stab);
    TEST("Robustness index ≥ 0", ri >= 0.0);

    /* Secondary stability */
    cascade_stability_t stab2;
    rc = cascade_secondary_stability(&sp, &sec, 1.0, 50, &stab2);
    TEST("Secondary stability computed", rc >= -1 && rc <= 1);

    /* Delay impact */
    double gm_db;
    rc = cascade_stability_delay_impact(0.5, &pp, &sp, &pri, &sec, 1.0, &gm_db);
    TEST("Delay impact computed", rc >= -1 && rc <= 0);
}

/*-------------------------------------------------------------------------*/

int main(void)
{
    printf("=== Cascade Tuning & Stability Tests ===\n\n");

    printf("--- Ziegler-Nichols ---\n");
    test_zn_secondary();
    test_zn_primary();

    printf("\n--- Cohen-Coon ---\n");
    test_cohen_coon();

    printf("\n--- SIMC ---\n");
    test_simc();

    printf("\n--- Lambda ---\n");
    test_lambda();

    printf("\n--- Sequential ---\n");
    test_sequential();

    printf("\n--- Phase Margin ---\n");
    test_phase_margin();

    printf("\n--- Ms-Constrained ---\n");
    test_ms_constrained();

    printf("\n--- Verify Margins ---\n");
    test_verify_margins();

    printf("\n--- Compare Methods ---\n");
    test_compare_methods();

    printf("\n--- Frequency Response ---\n");
    test_frequency_response();
    test_pid_frequency_response();

    printf("\n--- Cascade Stability ---\n");
    test_cascade_stability();

    printf("\n========================================\n");
    printf("Results: %d passed, %d failed\n", passed, failed);

    return (failed > 0) ? 1 : 0;
}
