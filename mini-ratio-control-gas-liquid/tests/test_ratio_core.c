/**
 * @file test_ratio_core.c
 * @brief Tests for core ratio computation functions.
 *
 * Tests: ratio computation, unit conversion, flow filtering,
 *        stoichiometric constants, ratio validation.
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>

/* Declare functions under test from ratio_core.c */
extern double ratio_compute_basic(double F_slave, double F_master);
extern int    ratio_setpoint_validate(double R_sp, double R_min, double R_max);
extern double ratio_clamp(double R, double R_min, double R_max);
extern double ratio_station(double R, double F_master);
extern double ratio_inverse(double SP_slave, double F_master);
extern double ratio_master_required(double F_slave, double R);
extern double ratio_error_compute(double R_actual, double R_sp);
extern double ratio_error_percent(double R_actual, double R_sp);
extern double flow_volume_to_mass(double F_volume, double density);
extern double flow_mass_to_volume(double F_mass, double density);
extern double flow_mass_to_molar(double F_mass, double molar_mass);
extern double flow_molar_to_mass(double F_molar, double molar_mass);
extern double flow_unit_convert(double flow, int from_unit, int to_unit,
                                 double density, double molar_mass, double full_scale);
extern double flow_ewma_filter(double x_raw, double y_prev, double Ts, double tau);
extern double flow_moving_average_update(double new_sample, double oldest_sample,
                                          double sum_old, int window_size, double *sum_new);
extern int    ratio_validate_measurement(double flow, int quality);
extern int    ratio_within_tolerance(double R_actual, double R_sp, double tolerance);
extern int    ratio_detect_divergence(double error_k, double error_km1, double error_km2);
extern double stoichiometric_afr_get(int fuel_code);
extern double lambda_from_afr(double AFR_actual, double AFR_stoich);
extern double excess_air_percent(double lambda);
extern double combustion_efficiency_loss(double excess_air_pct, double k_fuel);
extern double ratio_schedule_linear(double condition, double cond_min, double cond_max,
                                     double R_min, double R_max);
extern double ratio_deadband_apply(double ratio_error, double deadband);
extern double ratio_detect_hunting(double error_k, double error_km1, double error_km2, double error_km3);
extern double cascade_ratio_adjust(double outer_output, double output_min,
                                    double output_max, double R_trim_range);
extern int    cascade_anti_windup_check(double slave_output, double slave_output_max,
                                         double slave_output_min);

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { tests_run++; printf("  TEST: %s ... ", name); } while(0)
#define PASS() do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); } while(0)
#define ASSERT_EQ(a, b, eps) do { \
    if (fabs((a)-(b)) > (eps)) { FAIL("assertion failed"); return; } \
} while(0)
#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { FAIL("assertion failed"); return; } \
} while(0)

/* ================================================================
 * L1: Basic ratio computation
 * ================================================================ */
static void test_ratio_compute_basic(void)
{
    TEST("ratio_compute_basic normal");
    ASSERT_EQ(ratio_compute_basic(100.0, 50.0), 2.0, 1e-9);
    PASS();

    TEST("ratio_compute_basic unity");
    ASSERT_EQ(ratio_compute_basic(50.0, 50.0), 1.0, 1e-9);
    PASS();

    TEST("ratio_compute_basic zero master → returns 0");
    ASSERT_EQ(ratio_compute_basic(100.0, 0.0), 0.0, 1e-9);
    PASS();

    TEST("ratio_compute_basic negative master → returns 0");
    ASSERT_EQ(ratio_compute_basic(100.0, -1.0), 0.0, 1e-9);
    PASS();
}

/* ================================================================
 * L1: Ratio setpoint validation
 * ================================================================ */
static void test_ratio_setpoint_validate(void)
{
    TEST("valid setpoint");
    ASSERT_TRUE(ratio_setpoint_validate(2.0, 0.5, 5.0));
    PASS();

    TEST("zero setpoint → invalid");
    ASSERT_TRUE(!ratio_setpoint_validate(0.0, 0.5, 5.0));
    PASS();

    TEST("below min → invalid");
    ASSERT_TRUE(!ratio_setpoint_validate(0.1, 0.5, 5.0));
    PASS();

    TEST("above max → invalid");
    ASSERT_TRUE(!ratio_setpoint_validate(10.0, 0.5, 5.0));
    PASS();

    TEST("bad limits (max <= min) → invalid");
    ASSERT_TRUE(!ratio_setpoint_validate(3.0, 5.0, 3.0));
    PASS();
}

/* ================================================================
 * L1: Ratio clamping
 * ================================================================ */
static void test_ratio_clamp(void)
{
    TEST("clamp within range");
    ASSERT_EQ(ratio_clamp(3.0, 1.0, 5.0), 3.0, 1e-9);
    PASS();

    TEST("clamp below");
    ASSERT_EQ(ratio_clamp(0.5, 1.0, 5.0), 1.0, 1e-9);
    PASS();

    TEST("clamp above");
    ASSERT_EQ(ratio_clamp(10.0, 1.0, 5.0), 5.0, 1e-9);
    PASS();
}

/* ================================================================
 * L2: Ratio station
 * ================================================================ */
static void test_ratio_station(void)
{
    TEST("ratio station normal");
    ASSERT_EQ(ratio_station(2.0, 50.0), 100.0, 1e-9);
    PASS();

    TEST("ratio station zero master");
    ASSERT_EQ(ratio_station(2.0, 0.0), 0.0, 1e-9);
    PASS();

    TEST("inverse ratio computation");
    ASSERT_EQ(ratio_inverse(100.0, 50.0), 2.0, 1e-9);
    PASS();

    TEST("master required computation");
    ASSERT_EQ(ratio_master_required(100.0, 2.0), 50.0, 1e-9);
    PASS();
}

/* ================================================================
 * L2: Ratio error
 * ================================================================ */
static void test_ratio_error(void)
{
    TEST("ratio error positive");
    ASSERT_EQ(ratio_error_compute(2.5, 2.0), 0.5, 1e-9);
    PASS();

    TEST("ratio error percent");
    ASSERT_EQ(ratio_error_percent(2.1, 2.0), 5.0, 1e-9);
    PASS();

    TEST("ratio error percent zero SP");
    ASSERT_EQ(ratio_error_percent(1.0, 0.0), 0.0, 1e-9);
    PASS();
}

/* ================================================================
 * L3: Flow unit conversion
 * ================================================================ */
static void test_flow_conversion(void)
{
    TEST("volume to mass");
    ASSERT_EQ(flow_volume_to_mass(10.0, 1000.0), 10000.0, 1e-9);
    PASS();

    TEST("mass to volume");
    ASSERT_EQ(flow_mass_to_volume(10000.0, 1000.0), 10.0, 1e-9);
    PASS();

    TEST("mass to volume zero density → 0");
    ASSERT_EQ(flow_mass_to_volume(100.0, 0.0), 0.0, 1e-9);
    PASS();

    TEST("mass to molar");
    ASSERT_EQ(flow_mass_to_molar(44.0, 0.044), 1000.0, 1e-9);
    PASS();

    TEST("molar to mass");
    ASSERT_EQ(flow_molar_to_mass(1000.0, 0.044), 44.0, 1e-9);
    PASS();

    TEST("flow unit convert: mass→mass");
    ASSERT_EQ(flow_unit_convert(100.0, 0, 0, 1.0, 1.0, 1000.0), 100.0, 1e-9);
    PASS();

    TEST("flow unit convert: mass→normalized");
    ASSERT_EQ(flow_unit_convert(50.0, 0, 3, 1.0, 1.0, 100.0), 0.5, 1e-9);
    PASS();
}

/* ================================================================
 * L3: Flow filtering
 * ================================================================ */
static void test_flow_filtering(void)
{
    TEST("EWMA filter α=0.5 (Ts=1, tau=1)");
    double result = flow_ewma_filter(10.0, 0.0, 1.0, 1.0);
    /* α = 1/(1+1) = 0.5; y = 0.5*10 + 0.5*0 = 5 */
    ASSERT_EQ(result, 5.0, 1e-9);
    PASS();

    TEST("EWMA filter zero tau → no filtering");
    ASSERT_EQ(flow_ewma_filter(10.0, 5.0, 1.0, 0.0), 10.0, 1e-9);
    PASS();

    TEST("EWMA filter negative tau → pass through");
    ASSERT_EQ(flow_ewma_filter(10.0, 5.0, 1.0, -1.0), 10.0, 1e-9);
    PASS();

    TEST("moving average update");
    double sum_new;
    double avg = flow_moving_average_update(7.0, 1.0, 10.0, 4, &sum_new);
    ASSERT_EQ(avg, 4.0, 1e-9);
    ASSERT_EQ(sum_new, 16.0, 1e-9);
    PASS();
}

/* ================================================================
 * L4: Ratio safety & validation
 * ================================================================ */
static void test_ratio_validation(void)
{
    TEST("validate good measurement");
    ASSERT_TRUE(ratio_validate_measurement(50.0, 2));
    PASS();

    TEST("validate bad quality");
    ASSERT_TRUE(!ratio_validate_measurement(50.0, 0));
    PASS();

    TEST("validate negative flow");
    ASSERT_TRUE(!ratio_validate_measurement(-1.0, 2));
    PASS();

    TEST("within tolerance");
    ASSERT_TRUE(ratio_within_tolerance(2.02, 2.0, 0.05));
    ASSERT_TRUE(!ratio_within_tolerance(2.2, 2.0, 0.05));
    PASS();

    TEST("detect divergence");
    ASSERT_TRUE(ratio_detect_divergence(0.3, 0.2, 0.1));
    ASSERT_TRUE(!ratio_detect_divergence(0.1, -0.1, 0.2));
    PASS();
}

/* ================================================================
 * L5: Stoichiometric constants
 * ================================================================ */
static void test_stoichiometric(void)
{
    TEST("natural gas AFR");
    ASSERT_EQ(stoichiometric_afr_get(0), 17.2, 1e-9);
    PASS();

    TEST("hydrogen AFR");
    ASSERT_EQ(stoichiometric_afr_get(4), 34.3, 1e-9);
    PASS();

    TEST("invalid fuel code");
    ASSERT_EQ(stoichiometric_afr_get(99), 0.0, 1e-9);
    PASS();

    TEST("lambda from AFR");
    ASSERT_EQ(lambda_from_afr(20.0, 17.2), 20.0/17.2, 1e-9);
    PASS();

    TEST("excess air percent");
    ASSERT_EQ(excess_air_percent(1.2), 20.0, 1e-9);
    PASS();

    TEST("combustion efficiency loss");
    double loss = combustion_efficiency_loss(20.0, 0.03);
    ASSERT_EQ(loss, 0.6, 1e-9);
    PASS();
}

/* ================================================================
 * L6: Ratio scheduling and advanced features
 * ================================================================ */
static void test_ratio_scheduling(void)
{
    TEST("linear schedule midpoint");
    ASSERT_EQ(ratio_schedule_linear(50.0, 0.0, 100.0, 1.0, 2.0), 1.5, 1e-9);
    PASS();

    TEST("deadband apply within band");
    ASSERT_EQ(ratio_deadband_apply(0.005, 0.01), 0.0, 1e-9);
    PASS();

    TEST("deadband apply outside band");
    ASSERT_EQ(ratio_deadband_apply(0.02, 0.01), 0.02, 1e-9);
    PASS();

    TEST("hunting detection active");
    double amp = ratio_detect_hunting(0.1, -0.09, 0.11, -0.1);
    ASSERT_TRUE(amp > 0.0);
    PASS();

    TEST("hunting detection inactive");
    ASSERT_EQ(ratio_detect_hunting(0.1, 0.09, 0.11, 0.1), 0.0, 1e-9);
    PASS();

    TEST("cascade ratio adjust midpoint → no trim");
    ASSERT_EQ(cascade_ratio_adjust(50.0, 0.0, 100.0, 0.3), 0.0, 1e-9);
    PASS();

    TEST("cascade anti-windup: saturated high");
    ASSERT_TRUE(cascade_anti_windup_check(100.0, 100.0, 0.0));
    PASS();

    TEST("cascade anti-windup: in range");
    ASSERT_TRUE(!cascade_anti_windup_check(50.0, 100.0, 0.0));
    PASS();
}

int main(void)
{
    printf("=== Ratio Core Tests ===\n\n");

    test_ratio_compute_basic();
    test_ratio_setpoint_validate();
    test_ratio_clamp();
    test_ratio_station();
    test_ratio_error();
    test_flow_conversion();
    test_flow_filtering();
    test_ratio_validation();
    test_stoichiometric();
    test_ratio_scheduling();

    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);

    if (tests_passed < tests_run) {
        printf("SOME TESTS FAILED!\n");
        return 1;
    }
    printf("ALL TESTS PASSED!\n");
    return 0;
}
