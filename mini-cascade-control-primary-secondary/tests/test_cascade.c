/**
 * @file test_cascade.c
 * @brief Comprehensive tests for cascade control module
 *
 * Tests cover:
 *   - PID initialization, parameter validation
 *   - Positional and velocity PID algorithms
 *   - Bumpless transfer correctness
 *   - Anti-windup strategies
 *   - Cascade pair management
 *   - RGA/decoupling calculations
 *   - Gain scheduling
 *   - Split-range outputs
 *   - Open-loop detection
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#include "cascade_types.h"
#include "cascade_pid.h"
#include "cascade_control.h"
#include "cascade_tuning.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(desc, expr) do { \
    if (expr) { tests_passed++; } \
    else { tests_failed++; printf("FAIL: %s\n", desc); } \
} while(0)

#define TEST_FLOAT_EQ(desc, a, b, tol) do { \
    if (fabs((a) - (b)) < (tol)) { tests_passed++; } \
    else { tests_failed++; printf("FAIL: %s (%.6f != %.6f)\n", desc, a, b); } \
} while(0)

/*-------------------------------------------------------------------------*/

static void test_pid_init(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 2.0, 10.0, 1.0, 0.1, 0.0, 100.0);

    TEST("PID Kp initialized", fabs(pid.params.kp - 2.0) < 1e-9);
    TEST("PID Ti initialized", fabs(pid.params.ti - 10.0) < 1e-9);
    TEST("PID Td initialized", fabs(pid.params.td - 1.0) < 1e-9);
    TEST("PID sample time", fabs(pid.sample_time - 0.1) < 1e-9);
    TEST("PID output min", fabs(pid.params.output_min - 0.0) < 1e-9);
    TEST("PID output max", fabs(pid.params.output_max - 100.0) < 1e-9);
    TEST("PID integral zeroed", fabs(pid.state.integral) < 1e-9);
    TEST("PID integrator active", pid.state.integrator_active == true);
    TEST("PID sample count zero", pid.state.sample_count == 0);

    /* Test NULL safety */
    cascade_pid_init(NULL, 1.0, 1.0, 0.0, 0.1, 0.0, 100.0);
    TEST("PID init NULL safe", 1);
}

static void test_pid_reset(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 1.0, 10.0, 0.0, 0.1, 0.0, 100.0);

    /* Run a few updates to build up state */
    cascade_pid_update_positional(&pid, 50.0, 40.0);
    cascade_pid_update_positional(&pid, 50.0, 42.0);

    TEST("PID has non-zero state after updates",
         fabs(pid.state.last_error) > 0.0);

    cascade_pid_reset(&pid);
    TEST("PID integral zero after reset", fabs(pid.state.integral) < 1e-9);
    TEST("PID last error zero after reset", fabs(pid.state.last_error) < 1e-9);

    cascade_pid_reset(NULL);
    TEST("PID reset NULL safe", 1);
}

static void test_pid_set_params(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 1.0, 10.0, 0.0, 0.1, 0.0, 100.0);

    cascade_pid_set_params(&pid, 3.0, 20.0, 2.0);
    TEST("PID Kp updated", fabs(pid.params.kp - 3.0) < 1e-9);
    TEST("PID Ti updated", fabs(pid.params.ti - 20.0) < 1e-9);
    TEST("PID Td updated", fabs(pid.params.td - 2.0) < 1e-9);

    /* Invalid params should be rejected */
    cascade_pid_set_params(&pid, -1.0, 10.0, 0.0);
    TEST("PID rejects negative Kp", pid.params.kp > 0.0);

    cascade_pid_set_params(NULL, 1.0, 1.0, 0.0);
    TEST("PID set_params NULL safe", 1);
}

static void test_pid_positional(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 1.0, 10.0, 0.0, 0.1, 0.0, 100.0);

    /* P-only test: error = 10, Kp = 1, output should be ~10 */
    double u = cascade_pid_update_positional(&pid, 50.0, 40.0);
    TEST("PID P-action sign correct", u > 0.0);

    /* Repeated calls should accumulate integral */
    u = cascade_pid_update_positional(&pid, 50.0, 40.0);
    u = cascade_pid_update_positional(&pid, 50.0, 40.0);
    TEST("PID I-action grows output", u > 1.0);

    /* Output clamping */
    cascade_pid_controller_t pid2;
    cascade_pid_init(&pid2, 100.0, 0.1, 0.0, 0.1, 0.0, 10.0);
    u = cascade_pid_update_positional(&pid2, 100.0, 0.0);
    TEST("PID output clamped to max", u <= 10.0 + 1e-6);

    /* NULL safety */
    u = cascade_pid_update_positional(NULL, 50.0, 40.0);
    TEST("PID positional NULL safe", fabs(u) < 1e-9);
}

static void test_pid_velocity(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 1.0, 10.0, 0.0, 0.1, 0.0, 100.0);

    /* First update should give incremental output */
    double u = cascade_pid_update_velocity(&pid, 50.0, 40.0);
    TEST("PID velocity first update > 0", u > 0.0);

    /* Same error → no proportional increment, only integral */
    double u2 = cascade_pid_update_velocity(&pid, 50.0, 40.0);
    TEST("PID velocity accumulates", u2 > 0.0);

    /* Velocity form: PV change produces output change */
    double u3 = cascade_pid_update_velocity(&pid, 50.0, 50.0);
    TEST("PID velocity PV change detected", fabs(u3 - u2) > 0.0);

    cascade_pid_update_velocity(NULL, 50.0, 40.0);
    TEST("PID velocity NULL safe", 1);
}

static void test_bumpless_transfer(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 1.0, 10.0, 0.0, 0.1, 0.0, 100.0);

    /* Simulate manual operation at 75% */
    double manual_out = 75.0;
    cascade_pid_bumpless_manual_to_auto(&pid, manual_out, 50.0, 50.0);

    /* After bumpless init, the first auto output should be close to manual */
    double u = cascade_pid_update_positional(&pid, 50.0, 50.0);
    TEST_FLOAT_EQ("Bumpless manual→auto output near manual",
                  u, manual_out, 2.0);

    /* Auto→manual should return current output */
    double u_manual = cascade_pid_bumpless_auto_to_manual(&pid);
    TEST("Bumpless auto→manual returns output", u_manual > 0.0);

    cascade_pid_bumpless_manual_to_auto(NULL, 0, 0, 0);
    cascade_pid_bumpless_auto_to_manual(NULL);
    TEST("Bumpless NULL safe", 1);
}

static void test_anti_windup(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 1.0, 10.0, 0.0, 0.1, 5.0, 10.0);

    /* Clamping: drive output to max with positive error */
    int count = 0;
    for (int i = 0; i < 50; i++) {
        cascade_pid_update_positional(&pid, 100.0, 0.0);
        if (!pid.state.integrator_active) count++;
    }
    TEST("Clamping anti-windup activates", count > 0);

    /* Back-calculation */
    cascade_pid_controller_t pid2;
    cascade_pid_init(&pid2, 1.0, 10.0, 0.0, 0.1, 0.0, 100.0);
    pid2.aw_strategy = CASCADE_AW_BACK_CALCULATION;

    double u_unsat = 120.0;
    double u_sat = 100.0;
    double integral_before = pid2.state.integral;
    cascade_pid_anti_windup_back_calc(&pid2, u_unsat, u_sat, 10.0);
    TEST("Back-calc reduces integral when oversaturated",
         pid2.state.integral < integral_before);

    /* Conditional integration */
    cascade_pid_anti_windup_conditional(&pid, 5.0, 1.0, false);
    TEST("Conditional AW stops for large error",
         !pid.state.integrator_active);
    cascade_pid_anti_windup_conditional(&pid, 0.5, 1.0, false);
    /* (small error, not saturated → should integrate) */

    cascade_pid_anti_windup_clamping(NULL, 0);
    cascade_pid_anti_windup_back_calc(NULL, 0, 0, 0);
    cascade_pid_anti_windup_conditional(NULL, 0, 0, false);
    TEST("Anti-windup NULL safe", 1);
}

static void test_setpoint_handling(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 1.0, 10.0, 1.0, 0.1, 0.0, 100.0);

    /* Setpoint filter */
    double sp = cascade_pid_setpoint_filter(&pid, 50.0, 1.0);
    TEST("SP filter returns value", sp > 0.0);

    /* Setpoint ramp */
    double sp_ramped = cascade_pid_setpoint_ramp(0.0, 100.0, 10.0, 1.0);
    TEST_FLOAT_EQ("SP ramp limited", sp_ramped, 10.0, 1.0);

    sp_ramped = cascade_pid_setpoint_ramp(90.0, 100.0, 10.0, 1.0);
    TEST_FLOAT_EQ("SP ramp at target", sp_ramped, 100.0, 1.0);

    cascade_pid_setpoint_filter(NULL, 0, 0);
    cascade_pid_setpoint_ramp(0, 0, 0, 0);
    TEST("SP NULL safe", 1);
}

static void test_2dof_pid(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 1.0, 10.0, 1.0, 0.1, 0.0, 100.0);

    /* 2-DOF setpoint weighting: beta controls P-term SP sensitivity.
     * With beta=0: P = Kp*(0*SP - PV), SP change does not affect P-term.
     * With beta=1: P = Kp*(SP - PV), SP change directly affects P-term.
     * Testing: at steady state (SP=PV=50), output should be zero/near-zero.
     * Upon SP step to 100 with PV still at 50, beta=1 produces immediate P-term response. */
    cascade_pid_controller_t pid0;
    cascade_pid_init(&pid0, 2.0, 100.0, 0.0, 0.1, 0.0, 100.0);
    pid0.state.filtered_pv = 50.0;
    pid0.state.last_pv = 50.0;

    /* At equilibrium (SP=PV=50), output should be near zero for both */
    double u_eq_b0 = cascade_pid_setpoint_weighting(&pid0, 50.0, 50.0, 0.0, 0.0);
    TEST_FLOAT_EQ("2-DOF beta=0 equilibrium near 0", u_eq_b0, 0.0, 1.0);

    cascade_pid_controller_t pid1;
    cascade_pid_init(&pid1, 2.0, 100.0, 0.0, 0.1, 0.0, 100.0);
    pid1.state.filtered_pv = 50.0;
    pid1.state.last_pv = 50.0;
    double u_eq_b1 = cascade_pid_setpoint_weighting(&pid1, 50.0, 50.0, 1.0, 0.0);
    TEST_FLOAT_EQ("2-DOF beta=1 equilibrium near 0", u_eq_b1, 0.0, 1.0);

    /* SP step: PV stays at 50, SP jumps to 100.
     * beta=1: P = Kp*(100-50) = 100 → large immediate response
     * beta=0: P = Kp*(0-50) = -100, I grows slowly → gradual response */
    double u_step_b0 = cascade_pid_setpoint_weighting(&pid0, 100.0, 50.0, 0.0, 0.0);
    double u_step_b1 = cascade_pid_setpoint_weighting(&pid1, 100.0, 50.0, 1.0, 0.0);
    /* beta=1 should have larger output (P-term responds to SP change) */
    TEST("2-DOF beta=1 gives larger SP response", u_step_b1 > u_step_b0);

    cascade_pid_setpoint_weighting(NULL, 0, 0, 0, 0);
    TEST("2-DOF NULL safe", 1);
}

static void test_ideal_series_pid(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 1.0, 10.0, 0.0, 0.1, 0.0, 100.0);

    double u_ideal = cascade_pid_compute_ideal(&pid, 50.0, 45.0);
    TEST("Ideal PID returns value", u_ideal > 0.0);

    double u_series = cascade_pid_compute_series(&pid, 50.0, 45.0);
    TEST("Series PID returns value", u_series > 0.0);

    cascade_pid_compute_ideal(NULL, 0, 0);
    cascade_pid_compute_series(NULL, 0, 0);
    TEST("Ideal/series NULL safe", 1);
}

static void test_output_tracking(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 1.0, 10.0, 0.0, 0.1, 0.0, 100.0);

    cascade_pid_update_positional(&pid, 50.0, 40.0);
    /* force tracking to a new value */
    cascade_pid_output_tracking(&pid, 25.0);
    TEST_FLOAT_EQ("Output tracking sets output", pid.state.prev_output, 25.0, 1e-6);

    cascade_pid_output_tracking(NULL, 0);
    TEST("Output tracking NULL safe", 1);
}

/*-------------------------------------------------------------------------*/

static void test_rga_decoupling(void)
{
    /* Test case from Seborg et al., Example 18.1: Wood-Berry column */
    double K11 = 12.8, K12 = -18.9, K21 = 6.6, K22 = -19.4;
    double lambda11, NI;
    int pairing_safe;

    rga_compute_2x2(K11, K12, K21, K22, &lambda11, &NI, &pairing_safe);

    /* Expected: λ11 ≈ 2.0 (Wood-Berry is highly interactive) */
    TEST("RGA λ11 > 0 for stability", lambda11 > 0.0);
    TEST("Niederlinski index > 0", NI > 0.0);

    double d12, d21;
    decoupler_design_2x2(K11, K12, K21, K22, &d12, &d21);
    TEST("Decoupler d12 computed", fabs(d12) > 0.0);

    /* Test decoupler application */
    double u1_out, u2_out;
    decoupler_apply_2x2(10.0, 5.0, d12, d21, &u1_out, &u2_out);
    TEST("Decoupler apply produces output", fabs(u1_out) > 0.0);

    /* Zero-gain safety */
    rga_compute_2x2(0.0, 1.0, 1.0, 1.0, &lambda11, &NI, &pairing_safe);
    TEST("RGA handles zero gain", 1);
}

static void test_split_range(void)
{
    double out_a, out_b;

    /* Sequential: 0-50% → A, 50-100% → B */
    split_range_compute(25.0, 0, 50.0, &out_a, &out_b);
    TEST("Split seq: A at 25%", out_a > 0.0);
    TEST("Split seq: B at 25%", fabs(out_b) < 1e-9);

    split_range_compute(75.0, 0, 50.0, &out_a, &out_b);
    TEST("Split seq: A at 75%", fabs(out_a - 100.0) < 1e-6);
    TEST("Split seq: B at 75%", out_b > 0.0);

    /* Complementary: sum = 100% */
    split_range_compute(30.0, 2, 50.0, &out_a, &out_b);
    TEST_FLOAT_EQ("Split complement sum = 100", out_a + out_b, 100.0, 1e-6);

    /* NULL safety */
    split_range_compute(50.0, 0, 50.0, NULL, &out_b);
    split_range_compute(50.0, 0, 50.0, &out_a, NULL);
    TEST("Split range NULL safe", 1);
}

static void test_gain_scheduling(void)
{
    cascade_adaptive_state_t gs;
    gain_schedule_init(&gs);

    /* Add schedule points */
    gain_schedule_add_point(&gs, 0.0, 1.0, 60.0, 0.0);
    gain_schedule_add_point(&gs, 100.0, 3.0, 30.0, 5.0);

    TEST("Gain schedule has 2 points", gs.num_schedule_points == 2);
    TEST("Gain schedule enabled", gs.is_gain_scheduled);

    cascade_pid_params_t params;
    gain_schedule_update(&gs, 50.0, &params);

    /* At midpoint, gain should be between 1.0 and 3.0 */
    TEST("Scheduled gain interpolated", params.kp > 1.0 && params.kp < 3.0);

    gain_schedule_init(NULL);
    gain_schedule_add_point(NULL, 0, 0, 0, 0);
    gain_schedule_update(NULL, 0, NULL);
    TEST("Gain schedule NULL safe", 1);
}

static void test_open_loop_detection(void)
{
    cascade_pid_controller_t pid;
    cascade_pid_init(&pid, 1.0, 10.0, 0.0, 0.1, 0.0, 100.0);

    /* Normal operation: PV changes with MV */
    int ol = cascade_detect_open_loop(&pid, 50.0, 50.0, 50.0, 1.0, 1.0);
    TEST("Open-loop detect normal ≈ 0", ol == 0);

    cascade_detect_open_loop(NULL, 0, 0, 0, 0, 0);
    TEST("Open-loop detect NULL safe", 1);
}

static void test_alarms(void)
{
    int alarm;

    alarm = cascade_check_alarms(50.0, 10.0, 20.0, 80.0, 90.0);
    TEST("Alarm normal", alarm == 0);

    alarm = cascade_check_alarms(5.0, 10.0, 20.0, 80.0, 90.0);
    TEST("Alarm LO_LO", alarm == 1);

    alarm = cascade_check_alarms(15.0, 10.0, 20.0, 80.0, 90.0);
    TEST("Alarm LO", alarm == 2);

    alarm = cascade_check_alarms(85.0, 10.0, 20.0, 80.0, 90.0);
    TEST("Alarm HI", alarm == 3);

    alarm = cascade_check_alarms(95.0, 10.0, 20.0, 80.0, 90.0);
    TEST("Alarm HI_HI", alarm == 4);
}

/*-------------------------------------------------------------------------*/

int main(void)
{
    printf("=== Cascade Control Tests ===\n\n");

    printf("--- PID Initialization ---\n");
    test_pid_init();
    test_pid_reset();
    test_pid_set_params();

    printf("\n--- PID Algorithms ---\n");
    test_pid_positional();
    test_pid_velocity();

    printf("\n--- Bumpless Transfer ---\n");
    test_bumpless_transfer();

    printf("\n--- Anti-Windup ---\n");
    test_anti_windup();

    printf("\n--- Setpoint Handling ---\n");
    test_setpoint_handling();

    printf("\n--- 2-DOF PID ---\n");
    test_2dof_pid();

    printf("\n--- Ideal & Series PID ---\n");
    test_ideal_series_pid();

    printf("\n--- Output Tracking ---\n");
    test_output_tracking();

    printf("\n--- RGA & Decoupling ---\n");
    test_rga_decoupling();

    printf("\n--- Split Range ---\n");
    test_split_range();

    printf("\n--- Gain Scheduling ---\n");
    test_gain_scheduling();

    printf("\n--- Open-Loop Detection ---\n");
    test_open_loop_detection();

    printf("\n--- Alarms ---\n");
    test_alarms();

    printf("\n========================================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);

    return (tests_failed > 0) ? 1 : 0;
}
