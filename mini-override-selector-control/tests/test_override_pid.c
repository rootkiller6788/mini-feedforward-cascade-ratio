/**
 * @file test_override_pid.c
 * @brief Tests for Override PID Controller Module
 */
#include "override_pid.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>
static int tests_run = 0, tests_passed = 0;
#define TEST(n) do { tests_run++; printf("  %s ... ", n); } while(0)
#define PASS()  do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(m) do { printf("FAIL: %s\n", m); } while(0)

static void test_pid_init(void) {
    TEST("PID initialization");
    override_controller_t ctrl;
    override_pid_init(&ctrl, 2.0, 10.0, 0.5, 8.0, 0.1, 0.0, 100.0, 1.0);
    assert(ctrl.params.Kc == 2.0);
    assert(ctrl.params.Ti == 10.0);
    assert(ctrl.params.Td == 0.5);
    assert(ctrl.enabled == 1);
    assert(ctrl.output == 0.0);
    PASS();
}

static void test_pid_update_basic(void) {
    TEST("PID update - basic P action");
    override_controller_t ctrl;
    /* Pure P controller: Kc=1.0, Ti=HUGE (no I), Td=0 */
    override_pid_init(&ctrl, 1.0, 1e9, 0.0, 10.0, 0.1, 0.0, 100.0, 1.0);
    ctrl.active = 1;
    /* Error = 10, P output should be Kc * error = 10 */
    double out = override_pid_update(&ctrl, 50.0, 40.0, 0.1);
    assert(fabs(out - 10.0) < 1e-6);
    PASS();
}

static void test_pid_update_integral(void) {
    TEST("PID update - PI action");
    override_controller_t ctrl;
    override_pid_init(&ctrl, 2.0, 5.0, 0.0, 10.0, 0.1, 0.0, 100.0, 1.0);
    ctrl.active = 1;
    /* First step: error=5, dt=0.1, Ti=5 → I incr = 2.0*0.1/5*5=0.2 */
    double out1 = override_pid_update(&ctrl, 50.0, 45.0, 0.1);
    assert(fabs(out1 - (2.0*5.0 + 0.2)) < 0.1); /* P + I */
    /* Second step: */
    double out2 = override_pid_update(&ctrl, 50.0, 45.0, 0.1);
    assert(out2 > out1); /* Integral should accumulate */
    PASS();
}

static void test_pid_tracking(void) {
    TEST("PID tracking mode");
    override_controller_t ctrl;
    override_pid_init(&ctrl, 1.0, 10.0, 0.0, 10.0, 0.1, 0.0, 100.0, 1.0);
    ctrl.active = 0; /* inactive → tracking */
    ctrl.tracking_value = 50.0;
    ctrl.setpoint = 100.0;
    ctrl.pv = 90.0;
    double out = override_pid_update(&ctrl, 100.0, 90.0, 0.1);
    assert(fabs(out - 50.0) < 1e-6);
    PASS();
}

static void test_pid_output_clamping(void) {
    TEST("PID output clamping");
    override_controller_t ctrl;
    override_pid_init(&ctrl, 10.0, 1e9, 0.0, 10.0, 0.1, 10.0, 20.0, 1.0);
    ctrl.active = 1;
    /* Error=100 → raw output=1000, clamp to 20 */
    double out = override_pid_update(&ctrl, 100.0, 0.0, 0.1);
    assert(out <= 20.0);
    assert(out >= 10.0);
    PASS();
}

static void test_pid_bumpless_init(void) {
    TEST("PID bumpless initialization");
    override_controller_t ctrl;
    override_pid_init(&ctrl, 2.0, 10.0, 0.5, 8.0, 0.1, 0.0, 100.0, 1.0);
    override_pid_bumpless_init(&ctrl, 40.0, 60.0, 55.0);
    assert(fabs(ctrl.output - 40.0) < 1e-6);
    assert(fabs(ctrl.setpoint - 60.0) < 1e-6);
    PASS();
}

static void test_pid_form_conversion(void) {
    TEST("PID ISA to parallel conversion");
    override_pid_params_t isa;
    override_pid_params_init(&isa);
    isa.Kc = 2.0;
    isa.Ti = 10.0;
    isa.Td = 0.5;
    double Kp, Ki, Kd;
    override_pid_isa_to_parallel(&isa, &Kp, &Ki, &Kd);
    assert(fabs(Kp - 2.0) < 1e-9);
    assert(fabs(Ki - 0.2) < 1e-9); /* Kc/Ti = 2.0/10.0 = 0.2 */
    assert(fabs(Kd - 1.0) < 1e-9); /* Kc*Td = 2.0*0.5 = 1.0 */
    PASS();
}

static void test_pid_is_saturated(void) {
    TEST("PID saturation detection");
    override_controller_t ctrl;
    override_pid_init(&ctrl, 1.0, 1e9, 0.0, 10.0, 0.1, 0.0, 100.0, 1.0);
    assert(override_pid_is_saturated(&ctrl) == 1); /* output=0 at u_min=0 */
    ctrl.output = 50.0;
    assert(override_pid_is_saturated(&ctrl) == 0);
    ctrl.output = 100.0;
    assert(override_pid_is_saturated(&ctrl) == 1); /* at u_max */
    PASS();
}

int main(void) {
    printf("=== Override PID Tests ===\n\n");
    test_pid_init();
    test_pid_update_basic();
    test_pid_update_integral();
    test_pid_tracking();
    test_pid_output_clamping();
    test_pid_bumpless_init();
    test_pid_form_conversion();
    test_pid_is_saturated();
    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
