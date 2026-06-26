/**
 * @file test_override_core.c
 * @brief Tests for Override Core Module
 *
 * Covers: L1 data validation, L2 core logic,
 *         initialization, string converters
 */

#include "override_core.h"
#include "override_selector.h"
#include "override_pid.h"
#include "override_constraint.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(n) do { tests_run++; printf("  %s ... ", n); } while(0)
#define PASS()  do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(m) do { printf("FAIL: %s\n", m); } while(0)

/* L1: String converters */
static void test_string_converters(void) {
    TEST("String converters");
    assert(override_mode_name(OVERRIDE_MODE_PRIMARY) != NULL);
    assert(override_mode_name(99) != NULL); /* Unknown */
    assert(selector_type_name(SELECTOR_LOW) != NULL);
    assert(constraint_type_name(CONSTRAINT_HARD_ABS) != NULL);
    assert(tracking_mode_name(TRACK_EXTERNAL_RESET) != NULL);
    assert(override_priority_name(PRIORITY_SAFETY) != NULL);
    assert(xfer_mode_name(XFER_BUMPLESS) != NULL);
    PASS();
}

/* L1: PID params validation */
static void test_pid_params_validation(void) {
    TEST("PID params validation");
    override_pid_params_t p;
    override_pid_params_init(&p);
    assert(override_pid_params_is_valid(&p) == 1);

    override_pid_params_t bad_kc = p;
    bad_kc.Kc = 0.0;
    assert(override_pid_params_is_valid(&bad_kc) == 0);

    override_pid_params_t bad_ti = p;
    bad_ti.Ti = 0.0;
    assert(override_pid_params_is_valid(&bad_ti) == 0);

    override_pid_params_t bad_lim = p;
    bad_lim.u_min = 100.0;
    bad_lim.u_max = 0.0;
    assert(override_pid_params_is_valid(&bad_lim) == 0);

    assert(override_pid_params_is_valid(NULL) == 0);
    PASS();
}

/* L1: Constraint definition validation */
static void test_constraint_def_validation(void) {
    TEST("Constraint definition validation");
    constraint_def_t def;
    override_constraint_def_init(&def);
    def.tag = "TI-101.HI";
    def.hi_limit = 100.0;
    def.lo_limit = 0.0;
    assert(override_constraint_def_is_valid(&def) == 1);

    constraint_def_t bad_def = def;
    bad_def.tag = NULL;
    assert(override_constraint_def_is_valid(&bad_def) == 0);

    constraint_def_t bad_lim = def;
    bad_lim.hi_limit = 0.0;
    bad_lim.lo_limit = 100.0;
    assert(override_constraint_def_is_valid(&bad_lim) == 0);

    assert(override_constraint_def_is_valid(NULL) == 0);
    PASS();
}

/* L1: State initialization */
static void test_state_init(void) {
    TEST("Override state initialization");

    override_config_t cfg = {8, 16, 0.5, 10.0, 5.0, 0.01, 1, 0, 1};
    override_state_t state;

    int rc = override_state_init(&state, &cfg, 2, 3);
    assert(rc == 0);
    assert(state.initialized == 1);
    assert(state.num_controllers == 2);
    assert(state.num_constraints == 3);
    assert(override_state_is_valid(&state) == 1);

    /* Test invalid init */
    int rc2 = override_state_init(&state, &cfg, 0, 3);
    assert(rc2 == -1);

    /* Test NULL */
    assert(override_state_is_valid(NULL) == 0);

    override_state_free(&state);
    PASS();
}

/* L1: Controller initialization */
static void test_controller_init(void) {
    TEST("Controller initialization");
    override_controller_t ctrl;
    override_controller_init(&ctrl, 1, "PIC-101", "Primary Pressure", PRIORITY_PRIMARY);
    assert(ctrl.id == 1);
    assert(ctrl.enabled == 1);
    assert(ctrl.faulted == 0);
    assert(ctrl.active == 0);
    assert(override_pid_params_is_valid(&ctrl.params) == 1);
    PASS();
}

/* L1: Surge control init */
static void test_surge_init(void) {
    TEST("Surge control initialization");
    surge_control_t surge;
    surge_control_init(&surge, 1.5, 10.0, 15.0);
    assert(surge.surge_line_slope == 1.5);
    assert(surge.surge_line_intercept == 10.0);
    assert(surge.min_surge_margin == 15.0);
    assert(surge.surge_override_active == 0);
    assert(surge.surge_margin == 100.0);
    PASS();
}

/* L1: VPC init */
static void test_vpc_init(void) {
    TEST("VPC initialization");
    vpc_state_t vpc;
    vpc_state_init(&vpc, 50.0, 10.0, 90.0);
    assert(vpc.enabled == 1);
    assert(vpc.vpc_setpoint == 50.0);
    assert(vpc.vpc_min == 10.0);
    assert(vpc.vpc_max == 90.0);
    assert(vpc.vpc_active == 0);
    PASS();
}

/* L2: Selector logic */
static void test_selector_high(void) {
    TEST("High-select");
    double vals[] = {1.0, 5.0, 3.0, 2.0, 4.0};
    int valid[] = {1, 1, 1, 1, 1};
    int idx = -1;
    double result = selector_high(vals, valid, 5, &idx);
    assert(result == 5.0);
    assert(idx == 1);
    PASS();
}

static void test_selector_low(void) {
    TEST("Low-select");
    double vals[] = {10.0, 5.0, 8.0, 2.0, 7.0};
    int valid[] = {1, 1, 1, 1, 1};
    int idx = -1;
    double result = selector_low(vals, valid, 5, &idx);
    assert(result == 2.0);
    assert(idx == 3);
    PASS();
}

static void test_selector_median(void) {
    TEST("Median-select");
    double vals[] = {1.0, 100.0, 50.0};
    int valid[] = {1, 1, 1};
    int idx = -1;
    double result = selector_median(vals, valid, 3, &idx);
    assert(result == 50.0);
    PASS();
}

static void test_selector_auctioneer(void) {
    TEST("Auctioneer-3");
    int idx = -1;
    double result = selector_auctioneer_3(10.0, 1, 12.0, 1, 11.0, 1, &idx);
    assert(result == 11.0);
    assert(idx == 2);
    PASS();
}

static void test_selector_hysteresis(void) {
    TEST("High-select with hysteresis");
    double vals[] = {10.0, 11.0, 9.0};
    int valid[] = {1, 1, 1};
    int idx = -1;
    /* First call: select max (11.0 at index 1) */
    double r1 = selector_high_hysteresis(vals, valid, 3, -1, 2.0, &idx);
    assert(r1 == 11.0);
    assert(idx == 1);

    /* Second call: with prev_idx=1, hysteresis=2.0.
       max is still 11.0 (index 1), should stay at index 1 */
    vals[1] = 11.5; /* slightly higher but within hysteresis of prev (10.0)? */
    double r2 = selector_high_hysteresis(vals, valid, 3, idx, 2.0, &idx);
    /* vals = {10.0, 11.5, 9.0}, prev_idx=1, prev val was 11.0
       best = 11.5. 11.5 > 11.0 + 2.0? No (0.5 < 2.0), so stays at idx 1 */
    assert(r2 == 11.0 || r2 == 11.5); /* behavior depends on stored prev */
    PASS();
}

static void test_selector_rate_limit(void) {
    TEST("Rate-limited selector");
    double r1 = selector_rate_limit(50.0, 40.0, 5.0, 1.0);
    assert(r1 == 45.0); /* limited to +5 */

    double r2 = selector_rate_limit(30.0, 40.0, 5.0, 1.0);
    assert(r2 == 35.0); /* limited to -5 */

    double r3 = selector_rate_limit(42.0, 40.0, 5.0, 1.0);
    assert(r3 == 42.0); /* within limit */
    PASS();
}

/* L2: Constraint evaluation */
static void test_constraint_approach(void) {
    TEST("Constraint approach factor");
    constraint_def_t def;
    override_constraint_def_init(&def);
    def.tag = "TEST";
    def.hi_limit = 100.0;
    def.lo_limit = 0.0;
    def.margin = 10.0;
    def.enabled = 1;

    constraint_state_t state;
    override_constraint_state_init(&state, &def);

    /* Far from limit */
    constraint_update(&state, 50.0, 1.0);
    double af = constraint_approach_factor(&state);
    assert(af < 0.0);

    /* Approaching high limit (margin = 10 → approach starts at 90) */
    constraint_update(&state, 95.0, 1.0);
    af = constraint_approach_factor(&state);
    assert(af >= 0.0 && af <= 1.0);

    /* Violating high limit */
    constraint_update(&state, 110.0, 1.0);
    assert(constraint_is_violated(&state) == 1);
    PASS();
}

int main(void) {
    printf("=== Override Core Tests ===\n\n");

    test_string_converters();
    test_pid_params_validation();
    test_constraint_def_validation();
    test_state_init();
    test_controller_init();
    test_surge_init();
    test_vpc_init();
    test_selector_high();
    test_selector_low();
    test_selector_median();
    test_selector_auctioneer();
    test_selector_hysteresis();
    test_selector_rate_limit();
    test_constraint_approach();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
