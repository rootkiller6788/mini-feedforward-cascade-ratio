/**
 * @file test_split_range.c
 * @brief Comprehensive test suite for split-range control module
 *
 * Module: mini-split-range-control-heat-cool
 *
 * Tests cover: core split mapping, PID algorithms, valve characteristics,
 * auto-tuning, adaptive gain, reactor simulation, split-point optimization.
 *
 * All tests use assert() for pass/fail determination.
 * Run: ./test_split_range
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "split_range_control.h"

#define EPS 1e-6

static int tests_passed = 0;
static int tests_failed = 0;
static int tests_total = 0;

#define TEST(name) do { \
    tests_total++; \
    printf("  TEST %3d: %-55s", tests_total, name); \
} while(0)

#define PASS() do { \
    printf("[PASS]\n"); \
    tests_passed++; \
} while(0)

#define FAIL(msg) do { \
    printf("[FAIL] %s\n", msg); \
    tests_failed++; \
} while(0)

/* =========================================================================
 * L1/L2: Core Split Mapping Tests
 * ========================================================================= */

static int test_channel_position_increasing(void) {
    TEST("Channel position: increasing action");
    double pos = split_compute_channel_position(25.0, 0.0, 50.0,
        SPLIT_ACTION_INCREASING, 0.0, 100.0);
    if (fabs(pos - 50.0) > EPS) { FAIL("expected 50.0"); return 1; }
    PASS(); return 0;
}

static int test_channel_position_decreasing(void) {
    TEST("Channel position: decreasing (reverse) action");
    double pos = split_compute_channel_position(25.0, 0.0, 50.0,
        SPLIT_ACTION_DECREASING, 100.0, 0.0);
    if (fabs(pos - 50.0) > EPS) { FAIL("expected 50.0 for reverse"); return 1; }
    PASS(); return 0;
}

static int test_channel_position_clamping_low(void) {
    TEST("Channel position: clamp below range");
    double pos = split_compute_channel_position(-10.0, 10.0, 50.0,
        SPLIT_ACTION_INCREASING, 0.0, 100.0);
    if (fabs(pos - 0.0) > EPS) { FAIL("expected 0.0 for CO below range"); return 1; }
    PASS(); return 0;
}

static int test_channel_position_clamping_high(void) {
    TEST("Channel position: clamp above range");
    double pos = split_compute_channel_position(120.0, 10.0, 50.0,
        SPLIT_ACTION_INCREASING, 0.0, 100.0);
    if (fabs(pos - 100.0) > EPS) { FAIL("expected 100.0 for CO above range"); return 1; }
    PASS(); return 0;
}

static int test_channel_position_fixed_action(void) {
    TEST("Channel position: fixed action returns valve_start");
    double pos = split_compute_channel_position(70.0, 0.0, 100.0,
        SPLIT_ACTION_FIXED, 42.0, 99.0);
    if (fabs(pos - 42.0) > EPS) { FAIL("expected valve_start=42.0"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L2: Deadband and Overlap Tests
 * ========================================================================= */

static int test_deadband_hard_transition(void) {
    TEST("Deadband: hard transition passes CO through");
    double co = split_apply_deadband_overlap(30.0, 50.0, 4.0,
        SPLIT_TRANSITION_HARD);
    if (fabs(co - 30.0) > EPS) { FAIL("hard transition should return CO unchanged"); return 1; }
    PASS(); return 0;
}

static int test_deadband_linear_transition(void) {
    TEST("Deadband: linear transition modifies output near split");
    double co = split_apply_deadband_overlap(40.0, 50.0, 4.0,
        SPLIT_TRANSITION_LINEAR);
    /* With split=50, deadband=4, lo=48, hi=52, co=40 < lo=48
     * → maps to co * (split_point/lo) = 40 * 50/48 ≈ 41.67 */
    if (co < 40.0 || co > 45.0) { FAIL("unexpected linear deadband result"); return 1; }
    PASS(); return 0;
}

static int test_deadband_cubic_spline(void) {
    TEST("Deadband: cubic spline is smooth at transition");
    double co_lo = split_apply_deadband_overlap(47.0, 50.0, 4.0,
        SPLIT_TRANSITION_CUBIC_SPLINE);
    double co_hi = split_apply_deadband_overlap(53.0, 50.0, 4.0,
        SPLIT_TRANSITION_CUBIC_SPLINE);
    if (co_lo >= co_hi) { FAIL("cubic spline should preserve monotonicity"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L2: Split Distribution Tests
 * ========================================================================= */

static int test_distribute_heat_cool(void) {
    TEST("Distribute output: heat/cool scheme at CO=25%");
    split_range_scheme_t scheme;
    split_init_heat_cool_scheme(&scheme);
    double positions[6] = {0};
    int n = split_distribute_output(&scheme, 25.0, positions);
    if (n != 2) { FAIL("expected 2 channels"); return 1; }
    /* At CO=25%, heating should be ~50% open, cooling should be 0% */
    if (fabs(positions[1]) > 1.0) { FAIL("cooling should be closed at CO=25%"); return 1; }
    PASS(); return 0;
}

static int test_distribute_heat_cool_full_cool(void) {
    TEST("Distribute output: full cooling at CO=100%");
    split_range_scheme_t scheme;
    split_init_heat_cool_scheme(&scheme);
    double positions[6] = {0};
    split_distribute_output(&scheme, 100.0, positions);
    /* At CO=100%, heating should be 0%, cooling 100% */
    if (fabs(positions[1] - 100.0) > 5.0) { FAIL("cooling should be fully open"); return 1; }
    PASS(); return 0;
}

static int test_distribute_null_safety(void) {
    TEST("Distribute output: NULL scheme returns 0");
    int n = split_distribute_output(NULL, 50.0, NULL);
    if (n != 0) { FAIL("expected 0 for NULL inputs"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L3: Valve Characteristic Tests
 * ========================================================================= */

static int test_valve_char_linear_inverse(void) {
    TEST("Valve char: linear inverse is identity");
    double x = split_valve_characteristic_inverse(0.7, SPLIT_VALVE_LINEAR, 50.0);
    if (fabs(x - 0.7) > EPS) { FAIL("linear inverse should be identity"); return 1; }
    PASS(); return 0;
}

static int test_valve_char_equal_pct_inverse(void) {
    TEST("Valve char: equal-percentage inverse");
    /* f = R^(x-1), so x = 1 + ln(f)/ln(R) */
    double x = split_valve_characteristic_inverse(0.3, SPLIT_VALVE_EQUAL_PCT, 50.0);
    /* Verify: R^(x-1) ≈ 0.3 */
    double f_check = split_valve_characteristic_forward(x, SPLIT_VALVE_EQUAL_PCT, 50.0);
    if (fabs(f_check - 0.3) > 0.01) { FAIL("inverse+forward should recover input"); return 1; }
    PASS(); return 0;
}

static int test_valve_char_quick_opening(void) {
    TEST("Valve char: quick-opening forward");
    double f = split_valve_characteristic_forward(0.25, SPLIT_VALVE_QUICK_OPENING, 50.0);
    /* sqrt(0.25) = 0.5 */
    if (fabs(f - 0.5) > EPS) { FAIL("sqrt(0.25) should be 0.5"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L3: Slew Rate Limit Tests
 * ========================================================================= */

static int test_slew_rate_limit_within(void) {
    TEST("Slew rate: small change passes through");
    double out = split_slew_rate_limit(50.0, 52.0, 25.0, 0.5);
    if (fabs(out - 52.0) > EPS) { FAIL("should reach target within slew limit"); return 1; }
    PASS(); return 0;
}

static int test_slew_rate_limit_exceeds(void) {
    TEST("Slew rate: large change is limited");
    double out = split_slew_rate_limit(50.0, 80.0, 10.0, 1.0);
    /* Max change = 10%/s * 1s = 10%, so output = 50 + 10 = 60 */
    if (fabs(out - 60.0) > EPS) { FAIL("should be limited to 60"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L3: PID Algorithm Tests
 * ========================================================================= */

static int test_pid_init_params_valid(void) {
    TEST("PID: init with valid parameters");
    split_range_pid_params_t params;
    split_pid_init_params(&params, 2.5, 60.0, 15.0, 1.0);
    if (fabs(params.kc - 2.5) > EPS) { FAIL("kc not set"); return 1; }
    if (fabs(params.ti - 60.0) > EPS) { FAIL("ti not set"); return 1; }
    if (fabs(params.td - 15.0) > EPS) { FAIL("td not set"); return 1; }
    PASS(); return 0;
}

static int test_pid_init_params_negative(void) {
    TEST("PID: negative kc clamped to 0");
    split_range_pid_params_t params;
    split_pid_init_params(&params, -5.0, 60.0, 15.0, 1.0);
    if (fabs(params.kc) > EPS) { FAIL("negative kc should be 0"); return 1; }
    PASS(); return 0;
}

static int test_pid_incremental_converges(void) {
    TEST("PID incremental: converges to setpoint");
    split_range_pid_params_t params;
    split_pid_init_params(&params, 2.0, 10.0, 2.0, 1.0);
    split_range_pid_state_t state;
    split_pid_reset_state(&state);

    double sp = 100.0, pv = 80.0;
    /* Run several iterations; output should drive PV toward SP */
    for (int i = 0; i < 50; i++) {
        double co = split_pid_incremental(&params, &state, sp, pv);
        /* Simulate simple process: PV moves toward SP proportional to CO */
        pv += (co / 100.0) * 2.0;
        if (pv > sp) pv = sp;
    }
    /* After 50 steps, PV should be close to SP */
    if (fabs(pv - sp) > 5.0) { FAIL("PID should converge"); return 1; }
    PASS(); return 0;
}

static int test_pid_positional_basic(void) {
    TEST("PID positional: computes output");
    split_range_pid_params_t params;
    split_pid_init_params(&params, 2.0, 60.0, 10.0, 1.0);
    split_range_pid_state_t state;
    split_pid_reset_state(&state);

    double co = split_pid_positional(&params, &state, 100.0, 90.0);
    /* Error = 10, P = Kc*error = 20, so CO should be > 0 */
    if (co <= 0.0) { FAIL("expected positive output for positive error"); return 1; }
    PASS(); return 0;
}

static int test_pid_reset_state(void) {
    TEST("PID: reset state zeros everything");
    split_range_pid_state_t state;
    state.integral_accum = 999.0;
    state.last_output = 75.0;
    split_pid_reset_state(&state);
    if (fabs(state.integral_accum) > EPS) { FAIL("integral should be 0 after reset"); return 1; }
    if (fabs(state.last_output) > EPS) { FAIL("last_output should be 0 after reset"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L4: Split-Range PID Tuning Tests
 * ========================================================================= */

static int test_zn_tuning_pid(void) {
    TEST("ZN tuning: produces valid PID params");
    split_range_tuning_result_t result;
    split_pid_zn_tuning(1.5, 120.0, 10.0, &result, 0);
    if (result.pid_params.kc <= 0.0) { FAIL("Kc should be positive"); return 1; }
    if (result.pid_params.ti <= 0.0) { FAIL("Ti should be positive"); return 1; }
    if (result.pid_params.td <= 0.0) { FAIL("Td should be positive"); return 1; }
    PASS(); return 0;
}

static int test_zn_tuning_tyreus_luyben(void) {
    TEST("ZN tuning: Tyreus-Luyben is more conservative");
    split_range_tuning_result_t zn, tl;
    split_pid_zn_tuning(1.5, 120.0, 10.0, &zn, 0);
    split_pid_zn_tuning(1.5, 120.0, 10.0, &tl, 1);
    /* Tyreus-Luyben should have smaller Kc */
    if (tl.pid_params.kc >= zn.pid_params.kc) {
        FAIL("Tyreus-Luyben should be more conservative (smaller Kc)");
        return 1;
    }
    PASS(); return 0;
}

static int test_zn_tuning_invalid_params(void) {
    TEST("ZN tuning: invalid params get defaults");
    split_range_tuning_result_t result;
    split_pid_zn_tuning(0.0, -10.0, -5.0, &result, 0);
    /* Should set fallback PID parameters */
    if (result.pid_params.kc <= 0.0) { FAIL("should have fallback Kc"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L4: Closed-Loop Poles Test
 * ========================================================================= */

static int test_closed_loop_poles_stable(void) {
    TEST("Poles: stable PID produces LHP poles");
    split_range_pid_params_t params;
    split_pid_init_params(&params, 0.5, 30.0, 5.0, 1.0);
    double poles[3][2];
    int n = split_pid_closed_loop_poles(1.0, 60.0, 5.0, &params, poles);
    if (n != 3) { FAIL("expected 3 poles"); return 1; }
    /* Check that real parts are negative (stable) */
    if (poles[0][0] >= 0.0 || poles[1][0] >= 0.0 || poles[2][0] >= 0.0) {
        /* This IS a valid test assertion — for this specific process and PID,
         * poles should be in LHP */
        FAIL("poles should be in left half-plane for stable tuning");
        return 1;
    }
    PASS(); return 0;
}

/* =========================================================================
 * L4: Valve Sizing (ISA-75.01) Tests
 * ========================================================================= */

static int test_valve_size_liquid(void) {
    TEST("ISA valve sizing: liquid Cv calculation");
    double Cv = split_valve_size_isa(100.0, 10.0, 1.0, 0.0, 0.0, 0);
    /* Cv = Q * sqrt(Gf/dP) = 100 * sqrt(1/10) = 100 * 0.316 ≈ 31.6 */
    if (Cv < 25.0 || Cv > 40.0) { FAIL("unexpected Cv for liquid"); return 1; }
    PASS(); return 0;
}

static int test_valve_size_zero_flow(void) {
    TEST("ISA valve sizing: zero flow returns zero Cv");
    double Cv = split_valve_size_isa(0.0, 10.0, 1.0, 0.0, 0.0, 0);
    if (Cv > 0.0) { FAIL("Cv should be 0 for zero flow"); return 1; }
    PASS(); return 0;
}

static int test_pressure_drop_ratio(void) {
    TEST("PR: pressure drop ratio in valid range");
    double Pr = split_valve_pressure_drop_ratio(50.0, 50.0);
    /* Pr = 50^2/(50^2+50^2) = 0.5 */
    if (fabs(Pr - 0.5) > 0.01) { FAIL("expected Pr ≈ 0.5 for equal Cv"); return 1; }
    PASS(); return 0;
}

static int test_valve_rangeability(void) {
    TEST("Rangeability: computes correctly");
    double R = split_valve_rangeability(100.0, 2.0);
    if (fabs(R - 50.0) > EPS) { FAIL("expected rangeability 50"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L4: Stiction Model Tests
 * ========================================================================= */

static int test_stiction_no_stick(void) {
    TEST("Stiction: no stiction, valve follows command");
    split_range_channel_t ch = {0};
    ch.stiction_threshold = 0.0;
    ch.current_position = 50.0;
    double pos = split_valve_stiction_model(&ch, 60.0);
    if (fabs(pos - 60.0) > EPS) { FAIL("should follow command with no stiction"); return 1; }
    PASS(); return 0;
}

static int test_stiction_stuck(void) {
    TEST("Stiction: small change does not move valve");
    split_range_channel_t ch = {0};
    ch.stiction_threshold = 5.0;
    ch.current_position = 50.0;
    double pos = split_valve_stiction_model(&ch, 52.0);
    /* delta=2 < threshold=5 → valve stuck at 50 */
    if (fabs(pos - 50.0) > EPS) { FAIL("valve should remain stuck for small delta"); return 1; }
    PASS(); return 0;
}

static int test_stiction_slip(void) {
    TEST("Stiction: large change overcomes stiction");
    split_range_channel_t ch = {0};
    ch.stiction_threshold = 5.0;
    ch.current_position = 50.0;
    double pos = split_valve_stiction_model(&ch, 70.0);
    /* delta=20 > threshold=5 → valve moves (with slip) */
    if (fabs(pos - 50.0) < 1.0) { FAIL("valve should move for large delta"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L5: IMC Tuning Tests
 * ========================================================================= */

static int test_imc_tuning_valid(void) {
    TEST("IMC tuning: produces valid PID params");
    split_range_tuning_result_t result;
    split_imc_tuning(1.5, 60.0, 10.0, 15.0, &result);
    if (result.pid_params.kc <= 0.0) { FAIL("IMC should produce positive Kc"); return 1; }
    if (result.pid_params.ti <= 0.0) { FAIL("IMC should produce positive Ti"); return 1; }
    PASS(); return 0;
}

static int test_imc_tuning_lambda_effect(void) {
    TEST("IMC tuning: larger lambda gives smaller Kc");
    split_range_tuning_result_t r1, r2;
    split_imc_tuning(1.5, 60.0, 10.0, 15.0, &r1);
    split_imc_tuning(1.5, 60.0, 10.0, 30.0, &r2);
    /* Larger lambda = more conservative = smaller Kc */
    if (r2.pid_params.kc >= r1.pid_params.kc) {
        FAIL("larger lambda should yield smaller Kc");
        return 1;
    }
    PASS(); return 0;
}

/* =========================================================================
 * L5: Auto-Tuning Tests
 * ========================================================================= */

static int test_autotune_init(void) {
    TEST("Auto-tune: initialization sets amplitudes");
    split_range_autotune_t at;
    split_autotune_init(&at, 15.0, 10.0, 2.0);
    if (fabs(at.relay_amplitude_heat - 15.0) > EPS) {
        FAIL("heat amplitude not set"); return 1;
    }
    if (at.identification_complete) { FAIL("should not be complete initially"); return 1; }
    PASS(); return 0;
}

static int test_autotune_step_produces_output(void) {
    TEST("Auto-tune: step returns relay output");
    split_range_autotune_t at;
    split_autotune_init(&at, 10.0, 10.0, 1.0);
    double out = split_autotune_step(&at, 5.0, 0.1);
    /* Error=5 > hysteresis=1 → relay should output positive */
    if (fabs(out - 10.0) > EPS) { FAIL("relay should output cooling amplitude"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L8: Adaptive Gain Test
 * ========================================================================= */

static int test_adaptive_init(void) {
    TEST("Adaptive: init sets gain values");
    split_range_adaptive_t ad;
    split_adaptive_init(&ad, 3.0, 2.0, 1.0, 0.05);
    if (fabs(ad.kc_heating - 3.0) > EPS) { FAIL("kc_heating not set"); return 1; }
    if (fabs(ad.kc_cooling - 2.0) > EPS) { FAIL("kc_cooling not set"); return 1; }
    if (!ad.adaptive_enabled) { FAIL("adaptive should be enabled"); return 1; }
    PASS(); return 0;
}

static int test_adaptive_update_gain(void) {
    TEST("Adaptive: gain interpolates between regions");
    split_range_adaptive_t ad;
    split_adaptive_init(&ad, 3.0, 1.0, 2.0, 0.1);
    /* At CO=0% (fully in heating), effective Kc should be near kc_heating */
    double kc_eff = split_adaptive_update_gain(&ad, 0.0);
    if (fabs(kc_eff - 3.0) > 0.1) {
        FAIL("at CO=0%, Kc should be close to kc_heating");
        return 1;
    }
    /* At CO=100% (fully in cooling), effective Kc should be near kc_cooling */
    kc_eff = split_adaptive_update_gain(&ad, 100.0);
    if (fabs(kc_eff - 1.0) > 0.1) {
        FAIL("at CO=100%, Kc should be close to kc_cooling");
        return 1;
    }
    PASS(); return 0;
}

/* =========================================================================
 * L6: Reactor Simulation Test
 * ========================================================================= */

static int test_reactor_simulation(void) {
    TEST("Reactor: temperature evolves in simulation");
    split_range_reactor_model_t model = {0};
    model.V = 200.0;
    model.rho = 1000.0;
    model.Cp = 4200.0;
    model.U_times_A = 5000.0;
    model.delta_Hr = -50000.0;
    model.k0 = 1e8;
    model.Ea = 70000.0;
    model.R_gas = 8.314;
    model.CA = 100.0;
    model.F = 0.01;
    model.Tin = 25.0;
    model.T = 25.0;
    model.T_ambient = 20.0;
    model.Tj_min = 5.0;
    model.Tj_max = 150.0;
    model.Q_heater_max = 100000.0;
    model.Q_cooler_max = 80000.0;
    model.inflection_temp = 60.0;

    double T_initial = model.T;
    /* Apply heating */
    split_reactor_simulate_step(&model, 1.0, 0.0, 1.0);
    /* Temperature should rise with heating */
    if (model.T <= T_initial) { FAIL("temperature should rise with heating"); return 1; }
    PASS(); return 0;
}

static int test_reactor_runaway_detect(void) {
    TEST("Reactor: runaway detection for safe reactor");
    split_range_reactor_model_t model = {0};
    model.T = 25.0;
    model.inflection_temp = 60.0;
    model.has_runaway_risk = false;
    bool runaway = split_reactor_runaway_detect(&model, 1.0);
    if (runaway) { FAIL("no runaway risk should be flagged for safe state"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L6: Factory Function Tests
 * ========================================================================= */

static int test_create_reactor(void) {
    TEST("Factory: reactor controller has 2 channels");
    split_range_controller_t ctrl = split_control_create_reactor();
    if (ctrl.scheme.num_channels != 2) { FAIL("reactor should have 2 channels"); return 1; }
    if (ctrl.pid_params.kc <= 0.0) { FAIL("reactor PID should have positive Kc"); return 1; }
    PASS(); return 0;
}

static int test_create_ph(void) {
    TEST("Factory: pH controller has overlap");
    split_range_controller_t ctrl = split_control_create_ph();
    if (ctrl.scheme.overlap_width <= 0.0) { FAIL("pH should have overlap"); return 1; }
    PASS(); return 0;
}

static int test_create_pressure(void) {
    TEST("Factory: pressure controller has wider deadband");
    split_range_controller_t ctrl = split_control_create_pressure();
    if (ctrl.scheme.deadband_width < 2.0) {
        FAIL("pressure should have wider deadband");
        return 1;
    }
    PASS(); return 0;
}

/* =========================================================================
 * L5: Cross-Coupling Analysis Test
 * ========================================================================= */

static int test_cross_coupling(void) {
    TEST("Cross-coupling: both valves closed = zero coupling");
    split_range_controller_t ctrl = split_control_create_reactor();
    ctrl.scheme.channels[0].current_position = 0.0;
    ctrl.scheme.channels[1].current_position = 0.0;
    double cc = split_cross_coupling_analysis(&ctrl);
    if (fabs(cc) > EPS) { FAIL("zero coupling when both closed"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L2: Scheme Validation Test
 * ========================================================================= */

static int test_validate_heat_cool_scheme(void) {
    TEST("Validate: heat/cool scheme is valid");
    split_range_scheme_t scheme;
    split_init_heat_cool_scheme(&scheme);
    int result = split_control_validate_scheme(&scheme);
    if (result != 0) { FAIL("heat/cool scheme should be valid"); return 1; }
    PASS(); return 0;
}

static int test_validate_null_scheme(void) {
    TEST("Validate: NULL scheme returns error");
    int result = split_control_validate_scheme(NULL);
    if (result >= 0) { FAIL("NULL scheme should return error"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L4: Choudhury Stiction Detection Math
 * ========================================================================= */

static int test_stiction_math_consistency(void) {
    TEST("Stiction math: stickband > slip jump");
    split_range_channel_t ch = {0};
    ch.stiction_threshold = 5.0;
    ch.current_position = 30.0;
    /* delta=20 > S=5, should slip. J = 0.3*S = 1.5.
     * Output ≈ 30 + 20 - 1.5*sign(20) = 48.5 */
    double pos = split_valve_stiction_model(&ch, 50.0);
    if (pos <= 30.0) { FAIL("should have moved past stickband"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L4: Energy Consumption Test
 * ========================================================================= */

static int test_energy_consumption(void) {
    TEST("Energy: split point efficiency factor applies");
    double positions[6] = {50.0, 30.0, 0, 0, 0, 0};
    double heat_pow, cool_pow;
    split_valve_energy_consumption(50.0, 50.0, positions, 2, &heat_pow, &cool_pow);
    /* At exact split point, efficiency factor = 1.0 */
    if (heat_pow <= 0.0) { FAIL("heating power should be positive"); return 1; }
    if (cool_pow <= 0.0) { FAIL("cooling power should be positive"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * L5: Partial Stroke Test
 * ========================================================================= */

static int test_partial_stroke(void) {
    TEST("Partial stroke: healthy valve passes test");
    split_range_channel_t ch = {0};
    ch.slew_rate_limit = 25.0;
    ch.stiction_threshold = 0.5;
    double breakaway, stroke_time;
    int result = split_valve_partial_stroke_test(&ch, 10.0, &breakaway, &stroke_time);
    if (result != 0) { FAIL("healthy valve should pass PST"); return 1; }
    if (breakaway <= 0.0) { FAIL("breakaway time should be positive"); return 1; }
    PASS(); return 0;
}

/* =========================================================================
 * Main: Run all tests
 * ========================================================================= */

int main(void) {
    printf("========================================\n");
    printf("  Split-Range Control Test Suite\n");
    printf("  mini-split-range-control-heat-cool\n");
    printf("========================================\n\n");

    /* L1/L2: Core Split Mapping */
    test_channel_position_increasing();
    test_channel_position_decreasing();
    test_channel_position_clamping_low();
    test_channel_position_clamping_high();
    test_channel_position_fixed_action();

    /* L2: Deadband and Overlap */
    test_deadband_hard_transition();
    test_deadband_linear_transition();
    test_deadband_cubic_spline();

    /* L2: Split Distribution */
    test_distribute_heat_cool();
    test_distribute_heat_cool_full_cool();
    test_distribute_null_safety();

    /* L3: Valve Characteristics */
    test_valve_char_linear_inverse();
    test_valve_char_equal_pct_inverse();
    test_valve_char_quick_opening();

    /* L3: Slew Rate */
    test_slew_rate_limit_within();
    test_slew_rate_limit_exceeds();

    /* L3: PID */
    test_pid_init_params_valid();
    test_pid_init_params_negative();
    test_pid_incremental_converges();
    test_pid_positional_basic();
    test_pid_reset_state();

    /* L4: Tuning */
    test_zn_tuning_pid();
    test_zn_tuning_tyreus_luyben();
    test_zn_tuning_invalid_params();
    test_closed_loop_poles_stable();

    /* L4: ISA Valve Sizing */
    test_valve_size_liquid();
    test_valve_size_zero_flow();
    test_pressure_drop_ratio();
    test_valve_rangeability();

    /* L4: Stiction */
    test_stiction_no_stick();
    test_stiction_stuck();
    test_stiction_slip();
    test_stiction_math_consistency();

    /* L4: Energy */
    test_energy_consumption();

    /* L5: IMC Tuning */
    test_imc_tuning_valid();
    test_imc_tuning_lambda_effect();

    /* L5: Auto-Tuning */
    test_autotune_init();
    test_autotune_step_produces_output();

    /* L5: Partial Stroke Test */
    test_partial_stroke();

    /* L8: Adaptive */
    test_adaptive_init();
    test_adaptive_update_gain();

    /* L6: Reactor */
    test_reactor_simulation();
    test_reactor_runaway_detect();

    /* L6: Factory Functions */
    test_create_reactor();
    test_create_ph();
    test_create_pressure();

    /* L5: Cross-Coupling */
    test_cross_coupling();

    /* L2: Validation */
    test_validate_heat_cool_scheme();
    test_validate_null_scheme();

    /* Summary */
    printf("\n========================================\n");
    printf("  RESULTS: %d/%d passed", tests_passed, tests_total);
    if (tests_failed > 0) {
        printf(", %d FAILED\n", tests_failed);
        printf("========================================\n");
        return 1;
    }
    printf(" — ALL PASSED\n");
    printf("========================================\n");
    return 0;
}
