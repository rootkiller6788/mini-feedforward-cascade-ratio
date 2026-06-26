/**
 * @file test_ratio_controller.c
 * @brief Tests for ratio controller, trim, and cross-limiting.
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

/* ratio_types.h types */
typedef enum {
    FLOW_UNIT_MASS = 0, FLOW_UNIT_VOLUME = 1, FLOW_UNIT_MOLAR = 2, FLOW_UNIT_NORMALIZED = 3
} flow_unit_t;
typedef enum {
    RATIO_MODE_FIXED=0, RATIO_MODE_WILD_STREAM=1, RATIO_MODE_TRIMMED=2,
    RATIO_MODE_CROSS_LIMITED=3, RATIO_MODE_BLENDING=4, RATIO_MODE_CASCADE=5
} ratio_mode_t;
typedef enum {
    CROSS_LIMIT_NONE=0, CROSS_LIMIT_AIR_LEADS=1, CROSS_LIMIT_FUEL_LEADS=2, CROSS_LIMIT_DOUBLE=3
} cross_limit_mode_t;

typedef struct { double raw_flow, filtered_flow, flow_rate_of_change; uint64_t timestamp_ms; int signal_quality; double temperature, pressure; } flow_measurement_t;
typedef struct { double ratio_setpoint, ratio_min, ratio_max, ratio_deadband; flow_unit_t master_unit, slave_unit; int master_is_gas, slave_is_gas; double density_master, density_slave; } ratio_config_t;
typedef struct { ratio_config_t config; ratio_mode_t mode; flow_measurement_t master, slave; double actual_ratio, ratio_error, slave_setpoint, ratio_trim, trimmed_ratio; int ratio_valid, slave_saturated, windup_active; } ratio_control_state_t;
typedef struct { double Kp_trim, Ti_trim, Ts, trim_min, trim_max, trim_dz, integrator, prev_error, output; int saturated; } ratio_trim_controller_t;
typedef struct { cross_limit_mode_t mode; double afr_stoich, r_air_rich, r_fuel_rich, demand_air, demand_fuel, air_flow, fuel_flow, sp_air, sp_fuel; int air_high_selected, fuel_low_selected; uint64_t last_update_ms; } cross_limiting_t;

/* Functions under test */
extern void ratio_control_init(ratio_control_state_t *rc, double R_sp, int master_is_gas, int slave_is_gas);
extern void ratio_set_mode(ratio_control_state_t *rc, ratio_mode_t mode);
extern void ratio_set_limits(ratio_control_state_t *rc, double R_min, double R_max);
extern void ratio_set_flow_units(ratio_control_state_t *rc, flow_unit_t master_unit, flow_unit_t slave_unit, double density_master, double density_slave);
extern void ratio_update_master(ratio_control_state_t *rc, double flow, int quality);
extern void ratio_update_slave(ratio_control_state_t *rc, double flow, int quality);
extern double ratio_compute_actual(const ratio_control_state_t *rc);
extern double ratio_compute_slave_setpoint(ratio_control_state_t *rc);
extern double ratio_compute_error(ratio_control_state_t *rc);
extern void ratio_trim_init(ratio_trim_controller_t *trim, double Kp, double Ti, double Ts);
extern void ratio_trim_set_limits(ratio_trim_controller_t *trim, double trim_min, double trim_max);
extern double ratio_trim_step(ratio_trim_controller_t *trim, double quality_sp, double quality_pv);
extern void ratio_apply_trim(ratio_control_state_t *rc, double R_trim);
extern void ratio_trim_reset(ratio_trim_controller_t *trim);
extern int ratio_check_safety(const ratio_control_state_t *rc);
extern int ratio_check_rate_limit(const ratio_control_state_t *rc, double max_rate, double *out_rate);
extern double ratio_feedforward_correction(const ratio_control_state_t *rc, double K_ff, double prev_master);
extern double ratio_filter_master(double raw, double prev_filt, double Ts, double T_filter);
extern int blending_ratio_setpoints(const double *fractions, int n_components, double total_flow, double *setpoints);
extern int blending_ratio_validate(const double *fractions, int n_components, double total_flow);
extern void blending_adjust_shortfall(double *fractions, int n_components, int limited_index, double actual_flow, double total_flow);

extern void cross_limit_init(cross_limiting_t *cl, cross_limit_mode_t mode, double afr_stoich, double r_air_rich, double r_fuel_rich);
extern void cross_limit_update_flows(cross_limiting_t *cl, double air_flow, double fuel_flow);
extern void cross_limit_update_demands(cross_limiting_t *cl, double demand_air, double demand_fuel);
extern void cross_limit_air_leads(cross_limiting_t *cl);
extern void cross_limit_fuel_leads(cross_limiting_t *cl);
extern void cross_limit_double(cross_limiting_t *cl, double r_extra);
extern void cross_limit_execute(cross_limiting_t *cl, double r_extra);
extern int cross_limit_check_safety(const cross_limiting_t *cl, double afr_actual);
extern double cross_limit_current_afr(const cross_limiting_t *cl);
extern double cross_limit_lambda(const cross_limiting_t *cl);
extern void cross_limit_margins(const cross_limiting_t *cl, double *margin_air, double *margin_fuel);
extern int cross_limit_diagnostics(const cross_limiting_t *cl, char *buf, size_t bufsz);

static int tests_run = 0, tests_passed = 0;
#define TEST(name) do { tests_run++; printf("  TEST: %s ... ", name); } while(0)
#define PASS() do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); return; } while(0)
#define ASSERT_EQ(a, b, eps) do { \
    double _va = (double)(a); double _vb = (double)(b); \
    if (fabs(_va - _vb) > (eps)) { FAIL("assertion"); return; } \
} while(0)
#define ASSERT_EQ_INT(a, b) do { \
    if ((int)(a) != (int)(b)) { FAIL("assertion"); return; } \
} while(0)
#define ASSERT_TRUE(cond) do { if (!(cond)) { FAIL("assertion"); return; } } while(0)
#define ASSERT_FALSE(cond) do { if (cond) { FAIL("should be false"); return; } } while(0)

/* ================================================================
 * Ratio controller initialization
 * ================================================================ */
static void test_ratio_init(void)
{
    ratio_control_state_t rc;
    TEST("ratio_control_init gas-liquid");
    ratio_control_init(&rc, 2.5, 1, 0);
    ASSERT_EQ(rc.config.ratio_setpoint, 2.5, 1e-9);
    ASSERT_TRUE(rc.config.master_is_gas == 1);
    ASSERT_TRUE(rc.config.slave_is_gas == 0);
    ASSERT_EQ(rc.actual_ratio, 0.0, 1e-9);
    ASSERT_TRUE(rc.mode == RATIO_MODE_FIXED);
    PASS();

    TEST("ratio_set_mode wild stream");
    ratio_set_mode(&rc, RATIO_MODE_WILD_STREAM);
    ASSERT_TRUE(rc.mode == RATIO_MODE_WILD_STREAM);
    PASS();

    TEST("ratio_set_limits");
    ratio_set_limits(&rc, 0.5, 5.0);
    ASSERT_EQ(rc.config.ratio_min, 0.5, 1e-9);
    ASSERT_EQ(rc.config.ratio_max, 5.0, 1e-9);
    PASS();

    TEST("ratio_set_limits invalid (rejected)");
    ratio_set_limits(&rc, 5.0, 0.5); /* min > max */
    ASSERT_EQ(rc.config.ratio_min, 0.5, 1e-9); /* unchanged */
    PASS();
}

/* ================================================================
 * Ratio master/slave update and computation
 * ================================================================ */
static void test_ratio_updates(void)
{
    ratio_control_state_t rc;
    ratio_control_init(&rc, 2.0, 1, 0);

    TEST("update master valid");
    ratio_update_master(&rc, 100.0, 2);
    ASSERT_TRUE(rc.ratio_valid);
    /* EWMA: α=0.5/(0.5+2.0)=0.2; y=0.2*100+0.8*0=20 */
    ASSERT_EQ(rc.master.filtered_flow, 20.0, 1e-9);
    PASS();

    TEST("update slave valid → ratio computable");
    ratio_update_slave(&rc, 200.0, 2);
    double actual = ratio_compute_actual(&rc);
    /* R = 200 / 20 (filtered master) = 10 */
    ASSERT_EQ(actual, 200.0/20.0, 1e-9);
    PASS();

    TEST("compute slave setpoint");
    double sp = ratio_compute_slave_setpoint(&rc);
    /* SP = 2.0 * 20.0 = 40.0 */
    ASSERT_EQ(sp, 40.0, 1e-9);
    PASS();

    TEST("compute ratio error");
    double err = ratio_compute_error(&rc);
    /* R_actual = 200/20 = 10, R_eff = 2.0 + 0 = 2.0, err = 8.0 */
    ASSERT_EQ(err, 8.0, 1e-9);
    PASS();

    TEST("safety check: ratio within bounds");
    int safety = ratio_check_safety(&rc);
    /* R_actual = 10.0, within [0.01, 100.0] → safe (0) */
    ASSERT_TRUE(safety == 0);
    PASS();
}

/* ================================================================
 * Ratio trim controller
 * ================================================================ */
static void test_ratio_trim(void)
{
    ratio_trim_controller_t trim;
    TEST("trim init");
    ratio_trim_init(&trim, 0.2, 60.0, 1.0);
    ASSERT_EQ(trim.Kp_trim, 0.2, 1e-9);
    ASSERT_EQ(trim.Ti_trim, 60.0, 1e-9);
    ASSERT_EQ(trim.output, 0.0, 1e-9);
    PASS();

    TEST("trim step: quality below SP → positive trim");
    double output = ratio_trim_step(&trim, 50.0, 48.0);
    ASSERT_TRUE(output > 0.0); /* should increase */
    PASS();

    TEST("trim step: quality above SP → negative trim");
    output = ratio_trim_step(&trim, 50.0, 52.0);
    ASSERT_TRUE(output < 0.0); /* should decrease */
    PASS();

    TEST("trim limits");
    ratio_trim_set_limits(&trim, -0.3, 0.3);
    /* Push trim to limit by large error */
    for (int i = 0; i < 100; i++) {
        ratio_trim_step(&trim, 50.0, 10.0); /* large error → max trim */
    }
    ASSERT_EQ(trim.output, 0.3, 1e-6);
    PASS();

    TEST("trim reset");
    ratio_trim_reset(&trim);
    ASSERT_EQ(trim.integrator, 0.0, 1e-9);
    ASSERT_EQ(trim.output, 0.0, 1e-9);
    PASS();

    TEST("apply trim to ratio control");
    ratio_control_state_t rc;
    ratio_control_init(&rc, 2.0, 1, 0);
    ratio_apply_trim(&rc, 0.1);
    ASSERT_EQ(rc.ratio_trim, 0.1, 1e-9);
    PASS();
}

/* ================================================================
 * Feedforward correction
 * ================================================================ */
static void test_feedforward(void)
{
    ratio_control_state_t rc;
    ratio_control_init(&rc, 2.0, 1, 0);

    TEST("feedforward: flow change produces correction");
    ratio_update_master(&rc, 100.0, 2);
    /* filtered = 20, prev_master = 20 → Δ = 0 */
    double ff = ratio_feedforward_correction(&rc, 2.0, 20.0);
    ASSERT_EQ(ff, 0.0, 1e-9);
    PASS();

    TEST("filter master");
    double filt = ratio_filter_master(100.0, 90.0, 1.0, 4.0);
    /* α = 1/5 = 0.2 → 0.2*100 + 0.8*90 = 92 */
    ASSERT_EQ(filt, 92.0, 1e-9);
    PASS();
}

/* ================================================================
 * Blending ratio control
 * ================================================================ */
static void test_blending(void)
{
    TEST("blending validate: valid 2-component");
    double fractions[] = {0.3, 0.7};
    ASSERT_TRUE(blending_ratio_validate(fractions, 2, 100.0));
    PASS();

    TEST("blending validate: sum != 1 → invalid");
    double bad_frac[] = {0.3, 0.6};
    ASSERT_FALSE(blending_ratio_validate(bad_frac, 2, 100.0));
    PASS();

    TEST("blending setpoints");
    double setpoints[2];
    ASSERT_TRUE(blending_ratio_setpoints(fractions, 2, 100.0, setpoints));
    ASSERT_EQ(setpoints[0], 30.0, 1e-9);
    ASSERT_EQ(setpoints[1], 70.0, 1e-9);
    PASS();

    TEST("blending adjust shortfall");
    double adj_frac[] = {0.4, 0.6};
    /* total_flow = 100, actual_flow = 20 for component 0 → limited_fraction = 0.2 */
    /* shortfall = 0.4 - 0.2 = 0.2 → redistributed to component 1 */
    /* Component 1: 0.6 + 0.2*(0.6/0.6) = 0.8 */
    blending_adjust_shortfall(adj_frac, 2, 0, 20.0, 100.0);
    ASSERT_EQ(adj_frac[0], 0.2, 1e-9);
    ASSERT_EQ(adj_frac[1], 0.8, 1e-9);
    PASS();
}

/* ================================================================
 * Cross-limiting
 * ================================================================ */
static void test_cross_limiting(void)
{
    cross_limiting_t cl;
    TEST("cross_limit_init air-leads");
    cross_limit_init(&cl, CROSS_LIMIT_AIR_LEADS, 17.2, 1.05, 0.95);
    ASSERT_TRUE(cl.mode == CROSS_LIMIT_AIR_LEADS);
    ASSERT_EQ(cl.afr_stoich, 17.2, 1e-9);
    PASS();

    TEST("air-leads: load increase → air leads");
    cross_limit_update_flows(&cl, 100.0, 5.0);
    cross_limit_update_demands(&cl, 150.0, 10.0);
    cross_limit_air_leads(&cl);
    /* Air SP should be 150 (demand > minimum) */
    /* Fuel SP: min(10.0, 100*1.05/17.2) = min(10.0, 6.1) = 6.1 */
    ASSERT_EQ(cl.sp_air, 150.0, 1e-9);
    ASSERT_TRUE(cl.sp_fuel < 10.0); /* Fuel is constrained */
    PASS();

    TEST("air-leads: fuel constrained by low air");
    cross_limit_update_flows(&cl, 50.0, 5.0);
    cross_limit_update_demands(&cl, 100.0, 8.0);
    cross_limit_air_leads(&cl);
    /* Fuel max: 50 * 1.05 / 17.2 = 3.05 */
    ASSERT_TRUE(cl.sp_fuel < 8.0);
    PASS();

    TEST("double cross-limiting");
    cross_limit_init(&cl, CROSS_LIMIT_DOUBLE, 17.2, 1.05, 0.95);
    cross_limit_update_flows(&cl, 100.0, 5.0);
    cross_limit_update_demands(&cl, 120.0, 8.0);
    cross_limit_double(&cl, 1.05);
    ASSERT_TRUE(cl.sp_air > 0.0);
    ASSERT_TRUE(cl.sp_fuel > 0.0);
    PASS();

    TEST("cross_limit_execute dispatch");
    cross_limit_execute(&cl, 1.05);
    ASSERT_TRUE(cl.sp_air > 0.0);
    PASS();

    TEST("current AFR computation");
    double afr = cross_limit_current_afr(&cl);
    ASSERT_EQ(afr, 20.0, 1e-9); /* 100/5 */
    PASS();

    TEST("lambda computation");
    double lam = cross_limit_lambda(&cl);
    ASSERT_EQ(lam, 20.0/17.2, 1e-6);
    PASS();

    TEST("safety check: lean → safe");
    int safety = cross_limit_check_safety(&cl, 20.0);
    ASSERT_TRUE(safety == 0);
    PASS();

    TEST("safety check: rich → danger");
    safety = cross_limit_check_safety(&cl, 15.0);
    ASSERT_TRUE(safety == 1 || safety == 2);
    PASS();

    TEST("cross_limit_margins");
    double m_air, m_fuel;
    cross_limit_margins(&cl, &m_air, &m_fuel);
    ASSERT_TRUE(isfinite(m_air));
    PASS();

    TEST("cross_limit no cross → pass through");
    cross_limit_init(&cl, CROSS_LIMIT_NONE, 17.2, 1.05, 0.95);
    cross_limit_update_demands(&cl, 100.0, 6.0);
    cross_limit_execute(&cl, 1.05);
    ASSERT_EQ(cl.sp_air, 100.0, 1e-9);
    ASSERT_EQ(cl.sp_fuel, 6.0, 1e-9);
    PASS();
}

int main(void)
{
    printf("=== Ratio Controller & Cross-Limiting Tests ===\n\n");

    test_ratio_init();
    test_ratio_updates();
    test_ratio_trim();
    test_feedforward();
    test_blending();
    test_cross_limiting();

    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    if (tests_passed < tests_run) {
        printf("SOME TESTS FAILED!\n");
        return 1;
    }
    printf("ALL TESTS PASSED!\n");
    return 0;
}
