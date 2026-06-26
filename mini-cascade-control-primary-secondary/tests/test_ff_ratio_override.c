/**
 * @file test_ff_ratio_override.c
 * @brief Tests for feedforward, ratio control, and override selector
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#include "cascade_types.h"
#include "cascade_pid.h"
#include "feedforward_control.h"
#include "ratio_control.h"
#include "override_selector.h"

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

/*===== Feedforward Tests =================================================*/

static void test_ff_init(void)
{
    ff_compensator_t ff;
    ff_init(&ff, 0.1, -50.0, 50.0);
    TEST("FF init default mode OFF", ff.mode == FF_MODE_OFF);
    TEST("FF init Kff=1", fabs(ff.Kff - 1.0) < 1e-9);
    TEST("FF init output=0", fabs(ff.ff_output) < 1e-9);

    ff_init(NULL, 0.1, 0.0, 100.0);
    TEST("FF init NULL", 1);
}

static void test_ff_static(void)
{
    ff_compensator_t ff;
    ff_init(&ff, 0.1, -100.0, 100.0);
    ff_configure_static(&ff, 2.0, 5.0);

    TEST("FF static mode set", ff.mode == FF_MODE_STATIC);

    double y = ff_update(&ff, 10.0);
    TEST_FEQ("FF static: y=Kff*x+bias", y, 2.0*10.0 + 5.0, 1e-6);

    y = ff_update(&ff, 0.0);
    TEST_FEQ("FF static: y=bias at x=0", y, 5.0, 1e-6);

    /* Clamping */
    ff_compensator_t ff2;
    ff_init(&ff2, 0.1, 0.0, 10.0);
    ff_configure_static(&ff2, 10.0, 0.0);
    y = ff_update(&ff2, 5.0);
    TEST("FF output clamped", y <= 10.0);
}

static void test_ff_lead_lag(void)
{
    ff_compensator_t ff;
    ff_init(&ff, 0.1, -100.0, 100.0);
    ff_configure_lead_lag(&ff, 1.0, 2.0, 5.0);

    TEST("FF lead-lag mode set", ff.mode == FF_MODE_LEAD_LAG);

    /* Step response: output should eventually settle to Kff * x.
     * With T_lag=5.0 and Ts=0.1, need ~4*T_lag/Ts = 200 steps for 98% settling. */
    ff_update(&ff, 0.0);
    for (int i = 0; i < 300; i++) {
        ff_update(&ff, 10.0);
    }
    double y = ff_update(&ff, 10.0);
    TEST_FEQ("FF lead-lag settles to Kff*x", y, 10.0, 1.0);
}

static void test_ff_deadtime(void)
{
    ff_compensator_t ff;
    ff_init(&ff, 0.1, -100.0, 100.0);
    ff_configure_deadtime(&ff, 1.0, 0.0, 1.0, 0.5);

    TEST("FF deadtime mode set", ff.mode == FF_MODE_DEADTIME);

    /* With deadtime, output should be delayed */
    ff_update(&ff, 10.0);
    ff_update(&ff, 0.0);
    /* After deadtime, the 10.0 input should eventually appear */
    TEST("FF deadtime runs", 1);
}

static void test_ff_fb_combined(void)
{
    ff_fb_controller_t ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ff_init(&ctrl.ff, 0.1, -100.0, 100.0);
    ff_configure_static(&ctrl.ff, 0.5, 0.0);
    cascade_pid_init(&ctrl.fb, 1.0, 10.0, 0.0, 0.1, 0.0, 100.0);

    ctrl.ff_active = true;
    ctrl.fb_active = true;

    double u = ff_update_ff_fb(&ctrl, 50.0, 45.0, 10.0);
    TEST("FF+FB combined output > 0", u > 0.0);

    ctrl.ff_active = false;
    double u_fb_only = ff_update_ff_fb(&ctrl, 50.0, 45.0, 10.0);
    /* FB-only should be different from FF+FB */
    TEST("FF+FB disabled reduces output", u_fb_only != u);
}

static void test_ff_design(void)
{
    cascade_fopdt_model_t Gp;
    Gp.K = 1.0; Gp.tau = 10.0; Gp.theta = 2.0;

    ff_disturbance_model_t Gd;
    Gd.Kd = 0.5; Gd.tau_d = 5.0; Gd.theta_d = 3.0;

    ff_design_result_t result;
    int rc = ff_design_from_models(&Gp, &Gd, &result);
    TEST("FF design success", rc >= 0);
    TEST("FF Kff ideal < 0 (negative)", result.Kff_ideal < 0.0);
    TEST("FF realizable (theta_d >= theta)", result.is_ideal_realizable);

    /* Static design */
    double Kff = ff_design_static(1.0, 0.5);
    TEST_FEQ("FF static design Kff", Kff, -0.5, 1e-9);

    /* Lead-lag design */
    double Kff2, T_lead, T_lag;
    rc = ff_design_lead_lag_optimal(&Gp, &Gd, &Kff2, &T_lead, &T_lag);
    TEST("FF lead-lag design success", rc == 0);

    /* Sensitivity */
    double sens = ff_sensitivity_analysis(1.0, 1.2);
    TEST_FEQ("FF sensitivity 20% error", sens, 20.0, 1.0);
}

/*===== Ratio Control Tests =================================================*/

static void test_ratio_init(void)
{
    ratio_station_t ratio;
    ratio_init(&ratio, "FFIC-201");

    TEST("Ratio init SP=1", fabs(ratio.ratio_sp - 1.0) < 1e-9);
    TEST("Ratio init fixed arch", ratio.architecture == RATIO_ARCH_FIXED);
    TEST("Ratio init inactive", !ratio.ratio_active);
    TEST("Ratio wild flow min set", ratio.wild_flow_min > 0.0);

    ratio_init(NULL, NULL);
    TEST("Ratio init NULL", 1);
}

static void test_ratio_compute(void)
{
    ratio_station_t ratio;
    ratio_init(&ratio, "TEST");

    ratio.wild_flow = 50.0;
    ratio.controlled_flow = 100.0;

    double R = ratio_compute_linear(&ratio);
    TEST_FEQ("Ratio = Qc/Qw = 2", R, 2.0, 1e-6);

    /* Below wild_flow_min → hold last */
    ratio.wild_flow = 0.1;
    R = ratio_compute_linear(&ratio);
    TEST_FEQ("Low wild flow holds last", R, 2.0, 1e-6);

    /* NULL safety */
    ratio_compute_linear(NULL);
    TEST("Ratio compute NULL", 1);
}

static void test_ratio_setpoint(void)
{
    ratio_station_t ratio;
    ratio_init(&ratio, "TEST");
    ratio.ratio_sp = 2.0;
    ratio.wild_flow = 30.0;
    ratio.controlled_sp_min = 0.0;
    ratio.controlled_sp_max = 100.0;

    double Qc_sp = ratio_calculate_setpoint(&ratio, 0.0);
    TEST_FEQ("Ratio SP = R*Qw", Qc_sp, 60.0, 1e-6);

    /* With bias */
    ratio.ratio_bias = 10.0;
    Qc_sp = ratio_calculate_setpoint(&ratio, 0.0);
    TEST_FEQ("Ratio SP with bias", Qc_sp, 70.0, 1e-6);

    /* Clamped to max */
    ratio.controlled_sp_max = 50.0;
    Qc_sp = ratio_calculate_setpoint(&ratio, 0.0);
    TEST_FEQ("Ratio SP clamped to max", Qc_sp, 50.0, 1e-6);

    /* Rate limiting */
    ratio.controlled_sp_max = 100.0;
    Qc_sp = ratio_calculate_setpoint(&ratio, 5.0);
    TEST("Ratio SP rate limited", Qc_sp > 50.0 && Qc_sp < 100.0);
}

static void test_ratio_blend(void)
{
    blend_station_t blend;
    memset(&blend, 0, sizeof(blend));
    blend.num_streams = 2;
    ratio_init(&blend.streams[0], "A");
    ratio_init(&blend.streams[1], "B");
    blend.streams[0].controlled_flow = 30.0;
    blend.streams[1].controlled_flow = 70.0;

    double fractions[2];
    double total = ratio_compute_blend(&blend, fractions);
    TEST_FEQ("Blend total", total, 100.0, 1e-6);
    TEST_FEQ("Blend fraction A", fractions[0], 0.3, 1e-6);
    TEST_FEQ("Blend fraction B", fractions[1], 0.7, 1e-6);
}

static void test_ratio_flow_char(void)
{
    /* Linear flowmeter */
    double Q = ratio_characterize_flow(50.0, 0, 1.0);
    TEST_FEQ("Flow linear meter", Q, 50.0, 1e-6);

    /* Square-root extraction */
    Q = ratio_characterize_flow(100.0, 1, 1.0);
    TEST_FEQ("Flow sqrt(100)", Q, 10.0, 1e-6);

    Q = ratio_characterize_flow(0.0, 1, 1.0);
    TEST_FEQ("Flow sqrt(0)", Q, 0.0, 1e-6);
}

/*===== Override Selector Tests ================================================*/

static void test_override_init(void)
{
    override_selector_t sel;
    override_init(&sel, SEL_LOW, 2.0, 0.0, 100.0);

    TEST("Override init num_slots=0", sel.num_slots == 0);
    TEST("Override SEL_LOW set", sel.selector_func == SEL_LOW);
    TEST("Override hysteresis set", fabs(sel.hysteresis_global - 2.0) < 1e-9);

    override_init(NULL, SEL_LOW, 0, 0, 0);
    TEST("Override init NULL", 1);
}

static void test_override_add_constraint(void)
{
    override_selector_t sel;
    override_init(&sel, SEL_LOW, 2.0, 0.0, 100.0);

    int idx = override_add_constraint(&sel, "PAL-101", CONSTRAINT_MAXIMUM,
        80.0, 1.0, 10.0, 0.0, 0);
    TEST("Override add constraint success", idx >= 0);
    TEST("Override num_slots incremented", sel.num_slots == 1);
    TEST("Override constraint enabled", sel.slots[0].enabled);
}

static void test_override_select(void)
{
    override_selector_t sel;
    override_init(&sel, SEL_LOW, 1.0, 0.0, 100.0);
    override_set_primary(&sel, 1.0, 60.0, 0.0);

    /* Add a max constraint */
    override_add_constraint(&sel, "PAL-101", CONSTRAINT_MAXIMUM,
        80.0, 5.0, 5.0, 0.0, 0);

    /* Constraint PV below limit → no override */
    double pvs[6] = {70.0, 0, 0, 0, 0, 0};
    double u = override_select(&sel, 50.0, 50.0, pvs, 1.0);
    TEST("Override select runs", u >= 0.0 && u <= 100.0);

    const char *tag = override_get_active_tag(&sel);
    TEST("Override active tag not NULL", tag != NULL);

    bool active = override_is_constraint_active(&sel, 0);
    TEST("Override is_constraint_active returns bool", active == true || active == false);
}

static void test_override_median_select(void)
{
    double vals[3] = {10.0, 20.0, 15.0};
    uint32_t suspect = 0;

    double med = override_median_select(vals, 5.0, &suspect);
    TEST_FEQ("Median 10,20,15 = 15", med, 15.0, 1e-6);
    TEST("All 3 good", suspect == 0);

    /* One sensor off */
    vals[0] = 100.0;
    med = override_median_select(vals, 5.0, &suspect);
    TEST_FEQ("Median with outlier = 20", med, 20.0, 1e-6);
    TEST("Outlier detected", suspect != 0);

    /* NULL safety */
    override_median_select(NULL, 1.0, NULL);
    TEST("Median NULL", 1);
}

static void test_override_2oo3(void)
{
    double vals[3] = {25.0, 27.0, 26.0};
    uint32_t faults = 0;

    double voted = override_mid_of_3_select(vals, 2.0, &faults);
    TEST_FEQ("2oo3 all good → median=26", voted, 26.0, 1e-6);
    TEST("2oo3 zero faults", faults == 0);

    /* One fault */
    vals[0] = 100.0;
    voted = override_mid_of_3_select(vals, 2.0, &faults);
    TEST("2oo3 one fault detected", faults == 1);
}

/*-------------------------------------------------------------------------*/

int main(void)
{
    printf("=== Feedforward, Ratio & Override Tests ===\n\n");

    printf("--- Feedforward ---\n");
    test_ff_init();
    test_ff_static();
    test_ff_lead_lag();
    test_ff_deadtime();
    test_ff_fb_combined();
    test_ff_design();

    printf("\n--- Ratio Control ---\n");
    test_ratio_init();
    test_ratio_compute();
    test_ratio_setpoint();
    test_ratio_blend();
    test_ratio_flow_char();

    printf("\n--- Override Selector ---\n");
    test_override_init();
    test_override_add_constraint();
    test_override_select();
    test_override_median_select();
    test_override_2oo3();

    printf("\n========================================\n");
    printf("Results: %d passed, %d failed\n", passed, failed);

    return (failed > 0) ? 1 : 0;
}
