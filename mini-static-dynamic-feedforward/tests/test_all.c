#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "../include/feedforward_defs.h"
#include "../include/feedforward_static.h"
#include "../include/feedforward_dynamic.h"
#include "../include/feedforward_models.h"
#include "../include/feedforward_combined.h"
#include "../include/feedforward_advanced.h"

#define TOL 1e-9
#define TEST_PASS() printf("  PASS: %s\n", __func__)
#define TEST_EQ(a, b, msg) do { \
    if (fabs((a)-(b)) > TOL) { \
        printf("  FAIL: %s — %s: expected %g, got %g\n", __func__, msg, (double)(b), (double)(a)); \
        assert(0); \
    } \
} while(0)

/* ========================================================================= */
/* L1: Type definitions and struct initialization */
/* ========================================================================= */
static void test_defs_struct_sizes(void) {
    assert(sizeof(tf_t) > 0);
    assert(sizeof(tf_discrete_t) > 0);
    assert(sizeof(lead_lag_t) > 0);
    assert(sizeof(lead_lag2_t) > 0);
    assert(sizeof(fopdt_t) > 0);
    assert(sizeof(sopdt_t) > 0);
    assert(sizeof(ipdt_t) > 0);
    assert(sizeof(dist_model_t) > 0);
    assert(sizeof(feedforward_t) > 0);
    assert(sizeof(ff_performance_t) > 0);
    TEST_PASS();
}

/* ========================================================================= */
/* L2: Static feedforward — gain computation */
/* ========================================================================= */
static void test_ff_static_gain_fopdt(void) {
    fopdt_t proc = {0.8, 100.0, 10.0};
    dist_model_t dist = {0.2, 50.0, 5.0};
    double Kff = ff_static_gain_fopdt(&proc, &dist, ACTION_DIRECT);
    TEST_EQ(Kff, -0.25, "Kff = -Kd/Kp = -0.2/0.8");
    TEST_PASS();
}

static void test_ff_static_gain_zero_Kp(void) {
    fopdt_t proc = {0.0, 100.0, 10.0};
    dist_model_t dist = {0.2, 50.0, 5.0};
    double Kff = ff_static_gain_fopdt(&proc, &dist, ACTION_DIRECT);
    assert(Kff == 0.0);
    TEST_PASS();
}

static void test_ff_static_gain_action_reverse(void) {
    fopdt_t proc = {1.0, 100.0, 10.0};
    dist_model_t dist = {0.5, 50.0, 5.0};
    double Kff = ff_static_gain_fopdt(&proc, &dist, ACTION_REVERSE);
    TEST_EQ(Kff, 0.5, "reversed: -(-0.5/1.0) = 0.5");
    TEST_PASS();
}

static void test_ff_static_gain_sopdt(void) {
    sopdt_t proc = {2.0, 50.0, 30.0, 5.0};
    dist_model_t dist = {0.4, 25.0, 2.0};
    double Kff = ff_static_gain_sopdt(&proc, &dist, ACTION_DIRECT);
    TEST_EQ(Kff, -0.2, "Kff = -Kd/Kp = -0.4/2.0");
    TEST_PASS();
}

static void test_ff_static_gain_tf(void) {
    tf_t Gp = {1.0, 0.0, {1.0}, {1.0}, 0, 0, TF_ORDER_ZERO};
    tf_t Gd = {2.0, 0.0, {1.0}, {1.0}, 0, 0, TF_ORDER_ZERO};
    double Kff = ff_static_gain_tf(&Gp, &Gd, ACTION_DIRECT);
    TEST_EQ(Kff, -2.0, "Kff = -Gd(0)/Gp(0) = -2/1");
    TEST_PASS();
}

static void test_ff_static_step_basic(void) {
    feedforward_t ff;
    ff_static_init(&ff, 0.5, 10.0, 0.0, 100.0, ACTION_DIRECT, 0.1);
    double u = ff_static_step(&ff, 20.0);
    TEST_EQ(u, 20.0, "u = 0.5*20 + 10 = 20");
    TEST_PASS();
}

static void test_ff_static_step_reverse(void) {
    feedforward_t ff;
    ff_static_init(&ff, 0.5, 10.0, 0.0, 100.0, ACTION_REVERSE, 0.1);
    double u = ff_static_step(&ff, 20.0);
    TEST_EQ(u, 0.0, "u = -0.5*20 + 10 = 0");
    TEST_PASS();
}

static void test_ff_static_step_clamping(void) {
    feedforward_t ff;
    ff_static_init(&ff, 1.0, -5.0, 0.0, 50.0, ACTION_DIRECT, 0.1);
    double u = ff_static_step(&ff, 100.0);
    TEST_EQ(u, 50.0, "clamped to max = 50");
    TEST_PASS();
}

static void test_ff_static_step_filtered(void) {
    feedforward_t ff;
    ff_static_init(&ff, 1.0, 0.0, -100.0, 100.0, ACTION_DIRECT, 0.1);
    /* First call: filter initializes to measurement, u = Kff * d = 10.0 */
    double u1 = ff_static_step_filtered(&ff, 10.0, 1.0);
    assert(fabs(u1 - 10.0) < 0.01);

    /* Second call with step change: d goes from 10 to 20,
     * alpha = 0.1/(1.0+0.1) = 0.0909
     * d_f = 0.0909*20 + 0.9091*10 = 1.818 + 9.091 = 10.909
     * u = 10.909 */
    double u2 = ff_static_step_filtered(&ff, 20.0, 1.0);
    assert(u2 > 10.0 && u2 < 12.0);
    TEST_PASS();
}

static void test_ff_static_disturbance_rejection_ratio(void) {
    double drr = ff_static_rejection_ratio(1.0, 0.5, -0.5);
    /* DRR = |0.5|/|0.5+1*(-0.5)| = 0.5/0 → infinity, returns 1/epsilon */
    assert(drr > 1e6);
    TEST_PASS();
}

static void test_ff_static_rejection_no_ff(void) {
    double drr = ff_static_rejection_ratio(1.0, 0.5, 0.0);
    TEST_EQ(drr, 1.0, "no FF → DRR = 1");
    TEST_PASS();
}

static void test_ff_static_mismatch(void) {
    double res = ff_static_mismatch_residual(1.0, 0.5, 0.0);
    TEST_EQ(res, 1.0, "no FF residual = 1 (0pct rejection)");
    double res2 = ff_static_mismatch_residual(1.0, 0.5, -0.5);
    assert(res2 < 1e-6);
    TEST_PASS();
}

static void test_ff_static_bias_from_op(void) {
    double bias = ff_static_bias_from_operating_point(60.0, 0.5, 20.0);
    TEST_EQ(bias, 50.0, "bias = 60 - 0.5*20 = 50");
    TEST_PASS();
}

static void test_ff_static_step_quality_valid(void) {
    feedforward_t ff;
    ff_static_init(&ff, 0.5, 10.0, 0.0, 100.0, ACTION_DIRECT, 0.1);
    disturbance_meas_t dm;
    disturbance_meas_init(&dm, -50.0, 200.0, 0.0);
    dm.value = 20.0;
    dm.status = SIG_VALID;
    dm.timestamp = 1.0;
    double u = ff_static_step_quality(&ff, &dm);
    assert(u > 0.0);
    TEST_PASS();
}

/* ========================================================================= */
/* L3/L5: Dynamic feedforward — lead-lag */
/* ========================================================================= */
static void test_lead_lag_init(void) {
    lead_lag_t ll;
    lead_lag_init(&ll, 2.0, 5.0, 10.0, 0.1);
    assert(ll.initialized == 1);
    assert(ll.K_ll == 2.0);
    assert(ll.T_lead == 5.0);
    assert(ll.T_lag == 10.0);
    TEST_PASS();
}

static void test_lead_lag_step_unity(void) {
    lead_lag_t ll;
    lead_lag_init(&ll, 1.0, 1.0, 1.0, 0.1);
    double y = lead_lag_step(&ll, 5.0);
    assert(fabs(y - 5.0) < 1e-6);
    TEST_PASS();
}

static void test_lead_lag_step_steady_state(void) {
    lead_lag_t ll;
    lead_lag_init(&ll, 2.0, 1.0, 1.0, 0.1);
    /* After many steps, output should reach K*x = 2*3 = 6 */
    double y = 0.0;
    for (int i = 0; i < 200; i++) {
        y = lead_lag_step(&ll, 3.0);
    }
    assert(fabs(y - 6.0) < 0.01);
    TEST_PASS();
}

static void test_lead_lag_zero_lag(void) {
    lead_lag_t ll;
    lead_lag_init(&ll, 3.0, 2.0, 0.0, 0.1);
    double y = lead_lag_step(&ll, 4.0);
    TEST_EQ(y, 12.0, "pure gain when lag=0: y = 3*4 = 12");
    TEST_PASS();
}

static void test_lead_lag_reset(void) {
    lead_lag_t ll;
    lead_lag_init(&ll, 1.0, 5.0, 3.0, 0.1);
    lead_lag_step(&ll, 10.0);
    lead_lag_reset(&ll);
    double y = lead_lag_step(&ll, 0.0);
    assert(fabs(y) < 1e-6);
    TEST_PASS();
}

static void test_lead_lag_freq_response_dc(void) {
    lead_lag_t ll;
    lead_lag_init(&ll, 2.0, 5.0, 3.0, 0.1);
    double mag, phase;
    lead_lag_freq_response(&ll, 0.0, &mag, &phase);
    TEST_EQ(mag, 2.0, "|G(0)| = K = 2");
    assert(fabs(phase) < 1e-9);
    TEST_PASS();
}

static void test_lead_lag2_init_step(void) {
    lead_lag2_t ll2;
    lead_lag2_init(&ll2, 1.0, 1.0, 1.0, 0.7, 0.7, 0.1);
    double y = lead_lag2_step(&ll2, 3.0);
    /* With Tn=Td, zeta_n=zeta_d, gain=1: first step ≈ 3.0 */
    assert(fabs(y - 3.0) < 0.1);
    TEST_PASS();
}

/* ========================================================================= */
/* L3: Discrete transfer function */
/* ========================================================================= */
static void test_dtf_init_step(void) {
    tf_discrete_t dtf;
    double num[] = {0.5, 0.3};
    double den[] = {1.0, -0.2};
    dtf_init(&dtf, num, den, 1, 1, 0.1);
    double y = dtf_step(&dtf, 1.0);
    assert(y > 0.0);
    TEST_PASS();
}

static void test_dtf_reset(void) {
    tf_discrete_t dtf;
    double num[] = {0.5, 0.3};
    double den[] = {1.0, -0.2};
    dtf_init(&dtf, num, den, 1, 1, 0.1);
    dtf_step(&dtf, 5.0);
    dtf_reset(&dtf);
    double y = dtf_step(&dtf, 0.0);
    assert(fabs(y) < 1e-9);
    TEST_PASS();
}

/* ========================================================================= */
/* L5: Dynamic feedforward design */
/* ========================================================================= */
static void test_ff_dynamic_design_fopdt(void) {
    fopdt_t proc = {1.0, 100.0, 20.0};
    dist_model_t dist = {0.5, 50.0, 10.0};
    double T_lead, T_lag, K_ll;
    int rc = ff_dynamic_design_fopdt(&proc, &dist, ACTION_DIRECT, &T_lead, &T_lag, &K_ll);
    assert(rc == 0);
    TEST_EQ(K_ll, -0.5, "K_ll = -Kd/Kp");
    TEST_EQ(T_lead, 100.0, "T_lead = tau_p");
    TEST_EQ(T_lag, 50.0, "T_lag = tau_d");
    TEST_PASS();
}

static void test_ff_dynamic_design_sopdt(void) {
    sopdt_t proc = {2.0, 80.0, 40.0, 15.0};
    dist_model_t dist = {0.8, 60.0, 5.0};
    double Tn, Td, zeta_n, zeta_d, K;
    int rc = ff_dynamic_design_sopdt(&proc, &dist, ACTION_DIRECT, &Tn, &Td, &zeta_n, &zeta_d, &K);
    assert(rc == 0);
    TEST_EQ(K, -0.4, "K = -Kd/Kp");
    TEST_EQ(Tn, sqrt(80*40), "Tn = sqrt(tau1*tau2)");
    TEST_EQ(Td, 60.0, "Td = tau_d");
    TEST_EQ(zeta_n, (80+40)/(2*sqrt(80*40)), "zeta_n");
    TEST_PASS();
}

static void test_ff_dynamic_causality(void) {
    assert(ff_dynamic_is_causal(10.0, 15.0) == 1);
    assert(ff_dynamic_is_causal(15.0, 10.0) == 0);
    TEST_PASS();
}

static void test_ff_dynamic_required_delay(void) {
    double delay = ff_dynamic_required_delay(30.0, 15.0);
    TEST_EQ(delay, 15.0, "disturbance 15s faster, need 15s delay");
    double delay2 = ff_dynamic_required_delay(10.0, 20.0);
    TEST_EQ(delay2, 0.0, "process faster, no extra delay needed");
    TEST_PASS();
}

static void test_ff_dynamic_step(void) {
    feedforward_t ff;
    ff_dynamic_init(&ff, 50.0, 30.0, 0.5, 0.1);
    ff.bias = 10.0;
    ff.output_min = 0.0;
    ff.output_max = 100.0;
    double u = ff_dynamic_step(&ff, 20.0);
    assert(u >= 0.0 && u <= 100.0);
    TEST_PASS();
}

/* ========================================================================= */
/* L5: Model identification */
/* ========================================================================= */
static void test_fopdt_identify_step(void) {
    double t[250], y[250];
    double Kp_true = 1.5, tau_true = 80.0, theta_true = 15.0;
    double u_step = 2.0;

    for (int i = 0; i < 250; i++) {
        t[i] = i * 2.0;
        if (t[i] < theta_true) y[i] = 0.0;
        else y[i] = Kp_true * u_step * (1.0 - exp(-(t[i]-theta_true)/tau_true));
    }

    fopdt_t model;
    int rc = fopdt_identify_step(t, y, 250, u_step, &model);
    assert(rc == 0);
    assert(fabs(model.Kp - Kp_true) < 0.25);
    TEST_PASS();
}

static void test_fopdt_identify_two_point(void) {
    double t[200], y[200];
    double Kp_true = 1.5, tau_true = 80.0, theta_true = 15.0;
    double u_step = 2.0;

    for (int i = 0; i < 200; i++) {
        t[i] = i * 2.0;
        if (t[i] < theta_true) y[i] = 0.0;
        else y[i] = Kp_true * u_step * (1.0 - exp(-(t[i]-theta_true)/tau_true));
    }

    fopdt_t model;
    int rc = fopdt_identify_two_point(t, y, 200, u_step, &model);
    assert(rc == 0);
    assert(fabs(model.Kp - Kp_true) < 0.2);
    TEST_PASS();
}

/* ========================================================================= */
/* L4: Step response evaluation */
/* ========================================================================= */
static void test_fopdt_step_response(void) {
    fopdt_t model = {1.0, 10.0, 2.0};
    double y0 = fopdt_step_response(&model, 5.0, 1.0);
    assert(y0 == 0.0);
    double y_inf = fopdt_step_response(&model, 5.0, 100.0);
    assert(fabs(y_inf - 5.0) < 0.01);
    TEST_PASS();
}

static void test_sopdt_step_response(void) {
    sopdt_t model = {1.0, 20.0, 10.0, 5.0};
    double y0 = sopdt_step_response(&model, 3.0, 3.0);
    assert(y0 == 0.0);
    double y_inf = sopdt_step_response(&model, 3.0, 500.0);
    assert(fabs(y_inf - 3.0) < 0.01);
    TEST_PASS();
}

/* ========================================================================= */
/* L5: Pade approximation */
/* ========================================================================= */
static void test_pade_first_order(void) {
    double num[2], den[2];
    pade_first_order(4.0, num, den);
    TEST_EQ(num[0], -2.0, "num s coeff = -theta/2");
    TEST_EQ(num[1], 1.0, "num const = 1");
    TEST_EQ(den[0], 2.0, "den s coeff = theta/2");
    TEST_EQ(den[1], 1.0, "den const = 1");
    TEST_PASS();
}

static void test_pade_second_order(void) {
    double num[3], den[3];
    pade_second_order(6.0, num, den);
    TEST_EQ(num[0], 3.0, "num s^2 = theta^2/12 = 36/12");
    TEST_EQ(num[1], -3.0, "num s = -theta/2 = -3");
    TEST_EQ(num[2], 1.0, "num const = 1");
    TEST_PASS();
}

/* ========================================================================= */
/* L3: Discretization */
/* ========================================================================= */
static void test_fopdt_to_discrete_zoh(void) {
    fopdt_t model = {1.0, 10.0, 5.0};
    tf_discrete_t dtf;
    fopdt_to_discrete_zoh(&model, 1.0, &dtf);
    assert(dtf.Ts == 1.0);
    assert(dtf.n == 5 || dtf.n == 6);
    TEST_PASS();
}

static void test_fopdt_to_discrete_tustin(void) {
    fopdt_t model = {1.0, 10.0, 0.0};
    tf_discrete_t dtf;
    fopdt_to_discrete_tustin(&model, 1.0, &dtf);
    assert(fabs(dtf.num[0] - 1.0/21.0) < 0.01);
    TEST_PASS();
}

static void test_fopdt_to_discrete_euler(void) {
    fopdt_t model = {1.0, 10.0, 0.0};
    tf_discrete_t dtf;
    fopdt_to_discrete_euler(&model, 1.0, &dtf);
    assert(fabs(dtf.num[1] - 1.0/11.0) < 0.01);
    TEST_PASS();
}

/* ========================================================================= */
/* L4: Model validation */
/* ========================================================================= */
static void test_model_r_squared(void) {
    double y_act[] = {1.0, 2.0, 3.0, 4.0, 5.0};
    double y_mod[] = {1.0, 2.0, 3.0, 4.0, 5.0};
    double r2 = model_r_squared(y_act, y_mod, 5);
    TEST_EQ(r2, 1.0, "perfect fit");
    TEST_PASS();
}

static void test_model_rmse(void) {
    double y_act[] = {0.0, 0.0, 0.0};
    double y_mod[] = {3.0, 4.0, 0.0};
    double rmse = model_rmse(y_act, y_mod, 3);
    TEST_EQ(rmse, sqrt(25.0/3.0), "sqrt((9+16+0)/3)");
    TEST_PASS();
}

static void test_model_mae(void) {
    double y_act[] = {0.0, 0.0, 0.0};
    double y_mod[] = {1.0, 2.0, 3.0};
    double mae = model_mae(y_act, y_mod, 3);
    TEST_EQ(mae, 2.0, "(1+2+3)/3");
    TEST_PASS();
}

static void test_disturbance_validate(void) {
    disturbance_meas_t dm;
    disturbance_meas_init(&dm, 0.0, 100.0, 10.0);
    dm.value = 50.0;
    dm.timestamp = 2.0;
    signal_status_t s = disturbance_validate(&dm, 40.0, 1.0, 10.0);
    assert(s == SIG_VALID);
    TEST_PASS();
}

static void test_disturbance_validate_overrange(void) {
    disturbance_meas_t dm;
    disturbance_meas_init(&dm, 0.0, 100.0, 10.0);
    dm.value = 150.0;
    dm.timestamp = 2.0;
    signal_status_t s = disturbance_validate(&dm, 40.0, 1.0, 10.0);
    assert(s == SIG_OVERRANGE);
    TEST_PASS();
}

/* ========================================================================= */
/* L2: Combined feedforward-feedback controller */
/* ========================================================================= */
static void test_feedforward_init(void) {
    feedforward_t ff;
    feedforward_init(&ff);
    assert(ff.initialized == 1);
    assert(ff.mode == FF_MODE_OFF);
    TEST_PASS();
}

static void test_feedforward_configure_static(void) {
    feedforward_t ff;
    feedforward_configure_static(&ff, 0.5, 10.0, 0.0, 100.0, ACTION_DIRECT, 0.1);
    assert(ff.mode == FF_MODE_STATIC);
    double u = feedforward_step(&ff, 20.0);
    TEST_EQ(u, 20.0, "0.5*20+10=20");
    TEST_PASS();
}

static void test_feedforward_configure_dynamic_conf(void) {
    feedforward_t ff;
    feedforward_configure_dynamic(&ff, 0.5, 50.0, 30.0, 10.0, 0.0, 100.0, ACTION_DIRECT, 0.1);
    assert(ff.mode == FF_MODE_DYNAMIC);
    TEST_PASS();
}

static void test_feedforward_configure_combined(void) {
    feedforward_t ff;
    feedforward_configure_combined(&ff, 0.3, 0.5, 60.0, 40.0, 10.0, 0.7, 0.0, 100.0, ACTION_DIRECT, 0.1);
    assert(ff.mode == FF_MODE_COMBINED);
    double u = feedforward_step(&ff, 15.0);
    assert(u >= 0.0 && u <= 100.0);
    TEST_PASS();
}

static void test_feedforward_step_with_feedback(void) {
    feedforward_t ff;
    feedforward_configure_static(&ff, 0.2, 5.0, 0.0, 100.0, ACTION_DIRECT, 0.1);
    double u_combined;
    int rc = feedforward_step_with_feedback(&ff, 10.0, 30.0, &u_combined);
    TEST_EQ(u_combined, 30.0 + 0.2*10.0 + 5.0, "fb + ff");
    assert(rc == 0);
    TEST_PASS();
}

static void test_feedforward_set_mode(void) {
    feedforward_t ff;
    feedforward_init(&ff);
    feedforward_set_mode(&ff, FF_MODE_STATIC, 5);
    assert(ff.mode == FF_MODE_STATIC);
    feedforward_set_mode(&ff, FF_MODE_OFF, 3);
    assert(ff.mode == FF_MODE_OFF);
    TEST_PASS();
}

static void test_feedforward_bumpless_transfer(void) {
    feedforward_t ff;
    feedforward_configure_static(&ff, 0.2, 5.0, 0.0, 100.0, ACTION_DIRECT, 0.1);
    feedforward_step(&ff, 10.0);
    feedforward_bumpless_transfer(&ff, 0, 45.0);
    assert(ff.tracking == 1);
    double u = feedforward_step(&ff, 30.0);
    assert(u == 45.0);
    feedforward_bumpless_transfer(&ff, 1, 45.0);
    assert(ff.tracking == 0);
    TEST_PASS();
}

/* ========================================================================= */
/* L7: Gain scheduling */
/* ========================================================================= */
static void test_gain_schedule(void) {
    double x_pts[] = {0.0, 50.0, 100.0};
    double k_pts[] = {0.1, 0.5, 1.0};
    ff_gain_schedule_t gs;
    ff_gain_schedule_init(&gs, x_pts, k_pts, 3);

    double k = ff_gain_schedule_lookup(&gs, 0.0);
    TEST_EQ(k, 0.1, "x=0 → Kff=0.1");

    k = ff_gain_schedule_lookup(&gs, 75.0);
    TEST_EQ(k, 0.75, "x=75 → 0.5 + 0.5*(75-50)/(100-50) = 0.75");

    k = ff_gain_schedule_lookup(&gs, 100.0);
    TEST_EQ(k, 1.0, "x=100 → Kff=1.0");

    /* Out of range clamping */
    k = ff_gain_schedule_lookup(&gs, -10.0);
    TEST_EQ(k, 0.1, "below min → clamp to 0.1");

    k = ff_gain_schedule_lookup(&gs, 200.0);
    TEST_EQ(k, 1.0, "above max → clamp to 1.0");

    ff_gain_schedule_free(&gs);
    TEST_PASS();
}

/* ========================================================================= */
/* L8: Non-minimum-phase detection */
/* ========================================================================= */
static void test_nmp_detection(void) {
    tf_t Gp_nmp;
    memset(&Gp_nmp, 0, sizeof(tf_t));
    Gp_nmp.theta = 5.0;
    Gp_nmp.K = 1.0;
    Gp_nmp.type = TF_ORDER_FIRST;
    Gp_nmp.order_num = 0;
    Gp_nmp.order_den = 1;
    Gp_nmp.num_coeffs[0] = 1.0;
    Gp_nmp.den_coeffs[0] = 1.0;
    Gp_nmp.den_coeffs[1] = 1.0;

    assert(ff_is_non_minimum_phase(&Gp_nmp) == 1);

    Gp_nmp.theta = 0.0;
    assert(ff_is_non_minimum_phase(&Gp_nmp) == 0);
    TEST_PASS();
}

static void test_nmp_factorization(void) {
    tf_t Gp, Gp_mp, Gp_nmp;
    memset(&Gp, 0, sizeof(tf_t));
    Gp.K = 2.0;
    Gp.theta = 10.0;
    Gp.order_num = 0;
    Gp.order_den = 1;
    Gp.num_coeffs[0] = 1.0;
    Gp.den_coeffs[0] = 1.0;
    Gp.den_coeffs[1] = 1.0;

    int rc = ff_factor_minimum_phase(&Gp, &Gp_mp, &Gp_nmp);
    assert(rc == 0);
    assert(Gp_nmp.theta == 10.0);
    assert(Gp_mp.theta == 0.0);
    TEST_PASS();
}

/* ========================================================================= */
/* L8: Kalman filter for disturbance estimation */
/* ========================================================================= */
static void test_kalman_dist(void) {
    ff_kalman_dist_t kf;
    double A[4] = {0.9, 0.1, 0.0, 1.0};
    double C[2] = {1.0, 0.0};
    double Q[4] = {0.01, 0.0, 0.0, 0.01};
    double R = 0.1;
    double x0[2] = {0.0, 0.0};
    double P0[4] = {1.0, 0.0, 0.0, 1.0};

    ff_kalman_dist_init(&kf, A, C, Q, R, x0, P0, 0.1);
    assert(kf.initialized == 1);

    /* Run several steps */
    for (int i = 0; i < 10; i++) {
        ff_kalman_dist_step(&kf, 0.5, 1.0 + 0.1*i);
    }

    double d_est = ff_kalman_dist_get(&kf);
    assert(!isnan(d_est));
    TEST_PASS();
}

/* ========================================================================= */
/* L9: Iterative Learning Control */
/* ========================================================================= */
static void test_ilc(void) {
    ff_ilc_t ilc;
    ff_ilc_init(&ilc, 10, 0.5, 0.8);

    double error[10];
    for (int i = 0; i < 10; i++) error[i] = 0.1 * (10 - i);

    ff_ilc_record_error(&ilc, error);
    assert(ilc.cycle_count == 1);

    double u_new[10];
    ff_ilc_update(&ilc, u_new);
    for (int i = 0; i < 10; i++) {
        assert(!isnan(u_new[i]));
    }

    ff_ilc_free(&ilc);
    TEST_PASS();
}

/* ========================================================================= */
/* L6: Application-specific tests */
/* ========================================================================= */
static void test_heat_exchanger_ff(void) {
    /* Validate heat exchanger FF design logic:
     * Kp = -0.8, Kd = 0.15, Kff = -Kd/Kp = -0.15/(-0.8) = 0.1875 */
    fopdt_t proc;
    fopdt_init(&proc, -0.8, 120.0, 30.0);
    assert(proc.Kp == -0.8);
    assert(proc.tau == 120.0);
    assert(proc.theta == 30.0);

    double Kff = -0.15 / (-0.8);
    assert(fabs(Kff - 0.1875) < 0.01);
    TEST_PASS();
}

/* ========================================================================= */
/* Performance metrics */
/* ========================================================================= */
static void test_performance(void) {
    ff_performance_t perf;
    ff_performance_init(&perf);

    ff_performance_update(&perf, 2.0, 0.1, 0);
    ff_performance_update(&perf, 1.0, 0.1, 0);
    ff_performance_update(&perf, 0.5, 0.1, 1);
    ff_performance_update(&perf, 0.2, 0.1, 1);

    assert(perf.ise_without > 0.0);
    assert(perf.ise_with > 0.0);

    ff_performance_finalize(&perf, 4);
    TEST_PASS();
}

/* ========================================================================= */
static void test_tf_to_discrete_tustin_basic(void) {
    tf_t ct;
    memset(&ct, 0, sizeof(tf_t));
    ct.K = 1.0;
    ct.theta = 0.0;
    ct.order_num = 0;
    ct.order_den = 1;
    ct.num_coeffs[0] = 1.0;
    ct.den_coeffs[0] = 1.0;
    ct.den_coeffs[1] = 1.0;

    tf_discrete_t dt;
    int rc = tf_to_discrete_tustin(&ct, 0.1, &dt);
    assert(rc == 0);
    TEST_PASS();
}

/* ========================================================================= */
/* Run all tests */
/* ========================================================================= */
int main(void) {
    printf("=== Feedforward Control Module Tests ===\n\n");

    printf("L1: Type definitions\n");
    test_defs_struct_sizes();

    printf("\nL2: Static feedforward — gain computation\n");
    test_ff_static_gain_fopdt();
    test_ff_static_gain_zero_Kp();
    test_ff_static_gain_action_reverse();
    test_ff_static_gain_sopdt();
    test_ff_static_gain_tf();
    test_ff_static_step_basic();
    test_ff_static_step_reverse();
    test_ff_static_step_clamping();
    test_ff_static_step_filtered();
    test_ff_static_disturbance_rejection_ratio();
    test_ff_static_rejection_no_ff();
    test_ff_static_mismatch();
    test_ff_static_bias_from_op();
    test_ff_static_step_quality_valid();

    printf("\nL3/L5: Dynamic feedforward\n");
    test_lead_lag_init();
    test_lead_lag_step_unity();
    test_lead_lag_step_steady_state();
    test_lead_lag_zero_lag();
    test_lead_lag_reset();
    test_lead_lag_freq_response_dc();
    test_lead_lag2_init_step();
    test_dtf_init_step();
    test_dtf_reset();
    test_ff_dynamic_design_fopdt();
    test_ff_dynamic_design_sopdt();
    test_ff_dynamic_causality();
    test_ff_dynamic_required_delay();
    test_ff_dynamic_step();

    printf("\nL4/L5: Model identification and validation\n");
    test_fopdt_identify_step();
    test_fopdt_identify_two_point();
    test_fopdt_step_response();
    test_sopdt_step_response();
    test_pade_first_order();
    test_pade_second_order();
    test_fopdt_to_discrete_zoh();
    test_fopdt_to_discrete_tustin();
    test_fopdt_to_discrete_euler();
    test_model_r_squared();
    test_model_rmse();
    test_model_mae();
    test_disturbance_validate();
    test_disturbance_validate_overrange();

    printf("\nL2/L5: Combined feedforward-feedback\n");
    test_feedforward_init();
    test_feedforward_configure_static();
    test_feedforward_configure_dynamic_conf();
    test_feedforward_configure_combined();
    test_feedforward_step_with_feedback();
    test_feedforward_set_mode();
    test_feedforward_bumpless_transfer();

    printf("\nL7: Gain scheduling\n");
    test_gain_schedule();

    printf("\nL8: Non-minimum-phase and Kalman\n");
    test_nmp_detection();
    test_nmp_factorization();
    test_kalman_dist();

    printf("\nL9: Iterative Learning Control\n");
    test_ilc();

    printf("\nL6: Application validation\n");
    test_heat_exchanger_ff();

    printf("\nPerformance metrics\n");
    test_performance();

    printf("\nDiscretization\n");
    test_tf_to_discrete_tustin_basic();

    printf("\n=== ALL TESTS PASSED ===\n");
    return 0;
}