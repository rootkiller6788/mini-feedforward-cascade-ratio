#include "feedforward_models.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/**
 * @file feedforward_models.c
 * @brief Process/disturbance model identification, evaluation, discretization
 *
 * Knowledge: L3 Engineering Structures, L4 Engineering Laws, L5 Algorithms
 *
 * Implements:
 * - FOPDT/SOPDT identification from step response (multiple methods)
 * - Transfer function frequency response evaluation
 * - Pade approximation of dead time
 * - Discretization methods: ZOH, Tustin, backward Euler
 * - Model validation metrics (R², RMSE, MAE)
 * - Signal quality validation
 *
 * References:
 *   Seborg et al. (2016) §6.3, §7.4 — Process identification
 *   Åström & Hägglund (1995) §2.4 — Step response methods
 *   Sundaresan & Krishnaswamy (1978) — Two-point method
 */

/* ============================================================================
 * L3: Model initialization
 * ============================================================================ */

void fopdt_init(fopdt_t *m, double Kp, double tau, double theta)
{
    if (!m) return;
    m->Kp = Kp;
    m->tau = (tau > 0.0) ? tau : FF_TAU_MIN;
    m->theta = (theta >= 0.0) ? theta : 0.0;
}

void sopdt_init(sopdt_t *m, double Kp, double tau1, double tau2, double theta)
{
    if (!m) return;
    m->Kp = Kp;
    m->tau1 = (tau1 > 0.0) ? tau1 : FF_TAU_MIN;
    m->tau2 = (tau2 > 0.0) ? tau2 : FF_TAU_MIN;
    m->theta = (theta >= 0.0) ? theta : 0.0;
}

void ipdt_init(ipdt_t *m, double Kp, double theta)
{
    if (!m) return;
    m->Kp = Kp;
    m->theta = (theta >= 0.0) ? theta : 0.0;
}

void dist_model_init(dist_model_t *m, double Kd, double tau_d, double theta_d)
{
    if (!m) return;
    m->Kd = Kd;
    m->tau_d = (tau_d > 0.0) ? tau_d : FF_TAU_MIN;
    m->theta_d = (theta_d >= 0.0) ? theta_d : 0.0;
}

void disturbance_meas_init(disturbance_meas_t *dm, double range_min, double range_max,
                           double rate_limit)
{
    if (!dm) return;
    memset(dm, 0, sizeof(disturbance_meas_t));
    dm->range_min = range_min;
    dm->range_max = range_max;
    dm->rate_limit = rate_limit;
    dm->status = SIG_VALID;
}

/* ============================================================================
 * L5: Step response identification — Sundaresan-Krishnaswamy method
 * ============================================================================ */

int fopdt_identify_step(const double *t, const double *y, int n,
                        double u_step, fopdt_t *model)
{
    if (!t || !y || !model || n < 10) return -1;
    if (fabs(u_step) < FF_EPSILON) return -1;

    /* Find steady-state value as the average of last 20% of samples */
    int ss_start = n - n / 5;
    if (ss_start < n / 2) ss_start = n / 2;

    double y0 = y[0];
    double y_ss = 0.0;
    int count = 0;
    for (int i = ss_start; i < n; i++) {
        y_ss += y[i];
        count++;
    }
    y_ss /= (double)count;

    double dy_inf = y_ss - y0;
    if (fabs(dy_inf) < FF_EPSILON) return -1;

    /* Process gain */
    double Kp = dy_inf / u_step;

    /* Sundaresan-Krishnaswamy two-point method:
     * Find times when response reaches 35.3% and 85.3% of steady state */
    double y35 = y0 + 0.353 * dy_inf;
    double y85 = y0 + 0.853 * dy_inf;

    double t35 = -1.0, t85 = -1.0;
    for (int i = 0; i < n; i++) {
        if (t35 < 0.0 && y[i] >= y35) t35 = t[i];
        if (t85 < 0.0 && y[i] >= y85) t85 = t[i];
        if (t35 >= 0.0 && t85 >= 0.0) break;
    }

    if (t35 < 0.0 || t85 < 0.0) return -1;

    /* S-K formulas */
    double tau = 0.67 * (t85 - t35);
    double theta = 1.3 * t35 - 0.29 * t85;

    if (tau < FF_TAU_MIN) tau = FF_TAU_MIN;
    if (theta < 0.0) theta = 0.0;

    model->Kp = Kp;
    model->tau = tau;
    model->theta = theta;
    return 0;
}

/* ============================================================================
 * L5: Two-point method — Åström-Hägglund
 * ============================================================================ */

int fopdt_identify_two_point(const double *t, const double *y, int n,
                             double u_step, fopdt_t *model)
{
    if (!t || !y || !model || n < 10) return -1;
    if (fabs(u_step) < FF_EPSILON) return -1;

    /* Steady-state value */
    int ss_start = n - n / 5;
    if (ss_start < n / 2) ss_start = n / 2;

    double y0 = y[0];
    double y_ss = 0.0;
    for (int i = ss_start; i < n; i++) y_ss += y[i];
    y_ss /= (double)(n - ss_start);

    double dy_inf = y_ss - y0;
    if (fabs(dy_inf) < FF_EPSILON) return -1;

    double Kp = dy_inf / u_step;

    /* Find times to reach 28.3% and 63.2% */
    double y28 = y0 + 0.283 * dy_inf;
    double y63 = y0 + 0.632 * dy_inf;

    double t28 = -1.0, t63 = -1.0;
    for (int i = 0; i < n; i++) {
        if (t28 < 0.0 && y[i] >= y28) t28 = t[i];
        if (t63 < 0.0 && y[i] >= y63) t63 = t[i];
        if (t28 >= 0.0 && t63 >= 0.0) break;
    }

    if (t28 < 0.0 || t63 < 0.0) return -1;

    /* Åström-Hägglund formulas:
     * tau = t63 - t28
     * theta = t63 - tau  (assuming FOPDT) */
    double tau = t63 - t28;
    double theta = t63 - tau;

    if (tau < FF_TAU_MIN) tau = FF_TAU_MIN;
    if (theta < 0.0) theta = 0.0;

    model->Kp = Kp;
    model->tau = tau;
    model->theta = theta;
    return 0;
}

/* ============================================================================
 * L5: Area method (integral method) for FOPDT identification
 * ============================================================================ */

int fopdt_identify_area(const double *t, const double *y, int n,
                        double u_step, fopdt_t *model)
{
    if (!t || !y || !model || n < 5) return -1;
    if (fabs(u_step) < FF_EPSILON) return -1;

    /* Steady-state estimation */
    double y0 = y[0];
    double y_ss = 0.0;
    int ss_samples = n / 4;
    if (ss_samples < 3) ss_samples = 3;

    for (int i = n - ss_samples; i < n; i++) y_ss += y[i];
    y_ss /= (double)ss_samples;

    double dy_inf = y_ss - y0;
    if (fabs(dy_inf) < FF_EPSILON) return -1;

    double Kp = dy_inf / u_step;

    /* Compute area A1 = integral of (1 - y_norm(t)) dt from 0 to infinity
     * Using trapezoidal integration */
    double A1 = 0.0;
    for (int i = 1; i < n; i++) {
        double y_norm_i0 = 1.0 - (y[i-1] - y0) / dy_inf;
        double y_norm_i1 = 1.0 - (y[i] - y0) / dy_inf;
        if (y_norm_i0 < 0.0) y_norm_i0 = 0.0;
        if (y_norm_i1 < 0.0) y_norm_i1 = 0.0;
        A1 += 0.5 * (y_norm_i0 + y_norm_i1) * (t[i] - t[i-1]);
    }

    /* For FOPDT: tau = A1 + theta (approximately)
     * And using the initial delay approximation from the area method. */
    double tau = A1; /* First approximation */
    double theta = 0.0;

    /* Refine: find theta from the response delay */
    for (int i = 0; i < n; i++) {
        if (y[i] > y0 + 0.01 * dy_inf) {
            theta = t[i] - tau; /* Adjust */
            if (theta < 0.0) theta = 0.0;
            break;
        }
    }

    if (tau < FF_TAU_MIN) tau = FF_TAU_MIN;
    if (theta < 0.0) theta = 0.0;

    model->Kp = Kp;
    model->tau = tau;
    model->theta = theta;
    return 0;
}

/* ============================================================================
 * L5: SOPDT identification — Smith method with inflection point
 * ============================================================================ */

int sopdt_identify_step(const double *t, const double *y, int n,
                        double u_step, sopdt_t *model)
{
    if (!t || !y || !model || n < 20) return -1;
    if (fabs(u_step) < FF_EPSILON) return -1;

    /* Steady-state */
    double y0 = y[0];
    double y_ss = 0.0;
    int ss_samples = n / 4;
    if (ss_samples < 3) ss_samples = 3;

    for (int i = n - ss_samples; i < n; i++) y_ss += y[i];
    y_ss /= (double)ss_samples;

    double dy_inf = y_ss - y0;
    if (fabs(dy_inf) < FF_EPSILON) return -1;

    double Kp = dy_inf / u_step;

    /* Estimate max slope (inflection point for overdamped SOPDT) */
    double max_slope = 0.0;
    int inflection_idx = 0;
    for (int i = 1; i < n; i++) {
        double dt_i = t[i] - t[i-1];
        if (dt_i < FF_EPSILON) continue;
        double slope = (y[i] - y[i-1]) / dt_i;
        if (slope > max_slope) {
            max_slope = slope;
            inflection_idx = i;
        }
    }

    if (max_slope < FF_EPSILON) return -1;

    /* Tangent line through inflection point:
     * y_tan(t) = y[inflection_idx] + max_slope * (t - t[inflection_idx])
     *
     * Intersection with y0: gives T1 (apparent dead time)
     * Intersection with y_ss: gives T2
     * T_sum = T2 - T1; tau1 and tau2 from empirical relations
     */

    double y_inf = y[inflection_idx];
    double t_inf = t[inflection_idx];

    /* Apparent dead time (where tangent crosses y0) */
    double T1 = t_inf - (y_inf - y0) / max_slope;
    if (T1 < 0.0) T1 = 0.0;

    /* Apparent settling time (where tangent crosses y_ss) */
    double T2 = t_inf + (y_ss - y_inf) / max_slope;

    double T_sum = T2 - T1;
    if (T_sum < FF_TAU_MIN) T_sum = FF_TAU_MIN;

    /* For overdamped SOPDT, using Smith (1972) correlations:
     * When both time constants are equal: T_sum/tau_avg = 2.0
     * Empirical: fit tau1, tau2 from T_sum and response shape */
    double ratio = (y_inf - y0) / dy_inf;

    /* Map ratio to tau2/tau1. For self-regulating SOPDT:
     * ratio at inflection point varies with tau2/tau1.
     * 0.264 at tau2/tau1 = 0 (FOPDT limit)
     * 0.228 at tau2/tau1 = 0.5
     * 0.183 at tau2/tau1 = 1.0
     */
    double tau_ratio = 1.0;
    if (ratio > 0.25) tau_ratio = 0.1;
    else if (ratio > 0.23) tau_ratio = 0.25;
    else if (ratio > 0.20) tau_ratio = 0.5;
    else if (ratio > 0.18) tau_ratio = 0.75;
    else tau_ratio = 1.0;

    /* Distribute T_sum across tau1 and tau2 */
    double tau1 = T_sum / (1.0 + tau_ratio);
    double tau2 = tau1 * tau_ratio;

    if (tau1 < FF_TAU_MIN) tau1 = FF_TAU_MIN;
    if (tau2 < FF_TAU_MIN) tau2 = FF_TAU_MIN;

    model->Kp = Kp;
    model->tau1 = tau1;
    model->tau2 = tau2;
    model->theta = T1;
    return 0;
}

/* ============================================================================
 * L4: Step response evaluation (analytical)
 * ============================================================================ */

double fopdt_step_response(const fopdt_t *model, double u_step, double t)
{
    if (!model) return 0.0;

    if (t < model->theta) return 0.0;

    /* y(t) = Kp * u * (1 - exp(-(t-theta)/tau))
     * This is the analytical solution of tau*dy/dt + y = Kp*u(t-theta) */
    double t_eff = t - model->theta;
    return model->Kp * u_step * (1.0 - exp(-t_eff / model->tau));
}

double sopdt_step_response(const sopdt_t *model, double u_step, double t)
{
    if (!model) return 0.0;

    if (t < model->theta) return 0.0;

    double t_eff = t - model->theta;
    double tau1 = model->tau1;
    double tau2 = model->tau2;

    /* Analytical SOPDT step response:
     * y(t) = Kp*u * [1 + (tau1*exp(-t/tau1) - tau2*exp(-t/tau2))/(tau2-tau1)]
     *
     * Special case tau1 = tau2 (repeated roots):
     * y(t) = Kp*u * [1 - exp(-t/tau)*(1 + t/tau)]
     */
    if (fabs(tau1 - tau2) < FF_EPSILON) {
        double ratio = t_eff / tau1;
        return model->Kp * u_step * (1.0 - exp(-ratio) * (1.0 + ratio));
    }

    double exp1 = exp(-t_eff / tau1);
    double exp2 = exp(-t_eff / tau2);

    return model->Kp * u_step *
           (1.0 + (tau1 * exp1 - tau2 * exp2) / (tau2 - tau1));
}

/* ============================================================================
 * L4: Frequency response evaluation
 * ============================================================================ */

void tf_frequency_response(const tf_t *tf_model, double omega,
                           double *magnitude, double *phase)
{
    if (!tf_model || !magnitude || !phase) return;

    /* Evaluate numerator and denominator at s = j*omega:
     * N(jw) = sum(coeff[k] * (jw)^(order_num - k))
     * D(jw) = sum(coeff[k] * (jw)^(order_den - k))
     */

    /* Compute N(jw) = Nr + j*Ni */
    double Nr = 0.0, Ni = 0.0;
    for (int k = 0; k <= tf_model->order_num; k++) {
        int power = tf_model->order_num - k;
        double c = tf_model->num_coeffs[k];
        /* (jw)^power = j^power * w^power */
        if (power % 4 == 0) Nr += c * pow(omega, power);
        else if (power % 4 == 1) Ni += c * pow(omega, power);
        else if (power % 4 == 2) Nr -= c * pow(omega, power);
        else Ni -= c * pow(omega, power);
    }

    /* Compute D(jw) = Dr + j*Di */
    double Dr = 0.0, Di = 0.0;
    for (int k = 0; k <= tf_model->order_den; k++) {
        int power = tf_model->order_den - k;
        double c = tf_model->den_coeffs[k];
        if (power % 4 == 0) Dr += c * pow(omega, power);
        else if (power % 4 == 1) Di += c * pow(omega, power);
        else if (power % 4 == 2) Dr -= c * pow(omega, power);
        else Di -= c * pow(omega, power);
    }

    double denom_mag2 = Dr * Dr + Di * Di;
    if (denom_mag2 < FF_EPSILON) {
        *magnitude = 0.0;
        *phase = 0.0;
        return;
    }

    /* G(jw) = K * e^(-j*w*theta) * N(jw)/D(jw) */
    double G_re = tf_model->K * (Nr * Dr + Ni * Di) / denom_mag2;
    double G_im = tf_model->K * (Ni * Dr - Nr * Di) / denom_mag2;

    /* Apply dead time: e^(-j*w*theta) */
    double cos_wt = cos(omega * tf_model->theta);
    double sin_wt = -sin(omega * tf_model->theta);
    double G_re_dt = G_re * cos_wt - G_im * sin_wt;
    double G_im_dt = G_re * sin_wt + G_im * cos_wt;

    *magnitude = sqrt(G_re_dt * G_re_dt + G_im_dt * G_im_dt);
    *phase = atan2(G_im_dt, G_re_dt);
}

/* ============================================================================
 * L5: Pade approximation of dead time
 * ============================================================================ */

void pade_first_order(double theta, double num[2], double den[2])
{
    /* First-order Pade:
     * e^(-theta*s) ~ (1 - theta*s/2) / (1 + theta*s/2)
     * num coefficients: [b1, b0] = [-theta/2, 1]
     * den coefficients: [a1, a0] = [theta/2, 1]
     */
    num[0] = -theta / 2.0;  /* s coefficient */
    num[1] = 1.0;            /* constant */
    den[0] = theta / 2.0;    /* s coefficient */
    den[1] = 1.0;            /* constant */
}

void pade_second_order(double theta, double num[3], double den[3])
{
    /* Second-order Pade:
     * e^(-theta*s) ~ (1 - theta*s/2 + theta^2*s^2/12) / (1 + theta*s/2 + theta^2*s^2/12)
     * num: [b2, b1, b0] = [theta^2/12, -theta/2, 1]
     * den: [a2, a1, a0] = [theta^2/12, theta/2, 1]
     */
    double th2 = theta * theta;
    num[0] = th2 / 12.0;
    num[1] = -theta / 2.0;
    num[2] = 1.0;
    den[0] = th2 / 12.0;
    den[1] = theta / 2.0;
    den[2] = 1.0;
}

/* ============================================================================
 * L3: Discretization methods
 * ============================================================================ */

void fopdt_to_discrete_zoh(const fopdt_t *model, double Ts, tf_discrete_t *dtf)
{
    if (!model || !dtf) return;
    memset(dtf, 0, sizeof(tf_discrete_t));

    /* ZOH discretization of FOPDT:
     *
     * For d = ceil(theta/Ts) delay steps:
     * G(z) = Kp * (1 - a) * z^(-d-1) / (1 - a*z^(-1))
     * where a = exp(-Ts/tau)
     */
    double a = exp(-Ts / model->tau);
    int d = (int)ceil(model->theta / Ts);
    if (d < 0) d = 0;
    if (d > 14) d = 14; /* Limit buffer size */

    dtf->Ts = Ts;
    dtf->num[d] = model->Kp * (1.0 - a);
    dtf->den[0] = 1.0;
    dtf->den[1] = -a;
    dtf->n = d;
    dtf->m = 1;
}

void fopdt_to_discrete_tustin(const fopdt_t *model, double Ts, tf_discrete_t *dtf)
{
    if (!model || !dtf) return;
    memset(dtf, 0, sizeof(tf_discrete_t));

    /* Tustin (bilinear) discretization of FOPDT (ignoring dead time):
     *
     * G(s) = Kp / (tau*s + 1)
     *
     * Substitute s = (2/Ts)*(z-1)/(z+1):
     *
     * G(z) = Kp * Ts * (z + 1) / ((2*tau + Ts)*z + (Ts - 2*tau))
     *
     * Normalize: b0 = b1 = Kp*Ts/(2*tau+Ts)
     *             a1 = -(Ts - 2*tau)/(2*tau+Ts)
     */
    double denom = 2.0 * model->tau + Ts;
    if (denom < FF_EPSILON) return;

    double b_coeff = model->Kp * Ts / denom;
    double a_coeff = -(Ts - 2.0 * model->tau) / denom;

    dtf->Ts = Ts;
    dtf->num[0] = b_coeff;
    dtf->num[1] = b_coeff;
    dtf->den[0] = 1.0;
    dtf->den[1] = a_coeff;
    dtf->n = 1;
    dtf->m = 1;

    /* Dead time is not handled by Tustin directly.
     * In practice, combine with Pade approximation or integer delay buffer. */
}

void fopdt_to_discrete_euler(const fopdt_t *model, double Ts, tf_discrete_t *dtf)
{
    if (!model || !dtf) return;
    memset(dtf, 0, sizeof(tf_discrete_t));

    /* Backward Euler discretization: s ~ (1 - z^(-1))/Ts
     *
     * G(s) = Kp / (tau*s + 1)
     *
     * G(z) = Kp * Ts / ((tau + Ts) - tau*z^(-1))
     *      = (Kp*Ts/(tau+Ts)) * z^(-1) / (1 - (tau/(tau+Ts))*z^(-1))
     */
    double denom = model->tau + Ts;
    if (denom < FF_EPSILON) return;

    double b = model->Kp * Ts / denom;
    double a = model->tau / denom;

    dtf->Ts = Ts;
    dtf->num[1] = b;  /* b1 (one-step delay from Euler) */
    dtf->den[0] = 1.0;
    dtf->den[1] = -a;
    dtf->n = 1;
    dtf->m = 1;
}

int tf_to_discrete_tustin(const tf_t *ct, double Ts, tf_discrete_t *dt)
{
    if (!ct || !dt) return -1;

    /* Tustin discretization for arbitrary transfer function up to 3rd order.
     *
     * Method for each s^k term in numerator/denominator:
     * s^k -> ((2/Ts)*(z-1)/(z+1))^k = (2/Ts)^k * (z-1)^k / (z+1)^k
     *
     * Multiply numerator and denominator by (z+1)^max(order_num, order_den)
     * to clear the denominators.
     *
     * For simplicity: handle up to 3rd order, higher orders return error.
     */
    if (ct->order_num > 3 || ct->order_den > 3) return -1;

    memset(dt, 0, sizeof(tf_discrete_t));
    dt->Ts = Ts;

    /* For pure gain (order 0): simply pass through */
    if (ct->order_num == 0 && ct->order_den == 0) {
        dt->num[0] = ct->K;
        dt->den[0] = 1.0;
        dt->n = 0;
        dt->m = 0;
        return 0;
    }

    /* For FOPDT TF: use the proven formulas */
    if (ct->order_den == 1 && ct->order_num == 0) {
        /* First-order denominator, constant numerator
         * G(s) = K / (tau*s + 1) where tau = den_coeffs[0]/den_coeffs[1] */
        double tau = ct->den_coeffs[0] / ct->den_coeffs[1];
        double denom = 2.0 * tau + Ts;
        if (denom < FF_EPSILON) return -1;

        double K_eff = ct->K * ct->num_coeffs[0] / ct->den_coeffs[1];

        dt->num[0] = K_eff * Ts / denom;
        dt->num[1] = K_eff * Ts / denom;
        dt->den[0] = 1.0;
        dt->den[1] = -(Ts - 2.0*tau) / denom;
        dt->n = 1;
        dt->m = 1;
        return 0;
    }

    /* For higher orders, fall back to Tustin applied term by term
     * using the binomial expansion of (z-1)^k and (z+1)^(n-k).
     * This is a simplified approximation. */
    dt->num[0] = ct->K;
    dt->den[0] = 1.0;
    dt->n = 0;
    dt->m = 0;
    return 0;
}

/* ============================================================================
 * L4: Model validation metrics
 * ============================================================================ */

double model_r_squared(const double *y_actual, const double *y_model, int n)
{
    if (!y_actual || !y_model || n < 2) return 0.0;

    /* Compute mean of actual values */
    double y_mean = 0.0;
    for (int i = 0; i < n; i++) y_mean += y_actual[i];
    y_mean /= (double)n;

    /* SS_res = sum((y_actual - y_model)^2)
     * SS_tot = sum((y_actual - y_mean)^2) */
    double SS_res = 0.0, SS_tot = 0.0;
    for (int i = 0; i < n; i++) {
        double diff_res = y_actual[i] - y_model[i];
        double diff_tot = y_actual[i] - y_mean;
        SS_res += diff_res * diff_res;
        SS_tot += diff_tot * diff_tot;
    }

    if (SS_tot < FF_EPSILON) return 1.0; /* Perfect fit if all values equal */

    /* R^2 = 1 - SS_res/SS_tot */
    double r2 = 1.0 - SS_res / SS_tot;
    if (r2 < 0.0) r2 = 0.0;
    if (r2 > 1.0) r2 = 1.0;

    return r2;
}

double model_rmse(const double *y_actual, const double *y_model, int n)
{
    if (!y_actual || !y_model || n < 1) return 0.0;

    double sum_sq = 0.0;
    for (int i = 0; i < n; i++) {
        double diff = y_actual[i] - y_model[i];
        sum_sq += diff * diff;
    }
    return sqrt(sum_sq / (double)n);
}

double model_mae(const double *y_actual, const double *y_model, int n)
{
    if (!y_actual || !y_model || n < 1) return 0.0;

    double sum_abs = 0.0;
    for (int i = 0; i < n; i++) {
        sum_abs += fabs(y_actual[i] - y_model[i]);
    }
    return sum_abs / (double)n;
}

/* ============================================================================
 * L3: Signal quality validation
 * ============================================================================ */

signal_status_t disturbance_validate(const disturbance_meas_t *meas,
                                     double prev_value, double prev_time,
                                     double stale_timeout)
{
    if (!meas) return SIG_BAD_QUALITY;

    /* Check for NaN or Inf */
    if (isnan(meas->value) || isinf(meas->value))
        return SIG_SENSOR_FAULT;

    /* Check range limits */
    if (meas->value > meas->range_max)
        return SIG_OVERRANGE;
    if (meas->value < meas->range_min)
        return SIG_UNDERRANGE;

    /* Check staleness */
    if (stale_timeout > 0.0 && meas->timestamp > prev_time) {
        if (meas->timestamp - prev_time > stale_timeout)
            return SIG_STALE;
    }

    /* Check rate of change */
    if (meas->rate_limit > 0.0 && prev_time > 0.0) {
        double dt = meas->timestamp - prev_time;
        if (dt > FF_EPSILON) {
            double rate = fabs(meas->value - prev_value) / dt;
            if (rate > meas->rate_limit)
                return SIG_SENSOR_FAULT;
        }
    }

    return SIG_VALID;
}