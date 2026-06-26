#include "feedforward_advanced.h"
#include "feedforward_static.h"
#include "feedforward_dynamic.h"
#include "feedforward_combined.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/**
 * @file feedforward_advanced.c
 * @brief Advanced feedforward topics — gain scheduling, NMP, Kalman, ILC
 *
 * Knowledge: L7 Industrial Applications, L8 Advanced Topics, L9 Industry Frontiers
 *
 * Covers advanced feedforward implementations used in industry:
 * - Gain-scheduled feedforward for nonlinear processes (L7)
 * - Non-minimum-phase feedforward with factorization (L8)
 * - Feedforward with actuator rate/range constraints (L7)
 * - Kalman-based disturbance estimation (L8)
 * - Iterative learning control for batch processes (L9)
 *
 * References:
 *   Åström & Wittenmark (2008) "Adaptive Control" §3 — Gain scheduling
 *   Kalman (1960) "A New Approach to Linear Filtering"
 *   Bristow, Tharayil, Alleyne (2006) "A Survey of Iterative Learning Control"
 *   Skogestad & Postlethwaite (2005) §5 — NMP factorization
 */

/* ============================================================================
 * L7: Gain-scheduled feedforward
 * ============================================================================ */

void ff_gain_schedule_init(ff_gain_schedule_t *gs, const double *x,
                           const double *Kff, int n_points)
{
    if (!gs || !x || !Kff || n_points < 2) return;
    memset(gs, 0, sizeof(ff_gain_schedule_t));

    gs->n_points = n_points;
    gs->schedule_x = (double *)malloc((size_t)n_points * sizeof(double));
    gs->schedule_Kff = (double *)malloc((size_t)n_points * sizeof(double));

    if (!gs->schedule_x || !gs->schedule_Kff) {
        free(gs->schedule_x);
        free(gs->schedule_Kff);
        gs->schedule_x = NULL;
        gs->schedule_Kff = NULL;
        return;
    }

    for (int i = 0; i < n_points; i++) {
        gs->schedule_x[i] = x[i];
        gs->schedule_Kff[i] = Kff[i];
    }

    gs->x_min = x[0];
    gs->x_max = x[n_points - 1];
}

double ff_gain_schedule_lookup(const ff_gain_schedule_t *gs, double x)
{
    if (!gs || !gs->schedule_x || !gs->schedule_Kff || gs->n_points < 2)
        return 0.0;

    /* Clamp to schedule range */
    if (x <= gs->x_min) return gs->schedule_Kff[0];
    if (x >= gs->x_max) return gs->schedule_Kff[gs->n_points - 1];

    /* Binary search for interval */
    int lo = 0, hi = gs->n_points - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (gs->schedule_x[mid] <= x) lo = mid;
        else hi = mid;
    }

    /* Linear interpolation between lo and hi */
    double x_lo = gs->schedule_x[lo];
    double x_hi = gs->schedule_x[hi];
    double Kff_lo = gs->schedule_Kff[lo];
    double Kff_hi = gs->schedule_Kff[hi];

    if (fabs(x_hi - x_lo) < FF_EPSILON) return Kff_lo;

    double fraction = (x - x_lo) / (x_hi - x_lo);
    return Kff_lo + fraction * (Kff_hi - Kff_lo);
}

void ff_gain_schedule_free(ff_gain_schedule_t *gs)
{
    if (!gs) return;
    free(gs->schedule_x);
    free(gs->schedule_Kff);
    gs->schedule_x = NULL;
    gs->schedule_Kff = NULL;
    gs->n_points = 0;
}

/* ============================================================================
 * L8: Non-minimum-phase detection and factorization
 * ============================================================================ */

int ff_is_non_minimum_phase(const tf_t *Gp)
{
    if (!Gp) return 0;

    /* NMP detection criteria:
     * 1. Dead time > 0: always NMP (infinite-dimensional non-minimum-phase)
     * 2. Numerator has RHP zeros (positive real roots):
     *    For first-order num: a*s + 1 = 0 → s = -1/a
     *    RHP = positive real part → a < 0 (negative coefficient)
     * 3. Leading numerator coefficient sign is negative (RHP zero)
     */

    if (Gp->theta > FF_EPSILON) return 1;

    /* Check numerator coefficients for sign pattern indicating RHP zeros.
     * For first-order: a*s + 1, RHP zero if a < 0. */
    if (Gp->order_num >= 1) {
        for (int i = 0; i < Gp->order_num; i++) {
            if (Gp->num_coeffs[i] < -FF_EPSILON) return 1;
        }
    }

    return 0;
}

int ff_factor_minimum_phase(const tf_t *Gp, tf_t *Gp_mp, tf_t *Gp_nmp)
{
    if (!Gp || !Gp_mp || !Gp_nmp) return -1;

    /* Factor Gp(s) = Gp_mp(s) * Gp_nmp(s)
     *
     * Gp_nmp contains:
     * - Dead time (all-pass in magnitude, phase lag)
     * - RHP zeros (mirrored to LHP)
     *
     * Gp_mp contains:
     * - All LHP poles and zeros (invertible part)
     */

    memset(Gp_mp, 0, sizeof(tf_t));
    memset(Gp_nmp, 0, sizeof(tf_t));

    /* NMP factor: dead time part */
    Gp_nmp->K = 1.0;
    Gp_nmp->theta = Gp->theta;
    Gp_nmp->order_num = 0;
    Gp_nmp->order_den = 0;
    Gp_nmp->num_coeffs[0] = 1.0;
    Gp_nmp->den_coeffs[0] = 1.0;
    Gp_nmp->type = TF_ORDER_ZERO;

    /* MP factor: everything else */
    Gp_mp->K = Gp->K;
    Gp_mp->theta = 0.0; /* Dead time goes to NMP factor */
    Gp_mp->order_num = Gp->order_num;
    Gp_mp->order_den = Gp->order_den;
    Gp_mp->type = Gp->type;

    for (int i = 0; i <= Gp->order_num && i < 8; i++)
        Gp_mp->num_coeffs[i] = Gp->num_coeffs[i];
    for (int i = 0; i <= Gp->order_den && i < 8; i++)
        Gp_mp->den_coeffs[i] = Gp->den_coeffs[i];

    return 0;
}

/* ============================================================================
 * L7: Feedforward with actuator limits
 * ============================================================================ */

int feedforward_with_limits(feedforward_t *ff, double d_meas, double u_fb,
                            double rate_limit, double sat_upper, double sat_lower,
                            double *u_out, double *ff_unused)
{
    if (!ff || !u_out || !ff_unused) return -1;

    /* Compute ideal feedforward */
    double u_ff_ideal = feedforward_step(ff, d_meas);

    /* Compute ideal combined output */
    double u_ideal = u_fb + u_ff_ideal;

    /* Apply rate limit */
    static double u_prev = 0.0;
    static int first_call = 1;
    if (first_call) { u_prev = u_ideal; first_call = 0; }

    double max_change = rate_limit * ff->Ts;
    double u_rate_limited = u_ideal;
    double delta = u_ideal - u_prev;
    if (delta > max_change) u_rate_limited = u_prev + max_change;
    if (delta < -max_change) u_rate_limited = u_prev - max_change;

    /* Apply saturation */
    double u_saturated = u_rate_limited;
    if (u_saturated > sat_upper) u_saturated = sat_upper;
    if (u_saturated < sat_lower) u_saturated = sat_lower;

    /* Compute how much FF was actually applied */
    double ff_applied = u_saturated - u_fb;
    if (ff_applied > u_ff_ideal) ff_applied = u_ff_ideal;
    if (ff_applied < 0.0 && u_ff_ideal > 0.0) ff_applied = 0.0;
    if (ff_applied > 0.0 && u_ff_ideal < 0.0) ff_applied = 0.0;

    *ff_unused = u_ff_ideal - ff_applied;
    *u_out = u_saturated;
    u_prev = u_saturated;

    return (fabs(*ff_unused) > FF_EPSILON) ? -1 : 0;
}

/* ============================================================================
 * L8: Kalman filter for disturbance estimation
 * ============================================================================ */

void ff_kalman_dist_init(ff_kalman_dist_t *kf, const double A_model[4],
                         const double C_model[2], const double Q_noise[4],
                         double R_noise, const double x0[2],
                         const double P0[4], double Ts)
{
    if (!kf) return;
    memset(kf, 0, sizeof(ff_kalman_dist_t));

    for (int i = 0; i < 4; i++) {
        kf->A[i] = A_model[i];
        kf->Q[i] = Q_noise[i];
        kf->P[i] = P0[i];
    }

    kf->C[0] = C_model[0];
    kf->C[1] = C_model[1];
    kf->R = R_noise;
    kf->x_hat[0] = x0[0];
    kf->x_hat[1] = x0[1];
    kf->Ts = Ts;
    kf->initialized = 1;
}

void ff_kalman_dist_step(ff_kalman_dist_t *kf, double u, double y)
{
    (void)u; /* Control input reserved for future B-matrix coupling */
    if (!kf || !kf->initialized) return;

    /* Kalman filter for 2-state system:
     * State: x = [PV, disturbance]^T
     * Model: x[k+1] = A*x[k] + B*u[k] + w  (w ~ N(0,Q))
     * Measure: y[k] = C*x[k] + v           (v ~ N(0,R))
     *
     * Step 1: Predict
     *   x_hat_prior = A * x_hat
     *   P_prior = A * P * A^T + Q
     *
     * Step 2: Update
     *   K = P_prior * C^T / (C * P_prior * C^T + R)
     *   x_hat = x_hat_prior + K * (y - C * x_hat_prior)
     *   P = (I - K * C) * P_prior
     */

    double A00 = kf->A[0], A01 = kf->A[1];
    double A10 = kf->A[2], A11 = kf->A[3];

    /* Predict state */
    double x_pred0 = A00 * kf->x_hat[0] + A01 * kf->x_hat[1];
    double x_pred1 = A10 * kf->x_hat[0] + A11 * kf->x_hat[1];

    /* Predict covariance: P_pred = A * P * A^T + Q
     * P is 2x2 row-major: [P[0], P[1]; P[2], P[3]]
     * A*P*A^T computation for 2x2:
     * temp = A*P:
     *   T00 = A00*P[0] + A01*P[2]
     *   T01 = A00*P[1] + A01*P[3]
     *   T10 = A10*P[0] + A11*P[2]
     *   T11 = A10*P[1] + A11*P[3]
     * P_pred = temp * A^T + Q:
     *   Pp0 = T00*A00 + T01*A01 + Q[0]
     *   Pp1 = T00*A10 + T01*A11 + Q[1]
     *   Pp2 = T10*A00 + T11*A01 + Q[2]
     *   Pp3 = T10*A10 + T11*A11 + Q[3]
     */
    double T00 = A00 * kf->P[0] + A01 * kf->P[2];
    double T01 = A00 * kf->P[1] + A01 * kf->P[3];
    double T10 = A10 * kf->P[0] + A11 * kf->P[2];
    double T11 = A10 * kf->P[1] + A11 * kf->P[3];

    double Pp0 = T00 * A00 + T01 * A01 + kf->Q[0];
    double Pp1 = T00 * A10 + T01 * A11 + kf->Q[1];
    double Pp2 = T10 * A00 + T11 * A01 + kf->Q[2];
    double Pp3 = T10 * A10 + T11 * A11 + kf->Q[3];

    /* Innovation covariance: S = C*P_pred*C^T + R
     * = C[0]^2*Pp0 + 2*C[0]*C[1]*Pp1 + C[1]^2*Pp3 + R */
    double S = kf->C[0] * kf->C[0] * Pp0
             + 2.0 * kf->C[0] * kf->C[1] * Pp1
             + kf->C[1] * kf->C[1] * Pp3
             + kf->R;

    if (S < FF_EPSILON) {
        /* Covariance collapsed: skip update, keep prediction */
        kf->x_hat[0] = x_pred0;
        kf->x_hat[1] = x_pred1;
        kf->P[0] = Pp0; kf->P[1] = Pp1;
        kf->P[2] = Pp2; kf->P[3] = Pp3;
        return;
    }

    /* Kalman gain: K = P_pred * C^T / S
     * K[0] = (Pp0*C[0] + Pp1*C[1]) / S
     * K[1] = (Pp2*C[0] + Pp3*C[1]) / S */
    kf->K[0] = (Pp0 * kf->C[0] + Pp1 * kf->C[1]) / S;
    kf->K[1] = (Pp2 * kf->C[0] + Pp3 * kf->C[1]) / S;

    /* Innovation: y - C*x_pred */
    double innov = y - (kf->C[0] * x_pred0 + kf->C[1] * x_pred1);

    /* Update state */
    kf->x_hat[0] = x_pred0 + kf->K[0] * innov;
    kf->x_hat[1] = x_pred1 + kf->K[1] * innov;

    /* Update covariance: P = (I - K*C) * P_pred
     * I - K*C = [[1-K0*C0, -K0*C1], [-K1*C0, 1-K1*C1]]
     *
     * P_new = (I-KC)*P_pred:
     *   M00 = 1 - K[0]*C[0];  M01 = -K[0]*C[1]
     *   M10 = -K[1]*C[0];     M11 = 1 - K[1]*C[1]
     *   P[0] = M00*Pp0 + M01*Pp2
     *   P[1] = M00*Pp1 + M01*Pp3
     *   P[2] = M10*Pp0 + M11*Pp2
     *   P[3] = M10*Pp1 + M11*Pp3
     */
    double M00 = 1.0 - kf->K[0] * kf->C[0];
    double M01 = -kf->K[0] * kf->C[1];
    double M10 = -kf->K[1] * kf->C[0];
    double M11 = 1.0 - kf->K[1] * kf->C[1];

    kf->P[0] = M00 * Pp0 + M01 * Pp2;
    kf->P[1] = M00 * Pp1 + M01 * Pp3;
    kf->P[2] = M10 * Pp0 + M11 * Pp2;
    kf->P[3] = M10 * Pp1 + M11 * Pp3;
}

double ff_kalman_dist_get(const ff_kalman_dist_t *kf)
{
    if (!kf || !kf->initialized) return 0.0;
    return kf->x_hat[1]; /* Disturbance is second state */
}

/* ============================================================================
 * L9: Iterative Learning Control for batch processes
 * ============================================================================ */

void ff_ilc_init(ff_ilc_t *ilc, int n_samples, double gamma, double q_filter)
{
    if (!ilc || n_samples < 2) return;
    memset(ilc, 0, sizeof(ff_ilc_t));

    ilc->n_samples = n_samples;
    ilc->gamma = gamma;
    ilc->q_filter = q_filter;
    ilc->cycle_count = 0;

    ilc->u_ff_cycle = (double *)calloc((size_t)n_samples, sizeof(double));
    ilc->e_prev_cycle = (double *)calloc((size_t)n_samples, sizeof(double));

    if (!ilc->u_ff_cycle || !ilc->e_prev_cycle) {
        free(ilc->u_ff_cycle);
        free(ilc->e_prev_cycle);
        ilc->u_ff_cycle = NULL;
        ilc->e_prev_cycle = NULL;
    }
}

void ff_ilc_record_error(ff_ilc_t *ilc, const double *error)
{
    if (!ilc || !ilc->e_prev_cycle || !error) return;

    /* Store error profile and compute RMS */
    double sum_sq = 0.0;
    for (int i = 0; i < ilc->n_samples; i++) {
        ilc->e_prev_cycle[i] = error[i];
        sum_sq += error[i] * error[i];
    }

    ilc->e_rms_prev = ilc->e_rms;
    ilc->e_rms = sqrt(sum_sq / (double)ilc->n_samples);
    ilc->cycle_count++;
}

void ff_ilc_update(ff_ilc_t *ilc, double *u_ff_new)
{
    if (!ilc || !ilc->u_ff_cycle || !ilc->e_prev_cycle || !u_ff_new)
        return;

    /* P-type ILC update law with Q-filter:
     *
     * u_new[i] = Q(q) * (u_old[i] + gamma * e[i+1])
     *
     * where:
     *   gamma = learning rate (0 < gamma <= 1)
     *   Q(q) = low-pass filter for robustness (smooths the update)
     *   e[i+1] = error at the next sample (advance by one to account
     *            for one-step causality delay)
     *
     * Q-filter is a first-order FIR: y[i] = q_filter*y[i] + (1-q_filter)*x[i]
     * Or more commonly: y[i] = alpha*x[i] + (1-alpha)*x[i-1]
     * where alpha = q_filter determines bandwidth.
     */

    double *raw_update = (double *)malloc((size_t)ilc->n_samples * sizeof(double));
    if (!raw_update) return;

    /* Compute raw update */
    for (int i = 0; i < ilc->n_samples - 1; i++) {
        raw_update[i] = ilc->u_ff_cycle[i] + ilc->gamma * ilc->e_prev_cycle[i + 1];
    }
    /* Last sample: use same error (no future sample) */
    raw_update[ilc->n_samples - 1] = ilc->u_ff_cycle[ilc->n_samples - 1]
                                   + ilc->gamma * ilc->e_prev_cycle[ilc->n_samples - 1];

    /* Apply Q-filter (first-order low-pass):
     * y[i] = alpha * x[i] + (1 - alpha) * y[i-1]
     * where alpha = q_filter */
    double alpha = ilc->q_filter;
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;

    double y_prev = raw_update[0];
    u_ff_new[0] = alpha * raw_update[0] + (1.0 - alpha) * ilc->u_ff_cycle[0];

    for (int i = 1; i < ilc->n_samples; i++) {
        y_prev = alpha * raw_update[i] + (1.0 - alpha) * y_prev;
        u_ff_new[i] = y_prev;
    }

    /* Update stored profile for next cycle */
    for (int i = 0; i < ilc->n_samples; i++) {
        ilc->u_ff_cycle[i] = u_ff_new[i];
    }

    free(raw_update);
}

void ff_ilc_free(ff_ilc_t *ilc)
{
    if (!ilc) return;
    free(ilc->u_ff_cycle);
    free(ilc->e_prev_cycle);
    ilc->u_ff_cycle = NULL;
    ilc->e_prev_cycle = NULL;
    ilc->n_samples = 0;
}