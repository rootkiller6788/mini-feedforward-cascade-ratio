#include "feedforward_static.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/**
 * @file feedforward_static.c
 * @brief Static (steady-state) feedforward control implementation
 *
 * Knowledge: L1 Definitions, L2 Core Concepts, L5 Algorithms/Methods
 *
 * Implements the core static feedforward control law with signal validation,
 * filtering, and operating point management.
 *
 * Key equations:
 *   u_ff = Kff * d(t) + bias                    [static FF law]
 *   Kff_ideal = -Kd / Kp                          [ideal static FF gain]
 *   DRR = 1 / |1 + Kp*Kff/Kd|                    [disturbance rejection ratio]
 *
 * References:
 *   Seborg, Edgar, Mellichamp (2016) "Process Dynamics and Control" §15.3
 *   Åström & Hägglund (1995) "PID Controllers" §7.4
 */

/* ============================================================================
 * L5: Static feedforward gain computation
 * ============================================================================ */

double ff_static_gain_fopdt(const fopdt_t *process, const dist_model_t *dist,
                            action_t action)
{
    if (!process || !dist) return 0.0;
    if (fabs(process->Kp) < FF_EPSILON) return 0.0;

    /* Ideal static feedforward gain: Kff = -Kd / Kp
     * The negative sign ensures that the feedforward action opposes
     * the disturbance effect. The action direction sign is applied separately. */
    double Kff = -dist->Kd / process->Kp;

    /* Apply controller action direction */
    Kff *= (double)action;

    return Kff;
}

double ff_static_gain_sopdt(const sopdt_t *process, const dist_model_t *dist,
                            action_t action)
{
    if (!process || !dist) return 0.0;
    if (fabs(process->Kp) < FF_EPSILON) return 0.0;

    /* SOPDT steady-state gain is the same as FOPDT: Kp (at s=0)
     * So the ideal static FF gain formula is identical. */
    double Kff = -dist->Kd / process->Kp;
    Kff *= (double)action;

    return Kff;
}

double ff_static_gain_tf(const tf_t *Gp, const tf_t *Gd, action_t action)
{
    if (!Gp || !Gd) return 0.0;

    /* Evaluate DC gain: G(0) = K * N(0)/D(0)
     * N(0) = last numerator coefficient (constant term)
     * D(0) = last denominator coefficient (constant term) */
    double Np0 = (Gp->order_num >= 0) ? Gp->num_coeffs[Gp->order_num] : 1.0;
    double Dp0 = (Gp->order_den >= 0) ? Gp->den_coeffs[Gp->order_den] : 1.0;
    double Nd0 = (Gd->order_num >= 0) ? Gd->num_coeffs[Gd->order_num] : 1.0;
    double Dd0 = (Gd->order_den >= 0) ? Gd->den_coeffs[Gd->order_den] : 1.0;

    if (fabs(Dp0) < FF_EPSILON || fabs(Dd0) < FF_EPSILON)
        return 0.0;

    double Kp_dc = Gp->K * Np0 / Dp0;
    double Kd_dc = Gd->K * Nd0 / Dd0;

    if (fabs(Kp_dc) < FF_EPSILON) return 0.0;

    double Kff = -Kd_dc / Kp_dc;
    Kff *= (double)action;

    return Kff;
}

/* ============================================================================
 * L2: Static feedforward initialization and execution
 * ============================================================================ */

void ff_static_init(feedforward_t *ff, double Kff, double bias,
                    double out_min, double out_max, action_t action, double Ts)
{
    if (!ff) return;

    memset(ff, 0, sizeof(feedforward_t));
    ff->Kff = Kff;
    ff->bias = bias;
    ff->output_min = out_min;
    ff->output_max = out_max;
    ff->action = action;
    ff->Ts = Ts;
    ff->mode = FF_MODE_STATIC;
    ff->clamping = 1;
    ff->initialized = 1;
}

double ff_static_step(feedforward_t *ff, double d_meas)
{
    if (!ff || !ff->initialized) return 0.0;
    if (ff->mode == FF_MODE_OFF) return ff->bias;

    /* Core static feedforward law:
     * u_ff = (action) * Kff * d(t) + bias */
    double u_ff = (double)ff->action * ff->Kff * d_meas + ff->bias;

    /* Output clamping */
    if (ff->clamping) {
        if (u_ff > ff->output_max) u_ff = ff->output_max;
        if (u_ff < ff->output_min) u_ff = ff->output_min;
    }

    ff->d_meas = d_meas;
    ff->u_ff_static = u_ff;
    ff->u_ff_total = u_ff;

    return u_ff;
}

double ff_static_step_filtered(feedforward_t *ff, double d_meas, double tau_filter)
{
    if (!ff || !ff->initialized) return 0.0;
    if (tau_filter < FF_TAU_MIN) tau_filter = FF_TAU_MIN;

    /* First-order exponential filter:
     * alpha = Ts / (tau_filter + Ts)
     * d_f[k] = alpha * d_meas + (1 - alpha) * d_f[k-1] */
    double alpha = ff->Ts / (tau_filter + ff->Ts);
    if (alpha > 1.0) alpha = 1.0;

    if (!ff->d_filtered && ff->d_prev == 0.0) {
        /* First sample: initialize filter to measurement */
        ff->d_filtered = d_meas;
    } else {
        ff->d_filtered = alpha * d_meas + (1.0 - alpha) * ff->d_filtered;
    }

    ff->d_prev = d_meas;

    /* Apply static FF law to filtered measurement */
    double u_ff = (double)ff->action * ff->Kff * ff->d_filtered + ff->bias;

    if (ff->clamping) {
        if (u_ff > ff->output_max) u_ff = ff->output_max;
        if (u_ff < ff->output_min) u_ff = ff->output_min;
    }

    ff->u_ff_static = u_ff;
    ff->u_ff_total = u_ff;

    return u_ff;
}

double ff_static_step_quality(feedforward_t *ff, const disturbance_meas_t *meas)
{
    if (!ff || !ff->initialized || !meas) return ff->bias;

    /* Validate disturbance measurement quality */
    if (meas->status != SIG_VALID) {
        /* Bad signal: freezes output at last valid value,
         * or falls back to bias-only if no history */
        if (ff->d_meas != 0.0) {
            return ff->u_ff_static;
        }
        return ff->bias;
    }

    /* Check measurement range */
    if (meas->value > meas->range_max || meas->value < meas->range_min) {
        /* Out of range: clamp measurement and proceed with clamped value */
        double clamped_val = meas->value;
        if (clamped_val > meas->range_max) clamped_val = meas->range_max;
        if (clamped_val < meas->range_min) clamped_val = meas->range_min;
        return ff_static_step(ff, clamped_val);
    }

    /* Valid measurement with optional rate limiting */
    double d_val = meas->value;
    if (meas->rate_limit > 0.0 && ff->d_prev != 0.0) {
        double max_change = meas->rate_limit * ff->Ts;
        double delta = d_val - ff->d_prev;
        if (delta > max_change) d_val = ff->d_prev + max_change;
        if (delta < -max_change) d_val = ff->d_prev - max_change;
    }

    return ff_static_step(ff, d_val);
}

/* ============================================================================
 * L2: Performance analysis
 * ============================================================================ */

double ff_static_rejection_ratio(double Kp, double Kd, double Kff)
{
    /* Disturbance Rejection Ratio (DRR):
     *
     * Without FF: PV_dist = Kd * d  (open-loop disturbance effect)
     * With FF:    PV_dist = Kd * d + Kp * Kff * d = (Kd + Kp*Kff) * d
     *
     * DRR = |without| / |with| = |Kd| / |Kd + Kp*Kff|
     *
     * For ideal FF (Kff = -Kd/Kp): denominator = 0 → infinite DRR
     * For no FF (Kff = 0):        DRR = 1
     */
    double numerator = fabs(Kd);
    double denominator = fabs(Kd + Kp * Kff);

    if (denominator < FF_EPSILON) {
        /* Near-perfect rejection */
        return 1.0 / FF_EPSILON;
    }

    return numerator / denominator;
}

double ff_static_mismatch_residual(double Kp, double Kd, double Kff_actual)
{
    /* Relative residual steady-state error due to gain mismatch:
     *
     * Ideal Kff = -Kd/Kp
     * Actual Kff = Kff_actual
     *
     * Without FF: e_ss = Kd * d_step
     * With FF:    e_ss = (Kd + Kp*Kff_actual) * d_step
     *
     * Relative residual = e_with / e_without = |Kd + Kp*Kff_actual| / |Kd|
     *
     * Value ranges:
     *   0.0 = perfect rejection (Kff_actual = -Kd/Kp)
     *   1.0 = no improvement (Kff_actual = 0)
     *   >1.0 = FF makes things worse (wrong sign)
     */
    if (fabs(Kd) < FF_EPSILON) return 1.0;

    double residual = fabs(Kd + Kp * Kff_actual) / fabs(Kd);
    return residual;
}

double ff_static_bias_from_operating_point(double u0, double Kff, double d0)
{
    /* Bias ensures correct total output at the design operating point:
     * At design point: u0 = u_ff + u_fb
     * Assuming steady state with PV=sp, u_fb ≈ 0:
     *   u_ff = Kff * d0 + bias
     *   bias = u0 - Kff * d0
     */
    return u0 - Kff * d0;
}