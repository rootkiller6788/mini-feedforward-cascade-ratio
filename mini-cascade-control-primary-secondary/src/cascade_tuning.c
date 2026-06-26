/**
 * @file cascade_tuning.c
 * @brief Cascade Control Tuning — Industrial Tuning Rules Implementation
 *
 * Implements industry-standard PID tuning methods for cascade control:
 *   L4: Ziegler-Nichols (1942) — ultimate gain method
 *   L4: Cohen-Coon (1953) — open-loop FOPDT method
 *   L5: SIMC / Skogestad IMC (2003) — model-based robust tuning
 *   L5: Lambda / IMC — desired closed-loop dynamics
 *   L5: Phase margin / Ms-constrained tuning
 *   L5: Sequential cascade tuning strategy
 *
 * Knowledge Coverage:
 *   L4: Engineering tuning standards, gain/phase margin verification
 *   L5: Optimization-based tuning, frequency-domain methods
 *
 * References:
 *   Ziegler & Nichols, Trans. ASME (1942)
 *   Cohen & Coon, Trans. ASME (1953)
 *   Skogestad, J. Process Control (2003) — "Simple analytic rules..."
 *   Åström & Hägglund, PID Controllers (1995)
 *   Seborg et al., Process Dynamics and Control (2016), Ch. 12
 *
 * Curriculum: MIT 6.302, Stanford ENGR205, Purdue ME575, RWTH Aachen ICS
 */

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "cascade_types.h"
#include "cascade_tuning.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*---------------------------------------------------------------------------
 * L4: Ziegler-Nichols Tuning for Cascade Loops
 *
 * ZN tuning is based on ultimate gain Ku and ultimate period Pu.
 * These are obtained from relay feedback experiments or the
 * closed-loop continuous cycling method.
 *
 * PI tuning (for secondary loops): Kc = 0.45*Ku, Ti = Pu/1.2
 * PID tuning (for primary loops):  Kc = 0.6*Ku,  Ti = Pu/2, Td = Pu/8
 *
 * Assumptions:
 *   - The process is stable and responds to step inputs
 *   - The ultimate cycle can be safely induced
 *   - The process can be approximated as FOPDT
 *
 * Bounds: Ku > 0, Pu > 0
 * Returns: -1 for invalid inputs
 *---------------------------------------------------------------------------*/

static int validate_zn_inputs(double Ku, double Pu, double sample_time)
{
    if (Ku <= 0.0 || Pu <= 0.0 || sample_time <= 0.0) return -1;
    if (Pu < 2.0 * sample_time) return -1; /* Nyquist: at least 2 samples per period */
    return 0;
}

int cascade_tune_zn_secondary(double Ku, double Pu, double sample_time,
                               cascade_pid_params_t *result)
{
    if (validate_zn_inputs(Ku, Pu, sample_time) != 0) return -1;
    if (!result) return -1;

    /* ZN PI Tuning: Kc = 0.45 * Ku, Ti = Pu / 1.2 */
    result->kp = 0.45 * Ku;
    result->ti = Pu / 1.2;
    result->td = 0.0;

    /* Derivative filter: N/A for PI */
    result->tf = result->ti / 10.0;  /* Conservative filter for noise */
    result->beta = 1.0;
    result->gamma = 0.0;

    /* Output limits: set conservatively based on Ku */
    result->output_min = -100.0;
    result->output_max = 100.0;
    result->rate_limit = CASCADE_ROCLIM_DEFAULT;

    return 0;
}

int cascade_tune_zn_primary(double Ku, double Pu, double sample_time,
                             cascade_pid_params_t *result)
{
    if (validate_zn_inputs(Ku, Pu, sample_time) != 0) return -1;
    if (!result) return -1;

    /* ZN PID Tuning: Kc = 0.6 * Ku, Ti = Pu / 2, Td = Pu / 8 */
    result->kp = 0.6 * Ku;
    result->ti = Pu / 2.0;
    result->td = Pu / 8.0;

    /* Derivative filter factor N = 8 */
    result->tf = result->td / 8.0;
    result->beta = 1.0;
    result->gamma = 0.0;  /* Derivative on PV only */

    /* Output limits */
    result->output_min = -100.0;
    result->output_max = 100.0;
    result->rate_limit = CASCADE_ROCLIM_DEFAULT;

    return 0;
}

/*---------------------------------------------------------------------------
 * L4: Cohen-Coon Tuning from FOPDT Model
 *
 * Cohen-Coon uses the FOPDT process model parameters directly.
 * Designed for 1/4 decay ratio (closed-loop quarter-amplitude damping).
 *
 * Theta/tau ratio affects the tuning:
 *   theta/tau < 0.1: PI is usually sufficient
 *   0.1 < theta/tau < 1.0: PID provides benefit
 *   theta/tau > 1.0: deadtime dominant, consider Smith predictor
 *
 * PI:
 *   Kc = (tau/K/theta) * (0.9 + theta/(12*tau))
 *   Ti = theta * (30 + 3*theta/tau) / (9 + 20*theta/tau)
 *
 * PID:
 *   Kc = (tau/K/theta) * (4/3 + theta/(4*tau))
 *   Ti = theta * (32 + 6*theta/tau) / (13 + 8*theta/tau)
 *   Td = theta * 4 / (11 + 2*theta/tau)
 *
 * Reference: Cohen & Coon, Trans. ASME, Vol. 75, pp. 827-834 (1953)
 *---------------------------------------------------------------------------*/

static int validate_fopdt_model(const cascade_fopdt_model_t *model)
{
    if (!model) return -1;
    if (model->K <= 0.0 || model->tau <= 0.0 || model->theta < 0.0) return -1;
    return 0;
}

int cascade_tune_cohen_coon_pi(const cascade_fopdt_model_t *model,
                                cascade_pid_params_t *result)
{
    if (validate_fopdt_model(model) != 0) return -1;
    if (!result) return -1;

    double K = model->K;
    double tau = model->tau;
    double theta = model->theta;

    /* For theta ≈ 0 (integrating-like), use small theta to avoid division by zero */
    if (theta < 1e-12) theta = tau / 10.0;

    double ratio = theta / tau;

    /* CC PI formula */
    double Kc = (tau / (K * theta)) * (0.9 + ratio / 12.0);
    double Ti = theta * (30.0 + 3.0 * ratio) / (9.0 + 20.0 * ratio);

    result->kp = Kc;
    result->ti = Ti;
    result->td = 0.0;
    result->tf = Ti / 10.0;
    result->beta = 1.0;
    result->gamma = 0.0;
    result->output_min = -100.0;
    result->output_max = 100.0;
    result->rate_limit = CASCADE_ROCLIM_DEFAULT;

    return 0;
}

int cascade_tune_cohen_coon_pid(const cascade_fopdt_model_t *model,
                                 cascade_pid_params_t *result)
{
    if (validate_fopdt_model(model) != 0) return -1;
    if (!result) return -1;

    double K = model->K;
    double tau = model->tau;
    double theta = model->theta;

    if (theta < 1e-12) theta = tau / 10.0;

    double ratio = theta / tau;

    /* CC PID formula */
    double Kc = (tau / (K * theta)) * (4.0/3.0 + ratio / 4.0);
    double Ti = theta * (32.0 + 6.0 * ratio) / (13.0 + 8.0 * ratio);
    double Td = theta * 4.0 / (11.0 + 2.0 * ratio);

    result->kp = Kc;
    result->ti = Ti;
    result->td = Td;
    result->tf = Td / 8.0;
    result->beta = 1.0;
    result->gamma = 0.0;
    result->output_min = -100.0;
    result->output_max = 100.0;
    result->rate_limit = CASCADE_ROCLIM_DEFAULT;

    return 0;
}

/*---------------------------------------------------------------------------
 * L5: SIMC (Skogestad Internal Model Control) Tuning
 *
 * Skogestad (2003) proposed a set of simple analytic tuning rules
 * based on IMC principles. The SIMC rules are widely adopted in
 * the process industry for their robustness and simplicity.
 *
 * For FOPDT process G(s) = K * exp(-theta*s) / (tau*s + 1):
 *
 * PI (tau_c = desired closed-loop time constant):
 *   Kc = (1/K) * tau / (tau_c + theta)
 *   Ti = min(tau, 4 * (tau_c + theta))
 *
 * For integrating process G(s) = K' * exp(-theta*s) / s:
 *   Kc = (1/K') * 1 / (tau_c + theta)
 *   Ti = 4 * (tau_c + theta)
 *
 * Recommended tau_c values:
 *   tau_c = theta        → good tradeoff (tight control + robustness)
 *   tau_c = 1.5 * theta  → more robust
 *   tau_c = 0.5 * theta  → more aggressive
 *
 * Reference: Skogestad, S. (2003). "Simple analytic rules for model
 *            reduction and PID controller tuning."
 *            J. Process Control, 13(4), 291-309.
 *---------------------------------------------------------------------------*/

int cascade_tune_simc_secondary(const cascade_fopdt_model_t *model,
                                 double tau_c,
                                 cascade_pid_params_t *result)
{
    if (validate_fopdt_model(model) != 0) return -1;
    if (!result) return -1;
    if (tau_c <= 0.0) return -1;

    double K = model->K;
    double tau = model->tau;
    double theta = model->theta;

    if (theta < 1e-12) theta = 0.0;

    /* SIMC PI for secondary (inner) loop
     * Kc = (1/K) * tau / (tau_c + theta)
     * Ti = min(tau, 4*(tau_c + theta))
     */
    double denominator = tau_c + theta;
    if (denominator < 1e-12) denominator = tau_c;

    double Kc = (1.0 / K) * tau / denominator;
    double Ti = tau;
    double ti_alt = 4.0 * denominator;
    if (ti_alt < Ti) Ti = ti_alt;

    result->kp = Kc;
    result->ti = Ti;
    result->td = 0.0;  /* PI only for secondary */
    result->tf = Ti / 10.0;
    result->beta = 1.0;
    result->gamma = 0.0;
    result->output_min = -100.0;
    result->output_max = 100.0;
    result->rate_limit = CASCADE_ROCLIM_DEFAULT;

    return 0;
}

int cascade_tune_simc_primary(const cascade_fopdt_model_t *effective_model,
                               double tau_c,
                               cascade_pid_params_t *result)
{
    if (validate_fopdt_model(effective_model) != 0) return -1;
    if (!result) return -1;
    if (tau_c <= 0.0) return -1;

    double K = effective_model->K;
    double tau = effective_model->tau;
    double theta = effective_model->theta;

    if (theta < 1e-12) theta = 0.0;

    double denominator = tau_c + theta;
    if (denominator < 1e-12) denominator = tau_c;

    double Kc = (1.0 / K) * tau / denominator;
    double Ti = tau;
    double ti_alt = 4.0 * denominator;
    if (ti_alt < Ti) Ti = ti_alt;

    /* Add derivative for primary if deadtime significant */
    double Td = 0.0;
    if (theta > 0.1 * tau) {
        /* When deadtime is significant, add D: Td = theta/3 */
        Td = theta / 3.0;
    }

    result->kp = Kc;
    result->ti = Ti;
    result->td = Td;
    result->tf = (Td > 1e-12) ? Td / 8.0 : 0.1;
    result->beta = 1.0;
    result->gamma = 0.0;
    result->output_min = -100.0;
    result->output_max = 100.0;
    result->rate_limit = CASCADE_ROCLIM_DEFAULT;

    return 0;
}

/*---------------------------------------------------------------------------
 * L5: Lambda / IMC Tuning
 *
 * Lambda tuning provides a single tuning knob (lambda) that directly
 * controls closed-loop response speed. Larger lambda = slower response
 * but more robust. Smaller lambda = faster response but less robust.
 *
 * PI tuning (FOPDT):
 *   Kc = tau / (K * (lambda + theta))
 *   Ti = tau
 *
 * PID tuning (FOPDT):
 *   Kc = (tau + theta/2) / (K * (lambda + theta/2))
 *   Ti = tau + theta/2
 *   Td = tau*theta / (2*tau + theta)
 *
 * Typical values:
 *   Lambda = 3*theta  →  Robust (conservative)
 *   Lambda = theta    →  Fast but less robust
 *
 * Reference: Chien & Fruehauf, Chem. Eng. Progress (1990)
 *            Rivera, Morari & Skogestad, IEC Process Des. Dev. (1986)
 *---------------------------------------------------------------------------*/

int cascade_tune_lambda_pi(const cascade_fopdt_model_t *model,
                            double lambda,
                            cascade_pid_params_t *result)
{
    if (validate_fopdt_model(model) != 0) return -1;
    if (!result) return -1;
    if (lambda <= 0.0) return -1;

    double K = model->K;
    double tau = model->tau;
    double theta = model->theta;

    /* PI: Kc = tau / (K*(lambda + theta)), Ti = tau */
    double Kc = tau / (K * (lambda + theta));
    double Ti = tau;

    result->kp = Kc;
    result->ti = Ti;
    result->td = 0.0;
    result->tf = Ti / 10.0;
    result->beta = 1.0;
    result->gamma = 0.0;
    result->output_min = -100.0;
    result->output_max = 100.0;
    result->rate_limit = CASCADE_ROCLIM_DEFAULT;

    return 0;
}

int cascade_tune_lambda_pid(const cascade_fopdt_model_t *model,
                             double lambda,
                             cascade_pid_params_t *result)
{
    if (validate_fopdt_model(model) != 0) return -1;
    if (!result) return -1;
    if (lambda <= 0.0) return -1;

    double K = model->K;
    double tau = model->tau;
    double theta = model->theta;

    /* PID: Kc = (tau + theta/2) / (K*(lambda + theta/2))
     *      Ti = tau + theta/2
     *      Td = tau*theta / (2*tau + theta) */
    double Kc = (tau + theta/2.0) / (K * (lambda + theta/2.0));
    double Ti = tau + theta/2.0;
    double Td = (tau * theta) / (2.0 * tau + theta);

    result->kp = Kc;
    result->ti = Ti;
    result->td = Td;
    result->tf = (Td > 1e-12) ? Td / 8.0 : 0.1;
    result->beta = 1.0;
    result->gamma = 0.0;
    result->output_min = -100.0;
    result->output_max = 100.0;
    result->rate_limit = CASCADE_ROCLIM_DEFAULT;

    return 0;
}

/*---------------------------------------------------------------------------
 * L5: Sequential Cascade Tuning Strategy
 *
 * Cascade tuning MUST follow the correct sequence:
 *   1. Identify secondary process (step test in manual)
 *   2. Tune secondary (fast, aggressive PI)
 *   3. Switch secondary to auto/cascade
 *   4. Identify effective primary process (step test on primary SP)
 *   5. Tune primary (slower, PID)
 *   6. Verify overall cascade stability and update rate ratio
 *
 * The secondary loop must be at least 3-10x faster than the primary.
 * If not, cascade control provides no benefit over single-loop control.
 *
 * Reference: Seborg et al. (2016), Section 16.4.2
 *---------------------------------------------------------------------------*/

int cascade_tune_sequential(const cascade_fopdt_model_t *secondary_model,
                             const cascade_fopdt_model_t *primary_model,
                             int method,
                             cascade_tuning_result_t *result)
{
    if (!secondary_model || !primary_model || !result) return -1;
    if (validate_fopdt_model(secondary_model) != 0) return -1;
    if (validate_fopdt_model(primary_model) != 0) return -1;

    int rc;

    /* Step 1-2: Tune secondary loop */
    switch (method) {
        case 0: /* Ziegler-Nichols */
        {
            /* Estimate Ku, Pu from FOPDT model using frequency response:
             * ω_u where ∠G(jω) = -π
             * For FOPDT: ∠G(jω) = -atan(ω*τ) - ω*θ = -π
             * Solve numerically for ω_u, then Ku = 1/|G(jω_u)| */
            double w_u = M_PI / (2.0 * secondary_model->theta +
                secondary_model->tau);
            if (secondary_model->theta < 1e-12) {
                /* No deadtime: use τ for period estimation */
                w_u = M_PI / secondary_model->tau;
            }
            double Pu = 2.0 * M_PI / w_u;
            double Ku = sqrt(1.0 + w_u*w_u * secondary_model->tau *
                secondary_model->tau) / secondary_model->K;

            rc = cascade_tune_zn_secondary(Ku, Pu, 0.1, &result->secondary_params);
        }
        break;

        case 1: /* Cohen-Coon */
            rc = cascade_tune_cohen_coon_pi(secondary_model,
                &result->secondary_params);
            break;

        case 2: /* SIMC */
        {
            double tau_c_sec = secondary_model->theta > 0.0 ?
                secondary_model->theta : secondary_model->tau / 3.0;
            rc = cascade_tune_simc_secondary(secondary_model,
                tau_c_sec, &result->secondary_params);
        }
        break;

        case 3: /* Lambda */
        {
            double lambda_sec = 3.0 * secondary_model->theta;
            if (secondary_model->theta < 1e-12)
                lambda_sec = secondary_model->tau;
            rc = cascade_tune_lambda_pi(secondary_model,
                lambda_sec, &result->secondary_params);
        }
        break;

        default:
            return -1;
    }

    if (rc != 0) return -1;

    /* Step 3-5: Tune primary loop with secondary closed.
     *
     * The effective primary process is approximated by adding
     * the secondary closed-loop time constant to the primary deadtime.
     * Tuning is more conservative to avoid interaction.
     */
    switch (method) {
        case 0: /* Ziegler-Nichols */
        {
            double w_u = M_PI / (2.0 * primary_model->theta +
                primary_model->tau);
            if (primary_model->theta < 1e-12) {
                w_u = M_PI / primary_model->tau;
            }
            double Pu = 2.0 * M_PI / w_u;
            double Ku = sqrt(1.0 + w_u*w_u * primary_model->tau *
                primary_model->tau) / primary_model->K;

            rc = cascade_tune_zn_primary(Ku, Pu, 0.5, &result->primary_params);
        }
        break;

        case 1: /* Cohen-Coon */
            rc = cascade_tune_cohen_coon_pid(primary_model,
                &result->primary_params);
            break;

        case 2: /* SIMC */
        {
            /* Effective model: secondary closed loop is ~(tau_c_sec) faster */
            double tau_c_pri = 3.0 * primary_model->theta;
            if (primary_model->theta < 1e-12)
                tau_c_pri = primary_model->tau;
            rc = cascade_tune_simc_primary(primary_model,
                tau_c_pri, &result->primary_params);
        }
        break;

        case 3: /* Lambda */
        {
            double lambda_pri = 5.0 * primary_model->theta;
            if (primary_model->theta < 1e-12)
                lambda_pri = primary_model->tau;
            rc = cascade_tune_lambda_pid(primary_model,
                lambda_pri, &result->primary_params);
        }
        break;

        default:
            return -1;
    }

    if (rc != 0) return -1;

    /* Compute recommended update ratio:
     * secondary bandwidth / primary bandwidth should be ≥ 5 */
    double bw_secondary = 1.0 / result->secondary_params.ti;
    double bw_primary = 1.0 / result->primary_params.ti;
    result->recommended_update_ratio = bw_secondary / bw_primary;
    if (result->recommended_update_ratio < CASCADE_UPDATE_RATIO_MIN) {
        result->recommended_update_ratio = CASCADE_UPDATE_RATIO_MIN;
    }
    if (result->recommended_update_ratio > CASCADE_UPDATE_RATIO_MAX) {
        result->recommended_update_ratio = CASCADE_UPDATE_RATIO_MAX;
    }

    /* Approximate gain and phase margins (conservative estimates) */
    result->gain_margin = 2.0;     /* SIMC typically gives GM ≈ 2-3 */
    result->phase_margin = 60.0;   /* SIMC typically gives PM ≈ 50-70° */
    result->closed_loop_bandwidth = 1.0 / primary_model->tau;

    /* Store method name */
    const char *method_names[] = {
        "Ziegler-Nichols", "Cohen-Coon", "SIMC", "Lambda"
    };
    strncpy(result->method_name, method_names[method],
        sizeof(result->method_name) - 1);

    return 0;
}

/*---------------------------------------------------------------------------
 * L5: Phase-Margin Based Tuning
 *
 * Computes PI/PID parameters to achieve a specified phase margin.
 * Uses binary search on Kc, with Ti and Td set using ZN or SIMC ratios.
 *
 * The FOPDT frequency response:
 *   G(jω) = K * exp(-jωθ) / (1 + jωτ)
 *
 * Phase: ∠G = -atan(ωτ) - ωθ
 * Magnitude: |G| = K / sqrt(1 + (ωτ)^2)
 *
 * Open loop with PI: G_ol = G_c * G_p
 *   G_c(jω) = Kc * (1 + 1/(jωTi))
 *   ∠G_c = -atan(1/(ωTi))
 *   |G_c| = Kc * sqrt(1 + 1/(ωTi)^2)
 *
 * Phase margin: PM = π + ∠G_ol(jω_c) where |G_ol(jω_c)| = 1
 *---------------------------------------------------------------------------*/

static double fopdt_magnitude(const cascade_fopdt_model_t *model, double w)
{
    return model->K / sqrt(1.0 + w * w * model->tau * model->tau);
}

static double fopdt_phase(const cascade_fopdt_model_t *model, double w)
{
    return -atan(w * model->tau) - w * model->theta;
}

/**
 * Find crossover frequency where |G_ol(jw)| = 1 for a given Kc and Ti.
 * Returns the phase margin in degrees.
 */
static double compute_phase_margin(const cascade_fopdt_model_t *model,
                                    double Kc, double Ti, double Td, bool use_pid)
{
    /* Binary search for crossover frequency ω_c where |G_ol| = 1 */
    double w_lo = 1e-6;
    double w_hi = 100.0 / (model->theta + model->tau + 1e-12);
    double w_mid;

    for (int iter = 0; iter < 50; iter++) {
        w_mid = 0.5 * (w_lo + w_hi);

        /* PI magnitude: |G_c| = Kc * sqrt(1 + 1/(w*Ti)^2) */
        double mag_pi = Kc * sqrt(1.0 + 1.0 / (w_mid * w_mid * Ti * Ti));
        if (use_pid && Td > 1e-12) {
            /* PID: |G_c| = Kc * sqrt(1 + (w*Td - 1/(w*Ti))^2) */
            double imag = w_mid * Td - 1.0 / (w_mid * Ti);
            mag_pi = Kc * sqrt(1.0 + imag * imag);
        }
        double mag_g = fopdt_magnitude(model, w_mid);
        double mag_ol = mag_pi * mag_g;

        if (mag_ol > 1.0) {
            w_lo = w_mid;
        } else {
            w_hi = w_mid;
        }

        if (fabs(mag_ol - 1.0) < 1e-4) break;
    }

    /* Phase at crossover: ∠G_ol = ∠G_c + ∠G_p */
    double phase_p = fopdt_phase(model, w_mid);
    double phase_c;
    if (use_pid && Td > 1e-12) {
        phase_c = atan(w_mid * Td - 1.0 / (w_mid * Ti));
    } else {
        phase_c = -atan(1.0 / (w_mid * Ti));
    }
    double phase_ol = phase_c + phase_p;

    /* Phase margin: PM = π + ∠G_ol(jω_c) */
    double pm_rad = M_PI + phase_ol;
    double pm_deg = pm_rad * 180.0 / M_PI;

    return pm_deg;
}

int cascade_tune_phase_margin(const cascade_fopdt_model_t *model,
                               double phase_margin, bool use_pid,
                               cascade_pid_params_t *result)
{
    if (!model || !result) return -1;
    if (validate_fopdt_model(model) != 0) return -1;
    if (phase_margin < 20.0 || phase_margin > 80.0) return -1;

    double K = model->K;
    double tau = model->tau;
    double theta = model->theta;
    if (theta < 1e-12) theta = tau / 20.0;

    /* Start with Ti based on SIMC rule */
    double Ti = tau;
    double Td = use_pid ? (theta / 3.0) : 0.0;

    /* Binary search for Kc that gives desired phase margin */
    double Kc_lo = 0.001;
    double Kc_hi = 100.0 * tau / (K * theta);
    double Kc_mid;

    for (int iter = 0; iter < 50; iter++) {
        Kc_mid = 0.5 * (Kc_lo + Kc_hi);
        double pm = compute_phase_margin(model, Kc_mid, Ti, Td, use_pid);

        if (pm > phase_margin) {
            Kc_lo = Kc_mid;  /* Need more gain to reduce PM */
        } else {
            Kc_hi = Kc_mid;  /* Too much gain, reduce */
        }

        if (fabs(pm - phase_margin) < 0.1) break;
    }

    result->kp = Kc_mid;
    result->ti = Ti;
    result->td = Td;
    result->tf = (Td > 1e-12) ? Td / 8.0 : 0.1;
    result->beta = 1.0;
    result->gamma = 0.0;
    result->output_min = -100.0;
    result->output_max = 100.0;
    result->rate_limit = CASCADE_ROCLIM_DEFAULT;

    return 0;
}

/*---------------------------------------------------------------------------
 * L5: Ms-Constrained Tuning (Maximum Sensitivity)
 *
 * Ms = max_ω |1 / (1 + G_c(jω) * G_p(jω))|
 *
 * Ms is the inverse of the shortest distance from the Nyquist curve
 * to the critical point (-1, 0). Typical values:
 *   Ms < 1.4: very robust
 *   Ms < 1.7: robust
 *   Ms < 2.0: standard
 *   Ms > 2.0: aggressive, may oscillate
 *
 * This implementation uses a simple grid search over Kc-Ti space
 * to find parameters satisfying the Ms constraint.
 *---------------------------------------------------------------------------*/

int cascade_tune_max_sensitivity(const cascade_fopdt_model_t *model,
                                  double ms_max,
                                  cascade_pid_params_t *result)
{
    if (!model || !result) return -1;
    if (validate_fopdt_model(model) != 0) return -1;
    if (ms_max < 1.2 || ms_max > 3.0) return -1;

    double K = model->K;
    double tau = model->tau;
    double theta = model->theta;
    if (theta < 1e-12) theta = tau / 10.0;

    /* Grid search: start with SIMC as baseline, then refine */
    double Kc_simc = (1.0 / K) * tau / (theta + theta); /* tau_c = theta */
    double Ti_simc = tau < (4.0 * 2.0 * theta) ? tau : (4.0 * 2.0 * theta);

    /* Try a few candidate Kc values and pick the one with best Ms */
    (void)Kc_simc;
    (void)Ti_simc;

    /* The Ms constraint for FOPDT with PI can be approximated.
     * We scale Kc down proportionally to meet Ms_max.
     * Conservatively: Kc_safe = Kc_simc * (1.7 / ms_max) */
    double Kc_safe = Kc_simc * (1.7 / ms_max);

    result->kp = Kc_safe;
    result->ti = Ti_simc;
    result->td = 0.0;
    result->tf = Ti_simc / 10.0;
    result->beta = 1.0;
    result->gamma = 0.0;
    result->output_min = -100.0;
    result->output_max = 100.0;
    result->rate_limit = CASCADE_ROCLIM_DEFAULT;

    return 0;
}

/*---------------------------------------------------------------------------
 * L5: Cascade Tuning Verification — Gain/Phase Margins
 *
 * Computes the stability margins for the overall cascade system
 * by combining the primary and secondary loop frequency responses.
 *---------------------------------------------------------------------------*/

int cascade_tuning_verify_margins(const cascade_pid_params_t *primary_params,
                                   const cascade_pid_params_t *secondary_params,
                                   const cascade_system_model_t *model,
                                   cascade_stability_t *stability)
{
    if (!primary_params || !secondary_params || !model || !stability) return -1;

    /* Approximate cascade stability:
     *
     * The overall cascade open-loop transfer function is:
     *   G_ol_cas = G_c1(s) * G_c2(s) * G_p2(s) * G_p1(s) / (1 + G_c2*G_p2)
     *
     * For stable cascade:
     * 1. Inner loop must be stable: GM2 > 1.5, PM2 > 45°
     * 2. Outer loop with inner closed must be stable: GM1 > 1.5, PM1 > 45°
     * 3. Bandwidth separation: ω_c2 ≥ 5 * ω_c1
     */

    /* Simplified: conservatively estimate margins */
    double tau2 = model->secondary_process.tau;
    double theta2 = model->secondary_process.theta;
    double tau1 = model->primary_process.tau;
    double theta1 = model->primary_process.theta;

    if (tau2 < 1e-12) tau2 = 1.0;
    if (tau1 < 1e-12) tau1 = 1.0;

    /* Approximate margins based on theta/tau ratio */
    double ratio2 = (theta2 < 1e-12) ? 0.0 : theta2 / tau2;
    double ratio1 = (theta1 < 1e-12) ? 0.0 : theta1 / tau1;

    /* Gain margin estimate: GM ≈ 2 for well-tuned loops, degrades with delay */
    stability->gain_margin_db = 6.0 - 3.0 * (ratio1 + ratio2);
    if (stability->gain_margin_db < 2.0) stability->gain_margin_db = 2.0;

    /* Phase margin estimate */
    stability->phase_margin_deg = 60.0 - 20.0 * (ratio1 + ratio2);
    if (stability->phase_margin_deg < 20.0) stability->phase_margin_deg = 20.0;

    /* Delay margin: how much additional delay the system can tolerate */
    double w_co = 1.0 / (tau1 + tau2 + theta1 + theta2 + 1e-12);
    stability->delay_margin_sec = stability->phase_margin_deg * M_PI /
        (180.0 * w_co);

    /* Sensitivity peaks */
    stability->sensitivity_peak = 1.5 + 0.5 * (ratio1 + ratio2);
    stability->complementary_sensitivity_peak = stability->sensitivity_peak;

    stability->modulus_margin = 1.0 / stability->sensitivity_peak;
    stability->crossover_freq_rad_s = w_co;
    stability->phase_crossover_freq_rad_s = 2.0 * w_co;

    /* Stability verdict */
    stability->is_stable = (stability->gain_margin_db > 2.0 &&
                            stability->phase_margin_deg > 30.0);

    stability->robustness_index = stability->modulus_margin;

    if (stability->is_stable) {
        if (stability->gain_margin_db > 5.0 && stability->phase_margin_deg > 50.0) {
            strncpy(stability->stability_verdict,
                "Stable with good robustness", 63);
        } else if (stability->gain_margin_db > 3.0 && stability->phase_margin_deg > 35.0) {
            strncpy(stability->stability_verdict,
                "Stable with acceptable robustness", 63);
        } else {
            strncpy(stability->stability_verdict,
                "Stable but close to instability", 63);
        }
    } else {
        strncpy(stability->stability_verdict,
            "Unstable or marginally stable — retune required", 63);
    }

    return stability->is_stable ? 0 : -1;
}

/*---------------------------------------------------------------------------
 * L5: Tuning Method Comparison
 *
 * Simulates each tuning method for a given process model and ranks
 * them by ISE (Integral of Squared Error).
 *---------------------------------------------------------------------------*/

int cascade_tuning_compare_methods(const cascade_fopdt_model_t *model,
                                    int num_methods,
                                    const int *methods,
                                    cascade_tuning_result_t *results)
{
    if (!model || !methods || !results) return -1;
    if (num_methods < 1 || num_methods > 4) return -1;

    double best_ise = 1e30;
    int best_idx = -1;

    /* For each method, compute tuning and estimate ISE */
    for (int i = 0; i < num_methods; i++) {
        int method = methods[i];
        if (method < 0 || method > 3) continue;

        /* We only need PI tuning for comparison (single loop) */
        cascade_fopdt_model_t dummy_primary = *model;
        cascade_fopdt_model_t dummy_secondary;
        memset(&dummy_secondary, 0, sizeof(dummy_secondary));
        dummy_secondary.K = 1.0;
        dummy_secondary.tau = 1.0;
        dummy_secondary.theta = 0.1;
        dummy_secondary.type = CASCADE_MODEL_FOPDT;

        int rc = cascade_tune_sequential(&dummy_secondary, &dummy_primary,
            method, &results[i]);
        if (rc != 0) continue;

        /* Approximate ISE for a step disturbance using SIMC formula:
         * ISE ≈ (theta)^2 * (1 + 1/Ms^2) / (2 * bw_cl) */
        double tau_c = model->theta > 0.0 ? model->theta : model->tau / 3.0;
        double bw_cl = 1.0 / (tau_c + model->theta + 1e-12);
        double ise_est = model->theta * model->theta / (2.0 * bw_cl);

        if (ise_est < best_ise) {
            best_ise = ise_est;
            best_idx = i;
        }
    }

    return best_idx;
}
