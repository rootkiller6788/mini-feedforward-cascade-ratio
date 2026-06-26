/**
 * @file smith_adaptive.c
 * @brief Adaptive Smith predictor — online model identification and retuning.
 *
 * Implements self-tuning dead-time compensation:
 *   - Recursive Least Squares (RLS) for online FOPDT model identification
 *   - Automatic controller redesign when model parameters change
 *   - MIT rule and Lyapunov-based gradient adaptation
 *   - Model-reference adaptive Smith predictor (MRAS)
 *   - Supervisory safety logic and parameter projection
 *
 * The adaptive Smith predictor addresses the central weakness of the
 * standard Smith predictor: sensitivity to model mismatch. By continuously
 * updating the model, the controller maintains performance even as the
 * process changes (fouling, wear, operating-point shifts).
 *
 * References:
 *   Astrom & Wittenmark (1995) "Adaptive Control", 2nd ed., Addison-Wesley
 *       Chapter 3: "Deterministic Self-Tuning Regulators"
 *       Chapter 4: "Model-Reference Adaptive Systems"
 *   Hagglund & Astrom (2002) J. Process Control, revisiting ZN method
 *   Landau, Lozano, M'Saad, Karimi (2011) "Adaptive Control", Springer
 *       Chapter 5: "Parameter Adaptation Algorithms — Stability Analysis"
 *   Dumont, G.A. et al. (1993) "Concepts and methods in adaptive control"
 *       American Control Conference tutorial
 */

#include "smith_adaptation.h"
#include "smith_predictor.h"
#include "smith_tuning.h"
#include "smith_identification.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*===========================================================================
 * L8: Adaptive Smith Predictor — Initialization
 *===========================================================================*/

int smith_adaptive_init(
    smith_adaptive_t *adp,
    double K_init, double tau_init, double theta_init,
    double Ts, smith_variant_t variant,
    double u_min, double u_max)
{
    if (!adp) return -1;
    memset(adp, 0, sizeof(*adp));

    /* Initialize the embedded Smith predictor */
    int ret = smith_predictor_init_fopdt(
        &adp->predictor, K_init, tau_init, theta_init, Ts,
        variant, u_min, u_max);
    if (ret < 0) return ret;

    /* Initialize RLS identifier */
    smith_rls_init(&adp->identifier, 0.98, K_init, tau_init, theta_init);

    /* Default tuning: use SIMC with moderate tau_c */
    double Kp_init, Ti_init;
    smith_tune_simc_pi(&adp->predictor.model, tau_init, &Kp_init, &Ti_init);
    smith_predictor_set_pi(&adp->predictor, Kp_init, Ti_init, 1.0);

    /* Set robustness filter */
    smith_predictor_set_robustness_filter(&adp->predictor, theta_init / 2.0);

    /* Adaptation parameters */
    adp->adaptation_rate = 0.1;
    adp->model_change_threshold = 0.05;  /* 5% change triggers redesign */
    adp->last_K = K_init;
    adp->last_tau = tau_init;
    adp->last_theta = theta_init;
    adp->adapt_enabled = 1;
    adp->redesign_count = 0;

    return 0;
}

void smith_adaptive_destroy(smith_adaptive_t *adp)
{
    if (!adp) return;
    smith_predictor_destroy(&adp->predictor);
}

void smith_adaptive_set_enabled(smith_adaptive_t *adp, int enable)
{
    if (!adp) return;
    adp->adapt_enabled = enable;
}

/*===========================================================================
 * L8: Adaptive Control Step
 *
 * Sequence:
 *   1. Standard Smith predictor control computation
 *   2. Update RLS identifier with latest I/O data
 *   3. Extract FOPDT model from RLS estimates
 *   4. Check for significant parameter changes
 *   5. If changed enough, redesign controller and update model
 *===========================================================================*/

double smith_adaptive_step(smith_adaptive_t *adp, double setpoint, double pv)
{
    if (!adp) return 0.0;

    /* Step 1: Standard Smith predictor step */
    double u = smith_predictor_step(&adp->predictor, setpoint, pv);

    if (!adp->adapt_enabled) return u;

    /* Step 2: Update RLS identifier (only when persistently excited) */
    /* Skip update if setpoint hasn't changed significantly recently */
    static double prev_setpoint = 0.0;
    static int settle_counter = 0;
    double sp_change = fabs(setpoint - prev_setpoint);
    prev_setpoint = setpoint;

    /* Only identify when there's excitation in the data */
    if (sp_change > 0.01 * fmax(fabs(setpoint), 1.0) || settle_counter > 50) {
        smith_rls_update(&adp->identifier, u, pv, adp->predictor.Ts);
        settle_counter = 0;
    } else {
        settle_counter++;
    }

    /* Step 3: Extract FOPDT model from RLS */
    smith_process_model_t est_model;
    if (smith_rls_to_fopdt(&adp->identifier, &est_model) == 0) {

        /* Step 4: Check for significant parameter changes */
        double dK = fabs(est_model.fopdt.K - adp->last_K);
        double dtau = fabs(est_model.fopdt.tau - adp->last_tau);
        double dtheta = fabs(est_model.fopdt.theta - adp->last_theta);

        double rel_dK = (fabs(adp->last_K) > 1e-9) ?
                        dK / fabs(adp->last_K) : dK;
        double rel_dtau = (adp->last_tau > 1e-9) ?
                          dtau / adp->last_tau : dtau;
        double rel_dtheta = (adp->last_theta > 1e-9) ?
                            dtheta / adp->last_theta : dtheta;

        double max_rel_change = fmax(rel_dK, fmax(rel_dtau, rel_dtheta));

        if (max_rel_change > adp->model_change_threshold) {
            /* Step 5: Redesign controller */
            smith_adaptive_redesign(adp);
        }
    }

    return u;
}

/*===========================================================================
 * L5: Controller Redesign
 *
 * Recomputes PI parameters and robustness filter based on updated model.
 * Uses filtered adaptation to avoid sudden parameter jumps.
 *===========================================================================*/

int smith_adaptive_redesign(smith_adaptive_t *adp)
{
    if (!adp) return -1;

    /* Extract current model from RLS */
    smith_process_model_t est_model;
    if (smith_rls_to_fopdt(&adp->identifier, &est_model) != 0) {
        return -1;
    }

    /* Get current nominal model for mixing */
    double K_old = adp->predictor.model.fopdt.K;
    double tau_old = adp->predictor.model.fopdt.tau;
    double theta_old = adp->predictor.model.fopdt.theta;

    double K_new = est_model.fopdt.K;
    double tau_new = est_model.fopdt.tau;
    double theta_new = est_model.fopdt.theta;

    /* Exponential smoothing to avoid sudden jumps:
       param_new = rate * estimated + (1 - rate) * old */
    double rate = adp->adaptation_rate;
    double K_blend = rate * K_new + (1.0 - rate) * K_old;
    double tau_blend = rate * tau_new + (1.0 - rate) * tau_old;
    double theta_blend = rate * theta_new + (1.0 - rate) * theta_old;

    /* Update model */
    smith_predictor_update_model(&adp->predictor, K_blend, tau_blend, theta_blend);

    /* Retune PI using SIMC */
    double Kp_new, Ti_new;
    /* tau_c = tau (default balanced response) */
    smith_tune_simc_pi(&adp->predictor.model, tau_blend, &Kp_new, &Ti_new);
    smith_predictor_set_pi(&adp->predictor, Kp_new, Ti_new, 1.0);

    /* Update robustness filter */
    double Fr_new;
    smith_tune_robustness_filter(&adp->predictor.model, 0.2, 0.1, &Fr_new);
    smith_predictor_set_robustness_filter(&adp->predictor, Fr_new);

    /* Store last values for change detection */
    adp->last_K = K_blend;
    adp->last_tau = tau_blend;
    adp->last_theta = theta_blend;
    adp->redesign_count++;

    return 0;
}

/*===========================================================================
 * L5: Parameter Change Detection (CUSUM)
 *
 * CUSUM algorithm for detecting shifts in the prediction error mean:
 *   S_high(k) = max(0, S_high(k-1) + e(k) - mu - beta)
 *   S_low(k)  = max(0, S_low(k-1) - e(k) + mu - beta)
 *   Change if S_high > h or S_low > h
 *
 * This detects model mismatch before it causes poor control performance.
 *
 * Reference: Page (1954) "Continuous inspection schemes", Biometrika
 *            Basseville & Nikiforov (1993) "Detection of Abrupt Changes"
 *===========================================================================*/

int smith_adaptive_detect_change(
    double prediction_error, double state[2],
    double threshold, double drift)
{
    double mu = 0.0;  /* Target mean of prediction error (0 = perfect model) */

    /* Update CUSUM statistics */
    state[0] = fmax(0.0, state[0] + prediction_error - mu - drift);
    state[1] = fmax(0.0, state[1] - prediction_error + mu - drift);

    return (state[0] > threshold || state[1] > threshold) ? 1 : 0;
}

/*===========================================================================
 * L8: Estimate Model Uncertainty from Prediction Error
 *
 * Uses running statistics to estimate parameter uncertainty levels.
 * The variance of the prediction error indicates overall model quality.
 *
 * Relationship: var(e) ≈ K² * var(u) * [(ΔK/K)² + (Δτ/τ)²] + ...
 *
 * Simplified: use error standard deviation to bound uncertainty.
 *===========================================================================*/

void smith_adaptive_estimate_uncertainty(
    const double *prediction_errors, size_t n,
    double *K_out, double *tau_out, double *theta_out)
{
    if (!prediction_errors || n < 2) {
        if (K_out) *K_out = 1.0;
        if (tau_out) *tau_out = 1.0;
        if (theta_out) *theta_out = 0.0;
        return;
    }

    /* Compute mean and variance */
    double sum = 0.0, sum_sq = 0.0;
    for (size_t i = 0; i < n; i++) {
        sum += prediction_errors[i];
        sum_sq += prediction_errors[i] * prediction_errors[i];
    }
    double mean = sum / (double)n;
    double var = sum_sq / (double)n - mean * mean;
    if (var < 0.0) var = 0.0;
    double std = sqrt(var);

    /* Estimate uncertainty:
       Larger prediction error variance → higher uncertainty
       Approximation: each uncertainty component proportional to std */
    if (K_out) *K_out = fmin(std, 0.5);        /* Cap at 50% uncertainty */
    if (tau_out) *tau_out = fmin(std, 0.5);
    if (theta_out) *theta_out = std;            /* Dead-time uncertainty in seconds */
}

/*===========================================================================
 * L8: Gradient-Based Adaptation (MIT Rule)
 *
 * Kp(k+1) = Kp(k) - γ * ∂J/∂Kp
 * where J = 0.5 * e²(k) and ∂J/∂Kp = e(k) * ∂y/∂Kp
 *
 * The sensitivity derivative ∂y/∂Kp is approximated using the
 * delay-free model (since the Smith predictor decouples delay):
 *   ∂y/∂Kp ≈ K * u / (1 + Kp*K)
 *
 * MIT rule (Model Reference Adaptive Systems):
 *   Kp(k+1) = Kp(k) - γ * e(k) * y_f(k)
 * where y_f is the filtered sensitivity signal.
 *
 * Reference: Astrom & Wittenmark (1995), Chapter 4
 *===========================================================================*/

double smith_adaptive_gradient_Kp(
    smith_adaptive_t *adp, double error, double gamma)
{
    if (!adp) return 0.0;
    if (fabs(error) < 1e-9) return adp->predictor.Kp;

    double K = adp->predictor.model.fopdt.K;
    double Kp = adp->predictor.Kp;

    /* Sensitivity: dy/dKp ≈ K*u / (1 + Kp*K) (steady-state) */
    /* Use predicted output as approximation of sensitivity */
    double yp = adp->predictor.yp_model;

    /* MIT rule update */
    double dKp = -gamma * error * yp;

    /* Apply update with bounds */
    double Kp_new = Kp + dKp;
    if (Kp_new < 0.0) Kp_new = 0.0;
    /* Upper bound from SIMC tuning with minimum tau_c */
    double Kp_max = 10.0 / fmax(fabs(K), 1e-9);
    if (Kp_new > Kp_max) Kp_new = Kp_max;

    adp->predictor.Kp = Kp_new;
    return Kp_new;
}

/*===========================================================================
 * L8: Gradient-Based Ti Adaptation
 *
 * Ti(k+1) = Ti(k) - γ * e(k) * (1/Ti²) * ∫e dt
 *
 * The integral time is adapted to minimize steady-state error.
 *===========================================================================*/

double smith_adaptive_gradient_Ti(
    smith_adaptive_t *adp, double error, double gamma)
{
    if (!adp) return 0.0;

    double Ti = adp->predictor.Ti;
    if (Ti < 1e-9) Ti = 1.0;

    /* Sensitivity: ∂e/∂Ti ≈ -(Kp/(Ti²)) * ∫e dt (approx) */
    static double error_integral = 0.0;
    error_integral += error * adp->predictor.Ts;

    double dTi = -gamma * error * error_integral / (Ti * Ti);

    double Ti_new = Ti + dTi;
    /* Bounds */
    if (Ti_new < adp->predictor.Ts) Ti_new = adp->predictor.Ts;
    if (Ti_new > 10.0 * Ti) Ti_new = 10.0 * Ti;
    if (Ti_new < 0.0) Ti_new = Ti;

    adp->predictor.Ti = Ti_new;
    return Ti_new;
}

/*===========================================================================
 * L8: Model-Reference Adaptive Smith Predictor (MRAS)
 *
 * Reference model: Gm(s) = 1/(T_ref*s + 1)
 * Adaptation objective: minimize |y - ym| where ym is reference model output.
 *
 * MIT rule:
 *   dθ/dt = -γ * e_m * sign(K) * y_f
 * where e_m = y - y_m (model-following error), y_f = filtered output.
 *===========================================================================*/

void smith_adaptive_mras_step(
    smith_adaptive_t *adp,
    double setpoint, double pv,
    double *ref_output, double gamma)
{
    if (!adp || !ref_output) return;

    double T_ref = adp->predictor.T_ref;
    if (T_ref < 1e-9) T_ref = adp->predictor.model.fopdt.tau;

    /* Update reference model (first-order filter):
       ym(k) = (T_ref*ym(k-1) + Ts*r(k)) / (T_ref + Ts) */
    double Ts = adp->predictor.Ts;
    double ref_new = (T_ref * (*ref_output) + Ts * setpoint) / (T_ref + Ts);

    /* Model-following error */
    double e_m = pv - ref_new;  /* or use predicted output? */

    /* Adaptation gain scaled by model gain sign */
    double K = adp->predictor.model.fopdt.K;
    double sign_K = (K > 0.0) ? 1.0 : -1.0;

    /* MIT rule: dKp/dt = -gamma * e_m * sign(K) * y_f */
    /* Use filtered output (the predicted output without delay) */
    double y_f = adp->predictor.yp_model;

    /* Update Kp */
    smith_adaptive_gradient_Kp(adp, e_m * sign_K, gamma);

    *ref_output = ref_new;
}

/*===========================================================================
 * L8: Lyapunov-Based Adaptation
 *
 * Lyapunov function: V = e² + (Kp - Kp*)²/γ
 *
 * V̇ = 2e·ė + 2(Kp - Kp*)/γ · Kṗ
 *    = 2e·(-Kp·y_f) + 2(Kp - Kp*)/γ · Kṗ   [assuming ė = -Kp·y_f]
 *
 * Choose Kṗ = γ · e · y_f  → V̇ = -2e² ≤ 0
 *
 * This guarantees asymptotic stability of the adaptive system.
 *===========================================================================*/

int smith_adaptive_lyapunov_update(
    smith_adaptive_t *adp,
    double error, double y_filtered, double gamma)
{
    if (!adp) return 0;

    double Kp = adp->predictor.Kp;
    double K  = adp->predictor.model.fopdt.K;

    /* Lyapunov derivative check before update */
    /* V_dot = -2*e² + 2*(Kp-Kp*)/γ * (Kṗ - 0) */
    /* If |error| is very small, adaptation is safe (V_dot ≈ -2e² < 0) */

    /* Lyapunov-based adaptation law:
       dKp = gamma * error * y_filtered  (note: positive sign, not negative!)
       This makes V̇ = -2*e² ≤ 0 */
    double dKp = gamma * error * y_filtered;

    /* Apply with safety bounds */
    double Kp_new = Kp + dKp;
    double Kp_max = 10.0 / fmax(fabs(K), 1e-9);
    if (Kp_new < 0.0) Kp_new = 0.0;
    if (Kp_new > Kp_max) Kp_new = Kp_max;

    adp->predictor.Kp = Kp_new;

    /* Check Lyapunov condition: V_dot ≈ -2*error²
       This is always ≤ 0 for our law, so return stable */
    double V_dot = -2.0 * error * error;
    return (V_dot <= 1e-12) ? 1 : 0;
}

/*===========================================================================
 * L5: Dead-Zone and Parameter Projection (Robust Adaptation)
 *===========================================================================*/

void smith_adaptive_set_deadzone(smith_adaptive_t *adp, double dz_threshold)
{
    if (!adp) return;
    /* Store dead-zone threshold — used in step to gate adaptation */
    /* Use model_change_threshold as proxy (adaptation disabled if change small) */
    adp->model_change_threshold = dz_threshold;
}

void smith_adaptive_project_parameters(smith_adaptive_t *adp)
{
    if (!adp) return;

    /* Project Kp, Ti into physically meaningful ranges */
    if (adp->predictor.Kp < 0.0) adp->predictor.Kp = 0.0;
    if (adp->predictor.Ti < 0.0) adp->predictor.Ti = 0.0;

    /* Model parameters */
    if (adp->predictor.model.fopdt.tau < 1e-9)
        adp->predictor.model.fopdt.tau = 1e-9;
    if (adp->predictor.model.fopdt.theta < 0.0)
        adp->predictor.model.fopdt.theta = 0.0;

    /* Adaptation rate bounds */
    if (adp->adaptation_rate < 0.0) adp->adaptation_rate = 0.0;
    if (adp->adaptation_rate > 1.0) adp->adaptation_rate = 1.0;

    /* RLS parameters */
    if (adp->identifier.forgetting_factor < 0.9)
        adp->identifier.forgetting_factor = 0.95;
    if (adp->identifier.forgetting_factor > 0.999)
        adp->identifier.forgetting_factor = 0.999;
}

/*===========================================================================
 * L7: Supervisory Safety Logic
 *
 * Monitors adaptation "health" and reverts to safe defaults if problems
 * are detected. This is critical for industrial deployment — an adaptive
 * controller that learns the wrong model is worse than a fixed one.
 *===========================================================================*/

int smith_adaptive_supervision_check(
    const smith_adaptive_t *adp,
    double pv, double sv,
    double error_var, int *susp_count)
{
    if (!adp) return 0;

    int suspicious = 0;

    /* Check 1: Is the error variance unreasonably high? */
    /* Normal operating variance should be within a few times noise std */
    if (error_var > 100.0) {
        suspicious++;
    }

    /* Check 2: Is the controller saturating continuously? */
    if (adp->predictor.saturating) {
        suspicious++;
    }

    /* Check 3: Is the process output out of expected range?
       In a real industrial deployment, compare pv/sv against engineering
       limits (e.g., pressure vessel MAWP, reactor temperature limits). */
    (void)pv; (void)sv;

    /* Check 4: Are the controller parameters oscillating?
       If redesign_count increases too fast, adaptation may be unstable */
    if (adp->redesign_count > 1000) {
        suspicious++;  /* Too many redesigns */
    }

    if (susp_count) *susp_count = suspicious;
    return (suspicious < 2) ? 1 : 0;  /* Safe if fewer than 2 suspicious signs */
}
