/**
 * @file smith_tuning.h
 * @brief Primary controller tuning methods for the Smith Predictor.
 *
 * Levels: L4 (Engineering Laws), L5 (Algorithms/Methods)
 *
 * All methods design the primary controller C(s) for the delay-free
 * portion Gp(s) of the Smith predictor. Since dead time is structurally
 * separated, tuning reduces to standard PID design for dead-time-free plants.
 *
 * Methods:
 *   L4: IMC (Rivera-Morari-Skogestad 1986) — analytical PI/PID from H2-optimal
 *   L4: SIMC (Skogestad 2003) — industry-standard simplified IMC
 *   L4: Dahlin/Lambda — desired closed-loop synthesis
 *   L5: ISE-optimal via Golden-Section search
 *   L5: Robustness filter tuning — Normey-Rico criteria
 *   L4: Stability margin evaluation
 *   L5: Setpoint prefilter design
 *
 * References:
 *   Rivera, Morari, Skogestad (1986) IEC Proc. Des. Dev. 25,252-265
 *   Skogestad (2003) J. Process Control 13, 291-309
 *   Dahlin (1968) Instruments and Control Systems 41(6), 77-83
 */

#ifndef SMITH_TUNING_H
#define SMITH_TUNING_H

#include "smith_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** SIMC PI tuning for delay-free FOPDT model inside Smith predictor.
 *  Kp = (1/K)*(tau/tau_c), Ti = min(tau, 4*tau_c)
 *  Ref: Skogestad (2003), Table 4. */
int smith_tune_simc_pi(
    const smith_process_model_t *model, double tau_c,
    double *Kp_out, double *Ti_out);

/** SIMC PID tuning for SOPDT model.
 *  Kp = (1/K)*(tau1/tau_c), Ti = tau1, Td = tau2. */
int smith_tune_simc_pid(
    const smith_process_model_t *model, double tau_c,
    double *Kp_out, double *Ti_out, double *Td_out);

/** IMC PI tuning: Kp = tau/(K*lambda), Ti = tau.
 *  lambda controls trade-off (>= theta/2 forced). */
int smith_tune_imc_pi(
    const smith_process_model_t *model, double lambda_imc,
    double *Kp_out, double *Ti_out);

/** IMC PID tuning for SOPDT: Kp=(tau1+tau2)/(K*lambda),
 *  Ti=tau1+tau2, Td=tau1*tau2/(tau1+tau2). */
int smith_tune_imc_pid(
    const smith_process_model_t *model, double lambda_imc,
    double *Kp_out, double *Ti_out, double *Td_out);

/** Dahlin/Lambda PI tuning.
 *  Kp = tau/(K*(lambda + theta/2)), Ti = tau. */
int smith_tune_lambda_pi(
    const smith_process_model_t *model, double lambda,
    double *Kp_out, double *Ti_out);

/** Tune robustness filter: Fr = theta*(delta_K + delta_theta/tau).
 *  Ref: Normey-Rico et al. (1997). */
int smith_tune_robustness_filter(
    const smith_process_model_t *model, double delta_K, double delta_theta,
    double *Fr_out);

/** ISE-optimal PI using Golden-Section + grid search.
 *  Minimizes J = int e^2 dt via step-response simulation. */
int smith_tune_optimal_ise_pi(
    const smith_process_model_t *model, double Kp_init, double Ti_init,
    double *Kp_opt_out, double *Ti_opt_out, double *J_min_out);

/** Compute stability margins (GM, PM) via frequency sweep.
 *  Delegates to smith_robustness_margins(). */
int smith_tune_stability_margins(
    const smith_process_model_t *model, double Kp, double Ti, double Td,
    double *gm_db, double *pm_deg);

/** Quick stability check: GM >= 6 dB and PM >= 30 deg. */
int smith_tune_is_stable(
    double Kp, double Ti, const smith_process_model_t *model);

/** Design 2-DOF setpoint prefilter time constant T_ref.
 *  Default: T_ref = Ti, bounded to [tau/10, 10*tau]. */
void smith_tune_setpoint_filter(
    const smith_process_model_t *model, double Ti, double *T_ref_out);

#ifdef __cplusplus
}
#endif

#endif /* SMITH_TUNING_H */
