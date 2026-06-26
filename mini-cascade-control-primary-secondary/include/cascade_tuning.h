/**
 * @file cascade_tuning.h
 * @brief Cascade Control Tuning Algorithms — ZN, Cohen-Coon, SIMC, Lambda
 *
 * Module: mini-cascade-control-primary-secondary
 * Knowledge Coverage: L4 Engineering Laws, L5 Algorithms/Methods
 *
 * Cascade tuning follows a sequential strategy:
 *   1. Tune the secondary (inner) loop first (fastest)
 *   2. With secondary in auto/cascade, tune the primary (outer) loop
 *   3. Verify overall cascade stability and performance
 *
 * The secondary loop must be 3-10x faster than the primary loop for
 * effective cascade control. If this condition is not met, cascade
 * provides no benefit over single-loop control.
 *
 * Reference: Ziegler & Nichols (1942), Cohen & Coon (1953),
 *            Skogestad (2003) SIMC, Astrom & Hagglund (1995)
 * Curriculum: MIT 6.302, Purdue ME575, RWTH Aachen
 */

#ifndef CASCADE_TUNING_H
#define CASCADE_TUNING_H

#include "cascade_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L4: Ziegler-Nichols Tuning for Cascade
 *
 * ZN tuning uses ultimate gain (Ku) and ultimate period (Pu) from
 * closed-loop relay feedback or open-loop step response.
 *
 * PID: Kc = 0.6*Ku, Ti = Pu/2, Td = Pu/8
 * PI:  Kc = 0.45*Ku, Ti = Pu/1.2
 * ========================================================================= */

/**
 * cascade_tune_zn_secondary: ZN tuning for secondary (inner) loop.
 *
 * The secondary loop typically uses a PI controller with fast response.
 * ZN PI tuning: Kc = 0.45*Ku, Ti = Pu/1.2
 *
 * @param Ku           Ultimate gain of secondary process
 * @param Pu           Ultimate period (seconds)
 * @param sample_time  Secondary loop sample time (seconds)
 * @param result       Output tuning parameters
 * @return             0 on success, -1 on invalid inputs
 *
 * Complexity: O(1)
 */
int cascade_tune_zn_secondary(double Ku, double Pu, double sample_time,
                               cascade_pid_params_t *result);

/**
 * cascade_tune_zn_primary: ZN tuning for primary (outer) loop.
 *
 * The primary loop uses PID for better setpoint tracking.
 * ZN PID tuning: Kc = 0.6*Ku, Ti = Pu/2, Td = Pu/8
 *
 * @param Ku           Ultimate gain of primary process (with secondary closed)
 * @param Pu           Ultimate period (seconds)
 * @param sample_time  Primary loop sample time (seconds)
 * @param result       Output tuning parameters
 * @return             0 on success, -1 on invalid inputs
 */
int cascade_tune_zn_primary(double Ku, double Pu, double sample_time,
                             cascade_pid_params_t *result);

/* =========================================================================
 * L4: Cohen-Coon Tuning for Cascade
 *
 * Cohen-Coon uses the FOPDT model parameters (K, tau, theta) directly.
 * Designed for 1/4 decay ratio response.
 *
 * PI:  Kc = (1/K)*(tau/theta)*(0.9 + theta/(12*tau))
 *      Ti = theta*(30 + 3*theta/tau)/(9 + 20*theta/tau)
 * PID: Kc = (1/K)*(tau/theta)*(4/3 + theta/(4*tau))
 *      Ti = theta*(32 + 6*theta/tau)/(13 + 8*theta/tau)
 *      Td = theta*4/(11 + 2*theta/tau)
 * ========================================================================= */

/**
 * cascade_tune_cohen_coon_pi: Cohen-Coon PI tuning from FOPDT model.
 *
 * Used primarily for the secondary loop where PI control is standard.
 *
 * Complexity: O(1)
 * Ref: Cohen & Coon, Trans. ASME (1953)
 */
int cascade_tune_cohen_coon_pi(const cascade_fopdt_model_t *model,
                                cascade_pid_params_t *result);

/**
 * cascade_tune_cohen_coon_pid: Cohen-Coon PID tuning from FOPDT model.
 *
 * Used for the primary loop where PID provides better performance.
 *
 * Complexity: O(1)
 */
int cascade_tune_cohen_coon_pid(const cascade_fopdt_model_t *model,
                                 cascade_pid_params_t *result);

/* =========================================================================
 * L5: SIMC (Simple Internal Model Control) Tuning for Cascade
 *
 * Skogestad (2003) SIMC rules provide robust tuning with adjustable
 * closed-loop time constant (tau_c).
 *
 * For FOPDT:
 *   Kc = (1/K) * (tau / (tau_c + theta))
 *   Ti = min(tau, 4*(tau_c + theta))
 *   Td = 0  (PI control, unless dead-time dominant)
 *
 * For Integrating:
 *   Kc = (1/K) * (1 / (tau_c + theta))
 *   Ti = 4*(tau_c + theta)
 *
 * The tuning parameter tau_c can be adjusted:
 *   tau_c = theta    (tight/smooth tradeoff recommended by Skogestad)
 *   tau_c = 1.5*theta (more robust)
 *   tau_c = theta/2   (more aggressive)
 * ========================================================================= */

/**
 * cascade_tune_simc_secondary: SIMC tuning for secondary loop.
 *
 * Default tau_c = theta for balanced performance.
 *
 * Complexity: O(1)
 * Ref: Skogestad, J. Process Control (2003)
 */
int cascade_tune_simc_secondary(const cascade_fopdt_model_t *model,
                                 double tau_c,
                                 cascade_pid_params_t *result);

/**
 * cascade_tune_simc_primary: SIMC tuning for primary loop.
 *
 * The primary process model is the effective model seen with
 * the secondary loop closed. tau_c is typically chosen larger
 * than for the secondary to avoid interaction.
 *
 * Complexity: O(1)
 */
int cascade_tune_simc_primary(const cascade_fopdt_model_t *effective_model,
                               double tau_c,
                               cascade_pid_params_t *result);

/* =========================================================================
 * L5: Lambda Tuning for Cascade
 *
 * Lambda tuning (also called IMC tuning) provides explicit control
 * over closed-loop response speed via the lambda parameter.
 *
 * PI (FOPDT):  Kc = tau / (K*(lambda + theta))
 *              Ti = tau
 * PID (FOPDT): Kc = (tau + theta/2) / (K*(lambda + theta/2))
 *              Ti = tau + theta/2
 *              Td = tau*theta/(2*tau + theta)
 *
 * Typical lambda = 3*theta (robust) or lambda = theta (fast)
 * ========================================================================= */

/**
 * cascade_tune_lambda_pi: Lambda PI tuning for secondary loop.
 *
 * @param model  FOPDT process model
 * @param lambda Desired closed-loop time constant (seconds)
 * @param result Output tuning parameters
 * @return       0 on success
 *
 * Complexity: O(1)
 */
int cascade_tune_lambda_pi(const cascade_fopdt_model_t *model,
                            double lambda,
                            cascade_pid_params_t *result);

/**
 * cascade_tune_lambda_pid: Lambda PID tuning for primary loop.
 *
 * Complexity: O(1)
 */
int cascade_tune_lambda_pid(const cascade_fopdt_model_t *model,
                             double lambda,
                             cascade_pid_params_t *result);

/* =========================================================================
 * L5: Cascade Sequential Tuning Strategy
 * ========================================================================= */

/**
 * cascade_tune_sequential: Full cascade sequential tuning.
 *
 * Algorithm:
 *   1. Identify secondary process model (step test with primary in manual)
 *   2. Tune secondary loop using SIMC or ZN
 *   3. Place secondary in auto/cascade mode
 *   4. Identify effective primary process model (step test on primary SP)
 *   5. Tune primary loop using SIMC or Lambda
 *   6. Verify gain and phase margins for overall cascade
 *
 * @param secondary_model  Secondary (inner) loop FOPDT model
 * @param primary_model    Primary (outer) loop FOPDT model
 * @param method           Tuning method selection:
 *                          0 = Ziegler-Nichols, 1 = Cohen-Coon,
 *                          2 = SIMC, 3 = Lambda
 * @param result           Output: full cascade tuning parameters
 * @return                 0 on success, -1 on failure
 *
 * Complexity: O(1)
 */
int cascade_tune_sequential(const cascade_fopdt_model_t *secondary_model,
                             const cascade_fopdt_model_t *primary_model,
                             int method,
                             cascade_tuning_result_t *result);

/* =========================================================================
 * L5: Frequency-Domain Tuning for Cascade
 * ========================================================================= */

/**
 * cascade_tune_phase_margin: Tune PI/PID for specified phase margin.
 *
 * Uses the FOPDT model to compute the frequency response and
 * finds the gain that achieves the desired phase margin.
 *
 * Phase margin specification:
 *   > 60 deg: very robust (conservative)
 *   45-60 deg: standard industrial tuning
 *   30-45 deg: aggressive
 *   < 30 deg: too aggressive (risk of oscillations)
 *
 * @param model          FOPDT process model
 * @param phase_margin   Desired phase margin (degrees)
 * @param use_pid        true=PID, false=PI
 * @param result         Output tuning parameters
 * @return               0 on success
 *
 * Complexity: O(log(1/eps)) — binary search for gain
 */
int cascade_tune_phase_margin(const cascade_fopdt_model_t *model,
                               double phase_margin, bool use_pid,
                               cascade_pid_params_t *result);

/**
 * cascade_tune_max_sensitivity: Tune for maximum sensitivity constraint.
 *
 * Ms = max_w |1/(1 + G_c(w)*G_p(w))| is the maximum sensitivity.
 * Ms < 2.0 is standard; Ms < 1.7 is robust; Ms < 1.4 is very robust.
 *
 * Uses Nelder-Mead simplex search to find PID parameters that meet
 * the Ms constraint while maximizing integral gain (Ki = Kc/Ti).
 *
 * Complexity: O(N * log(1/eps)) where N = number of iterations
 * Ref: Astrom & Hagglund (1995), Section 5.8
 */
int cascade_tune_max_sensitivity(const cascade_fopdt_model_t *model,
                                  double ms_max,
                                  cascade_pid_params_t *result);

/* =========================================================================
 * L5: Cascade Performance Verification
 * ========================================================================= */

/**
 * cascade_tuning_verify_margins: Verify gain and phase margins.
 *
 * Computes the open-loop frequency response of the cascade system
 * and determines stability margins.
 *
 * @param primary_params    Primary controller parameters
 * @param secondary_params  Secondary controller parameters
 * @param model             Cascade system model
 * @param stability         Output stability analysis
 * @return                  0 if stable, -1 if unstable, 1 if marginal
 *
 * Complexity: O(N_freq)
 */
int cascade_tuning_verify_margins(const cascade_pid_params_t *primary_params,
                                   const cascade_pid_params_t *secondary_params,
                                   const cascade_system_model_t *model,
                                   cascade_stability_t *stability);

/**
 * cascade_tuning_compare_methods: Compare multiple tuning methods.
 *
 * Simulates each method and computes performance metrics (IAE, ISE, etc.)
 * to help select the best tuning for a given application.
 *
 * @param model     Process model
 * @param num_methods Number of methods to compare
 * @param methods   Array of method indices
 * @param results   Output array of tuning results (must be pre-allocated)
 * @return          Index of best method per ISE criterion, -1 on error
 */
int cascade_tuning_compare_methods(const cascade_fopdt_model_t *model,
                                    int num_methods,
                                    const int *methods,
                                    cascade_tuning_result_t *results);

#ifdef __cplusplus
}
#endif

#endif /* CASCADE_TUNING_H */