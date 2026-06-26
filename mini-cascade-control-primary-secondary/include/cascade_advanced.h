/**
 * @file cascade_advanced.h
 * @brief Advanced Cascade Control — Gain Scheduling, Adaptive Control,
 *        Stochastic Robustness, Smith Predictor Cascade
 *
 * Module: mini-cascade-control-primary-secondary
 * Knowledge Coverage: L5 Algorithms, L7 Industrial Applications, L8 Advanced
 *
 * Advanced topics in cascade control beyond standard PID cascade:
 * - Gain-scheduled cascade for nonlinear processes
 * - Adaptive cascade with online model identification
 * - Smith predictor in cascade for dead-time dominant processes
 * - Stochastic robustness analysis (Monte Carlo)
 * - Feedforward augmentation of cascade
 *
 * Reference: Astrom & Wittenmark, Adaptive Control (2008)
 *            Smith, ISA Journal (1957) — Smith Predictor
 *            Seborg et al. (2016) Chapter 16
 * Curriculum: MIT 6.302, Stanford ENGR205, CMU 24-677
 */

#ifndef CASCADE_ADVANCED_H
#define CASCADE_ADVANCED_H

#include "cascade_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L5: Gain-Scheduled Cascade Control
 *
 * When process gain varies significantly with operating point,
 * gain scheduling adjusts PID parameters based on a scheduling
 * variable (typically the primary PV or SP).
 *
 * Implementation:
 *   1. Define breakpoints: (sched_var_1, Kc_1, Ti_1), ..., (sched_var_N, Kc_N, Ti_N)
 *   2. At runtime: linear interpolation of Kc, Ti between breakpoints
 *   3. Apply interpolated parameters to the primary controller
 * ========================================================================= */

/**
 * cascade_gain_schedule_init: Initialize gain scheduling table.
 *
 * @param state            Adaptive state to initialize
 * @param sched_points     Array of [scheduling_variable, gain] pairs
 * @param ti_points        Array of [scheduling_variable, Ti] pairs
 * @param num_points       Number of scheduling breakpoints (max 10)
 * @return                 0 on success, -1 on failure
 *
 * Complexity: O(n)
 */
int cascade_gain_schedule_init(cascade_adaptive_state_t *state,
                                const double sched_points[][2],
                                const double ti_points[][2],
                                int num_points);

/**
 * cascade_gain_schedule_update: Compute scheduled PID parameters.
 *
 * Linear interpolation between nearest breakpoints:
 *   Kc(var) = Kc_i + (Kc_{i+1} - Kc_i)/(var_{i+1} - var_i) * (var - var_i)
 *
 * Extrapolation: constant beyond first/last breakpoint (clamped).
 *
 * @param state       Gain schedule state
 * @param sched_var   Current scheduling variable value
 * @param kp_out      Output: scheduled proportional gain
 * @param ti_out      Output: scheduled integral time
 * @return            0 on success, -1 if no schedule defined
 *
 * Complexity: O(log n) with binary search (or O(n) linear)
 */
int cascade_gain_schedule_update(const cascade_adaptive_state_t *state,
                                  double sched_var,
                                  double *kp_out, double *ti_out);

/* =========================================================================
 * L8: Adaptive Cascade Control with Online Identification
 *
 * Recursive least squares (RLS) with exponential forgetting for
 * online identification of the primary process model with the
 * secondary loop closed. Adapts PID parameters using SIMC rules.
 *
 * The adaptation works as follows:
 *   1. At each primary sample, collect (u, y) pair
 *   2. Update RLS parameter estimates (theta)
 *   3. Convert parameter estimates to FOPDT model
 *   4. Recompute PID parameters using SIMC
 *   5. Apply new parameters with bumpless transition
 * ========================================================================= */

/**
 * cascade_rls_init: Initialize Recursive Least Squares estimator.
 *
 * RLS model: y(k) = a1*y(k-1) + b1*u(k-d-1) + b2*u(k-d-2)
 * Parameter vector theta = [a1, b1, b2]^T
 *
 * @param lambda        Forgetting factor (0.95-0.999)
 * @param p0_diag       Initial covariance diagonal value
 *                      (large = fast adaptation, small = slow)
 * @param theta         Output: initial parameter estimate (set to zeros)
 *
 * Complexity: O(1)
 */
void cascade_rls_init(double *theta, double *P,
                       int n_params, double lambda, double p0_diag);

/**
 * cascade_rls_update: Single RLS update step.
 *
 * P(k) = (1/lambda) * (P(k-1) - P(k-1)*phi*phi^T*P(k-1) / (lambda + phi^T*P(k-1)*phi))
 * theta(k) = theta(k-1) + P(k)*phi*(y - phi^T*theta(k-1))
 *
 * @param theta   Parameter estimate (in-place update, length n)
 * @param P       Covariance matrix (in-place update, n x n, stored row-major)
 * @param phi     Regressor vector (length n)
 * @param y       Measurement
 * @param lambda  Forgetting factor
 * @param n       Number of parameters
 * @return        Prediction error (y - y_hat)
 *
 * Complexity: O(n^2)
 * Reference: Ljung, System Identification (1999)
 */
double cascade_rls_update(double *theta, double *P,
                           const double *phi, double y,
                           double lambda, int n);

/**
 * cascade_model_from_rls: Convert RLS parameters to FOPDT model.
 *
 * From discrete-time parameters:
 *   y(k) + a1*y(k-1) = b1*u(k-d-1)
 *
 * Convert to continuous-time FOPDT:
 *   tau = -Ts / ln(-a1)
 *   K = b1 / (1 + a1)
 *   theta estimated from delay index d
 *
 * @param theta  RLS parameter estimate
 * @param ts     Sample time
 * @param delay  Estimated dead time in samples
 * @param model  Output FOPDT model
 * @return       0 on success, -1 if conversion fails
 *
 * Complexity: O(1)
 */
int cascade_model_from_rls(const double *theta, double ts, int delay,
                            cascade_fopdt_model_t *model);

/**
 * cascade_adaptive_update: Full adaptive cascade update cycle.
 *
 * 1. Collect measurement
 * 2. RLS update
 * 3. Convert to FOPDT
 * 4. SIMC tuning
 * 5. Apply new parameters
 *
 * @return 0 on success
 *
 * Complexity: O(n^2) per update (n = number of RLS parameters)
 */
int cascade_adaptive_update(cascade_adaptive_state_t *state,
                             cascade_pid_controller_t *primary_pid,
                             double setpoint, double pv, double output,
                             double ts);

/* =========================================================================
 * L8: Smith Predictor in Cascade
 *
 * For processes with significant dead time (theta/tau > 1), the
 * Smith predictor can be integrated into the cascade structure.
 *
 * The Smith predictor consists of:
 *   1. Process model without dead time: G_m(s) = K/(tau*s + 1)
 *   2. Process model with dead time: G_m_theta(s) = G_m(s) * exp(-theta*s)
 *   3. The controller acts on the delay-free prediction
 *
 * In cascade, the Smith predictor is typically applied to the primary
 * loop, while the secondary loop uses standard PI/PID with the fast
 * inner process.
 * ========================================================================= */

/**
 * cascade_smith_predictor_init: Initialize Smith predictor state.
 *
 * @param model     FOPDT model of primary process
 * @param buffer    Ring buffer for delayed signal (length = ceil(theta/ts))
 * @param buf_size  Size of delay buffer
 * @param sp_state  Smith predictor state (output)
 *
 * Complexity: O(1)
 */
void cascade_smith_predictor_init(const cascade_fopdt_model_t *model,
                                   double *buffer, int buf_size,
                                   cascade_pid_state_t *sp_state);

/**
 * cascade_smith_predictor_update: Update Smith predictor.
 *
 *   1. Delay-free model output: ym = model_step(u)
 *   2. Delayed model output: ym_theta = delay(ym, theta)
 *   3. Prediction: yp = ym + (y - ym_theta)
 *   4. Controller uses yp instead of y
 *
 * @param input  Current control input
 * @param pv     Current process variable
 * @param sp_state Smith predictor state (in-place)
 * @return       Predicted process variable for controller
 *
 * Complexity: O(1) per sample
 *
 * Reference: Smith, ISA Journal (1957)
 *            Palmor, Automatica (1996) — Robust Smith Predictor
 */
double cascade_smith_predictor_update(double input, double pv,
                                       cascade_pid_state_t *sp_state,
                                       const cascade_fopdt_model_t *model,
                                       double *delay_buffer, int buf_size);

/* =========================================================================
 * L8: Monte Carlo Robustness Analysis for Cascade
 *
 * Process model parameters have uncertainty. Monte Carlo analysis
 * samples from parameter distributions to estimate the probability
 * of instability or performance degradation.
 * ========================================================================= */

/**
 * cascade_monte_carlo_robustness: Monte Carlo stability analysis.
 *
 * Samples from uniform distributions around nominal model parameters
 * and estimates the probability of meeting stability criteria.
 *
 * @param pri_nominal  Nominal primary model
 * @param sec_nominal  Nominal secondary model
 * @param pri_pid      Primary PID parameters
 * @param sec_pid      Secondary PID parameters
 * @param uncertainty_pct Parameter uncertainty (+/- %)
 * @param n_samples    Number of Monte Carlo samples
 * @param stable_count Output: number of stable samples
 * @param mean_gm_db   Output: mean gain margin (dB)
 * @param mean_pm_deg  Output: mean phase margin (deg)
 * @param min_gm_db    Output: worst-case gain margin (dB)
 * @return             0 on success
 *
 * Complexity: O(n_samples * n_freq)
 */
int cascade_monte_carlo_robustness(
    const cascade_fopdt_model_t *pri_nominal,
    const cascade_fopdt_model_t *sec_nominal,
    const cascade_pid_params_t *pri_pid,
    const cascade_pid_params_t *sec_pid,
    double uncertainty_pct, int n_samples,
    int *stable_count, double *mean_gm_db,
    double *mean_pm_deg, double *min_gm_db);

/* =========================================================================
 * L8: Lyapunov-Based Cascade Stability (Time-Varying)
 *
 * For nonlinear or time-varying cascade systems, Lyapunov analysis
 * can provide stability guarantees beyond linear frequency-domain
 * methods.
 * ========================================================================= */

/**
 * cascade_lyapunov_quadratic: Check quadratic stability of cascade.
 *
 * For a closed-loop cascade described as a linear parameter-varying
 * (LPV) system, check if there exists a common quadratic Lyapunov
 * function V(x) = x^T*P*x such that V_dot < 0 for all vertices.
 *
 * This function checks the LMI (Linear Matrix Inequality) condition
 * using a simple eigenvalue test on the closed-loop A matrices.
 *
 * @param A_vertices  Array of closed-loop A matrices (n_states x n_states)
 * @param n_vertices  Number of LPV vertices
 * @param n_states    State dimension
 * @param P           Output: Lyapunov matrix (if stable, n_states x n_states)
 * @return            0 if quadratically stable, -1 if unstable, 1 if uncertain
 *
 * Complexity: O(n_vertices * n_states^3) for eigenvalue computation
 */
int cascade_lyapunov_quadratic(const double *A_vertices,
                                int n_vertices, int n_states,
                                double *P);

/**
 * cascade_lyapunov_inner_loop: Inner loop Lyapunov analysis.
 *
 * For the secondary loop modeled as:
 *   dx/dt = Ax + Bu
 *   y = Cx
 *
 * Closed-loop with PI: u = Kp*(r-y) + Ki*integral(r-y)
 *
 * Augmented state: z = [x; integral_error]
 * Check stability of augmented A matrix.
 *
 * @return 0 if stable, -1 if unstable
 *
 * Complexity: O(n^3)
 */
int cascade_lyapunov_inner_loop(const double *A, const double *B,
                                 const double *C, int n_states,
                                 double kp, double ki);

/* =========================================================================
 * L7: Industrial Cascade Health Monitoring
 * ========================================================================= */

/**
 * cascade_health_monitor: Comprehensive cascade health assessment.
 *
 * Monitors:
 * - Primary/secondary PV oscillation (stiction, aggressive tuning)
 * - Control valve travel (excessive movement?)
 * - Mode changes (frequent manual interventions?)
 * - Cascade utilization (is secondary in cascade or local?)
 * - Performance degradation over time
 *
 * @param config      Cascade configuration
 * @param perf        Filled with current performance metrics
 * @return            Health status per NAMUR NE107
 *
 * Complexity: O(1)
 */
cascade_health_t cascade_health_monitor(const cascade_config_t *config,
                                         cascade_performance_t *perf);

/**
 * cascade_calculate_stiction_index: Estimate valve stiction from loop data.
 *
 * Stiction causes limit cycles in cascade loops. It can be detected
 * from PV-OP patterns: when the output changes direction, the PV
 * should respond. If there's a deadband, stiction is present.
 *
 * Based on the Horch (1999) method and Yamashita (2006) improvements.
 *
 * @param pv_history Array of PV values (length n)
 * @param op_history Array of OP values (length n)
 * @param n          Number of data points
 * @return           Stiction index (0 = no stiction, >0.5 = significant)
 *
 * Complexity: O(n)
 * Reference: Horch, Automatica (1999)
 */
double cascade_calculate_stiction_index(const double *pv_history,
                                         const double *op_history,
                                         int n);

/**
 * cascade_calculate_oscillation_index: Detect oscillations in loop.
 *
 * Uses autocorrelation to detect periodic behavior.
 * High oscillation index -> likely aggressive tuning or stiction.
 *
 * @param data   Array of PV or OP values
 * @param n      Number of data points
 * @return       Oscillation index (0 = steady, 1 = strongly oscillatory)
 *
 * Complexity: O(n * lag_max)
 */
double cascade_calculate_oscillation_index(const double *data, int n);

/* =========================================================================
 * L8: Balanced Cascade Tuning (Girvan-Newman社区检测思路)
 *
 * Balanced tuning considers both loops simultaneously rather than
 * sequentially, using constrained optimization to minimize IAE
 * while respecting gain/phase margin constraints.
 * ========================================================================= */

/**
 * cascade_balanced_optimize: Simultaneous optimization of cascade PID.
 *
 * Minimizes: w1*IAE_primary + w2*IAE_secondary
 * Subject to: GM > 6 dB, PM > 45 deg for both loops
 *
 * Uses Nelder-Mead simplex for constrained optimization with
 * barrier penalty functions.
 *
 * @param pri_model  Primary process model
 * @param sec_model  Secondary process model
 * @param w1         Weight for primary IAE
 * @param w2         Weight for secondary IAE
 * @param result     Output: optimized tuning
 * @return           0 on success
 *
 * Complexity: O(N_iter * n_freq) where N_iter ~ 500-1000
 */
int cascade_balanced_optimize(const cascade_fopdt_model_t *pri_model,
                               const cascade_fopdt_model_t *sec_model,
                               double w1, double w2,
                               cascade_tuning_result_t *result);

/* =========================================================================
 * L7: Application-Specific Cascade Templates
 * ========================================================================= */

/**
 * cascade_template_boiler_drum: Pre-tuned cascade for boiler drum level.
 *
 * Typical application: drum level (primary) / feedwater flow (secondary)
 *
 * Secondary FOPDT: K=0.5, tau=3s, theta=1s (valve + flow)
 * Primary FOPDT: K=0.02%/tph, tau=60s, theta=0s (drum mass balance)
 *
 * Configures both loops with industrial-standard parameters
 * for a 500 MW coal-fired boiler.
 */
void cascade_template_boiler_drum(cascade_config_t *config);

/**
 * cascade_template_heat_exchanger: Pre-tuned cascade for heat exchanger.
 *
 * Primary: outlet temperature  (slow, tau ~ 30-120s)
 * Secondary: steam/flow valve   (fast, tau ~ 2-5s)
 *
 * Typical in refinery crude preheat trains, chemical reactor
 * temperature control.
 */
void cascade_template_heat_exchanger(cascade_config_t *config);

/**
 * cascade_template_distillation_column: Pre-tuned cascade for distillation.
 *
 * Primary: tray temperature (composition inferential)
 * Secondary: reflux or reboiler flow
 *
 * Typical in petrochemical distillation (ethylene, propylene splitters).
 */
void cascade_template_distillation_column(cascade_config_t *config);

/**
 * cascade_template_reactor_temp: Pre-tuned cascade for exothermic reactor.
 *
 * Primary: reactor temperature
 * Secondary: jacket/cooling flow
 *
 * Critical for runaway prevention in batch and semi-batch reactors.
 */
void cascade_template_reactor_temp(cascade_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* CASCADE_ADVANCED_H */