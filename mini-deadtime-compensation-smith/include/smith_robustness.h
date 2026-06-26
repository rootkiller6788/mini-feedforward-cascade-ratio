/**
 * @file smith_robustness.h
 * @brief Robustness analysis and sensitivity computations for Smith predictor.
 *
 * Levels: L4 (Engineering Laws), L8 (Advanced Topics)
 *
 * The Smith predictor's main weakness is sensitivity to model mismatch.
 * This module quantifies robustness margins and provides tools for
 * analyzing stability under parameter uncertainty.
 *
 * Key robustness concepts:
 *   - Structured uncertainty: gain, time constant, dead time errors
 *   - Small-gain theorem: robust stability condition
 *   - Mu-analysis: structured singular value for worst-case uncertainty
 *   - Robust performance: combined stability + performance criteria
 *
 * References:
 *   Morari, M. & Zafiriou, E. (1989) "Robust Process Control"
 *       Chapter 3: "Internal Model Control", Section 3.7: Robustness
 *   Skogestad, S. & Postlethwaite, I. (2005) "Multivariable Feedback Control"
 *       Chapter 7: "Uncertainty and Robustness", Section 7.5: SISO Uncertainty
 *   Normey-Rico, J.E. & Camacho, E.F. (2008) "Dead-time compensators: A survey"
 *       Control Engineering Practice, 16(4), 407-428
 *   Palmor, Z.J. (1996) "Time-delay compensation", The Control Handbook, CRC
 *       Section on robustness of Smith predictor
 *
 * Course mapping:
 *   MIT 6.302: Robustness margins, sensitivity functions
 *   Stanford ENGR205: Model uncertainty quantification
 *   Berkeley ME233: Structured singular value analysis
 *   CMU 24-677: Robust stability theory
 *   Georgia Tech ECE 6550: Lyapunov-based robustness
 */

#ifndef SMITH_ROBUSTNESS_H
#define SMITH_ROBUSTNESS_H

#include "smith_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L4: Sensitivity Functions
 *===========================================================================*/

/**
 * @brief Compute the closed-loop sensitivity function magnitude.
 *
 * The sensitivity S(s) = 1 / [1 + C(s) * G(s)] measures disturbance
 * attenuation and is the key robustness indicator.
 *
 * For the nominal Smith predictor (perfect model):
 *   S(s) = 1 / [1 + C(s) * Gp(s)]   (delay-free sensitivity)
 *
 * At a specific frequency w, |S(jw)| quantifies:
 *   - |S| < 1    : disturbance attenuation (good)
 *   - |S| > 1    : disturbance amplification (bad)
 *   - Ms = max|S| : peak sensitivity (robustness measure)
 *
 * Good design: Ms < 2.0 (ISA recommendation: Ms < 1.7)
 *
 * @param model   Process model
 * @param Kp      Controller gain
 * @param Ti      Integral time (0 = P only)
 * @param w       Frequency (rad/s) at which to evaluate
 * @return        |S(jw)|
 */
double smith_robustness_sensitivity(
    const smith_process_model_t *model,
    double Kp, double Ti, double w);

/**
 * @brief Compute the complementary sensitivity function magnitude.
 *
 * T(s) = C(s)G(s) / [1 + C(s)G(s)] = 1 - S(s)
 *
 * For the Smith predictor with perfect model:
 *   T(s) = C(s)Gp(s) / [1 + C(s)Gp(s)]  (delay-free complementary sensitivity)
 *
 * Peak complementary sensitivity Mt = max|T(jw)| relates to overshoot.
 * Mt < 1.25 recommended for good robustness.
 *
 * @param model   Process model
 * @param Kp      Controller gain
 * @param Ti      Integral time
 * @param w       Frequency (rad/s)
 * @return        |T(jw)|
 */
double smith_robustness_complementary_sensitivity(
    const smith_process_model_t *model,
    double Kp, double Ti, double w);

/**
 * @brief Compute the peak sensitivity Ms = max_w |S(jw)|.
 *
 * Uses frequency sweep to find the maximum sensitivity peak.
 * Ms is the key robustness metric:
 *   Ms < 1.4 : excellent robustness
 *   Ms < 1.7 : good (ISA standard)
 *   Ms < 2.0 : acceptable
 *   Ms > 2.0 : poor robustness, retune required
 *
 * Relationship to gain margin: GM >= Ms/(Ms-1)
 * Relationship to phase margin: PM >= 2*arcsin(1/(2*Ms))
 *
 * @param model    Process model
 * @param Kp       Controller gain
 * @param Ti       Integral time
 * @param Td       Derivative time
 * @param w_start  Start of frequency sweep (rad/s)
 * @param w_end    End of frequency sweep
 * @param n_points Number of frequency points
 * @return         Peak sensitivity Ms
 */
double smith_robustness_peak_sensitivity(
    const smith_process_model_t *model,
    double Kp, double Ti, double Td,
    double w_start, double w_end, int n_points);

/*===========================================================================
 * L4: Robust Stability under Model Mismatch
 *===========================================================================*/

/**
 * @brief Check robust stability for gain uncertainty.
 *
 * For multiplicative input uncertainty: G_actual(s) = G(s) * (1 + W_I(s)*Delta)
 * with |Delta| <= 1. The robust stability condition (small-gain theorem):
 *   |T_I(jw)| < 1/|W_I(jw)|  for all w
 *
 * For gain-only uncertainty: |Delta_K| <= delta_K_max
 *   Stable if: |T(jw)| < 1/delta_K  for all w
 *
 * @param model       Process model
 * @param Kp          Controller gain
 * @param Ti          Integral time
 * @param delta_K     Relative gain uncertainty (e.g., 0.2 = ±20%)
 * @param w_start     Start frequency
 * @param w_end       End frequency
 * @param n_points    Number of points
 * @return            1 if robustly stable, 0 if violated
 */
int smith_robustness_gain_uncertainty(
    const smith_process_model_t *model,
    double Kp, double Ti,
    double delta_K,
    double w_start, double w_end, int n_points);

/**
 * @brief Check robust stability for dead-time uncertainty.
 *
 * Dead-time uncertainty is more challenging because e^(-j*w*theta)
 * causes phase lag that increases with frequency.
 *
 * Robust stability condition for dead-time uncertainty ±delta_theta:
 *   The phase margin must accommodate: delta_theta * w_gc (radians)
 *   at the gain-crossover frequency w_gc.
 *
 * Conservative bound: the Smith predictor is robustly stable if:
 *   delta_theta < PM * pi/(180 * w_gc)
 * where PM is the nominal phase margin in degrees.
 *
 * @param model        Process model
 * @param Kp           Controller gain
 * @param Ti           Integral time
 * @param delta_theta  Dead time uncertainty (seconds)
 * @return             1 if robustly stable, 0 if violated
 */
int smith_robustness_deadtime_uncertainty(
    const smith_process_model_t *model,
    double Kp, double Ti,
    double delta_theta);

/**
 * @brief Compute combined robust stability margin.
 *
 * Handles simultaneous gain AND dead-time uncertainty using:
 *   1 + C(s)G(s) * (1 + Delta_K)/(1 - Delta_K * approximation) != 0
 *
 * Uses the structured singular value (mu) concept for the worst-case
 * combination of uncertainties.
 *
 * @param model       Process model
 * @param Kp          Controller gain
 * @param Ti          Integral time
 * @param delta_K     Gain uncertainty
 * @param delta_theta Dead-time uncertainty (seconds)
 * @param w_start     Start frequency
 * @param w_end       End frequency
 * @param n_points    Points
 * @return            1 if robustly stable, 0 otherwise
 */
int smith_robustness_combined(
    const smith_process_model_t *model,
    double Kp, double Ti,
    double delta_K, double delta_theta,
    double w_start, double w_end, int n_points);

/*===========================================================================
 * L4: Stability Margins
 *===========================================================================*/

/**
 * @brief Compute exact gain and phase margins for the nominal system.
 *
 * For the Smith predictor with perfect model, the closed-loop
 * characteristic equation is dead-time-free:
 *   1 + C(jw) * Gp(jw) = 0
 *
 * Gain margin: GM = 20*log10(1/|C(jw180)*Gp(jw180)|) dB
 * where w180 is the phase crossover frequency (arg = -180 deg).
 *
 * Phase margin: PM = 180 + arg(C(jw_gc)*Gp(jw_gc)) degrees
 * where w_gc is the gain crossover frequency (|C*Gp| = 1).
 *
 * @param model    Process model
 * @param Kp       Controller gain
 * @param Ti       Integral time
 * @param Td       Derivative time (0 = PI)
 * @param gm_db    Output: gain margin (dB)
 * @param pm_deg   Output: phase margin (degrees)
 * @param w_gc     Output: gain crossover frequency (rad/s)
 * @param w_pc     Output: phase crossover frequency (rad/s)
 * @return         0 on success
 */
int smith_robustness_margins(
    const smith_process_model_t *model,
    double Kp, double Ti, double Td,
    double *gm_db, double *pm_deg,
    double *w_gc, double *w_pc);

/**
 * @brief Compute the maximum extra delay the system can tolerate.
 *
 * Delay margin: the additional dead time D that can be added before
 * the system goes unstable.
 *
 * D = PM / w_gc   (delay margin in seconds)
 * where PM is phase margin in radians, w_gc is gain-crossover freq.
 *
 * For the Smith predictor, this tells how much dead-time error
 * the controller can tolerate.
 *
 * @param model    Process model
 * @param Kp       Controller gain
 * @param Ti       Integral time
 * @param Td       Derivative time
 * @return         Delay margin (seconds)
 */
double smith_robustness_delay_margin(
    const smith_process_model_t *model,
    double Kp, double Ti, double Td);

/*===========================================================================
 * L8: Lyapunov-Based Robustness (advanced)
 *===========================================================================*/

/**
 * @brief Check asymptotic stability using a Lyapunov function.
 *
 * For the discretized delay-free model with state x(k):
 *   V(x) = x' * P * x  is a Lyapunov function if P > 0 and
 *   A' * P * A - P < 0  (discrete Lyapunov equation)
 *
 * Solves the discrete Lyapunov equation for the closed-loop system matrix.
 *
 * @param model  Process model
 * @param Kp     Controller gain
 * @param Ti     Integral time
 * @param Ts     Sampling period
 * @return       1 if Lyapunov stable, 0 if not conclusively stable
 */
int smith_robustness_lyapunov_stable(
    const smith_process_model_t *model,
    double Kp, double Ti, double Ts);

/*===========================================================================
 * L8: Monte Carlo Robustness Verification
 *===========================================================================*/

/**
 * @brief Monte Carlo robustness verification under random model mismatch.
 *
 * Randomly samples model parameters (K, tau, theta) within their uncertainty
 * bounds and checks closed-loop stability for each sample.
 *
 * @param model       Nominal process model
 * @param Kp          Controller gain
 * @param Ti          Integral time
 * @param delta_K     Relative gain uncertainty
 * @param delta_tau   Relative tau uncertainty
 * @param delta_theta Dead-time uncertainty (seconds)
 * @param n_samples   Number of Monte Carlo samples
 * @param stable_count Output: number of stable samples
 * @return            Fraction of stable samples [0, 1]
 *
 * Complexity: O(n_samples * model_dimension)
 */
double smith_robustness_monte_carlo(
    const smith_process_model_t *model,
    double Kp, double Ti,
    double delta_K, double delta_tau, double delta_theta,
    int n_samples, int *stable_count);

/*===========================================================================
 * L4: Nyquist Criterion for Smith Predictor
 *===========================================================================*/

/**
 * @brief Evaluate the Nyquist criterion for the nominal Smith predictor.
 *
 * For 1 + C(s)Gp(s) = 0, checks encirclements of the -1 point.
 *
 * @param model    Process model
 * @param Kp       Controller gain
 * @param Ti       Integral time
 * @param n_points Number of frequency points on Nyquist contour
 * @param n_encirclements Output: number of counterclockwise encirclements of -1
 * @return         1 if Nyquist stable, 0 if unstable
 */
int smith_robustness_nyquist_criterion(
    const smith_process_model_t *model,
    double Kp, double Ti,
    int n_points, int *n_encirclements);

#ifdef __cplusplus
}
#endif

#endif /* SMITH_ROBUSTNESS_H */
