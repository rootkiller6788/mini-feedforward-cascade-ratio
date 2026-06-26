/**
 * @file smith_identification.h
 * @brief Process model identification for Smith predictor configuration.
 *
 * Levels: L5 (Algorithms/Methods), L6 (Canonical Problems)
 *
 * Accurate process model identification is critical for Smith predictor
 * performance. Identification methods include:
 *
 *   - Open-loop step-response (classic, requires process shutdown)
 *   - Relay feedback (closed-loop, can be done during operation)
 *   - Recursive least squares (online, for adaptive Smith predictor)
 *
 * References:
 *   Astrom & Hagglund (1995) "PID Controllers: Theory, Design, and Tuning"
 *       Chapter 2 "Process Models", pp. 18-52
 *   Ljung (1999) "System Identification", 2nd ed., Prentice Hall
 *       Chapter 7 "Recursive Identification Methods"
 *   Bi, Q. et al. (1999) "Robust identification of first-order plus
 *       dead-time model from step response", Control Eng. Practice, 7(1), 71-77
 *   Ahmed, S. et al. (2007) "Relay-based autotuning for Smith predictor",
 *       J. Process Control, 17(2), 135-147
 *
 * Course mapping:
 *   MIT 6.302: System identification fundamentals
 *   Stanford ENGR205: Step-response identification
 *   CMU 24-677: Recursive estimation
 *   Georgia Tech ECE 6550: Nonlinear identification methods
 */

#ifndef SMITH_IDENTIFICATION_H
#define SMITH_IDENTIFICATION_H

#include "smith_types.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L5: Open-Loop Step-Response Identification
 *===========================================================================*/

/**
 * @brief Identify FOPDT model from step response data.
 *
 * Uses the two-point method (Hoopes method):
 *
 *   1. Find the time t28 when output reaches 28.3% of full response
 *   2. Find the time t63 when output reaches 63.2% of full response
 *   3. tau = 1.5 * (t63 - t28)
 *   4. theta = t63 - tau
 *
 * Alternatively uses the area method when noise is significant:
 *   1. Compute area A0 under the normalized response curve
 *   2. tau = (A0 * K) / (step_size)
 *   3. theta = delay where response first exceeds noise threshold
 *
 * For SOPDT: uses least-squares curve fitting on the parameterized response.
 *
 * @param test_data  Step response data (time, output, input arrays)
 * @param model_out  Output FOPDT/SOPDT model parameters
 * @return           0 on success, -1 if data insufficient
 *
 * Theorem basis: For a FOPDT system G(s)=K*e^(-θs)/(τs+1), the step
 * response is y(t) = K*(1 - exp(-(t-θ)/τ)) for t >= θ, 0 otherwise.
 * The 28.3% and 63.2% points give unique τ identification.
 *
 * Complexity: O(n) single pass through data
 */
int smith_identify_step_fopdt(
    const smith_step_test_t *test_data,
    smith_process_model_t   *model_out);

/**
 * @brief Identify SOPDT model from step response data.
 *
 * Two methods depending on damping:
 *   - Overdamped (zeta >= 1): Two-point method matching t35 and t85
 *   - Underdamped (zeta < 1): Peak amplitude and period matching
 *
 * @param test_data  Step response data
 * @param model_out  Output SOPDT model
 * @return           0 on success, -1 on error
 */
int smith_identify_step_sopdt(
    const smith_step_test_t *test_data,
    smith_process_model_t   *model_out);

/**
 * @brief Compute model quality metric (normalized RMSE fit).
 *
 * FIT = 100 * (1 - ||y - y_model|| / ||y - mean(y)||)
 *
 * FIT > 90% : excellent model
 * FIT 70-90% : acceptable
 * FIT < 70% : model may need improvement
 *
 * @param test_data   Measured response
 * @param model       FOPDT model to evaluate
 * @param fit_percent Output: normalized RMSE fit [0, 100]
 * @return            0 on success
 */
int smith_validate_model_fit(
    const smith_step_test_t  *test_data,
    const smith_process_model_t *model,
    double *fit_percent);

/*===========================================================================
 * L5: Relay-Feedback Identification (Astrom-Hagglund method)
 *===========================================================================*/

/**
 * @brief Identify FOPDT model using relay feedback (closed-loop).
 *
 * The relay feedback experiment creates a limit cycle oscillation.
 * From the oscillation period Tu and amplitude a:
 *   - Ultimate gain: Ku = 4*d / (pi*a) where d = relay amplitude
 *   - Ultimate period: Tu (measured from oscillation)
 *   - FOPDT model can be fitted: K from open-loop test, tau, theta from Ku, Tu
 *
 * For Smith predictor configuration, the model is fitted using:
 *   tau = Tu / (2*pi) * sqrt((Ku*K)^2 - 1)
 *   theta = Tu / (2*pi) * (pi - arctan(sqrt((Ku*K)^2 - 1)))
 *
 * @param K_process   Known static gain of process (from open-loop test)
 * @param relay_amplitude Amplitude of relay (+/- d)
 * @param oscillation_period Measured oscillation period Tu (seconds)
 * @param oscillation_amplitude Measured output oscillation amplitude a
 * @param model_out   Output FOPDT model
 * @return            0 on success
 *
 * Reference: Astrom & Hagglund (1984) "Automatic tuning of simple
 * regulators", Automatica, 20(5), 645-651
 *   Table 1: Ku = 4d/(pi*a), K, tau, theta derivation
 */
int smith_identify_relay_fopdt(
    double K_process,
    double relay_amplitude,
    double oscillation_period,
    double oscillation_amplitude,
    smith_process_model_t *model_out);

/**
 * @brief Run a complete relay-feedback experiment with automatic detection.
 *
 * Emulates a relay feedback loop to identify ultimate gain and period.
 *
 * @param y_initial    Initial process output
 * @param u_initial    Initial process input
 * @param relay_d      Relay amplitude
 * @param K            Known process gain
 * @param Ts           Sampling period
 * @param model_out    Output model
 * @return             0 on success
 */
int smith_identify_relay_experiment(
    double y_initial, double u_initial,
    double relay_d, double K, double Ts,
    smith_process_model_t *model_out);

/*===========================================================================
 * L5: Recursive Least Squares (Online identification)
 *===========================================================================*/

/**
 * @brief Initialize Recursive Least Squares identifier for Smith model.
 *
 * RLS estimates the ARX model: y(k) = -a1*y(k-1) + b1*u(k-d-1) + b2*u(k-d-2)
 * for the delay-free portion of the process (after removing known dead time).
 *
 * @param rls           RLS identifier state
 * @param forgetting_f  Forgetting factor (0.95 = fast adapt, 0.999 = slow adapt)
 * @param initial_K     Initial gain estimate
 * @param initial_tau   Initial time constant estimate
 * @param initial_theta Initial dead time estimate
 */
void smith_rls_init(
    smith_rls_identifier_t *rls,
    double forgetting_f,
    double initial_K, double initial_tau, double initial_theta);

/**
 * @brief Update RLS estimate with new measurement.
 *
 * Algorithm: Standard RLS with exponential forgetting.
 *   P(k) = (1/lambda) * [P(k-1) - P(k-1)*phi*phi'*P(k-1) / (lambda + phi'*P*phi)]
 *   theta(k) = theta(k-1) + P(k)*phi*[y(k) - phi'*theta(k-1)]
 *
 * @param rls   RLS identifier
 * @param u     Current process input (after delay removal)
 * @param y     Current process output
 * @param Ts    Sampling period
 * @return      0 on success, -1 if not yet converged
 *
 * Complexity: O(p^2) where p = number of parameters (2 for FOPDT)
 *
 * Reference: Ljung (1999) "System Identification", Eq. 7.34-7.36
 */
int smith_rls_update(
    smith_rls_identifier_t *rls,
    double u, double y, double Ts);

/**
 * @brief Extract FOPDT parameters from RLS estimates.
 *
 * Converts discrete-time ARX parameters to continuous-time FOPDT.
 *
 * @param rls       RLS identifier (with converged estimates)
 * @param model_out Output FOPDT model
 * @return          0 on success, -1 if RLS not converged
 */
int smith_rls_to_fopdt(
    const smith_rls_identifier_t *rls,
    smith_process_model_t        *model_out);

/*===========================================================================
 * L6: Process Signal Processing (for identification)
 *===========================================================================*/

/**
 * @brief Detect steady-state condition in process data.
 *
 * Checks if the process output has settled within a specified tolerance
 * band for a minimum number of consecutive samples.
 *
 * @param data         Process output data
 * @param n            Number of samples
 * @param tolerance    Fractional tolerance band (e.g., 0.02 = 2%)
 * @param min_samples  Minimum consecutive samples within band
 * @return             1 if steady state detected, 0 otherwise
 */
int smith_detect_steady_state(
    const double *data, size_t n,
    double tolerance, size_t min_samples);

/**
 * @brief Pre-process step test data: remove outliers, filter noise.
 *
 * Applies median filter for outlier removal and exponential smoothing
 * for noise reduction prior to model identification.
 *
 * @param data_in   Raw process data
 * @param data_out  Filtered output (caller-allocated, same size as input)
 * @param n         Number of samples
 * @param alpha     Exponential smoothing factor (0 = no smoothing, 1 = max)
 */
void smith_preprocess_data(
    const double *data_in, double *data_out,
    size_t n, double alpha);

/**
 * @brief Estimate dead time from step response using threshold crossing.
 *
 * Dead time = time when output first exceeds noise_floor above baseline.
 * Uses linear interpolation between samples for fractional accuracy.
 *
 * @param time       Time vector
 * @param output     Output vector
 * @param n          Number of samples
 * @param threshold  Fraction of final change to detect (e.g., 0.01 = 1%)
 * @param theta_out  Estimated dead time (seconds)
 * @return           0 on success, -1 if dead time not detectable
 */
int smith_estimate_deadtime(
    const double *time, const double *output, size_t n,
    double threshold, double *theta_out);

/**
 * @brief Compute the relative dead-time ratio (dead time / time constant).
 *
 * This key metric determines whether a Smith predictor is beneficial:
 *   theta/tau < 0.1 : PI controller sufficient (easy process)
 *   0.1 <= theta/tau < 1.0 : Smith predictor beneficial
 *   theta/tau >= 1.0 : Smith predictor strongly recommended
 *
 * @param model  Process model
 * @return       theta / tau ratio
 */
double smith_deadtime_ratio(const smith_process_model_t *model);

#ifdef __cplusplus
}
#endif

#endif /* SMITH_IDENTIFICATION_H */
