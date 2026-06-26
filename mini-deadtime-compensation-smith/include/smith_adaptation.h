/**
 * @file smith_adaptation.h
 * @brief Adaptive Smith predictor with online model identification and retuning.
 *
 * Levels: L5 (Algorithms), L8 (Advanced Topics — time-varying, Lyapunov)
 *
 * The adaptive Smith predictor continuously updates the process model
 * using recursive least squares (RLS) and periodically redesigns the
 * primary controller. This addresses:
 *   - Process nonlinearity (operating point dependent parameters)
 *   - Slow process drift (aging, fouling, catalyst deactivation)
 *   - Unknown initial parameters (auto-commissioning)
 *
 * References:
 *   Hagglund, T. & Astrom, K.J. (2002) "Revisiting the Ziegler-Nichols
 *       step response method for PID control", J. Process Control
 *   Dumont, G.A. et al. (1993) "Concepts, methods and techniques in
 *       adaptive control", American Control Conference
 *   Landau, I.D. et al. (2011) "Adaptive Control", 2nd ed., Springer
 *       Chapter 5: "Parameter Adaptation Algorithms"
 *   Normey-Rico, J.E. & Camacho, E.F. (2008) "Dead-time compensators:
 *       A survey and new trends", Control Eng. Practice, 16(4), 407-428
 *
 * Course mapping:
 *   MIT 6.302: Adaptive control principles
 *   Stanford ENGR205: Self-tuning regulators
 *   CMU 24-677: Recursive parameter estimation
 *   Berkeley ME233: Adaptive control with Lyapunov methods
 */

#ifndef SMITH_ADAPTATION_H
#define SMITH_ADAPTATION_H

#include "smith_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L8: Adaptive Smith Predictor Core
 *===========================================================================*/

/**
 * @brief Initialize an adaptive Smith predictor.
 *
 * Sets up both the Smith predictor and the RLS identifier with
 * initial parameter estimates.
 *
 * @param adp           Adaptive Smith predictor state
 * @param K_init        Initial gain estimate
 * @param tau_init      Initial time constant estimate
 * @param theta_init    Initial dead time estimate
 * @param Ts            Sampling period
 * @param variant       Smith predictor variant
 * @param u_min         Lower saturation limit
 * @param u_max         Upper saturation limit
 * @return              0 on success
 */
int smith_adaptive_init(
    smith_adaptive_t *adp,
    double K_init, double tau_init, double theta_init,
    double Ts, smith_variant_t variant,
    double u_min, double u_max);

/**
 * @brief Execute one adaptive control step.
 *
 *  1. Run one Smith predictor step → get u(k)
 *  2. Update RLS identifier with u(k) and y(k)
 *  3. Check if model has changed significantly
 *  4. If yes, redesign controller and update Smith predictor model
 *  5. Apply robustness filter update if needed
 *
 * @param adp       Adaptive Smith predictor
 * @param setpoint  Current setpoint r(k)
 * @param pv        Current process measurement y(k)
 * @return          Controller output u(k)
 *
 * Complexity: O(1) + O(p^2) for RLS update (p=2 for FOPDT)
 */
double smith_adaptive_step(smith_adaptive_t *adp, double setpoint, double pv);

/**
 * @brief Enable or disable parameter adaptation.
 *
 * Adaptation should be disabled during:
 *   - Large disturbances (to avoid model corruption)
 *   - Manual operation
 *   - Process startup
 *
 * @param adp     Adaptive Smith predictor
 * @param enable  1 = enable, 0 = disable
 */
void smith_adaptive_set_enabled(smith_adaptive_t *adp, int enable);

/**
 * @brief Force immediate controller redesign from current RLS estimates.
 *
 * @param adp  Adaptive Smith predictor
 * @return     0 on success
 */
int smith_adaptive_redesign(smith_adaptive_t *adp);

/**
 * @brief Free resources associated with the adaptive Smith predictor.
 *
 * @param adp  Adaptive Smith predictor
 */
void smith_adaptive_destroy(smith_adaptive_t *adp);

/*===========================================================================
 * L5: Parameter Change Detection
 *===========================================================================*/

/**
 * @brief Detect significant change in process model parameters.
 *
 * Uses a CUSUM (cumulative sum) test to detect shifts in the prediction
 * error that indicate model mismatch requiring adaptation.
 *
 * Algorithm:
 *   S_high(k) = max(0, S_high(k-1) + e(k) - mu - beta)
 *   S_low(k)  = max(0, S_low(k-1) - e(k) + mu - beta)
 *   Detection if S_high > h or S_low > h
 *
 * Reference: Page, E.S. (1954) "Continuous inspection schemes", Biometrika
 *
 * @param prediction_error  Current model prediction error
 * @param state             CUSUM state array [S_high, S_low] (caller-maintained)
 * @param threshold         Detection threshold h
 * @param drift              Allowed drift beta
 * @return                  1 if change detected, 0 otherwise
 */
int smith_adaptive_detect_change(
    double prediction_error, double state[2],
    double threshold, double drift);

/**
 * @brief Estimate model uncertainty from prediction error statistics.
 *
 * Uses running statistics (mean, variance) of the prediction error
 * to estimate current model parameter uncertainty levels.
 *
 * @param prediction_errors  Array of recent prediction errors
 * @param n                  Number of points
 * @param K_out              Output: estimated gain uncertainty
 * @param tau_out            Output: estimated tau uncertainty
 * @param theta_out          Output: estimated theta uncertainty (seconds)
 */
void smith_adaptive_estimate_uncertainty(
    const double *prediction_errors, size_t n,
    double *K_out, double *tau_out, double *theta_out);

/*===========================================================================
 * L8: Gradient-Descent Adaptation
 *===========================================================================*/

/**
 * @brief Adapt controller gain using gradient descent on a cost function.
 *
 * Minimizes J(Kp) = 0.5 * e^2(k) (instantaneous squared error)
 * using the MIT rule (gradient descent):
 *   Kp(k+1) = Kp(k) - gamma * dJ/dKp
 *
 * The gradient is computed using the sensitivity derivative of the
 * closed-loop response to changes in Kp.
 *
 * Reference: Astrom & Wittenmark (1995) "Adaptive Control", Chapter 4
 *   "Model-Reference Adaptive Systems (MRAS)"
 *
 * @param adp       Adaptive Smith predictor
 * @param error     Current control error e(k)
 * @param gamma     Adaptation gain (learning rate, typical: 0.001-0.1)
 * @return          Updated Kp
 */
double smith_adaptive_gradient_Kp(
    smith_adaptive_t *adp, double error, double gamma);

/**
 * @brief Adapt integral time using gradient descent.
 *
 * Minimizes long-term offset by adjusting Ti.
 * Ti(k+1) = Ti(k) - gamma * e(k) * (1/Ti^2) * integral of e
 *
 * @param adp       Adaptive Smith predictor
 * @param error     Current error
 * @param gamma     Adaptation gain
 * @return          Updated Ti
 */
double smith_adaptive_gradient_Ti(
    smith_adaptive_t *adp, double error, double gamma);

/*===========================================================================
 * L8: Model-Reference Adaptive Smith Predictor
 *===========================================================================*/

/**
 * @brief Implement model-reference adaptive Smith predictor.
 *
 * Defines a reference model Gm(s) = 1/(T_ref*s + 1) representing the
 * desired closed-loop behavior. Adapts controller parameters to make
 * the actual closed-loop response match the reference model.
 *
 * The adaptation law (MIT rule):
 *   dKp/dt = -gamma * e_m * sign(K) * y_f
 * where e_m = y - y_m (model-following error), y_f is filtered output.
 *
 * @param adp         Adaptive Smith predictor
 * @param setpoint    Current setpoint
 * @param pv          Process measurement
 * @param ref_output  Reference model output y_m(k)
 * @param gamma       Adaptation gain
 */
void smith_adaptive_mras_step(
    smith_adaptive_t *adp,
    double setpoint, double pv,
    double *ref_output, double gamma);

/*===========================================================================
 * L8: Lyapunov-Based Adaptation Law
 *===========================================================================*/

/**
 * @brief Compute Lyapunov-stable adaptation law for Smith predictor.
 *
 * Uses Lyapunov's direct method to guarantee stable adaptation.
 *
 * For a FOPDT process, the adaptive law:
 *   dKp/dt = -gamma * e * y_f
 *   dTi/dt = -gamma * e * integral(e) / Ti^2
 *
 * with Lyapunov function V = e^2 + (Kp - Kp*)^2/gamma + (Ti - Ti*)^2/gamma
 * ensures V_dot <= 0 (asymptotic stability of the adaptive system).
 *
 * @param adp       Adaptive Smith predictor
 * @param error     Control error
 * @param y_filtered Filtered output for parameter update
 * @param gamma     Adaptation gain
 * @return          1 if V_dot <= 0 (stable), 0 if V_dot > 0
 */
int smith_adaptive_lyapunov_update(
    smith_adaptive_t *adp,
    double error, double y_filtered, double gamma);

/*===========================================================================
 * L5: Robust Adaptation (Dead-Zone, Projection)
 *===========================================================================*/

/**
 * @brief Apply dead-zone to adaptation to prevent drift during low excitation.
 *
 * When the control error is small, parameter adaptation can drift due to
 * noise. A dead-zone disables adaptation when |error| < dz_threshold.
 *
 * This implements the modified adaptation law:
 *   if |e| < dz:  parameters frozen
 *   if |e| >= dz: normal RLS + gradient update
 *
 * @param adp           Adaptive Smith predictor
 * @param dz_threshold  Dead-zone threshold (typically 2-5 * noise_std)
 */
void smith_adaptive_set_deadzone(smith_adaptive_t *adp, double dz_threshold);

/**
 * @brief Project parameters onto valid ranges after adaptation.
 *
 * Ensures adapted parameters remain physically meaningful:
 *   Kp > 0, Ti > 0, tau > 0, theta >= 0
 *
 * @param adp  Adaptive Smith predictor
 */
void smith_adaptive_project_parameters(smith_adaptive_t *adp);

/*===========================================================================
 * L7: Industrial Adaptive Supervision
 *===========================================================================*/

/**
 * @brief Supervisory logic for safe adaptive Smith predictor operation.
 *
 * Monitors adaptation health and reverts to fixed parameters if:
 *   - Prediction error variance exceeds threshold (poor model)
 *   - Controller parameters oscillate (adaptation instability)
 *   - Process enters abnormal operating region
 *
 * @param adp         Adaptive Smith predictor
 * @param pv          Process variable
 * @param sv          Setpoint variable
 * @param error_var   Running error variance
 * @param susp_count  Output: number of suspicious events
 * @return            1 if operation is safe, 0 if supervision triggered
 */
int smith_adaptive_supervision_check(
    const smith_adaptive_t *adp,
    double pv, double sv,
    double error_var, int *susp_count);

#ifdef __cplusplus
}
#endif

#endif /* SMITH_ADAPTATION_H */
