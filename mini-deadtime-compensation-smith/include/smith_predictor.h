/**
 * @file smith_predictor.h
 * @brief Core Smith Predictor API declarations.
 *
 * Level: L1-L6 complete coverage
 *
 * The Smith predictor addresses the fundamental challenge of controlling
 * processes with significant dead time. By using an internal model to
 * predict the delay-free output, the controller design reduces to a
 * delay-free problem for which standard methods apply.
 *
 * Structure:
 *   - Primary controller C(s) designed for delay-free process Gp(s)
 *   - Internal model Gp̃(s)·e^(-θ̃s) in parallel to actual process
 *   - Prediction feedback: yp = Gp̃(s)·u, ym = yp·e^(-θ̃s)
 *   - Control error to C(s): e = r - (yp + (y - ym))
 *     = r - [yp + (actual_output - delayed_model_output)]
 *
 * Key property: when model = process exactly, the characteristic equation
 *   1 + C(s)Gp(s) = 0  (dead-time-free!)
 *
 * Reference:
 *   Smith, O.J.M. (1957) Chem. Eng. Prog. 53(5), 217-219
 *   Smith, O.J.M. (1959) ISA Journal 6(2), 28-33
 *   Normey-Rico & Camacho (2007) "Control of Dead-time Processes", Springer
 *   Astrom & Hagglund (2005) "Advanced PID Control", Chapter 7
 *
 * Course mapping:
 *   MIT 6.302: Nyquist analysis with time delay
 *   Stanford ENGR205: Smith predictor structure
 *   Berkeley ME233: delay systems compensation
 *   CMU 24-677: dead-time compensation
 *   Purdue ME 575: industrial dead-time PID structures
 */

#ifndef SMITH_PREDICTOR_H
#define SMITH_PREDICTOR_H

#include "smith_types.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L1: Initialization — Create and configure Smith predictor
 *===========================================================================*/

/**
 * @brief Initialize a Smith predictor with FOPDT model.
 *
 * Configures all internal states and allocates the delay buffer.
 * The primary controller is initially set to open-loop (Kp=0).
 *
 * @param sp        Pointer to Smith predictor state (caller-allocated)
 * @param K         Process gain
 * @param tau       Process time constant (seconds, must be > 0)
 * @param theta     Dead time (seconds, must be >= 0)
 * @param Ts        Sampling period (seconds, must be > 0)
 * @param variant   Smith predictor variant
 * @param u_min     Actuator lower saturation limit
 * @param u_max     Actuator upper saturation limit
 * @return          0 on success, -1 on parameter error
 *
 * Complexity: O(theta/Ts) for buffer allocation
 */
int smith_predictor_init_fopdt(
    smith_predictor_t *sp,
    double K, double tau, double theta, double Ts,
    smith_variant_t variant,
    double u_min, double u_max);

/**
 * @brief Initialize a Smith predictor with SOPDT model.
 *
 * Supports both overdamped (zeta >= 1) and underdamped (0 < zeta < 1) models.
 *
 * @param sp        Pointer to Smith predictor state
 * @param K         Process gain
 * @param tau1      First time constant (seconds)
 * @param tau2      Second time constant (seconds, set 0 for underdamped form)
 * @param zeta      Damping ratio (used when tau2 == 0)
 * @param omega_n   Natural frequency (rad/s, used when tau2 == 0)
 * @param theta     Dead time (seconds)
 * @param Ts        Sampling period (seconds)
 * @param variant   Smith predictor variant
 * @param u_min     Lower saturation limit
 * @param u_max     Upper saturation limit
 * @return          0 on success, -1 on error
 *
 * Complexity: O(theta/Ts) for buffer allocation
 */
int smith_predictor_init_sopdt(
    smith_predictor_t *sp,
    double K, double tau1, double tau2,
    double zeta, double omega_n, double theta, double Ts,
    smith_variant_t variant,
    double u_min, double u_max);

/**
 * @brief Free resources associated with a Smith predictor.
 *
 * Deallocates the delay buffer. Safe to call with NULL.
 *
 * @param sp  Pointer to Smith predictor to destroy
 */
void smith_predictor_destroy(smith_predictor_t *sp);

/**
 * @brief Reset the Smith predictor to initial state.
 *
 * Clears integrator, derivative filter, model states, and delay buffer.
 * Preserves tuning parameters and model.
 *
 * @param sp  Pointer to Smith predictor
 */
void smith_predictor_reset(smith_predictor_t *sp);

/*===========================================================================
 * L2: Primary Controller Configuration (designed for delay-free process)
 *===========================================================================*/

/**
 * @brief Configure the primary controller using PI parameters.
 *
 * The controller C(s) is designed as if the process had NO dead time.
 * The Smith predictor handles the dead time internally.
 *
 * PI form (parallel/interacting):
 *   u(s) = Kp * [ (b*r(s) - yp_fb(s)) + 1/(Ti*s) * e(s) ]
 *
 * where yp_fb is the Smith-predicted feedback signal.
 *
 * @param sp   Smith predictor
 * @param Kp   Proportional gain
 * @param Ti   Integral time (seconds, 0 = disable integral)
 * @param b    Setpoint weight (typically 0..1)
 * @return     0 on success, -1 if Kp < 0 (non-minimum-phase special case)
 */
int smith_predictor_set_pi(smith_predictor_t *sp, double Kp, double Ti, double b);

/**
 * @brief Configure the primary controller with full PID.
 *
 * PID form with derivative on measurement (recommended for Smith predictor):
 *   u(s) = Kp * [ (b*r(s) - yp_fb(s)) + 1/(Ti*s)*e(s) + Td*s/(1+Td/N*s)*(c*r-yp_fb) ]
 *
 * Derivative on the predicted feedback avoids "derivative kick" from setpoint.
 *
 * @param sp   Smith predictor
 * @param Kp   Proportional gain
 * @param Ti   Integral time (seconds)
 * @param Td   Derivative time (seconds, 0 = no derivative)
 * @param N    Derivative filter factor (typically 8..20)
 * @param b    Setpoint weight on P
 * @param c    Setpoint weight on D
 * @return     0 on success
 */
int smith_predictor_set_pid(
    smith_predictor_t *sp,
    double Kp, double Ti, double Td, double N,
    double b, double c);

/**
 * @brief Configure the robustness filter (Filtered Smith predictor).
 *
 * Reference: Normey-Rico, J.E., Bordons, C., & Camacho, E.F. (1997)
 * "Improving the robustness of dead-time compensating PI controllers"
 * Control Engineering Practice, 5(6), 801-810.
 *
 * The robustness filter F_r(s) = 1/(T_r*s + 1) is inserted in the
 * prediction error path to attenuate model mismatch at high frequencies.
 *
 * Rule of thumb for FOPDT: T_r = theta/2 (balanced robustness/performance)
 *
 * @param sp       Smith predictor
 * @param Fr       Filter time constant (seconds, >= 0)
 * @return         0 on success
 */
int smith_predictor_set_robustness_filter(smith_predictor_t *sp, double Fr);

/**
 * @brief Set the discretization method for the internal model.
 *
 * @param sp     Smith predictor
 * @param method Discretization method
 * @return       0 on success
 */
int smith_predictor_set_disc_method(smith_predictor_t *sp, smith_disc_method_t method);

/*===========================================================================
 * L3: Core Smith Predictor Computation (one sample step)
 *===========================================================================*/

/**
 * @brief Execute one control iteration of the Smith predictor.
 *
 * This is the main runtime function called every Ts seconds.
 *
 * Algorithm:
 *   1. Update delay-free model: yp(k) = model_step(u(k-1))
 *   2. Push yp(k) into delay buffer
 *   3. Pop delayed value ym(k) = yp(k - d)
 *   4. Compute prediction error: ep = y(k) - ym(k)
 *   5. Filter prediction error (if robustness filter active)
 *   6. Form feedback to controller: y_fb = yp(k) + ep_filtered
 *   7. Compute primary controller output: u = C(r, y_fb)
 *   8. Apply saturation and rate limits
 *   9. Anti-windup back-calculation if saturated
 *  10. Update performance metrics
 *
 * @param sp       Smith predictor state
 * @param setpoint Current setpoint r(k)
 * @param pv       Current process variable measurement y(k)
 * @return         Controller output u(k), saturated to [u_min, u_max]
 *
 * Complexity: O(1) — fixed number of arithmetic operations per call
 */
double smith_predictor_step(smith_predictor_t *sp, double setpoint, double pv);

/**
 * @brief Get the current delay-free prediction (yp).
 *
 * This is the predicted process output IF there were no dead time.
 * Useful for diagnostics, display, and evaluating how well the model
 * captures process dynamics.
 *
 * @param sp   Smith predictor
 * @return     Current delay-free prediction yp(k)
 */
double smith_predictor_get_prediction(const smith_predictor_t *sp);

/**
 * @brief Get the current model mismatch indicator.
 *
 * prediction_error = y(k) - ym(k) = actual_pv - delayed_model_output
 *
 * A non-zero steady-state mismatch indicates either:
 *   - Model gain error (proportional to process gain mismatch)
 *   - Disturbance not captured by the model
 *
 * @param sp   Smith predictor
 * @return     Current model mismatch
 */
double smith_predictor_get_mismatch(const smith_predictor_t *sp);

/*===========================================================================
 * L5: Model Update (online model correction)
 *===========================================================================*/

/**
 * @brief Update the Smith predictor's internal model parameters.
 *
 * Allows online correction of model parameters without resetting
 * the controller state. The delay buffer is adjusted if dead time changes.
 *
 * @param sp     Smith predictor
 * @param K      New process gain
 * @param tau    New time constant
 * @param theta  New dead time
 * @return       0 on success, -1 if reallocation fails
 */
int smith_predictor_update_model(
    smith_predictor_t *sp, double K, double tau, double theta);

/**
 * @brief Apply model mismatch correction to controller parameters.
 *
 * Uses the detected model mismatch to adjust the primary controller gain.
 * Conservative approach: reduce Kp when model uncertainty is high.
 *
 * Theory: If actual gain K_actual differs from model gain K_model,
 * the effective loop gain changes. This function compensates:
 *   Kp_effective = Kp * (K_model / max(K_model, |prediction_error / u|))
 *
 * Reference: Skogestad & Postlethwaite (2005) "Multivariable Feedback
 * Control", Chapter 7 on robustness to model uncertainty.
 *
 * @param sp   Smith predictor
 */
void smith_predictor_apply_mismatch_correction(smith_predictor_t *sp);

/*===========================================================================
 * L6: Performance Metrics Computations
 *===========================================================================*/

/**
 * @brief Compute Smith predictor performance metrics.
 *
 * Evaluates the control loop performance based on collected IAE, ISE,
 * ITAE metrics and computes robustness margins.
 *
 * @param sp       Smith predictor (with accumulated metrics)
 * @param perf_out Output performance structure (filled by this function)
 */
void smith_predictor_compute_performance(
    const smith_predictor_t *sp, smith_performance_t *perf_out);

/**
 * @brief Reset all accumulated performance metrics.
 *
 * @param sp  Smith predictor
 */
void smith_predictor_reset_metrics(smith_predictor_t *sp);

/*===========================================================================
 * L7: Industrial Interface Functions
 *===========================================================================*/

/**
 * @brief Map Smith predictor parameters to Modbus registers.
 *
 * Configures the register mapping for PLC/SCADA communication.
 *
 * @param sp       Smith predictor
 * @param modbus   Output Modbus register map
 * @param base_reg Starting register address
 */
void smith_predictor_map_modbus(
    const smith_predictor_t *sp, smith_modbus_map_t *modbus, uint16_t base_reg);

/**
 * @brief Set Smith predictor parameter from Modbus register value.
 *
 * @param sp     Smith predictor
 * @param reg    Register address
 * @param value  IEEE 754 float value as uint32_t
 * @return       0 on success, -1 if register not writable
 */
int smith_predictor_write_modbus(
    smith_predictor_t *sp, uint16_t reg, uint32_t value);

/**
 * @brief Map Smith predictor to OPC UA nodes.
 *
 * @param sp     Smith predictor
 * @param opcua  Output OPC UA node map
 * @param ns_idx Namespace index
 * @param base   Base node ID
 */
void smith_predictor_map_opcua(
    const smith_predictor_t *sp, smith_opcua_map_t *opcua,
    uint32_t ns_idx, uint32_t base);

#ifdef __cplusplus
}
#endif

#endif /* SMITH_PREDICTOR_H */
