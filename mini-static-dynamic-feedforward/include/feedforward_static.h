#ifndef FEEDFORWARD_STATIC_H
#define FEEDFORWARD_STATIC_H

#include "feedforward_defs.h"

/**
 * @file feedforward_static.h
 * @brief Static (steady-state) feedforward control API
 *
 * Knowledge: L1 Definitions, L2 Core Concepts, L5 Algorithms
 *
 * Static feedforward computes corrective action based on steady-state
 * mass/energy balance. It does NOT compensate for dynamics (time delays,
 * time constants) — that is handled by dynamic feedforward.
 *
 * Static FF law:
 *   u_ff = Kff * d(t) + bias
 *   where Kff = -Kd/Kp  (for ideal static disturbance rejection)
 *
 * Design principle (Seborg et al. §15.3):
 *   At steady state: PV = Kp * u + Kd * d
 *   For PV = 0:     u = -(Kd/Kp) * d  =>  Kff = -Kd/Kp
 */

/**
 * @brief Compute ideal static feedforward gain from process and disturbance models
 *
 * Kff = -Kd / Kp  (assuming both models have unity steady-state numerator)
 *
 * For FOPDT models: Gp(s) = Kp * e^(-theta*s)/(tau*s+1), Kp_ss = Kp
 *                    Gd(s) = Kd * e^(-theta_d*s)/(tau_d*s+1), Kd_ss = Kd
 *
 * @param process    Process model (FOPDT)
 * @param dist       Disturbance model
 * @param action     Controller action direction
 * @return Ideal static feedforward gain; returns 0.0 if Kp ~ 0
 */
double ff_static_gain_fopdt(const fopdt_t *process, const dist_model_t *dist,
                            action_t action);

/**
 * @brief Compute static feedforward gain for SOPDT process model
 *
 * Kff = -Kd / Kp  (steady-state gain ratio, SOPDT has same SS gain as FOPDT)
 *
 * @param process    Process model (SOPDT)
 * @param dist       Disturbance model
 * @param action     Controller action direction
 * @return Ideal static feedforward gain
 */
double ff_static_gain_sopdt(const sopdt_t *process, const dist_model_t *dist,
                            action_t action);

/**
 * @brief Compute static feedforward gain from arbitrary transfer functions
 *
 * Uses the DC gain (s=0) of both transfer functions.
 * For TF: G(0) = K * N(0)/D(0) = K * num[last] / den[last]
 *
 * @param Gp         Process transfer function
 * @param Gd         Disturbance transfer function  
 * @param action     Controller action direction
 * @return Ideal static feedforward gain
 */
double ff_static_gain_tf(const tf_t *Gp, const tf_t *Gd, action_t action);

/**
 * @brief Initialize static-only feedforward controller
 *
 * Sets mode to FF_MODE_STATIC, configures gain and clamping.
 *
 * @param ff         Feedforward controller to initialize
 * @param Kff        Static feedforward gain
 * @param bias       Output bias [engineering units]
 * @param out_min    Output low clamp
 * @param out_max    Output high clamp
 * @param action     Controller action (direct/reverse)
 * @param Ts         Sample time [seconds]
 */
void ff_static_init(feedforward_t *ff, double Kff, double bias,
                    double out_min, double out_max, action_t action, double Ts);

/**
 * @brief Execute one step of static feedforward
 *
 * u_ff(t) = action * Kff * d(t) + bias
 *
 * Applies output clamping if enabled.
 *
 * @param ff         Feedforward controller
 * @param d_meas     Current disturbance measurement
 * @return Total feedforward output (u_ff_static + bias)
 */
double ff_static_step(feedforward_t *ff, double d_meas);

/**
 * @brief Static feedforward with measurement filtering
 *
 * Applies first-order exponential filter to disturbance signal
 * before computing feedforward action.
 *
 * Filter: d_f[k] = alpha*d_meas + (1-alpha)*d_f[k-1]
 * where alpha = Ts/(tau_filter + Ts), tau_filter is the filter time constant
 *
 * @param ff           Feedforward controller
 * @param d_meas       Raw disturbance measurement
 * @param tau_filter   Filter time constant [s] (> 0)
 * @return Filtered feedforward output
 */
double ff_static_step_filtered(feedforward_t *ff, double d_meas, double tau_filter);

/**
 * @brief Static feedforward with signal quality check
 *
 * Validates disturbance measurement before using it.
 * Falls back to last valid value or bias-only output on bad signal.
 *
 * @param ff         Feedforward controller
 * @param meas       Disturbance measurement with quality metadata
 * @return Feedforward output; may fall back to bias on bad quality
 */
double ff_static_step_quality(feedforward_t *ff, const disturbance_meas_t *meas);

/**
 * @brief Compute disturbance rejection ratio for static feedforward
 *
 * At steady state, the disturbance rejection ratio (DRR) quantifies
 * how much the disturbance effect is reduced:
 *
 * DRR = 1 / |1 + Kp * Kff / Kd|
 *
 * For ideal feedforward (Kff = -Kd/Kp), DRR -> infinity (perfect rejection).
 * For no feedforward (Kff = 0), DRR = 1 (no rejection).
 *
 * @param Kp         Process steady-state gain
 * @param Kd         Disturbance steady-state gain
 * @param Kff        Implemented feedforward gain
 * @return Disturbance rejection ratio (>= 1.0)
 */
double ff_static_rejection_ratio(double Kp, double Kd, double Kff);

/**
 * @brief Evaluate sensitivity of static feedforward to gain mismatch
 *
 * When the implemented Kff differs from the ideal Kff_ideal = -Kd/Kp,
 * the residual steady-state error is:
 *
 * e_ss = (Kd + Kp*Kff_actual) / (1 + Kp*Kc) * d_step
 *
 * This function returns the relative residual: (Kd + Kp*Kff)/(Kd - Kp*Kd/Kp) = 1 + Kp*Kff/Kd
 *
 * @param Kp           Process gain
 * @param Kd           Disturbance gain
 * @param Kff_actual   Actual (potentially mismatched) feedforward gain
 * @return Relative residual (0 = perfect, 1 = no FF)
 */
double ff_static_mismatch_residual(double Kp, double Kd, double Kff_actual);

/**
 * @brief Compute bias from steady-state process data
 *
 * Given a desired operating point (d0, u0) where PV = sp:
 *   u0 = u_fb + u_ff
 * At steady state with PV=sp, u_fb = 0 in ideal case:
 *   u_ff = Kff * d0
 *   bias = u0 - Kff * d0  (ensures correct total output at design point)
 *
 * @param u0         Total controller output at design point
 * @param Kff        Static feedforward gain
 * @param d0         Disturbance at design point
 * @return Feedforward bias
 */
double ff_static_bias_from_operating_point(double u0, double Kff, double d0);

#endif