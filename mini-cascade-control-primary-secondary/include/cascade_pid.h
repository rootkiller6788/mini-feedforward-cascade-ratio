/**
 * @file cascade_pid.h
 * @brief Cascade PID Controller — Algorithm Implementation & Bumpless Transfer
 *
 * Module: mini-cascade-control-primary-secondary
 * Knowledge Coverage: L2 Core Concepts, L3 Engineering Structures, L5 Algorithms
 *
 * Implements the PID algorithm variants used in cascade control:
 * - Positional (absolute) form: u(k) = Kp*e(k) + Ki*sum(e) + Kd*(e(k)-e(k-1))/Ts
 * - Velocity (incremental) form: du(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + ...
 * - Bumpless transfer between manual/auto and primary/secondary modes
 * - Anti-windup with tracking mode for cascade integration
 *
 * In cascade, the secondary loop uses velocity-form PID for bumpless integration
 * with primary output changes. The primary loop may use positional form with
 * anti-windup that tracks the secondary setpoint limits.
 *
 * Reference: Astrom & Hagglund, PID Controllers (1995), Chapter 3 & 8
 *            Seborg et al., Process Dynamics and Control (2016), Chapter 8
 * Curriculum: MIT 6.302, Stanford ENGR205, Berkeley ME233
 */

#ifndef CASCADE_PID_H
#define CASCADE_PID_H

#include "cascade_types.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L2: PID Controller Initialization & Configuration
 * ========================================================================= */

/**
 * cascade_pid_init: Initialize a PID controller with default parameters.
 *
 * Sets all state variables to zero, configures standard parallel form,
 * reverse-acting direction, and clamping anti-windup.
 *
 * Complexity: O(1)
 */
void cascade_pid_init(cascade_pid_controller_t *pid,
                      double kp, double ti, double td,
                      double ts, double out_min, double out_max);

/**
 * cascade_pid_reset: Reset PID controller state.
 *
 * Zeroes the integral accumulator, filters, and error history.
 * Used during mode transitions (manual->auto) for bumpless transfer.
 *
 * Complexity: O(1)
 */
void cascade_pid_reset(cascade_pid_controller_t *pid);

/**
 * cascade_pid_set_params: Update PID parameters at runtime.
 *
 * Allows gain-scheduled or adaptive parameter updates without
 * resetting the controller state.
 *
 * Complexity: O(1)
 */
void cascade_pid_set_params(cascade_pid_controller_t *pid,
                            double kp, double ti, double td);

/* =========================================================================
 * L3: PID Algorithm Variants
 * ========================================================================= */

/**
 * cascade_pid_update_positional: Positional-form PID update.
 *
 * Standard ISA/IEC form (parallel non-interacting):
 *   u(t) = Kp * [e(t) + 1/Ti * integral(e(t)) + Td * de(t)/dt]
 *
 * Discretized using backward Euler for integral, backward difference
 * for derivative, with derivative on PV (not error) to avoid derivative
 * kick on setpoint changes.
 *
 * Derivative filter: Tf = Td / N, typically N = 8-20
 *
 * @param pid       PID controller state and parameters
 * @param setpoint  Desired setpoint value
 * @param pv        Current process variable
 * @return          Controller output (scaled to [out_min, out_max])
 *
 * Complexity: O(1)
 * Reference: IEC 61131-3 PID function block specification
 */
double cascade_pid_update_positional(cascade_pid_controller_t *pid,
                                      double setpoint, double pv);

/**
 * cascade_pid_update_velocity: Velocity-form (incremental) PID update.
 *
 * Computes du(k) instead of u(k), suitable for cascaded configurations
 * where the secondary loop must respond incrementally to primary output
 * changes.
 *
 * du(k) = Kp * (e(k) - e(k-1)) + (Kp*Ts/Ti) * e(k)
 *       + (Kp*Td/Ts) * (2*PV(k-1) - PV(k) - PV(k-2))
 *
 * The velocity form is inherently bumpless: no integral accumulator
 * to wind up or require reset during mode changes.
 *
 * Complexity: O(1)
 */
double cascade_pid_update_velocity(cascade_pid_controller_t *pid,
                                    double setpoint, double pv);

/* =========================================================================
 * L3: Bumpless Transfer Mechanisms
 * ========================================================================= */

/**
 * cascade_pid_bumpless_manual_to_auto: Transition from manual to automatic.
 *
 * When switching from manual to auto mode, the integral term is back-calculated
 * so that the controller output equals the manual output at the transition
 * instant. This prevents a "bump" in the control signal.
 *
 * Method (positional form):
 *   I = u_manual - Kp*(beta*SP - PV) - D
 *
 * Method (velocity form):
 *   No action needed — the incremental form is inherently bumpless.
 *
 * Complexity: O(1)
 */
void cascade_pid_bumpless_manual_to_auto(cascade_pid_controller_t *pid,
                                          double manual_output,
                                          double setpoint, double pv);

/**
 * cascade_pid_bumpless_auto_to_manual: Transition from auto to manual.
 *
 * Records the current output for smooth transition. In industrial practice,
 * the output is typically held at the last automatic value.
 *
 * Complexity: O(1)
 */
double cascade_pid_bumpless_auto_to_manual(cascade_pid_controller_t *pid);

/* =========================================================================
 * L3: Setpoint Filtering & Handling
 * ========================================================================= */

/**
 * cascade_pid_setpoint_filter: First-order exponential setpoint filter.
 *
 * SP_filtered(k) = SP_filtered(k-1) + (Ts/Tf) * (SP_raw - SP_filtered(k-1))
 *
 * Reduces derivative kick and allows smooth setpoint transitions.
 * Used especially for cascade primary SP changes to avoid shocking
 * the secondary loop.
 *
 * Complexity: O(1)
 */
double cascade_pid_setpoint_filter(cascade_pid_controller_t *pid,
                                    double raw_sp, double filter_tau);

/**
 * cascade_pid_setpoint_ramp: Ramp-limited setpoint change.
 *
 * Limits the rate of setpoint change to `max_rate` per sample period.
 * Prevents large setpoint steps from causing output saturation.
 *
 * Complexity: O(1)
 */
double cascade_pid_setpoint_ramp(double current_sp, double target_sp,
                                  double max_rate, double ts);

/* =========================================================================
 * L5: Anti-Windup Methods for Cascade PID
 * ========================================================================= */

/**
 * cascade_pid_anti_windup_clamping: Clamping anti-windup.
 *
 * Simplest method: stop integrating when output saturates.
 * When output == output_max and error > 0: freeze integral
 * When output == output_min and error < 0: freeze integral
 *
 * Complexity: O(1)
 */
void cascade_pid_anti_windup_clamping(cascade_pid_controller_t *pid, double error);

/**
 * cascade_pid_anti_windup_back_calc: Back-calculation anti-windup.
 *
 * Adds a tracking term to the integrator:
 *   I(k) += (Ts/Tt) * (u_sat - u_unsat)
 *
 * where Tt is the tracking time constant (typically Tt = Ti or Tt = sqrt(Ti*Td)).
 * When the output is not saturated, u_sat == u and the tracking term is zero.
 *
 * Complexity: O(1)
 */
void cascade_pid_anti_windup_back_calc(cascade_pid_controller_t *pid,
                                        double u_unsat, double u_sat,
                                        double tracking_time);

/**
 * cascade_pid_anti_windup_conditional: Conditional integration anti-windup.
 *
 * Only integrates when:
 * 1. Control error is small (|e| < threshold)
 * 2. Output is not saturated
 * This prevents the integrator from "charging up" when the error
 * is large and saturation is likely.
 *
 * Complexity: O(1)
 */
void cascade_pid_anti_windup_conditional(cascade_pid_controller_t *pid,
                                          double error, double threshold,
                                          bool output_saturated);

/* =========================================================================
 * L5: Cascade-Specific PID Functions
 * ========================================================================= */

/**
 * cascade_pid_setpoint_weighting: Two-degree-of-freedom setpoint.
 *
 * PID with setpoint weights (beta, gamma):
 *   u = Kp * [beta*SP - PV + 1/Ti*I + Td*(gamma*SP - PV)/dt]
 *
 * beta = 0: eliminates proportional kick on SP change
 * gamma = 0: eliminates derivative kick on SP change
 *
 * Default: beta=1.0, gamma=0.0 (derivative on PV only)
 *
 * Complexity: O(1)
 */
double cascade_pid_setpoint_weighting(cascade_pid_controller_t *pid,
                                       double setpoint, double pv,
                                       double beta, double gamma);

/**
 * cascade_pid_compute_ideal: Ideal (non-interacting) PID form.
 *
 *   G_c(s) = Kc * (1 + 1/(Ti*s) + Td*s)
 *
 * This is the standard ISA form. Suitable for primary loop in cascade.
 *
 * Complexity: O(1)
 */
double cascade_pid_compute_ideal(cascade_pid_controller_t *pid,
                                  double setpoint, double pv);

/**
 * cascade_pid_compute_series: Series (interacting) PID form.
 *
 *   G_c(s) = Kc * (1 + 1/(Ti*s)) * (Td*s + 1)
 *
 * The derivative and integral interact. Historically used in pneumatic
 * controllers. Still found in some DCS implementations.
 *
 * Conversion to parallel form:
 *   Kc_parallel = Kc*(1 + Td/Ti), Ti_parallel = Ti + Td, Td_parallel = Ti*Td/(Ti+Td)
 *
 * Complexity: O(1)
 */
double cascade_pid_compute_series(cascade_pid_controller_t *pid,
                                   double setpoint, double pv);

/**
 * cascade_pid_output_tracking: Track external signal for cascade integration.
 *
 * When in cascade, the primary loop output tracks the secondary PV
 * when secondary is in local/manual mode. This allows bumpless
 * reconnection of cascade.
 *
 * Complexity: O(1)
 */
void cascade_pid_output_tracking(cascade_pid_controller_t *pid,
                                  double tracking_value);

#ifdef __cplusplus
}
#endif

#endif /* CASCADE_PID_H */