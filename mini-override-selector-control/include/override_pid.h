/**
 * @file override_pid.h
 * @brief Override/Selector Control — PID Controller with Tracking
 *
 * Implements PID controllers specifically designed for override
 * selector schemes. Each inactive controller tracks the active
 * controller's output using external reset feedback to prevent
 * integral windup and enable bumpless transfer.
 *
 * Reference:
 *   Astrom, K.J. & Hagglund, T. (1995). PID Controllers: Theory,
 *   Design, and Tuning (2nd ed.). ISA. Chapter 3: Controller
 *   Implementation, Section 3.5: Bumpless Transfer.
 *
 *   IEC 61131-3 (2013). "Programmable Controllers — Part 3:
 *   Programming Languages." Section: PID Function Block with
 *   External Reset (TRACK/TRACK_REF inputs).
 *
 * Knowledge Coverage:
 *   L2 — Core Concepts: anti-windup, external reset, bumpless transfer
 *   L3 — Engineering Structures: PID discretization, tracking implementation
 *   L4 — Engineering Laws: IEC 61131-3 PID function block specification
 *   L5 — Algorithms: Positional/velocity PID, back-calculation, tracking
 *
 * Course Alignment:
 *   MIT 6.302: Digital PID implementation
 *   Stanford ENGR205: Industrial PID with constraints
 *   RWTH Aachen: IEC 61131-3 PID function block
 *   Tsinghua: PID数字实现与无扰切换
 */

#ifndef OVERRIDE_PID_H
#define OVERRIDE_PID_H

#include "override_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L2 — PID Controller Initialization
 * ========================================================================= */

/**
 * Initialize a PID controller for use in an override scheme.
 *
 * Sets up the controller state for positional PID with derivative
 * on PV (not error) and trapezoidal (Tustin) discretization for
 * the integral term.
 *
 * @param ctrl   Pointer to controller instance
 * @param Kc     Proportional gain
 * @param Ti     Integral time [s]
 * @param Td     Derivative time [s]
 * @param N      Derivative filter coefficient
 * @param Ts     Sampling period [s]
 * @param u_min  Output low limit
 * @param u_max  Output high limit
 * @param tr_gain External reset tracking gain (1/Tt)
 */
void override_pid_init(override_controller_t *ctrl,
                       double Kc, double Ti, double Td,
                       double N, double Ts,
                       double u_min, double u_max,
                       double tr_gain);

/**
 * Initialize a PID controller from a parameters structure
 *
 * @param ctrl   Pointer to controller instance
 * @param params Pointer to PID parameter structure
 */
void override_pid_init_from_params(override_controller_t *ctrl,
                                   const override_pid_params_t *params);

/* =========================================================================
 * L3 — PID Update Methods
 * ========================================================================= */

/**
 * Positional PID update with tracking and anti-windup
 *
 * Computes the standard ISA-form PID output:
 *   u(t) = Kc * [b*r(t) - y(t) + 1/Ti*∫e(τ)dτ + Td*(c*dr/dt - dy/dt)]
 *
 * When the controller is inactive (tracking), the integral term is
 * set so that the output equals the tracking value (external reset).
 *
 * Anti-windup: Back-calculation method when output saturates.
 *
 * @param ctrl   Controller instance
 * @param sp     Setpoint
 * @param pv     Process variable
 * @param dt     Time step since last update [s] (usually = Ts)
 * @return PID output (clamped to [u_min, u_max])
 */
double override_pid_update(override_controller_t *ctrl,
                           double sp, double pv, double dt);

/**
 * Velocity (incremental) PID update
 *
 * Computes Δu(t) = u(t) - u(t-1) using the velocity form.
 * This form inherently provides bumpless transfer and anti-windup
 * since the integral term is not accumulated.
 *
 * The velocity form is:
 *   Δu = Kc * [Δe + Ts/Ti*e + Td/Ts*(Δe - Δe_prev)]  (Euler approximation)
 *
 * @param ctrl   Controller instance
 * @param sp     Setpoint
 * @param pv     Process variable
 * @param dt     Time step [s]
 * @return Incremental output Δu (may be negative)
 */
double override_pid_update_velocity(override_controller_t *ctrl,
                                    double sp, double pv, double dt);

/**
 * PID update with setpoint weighting (2-DOF)
 *
 * Uses independent setpoint weights for proportional (b) and
 * derivative (c) terms. When b=1, c=1: standard PID.
 * When b=0, c=0: I-PD (derivative only on PV).
 *
 * The 2-DOF form:
 *   P-term = Kc * (b*sp - pv)
 *   I-term = Kc/Ti * ∫(sp - pv)
 *   D-term = Kc*Td * (c*d(sp)/dt - d(pv)/dt)
 *
 * @param ctrl   Controller instance
 * @param sp     Setpoint
 * @param pv     Process variable
 * @param dt     Time step [s]
 * @return PID output
 */
double override_pid_update_2dof(override_controller_t *ctrl,
                                double sp, double pv, double dt);

/**
 * PID update with rate-limited setpoint change
 *
 * Applies a rate limit to setpoint changes to prevent derivative
 * kick during setpoint steps.
 *
 * @param ctrl        Controller instance
 * @param sp          Target setpoint (raw)
 * @param pv          Process variable
 * @param dt          Time step [s]
 * @param sp_rate_lim Setpoint rate limit [units/s]
 * @return PID output
 */
double override_pid_update_rate_limited_sp(override_controller_t *ctrl,
                                           double sp, double pv,
                                           double dt, double sp_rate_lim);

/* =========================================================================
 * L3 — Anti-Windup Methods
 * ========================================================================= */

/**
 * Back-calculation anti-windup
 *
 * When the output saturates (exceeds u_min or u_max), the integral
 * term is reduced using a tracking time constant Tt:
 *   I_new = I_old - (Ts/Tt) * (u - u_saturated)
 *
 * Default Tt = sqrt(Ti * Td) or Ti (whichever is defined).
 *
 * @param ctrl     Controller instance
 * @param u_raw    Unclamped PID output
 * @param u_clamped Clamped PID output
 * @param dt       Time step [s]
 */
void override_pid_back_calc(override_controller_t *ctrl,
                            double u_raw, double u_clamped, double dt);

/**
 * Conditional integration anti-windup
 *
 * Integral action is frozen when:
 * 1. Output is saturated AND
 * 2. Error and output have the same sign (i.e., integral would
 *    push output further into saturation)
 *
 * @param ctrl   Controller instance
 * @param error  Current control error (sp - pv)
 * @param u_sat  1 if saturated, 0 otherwise
 */
void override_pid_cond_integration(override_controller_t *ctrl,
                                   double error, int u_sat);

/**
 * Clamping anti-windup (simplest form)
 *
 * Simply clamp the integral term to prevent the output from
 * going beyond the limits. The integral is not updated when
 * the output is already at its limit.
 *
 * @param ctrl    Controller instance
 * @param di_raw  Raw integral increment
 * @param u_clamped Clamped output
 * @param u_limits Active limit (0=none, 1=upper, -1=lower)
 */
double override_pid_clamp_integral(override_controller_t *ctrl,
                                   double di_raw, double u_clamped,
                                   int u_limits);

/* =========================================================================
 * L3 — Tracking and Bumpless Transfer
 * ========================================================================= */

/**
 * Set tracking mode for inactive PID controller
 *
 * When a controller is inactive, its output is forced to track
 * a specified value (typically the active controller's output).
 * This enables smooth bumpless transfer when the controller
 * becomes active again.
 *
 * Implementation: External Reset per IEC 61131-3
 * The integral term is back-calculated so that:
 *   I = tracking_value - Kc*(b*sp - pv) - D
 *
 * @param ctrl            Controller instance
 * @param tracking_value  Value to track
 * @param dt              Time step [s]
 */
void override_pid_set_tracking(override_controller_t *ctrl,
                               double tracking_value, double dt);

/**
 * Initialize PID controller state for bumpless transfer
 *
 * Sets the integral term so that the initial PID output equals
 * a specified value (the "manual" or "tracking" initialization
 * point), preventing a bump when switching from manual to auto.
 *
 * @param ctrl         Controller instance
 * @param init_output  Desired initial output
 * @param sp           Current setpoint
 * @param pv           Current process variable
 */
void override_pid_bumpless_init(override_controller_t *ctrl,
                                double init_output,
                                double sp, double pv);

/**
 * Determine if controller is in saturation
 *
 * @param ctrl Controller instance
 * @return 1 if saturated, 0 otherwise
 */
int override_pid_is_saturated(const override_controller_t *ctrl);

/**
 * Compute the tracking error (difference between output and tracking)
 *
 * @param ctrl Controller instance
 * @return Tracking error (output - tracking_value)
 */
double override_pid_tracking_error(const override_controller_t *ctrl);

/* =========================================================================
 * L4 — PID Form Conversion (IEC 61131-3 / ISA)
 * ========================================================================= */

/**
 * Convert PID parameters between ISA Standard form and Parallel form
 *
 * ISA Standard:  u = Kc*(e + 1/Ti*∫e + Td*de/dt)
 * Parallel:      u = Kp*e + Ki*∫e + Kd*de/dt
 *
 * Conversion:
 *   Kp = Kc
 *   Ki = Kc / Ti
 *   Kd = Kc * Td
 *
 * @param src       Source parameters (ISA form)
 * @param Kp, Ki, Kd Output: Parallel form parameters
 */
void override_pid_isa_to_parallel(const override_pid_params_t *src,
                                  double *Kp, double *Ki, double *Kd);

/**
 * Convert PID parameters from Parallel form to ISA Standard form
 *
 * Conversion:
 *   Kc = Kp
 *   Ti = Kp / Ki  (Ki > 0)
 *   Td = Kd / Kp
 *
 * @param Kp, Ki, Kd Parallel form inputs
 * @param dst       Destination parameters (ISA form)
 * @return 0 on success, -1 if conversion invalid (Ki <= 0)
 */
int override_pid_parallel_to_isa(double Kp, double Ki, double Kd,
                                 override_pid_params_t *dst);

/**
 * Convert PID to Series (Interacting) form
 *
 * Series:  u = Kc' * (1 + 1/(Ti'*s)) * (1 + Td'*s) * e
 *
 * Conversion from ISA:
 *   Kc' = Kc * (1 + Td/Ti)
 *   Ti' = Ti + Td
 *   Td' = Ti*Td / (Ti + Td)
 *
 * @param src  ISA form parameters
 * @param Kc_s, Ti_s, Td_s Output: Series form parameters
 */
void override_pid_isa_to_series(const override_pid_params_t *src,
                                double *Kc_s, double *Ti_s, double *Td_s);

#ifdef __cplusplus
}
#endif

#endif /* OVERRIDE_PID_H */
