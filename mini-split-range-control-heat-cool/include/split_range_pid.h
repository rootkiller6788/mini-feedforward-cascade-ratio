/**
 * @file split_range_pid.h
 * @brief PID controller with split-range output mapping
 *
 * Module: mini-split-range-control-heat-cool
 * Knowledge Coverage: L3 Engineering Structures, L5 Algorithms
 *
 * Implements the incremental (velocity-form) PID algorithm optimized for
 * split-range control applications.  Key features:
 *   - Velocity-form PID for inherent bumpless transfer
 *   - Anti-windup: conditional integration + external reset feedback
 *   - Derivative on PV (not error) to avoid derivative kick
 *   - Setpoint weighting (2-DOF PID) via beta/gamma parameters
 *   - Asymmetric output limits (different for heating vs cooling)
 *   - Back-calculation anti-windup for the split-range context
 *
 * The velocity form is:
 *   du(k) = Kc * [ (beta*SP - PV)(k) - (beta*SP - PV)(k-1)
 *                + (Ts/Ti)*(SP - PV)(k)
 *                + (Td/(Ts))*( (gamma*SP-PV)(k) - 2*(gamma*SP-PV)(k-1)
 *                             + (gamma*SP-PV)(k-2) ) ]
 *   u(k) = u(k-1) + du(k)
 *
 * Reference:
 *   Astrom & Hagglund (1995) PID Controllers: Theory, Design, and Tuning
 *   Seborg, Edgar, Mellichamp (2016) Process Dynamics and Control
 *
 * Curriculum:
 *   MIT 6.302 — PID implementation, anti-windup
 *   Stanford ENGR205 — Velocity-form PID
 *   Purdue ME575 — Industrial PID with split-range
 *   RWTH Aachen — SPS Programmierung (PLC PID blocks)
 */

#ifndef SPLIT_RANGE_PID_H
#define SPLIT_RANGE_PID_H

#include "split_range_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize PID parameters with defaults suitable for split-range
 * temperature control.  Uses conservative gains (Kc = 1.0, Ti = 60s,
 * Td = 15s) as a starting point per Ziegler-Nichols temperature loop
 * recommendations.
 *
 * @param params  Pointer to uninitialized PID params struct
 * @param kc      Proportional gain
 * @param ti      Integral time (seconds, 0 = disable integral)
 * @param td      Derivative time (seconds, 0 = disable derivative)
 * @param ts      Sample time (seconds)
 *
 * Complexity: O(1).
 */
void split_pid_init_params(split_range_pid_params_t *params,
                            double kc, double ti, double td, double ts);

/**
 * Reset all PID state variables to zero/false.  Call this when
 * transitioning from manual to auto mode to ensure a bumpless start.
 *
 * @param state Pointer to PID state struct
 *
 * Complexity: O(1).
 */
void split_pid_reset_state(split_range_pid_state_t *state);

/**
 * Execute one iteration of the incremental (velocity-form) PID algorithm.
 *
 * The velocity form is inherently bumpless because mode changes only
 * affect the INCREMENT, not the absolute output.  The integral term
 * accumulates du without requiring explicit integral reset.
 *
 * @param params  PID tuning parameters
 * @param state   PID state (updated in-place)
 * @param sp      Setpoint
 * @param pv      Process variable
 * @return        New controller output (0-100%), clamped to [0, 100]
 *
 * Complexity: O(1).  All operations are scalar arithmetic.
 *
 * Safety features:
 *   - Division by zero: Ti <= 0 disables integral action; Td <= 0 disables derivative
 *   - Output clamping: result always in [0, 100]
 *   - NaN/Inf: checked and replaced with last valid output
 *   - Integrator anti-windup: conditional integration when output saturated
 */
double split_pid_incremental(split_range_pid_params_t *params,
                              split_range_pid_state_t *state,
                              double sp, double pv);

/**
 * Execute one iteration of the positional (absolute) PID algorithm.
 * Less preferred for split-range due to bumpless transfer challenges,
 * but included for comparison and legacy DCS compatibility.
 *
 * @param params  PID tuning parameters
 * @param state   PID state (updated in-place)
 * @param sp      Setpoint
 * @param pv      Process variable
 * @return        New controller output (0-100%)
 *
 * Complexity: O(1).
 *
 * Anti-windup: Uses back-calculation with tracking time constant Tt = sqrt(Ti*Td)
 * per Astrom & Hagglund (1995), Section 3.5.
 *
 * Theorem: For a first-order process, the tracking time Tt = sqrt(Ti*Td)
 *          guarantees the anti-windup loop is faster than the closed-loop
 *          dynamics, preventing integrator windup.
 */
double split_pid_positional(split_range_pid_params_t *params,
                             split_range_pid_state_t *state,
                             double sp, double pv);

/**
 * Apply external reset feedback for anti-windup in cascade configurations.
 * When the split-range controller is the secondary loop in a cascade,
 * the primary controller output provides an external reset signal that
 * prevents the secondary integral from winding up.
 *
 * @param state         PID state
 * @param external_reset External tracking signal (0-100%)
 * @param tracking_gain Tracking gain (typically ~1.0)
 *
 * Complexity: O(1).
 * Reference: ISA-77.44.01 — Fossil Fuel Plant: Steam Temperature Controls
 */
void split_pid_external_reset(split_range_pid_state_t *state,
                               double external_reset, double tracking_gain);

/**
 * Setpoint filter: first-order low-pass filter on the setpoint to
 * prevent abrupt setpoint changes from causing large derivative kicks.
 *
 * @param state          PID state (filtered_setpoint updated)
 * @param raw_setpoint   Unfiltered setpoint
 * @param filter_tau_sec Filter time constant (seconds)
 * @param dt_sec         Sample time (seconds)
 * @return               Filtered setpoint
 *
 * Complexity: O(1).  Implements: Y(k) = Y(k-1) + (dt/tau)*(X(k) - Y(k-1))
 */
double split_pid_setpoint_filter(split_range_pid_state_t *state,
                                  double raw_setpoint,
                                  double filter_tau_sec, double dt_sec);

/**
 * Process variable filter: first-order low-pass or moving average filter
 * to reduce measurement noise before feeding to PID.
 *
 * @param state     PID state (filtered_pv updated)
 * @param raw_pv    Raw process variable measurement
 * @param alpha     Filter constant (0 = no filtering, 1 = infinite memory)
 * @return          Filtered PV
 *
 * Complexity: O(1).  Exponential moving average: Y(k) = alpha*Y(k-1) + (1-alpha)*X(k)
 */
double split_pid_pv_filter(split_range_pid_state_t *state,
                            double raw_pv, double alpha);

/**
 * Apply the full split-range control cycle: read PV, compute PID output,
 * distribute to valve positions via the split scheme, apply valve
 * characteristics and slew limits.
 *
 * This is the main "execute" function for the split-range controller.
 *
 * @param ctrl       Split-range controller (state updated)
 * @param dt_sec     Time step (seconds)
 * @return           Overall health status after this cycle
 *
 * Complexity: O(n) where n = number of channels.  Calls PID, then split
 * distribution, then per-channel slew limiting and characteristic mapping.
 */
split_range_health_t split_pid_control_cycle(split_range_controller_t *ctrl,
                                              double dt_sec);

/**
 * Transition the controller between modes (MANUAL/AUTO/CASCADE) with
 * bumpless transfer.  In velocity-form PID this is natural; in positional
 * form, the integrator must be tracked to match the current output.
 *
 * @param ctrl     Controller
 * @param new_mode New operating mode (enabled/disabled, cascade on/off)
 *
 * Complexity: O(1). Updates tracking signals for bumpless transfer.
 */
void split_pid_mode_transition(split_range_controller_t *ctrl,
                                bool enable, bool cascade);

/**
 * Ziegler-Nichols tuning for split-range temperature control.
 * Uses the open-loop step response method (process reaction curve).
 * The ultimate gain Ku and ultimate period Pu are identified from the
 * process step test, and PID parameters are computed per the ZN rules.
 *
 * ZN-PID: Kc = 0.6*Ku, Ti = 0.5*Pu, Td = 0.125*Pu
 * Tyreus-Luyben (more conservative): Kc = 0.45*Ku, Ti = 2.2*Pu, Td = Pu/6.3
 *
 * @param process_gain    K = steady-state PV change / CO step change
 * @param process_tau     tau = time constant (seconds)
 * @param process_theta   theta = apparent dead time (seconds)
 * @param result          Output: computed PID parameters
 * @param method          0=ZN, 1=Tyreus-Luyben, 2=Cohen-Coon
 *
 * Complexity: O(1).  Direct formula application.
 *
 * Theorem (Ziegler-Nichols, 1942): The PID settings Kc=0.6*Ku, Ti=0.5*Pu,
 * Td=0.125*Pu give a quarter-amplitude damping ratio for a wide class
 * of industrial processes.
 */
void split_pid_zn_tuning(double process_gain, double process_tau,
                          double process_theta,
                          split_range_tuning_result_t *result, int method);

/**
 * Compute the closed-loop poles of the split-range control system
 * assuming a FOPDT process model and the given PID parameters.
 * Uses Pade approximation for the dead time term.
 *
 * @param K       Process gain
 * @param tau     Process time constant
 * @param theta   Process dead time
 * @param params  PID parameters
 * @param poles   Output: array of 3 complex poles (real, imag) pairs
 * @return        Number of poles (always 3 for PID with Pade approx)
 *
 * Complexity: O(1).  Solves a cubic characteristic equation.
 * Theorem: With Pade(1,1) approximation, the closed-loop characteristic
 *          polynomial is cubic; stability requires all poles in LHP.
 */
int split_pid_closed_loop_poles(double K, double tau, double theta,
                                 const split_range_pid_params_t *params,
                                 double poles[3][2]);

#ifdef __cplusplus
}
#endif

#endif /* SPLIT_RANGE_PID_H */
