#ifndef GAIN_SCHEDULE_PID_H
#define GAIN_SCHEDULE_PID_H

#include "gain_schedule_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file    gain_schedule_pid.h
 * @brief   Gain-Scheduled PID Controller L2/L3/L6
 *
 * Implements a full gain-scheduled PID controller with:
 *   - Online gain interpolation from the schedule table
 *   - Multiple PID forms (parallel, series, ISA, 2-DOF, incremental)
 *   - Anti-windup via back-calculation and clamping
 *   - Bumpless transfer during gain transitions
 *   - Derivative filtering (first-order low-pass)
 *   - Scheduling variable filtering
 *   - Gain change rate limiting
 */

void gs_pid_init(gs_pid_state_t *state,
                 const char *tag,
                 gs_pid_form_t form);

void gs_pid_set_saturation(gs_pid_state_t *state,
                            double high, double low);

void gs_pid_set_dt(gs_pid_state_t *state, double dt);

/**
 * Main PID update function with gain scheduling.
 * Called every sample period.
 *
 * @param state         PID controller state
 * @param table         Gain schedule table (interpolation source)
 * @param setpoint      Reference setpoint r(t)
 * @param pv            Process variable y(t)
 * @param sched_val     Current scheduling variable value
 * @param[out] output   Computed control output u(t)
 *
 * Steps:
 *   1. Filter and store scheduling variable
 *   2. Interpolate gains from schedule table
 *   3. Apply gain smoothing (rate limiting)
 *   4. Compute tracking error
 *   5. Calculate P, I, D terms per selected PID form
 *   6. Anti-windup (back-calculation + clamping)
 *   7. Output saturation
 */
void gs_pid_update(gs_pid_state_t *state,
                   const gain_schedule_table_t *table,
                   double setpoint,
                   double pv,
                   double sched_val,
                   double *output);

/**
 * Simplified update using explicit gains (no table lookup).
 * Useful for manual gain override or testing.
 */
void gs_pid_update_direct(gs_pid_state_t *state,
                           double Kp, double Ki, double Kd,
                           double setpoint, double pv,
                           double *output);

/**
 * Reset integrator to zero. Called during mode changes.
 */
void gs_pid_reset_integral(gs_pid_state_t *state);

/**
 * Set integrator to a specific value (bumpless initialization).
 */
void gs_pid_set_integral(gs_pid_state_t *state, double value);

/**
 * Enter tracking mode: output follows tracking_input.
 * Used for bumpless transfer from manual to auto.
 */
void gs_pid_tracking_mode(gs_pid_state_t *state,
                           double tracking_input);

/**
 * Compute the current PID gains by interpolating from the schedule table.
 * Does NOT update the controller state. Useful for diagnostics.
 */
pid_gain_set_t gs_pid_compute_gains(
    const gain_schedule_table_t *table,
    double sched_val);

/**
 * Get controller diagnostics as a formatted string.
 */
void gs_pid_diagnostics(const gs_pid_state_t *state,
                         char *buffer, size_t bufsize);

/**
 * Estimate closed-loop bandwidth at the current operating point
 * based on interpolated gains and a local first-order approximation.
 */
double gs_pid_estimate_bandwidth(const gs_pid_state_t *state,
                                  double process_gain,
                                  double process_tau);

/**
 * Get the number of schedule switches since initialization.
 */
uint64_t gs_pid_get_switch_count(const gs_pid_state_t *state);

#ifdef __cplusplus
}
#endif
#endif /* GAIN_SCHEDULE_PID_H */
