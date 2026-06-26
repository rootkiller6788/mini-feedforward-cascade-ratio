/**
 * @file override_valve_pos.h
 * @brief Override/Selector Control — Valve Position Control (VPC)
 *
 * Valve Position Control (VPC) is a specialized override strategy
 * where a VPC controller manipulates a secondary valve (bypass,
 * recycle, or vent) to maintain the main control valve within its
 * effective operating range (typically 10-90% open).
 *
 * This prevents the main valve from operating at extremes where
 * control authority is lost (fully open or fully closed), and
 * reduces wear on the main valve.
 *
 * Reference:
 *   Shinskey, F.G. (1996). Process Control Systems (4th ed.),
 *   McGraw-Hill. Chapter 9.4: Valve-Position Control.
 *
 *   Luyben, W.L. (2007). Chemical Reactor Design and Control.
 *   Wiley. Chapter 8: Reactor Temperature Control with VPC.
 *
 * Knowledge Coverage:
 *   L2 — Core Concepts: Valve position control, operating range
 *   L3 — Engineering Structures: VPC override topology
 *   L5 — Algorithms: VPC PID tuning, float control
 *   L6 — Canonical Problems: Reactor temperature with VPC
 *
 * Course Alignment:
 *   Purdue ME 575: Advanced process control strategies
 *   Tsinghua: 阀位控制
 *   RWTH Aachen: Ventilpositionsregelung
 */

#ifndef OVERRIDE_VALVE_POS_H
#define OVERRIDE_VALVE_POS_H

#include "override_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L2 — VPC Core Functions
 * ========================================================================= */

/**
 * Execute one cycle of VPC update
 *
 * The VPC controller works as follows:
 * 1. Read the current main valve position
 * 2. If main valve is outside [vpc_min, vpc_max]:
 *    - VPC output is updated by a PID controller tracking
 *      the deviation from vpc_setpoint
 * 3. If main valve is within range:
 *    - VPC output is held at last value (or slowly ramped to 0)
 *
 * The VPC output typically controls a bypass or recycle valve
 * that affects the process in the same direction as the main valve.
 *
 * @param vpc           VPC state structure
 * @param main_valve_pos Current main valve position [%]
 * @param dt            Time step [s]
 * @return VPC output value (typically 0-100%)
 */
double vpc_update(vpc_state_t *vpc, double main_valve_pos, double dt);

/**
 * Determine if VPC should be active
 *
 * VPC is active when the main valve position is outside the
 * desired range [vpc_min, vpc_max] plus a small deadband.
 *
 * @param vpc VPC state
 * @return 1 if VPC should be active, 0 otherwise
 */
int vpc_should_activate(const vpc_state_t *vpc);

/**
 * Compute the VPC error signal (deviation from setpoint)
 *
 * Error = 0 when main valve is within range.
 * Error > 0 when main valve > vpc_max (valve too open → reduce)
 * Error < 0 when main valve < vpc_min (valve too closed → increase)
 *
 * @param vpc VPC state
 * @return VPC error (main_valve_pos - vpc_setpoint when out of range)
 */
double vpc_error(const vpc_state_t *vpc);

/* =========================================================================
 * L3 — VPC Tuning Methods
 * ========================================================================= */

/**
 * Set VPC PID tuning parameters
 *
 * VPC tuning is typically gentle (slow) because the VPC loop
 * should not compete with the primary control loop. A common
 * rule of thumb is:
 *   VPC_Kc ≈ 0.25 to 0.5
 *   VPC_Ti ≈ 30 to 120 seconds (slow integral)
 *   VPC_Td ≈ 0 (no derivative, or very small)
 *
 * @param vpc VPC state
 * @param Kc  Proportional gain
 * @param Ti  Integral time [s]
 * @param Td  Derivative time [s]
 * @param N   Derivative filter coefficient
 */
void vpc_set_tuning(vpc_state_t *vpc,
                    double Kc, double Ti, double Td, double N);

/**
 * Auto-tune VPC based on process characteristics
 *
 * Uses a conservative tuning rule based on the main loop's
 * integral time and the valve dynamics.
 *
 * Rule: VPC_Ti ≈ 5 * Main_Ti (5x slower than main loop)
 *       VPC_Kc ≈ 0.2 * Main_Kc (5x less aggressive)
 *
 * This ensures the VPC does not interfere with the primary
 * control loop under normal conditions.
 *
 * @param vpc         VPC state
 * @param main_Kc     Main controller gain
 * @param main_Ti     Main controller integral time [s]
 */
void vpc_autotune_from_main(vpc_state_t *vpc,
                            double main_Kc, double main_Ti);

/* =========================================================================
 * L3 — VPC Anti-Windup and Saturation Handling
 * ========================================================================= */

/**
 * VPC integral anti-windup
 *
 * When the VPC output saturates (0% or 100%), the integral term
 * is frozen or reduced to prevent windup.
 *
 * @param vpc VPC state
 */
void vpc_antiwindup(vpc_state_t *vpc);

/**
 * Check if VPC output is saturated
 *
 * @param vpc VPC state
 * @return 1 if saturated at min (0%), -1 if saturated at max (100%), 0 if OK
 */
int vpc_is_saturated(const vpc_state_t *vpc);

/**
 * Set VPC output range (clamping limits)
 *
 * @param vpc    VPC state
 * @param vpc_out_min Minimum output [%]
 * @param vpc_out_max Maximum output [%]
 */
void vpc_set_output_limits(vpc_state_t *vpc,
                           double vpc_out_min, double vpc_out_max);

/* =========================================================================
 * L5 — VPC with Multiple Valves
 * ========================================================================= */

/**
 * Split-range VPC: distribute VPC output between two valves
 *
 * In a split-range VPC configuration, two valves share the VPC
 * output range. For example:
 *   Valve A: 0-50% VPC output → A: 0-100% open
 *   Valve B: 50-100% VPC output → B: 0-100% open
 *
 * @param vpc_output      VPC output [0-100%]
 * @param split_point     Split point [0-100%] (where valve A hands off to B)
 * @param valve_a_open    Output: Valve A opening [0-100%]
 * @param valve_b_open    Output: Valve B opening [0-100%]
 */
void vpc_split_range(double vpc_output, double split_point,
                     double *valve_a_open, double *valve_b_open);

/**
 * VPC with deadband
 *
 * Apply a deadband to the VPC error signal so that small valve
 * position deviations do not cause unnecessary VPC action.
 *
 * @param error    Raw VPC error
 * @param deadband Deadband width (>0)
 * @return Deadband-filtered error
 */
double vpc_deadband_filter(double error, double deadband);

/**
 * VPC output rate limiting
 *
 * Prevents abrupt changes in the VPC valve position to avoid
 * disturbing the process.
 *
 * @param vpc           VPC state
 * @param raw_output    Desired VPC output
 * @param rate_limit    Maximum rate of change [%/s]
 * @param dt            Time step [s]
 * @return Rate-limited VPC output
 */
double vpc_rate_limit(vpc_state_t *vpc, double raw_output,
                      double rate_limit, double dt);

/* =========================================================================
 * L6 — VPC Process Model Simulation
 * ========================================================================= */

/**
 * Simulate the effect of VPC action on the main valve position
 *
 * Simplified model: increasing VPC output (opening bypass) reduces
 * the load on the main valve, causing it to close toward its
 * desired range.
 *
 * @param vpc               VPC state
 * @param main_valve_pos    Current main valve position [%]
 * @param process_gain      Process gain (Δvalve_pos / ΔVPC_out)
 * @param dt                Time step [s]
 * @return New main valve position [%]
 */
double vpc_simulate_main_valve(const vpc_state_t *vpc,
                               double main_valve_pos,
                               double process_gain, double dt);

/**
 * Compute VPC performance metrics
 *
 * Evaluates how well the VPC is maintaining the main valve
 * within its desired range.
 *
 * @param vpc VPC state
 * @param time_in_range  Output: fraction of time valve is in range [0-1]
 * @param avg_deviation  Output: average deviation from range [%]
 * @param max_deviation  Output: maximum deviation from range [%]
 */
void vpc_performance(const vpc_state_t *vpc,
                     double *time_in_range,
                     double *avg_deviation,
                     double *max_deviation);

#ifdef __cplusplus
}
#endif

#endif /* OVERRIDE_VALVE_POS_H */
