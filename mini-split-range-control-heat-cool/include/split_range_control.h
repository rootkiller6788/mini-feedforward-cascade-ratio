/**
 * @file split_range_control.h
 * @brief Unified split-range control interface — high-level control functions
 *
 * Module: mini-split-range-control-heat-cool
 * Knowledge Coverage: L2 Core Concepts, L6 Canonical Problems
 *
 * This is the top-level interface for the split-range control module.
 * It provides factory functions, high-level initialization, and
 * integrated control loop execution intended for use in:
 *   - Reactor temperature control (heat/cool)
 *   - pH neutralization (acid/base)
 *   - Pressure control with vent/inert
 *
 * Reference:
 *   Myke King (2016) Process Control: A Practical Approach
 *   Seborg et al. (2016) Process Dynamics and Control
 *
 * Curriculum:
 *   MIT 6.302 — Control system integration
 *   Stanford ENGR205 — Industrial control architectures
 *   Purdue ME575 — Process unit operations control
 *   Tsinghua — 过程控制系统
 */

#ifndef SPLIT_RANGE_CONTROL_H
#define SPLIT_RANGE_CONTROL_H

#include "split_range_types.h"
#include "split_range_core.h"
#include "split_range_pid.h"
#include "split_range_valve.h"
#include "split_range_advanced.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Factory: Create a fully initialized split-range controller for
 * reactor temperature control (heat/cool).
 *
 * Sets up:
 *   - 2-channel split scheme (heating + cooling)
 *   - PID parameters: Kc=2.0, Ti=120s, Td=30s (conservative start)
 *   - Sample time: 1.0 second
 *   - Split point: 50%, deadband: 2%
 *   - Valve characteristics: equal-percentage for heating, linear for cooling
 *
 * @return Initialized controller struct
 *
 * Complexity: O(1).  This is the recommended starting point for
 * jacketed reactor temperature control.
 *
 * Reference: Myke King (2016) Ch. 9, "Reactor Temperature Control Design"
 */
split_range_controller_t split_control_create_reactor(void);

/**
 * Factory: Create a split-range controller for pH neutralization.
 *
 * Sets up:
 *   - 2-channel split scheme (acid + base)
 *   - PID with high gain (Kc=5.0) for the steep titration curve
 *   - Overlap of 5% for smoother transitions near neutral pH 7
 *   - Equal-percentage valves (better turndown for pH)
 *
 * @return Initialized controller struct
 *
 * Complexity: O(1).
 *
 * Note: pH control is challenging due to the highly nonlinear
 * titration curve.  Consider adding gain scheduling or nonlinear
 * compensation (see split_range_advanced.h).
 */
split_range_controller_t split_control_create_ph(void);

/**
 * Factory: Create a split-range controller for pressure control
 * with vent and inert gas makeup.
 *
 * Sets up:
 *   - 2-channel split scheme (vent + inert)
 *   - Vent valve (CO 0->50%, reverse acting: opens on high pressure)
 *   - Inert valve (CO 50->100%, direct acting: adds gas on low pressure)
 *
 * @return Initialized controller struct
 *
 * Complexity: O(1).
 */
split_range_controller_t split_control_create_pressure(void);

/**
 * Initialize the complete split-range controller with custom settings.
 *
 * @param ctrl       Controller to initialize (caller-allocated)
 * @param scheme     Pre-configured split-range scheme (may be NULL for default)
 * @param kc,ti,td,ts PID parameters
 * @param pv_min,pv_max PV scaling range
 *
 * Complexity: O(1).
 */
void split_control_init(split_range_controller_t *ctrl,
                          const split_range_scheme_t *scheme,
                          double kc, double ti, double td, double ts,
                          double pv_min, double pv_max);

/**
 * Set the process variable (PV) for the next control cycle.
 * Applies filtering and rate-of-change calculation.
 *
 * @param ctrl   Controller
 * @param pv     Raw process variable value
 * @param dt_sec Time since last update
 *
 * Complexity: O(1).
 */
void split_control_set_pv(split_range_controller_t *ctrl,
                            double pv, double dt_sec);

/**
 * Set the setpoint (SP) for the controller.
 *
 * @param ctrl Controller
 * @param sp   New setpoint value
 *
 * Complexity: O(1).
 */
void split_control_set_sp(split_range_controller_t *ctrl, double sp);

/**
 * Execute one complete control cycle: compute PID output, distribute
 * to split-range channels, apply valve characteristics and slew limits.
 *
 * This is the main real-time function to be called at each sample period.
 *
 * @param ctrl   Controller (all state updated)
 * @param dt_sec Time step (seconds)
 *
 * Complexity: O(n) where n = number of channels.
 *
 * Call sequence: split_control_set_pv() -> split_control_set_sp()
 *                -> split_control_execute()
 */
void split_control_execute(split_range_controller_t *ctrl, double dt_sec);

/**
 * Evaluate control performance metrics (IAE, ISE, ITAE, overshoot, etc.)
 * based on stored PV/SP history. The state tracks necessary data for
 * ongoing performance monitoring.
 *
 * @param ctrl   Controller with running state
 * @param perf   Output: computed performance metrics
 *
 * Complexity: O(1) per call (incremental update of integral metrics).
 *
 * IAE (Integral Absolute Error): sum |SP-PV|*dt — penalizes sustained error
 * ISE (Integral Squared Error):   sum (SP-PV)^2*dt — penalizes large errors
 * ITAE (Integral Time*Absolute):  sum t*|SP-PV|*dt — penalizes late errors
 * ITSE (Integral Time*Squared):   sum t*(SP-PV)^2*dt — penalizes late, large errors
 */
void split_control_evaluate(const split_range_controller_t *ctrl,
                              split_range_performance_t *perf);

/**
 * Set a feedforward signal to improve disturbance rejection.
 * The feedforward signal is added to the PID output before split-range
 * distribution.
 *
 * Feedforward for heat/cool: ambient temperature, feed temperature,
 * or other measurable disturbances.
 *
 * @param ctrl  Controller
 * @param ff    Feedforward value (scaled to controller output %)
 *
 * Complexity: O(1).
 */
void split_control_set_feedforward(split_range_controller_t *ctrl, double ff);

/**
 * Compute the overall health status of the split-range control loop.
 * Aggregates per-channel health and checks for anomalies.
 *
 * Checked conditions:
 *   - Any channel in FAILURE → overall FAILURE
 *   - Any channel OUT_OF_SPEC → overall OUT_OF_SPEC
 *   - All channels OK → overall OK
 *
 * @param ctrl Controller
 * @return     Aggregated health status per NAMUR NE107
 *
 * Complexity: O(n).
 */
split_range_health_t split_control_health_check(const split_range_controller_t *ctrl);

/**
 * Validate the split-range scheme for consistency.
 * Checks:
 *   1. Channel ranges cover [0, 100] without gaps
 *   2. No overlapping ranges (unless mode=OVERLAP)
 *   3. All channels have valid valve characteristics
 *   4. Split point is within valid range
 *
 * @param scheme  Scheme to validate
 * @return        0 if valid, negative error code otherwise
 *   -1: num_channels out of range
 *   -2: channel ranges have gaps
 *   -3: invalid split point
 *   -4: overlapping ranges in non-overlap mode
 *   -5: invalid valve characteristic
 *
 * Complexity: O(n log n) for range sorting.
 */
int split_control_validate_scheme(const split_range_scheme_t *scheme);

#ifdef __cplusplus
}
#endif

#endif /* SPLIT_RANGE_CONTROL_H */
