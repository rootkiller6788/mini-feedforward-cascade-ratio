/**
 * @file split_range_core.h
 * @brief Core split-range mapping algorithms — controller output to valve positions
 *
 * Module: mini-split-range-control-heat-cool
 * Knowledge Coverage: L2 Core Concepts, L3 Engineering Structures
 *
 * This file declares the core functions that implement the split-range
 * mapping: converting a single PID controller output (0-100%) into multiple
 * valve position commands according to the configured split scheme.
 *
 * Key algorithms:
 *   - Linear split mapping with deadband/overlap
 *   - Valve characteristic inversion (linear, equal-percentage, quick-opening)
 *   - Slew-rate-limited position tracking
 *   - Bumpless transfer between split modes
 *   - Hysteresis-aware valve positioning
 *
 * Reference:
 *   Myke King (2016) Process Control: A Practical Approach, Ch. 9
 *   ISA-75.01.01 — Industrial-Process Control Valves: Sizing Equations
 *   IEC 60534 — Industrial-process control valves
 *
 * Curriculum:
 *   MIT 6.302 — Actuator models and saturation
 *   Stanford ENGR205 — Split-range implementation
 *   Purdue ME575 — Valve sequencing logic
 *   Tsinghua — 分程控制系统
 */

#ifndef SPLIT_RANGE_CORE_H
#define SPLIT_RANGE_CORE_H

#include "split_range_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Compute the normalized position [0, 1] for a single channel given
 * the controller output [0, 100].
 *
 * @param co          Controller output (0-100%)
 * @param range_start CO% where this channel begins to open
 * @param range_end   CO% where this channel is fully open
 * @param action      Direction (increasing = valve opens with CO)
 * @param valve_start Valve position at range_start (% open)
 * @param valve_end   Valve position at range_end (% open)
 * @return            Valve position (0-100%)
 *
 * Complexity: O(1). Handles edge cases: co out of bounds, zero-width range.
 * Theorem: This is an affine mapping; composition with valve characteristic
 *          preserves monotonicity and thus loop stability (BIBO sense).
 */
double split_compute_channel_position(double co,
                                       double range_start, double range_end,
                                       split_range_action_t action,
                                       double valve_start, double valve_end);

/**
 * Apply deadband and overlap logic to the controller output before
 * distributing to channels. Deadband creates a region around the
 * split point where all valves are closed. Overlap (negative deadband)
 * creates a region where both heating and cooling valves are partially open.
 *
 * @param co             Raw controller output (0-100%)
 * @param split_point    Transition point (default 50%)
 * @param deadband_width Width of dead zone (> 0) or overlap zone (< 0)
 * @param transition_type Transition smoothing method
 * @return               Modified controller output for channel mapping
 *
 * Complexity: O(1). The cubic spline transition requires solving a small
 * linear system at initialization (O(1) precomputation) and O(1) evaluation.
 */
double split_apply_deadband_overlap(double co,
                                     double split_point,
                                     double deadband_width,
                                     split_range_transition_t transition_type);

/**
 * Compute all channel positions for a complete split-range scheme.
 * This is the main entry point for converting a PID output into a set
 * of valve position commands.
 *
 * @param scheme   The split-range scheme configuration (read-only)
 * @param co       Controller output (0-100%)
 * @param positions Output array of length scheme.num_channels (0-100% each)
 * @return         Number of channels processed (scheme.num_channels)
 *
 * Complexity: O(n) where n = num_channels (max 6).
 * Safety: positions array must be pre-allocated with at least n elements.
 */
int split_distribute_output(const split_range_scheme_t *scheme,
                             double co,
                             double *positions);

/**
 * Apply valve characteristic to convert a desired valve position (0-100%
 * stem travel) into an equivalent position that accounts for the
 * installed flow characteristic of the control valve.
 *
 * The valve characteristic relates stem travel to flow:
 *   Linear:           Q/Qmax = x
 *   Equal-percentage: Q/Qmax = R^(x-1) where R = rangeability
 *   Quick-opening:    Q/Qmax = sqrt(x)
 *   Modified parabolic: Q/Qmax = a*x^2 + (1-a)*x (a in [0,1])
 *
 * This function computes the INVERSE: given desired flow, what stem
 * position is needed. Called "characteristic linearization" in industry.
 *
 * @param desired_flow   Desired flow as fraction of max (0-1)
 * @param char_type      Valve characteristic type
 * @param rangeability   Valve rangeability R (typically 20-50)
 * @return               Required stem position (0-1)
 *
 * Complexity: O(1) for analytic types, O(log n) for user table.
 * Theorem: The inverse characteristic linearizes the valve from the
 *          controller's perspective, making loop gain constant.
 */
double split_valve_characteristic_inverse(double desired_flow,
                                           split_range_valve_char_t char_type,
                                           double rangeability);

/**
 * Forward valve characteristic: given stem position, compute flow fraction.
 * Used for simulation and diagnostic purposes.
 *
 * @param stem_position Stem position fraction (0-1)
 * @param char_type     Valve characteristic type
 * @param rangeability  Valve rangeability R
 * @return              Flow as fraction of max (0-1)
 */
double split_valve_characteristic_forward(double stem_position,
                                           split_range_valve_char_t char_type,
                                           double rangeability);

/**
 * Apply slew-rate limiting to a valve position command. Ensures the
 * valve does not move faster than its physical limit, preventing
 * actuator damage and process upsets.
 *
 * @param current_pos  Current valve position (%)
 * @param target_pos   Desired valve position (%)
 * @param slew_limit   Maximum rate of change (%/sec)
 * @param dt_sec       Time step (seconds)
 * @return             Rate-limited valve position command (%)
 *
 * Complexity: O(1). Standard clamp: |target - current| <= limit*dt.
 */
double split_slew_rate_limit(double current_pos, double target_pos,
                              double slew_limit, double dt_sec);

/**
 * Apply hysteresis compensation to valve positioning. When the valve
 * exhibits stick-slip (stiction), a small hysteresis band prevents
 * the controller from oscillating by ignoring small position changes.
 *
 * @param channel        Valve channel (with hysteresis_memory updated)
 * @param desired_pos    Requested valve position (%)
 * @return               Hysteresis-compensated position (%)
 *
 * Complexity: O(1). Schmitt-trigger-like deadband.
 */
double split_hysteresis_compensate(split_range_channel_t *channel,
                                    double desired_pos);

/**
 * Initialize a default heat/cool split-range scheme.
 * Channel 0: Heating (CO 0->50%, Valve 100->0%, reverse acting)
 * Channel 1: Cooling (CO 50->100%, Valve 0->100%, direct acting)
 * Split point at 50%, deadband of 2%.
 *
 * @param scheme  Pointer to uninitialized scheme struct
 *
 * Complexity: O(1). Sets all fields to sensible defaults.
 * Reference: Myke King (2016) Ch. 9, "Split-Range Control of Reactors"
 */
void split_init_heat_cool_scheme(split_range_scheme_t *scheme);

/**
 * Initialize an acid/base (pH) split-range scheme.
 * Channel 0: Acid valve (CO 0->50%, Valve 100->0%, reverse)
 * Channel 1: Base valve (CO 50->100%, Valve 0->100%, direct)
 * Split point at 50%, overlap of 5% for smoother pH transitions.
 *
 * @param scheme  Pointer to uninitialized scheme struct
 */
void split_init_ph_scheme(split_range_scheme_t *scheme);

/**
 * Initialize a three-way split-range scheme (heat / bypass / cool).
 * Channel 0: Heating  (CO 0->33%, Valve 0->100%)
 * Channel 1: Bypass   (CO 33->66%, Valve 100->0%)
 * Channel 2: Cooling  (CO 66->100%, Valve 0->100%)
 *
 * @param scheme  Pointer to uninitialized scheme struct
 */
void split_init_three_way_scheme(split_range_scheme_t *scheme);

/**
 * Add a channel to a split-range scheme with custom range and direction.
 *
 * @param scheme       Scheme to add channel to
 * @param co_start     CO% where channel starts opening
 * @param co_end       CO% where channel is fully open
 * @param valve_start  Valve position at co_start (%)
 * @param valve_end    Valve position at co_end (%)
 * @param action       Direction of action
 * @param characteristic Valve characteristic
 * @return             Channel index on success, -1 on failure (scheme full)
 *
 * Complexity: O(1). Validates range and action consistency.
 */
int split_add_channel(split_range_scheme_t *scheme,
                       double co_start, double co_end,
                       double valve_start, double valve_end,
                       split_range_action_t action,
                       split_range_valve_char_t characteristic);

/**
 * Compute the effective steady-state gain for each channel in the
 * split-range scheme. Needed for loop analysis and tuning,
 * since the overall loop gain is piecewise-constant in each region.
 *
 * @param scheme  The split-range scheme
 * @param gains   Output array of gains (length num_channels)
 * @return        Number of gains computed
 *
 * Complexity: O(n). The gain for channel i is (valve_end - valve_start) /
 *   (co_end - co_start) for the channel's active region.
 */
int split_compute_channel_gains(const split_range_scheme_t *scheme,
                                 double *gains);

#ifdef __cplusplus
}
#endif

#endif /* SPLIT_RANGE_CORE_H */
