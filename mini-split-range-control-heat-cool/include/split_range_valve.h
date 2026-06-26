/**
 * @file split_range_valve.h
 * @brief Control valve modeling and characterization for split-range systems
 *
 * Module: mini-split-range-control-heat-cool
 * Knowledge Coverage: L3 Engineering Structures, L4 Engineering Laws
 *
 * Control valves are the most common final control element in process
 * industries.  Their installed flow characteristic deviates from the
 * inherent characteristic due to piping pressure drops.  This module
 * models both inherent and installed characteristics per ISA-75.01.
 *
 * Key standards:
 *   ISA-75.01.01 — Flow Equations for Sizing Control Valves
 *   ISA-75.11.01 — Inherent Flow Characteristic
 *   IEC 60534-2-1 — Industrial-process control valves, Flow capacity
 *   IEC 60534-7   — Control valve data sheet
 *
 * Valve types:
 *   - Globe (linear, equal-percentage)
 *   - Ball (modified equal-percentage)
 *   - Butterfly (approximately equal-percentage)
 *   - V-notch ball (linear to equal-percentage)
 *
 * Curriculum:
 *   MIT 6.302 — Actuator dynamics and saturation
 *   Stanford ENGR205 — Final control elements
 *   Purdue ME575 — Control valve engineering
 *   RWTH Aachen — Stellgerate (actuators)
 *   Tsinghua — 控制阀工程
 */

#ifndef SPLIT_RANGE_VALVE_H
#define SPLIT_RANGE_VALVE_H

#include "split_range_types.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Compute the installed flow characteristic of a control valve.
 *
 * The installed characteristic accounts for the pressure drop ratio:
 *   Pr = delta_P_valve_min / delta_P_system_total
 *
 * When Pr < 1, the valve characteristic "flattens" — the effective
 * rangeability decreases and the equal-percentage shape distorts.
 *
 * @param stem_pos    Stem position (0-1)
 * @param inherent    Inherent characteristic function
 * @param Pr          Pressure drop ratio (0-1, typically 0.3-0.7)
 * @param rangeability Valve rangeability
 * @return            Installed flow fraction (0-1)
 *
 * Complexity: O(1).
 *
 * Theorem (ISA-75.01): For an equal-percentage valve with Pr < 1,
 *   Q_installed = sqrt(Pr) * Q_inherent / sqrt(Pr + (1-Pr)*Q_inherent^2)
 *
 * Reference: ISA-75.01.01, Annex B — Installed Flow Characteristics
 */
double split_valve_installed_characteristic(double stem_pos,
                                              split_range_valve_char_t inherent,
                                              double Pr,
                                              double rangeability);

/**
 * Size a control valve per ISA-75.01.01 equations.
 *
 * For incompressible (liquid) flow:
 *   Cv = Q * sqrt(Gf / delta_P)
 *
 * For compressible (gas/vapor) flow:
 *   Cv = (Q / (N1*Fp*P1*Y)) * sqrt(T1*Gg*Z / (x))
 *
 * @param Q          Volumetric flow rate (gpm for liquid, scfh for gas)
 * @param delta_P    Pressure drop across valve (psi)
 * @param Gf         Specific gravity (liquid) or gas specific gravity (gas)
 * @param T1         Upstream temperature (degR, gas only, 0 for liquid)
 * @param P1         Upstream pressure (psia, gas only, 0 for liquid)
 * @param is_gas     0 = liquid, 1 = gas
 * @return           Required Cv (flow coefficient)
 *
 * Complexity: O(1). Implements ISA-75.01.01 eq. 1 (liquid) and eq. 3 (gas).
 * Edge cases: returns 0.0 for non-positive flow or delta_P.
 *
 * Reference: ISA-75.01.01-2012, Section 5 — Sizing Equations
 */
double split_valve_size_isa(double Q, double delta_P, double Gf,
                              double T1, double P1, int is_gas);

/**
 * Estimate the installed pressure drop ratio (Pr) for a control valve
 * in a piping system.
 *
 * Pr = (Cv_rated / (Cv_rated + Cv_piping))^2  (approximately)
 *
 * where Cv_piping is the equivalent Cv of the piping system.
 * Low Pr (< 0.3) indicates an "undersized" valve where most pressure
 * drop is across the piping, degrading controllability.
 *
 * @param Cv_valve    Rated Cv of the control valve
 * @param Cv_piping   Equivalent Cv of upstream + downstream piping
 * @return            Pressure drop ratio Pr (0-1)
 *
 * Complexity: O(1).
 * Rule of thumb: Pr should be >= 0.3 for good controllability (ISA).
 */
double split_valve_pressure_drop_ratio(double Cv_valve, double Cv_piping);

/**
 * Compute control valve rangeability: the ratio of maximum to minimum
 * controllable flow.
 *
 * R = Cv_max / Cv_min_controllable
 *
 * Typical values:
 *   Globe (equal-%): 25-50
 *   V-ball:          50-100
 *   Butterfly:       10-20
 *   Ball (segmented): 100-300
 *
 * @param Cv_max          Maximum flow coefficient
 * @param Cv_min_controllable Minimum controllable flow coefficient
 * @return                Rangeability (>= 1.0)
 *
 * Complexity: O(1). Returns 1.0 for invalid inputs.
 */
double split_valve_rangeability(double Cv_max, double Cv_min_controllable);

/**
 * Compute valve gain (dQ/dx): the derivative of flow with respect to
 * stem position.  This is the "local" gain used in linearity analysis.
 *
 * For linear valve:    dQ/dx = Qmax (constant)
 * For equal-% valve:   dQ/dx = Qmax * R^(x-1) * ln(R)
 * For quick-opening:   dQ/dx = Qmax / (2*sqrt(x))
 *
 * @param stem_pos    Stem position (0-1)
 * @param char_type   Valve characteristic type
 * @param rangeability Rangeability R
 * @return            Valve gain dQ/dx at the given stem position
 *
 * Complexity: O(1).
 *
 * Design insight: The valve gain is the "process gain" from the
 * controller's perspective. A constant valve gain (linear characteristic)
 * simplifies controller tuning; varying gain (equal-percentage) requires
 * gain scheduling or conservative tuning.
 */
double split_valve_gain(double stem_pos,
                          split_range_valve_char_t char_type,
                          double rangeability);

/**
 * Build a user-defined valve characteristic table from measured data
 * points. The table is interpolated linearly between points.
 *
 * @param table    Output: populated user table
 * @param co_data  Array of stem position percentages (0-100)
 * @param flow_data Array of flow percentages (0-100)
 * @param n        Number of data points (>= 2, <= 64)
 * @return         0 on success, -1 on error (too many points, unsorted, etc.)
 *
 * Complexity: O(n log n) for sorting validation; O(log n) for lookup.
 * Monotonicity is enforced: duplicate or non-monotonic flow values are
 * replaced with interpolated values to guarantee a valid function.
 */
int split_valve_build_user_table(split_range_user_table_t *table,
                                   const double *co_data,
                                   const double *flow_data,
                                   int n);

/**
 * Look up flow fraction from a user-defined valve table.
 * Performs linear interpolation between the two nearest points.
 *
 * @param table    User-defined valve table
 * @param stem_pos Stem position (0-1)
 * @return         Flow fraction (0-1), extrapolated at boundaries
 *
 * Complexity: O(log n) with binary search.
 * Edge cases: clamps stem_pos to [0, 1].
 */
double split_valve_user_table_lookup(const split_range_user_table_t *table,
                                       double stem_pos);

/**
 * Model control valve stiction (static friction) behavior.
 * Stiction causes the valve to "stick" until the actuator force exceeds
 * the static friction, then "slip" to a new position, causing limit
 * cycles in the control loop.
 *
 * This model is the Choudhury-Sirish-Shah (2005) stiction model:
 *   If |u - u_prev| > S (stickband) + J (slip jump): valve moves
 *   Otherwise: valve stays at last position
 *
 * @param channel       Valve channel
 * @param desired_pos   Controller-requested position (%)
 * @return              Actual position after stiction effect (%)
 *
 * Complexity: O(1).
 * Reference: Choudhury et al. (2005) "A simple method for detection of
 *   stiction in control valves", Control Engineering Practice.
 */
double split_valve_stiction_model(split_range_channel_t *channel,
                                    double desired_pos);

/**
 * Compute the energy consumption split between heating and cooling
 * for a given operating point. Useful for economic optimization of
 * split-point selection.
 *
 * @param co               Current controller output (%)
 * @param split_point      Split point (%)
 * @param valve_positions  Array of valve positions for each channel
 * @param n_channels       Number of channels
 * @param heating_power    Output: heating energy rate (kW or similar)
 * @param cooling_power    Output: cooling energy rate (kW or similar)
 *
 * Complexity: O(n).  Aggregates flow rates and converts to energy.
 */
void split_valve_energy_consumption(double co, double split_point,
                                      const double *valve_positions,
                                      int n_channels,
                                      double *heating_power,
                                      double *cooling_power);

/**
 * Perform a partial stroke test (PST) on a control valve to verify
 * operability without fully closing it (critical for safety valves
 * that must remain in process). Per IEC 61508 / ISA-84.00.01.
 *
 * @param channel           Valve to test
 * @param stroke_percentage How far to stroke (5-20% typical)
 * @param breakaway_time_ms Output: time to start moving (stiction indicator)
 * @param stroke_time_ms    Output: time to complete the stroke
 * @return                  0 if valve moves freely, 1 if stiction detected
 *
 * Complexity: O(1) — simplified model for control valves.
 * Reference: ISA-96.02.01 — Partial Stroke Testing of Automated Valves
 */
int split_valve_partial_stroke_test(split_range_channel_t *channel,
                                      double stroke_percentage,
                                      double *breakaway_time_ms,
                                      double *stroke_time_ms);

#ifdef __cplusplus
}
#endif

#endif /* SPLIT_RANGE_VALVE_H */
