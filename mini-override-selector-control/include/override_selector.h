/**
 * @file override_selector.h
 * @brief Override/Selector Control — Selector Algorithm Implementations
 *
 * Reference:
 *   Shinskey, F.G. (1996). Process Control Systems (4th ed.), McGraw-Hill.
 *   Chapter 9.2: Auctioneering and Median Selectors.
 *
 *   Blevins, T.L. & Nixon, M. (2010). "Control Loop Foundation."
 *   ISA. Chapter 8: Override Control.
 *
 * Knowledge Coverage:
 *   L2 — Core Concepts: high/low/median select, auctioneering, hysteresis
 *   L3 — Engineering Structures: Selector function block (IEC 61131-3 SEL)
 *   L5 — Algorithms: Selection with hysteresis, rate limiting, validity
 *
 * Course Alignment:
 *   Purdue ME 575: Auctioneering and selector logic
 *   RWTH Aachen: Industrial selector function blocks
 *   Tsinghua: 选择控制算法
 */

#ifndef OVERRIDE_SELECTOR_H
#define OVERRIDE_SELECTOR_H

#include "override_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L2 — Selector Core Functions
 * ========================================================================= */

/**
 * High-select: choose the maximum of N values
 *
 * @param values      Array of N values
 * @param valid       Array of N validity flags (1=valid)
 * @param n           Number of values
 * @param selected_idx Output: index of selected value (-1 if none valid)
 * @return Maximum valid value (0.0 if none valid)
 * @note O(n) time complexity
 */
double selector_high(const double *values, const int *valid, int n,
                     int *selected_idx);

/**
 * Low-select: choose the minimum of N values
 *
 * @param values      Array of N values
 * @param valid       Array of N validity flags (1=valid)
 * @param n           Number of values
 * @param selected_idx Output: index of selected value (-1 if none valid)
 * @return Minimum valid value (0.0 if none valid)
 * @note O(n) time complexity
 */
double selector_low(const double *values, const int *valid, int n,
                    int *selected_idx);

/**
 * Median-select: choose the median of N values
 *
 * For N odd: returns the exact median.
 * For N even: returns the lower median index.
 *
 * @param values      Array of N values
 * @param valid       Array of N validity flags (1=valid)
 * @param n           Number of values
 * @param selected_idx Output: index of selected value (-1 if none valid)
 * @return Median value (0.0 if none valid)
 * @note O(n²) worst-case (partial sort), O(n) expected
 */
double selector_median(const double *values, const int *valid, int n,
                       int *selected_idx);

/**
 * Auctioneering selector: discard hi/lo, choose middle of remaining
 *
 * Standard 3-channel auctioneering: three redundant sensors,
 * select the middle value (median of 3), rejecting the outliers.
 * This is the most common industrial redundancy pattern.
 *
 * @param v1, v2, v3  Three sensor values
 * @param valid1,2,3  Validity flags
 * @param selected_idx Output: 0, 1, or 2 (index of selected sensor)
 * @return Selected value
 * @note Used in safety-critical triple-redundant voting (2oo3)
 */
double selector_auctioneer_3(double v1, int valid1,
                             double v2, int valid2,
                             double v3, int valid3,
                             int *selected_idx);

/**
 * General auctioneering: select from N values by trimming k extremes
 *
 * @param values      Array of N values
 * @param valid       Array of N validity flags
 * @param n           Number of values
 * @param k_trim      Number of extremes to trim from each end
 * @param selected_idx Output: selected index
 * @return Average of remaining values after trimming
 */
double selector_auctioneer_n(const double *values, const int *valid,
                             int n, int k_trim, int *selected_idx);

/**
 * Weighted average selector
 *
 * Each value contributes proportionally to its weight.
 * Invalid values get zero weight.
 *
 * @param values      Array of N values
 * @param weights     Array of N weights (sum should be 1.0)
 * @param valid       Array of N validity flags
 * @param n           Number of values
 * @return Weighted average of valid values
 */
double selector_weighted(const double *values, const double *weights,
                         const int *valid, int n);

/**
 * Average-select: compute mean of all valid values
 *
 * @param values      Array of N values
 * @param valid       Array of N validity flags
 * @param n           Number of values
 * @return Mean of valid values (0.0 if none valid)
 */
double selector_average(const double *values, const int *valid, int n);

/* =========================================================================
 * L2 — Hysteresis-Enabled Selector
 * ========================================================================= */

/**
 * High-select with hysteresis band
 *
 * Once a value has been selected, the selector will not switch to
 * another value unless the new value exceeds (selected - hysteresis).
 * This prevents rapid switching (chatter) when values are similar.
 *
 * @param values      Array of N values
 * @param valid       Array of N validity flags
 * @param n           Number of values
 * @param prev_idx    Previously selected index
 * @param hysteresis  Hysteresis band (>0)
 * @param selected_idx Output: new selected index
 * @return Selected value
 */
double selector_high_hysteresis(const double *values, const int *valid,
                                int n, int prev_idx, double hysteresis,
                                int *selected_idx);

/**
 * Low-select with hysteresis band
 *
 * Analogous to high-select hysteresis but for minimum selection.
 *
 * @param values      Array of N values
 * @param valid       Array of N validity flags
 * @param n           Number of values
 * @param prev_idx    Previously selected index
 * @param hysteresis  Hysteresis band (>0)
 * @param selected_idx Output: new selected index
 * @return Selected value
 */
double selector_low_hysteresis(const double *values, const int *valid,
                               int n, int prev_idx, double hysteresis,
                               int *selected_idx);

/**
 * Median-select with hysteresis
 *
 * Prevents switching from the previously selected value unless
 * the candidate median differs by more than hysteresis.
 *
 * @param values      Array of N values
 * @param valid       Array of N validity flags
 * @param n           Number of values
 * @param prev_idx    Previously selected index
 * @param hysteresis  Hysteresis band (>0)
 * @param selected_idx Output: new selected index
 * @return Selected value
 */
double selector_median_hysteresis(const double *values, const int *valid,
                                  int n, int prev_idx, double hysteresis,
                                  int *selected_idx);

/* =========================================================================
 * L3 — Rate-Limited Selector
 * ========================================================================= */

/**
 * Rate-limited selector: limit rate of change of selected output
 *
 * Prevents abrupt changes in the selector output by applying
 * a maximum rate-of-change constraint. This is essential for
 * smooth transitions when switching between controllers.
 *
 * @param raw_value        Raw selected value
 * @param prev_output      Previous selector output
 * @param rate_limit       Maximum rate of change [units/s]
 * @param dt               Time step [s]
 * @return Rate-limited output
 */
double selector_rate_limit(double raw_value, double prev_output,
                           double rate_limit, double dt);

/* =========================================================================
 * L3 — Signal Validation Functions
 * ========================================================================= */

/**
 * Check which signals deviate significantly from the median
 *
 * Used for sensor fault detection in redundant measurement systems.
 *
 * @param values      Array of N values
 * @param valid       Input/output: validity flags
 * @param n           Number of values
 * @param threshold   Deviation threshold (absolute)
 * @return Number of newly invalidated signals
 */
int selector_deviation_check(double *values, int *valid, int n,
                             double threshold);

/**
 * Compute the standard deviation of valid signals
 *
 * @param values      Array of N values
 * @param valid       Array of N validity flags
 * @param n           Number of values
 * @return Standard deviation (0.0 if <2 valid)
 */
double selector_std_dev(const double *values, const int *valid, int n);

/**
 * Compute the range (max - min) of valid signals
 *
 * @param values      Array of N values
 * @param valid       Array of N validity flags
 * @param n           Number of values
 * @return Range (0.0 if <2 valid)
 */
double selector_range(const double *values, const int *valid, int n);

/**
 * Sort values array in-place (ascending) — used by median selector
 *
 * @param values Array of N values to sort
 * @param n     Number of values
 */
void selector_sort_values(double *values, int n);

/**
 * Sort values array with index tracking — needed for median selector
 *
 * @param values  Array of N values to sort (modified)
 * @param indices Array of N indices (modified to track sorting)
 * @param n       Number of values
 */
void selector_sort_with_indices(double *values, int *indices, int n);

#ifdef __cplusplus
}
#endif

#endif /* OVERRIDE_SELECTOR_H */
