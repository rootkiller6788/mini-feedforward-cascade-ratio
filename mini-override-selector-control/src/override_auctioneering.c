/**
 * @file override_auctioneering.c
 * @brief Override/Selector Control — Signal Auctioneering Implementation
 *
 * Implements signal auctioneering, a reliability technique where
 * multiple redundant measurements (typically 3 sensors) are
 * combined. The median value is selected, rejecting outliers.
 * This is used extensively in:
 *   - Turbine/compressor control (2oo3 voting)
 *   - Boiler drum level (triple-redundant transmitters)
 *   - Nuclear reactor instrumentation
 *   - Safety instrumented systems (SIS)
 *
 * Knowledge Coverage:
 *   L2: Auctioneering concept, 2oo3 voting, redundant transmitters
 *   L3: Multi-channel signal validation, deviation alarms
 *   L5: Statistical signal selection, fault detection
 *   L7: Industrial redundancy (ISA-84/IEC 61511, NASA, nuclear)
 *
 * Reference:
 *   Liptak, B.G. (2006). Instrument Engineers' Handbook, Vol. 2.
 *   IEC 61508/61511: Functional Safety.
 *   ISA-84: Safety Instrumented Systems.
 */

#include "override_selector.h"
#include "override_core.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifndef INFINITY
#define INFINITY (1.0/0.0)
#endif

/* =========================================================================
 * L2 — Redundant Signal Voting (2oo3, 3oo3, 1oo3)
 * ========================================================================= */

/**
 * 2-out-of-3 (2oo3) voting: the standard triple-redundant voting
 * pattern used in safety systems.
 *
 * Logic:
 * - If 3 valid: select median (2oo3)
 * - If 2 valid: select average (2oo2)
 * - If 1 valid: select it (1oo1) + alarm
 * - If 0 valid: fail-safe value
 *
 * @param v1, v2, v4 Three sensor values
 * @param valid1,2,3 Validity flags
 * @param fail_safe   Output value when all sensors fail
 * @param voted_value Output: voted value
 * @param alarm       Output: 0=OK, 1=degraded(2oo2), 2=critical(1oo1), 3=failed
 * @return 0 on success, -1 on total failure
 */
int auctioneer_2oo3(double v1, int valid1,
                    double v2, int valid2,
                    double v3, int valid3,
                    double fail_safe,
                    double *voted_value, int *alarm) {
    int nv = (valid1 ? 1 : 0) + (valid2 ? 1 : 0) + (valid3 ? 1 : 0);

    if (voted_value == NULL) return -1;

    if (nv == 3) {
        /* Full triple redundancy — use median (auctioneering) */
        int sel_idx = -1;
        *voted_value = selector_auctioneer_3(v1, valid1, v2, valid2,
                                             v3, valid3, &sel_idx);
        if (alarm) *alarm = 0;
        return 0;
    } else if (nv == 2) {
        /* Degraded — average the two valid sensors */
        double sum = 0.0;
        sum += valid1 ? v1 : 0.0;
        sum += valid2 ? v2 : 0.0;
        sum += valid3 ? v3 : 0.0;
        *voted_value = sum / 2.0;
        if (alarm) *alarm = 1;
        return 0;
    } else if (nv == 1) {
        /* Critical — only one sensor working */
        *voted_value = valid1 ? v1 : (valid2 ? v2 : v3);
        if (alarm) *alarm = 2;
        return 0;
    } else {
        /* All failed — use fail-safe value */
        *voted_value = fail_safe;
        if (alarm) *alarm = 3;
        return -1;
    }
}

/**
 * 3-out-of-3 (3oo3) voting: requires all three sensors to agree
 * within a tolerance band. Most conservative — high safety integrity,
 * low availability.
 *
 * @param v1, v2, v3 Three sensor values
 * @param valid1,2,3 Validity flags
 * @param tolerance   Maximum allowed deviation between sensors
 * @param voted_value Output: voted value (average of 3 if all agree)
 * @param trip        Output: 1 if voting causes a trip (disagreement)
 * @return 0 on success, -1 on disagreement
 */
int auctioneer_3oo3(double v1, int valid1,
                    double v2, int valid2,
                    double v3, int valid3,
                    double tolerance,
                    double *voted_value, int *trip) {
    if (voted_value == NULL) return -1;

    int nv = (valid1 ? 1 : 0) + (valid2 ? 1 : 0) + (valid3 ? 1 : 0);

    if (nv < 3) {
        /* Not enough valid sensors — trip on disagreement */
        *voted_value = 0.0;
        if (trip) *trip = 1;
        return -1;
    }

    /* Check if all three agree within tolerance */
    double d12 = fabs(v1 - v2);
    double d23 = fabs(v2 - v3);
    double d13 = fabs(v1 - v3);

    if (d12 <= tolerance && d23 <= tolerance && d13 <= tolerance) {
        /* All agree — average them */
        *voted_value = (v1 + v2 + v3) / 3.0;
        if (trip) *trip = 0;
        return 0;
    } else {
        /* Disagreement — trip */
        *voted_value = 0.0;
        if (trip) *trip = 1;
        return -1;
    }
}

/**
 * 1-out-of-3 (1oo3) voting: trips if ANY sensor detects a fault.
 * Highest availability, lower safety integrity than 2oo3.
 *
 * @param v1, v2, v3 Three sensor values
 * @param valid1,2,3 Validity flags
 * @param hi_trip     High trip threshold
 * @param lo_trip     Low trip threshold
 * @param voted_value Output: median of valid sensors
 * @param trip        Output: 1 if any sensor trips
 * @return 0 on success
 */
int auctioneer_1oo3(double v1, int valid1,
                    double v2, int valid2,
                    double v3, int valid3,
                    double hi_trip, double lo_trip,
                    double *voted_value, int *trip) {
    if (voted_value == NULL) return -1;

    /* Voted value = median of valid sensors */
    int sel_idx = -1;
    *voted_value = selector_auctioneer_3(v1, valid1, v2, valid2,
                                         v3, valid3, &sel_idx);

    /* Trip if ANY valid sensor exceeds limits */
    int t = 0;
    if (valid1 && (v1 > hi_trip || v1 < lo_trip)) t = 1;
    if (valid2 && (v2 > hi_trip || v2 < lo_trip)) t = 1;
    if (valid3 && (v3 > hi_trip || v3 < lo_trip)) t = 1;

    if (trip) *trip = t;
    return 0;
}

/* =========================================================================
 * L3 — Multi-Channel Signal Processing
 * ========================================================================= */

/**
 * Compute average of N redundant channels, excluding outliers
 *
 * Outliers are identified by their deviation from the median
 * exceeding a configurable threshold.
 *
 * @param values      Array of N sensor values
 * @param valid       Array of N validity flags
 * @param n           Number of channels
 * @param threshold   Outlier detection threshold (absolute deviation)
 * @param result      Output: average of non-outlier valid channels
 * @return Number of channels used in average
 */
int auctioneer_average_excl_outliers(const double *values, const int *valid,
                                     int n, double threshold,
                                     double *result) {
    if (values == NULL || valid == NULL || n <= 0 || result == NULL) {
        return 0;
    }

    /* Compute median */
    int med_idx = -1;
    double med = selector_median(values, valid, n, &med_idx);

    if (med_idx < 0) {
        *result = 0.0;
        return 0;
    }

    /* Average channels within threshold of median */
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < n; i++) {
        if (valid[i] && fabs(values[i] - med) <= threshold) {
            sum += values[i];
            count++;
        }
    }

    *result = (count > 0) ? sum / (double)count : 0.0;
    return count;
}

/**
 * Compute the deviation of each channel from the voted value
 *
 * Useful for generating deviation alarms and identifying
 * drifting sensors.
 *
 * @param values    Array of N sensor values
 * @param valid     Array of N validity flags
 * @param n         Number of channels
 * @param voted     Voted (selected) value
 * @param deviation Output: array of N deviation values
 * @param max_dev   Output: maximum deviation found
 */
void auctioneer_channel_deviations(const double *values, const int *valid,
                                   int n, double voted,
                                   double *deviation, double *max_dev) {
    if (values == NULL || valid == NULL || deviation == NULL) return;

    double mx = 0.0;
    for (int i = 0; i < n; i++) {
        if (valid[i]) {
            deviation[i] = fabs(values[i] - voted);
            if (deviation[i] > mx) mx = deviation[i];
        } else {
            deviation[i] = -1.0; /* Invalid channel indicator */
        }
    }

    if (max_dev) *max_dev = mx;
}

/**
 * Long-term drift detection for redundant sensors
 *
 * Detects slow drift of individual sensors away from the median
 * by tracking cumulative deviation over time.
 *
 * @param values       Current sensor values
 * @param valid        Validity flags
 * @param n            Number of channels
 * @param cum_dev      Input/output: accumulated deviation per channel
 * @param drift_thresh Threshold for declaring a drift fault
 * @param faulted      Output: 1 if channel has drifted, 0 otherwise
 * @return Number of drifted channels
 */
int auctioneer_drift_detect(const double *values, const int *valid,
                            int n, double *cum_dev,
                            double drift_thresh, int *faulted) {
    if (values == NULL || valid == NULL || cum_dev == NULL || faulted == NULL) {
        return 0;
    }

    int med_idx = -1;
    double med = selector_median(values, valid, n, &med_idx);

    int drifted = 0;
    for (int i = 0; i < n; i++) {
        if (valid[i]) {
            double dev = fabs(values[i] - med);
            cum_dev[i] += dev;
            if (cum_dev[i] > drift_thresh && i != med_idx) {
                faulted[i] = 1;
                drifted++;
            }
        } else {
            faulted[i] = 1;
        }
    }

    return drifted;
}

/**
 * Compute the statistical confidence level for a voted signal
 *
 * Based on the spread of the redundant measurements.
 *   σ = standard deviation of valid channels
 *   Confidence = max(0, 100 - 100*(σ / |voted|))
 *
 * @param values Array of N sensor values
 * @param valid  Array of N validity flags
 * @param n      Number of channels
 * @param voted  Voted value
 * @return Confidence [0-100%]
 */
double auctioneer_confidence(const double *values, const int *valid,
                             int n, double voted) {
    if (values == NULL || valid == NULL || n < 2) return 0.0;

    /* Count valid channels */
    int nv = 0;
    for (int i = 0; i < n; i++) {
        if (valid[i]) nv++;
    }

    if (nv < 2) {
        /* Single channel: low confidence */
        return (nv == 1) ? 50.0 : 0.0;
    }

    /* Compute variance relative to voted value */
    double var_sum = 0.0;
    for (int i = 0; i < n; i++) {
        if (valid[i]) {
            double d = values[i] - voted;
            var_sum += d * d;
        }
    }

    double sigma = sqrt(var_sum / (double)nv);

    if (fabs(voted) < 1e-9) {
        /* Voted value near zero — use absolute sigma */
        return (sigma < 1.0) ? 100.0 : (sigma < 10.0 ? 50.0 : 10.0);
    }

    double conf = 100.0 - 100.0 * (sigma / fabs(voted));
    if (conf < 0.0) conf = 0.0;
    if (conf > 100.0) conf = 100.0;

    return conf;
}

/* =========================================================================
 * L5 — Statistical Signal Processing
 * ========================================================================= */

/**
 * Exponentially weighted moving average (EWMA) of voted signal
 *
 * Used for filtering noise on the selected (voted) signal.
 *
 * EWMA(t) = α * value(t) + (1-α) * EWMA(t-1)
 * where α = 2 / (N + 1), N = effective window length
 *
 * @param current   Current voted value
 * @param prev_ewma Previous EWMA value
 * @param alpha     Smoothing factor [0-1] (1=no filtering, 0=no response)
 * @return Filtered value
 */
double auctioneer_ewma(double current, double prev_ewma, double alpha) {
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    return alpha * current + (1.0 - alpha) * prev_ewma;
}

/**
 * Rate-of-change detection for voted signal
 *
 * Computes the rate of change of the voted signal and compares
 * against a threshold. Used for detecting sudden signal changes
 * that may indicate sensor failure or process upset.
 *
 * @param current   Current voted value
 * @param prev      Previous voted value
 * @param dt        Time step [s]
 * @param max_rate  Maximum acceptable rate of change
 * @return 0 if OK, 1 if rate exceeds threshold
 */
int auctioneer_rate_check(double current, double prev,
                          double dt, double max_rate) {
    if (dt <= 0.0) return 0;

    double rate = fabs(current - prev) / dt;
    return (rate > max_rate) ? 1 : 0;
}

/**
 * Freeze detection: check if a sensor is "frozen" (stuck at
 * a constant value while others vary).
 *
 * @param values     Current sensor values
 * @param prev_values Previous sensor values
 * @param valid      Validity flags
 * @param n          Number of channels
 * @param tolerance  Tolerance for "no change"
 * @param frozen     Output: 1 if frozen, 0 otherwise
 * @return Number of frozen channels
 */
int auctioneer_frozen_detect(const double *values, const double *prev_values,
                             const int *valid, int n,
                             double tolerance, int *frozen) {
    if (values == NULL || prev_values == NULL || frozen == NULL) return 0;

    int count = 0;
    for (int i = 0; i < n; i++) {
        frozen[i] = 0;
        if (valid[i]) {
            double diff = fabs(values[i] - prev_values[i]);
            /* Check if other channels are varying while this one isn't */
            int others_vary = 0;
            for (int j = 0; j < n; j++) {
                if (j != i && valid[j]) {
                    double d = fabs(values[j] - prev_values[j]);
                    if (d > tolerance) {
                        others_vary = 1;
                        break;
                    }
                }
            }
            if (diff <= tolerance && others_vary) {
                frozen[i] = 1;
                count++;
            }
        }
    }

    return count;
}
