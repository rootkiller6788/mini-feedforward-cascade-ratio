/**
 * @file override_selector.c
 * @brief Override/Selector Control — Selector Algorithm Implementations
 *
 * Implements all selector algorithms: high-select, low-select,
 * median-select, auctioneering, weighted average, and average.
 * Includes hysteresis variants to prevent selector chatter.
 *
 * Knowledge Coverage:
 *   L2: Selector concepts and operational semantics
 *   L3: Selector function block implementation
 *   L5: Selection algorithms with O(n), O(n²) complexity
 *   L7: Hysteresis and rate-limiting for industrial robustness
 */

#include "override_selector.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>

/* =========================================================================
 * L2 — Basic Selector Implementations (O(n) complexity)
 * ========================================================================= */

double selector_high(const double *values, const int *valid, int n,
                     int *selected_idx) {
    if (values == NULL || valid == NULL || n <= 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    double best = -DBL_MAX;
    int idx = -1;

    for (int i = 0; i < n; i++) {
        if (valid[i] && values[i] > best) {
            best = values[i];
            idx = i;
        }
    }

    if (selected_idx) *selected_idx = idx;
    return (idx >= 0) ? best : 0.0;
}

double selector_low(const double *values, const int *valid, int n,
                    int *selected_idx) {
    if (values == NULL || valid == NULL || n <= 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    double best = DBL_MAX;
    int idx = -1;

    for (int i = 0; i < n; i++) {
        if (valid[i] && values[i] < best) {
            best = values[i];
            idx = i;
        }
    }

    if (selected_idx) *selected_idx = idx;
    return (idx >= 0) ? best : 0.0;
}

double selector_average(const double *values, const int *valid, int n) {
    if (values == NULL || valid == NULL || n <= 0) return 0.0;

    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < n; i++) {
        if (valid[i]) {
            sum += values[i];
            count++;
        }
    }
    return (count > 0) ? sum / (double)count : 0.0;
}

/* =========================================================================
 * L5 — Median Selector (O(n²) worst-case, O(n log n) expected)
 * ========================================================================= */

double selector_median(const double *values, const int *valid, int n,
                       int *selected_idx) {
    if (values == NULL || valid == NULL || n <= 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    /* Count valid entries and copy them */
    int nv = 0;
    for (int i = 0; i < n; i++) {
        if (valid[i]) nv++;
    }

    if (nv == 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    /* Allocate temporary arrays for valid values and their original indices */
    double *v = (double*)malloc((size_t)nv * sizeof(double));
    int    *idxs = (int*)malloc((size_t)nv * sizeof(int));

    if (v == NULL || idxs == NULL) {
        free(v);
        free(idxs);
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    int j = 0;
    for (int i = 0; i < n; i++) {
        if (valid[i]) {
            v[j] = values[i];
            idxs[j] = i;
            j++;
        }
    }

    /* Sort by value (simple insertion sort for small N) */
    for (int i = 1; i < nv; i++) {
        double key_v = v[i];
        int key_idx = idxs[i];
        int k = i - 1;
        while (k >= 0 && v[k] > key_v) {
            v[k + 1] = v[k];
            idxs[k + 1] = idxs[k];
            k--;
        }
        v[k + 1] = key_v;
        idxs[k + 1] = key_idx;
    }

    /* Median: if odd, exact middle; if even, lower middle */
    int mid = (nv - 1) / 2;
    double result = v[mid];
    int orig_idx = idxs[mid];

    free(v);
    free(idxs);

    if (selected_idx) *selected_idx = orig_idx;
    return result;
}

/* =========================================================================
 * L2 — Auctioneering (Median-of-3 Voting — 2oo3 Redundancy Pattern)
 * ========================================================================= */

double selector_auctioneer_3(double v1, int valid1,
                             double v2, int valid2,
                             double v3, int valid3,
                             int *selected_idx) {
    /* Handle cases where not all 3 are valid */
    int nv = (valid1 ? 1 : 0) + (valid2 ? 1 : 0) + (valid3 ? 1 : 0);

    if (nv == 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    if (nv == 1) {
        /* Only one valid: use it */
        if (valid1) { if (selected_idx) *selected_idx = 0; return v1; }
        if (valid2) { if (selected_idx) *selected_idx = 1; return v2; }
        if (selected_idx) *selected_idx = 2;
        return v3;
    }

    if (nv == 2) {
        /* Two valid: average them */
        double sum = 0.0;
        sum += valid1 ? v1 : 0.0;
        sum += valid2 ? v2 : 0.0;
        sum += valid3 ? v3 : 0.0;
        /* For indexing, return the first valid */
        if (valid1) { if (selected_idx) *selected_idx = 0; }
        else if (valid2) { if (selected_idx) *selected_idx = 1; }
        else { if (selected_idx) *selected_idx = 2; }
        return sum / 2.0;
    }

    /* All three valid: standard median-of-3 auctioneering */
    double a = v1, b = v2, c = v3;
    int ia = 0, ib = 1, ic = 2;

    /* Sort to find median using comparisons */
    if (a > b) { double t = a; a = b; b = t; int ti = ia; ia = ib; ib = ti; }
    if (b > c) { double t = b; b = c; c = t; int ti = ib; ib = ic; ic = ti; }
    if (a > b) { double t = a; a = b; b = t; int ti = ia; ia = ib; ib = ti; }

    /* b is now the median */
    if (selected_idx) *selected_idx = ib;
    return b;
}

double selector_auctioneer_n(const double *values, const int *valid,
                             int n, int k_trim, int *selected_idx) {
    if (values == NULL || valid == NULL || n <= 0 || k_trim < 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    /* Count valid entries */
    int nv = 0;
    for (int i = 0; i < n; i++) {
        if (valid[i]) nv++;
    }

    if (nv <= 2 * k_trim) {
        /* Not enough points to trim — fall back to average */
        double sum = 0.0;
        int count = 0;
        for (int i = 0; i < n; i++) {
            if (valid[i]) { sum += values[i]; count++; }
        }
        if (selected_idx) *selected_idx = (count > 0) ? 0 : -1;
        return (count > 0) ? sum / count : 0.0;
    }

    /* Collect valid values */
    double *v = (double*)malloc((size_t)nv * sizeof(double));
    if (v == NULL) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    int j = 0;
    for (int i = 0; i < n; i++) {
        if (valid[i]) v[j++] = values[i];
    }

    /* Sort */
    for (int i = 1; i < nv; i++) {
        double key = v[i];
        int k = i - 1;
        while (k >= 0 && v[k] > key) { v[k + 1] = v[k]; k--; }
        v[k + 1] = key;
    }

    /* Trim k_trim from each end and average the remainder */
    int remaining = nv - 2 * k_trim;
    double sum = 0.0;
    for (int i = k_trim; i < nv - k_trim; i++) {
        sum += v[i];
    }

    double result = sum / remaining;
    free(v);

    if (selected_idx) *selected_idx = 0; /* Index is approximate for general N */
    return result;
}

/* =========================================================================
 * L2 — Weighted Average Selector
 * ========================================================================= */

double selector_weighted(const double *values, const double *weights,
                         const int *valid, int n) {
    if (values == NULL || weights == NULL || valid == NULL || n <= 0) {
        return 0.0;
    }

    double num = 0.0;
    double den = 0.0;

    for (int i = 0; i < n; i++) {
        if (valid[i] && weights[i] > 0.0) {
            num += values[i] * weights[i];
            den += weights[i];
        }
    }

    return (den > 0.0) ? num / den : 0.0;
}

/* =========================================================================
 * L2 — Hysteresis-Enabled Selectors
 * ========================================================================= */

double selector_high_hysteresis(const double *values, const int *valid,
                                int n, int prev_idx, double hysteresis,
                                int *selected_idx) {
    if (values == NULL || valid == NULL || n <= 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    int best_idx = -1;
    double best_val = selector_high(values, valid, n, &best_idx);

    if (best_idx < 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    /* If previous selection is still valid, apply hysteresis */
    if (prev_idx >= 0 && prev_idx < n && valid[prev_idx]) {
        /* Only switch if new best exceeds previous by more than hysteresis */
        if (best_val > values[prev_idx] + hysteresis) {
            if (selected_idx) *selected_idx = best_idx;
            return best_val;
        } else {
            /* Keep previous selection */
            if (selected_idx) *selected_idx = prev_idx;
            return values[prev_idx];
        }
    }

    /* No previous selection or prev is now invalid */
    if (selected_idx) *selected_idx = best_idx;
    return best_val;
}

double selector_low_hysteresis(const double *values, const int *valid,
                               int n, int prev_idx, double hysteresis,
                               int *selected_idx) {
    if (values == NULL || valid == NULL || n <= 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    int best_idx = -1;
    double best_val = selector_low(values, valid, n, &best_idx);

    if (best_idx < 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    /* If previous selection is still valid, apply hysteresis */
    if (prev_idx >= 0 && prev_idx < n && valid[prev_idx]) {
        /* Only switch if new best is lower than previous by more than hysteresis */
        if (best_val < values[prev_idx] - hysteresis) {
            if (selected_idx) *selected_idx = best_idx;
            return best_val;
        } else {
            if (selected_idx) *selected_idx = prev_idx;
            return values[prev_idx];
        }
    }

    if (selected_idx) *selected_idx = best_idx;
    return best_val;
}

double selector_median_hysteresis(const double *values, const int *valid,
                                  int n, int prev_idx, double hysteresis,
                                  int *selected_idx) {
    if (values == NULL || valid == NULL || n <= 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    int med_idx = -1;
    double med_val = selector_median(values, valid, n, &med_idx);

    if (med_idx < 0) {
        if (selected_idx) *selected_idx = -1;
        return 0.0;
    }

    /* Apply hysteresis: only switch if the new median differs from
       the previous selected value by more than the hysteresis band */
    if (prev_idx >= 0 && prev_idx < n && valid[prev_idx]) {
        if (fabs(med_val - values[prev_idx]) > hysteresis) {
            if (selected_idx) *selected_idx = med_idx;
            return med_val;
        } else {
            if (selected_idx) *selected_idx = prev_idx;
            return values[prev_idx];
        }
    }

    if (selected_idx) *selected_idx = med_idx;
    return med_val;
}

/* =========================================================================
 * L3 — Rate-Limited Selector
 * ========================================================================= */

double selector_rate_limit(double raw_value, double prev_output,
                           double rate_limit, double dt) {
    if (rate_limit <= 0.0 || dt <= 0.0) {
        /* No rate limiting */
        return raw_value;
    }

    double max_change = rate_limit * dt;
    double delta = raw_value - prev_output;

    if (delta > max_change) {
        return prev_output + max_change;
    } else if (delta < -max_change) {
        return prev_output - max_change;
    } else {
        return raw_value;
    }
}

/* =========================================================================
 * L3 — Signal Validation and Diagnostics
 * ========================================================================= */

int selector_deviation_check(double *values, int *valid, int n,
                             double threshold) {
    if (values == NULL || valid == NULL || n < 3) return 0;

    /* Find median */
    int med_idx = -1;
    double med = selector_median(values, valid, n, &med_idx);

    if (med_idx < 0) return 0;

    int invalidated = 0;
    for (int i = 0; i < n; i++) {
        if (valid[i] && i != med_idx) {
            if (fabs(values[i] - med) > threshold) {
                valid[i] = 0;
                invalidated++;
            }
        }
    }

    return invalidated;
}

double selector_std_dev(const double *values, const int *valid, int n) {
    if (values == NULL || valid == NULL || n < 2) return 0.0;

    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < n; i++) {
        if (valid[i]) { sum += values[i]; count++; }
    }

    if (count < 2) return 0.0;

    double mean = sum / count;
    double var_sum = 0.0;
    for (int i = 0; i < n; i++) {
        if (valid[i]) {
            double d = values[i] - mean;
            var_sum += d * d;
        }
    }

    return sqrt(var_sum / (count - 1)); /* Sample standard deviation (Bessel) */
}

double selector_range(const double *values, const int *valid, int n) {
    if (values == NULL || valid == NULL || n < 2) return 0.0;

    double min_val = DBL_MAX;
    double max_val = -DBL_MAX;
    int found = 0;

    for (int i = 0; i < n; i++) {
        if (valid[i]) {
            if (values[i] < min_val) min_val = values[i];
            if (values[i] > max_val) max_val = values[i];
            found = 1;
        }
    }

    return found ? (max_val - min_val) : 0.0;
}

void selector_sort_values(double *values, int n) {
    if (values == NULL || n <= 1) return;

    /* Insertion sort — efficient for small arrays typical of selector inputs */
    for (int i = 1; i < n; i++) {
        double key = values[i];
        int j = i - 1;
        while (j >= 0 && values[j] > key) {
            values[j + 1] = values[j];
            j--;
        }
        values[j + 1] = key;
    }
}

void selector_sort_with_indices(double *values, int *indices, int n) {
    if (values == NULL || indices == NULL || n <= 1) return;

    for (int i = 1; i < n; i++) {
        double key_v = values[i];
        int key_i = indices[i];
        int j = i - 1;
        while (j >= 0 && values[j] > key_v) {
            values[j + 1] = values[j];
            indices[j + 1] = indices[j];
            j--;
        }
        values[j + 1] = key_v;
        indices[j + 1] = key_i;
    }
}
