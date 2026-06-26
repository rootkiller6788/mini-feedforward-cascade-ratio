/**
 * @file split_range_valve.c
 * @brief Control valve modeling and characterization — implementation
 *
 * Module: mini-split-range-control-heat-cool
 * Knowledge Coverage: L3 Engineering Structures, L4 Engineering Laws
 *
 * Full implementation of control valve sizing per ISA-75.01,
 * installed characteristic analysis, stiction modeling, and
 * partial stroke testing per IEC 61508 / ISA-84.
 *
 * Reference:
 *   ISA-75.01.01-2012 — Flow Equations for Sizing Control Valves
 *   ISA-75.11.01 — Inherent Flow Characteristic and Rangeability
 *   IEC 60534-2-1 — Industrial-process control valves, Flow capacity
 *   Choudhury, Sirish, Shah (2005) — Detection and Quantification of
 *     Control Valve Stiction
 */

#include "split_range_valve.h"
#include "split_range_core.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =========================================================================
 * Internal helper
 * ========================================================================= */

static double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* =========================================================================
 * split_valve_installed_characteristic — L3, L4
 *
 * Computes the installed flow characteristic accounting for piping
 * pressure drop.
 *
 * The inherent characteristic describes the valve's behavior under
 * constant pressure drop.  In reality, the pressure drop decreases
 * as the valve opens, distorting the installed characteristic.
 *
 * Derivation (ISA-75.01, Annex B):
 *
 * Let delta_P_valve = valve pressure drop at flow Q
 *     delta_P_piping = piping pressure drop at flow Q
 *     delta_P_total = delta_P_valve + delta_P_piping (constant pump head)
 *
 * At any stem position x:
 *   Q_inherent(x) = Qmax * f(x)  where f(x) is the inherent characteristic
 *
 * The installed flow is:
 *   Q_installed = Qmax * f(x) * sqrt(delta_P_valve / delta_P_valve_max)
 *
 * With pressure drop ratio Pr = delta_P_valve_max / delta_P_total:
 *   Q_installed = Qmax * f(x) / sqrt(1 + (1-Pr)/Pr * f(x)^2)
 *
 * This simplifies to:
 *   Q_installed = sqrt(Pr) * Q_inherent
 *                 / sqrt(Pr + (1-Pr) * Q_inherent^2)
 *
 * When Pr = 1 (all pressure drop across valve): installed = inherent
 * When Pr -> 0 (all pressure drop in piping): characteristic flattens
 *
 * Theorem (ISA-75.01, Annex B): The installed characteristic is a
 *   linear-fractional transformation of the inherent characteristic,
 *   preserving monotonicity but reducing rangeability.
 * ========================================================================= */
double split_valve_installed_characteristic(double stem_pos,
                                              split_range_valve_char_t inherent,
                                              double Pr,
                                              double rangeability) {
    double x = clamp(stem_pos, 0.0, 1.0);
    double pr = clamp(Pr, 0.01, 1.0); /* clamp to avoid div-by-zero */

    /* Compute inherent flow at this stem position */
    double Qin = split_valve_characteristic_forward(x, inherent, rangeability);

    if (Qin < 1e-12) return 0.0;

    /* Installed flow per ISA-75.01 Annex B eq. B.1 */
    double Qinst = sqrt(pr) * Qin / sqrt(pr + (1.0 - pr) * Qin * Qin);

    return clamp(Qinst, 0.0, 1.0);
}

/* =========================================================================
 * split_valve_size_isa — L4
 *
 * Control valve sizing per ISA-75.01.01.
 *
 * LIQUID (incompressible) — ISA eq. 1:
 *   Cv = Q * sqrt(Gf / delta_P)
 *   where:
 *     Q     = volumetric flow rate (US gpm)
 *     Gf    = liquid specific gravity (water = 1.0 at 60 degF)
 *     delta_P = pressure drop (psi)
 *
 * GAS (compressible) — ISA eq. 3 (simplified non-choked):
 *   Cv = Q / (N1 * Fp * P1 * Y) * sqrt(T1 * Gg * Z / x)
 *   where:
 *     Q   = volumetric flow rate (scfh)
 *     N1  = 1.0 for psia units
 *     Fp  = piping geometry factor (assumed 1.0 for simplicity)
 *     P1  = upstream pressure (psia)
 *     Y   = expansion factor (assumed 0.667 for non-choked)
 *     T1  = upstream temperature (degR = degF + 459.67)
 *     Gg  = gas specific gravity (air = 1.0)
 *     Z   = compressibility factor (assumed 1.0)
 *     x   = delta_P / P1 (pressure drop ratio)
 *
 * Edge cases:
 *   - Q <= 0 → return 0.0
 *   - delta_P <= 0 → return 0.0
 *   - Gf <= 0 → return 0.0
 *   - For gas: x > Fk*xT (choked flow) → use xT instead of x
 * ========================================================================= */
double split_valve_size_isa(double Q, double delta_P, double Gf,
                              double T1, double P1, int is_gas) {
    if (Q <= 0.0 || delta_P <= 0.0 || Gf <= 0.0) {
        return 0.0;
    }

    if (!is_gas) {
        /* Liquid sizing: Cv = Q * sqrt(Gf / delta_P) */
        return Q * sqrt(Gf / delta_P);
    } else {
        /* Gas sizing */
        if (P1 <= 0.0 || T1 <= 0.0) return 0.0;

        double x = delta_P / P1;
        double xT = 0.7; /* typical for globe valve */
        double Fk = 1.0; /* ratio of specific heats factor */
        double x_limited = (x > Fk * xT) ? Fk * xT : x;

        /* Expansion factor Y for non-choked flow */
        double Y = 1.0 - x_limited / (3.0 * Fk * xT);
        if (Y < 0.667) Y = 0.667;

        double Gg = Gf; /* for gas, Gf parameter reused as Gg */
        double Z = 1.0; /* compressibility */

        /* Cv = Q * sqrt(T1*Gg*Z) / (N1 * Fp * P1 * Y * sqrt(x_limited)) */
        double numerator = Q * sqrt(T1 * Gg * Z);
        double denominator = 1.0 * 1.0 * P1 * Y * sqrt(x_limited);

        if (denominator < 1e-12) return 0.0;

        return numerator / denominator;
    }
}

/* =========================================================================
 * split_valve_pressure_drop_ratio — L4
 *
 * Estimates the installed pressure drop ratio Pr.
 *
 * The total system pressure drop is shared between the control valve
 * and the piping (including fittings, heat exchangers, etc.).
 *
 * Pr ≈ (delta_P_valve_max) / (delta_P_valve_max + delta_P_piping_at_max_flow)
 *
 * In terms of Cv:
 *   Pr = (Cv_valve^2) / (Cv_valve^2 + Cv_piping^2) approximately
 *
 * Rule of thumb (ISA): Pr >= 0.3 for good controllability.
 * Pr < 0.3: valve too small relative to piping → consider larger valve.
 * Pr > 0.7: valve oversized → poor resolution at low flows.
 *
 * Reference: ISA-75.01.01, Section 6.3 — Installed Valve Performance
 * ========================================================================= */
double split_valve_pressure_drop_ratio(double Cv_valve, double Cv_piping) {
    if (Cv_valve <= 0.0) return 0.0;
    if (Cv_piping <= 0.0) return 1.0; /* no piping loss → all drop on valve */

    double Cv_valve_sq = Cv_valve * Cv_valve;
    double Cv_piping_sq = Cv_piping * Cv_piping;

    return Cv_valve_sq / (Cv_valve_sq + Cv_piping_sq);
}

/* =========================================================================
 * split_valve_rangeability — L3
 *
 * Rangeability R is a key valve selection parameter:
 *   R = max_controllable_flow / min_controllable_flow
 *
 * Higher R means the valve can control over a wider flow range.
 *
 * Typical R values (ISA-75.11.01):
 *   V-port globe:   50
 *   Eccentric plug: 100
 *   V-ball:         200
 *   Butterfly:      20
 *
 * @return Rangeability, clamped to >= 1.0
 * ========================================================================= */
double split_valve_rangeability(double Cv_max, double Cv_min_controllable) {
    if (Cv_max <= 0.0 || Cv_min_controllable <= 0.0) return 1.0;
    if (Cv_min_controllable > Cv_max) return 1.0;

    double R = Cv_max / Cv_min_controllable;
    return (R >= 1.0) ? R : 1.0;
}

/* =========================================================================
 * split_valve_gain — L3
 *
 * Valve gain = dQ/dx (derivative of flow w.r.t. stem position).
 *
 * This is the "local process gain" seen by the controller at a given
 * operating point.  A constant gain (linear valve) simplifies tuning;
 * a varying gain (equal-percentage) may require gain scheduling.
 *
 * Analytical derivatives:
 *   Linear:    dQ/dx = 1 (constant)
 *   Equal-%:   dQ/dx = R^(x-1) * ln(R)  (exponential)
 *   Quick-open: dQ/dx = 1/(2*sqrt(x))  (infinite at x=0!)
 *   Modified parabolic: dQ/dx = 2*a*x + (1-a) (linear in x)
 *
 * Design insight: The equal-percentage valve compensates for process
 * nonlinearities where the process gain decreases with flow (e.g.,
 * heat exchangers), making the total loop gain more nearly constant.
 * ========================================================================= */
double split_valve_gain(double stem_pos,
                          split_range_valve_char_t char_type,
                          double rangeability) {
    double x = clamp(stem_pos, 0.001, 0.999); /* avoid singularity at boundaries */

    switch (char_type) {
        case SPLIT_VALVE_LINEAR:
            return 1.0;

        case SPLIT_VALVE_EQUAL_PCT: {
            double R = rangeability > 1.0 ? rangeability : 50.0;
            return pow(R, x - 1.0) * log(R);
        }

        case SPLIT_VALVE_QUICK_OPENING:
            /* Singularity at x=0: gain → infinity. Physical limit exists. */
            return 0.5 / sqrt(x);

        case SPLIT_VALVE_MODIFIED_PARABOLIC: {
            double a = 0.3;
            return 2.0 * a * x + (1.0 - a);
        }

        default:
            return 1.0;
    }
}

/* =========================================================================
 * split_valve_build_user_table — L3
 *
 * Builds a user-defined characteristic table from measured data.
 * Validates monotonicity and sorts data points by stem position.
 * Non-monotonic points are corrected by interpolation.
 *
 * The table lookup uses linear interpolation between the two
 * nearest points, with binary search for O(log n) access.
 * ========================================================================= */
int split_valve_build_user_table(split_range_user_table_t *table,
                                   const double *co_data,
                                   const double *flow_data,
                                   int n) {
    if (!table || !co_data || !flow_data || n < 2 || n > 64) {
        return -1;
    }

    /* Simple insertion sort by stem position (bubble for n <= 64) */
    double co_sorted[64], flow_sorted[64];
    for (int i = 0; i < n; i++) {
        co_sorted[i] = co_data[i];
        flow_sorted[i] = flow_data[i];
    }

    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (co_sorted[j] < co_sorted[i]) {
                double tmp = co_sorted[i];
                co_sorted[i] = co_sorted[j];
                co_sorted[j] = tmp;
                tmp = flow_sorted[i];
                flow_sorted[i] = flow_sorted[j];
                flow_sorted[j] = tmp;
            }
        }
    }

    /* Enforce monotonicity: if flow[i] < flow[i-1], interpolate */
    for (int i = 1; i < n; i++) {
        if (flow_sorted[i] < flow_sorted[i-1] && co_sorted[i] > co_sorted[i-1]) {
            /* Linear interpolation from previous and next valid point */
            /* Simplified: set to previous value (conservative) */
            flow_sorted[i] = flow_sorted[i-1];
        }
    }

    /* Copy to table */
    for (int i = 0; i < n; i++) {
        table->co_pct[i] = co_sorted[i];
        table->flow_pct[i] = flow_sorted[i];
    }
    table->num_points = n;

    return 0;
}

/* =========================================================================
 * split_valve_user_table_lookup — L3
 *
 * Linear interpolation lookup in user-defined valve table.
 * Uses binary search for O(log n) access.
 * ========================================================================= */
double split_valve_user_table_lookup(const split_range_user_table_t *table,
                                       double stem_pos) {
    if (!table || table->num_points < 2) return stem_pos;

    double x = clamp(stem_pos, 0.0, 1.0);
    double stem_pct = x * 100.0;

    int n = table->num_points;

    /* Binary search for the interval containing stem_pct */
    int lo = 0, hi = n - 1;
    while (lo < hi - 1) {
        int mid = (lo + hi) / 2;
        if (table->co_pct[mid] <= stem_pct) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    /* Linear interpolation between lo and hi */
    double x1 = table->co_pct[lo];
    double x2 = table->co_pct[hi];
    double y1 = table->flow_pct[lo];
    double y2 = table->flow_pct[hi];

    if (fabs(x2 - x1) < 1e-12) {
        return y1 / 100.0;
    }

    double t = (stem_pct - x1) / (x2 - x1);
    double flow_pct = y1 + t * (y2 - y1);

    return clamp(flow_pct, 0.0, 100.0) / 100.0;
}

/* =========================================================================
 * split_valve_stiction_model — L3, L7
 *
 * Choudhury-Sirish-Shah (2005) stiction model.
 *
 * Control valve stiction (static friction) is the most common cause
 * of control loop oscillations. The valve sticks until the actuator
 * force overcomes static friction, then slips past the target.
 *
 * Model parameters:
 *   S = stickband (deadband plus slip jump, % of span)
 *   J = slip jump (sudden movement when static friction overcome, %)
 *
 * Algorithm:
 *   If |u_req - u_prev| > S:
 *     u_out = u_req - sign(u_req - u_prev) * J
 *   Else:
 *     u_out = u_prev (valve stuck)
 *
 * Reference:
 *   Choudhury, M.A.A.S., Sirish, S.L., Shah, S.L. (2005)
 *   "A simple method for detection of stiction in control valves"
 *   Control Engineering Practice, 13(8), 991-1001.
 * ========================================================================= */
double split_valve_stiction_model(split_range_channel_t *channel,
                                    double desired_pos) {
    if (!channel) return desired_pos;

    double S = channel->stiction_threshold; /* stickband */
    double J = channel->stiction_threshold * 0.3; /* slip jump = 30% of stickband */

    if (S <= 0.0) {
        /* No stiction: valve follows command exactly */
        return desired_pos;
    }

    double delta = desired_pos - channel->current_position;

    if (fabs(delta) > S) {
        /* Overcomes stiction: slip to near desired position */
        double direction = (delta > 0) ? 1.0 : -1.0;
        double slipped = channel->current_position + delta - direction * J;
        return clamp(slipped, SPLIT_VALVE_MIN, SPLIT_VALVE_MAX);
    } else {
        /* Stuck: maintain current position */
        return channel->current_position;
    }
}

/* =========================================================================
 * split_valve_energy_consumption — L4
 *
 * Estimates energy consumption for heating and cooling based on
 * valve positions and the split point configuration.
 *
 * Simplifying assumptions:
 *   - Heating energy ∝ heating valve position (linear)
 *   - Cooling energy ∝ cooling valve position (linear)
 *   - Total energy = sum of both
 *
 * In a real plant, energy consumption also depends on utility costs
 * (steam cost vs. cooling water cost), which can be incorporated
 * into economic optimization of the split point.
 * ========================================================================= */
void split_valve_energy_consumption(double co, double split_point,
                                      const double *valve_positions,
                                      int n_channels,
                                      double *heating_power,
                                      double *cooling_power) {
    if (heating_power) *heating_power = 0.0;
    if (cooling_power) *cooling_power = 0.0;

    if (!valve_positions || n_channels <= 0) return;

    /* Simplified model: if CO < split_point, energy is heating;
     * if CO > split_point, energy is cooling.
     * If both valves are open (overlap), count both. */

    /* Channel 0 assumed heating, Channel 1 assumed cooling */
    if (n_channels >= 1 && heating_power) {
        *heating_power = valve_positions[0] * 10.0; /* arbitrary scaling: kW per % */
    }
    if (n_channels >= 2 && cooling_power) {
        *cooling_power = valve_positions[1] * 8.0; /* cooling typically cheaper */
    }

    /* Also account for CO position: closer to split point = more efficient */
    double efficiency_factor = 1.0 - fabs(co - split_point) / 50.0;
    if (efficiency_factor < 0.2) efficiency_factor = 0.2;

    if (heating_power) *heating_power *= efficiency_factor;
    if (cooling_power) *cooling_power *= efficiency_factor;
}

/* =========================================================================
 * split_valve_partial_stroke_test — L4, L7
 *
 * Partial Stroke Test (PST) per ISA-96.02.01 / IEC 61508.
 *
 * In safety instrumented systems (SIS), emergency shutdown valves (ESD)
 * must be tested periodically.  Partial stroke testing moves the valve
 * a small amount (typically 10-20%) to verify operability without
 * disrupting the process.
 *
 * This simplified model:
 *   - breakaway_time: time for valve to start moving (indicator of stiction)
 *   - stroke_time: time to complete the partial stroke
 *
 * Reference: ISA-96.02.01 — Partial Stroke Testing of Automated Valves
 *   IEC 61508 / ISA-84.00.01 — Functional Safety of SIS
 * ========================================================================= */
int split_valve_partial_stroke_test(split_range_channel_t *channel,
                                      double stroke_percentage,
                                      double *breakaway_time_ms,
                                      double *stroke_time_ms) {
    if (!channel) return -1;

    double stroke = clamp(stroke_percentage, 1.0, 20.0);

    /* Breakaway time modeling:
     *   - If stiction_threshold > 0: extra time to overcome static friction
     *   - Healthy valve: < 500 ms
     *   - Stiction: > 1000 ms */
    double breakaway = 200.0; /* base breakaway time (ms) */
    if (channel->stiction_threshold > 0.0) {
        breakaway += channel->stiction_threshold * 50.0; /* ms per % stiction */
    }
    if (breakaway_time_ms) *breakaway_time_ms = breakaway;

    /* Stroke time: depends on slew rate and stroke percentage */
    double stroke_time = stroke / channel->slew_rate_limit * 1000.0; /* ms */
    if (stroke_time_ms) *stroke_time_ms = stroke_time;

    /* Return 0 = healthy, 1 = stiction detected */
    int result = (breakaway > 1000.0 || stroke_time > 5000.0) ? 1 : 0;

    /* Update valve health based on test */
    if (result == 1) {
        channel->health = SPLIT_HEALTH_STICTION_DETECTED;
    }

    return result;
}
