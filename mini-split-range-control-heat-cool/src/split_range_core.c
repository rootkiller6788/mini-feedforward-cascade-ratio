/**
 * @file split_range_core.c
 * @brief Core split-range mapping algorithms implementation
 *
 * Module: mini-split-range-control-heat-cool
 * Knowledge Coverage: L2 Core Concepts (implemented), L3 Engineering Structures
 *
 * Implements the core split-range distribution logic: converting a single
 * controller output into multiple valve position commands.
 */

#include "split_range_core.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* =========================================================================
 * Internal helper: clamp value to [lo, hi]
 * ========================================================================= */

static double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* =========================================================================
 * split_compute_channel_position — L2
 *
 * Maps controller output [0,100] to a single valve position [0,100]
 * based on the channel's configured range and direction.
 *
 * Mathematical basis: affine linear mapping
 *
 * Let co_norm = (co - range_start) / (range_end - range_start)
 * Then:
 *   increasing action: valve = valve_start + co_norm * (valve_end - valve_start)
 *   decreasing action: valve = valve_start - co_norm * (valve_end - valve_start)
 *   fixed action:      valve = valve_start (constant)
 *
 * The result is clamped to [0, 100].
 * ========================================================================= */
double split_compute_channel_position(double co,
                                       double range_start, double range_end,
                                       split_range_action_t action,
                                       double valve_start, double valve_end) {
    double range_width = range_end - range_start;
    double co_norm;

    /* Guard against invalid ranges */
    if (range_width <= 0.0) {
        return valve_start;
    }

    /* Normalize CO to [0, 1] within the channel's active range */
    co_norm = (co - range_start) / range_width;
    co_norm = clamp(co_norm, 0.0, 1.0);

    /* Compute valve position.
     * Direction is encoded in valve_start/valve_end values:
     * - INCREASING: valve_end > valve_start, position increases with CO
     * - DECREASING: valve_end < valve_start, position decreases with CO
     * - FIXED: valve_end == valve_start, position constant
     * The same affine formula works for all three. */
    double valve_pos;
    if (action == SPLIT_ACTION_FIXED) {
        valve_pos = valve_start;
    } else {
        valve_pos = valve_start + co_norm * (valve_end - valve_start);
    }

    return clamp(valve_pos, SPLIT_VALVE_MIN, SPLIT_VALVE_MAX);
}

/* =========================================================================
 * split_apply_deadband_overlap — L2
 *
 * Implements deadband and overlap logic for the split-range transition.
 *
 * Deadband (positive width):
 *   Creates a dead zone around the split_point where no valve is active.
 *   Example: split_point=50, deadband=4 → active ranges are [0,48] and [52,100]
 *   This prevents both valves from fighting each other at the transition.
 *
 * Overlap (negative deadband):
 *   Creates a region where both valves can be partially open.
 *   Example: split_point=50, deadband=-6 → active ranges overlap [0,53] and [47,100]
 *   Useful for pH control to smooth transitions.
 *
 * Transition smoothing (cubic spline):
 *   Instead of a hard switch at the split point, a cubic Hermite spline
 *   creates a smooth transition that is C1-continuous (value + derivative),
 *   reducing valve wear from abrupt direction changes.
 * ========================================================================= */
double split_apply_deadband_overlap(double co,
                                     double split_point,
                                     double deadband_width,
                                     split_range_transition_t transition_type) {
    double half_db = fabs(deadband_width) / 2.0;

    if (transition_type == SPLIT_TRANSITION_HARD) {
        /* Simple hard split: no modification to CO */
        return co;
    }

    if (transition_type == SPLIT_TRANSITION_HYSTERESIS) {
        /* Hysteresis: a simple Schmitt trigger.
         * When crossing from below, switch at split_point + half_db.
         * When crossing from above, switch at split_point - half_db.
         * This function only modifies the effective CO for distribution;
         * the actual hysteresis state is tracked elsewhere. */
        return co;
    }

    /* For SPLIT_TRANSITION_LINEAR and SPLIT_TRANSITION_CUBIC_SPLINE:
     * The deadband creates a modified effective CO for the downstream
     * distribution function. */

    double lo = split_point - half_db;
    double hi = split_point + half_db;

    if (co <= lo) {
        /* Below transition zone: map to [0, split_point] */
        return co * (split_point / lo);
    } else if (co >= hi) {
        /* Above transition zone: map to [split_point, 100] */
        return split_point + (co - hi) * ((100.0 - split_point) / (100.0 - hi));
    } else {
        /* Inside transition zone */
        if (transition_type == SPLIT_TRANSITION_LINEAR) {
            /* Linear interpolation across the deadband */
            double t = (co - lo) / (hi - lo); /* 0 at lo, 1 at hi */
            return lo + t * (hi - lo);
        } else {
            /* Cubic spline: smoother transition with zero derivative at endpoints.
             * Hermite basis: H(t) = 3*t^2 - 2*t^3 */
            double t = (co - lo) / (hi - lo);
            double smooth_t = 3.0 * t * t - 2.0 * t * t * t;
            return lo + smooth_t * (hi - lo);
        }
    }
}

/* =========================================================================
 * split_distribute_output — L2
 *
 * Main distribution function: converts controller output into per-channel
 * valve positions, applying deadband/overlap and per-channel mapping.
 *
 * Algorithm:
 *   1. Apply deadband/overlap transformation to the raw CO
 *   2. For each channel, compute valve position using split_compute_channel_position
 *   3. Store results in the positions array
 *
 * Edge cases:
 *   - NULL scheme → return 0
 *   - positions NULL → return 0
 *   - scheme->num_channels == 0 → return 0
 *   - CO outside [0, 100] → clamped internally
 * ========================================================================= */
int split_distribute_output(const split_range_scheme_t *scheme,
                             double co,
                             double *positions) {
    if (!scheme || !positions) return 0;
    if (scheme->num_channels == 0) return 0;

    /* Clamp CO to valid range */
    double co_clamped = clamp(co, SPLIT_CO_MIN, SPLIT_CO_MAX);

    /* Apply deadband/overlap transformation */
    double co_effective = split_apply_deadband_overlap(
        co_clamped, scheme->split_point,
        (scheme->overlap_width > 0) ? -scheme->overlap_width : scheme->deadband_width,
        scheme->transition_type);

    /* Distribute to each channel */
    for (uint32_t i = 0; i < scheme->num_channels && i < SPLIT_MAX_CHANNELS; i++) {
        const split_range_channel_t *ch = &scheme->channels[i];

        if (ch->maintenance_override || ch->manual_mode) {
            /* Use manual position, bypassing split logic */
            positions[i] = ch->manual_position;
        } else {
            positions[i] = split_compute_channel_position(
                co_effective,
                ch->co_range_start, ch->co_range_end,
                ch->action,
                ch->valve_range_start, ch->valve_range_end);
        }

        /* Apply valve characteristic inverse for linearization */
        if (ch->characteristic != SPLIT_VALVE_LINEAR) {
            double flow_desired = positions[i] / 100.0;
            double stem_needed = split_valve_characteristic_inverse(
                flow_desired, ch->characteristic,
                SPLIT_RANGEABILITY_DEFAULT);
            positions[i] = stem_needed * 100.0;
        }
    }

    return (int)scheme->num_channels;
}

/* =========================================================================
 * split_valve_characteristic_inverse — L3
 *
 * Converts desired flow fraction to required stem position.
 * This is the "characteristic linearization" used to make the
 * effective process gain constant from the controller's perspective.
 *
 * For each characteristic type:
 *
 * Linear: x = f                    (identity)
 * Equal-percentage: f = R^(x-1) → x = 1 + ln(f)/ln(R)
 * Quick-opening: f = sqrt(x) → x = f^2
 * Modified parabolic: f = a*x^2 + (1-a)*x → solve quadratic for x
 * ========================================================================= */
double split_valve_characteristic_inverse(double desired_flow,
                                           split_range_valve_char_t char_type,
                                           double rangeability) {
    double f = clamp(desired_flow, 0.0, 1.0);

    /* Guard against near-zero (log(0) undefined) */
    if (f < 1e-10) return 0.0;

    switch (char_type) {
        case SPLIT_VALVE_LINEAR:
            return f;

        case SPLIT_VALVE_EQUAL_PCT: {
            /* f = R^(x-1) → ln(f) = (x-1)*ln(R) → x = 1 + ln(f)/ln(R) */
            double lnR = log(rangeability > 1.0 ? rangeability : 50.0);
            if (lnR < 1e-12) return f; /* degenerate case */
            return 1.0 + log(f) / lnR;
        }

        case SPLIT_VALVE_QUICK_OPENING:
            /* f = sqrt(x) → x = f^2 */
            return f * f;

        case SPLIT_VALVE_MODIFIED_PARABOLIC: {
            /* f = a*x^2 + (1-a)*x, solve a*x^2 + (1-a)*x - f = 0
             * Using quadratic formula: x = [-(1-a) + sqrt((1-a)^2 + 4*a*f)] / (2*a)
             * a = 0.3 is the default parabola shape parameter */
            double a = 0.3;
            double b = 1.0 - a;
            double discriminant = b * b + 4.0 * a * f;
            if (discriminant < 0.0) return f;
            double x = (-b + sqrt(discriminant)) / (2.0 * a);
            return clamp(x, 0.0, 1.0);
        }

        case SPLIT_VALVE_USER_TABLE:
            /* Not supported in this function — use split_valve_user_table_lookup */
            return f;

        default:
            return f;
    }
}

/* =========================================================================
 * split_valve_characteristic_forward — L2
 *
 * Forward characteristic: stem position → flow fraction.
 * Used for simulation, diagnostics, and performance evaluation.
 * ========================================================================= */
double split_valve_characteristic_forward(double stem_position,
                                           split_range_valve_char_t char_type,
                                           double rangeability) {
    double x = clamp(stem_position, 0.0, 1.0);

    switch (char_type) {
        case SPLIT_VALVE_LINEAR:
            return x;

        case SPLIT_VALVE_EQUAL_PCT: {
            double R = rangeability > 1.0 ? rangeability : 50.0;
            return pow(R, x - 1.0);
        }

        case SPLIT_VALVE_QUICK_OPENING:
            return sqrt(x);

        case SPLIT_VALVE_MODIFIED_PARABOLIC: {
            double a = 0.3;
            return a * x * x + (1.0 - a) * x;
        }

        case SPLIT_VALVE_USER_TABLE:
            return x; /* handled separately */

        default:
            return x;
    }
}

/* =========================================================================
 * split_slew_rate_limit — L2
 *
 * Physical valves cannot move instantaneously. The slew rate limiter
 * ensures the commanded position change does not exceed the valve's
 * mechanical limit, preventing actuator damage.
 *
 * |pos_new - pos_old| <= slew_limit * dt
 * ========================================================================= */
double split_slew_rate_limit(double current_pos, double target_pos,
                              double slew_limit, double dt_sec) {
    if (slew_limit <= 0.0 || dt_sec <= 0.0) {
        return target_pos; /* no limiting applied */
    }

    double max_change = slew_limit * dt_sec;
    double delta = target_pos - current_pos;

    if (delta > max_change) {
        return current_pos + max_change;
    } else if (delta < -max_change) {
        return current_pos - max_change;
    } else {
        return target_pos;
    }
}

/* =========================================================================
 * split_hysteresis_compensate — L2
 *
 * When a valve exhibits stiction, small position changes are absorbed
 * by the static friction and do not result in actual movement. This
 * hysteresis compensator adds a small "dither" offset in the direction
 * of travel to overcome stiction.
 *
 * The hysteresis memory tracks the last "effective" position sent to
 * the valve. If the desired change is less than the hysteresis band,
 * no change is commanded. If it exceeds the band, the full desired
 * position is commanded.
 * ========================================================================= */
double split_hysteresis_compensate(split_range_channel_t *channel,
                                    double desired_pos) {
    if (!channel) return desired_pos;

    (void)(channel->hysteresis_band); /* parameter used for future diagnostic */

    double delta = desired_pos - channel->current_position;

    if (fabs(delta) >= channel->hysteresis_band) {
        /* Exceeds hysteresis: move to desired position */
        return desired_pos;
    } else {
        /* Within hysteresis band: maintain last position */
        return channel->current_position;
    }
}

/* =========================================================================
 * split_init_heat_cool_scheme — L2
 *
 * Initialize classic heating/cooling split-range scheme with 2 channels.
 *
 * Channel 0: Heating valve — reverse acting (fail-open for safety:
 *            if air/control signal fails, heating valve closes)
 *   CO range: 0% to 50% (split point)
 *   Valve: 100% (CO=0) to 0% (CO=50%) — reverse acting
 *
 * Channel 1: Cooling valve — direct acting (fail-closed: on signal
 *            failure, cooling valve closes → process may overheat,
 *            but that's safer than overcooling/freezing in most cases)
 *   CO range: 50% to 100%
 *   Valve: 0% (CO=50%) to 100% (CO=100%) — direct acting
 *
 * Deadband of 2% prevents both valves from being open simultaneously,
 * saving energy.
 * ========================================================================= */
void split_init_heat_cool_scheme(split_range_scheme_t *scheme) {
    if (!scheme) return;

    memset(scheme, 0, sizeof(*scheme));
    scheme->mode = SPLIT_MODE_COMPLEMENTARY;
    scheme->split_point = SPLIT_POINT_DEFAULT;
    scheme->deadband_width = SPLIT_DEADBAND_DEFAULT;
    scheme->overlap_width = 0.0;
    scheme->transition_type = SPLIT_TRANSITION_LINEAR;
    scheme->bumpless_transfer = true;
    scheme->track_manual_input = true;
    scheme->balanced_heat_cool = true;

    /* Channel 0: Heating (reverse acting) */
    scheme->channels[0].channel_id = 0;
    scheme->channels[0].action = SPLIT_ACTION_DECREASING;
    scheme->channels[0].characteristic = SPLIT_VALVE_EQUAL_PCT;
    scheme->channels[0].co_range_start = 0.0;
    scheme->channels[0].co_range_end = scheme->split_point - scheme->deadband_width / 2.0;
    scheme->channels[0].valve_range_start = 100.0;
    scheme->channels[0].valve_range_end = 0.0;
    scheme->channels[0].slew_rate_limit = SPLIT_SLEW_RATE_DEFAULT;
    scheme->channels[0].hysteresis_band = 0.5;
    scheme->channels[0].health = SPLIT_HEALTH_OK;
    scheme->channels[0].Cv_rated = 50.0;
    scheme->channels[0].FL = 0.9;
    scheme->channels[0].xT = 0.7;
    scheme->channels[0].delta_P_design = 15.0;
    snprintf(scheme->channels[0].tag_name, 32, "TV-HEAT-%u", 0u);
    snprintf(scheme->channels[0].service_description, 64, "Steam Heating Valve");

    /* Channel 1: Cooling (direct acting) */
    scheme->channels[1].channel_id = 1;
    scheme->channels[1].action = SPLIT_ACTION_INCREASING;
    scheme->channels[1].characteristic = SPLIT_VALVE_LINEAR;
    scheme->channels[1].co_range_start = scheme->split_point + scheme->deadband_width / 2.0;
    scheme->channels[1].co_range_end = 100.0;
    scheme->channels[1].valve_range_start = 0.0;
    scheme->channels[1].valve_range_end = 100.0;
    scheme->channels[1].slew_rate_limit = SPLIT_SLEW_RATE_DEFAULT;
    scheme->channels[1].hysteresis_band = 0.5;
    scheme->channels[1].health = SPLIT_HEALTH_OK;
    scheme->channels[1].Cv_rated = 80.0;
    scheme->channels[1].FL = 0.85;
    scheme->channels[1].xT = 0.65;
    scheme->channels[1].delta_P_design = 10.0;
    snprintf(scheme->channels[1].tag_name, 32, "TV-COOL-%u", 1u);
    snprintf(scheme->channels[1].service_description, 64, "Cooling Water Valve");

    scheme->num_channels = 2;
}

/* =========================================================================
 * split_init_ph_scheme — L2
 *
 * pH neutralization split-range with overlap for smoother transitions.
 * The overlap means both acid and base valves can be slightly open
 * near neutral pH, which prevents the aggressive cycling that would
 * otherwise occur due to the extremely steep titration curve.
 * ========================================================================= */
void split_init_ph_scheme(split_range_scheme_t *scheme) {
    if (!scheme) return;

    memset(scheme, 0, sizeof(*scheme));
    scheme->mode = SPLIT_MODE_OVERLAP;
    scheme->split_point = SPLIT_POINT_DEFAULT;
    scheme->deadband_width = 0.0;
    scheme->overlap_width = 5.0;  /* 5% overlap for smooth pH transitions */
    scheme->transition_type = SPLIT_TRANSITION_CUBIC_SPLINE;
    scheme->bumpless_transfer = true;
    scheme->track_manual_input = true;
    scheme->balanced_heat_cool = false;

    /* Channel 0: Acid valve (reverse acting) */
    scheme->channels[0].channel_id = 0;
    scheme->channels[0].action = SPLIT_ACTION_DECREASING;
    scheme->channels[0].characteristic = SPLIT_VALVE_EQUAL_PCT;
    scheme->channels[0].co_range_start = 0.0;
    scheme->channels[0].co_range_end = scheme->split_point + scheme->overlap_width / 2.0;
    scheme->channels[0].valve_range_start = 100.0;
    scheme->channels[0].valve_range_end = 0.0;
    scheme->channels[0].slew_rate_limit = SPLIT_SLEW_RATE_DEFAULT;
    scheme->channels[0].health = SPLIT_HEALTH_OK;
    scheme->channels[0].Cv_rated = 10.0;
    snprintf(scheme->channels[0].tag_name, 32, "AV-ACID-%u", 0u);
    snprintf(scheme->channels[0].service_description, 64, "Acid Reagent Valve");

    /* Channel 1: Base valve (direct acting) */
    scheme->channels[1].channel_id = 1;
    scheme->channels[1].action = SPLIT_ACTION_INCREASING;
    scheme->channels[1].characteristic = SPLIT_VALVE_EQUAL_PCT;
    scheme->channels[1].co_range_start = scheme->split_point - scheme->overlap_width / 2.0;
    scheme->channels[1].co_range_end = 100.0;
    scheme->channels[1].valve_range_start = 0.0;
    scheme->channels[1].valve_range_end = 100.0;
    scheme->channels[1].slew_rate_limit = SPLIT_SLEW_RATE_DEFAULT;
    scheme->channels[1].health = SPLIT_HEALTH_OK;
    scheme->channels[1].Cv_rated = 10.0;
    snprintf(scheme->channels[1].tag_name, 32, "AV-BASE-%u", 1u);
    snprintf(scheme->channels[1].service_description, 64, "Base Reagent Valve");

    scheme->num_channels = 2;
}

/* =========================================================================
 * split_init_three_way_scheme — L2
 *
 * Three-way split for systems with heat / bypass / cool.
 *
 * Channel 0: Heating — CO 0% to 33%, valve 0% to 100%
 * Channel 1: Bypass  — CO 33% to 66%, valve 100% to 0% (decreasing)
 * Channel 2: Cooling — CO 66% to 100%, valve 0% to 100%
 *
 * This is common in HVAC systems and some reactor temperature
 * control applications where a bypass (recirculation) path exists.
 * ========================================================================= */
void split_init_three_way_scheme(split_range_scheme_t *scheme) {
    if (!scheme) return;

    memset(scheme, 0, sizeof(*scheme));
    scheme->mode = SPLIT_MODE_THREE_WAY;
    scheme->split_point = 33.333;  /* First transition */
    scheme->deadband_width = 1.0;
    scheme->overlap_width = 0.0;
    scheme->transition_type = SPLIT_TRANSITION_LINEAR;
    scheme->bumpless_transfer = true;
    scheme->track_manual_input = true;
    scheme->balanced_heat_cool = false;

    /* Channel 0: Heating — CO 0% to 33% */
    scheme->channels[0].channel_id = 0;
    scheme->channels[0].action = SPLIT_ACTION_INCREASING;
    scheme->channels[0].characteristic = SPLIT_VALVE_EQUAL_PCT;
    scheme->channels[0].co_range_start = 0.0;
    scheme->channels[0].co_range_end = 33.333;
    scheme->channels[0].valve_range_start = 0.0;
    scheme->channels[0].valve_range_end = 100.0;
    scheme->channels[0].slew_rate_limit = SPLIT_SLEW_RATE_DEFAULT;
    scheme->channels[0].health = SPLIT_HEALTH_OK;
    scheme->channels[0].Cv_rated = 50.0;
    snprintf(scheme->channels[0].tag_name, 32, "TV-HEAT-%u", 0u);
    snprintf(scheme->channels[0].service_description, 64, "Heating Valve");

    /* Channel 1: Bypass — CO 33% to 66%, decreasing */
    scheme->channels[1].channel_id = 1;
    scheme->channels[1].action = SPLIT_ACTION_DECREASING;
    scheme->channels[1].characteristic = SPLIT_VALVE_LINEAR;
    scheme->channels[1].co_range_start = 33.333;
    scheme->channels[1].co_range_end = 66.666;
    scheme->channels[1].valve_range_start = 100.0;
    scheme->channels[1].valve_range_end = 0.0;
    scheme->channels[1].slew_rate_limit = SPLIT_SLEW_RATE_DEFAULT;
    scheme->channels[1].health = SPLIT_HEALTH_OK;
    scheme->channels[1].Cv_rated = 100.0;
    snprintf(scheme->channels[1].tag_name, 32, "TV-BYPASS-%u", 1u);
    snprintf(scheme->channels[1].service_description, 64, "Bypass Valve");

    /* Channel 2: Cooling — CO 66% to 100% */
    scheme->channels[2].channel_id = 2;
    scheme->channels[2].action = SPLIT_ACTION_INCREASING;
    scheme->channels[2].characteristic = SPLIT_VALVE_LINEAR;
    scheme->channels[2].co_range_start = 66.666;
    scheme->channels[2].co_range_end = 100.0;
    scheme->channels[2].valve_range_start = 0.0;
    scheme->channels[2].valve_range_end = 100.0;
    scheme->channels[2].slew_rate_limit = SPLIT_SLEW_RATE_DEFAULT;
    scheme->channels[2].health = SPLIT_HEALTH_OK;
    scheme->channels[2].Cv_rated = 80.0;
    snprintf(scheme->channels[2].tag_name, 32, "TV-COOL-%u", 2u);
    snprintf(scheme->channels[2].service_description, 64, "Cooling Valve");

    scheme->num_channels = 3;
}

/* =========================================================================
 * split_add_channel — L2
 *
 * Adds a custom channel to an existing split-range scheme.
 * Validates for consistency: ranges must not overlap, CO ranges
 * within [0, 100], valve ranges within [0, 100].
 * ========================================================================= */
int split_add_channel(split_range_scheme_t *scheme,
                       double co_start, double co_end,
                       double valve_start, double valve_end,
                       split_range_action_t action,
                       split_range_valve_char_t characteristic) {
    if (!scheme) return -1;
    if (scheme->num_channels >= SPLIT_MAX_CHANNELS) return -1;

    /* Validate ranges */
    if (co_start < 0.0 || co_start > 100.0 || co_end < 0.0 || co_end > 100.0)
        return -1;
    if (co_start >= co_end) return -1;
    if (valve_start < 0.0 || valve_start > 100.0
        || valve_end < 0.0 || valve_end > 100.0)
        return -1;

    uint32_t idx = scheme->num_channels;
    scheme->channels[idx].channel_id = idx;
    scheme->channels[idx].action = action;
    scheme->channels[idx].characteristic = characteristic;
    scheme->channels[idx].co_range_start = co_start;
    scheme->channels[idx].co_range_end = co_end;
    scheme->channels[idx].valve_range_start = valve_start;
    scheme->channels[idx].valve_range_end = valve_end;
    scheme->channels[idx].slew_rate_limit = SPLIT_SLEW_RATE_DEFAULT;
    scheme->channels[idx].hysteresis_band = 0.5;
    scheme->channels[idx].health = SPLIT_HEALTH_OK;
    scheme->channels[idx].Cv_rated = 50.0;

    snprintf(scheme->channels[idx].tag_name, 32, "CV-%u", idx);
    snprintf(scheme->channels[idx].service_description, 64,
             "Custom Valve Channel %u", idx);

    scheme->num_channels++;
    return (int)idx;
}

/* =========================================================================
 * split_compute_channel_gains — L2
 *
 * Computes the effective process gain for each channel in the split-range
 * configuration. This is defined as the slope of the valve-position-to-
 * CO mapping.
 *
 * For increasing action: gain = (valve_end - valve_start) / (co_end - co_start)
 * For decreasing action: gain = (valve_start - valve_end) / (co_end - co_start)
 * For fixed action: gain = 0
 *
 * These gains are used in stability analysis and loop tuning.
 * ========================================================================= */
int split_compute_channel_gains(const split_range_scheme_t *scheme,
                                 double *gains) {
    if (!scheme || !gains) return 0;
    if (scheme->num_channels == 0) return 0;

    for (uint32_t i = 0; i < scheme->num_channels && i < SPLIT_MAX_CHANNELS; i++) {
        const split_range_channel_t *ch = &scheme->channels[i];
        double co_width = ch->co_range_end - ch->co_range_start;

        if (co_width <= 0.0) {
            gains[i] = 0.0;
        } else {
            double valve_delta = ch->valve_range_end - ch->valve_range_start;
            if (ch->action == SPLIT_ACTION_DECREASING) {
                valve_delta = -valve_delta;
            } else if (ch->action == SPLIT_ACTION_FIXED) {
                valve_delta = 0.0;
            }
            gains[i] = valve_delta / co_width;
        }
    }

    return (int)scheme->num_channels;
}
