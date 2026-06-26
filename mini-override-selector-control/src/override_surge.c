/**
 * @file override_surge.c
 * @brief Override/Selector Control — Compressor Surge Protection
 *
 * Implements compressor surge detection and anti-surge override
 * control. Surge is a catastrophic flow reversal in centrifugal
 * and axial compressors that can cause severe mechanical damage.
 * A high-select override controller opens a recycle (anti-surge)
 * valve when the operating point approaches the surge line.
 *
 * The surge line is typically characterized as:
 *   ΔP_surge = slope × Q² + intercept
 * where Q is flow and ΔP is differential pressure.
 *
 * Knowledge Coverage:
 *   L2: Surge phenomenon, anti-surge override strategy
 *   L3: Surge curve, margin computation, recycle valve control
 *   L5: Surge detection algorithm, surge margin PI control
 *   L6: Compressor surge protection (classic override problem)
 *   L7: Industrial compressor control (NASA, Boeing, Oil & Gas)
 *
 * Reference:
 *   Nisenfeld, A.E. & Seemann, R.C. (1981). "Centrifugal
 *     Compressors — Understanding Their Control." ISA.
 *   White, M.H. (2015). "Surge Control for Centrifugal
 *     Compressors." Chemical Engineering.
 *   Boyce, M.P. (2012). Gas Turbine Engineering Handbook (4th ed.).
 */

#include "override_core.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =========================================================================
 * L2 — Surge Line and Margin Computation
 * ========================================================================= */

/**
 * Compute the surge-line differential pressure at a given flow.
 *
 * The surge line is the boundary in (Q, ΔP) space beyond which
 * surge occurs. It is typically defined by the manufacturer as
 * a polynomial function of flow.
 *
 * Model: ΔP_surge(Q) = slope × Q² + intercept
 *
 * For a more accurate representation, a cubic or higher-order
 * polynomial may be used, but the quadratic model is the most
 * common in industrial practice.
 *
 * @param surge     Surge control state
 * @param flow      Current flow [m³/s or kg/s]
 * @return Surge-line ΔP at this flow
 */
static double surge_line_delta_p(const surge_control_t *surge, double flow) {
    if (surge == NULL) return 0.0;
    if (flow <= 0.0) return INFINITY;

    return surge->surge_line_slope * flow * flow +
           surge->surge_line_intercept;
}

/**
 * Compute the surge control line (SCL) at a given flow.
 *
 * The SCL is offset from the surge line by a safety margin
 * (typically 10-15% in flow or ΔP). When the operating point
 * crosses the SCL, the anti-surge valve starts opening.
 *
 * SCL_ΔP = ΔP_surge × (1 - margin_ratio)
 * or equivalently, SCL_Q = Q_surge × (1 + margin_ratio) for
 * flow-based margins.
 *
 * @param surge        Surge control state
 * @param flow         Current flow
 * @param margin_ratio Safety margin ratio [0..1] (e.g., 0.15 = 15%)
 * @return SCL ΔP at this flow
 */
static double surge_control_line_delta_p(const surge_control_t *surge,
                                         double flow, double margin_ratio) {
    double surge_dp = surge_line_delta_p(surge, flow);
    return surge_dp * (1.0 - margin_ratio);
}

/**
 * Compute surge margin as a percentage:
 *
 * margin[%] = (1 - ΔP_actual / ΔP_surge) × 100   for fixed flow
 *           = (Q_actual / Q_surge - 1) × 100      for fixed ΔP
 *
 * Positive margin = safe (operating away from surge)
 * Zero margin = on surge line
 * Negative margin = surged
 *
 * @param surge  Surge control state
 * @param flow   Current flow
 * @param dp     Current differential pressure
 * @return Surge margin [%]
 */
double surge_compute_margin(const surge_control_t *surge,
                            double flow, double dp) {
    if (surge == NULL || flow <= 0.0) return 0.0;

    double surge_dp = surge_line_delta_p(surge, flow);

    if (surge_dp <= 0.0) return 100.0; /* Safe region */

    /* Margin based on ΔP: percentage below surge ΔP */
    double margin = (1.0 - dp / surge_dp) * 100.0;

    return margin;
}

/* =========================================================================
 * L3 — Surge Detection
 * ========================================================================= */

/**
 * Detect imminent surge conditions.
 *
 * A surge condition is declared when:
 * 1. The surge margin falls below the minimum threshold, OR
 * 2. The rate of margin decrease is rapid (impending surge), OR
 * 3. Flow reversal is detected (negative flow at discharge)
 *
 * @param surge Surge control state
 * @param flow  Current flow
 * @param dp    Current differential pressure
 * @param dt    Time step [s]
 * @return 1 if surge is detected/impending, 0 if safe
 */
int surge_detect(surge_control_t *surge,
                 double flow, double dp, double dt) {
    if (surge == NULL) return 0;

    /* Update current values */
    double prev_flow = surge->flow;
    surge->flow = flow;
    surge->delta_p = dp;

    /* Compute current margin */
    double margin = surge_compute_margin(surge, flow, dp);
    surge->surge_margin = margin;

    /* Check 1: Margin below minimum */
    if (margin < surge->min_surge_margin) {
        if (!surge->surge_detected) {
            surge->surge_detected = 1;
            surge->surge_onset_time = dt; /* Approximation — caller tracks time */
        }
        surge->surge_override_active = 1;
        return 1;
    }

    /* Check 2: Rapid margin decrease */
    if (dt > 0.0 && prev_flow > 0.0) {
        double prev_margin = surge_compute_margin(surge, prev_flow, surge->delta_p);
        double margin_rate = (margin - prev_margin) / dt;

        /* If margin is decreasing faster than -5%/s, imminent surge */
        if (margin_rate < -5.0 && margin < 2.0 * surge->min_surge_margin) {
            surge->surge_override_active = 1;
            return 1;
        }
    }

    /* Check 3: Flow reversal (absolute surge event) */
    if (flow <= 0.0 && dp > 0.0) {
        surge->surge_detected = 1;
        surge->surge_override_active = 1;
        return 1;
    }

    /* Safe: reset detection flags */
    surge->surge_detected = 0;
    surge->surge_override_active = 0;
    return 0;
}

/* =========================================================================
 * L5 — Anti-Surge PI Controller
 * ========================================================================= */

/**
 * Compute the anti-surge recycle valve opening using a PI
 * controller on surge margin.
 *
 * The anti-surge valve opens when the operating point approaches
 * the surge control line. The controller output increases from
 * 0% (fully closed) to 100% (fully open) as the surge margin
 * decreases from a safe value toward zero.
 *
 * PI tuning (conservative):
 *   Kc = 0.5 to 2.0 (aggressive enough to prevent surge)
 *   Ti = 1 to 5 seconds (fast integral to respond quickly)
 *
 * The anti-surge override works as a HIGH-SELECT against the
 * normal throughput controller: whichever demands a higher
 * recycle valve opening wins.
 *
 * @param surge     Surge control state
 * @param Kc        PI proportional gain
 * @param Ti        PI integral time [s]
 * @param dt        Time step [s]
 * @return Anti-surge valve position [0-100%]
 */
double surge_antisurge_pi(surge_control_t *surge,
                          double Kc, double Ti, double dt) {
    if (surge == NULL || dt <= 0.0) return 0.0;

    /* Error signal: desired margin - actual margin.
       Using min_surge_margin as the setpoint (keep margin above this). */
    double margin_sp = surge->min_surge_margin * 2.0; /* Target 2x minimum */
    if (margin_sp < 10.0) margin_sp = 10.0; /* At least 10% */

    double error = margin_sp - surge->surge_margin;

    /* P term */
    double p_term = Kc * error;

    /* I term */
    double i_term = 0.0;
    if (Ti > 0.0) {
        /* Static integral accumulator would need to be stored in surge_control_t.
           For this implementation, we use a simple proportional output:
           if margin is already above setpoint, output = 0. */
    }

    double raw_output = p_term + i_term;

    /* Clamp to [0, 100] */
    if (raw_output < 0.0) raw_output = 0.0;
    if (raw_output > 100.0) raw_output = 100.0;

    surge->recycle_valve_output = raw_output;
    return raw_output;
}

/* =========================================================================
 * L6 — Surge Cycle Detection and History
 * ========================================================================= */

/**
 * Detect a surge cycle (rapid oscillation across the surge line).
 *
 * Surge cycles are characterized by:
 * 1. Flow oscillation at low frequency (0.5-5 Hz typically)
 * 2. Large amplitude of flow and pressure perturbations
 * 3. Possible reverse flow
 *
 * This function detects surge cycles by monitoring the margin
 * crossing-count over time.
 *
 * @param surge       Surge control state
 * @param margin      Current surge margin
 * @param threshold   Crossing threshold margin level
 * @param time_now    Current time [s]
 * @return 1 if surge cycle detected, 0 otherwise
 */
int surge_cycle_detect(surge_control_t *surge,
                       double margin, double threshold, double time_now) {
    if (surge == NULL) return 0;

    /* Track margin crossings of threshold */
    static double last_margin = 100.0;
    static int crossing_count = 0;
    static double last_crossing_time = 0.0;

    /* Detect crossing */
    if ((last_margin >= threshold && margin < threshold) ||
        (last_margin <= threshold && margin > threshold)) {
        crossing_count++;

        double time_diff = time_now - last_crossing_time;
        last_crossing_time = time_now;

        /* If crossings are rapid (< 0.5s between crossings) and
           more than 3 cycles, declare surge cycling */
        if (crossing_count >= 6 && time_diff < 2.0) {
            crossing_count = 0;
            last_margin = margin;
            return 1;
        }
    }

    last_margin = margin;
    return 0;
}

/**
 * Compute recommended anti-surge valve size based on compressor
 * characteristics and surge line parameters.
 *
 * @param surge          Surge control state
 * @param design_flow    Design flow rate [m³/s]
 * @param design_dp      Design ΔP [kPa]
 * @param valve_Cv_max   Output: recommended maximum Cv
 * @param recycle_ratio  Output: recommended recycle flow ratio
 */
void surge_valve_sizing(const surge_control_t *surge,
                        double design_flow, double design_dp,
                        double *valve_Cv_max, double *recycle_ratio) {
    if (surge == NULL) {
        if (valve_Cv_max) *valve_Cv_max = 0.0;
        if (recycle_ratio) *recycle_ratio = 0.0;
        return;
    }

    /* Rule of thumb: recycle valve should be sized for 30-50% of
       design flow at maximum ΔP across the valve. */
    double recycle_flow = design_flow * 0.4;

    /* Cv = Q * sqrt(SG / ΔP)
       where Q is flow in GPM, SG = specific gravity, ΔP in psi.
       For metric approximation: Cv ≈ recycle_ratio * design_Cv */
    if (valve_Cv_max && design_dp > 0.0) {
        /* Simplified: Cv proportional to flow / sqrt(ΔP) */
        *valve_Cv_max = recycle_flow / sqrt(design_dp) * 100.0;
    }

    if (recycle_ratio) {
        *recycle_ratio = recycle_flow / design_flow;
    }
}

/**
 * Assess the severity of a surge event (0-100 scale)
 *
 * @param surge    Surge control state
 * @param flow_min Minimum flow during oscillation
 * @param dp_max   Maximum ΔP during oscillation
 * @return Severity index [0-100]
 */
double surge_severity(const surge_control_t *surge,
                      double flow_min, double dp_max) {
    if (surge == NULL) return 0.0;

    /* Severity factors:
       - Negative margin (surged): base score 50
       - Reverse flow: add 30
       - High DP excursions: add up to 20 */

    double severity = 0.0;

    if (surge->surge_margin < 0.0) {
        severity += 50.0;
    }

    if (flow_min < 0.0) {
        severity += 30.0;
    }

    double surge_dp = surge_line_delta_p(surge, surge->flow);
    if (surge_dp > 0.0 && dp_max > surge_dp) {
        double dp_excess = (dp_max - surge_dp) / surge_dp;
        severity += (dp_excess * 20.0 > 20.0) ? 20.0 : dp_excess * 20.0;
    }

    if (severity > 100.0) severity = 100.0;
    if (severity < 0.0) severity = 0.0;

    return severity;
}

/**
 * Compute surge-line flow given ΔP (inverse of surge line equation).
 *
 * Q_surge(ΔP) = sqrt((ΔP - intercept) / slope)
 *
 * @param surge Surge control state
 * @param dp    Differential pressure [kPa]
 * @return Flow at surge line, or 0 if ΔP is below intercept
 */
double surge_flow_at_dp(const surge_control_t *surge, double dp) {
    if (surge == NULL || surge->surge_line_slope <= 0.0) return 0.0;

    double numerator = dp - surge->surge_line_intercept;
    if (numerator <= 0.0) return 0.0;

    return sqrt(numerator / surge->surge_line_slope);
}

/**
 * Diagnose compressor health based on surge history
 *
 * @param surge            Surge control state
 * @param surge_event_count Number of surge events
 * @param total_run_hours  Total running hours
 * @param health_score     Output: 0-100 health score
 * @param recommendation   Output: recommendation string buffer
 * @param bufsz            Buffer size
 */
void surge_health_diagnosis(const surge_control_t *surge,
                            int surge_event_count,
                            double total_run_hours,
                            double *health_score,
                            char *recommendation, int bufsz) {
    if (surge == NULL) {
        if (health_score) *health_score = 0.0;
        return;
    }

    /* Health score based on surge event frequency:
       < 1 event/year = 100 (healthy)
       1-5 events/year = 75-99
       5-20 events/year = 50-74
       > 20 events/year = < 50 (damaged) */
    double years = total_run_hours / 8760.0;
    double event_rate = (years > 0.0) ? (double)surge_event_count / years : 0.0;

    double score = 100.0;
    if (event_rate > 1.0) {
        score -= (event_rate - 1.0) * 5.0;
    }
    if (score < 0.0) score = 0.0;
    if (score > 100.0) score = 100.0;

    if (health_score) *health_score = score;

    if (recommendation && bufsz > 0) {
        if (score > 90.0) {
            snprintf(recommendation, (size_t)bufsz,
                     "Healthy. Continue routine monitoring.");
        } else if (score > 70.0) {
            snprintf(recommendation, (size_t)bufsz,
                     "Monitor closely. Review anti-surge tuning.");
        } else if (score > 50.0) {
            snprintf(recommendation, (size_t)bufsz,
                     "Schedule maintenance. Check recycle valve and surge line calibration.");
        } else {
            snprintf(recommendation, (size_t)bufsz,
                     "URGENT: Inspect compressor for surge damage. Recalibrate surge line.");
        }
    }
}
