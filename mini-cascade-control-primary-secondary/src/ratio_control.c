/**
 * @file ratio_control.c
 * @brief Ratio Control — Flow Ratio, Blending & Stoichiometric Control
 *
 * Implements industrial ratio control strategies:
 * - Fixed ratio, ratio-with-trim, and scheduled ratio
 * - Ratio computation (linear, blending, stoichiometric)
 * - Ratio setpoint calculation with rate limiting
 * - Cross-ratio control for multiple streams
 * - Blend optimization for minimum cost
 * - Flow characterization (square-root compensation)
 * - Ratio-cascade integration
 *
 * Knowledge Coverage:
 *   L1-L2: Ratio control definition, architectures
 *   L3: Digital ratio computation, flow characterization
 *   L5: Cross-ratio, blend optimization
 *   L6: Combustion ratio, blending ratio, stoichiometric feeding
 *
 * References:
 *   Seborg et al., Process Dynamics and Control (2016), Ch. 16
 *   Liptak, Instrument Engineers' Handbook (2006), Vol. 2, Ch. 8
 *   Shinskey, Process Control Systems (1996), Ch. 6
 *
 * Curriculum: MIT 6.302, Stanford ENGR205, Purdue ME575, RWTH Aachen ICS
 */

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "cascade_types.h"
#include "cascade_pid.h"
#include "ratio_control.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*===========================================================================
 * L1: Ratio Station Initialization
 *===========================================================================*/

void ratio_init(ratio_station_t *ratio, const char *tag)
{
    if (!ratio) return;

    memset(ratio, 0, sizeof(*ratio));

    ratio->architecture = RATIO_ARCH_FIXED;
    ratio->calc_method = RATIO_CALC_LINEAR;
    ratio->ratio_sp = 1.0;
    ratio->ratio_pv = 1.0;
    ratio->ratio_trim = 0.0;
    ratio->ratio_bias = 0.0;
    ratio->wild_flow_min = 1.0;
    ratio->wild_flow_max = 1000.0;
    ratio->controlled_sp_min = 0.0;
    ratio->controlled_sp_max = 100.0;
    ratio->ratio_active = false;
    ratio->trim_active = false;

    ratio->wild_flow = 0.0;
    ratio->controlled_flow = 0.0;
    ratio->controlled_sp = 0.0;
    ratio->controlled_pv = 0.0;
    ratio->analyzer_signal = 0.0;

    ratio->accumulated_error = 0.0;
    ratio->last_wild_flow = 0.0;
    ratio->last_ratio = 1.0;
    ratio->sample_count = 0;

    ratio->mw_factor_a = 1.0;
    ratio->mw_factor_b = 1.0;
    ratio->density_a = 1.0;
    ratio->density_b = 1.0;

    if (tag) {
        strncpy(ratio->tag, tag, sizeof(ratio->tag) - 1);
        ratio->tag[sizeof(ratio->tag) - 1] = '\0';
    }
}

void ratio_configure_fixed(ratio_station_t *ratio,
                            double ratio_sp, double bias,
                            double wild_flow_min)
{
    if (!ratio) return;
    if (ratio_sp <= 0.0 || wild_flow_min <= 0.0) return;

    ratio->architecture = RATIO_ARCH_FIXED;
    ratio->ratio_sp = ratio_sp;
    ratio->ratio_bias = bias;
    ratio->wild_flow_min = wild_flow_min;
    ratio->trim_active = false;
}

void ratio_configure_trim(ratio_station_t *ratio,
                           double base_ratio_sp,
                           double trim_gain,
                           double trim_integral_time,
                           double analyzer_sp)
{
    if (!ratio) return;
    if (base_ratio_sp <= 0.0 || trim_gain <= 0.0) return;

    ratio->architecture = RATIO_ARCH_TRIM;
    ratio->ratio_sp = base_ratio_sp;
    ratio->trim_active = true;
    ratio->ratio_trim = 0.0;

    /* The trim controller's integral time is stored in the ratio bias
     * field for simplicity. A full implementation would have a separate
     * trim PID structure. */
    ratio->ratio_bias = trim_integral_time;
    ratio->analyzer_signal = analyzer_sp;
    ratio->accumulated_error = trim_gain;  /* Store trim gain here */
}

/*===========================================================================
 * L3: Ratio Computation Algorithms
 *
 * Linear ratio:        R = Q_controlled / Q_wild
 * Blend fraction:      f_A = Q_A / (Q_A + Q_B)
 * Stoichiometric:      R_s = (Q_c * ρ_c / MW_c) / (Q_w * ρ_w / MW_w) * ν
 *
 * Wild flow minimum: Prevents division by zero at low/no flow.
 *   If Q_wild < wild_flow_min → hold last valid ratio.
 *===========================================================================*/

double ratio_compute_linear(ratio_station_t *ratio)
{
    if (!ratio) return 1.0;

    if (ratio->wild_flow < ratio->wild_flow_min) {
        /* Wild flow too low — hold last valid ratio to avoid blow-up */
        return ratio->last_ratio;
    }

    double R = ratio->controlled_flow / ratio->wild_flow;
    ratio->ratio_pv = R;
    ratio->last_ratio = R;
    ratio->sample_count++;

    return R;
}

double ratio_compute_blend(blend_station_t *blend, double *fractions)
{
    if (!blend || !fractions) return -1.0;
    if (blend->num_streams < 2) return -1.0;

    double total = 0.0;
    for (uint32_t i = 0; i < blend->num_streams; i++) {
        total += blend->streams[i].controlled_flow;
    }

    if (total < 1e-12) {
        /* No flow: equal fractions */
        for (uint32_t i = 0; i < blend->num_streams; i++) {
            fractions[i] = 1.0 / (double)blend->num_streams;
        }
        return 0.0;
    }

    for (uint32_t i = 0; i < blend->num_streams; i++) {
        fractions[i] = blend->streams[i].controlled_flow / total;
    }

    return total;
}

double ratio_compute_mole(ratio_station_t *ratio, double stoich_coeff)
{
    if (!ratio) return 1.0;
    if (ratio->wild_flow < ratio->wild_flow_min) {
        return ratio->last_ratio;
    }

    /* Molar flow rates: n_i = Q_i * ρ_i / MW_i */
    double n_controlled = ratio->controlled_flow *
        ratio->density_a / ratio->mw_factor_a;
    double n_wild = ratio->wild_flow *
        ratio->density_b / ratio->mw_factor_b;

    if (n_wild < 1e-12) return ratio->last_ratio;

    double R_mole = (n_controlled / n_wild) * stoich_coeff;
    ratio->ratio_pv = R_mole;
    ratio->last_ratio = R_mole;
    ratio->sample_count++;

    return R_mole;
}

/*===========================================================================
 * L3: Ratio Setpoint Calculation
 *
 * Qc_sp = R_effective * Q_wild + bias
 *
 * R_effective = ratio_sp + ratio_trim  (if trim active)
 *
 * The controlled SP is rate-limited to avoid hydraulic shock
 * in piping systems.
 *===========================================================================*/

double ratio_calculate_setpoint(ratio_station_t *ratio,
                                 double max_rate_change)
{
    if (!ratio) return 0.0;

    /* Compute effective ratio */
    double R_effective = ratio->ratio_sp;
    if (ratio->trim_active) {
        R_effective += ratio->ratio_trim;
        /* R must remain positive */
        if (R_effective < 0.01) R_effective = 0.01;
    }

    /* Primary ratio equation */
    double Qc_sp_desired = R_effective * ratio->wild_flow + ratio->ratio_bias;

    /* Rate limiting */
    double Qc_sp = Qc_sp_desired;
    double prev_sp = ratio->controlled_sp;

    if (max_rate_change > 0.0) {
        double delta = Qc_sp_desired - prev_sp;
        double max_delta = max_rate_change;

        if (fabs(delta) > max_delta) {
            if (delta > 0.0) {
                Qc_sp = prev_sp + max_delta;
            } else {
                Qc_sp = prev_sp - max_delta;
            }
        }
    }

    /* Clamp to valid range */
    if (Qc_sp > ratio->controlled_sp_max) Qc_sp = ratio->controlled_sp_max;
    if (Qc_sp < ratio->controlled_sp_min) Qc_sp = ratio->controlled_sp_min;

    ratio->controlled_sp = Qc_sp;
    ratio->sample_count++;

    return Qc_sp;
}

/*===========================================================================
 * L3: Ratio Trim Update
 *
 * The trim controller adjusts the ratio setpoint based on a slow
 * process analyzer (e.g., gas chromatograph, octane analyzer).
 *
 * PI trim:
 *   error = analyzer_sp - analyzer_pv
 *   ratio_trim += K_trim * (Ts/Ti_trim * error + (error - error_prev))
 *===========================================================================*/

double ratio_trim_update(ratio_station_t *ratio, double Ts)
{
    if (!ratio || !ratio->trim_active) return 0.0;
    if (Ts <= 0.0) return ratio->ratio_trim;

    double K_trim = ratio->accumulated_error;  /* Stored in this field */
    double Ti_trim = ratio->ratio_bias;         /* Stored in this field */

    if (Ti_trim < 1e-12) return ratio->ratio_trim;

    /* The analyzer_sp is stored in analyzer_signal field initially.
     * In a real implementation, analyzer_sp and analyzer_pv would be
     * separate fields. Here we use accumulated_error as K_trim proxy. */

    /* Simple PI trim:
     * trim(k+1) = trim(k) + K_trim * (Ts/Ti) * error */
    double error = ratio->analyzer_signal - ratio->controlled_pv;

    /* Accumulated error serves as integral accumulator */
    ratio->accumulated_error += error * Ts;

    /* Trim = Kp * (error + 1/Ti * ∫error) */
    ratio->ratio_trim = K_trim * (error + ratio->accumulated_error / Ti_trim);

    /* Clamp trim to reasonable range (±50% of base ratio) */
    double max_trim = ratio->ratio_sp * 0.5;
    if (ratio->ratio_trim > max_trim) ratio->ratio_trim = max_trim;
    if (ratio->ratio_trim < -max_trim) ratio->ratio_trim = -max_trim;

    return ratio->ratio_trim;
}

/*===========================================================================
 * L5: Cross-Ratio Control
 *
 * For dual-stream blending where both streams follow ratio setpoints:
 *   Q_A = R_A * Q_wild
 *   Q_B = R_B * (Q_wild + Q_A)     ← B follows total, not just wild
 *
 * This ensures B's ratio is relative to the total combined flow,
 * providing true blend-ratio control rather than independent ratios.
 *
 * Solving:
 *   Q_B = R_B * (Q_wild + R_A * Q_wild) = R_B * Q_wild * (1 + R_A)
 *
 * This gives Q_B based on Q_wild, R_A, and R_B.
 *===========================================================================*/

int ratio_cross_update(ratio_station_t *station_a,
                        ratio_station_t *station_b,
                        double wild_flow)
{
    if (!station_a || !station_b) return -1;
    if (wild_flow < station_a->wild_flow_min) return -1;

    /* Stream A: standard ratio */
    station_a->wild_flow = wild_flow;
    double Qa_sp = station_a->ratio_sp * wild_flow + station_a->ratio_bias;

    if (Qa_sp > station_a->controlled_sp_max) Qa_sp = station_a->controlled_sp_max;
    if (Qa_sp < station_a->controlled_sp_min) Qa_sp = station_a->controlled_sp_min;
    station_a->controlled_sp = Qa_sp;

    /* Stream B: ratio to TOTAL flow (wild + A) */
    double total_flow = wild_flow + Qa_sp;
    station_b->wild_flow = total_flow;
    double Qb_sp = station_b->ratio_sp * total_flow + station_b->ratio_bias;

    if (Qb_sp > station_b->controlled_sp_max) Qb_sp = station_b->controlled_sp_max;
    if (Qb_sp < station_b->controlled_sp_min) Qb_sp = station_b->controlled_sp_min;
    station_b->controlled_sp = Qb_sp;

    /* Check for infeasibility:
     * Total demand > available capacity → infeasible */
    double total_demand = Qa_sp + Qb_sp;
    double total_capacity = station_a->controlled_sp_max +
        station_b->controlled_sp_max;
    if (total_demand > total_capacity) {
        /* Scale back proportionally */
        double scale = total_capacity / total_demand;
        station_a->controlled_sp *= scale;
        station_b->controlled_sp *= scale;
        return -1;  /* Infeasible but scaled back */
    }

    return 0;
}

/*===========================================================================
 * L5: Ratio Performance Metrics
 *
 * Coefficient of Variation:
 *   CV_ratio = σ_ratio / μ_ratio * 100  [%]
 *
 * A CV < 1% indicates excellent ratio control.
 * A CV > 5% indicates poor ratio control needing retune.
 *===========================================================================*/

int ratio_performance_metrics(const ratio_station_t *ratio,
                               double *cv_ratio, double *std_ratio)
{
    if (!ratio || !cv_ratio || !std_ratio) return 0;
    if (ratio->sample_count < 10) {
        *cv_ratio = 0.0;
        *std_ratio = 0.0;
        return 0;
    }

    /* For a running estimate, we use the ratio SP vs PV deviation.
     * In a production system, full statistics would be maintained
     * in circular buffers. */

    double mean_error = ratio->accumulated_error / (double)ratio->sample_count;
    *std_ratio = fabs(mean_error) * 2.0;   /* Conservative 2σ estimate */
    *cv_ratio = (*std_ratio / ratio->ratio_sp) * 100.0;

    return (int)ratio->sample_count;
}

/*===========================================================================
 * L5: Blend Optimization — Minimum Cost Subject to Quality
 *
 * Linear blending model:
 *   Q_total = Σ Q_i
 *   property = Σ (p_i * Q_i) / Q_total     (mass-average property)
 *
 * Minimize: Σ (cost_i * Q_i)
 * Subject to: prop_min ≤ property ≤ prop_max, Q_i ≥ 0, Σ Q_i = Q_total
 *
 * This implements a simple greedy approach: sort by cost-effectiveness
 * and allocate to lower-cost streams first.
 *===========================================================================*/

double ratio_optimize_blend(blend_station_t *blend,
                             const double *costs,
                             const double *property_coeff,
                             double prop_min, double prop_max,
                             double *optimal)
{
    if (!blend || !costs || !property_coeff || !optimal) return -1.0;
    if (blend->num_streams < 2) return -1.0;

    uint32_t n = blend->num_streams;
    double total_flow = blend->total_flow_sp;

    if (total_flow < 1e-12) return -1.0;

    /* Simple approach: compute optimal fractions analytically.
     *
     * For two-stream blending with one property constraint:
     *   f1 + f2 = 1
     *   p1*f1 + p2*f2 = prop_target
     *
     *   f1 = (prop_target - p2) / (p1 - p2)
     *   f2 = 1 - f1
     *
     * For multi-stream, use a cost-weighted allocation heuristic.
     */

    /* Start with equal fractions */
    for (uint32_t i = 0; i < n; i++) {
        optimal[i] = total_flow / (double)n;
    }

    /* Adjust toward minimum-cost blend while meeting property target */
    double prop_target = (prop_min + prop_max) / 2.0;

    /* Compute current blend property with equal fractions */
    double current_prop = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        current_prop += property_coeff[i] * optimal[i];
    }
    current_prop /= total_flow;

    /* Iteratively shift flow from high-cost to low-cost streams
     * while maintaining property target. Simple gradient descent. */
    for (int iter = 0; iter < 20; iter++) {
        double prop_error = current_prop - prop_target;
        if (fabs(prop_error) < 0.001) break;

        /* Find cheapest stream with property above target (increase its flow)
         * and most expensive stream with property below target (decrease) */
        int best_inc = -1;
        double best_cost_inc = 1e30;
        int best_dec = -1;
        double best_cost_dec = -1e30;

        for (uint32_t i = 0; i < n; i++) {
            if (prop_error > 0 && property_coeff[i] < prop_target) {
                if (costs[i] < best_cost_inc && optimal[i] < blend->streams[i].controlled_sp_max) {
                    best_cost_inc = costs[i];
                    best_inc = (int)i;
                }
            }
            if (prop_error > 0 && property_coeff[i] > prop_target) {
                if (costs[i] > best_cost_dec && optimal[i] > blend->streams[i].controlled_sp_min) {
                    best_cost_dec = costs[i];
                    best_dec = (int)i;
                }
            }
            if (prop_error < 0 && property_coeff[i] > prop_target) {
                if (costs[i] < best_cost_inc && optimal[i] < blend->streams[i].controlled_sp_max) {
                    best_cost_inc = costs[i];
                    best_inc = (int)i;
                }
            }
            if (prop_error < 0 && property_coeff[i] < prop_target) {
                if (costs[i] > best_cost_dec && optimal[i] > blend->streams[i].controlled_sp_min) {
                    best_cost_dec = costs[i];
                    best_dec = (int)i;
                }
            }
        }

        if (best_inc < 0 || best_dec < 0) break;

        /* Shift 5% of flow */
        double delta = total_flow * 0.05;
        if (optimal[best_dec] - delta < 0.0) delta = optimal[best_dec];
        if (optimal[best_inc] + delta > blend->streams[best_inc].controlled_sp_max)
            delta = blend->streams[best_inc].controlled_sp_max - optimal[best_inc];

        optimal[best_inc] += delta;
        optimal[best_dec] -= delta;

        /* Recompute property */
        current_prop = 0.0;
        for (uint32_t i = 0; i < n; i++) {
            current_prop += property_coeff[i] * optimal[i];
        }
        current_prop /= total_flow;
    }

    /* Compute total cost */
    double total_cost = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        total_cost += costs[i] * optimal[i];
    }

    return total_cost;
}

/*===========================================================================
 * L3: Flow Characterization — Square-Root Compensation
 *
 * Orifice plates and Venturi meters produce a differential pressure
 * proportional to the square of the flow rate:
 *
 *   ΔP = (Q / K)^2   →   Q = K * sqrt(ΔP)
 *
 * For accurate ratio control across the full flow range, the DP
 * signal must be linearized (square-root extraction).
 *
 * At very low flows (ΔP < cut_off), linear interpolation prevents
 * the infinite gain of sqrt at zero.
 *===========================================================================*/

double ratio_characterize_flow(double raw_measurement,
                                int is_square_root, double K)
{
    if (!is_square_root) {
        /* Linear flowmeter (magnetic, Coriolis, ultrasonic) */
        return K * raw_measurement;
    }

    if (K < 1e-12) return raw_measurement;

    /* Square-root extraction with low-flow cut-off
     * Below cut-off, linear interpolation to avoid infinite gain at zero */
    double cut_off = K * K * 0.01;  /* 1% of full scale */
    if (raw_measurement < 0.0) raw_measurement = 0.0;

    if (raw_measurement < cut_off) {
        /* Linear interpolation to zero */
        return K * raw_measurement / (K * sqrt(cut_off)) * sqrt(cut_off);
    } else {
        return K * sqrt(raw_measurement);
    }
}

/*===========================================================================
 * L5: Ratio + Cascade Integration
 *
 * Ratio control as the primary loop in a cascade:
 *   Primary (ratio): Maintains desired ratio R_sp
 *   Secondary (flow): Maintains computed flow setpoint
 *
 * The ratio controller computes Qc_sp = R_sp * Q_wild and sends
 * it as the remote setpoint to the flow controller.
 *===========================================================================*/

int ratio_cascade_setpoint(ratio_station_t *ratio,
                            cascade_pid_controller_t *flow_controller,
                            double wild_flow)
{
    if (!ratio || !flow_controller) return -1;

    ratio->wild_flow = wild_flow;

    /* Compute controlled flow setpoint */
    double Qc_sp = ratio_calculate_setpoint(ratio, ratio->controlled_sp_max * 0.1);

    /* Set as remote SP to the flow controller */
    /* In a real DCS, this would be written to the secondary controller's
     * remote SP register. Here we simulate it. */
    flow_controller->params.output_max = ratio->controlled_sp_max;
    flow_controller->params.output_min = ratio->controlled_sp_min;

    ratio->controlled_sp = Qc_sp;

    return 0;
}
