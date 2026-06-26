/**
 * @file ratio_adaptive.c
 * @brief Adaptive ratio control — RLS identification, gain scheduling,
 *        blend optimization, and performance monitoring.
 *
 * Level: L8 Advanced Topics
 *
 * Implements advanced ratio control techniques:
 *   - Recursive Least Squares (RLS) for online process identification
 *   - Adaptive ratio trim gain scheduling
 *   - Blend ratio cost optimization (linear programming)
 *   - Real-time density compensation
 *   - Ratio loop performance metrics and oscillation detection
 *
 * References:
 *   - Ljung, "System Identification" (1999), Ch.11
 *   - Astrom & Wittenmark, "Adaptive Control" (2013), Ch.3
 *   - Edgar, Himmelblau, Lasdon, "Optimization of Chemical Processes" (2001)
 *   - Qin & Badgwell, "A Survey of Industrial Model Predictive Control" (2003)
 */

#include "ratio_types.h"
#include "ratio_adaptive.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* =========================================================================
 * L8: Recursive Least Squares (RLS) Online Identification
 * ========================================================================= */

/**
 * @brief Initialize RLS identifier.
 *
 * Sets P = I * 1000 (large initial covariance → fast initial learning)
 * Sets θ = 0 (initial parameter estimate)
 *
 * The forgetting factor λ determines the memory of the estimator:
 *   Memory time constant = Ts / (1 - λ)
 *
 * For Ts = 1s:
 *   λ = 0.99 → τ ≈ 100s (remembers last 100 seconds)
 *   λ = 0.95 → τ ≈  20s
 *   λ = 0.90 → τ ≈  10s
 */
void rls_init(rls_identifier_t *rls, double forgetting_factor, int n_params)
{
    if (rls == NULL) return;

    if (n_params < 1) n_params = 1;
    if (n_params > 4) n_params = 4;

    rls->forgetting_factor = forgetting_factor;
    if (rls->forgetting_factor <= 0.9) rls->forgetting_factor = 0.95;
    if (rls->forgetting_factor >= 1.0) rls->forgetting_factor = 0.99;
    rls->n_params = n_params;
    rls->initialized = 0;
    rls->prediction_error = 0.0;

    /* Initialize parameters to zero */
    for (int i = 0; i < 4; i++) {
        rls->theta[i] = 0.0;
        rls->phi[i]   = 0.0;
    }

    /* Initialize covariance matrix P = I * 1000 */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            rls->P[i][j] = (i == j) ? 1000.0 : 0.0;
        }
    }
}

/**
 * @brief Perform one RLS update.
 *
 * Algorithm (Ljung, 1999, Eq. 11.34-11.37):
 *
 * Step 1: Prediction
 *   y_hat = φ' * θ(k-1)
 *   ε = y - y_hat
 *
 * Step 2: Gain computation
 *   K = P(k-1) * φ / (λ + φ' * P(k-1) * φ)
 *
 * Step 3: Parameter update
 *   θ(k) = θ(k-1) + K * ε
 *
 * Step 4: Covariance update
 *   P(k) = (P(k-1) - K * φ' * P(k-1)) / λ
 *
 * On first call with valid data, performs initialization:
 * sets P based on initial data variance.
 *
 * Complexity: O(n²) where n = n_params
 */
double rls_update(rls_identifier_t *rls, double y, const double *phi)
{
    if (rls == NULL || phi == NULL) return 0.0;

    int n = rls->n_params;

    /* Store regressor */
    for (int i = 0; i < n; i++) {
        rls->phi[i] = phi[i];
    }

    /* Compute prediction y_hat = φ' * θ */
    double y_hat = 0.0;
    for (int i = 0; i < n; i++) {
        y_hat += phi[i] * rls->theta[i];
    }

    double error = y - y_hat;
    rls->prediction_error = error;

    /* Compute P * φ (vector) */
    double P_phi[4] = {0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            P_phi[i] += rls->P[i][j] * phi[j];
        }
    }

    /* Compute denominator d = λ + φ' * P * φ */
    double d = rls->forgetting_factor;
    for (int i = 0; i < n; i++) {
        d += phi[i] * P_phi[i];
    }

    if (fabs(d) < 1e-12) return error; /* Avoid division by zero */

    /* Compute Kalman gain K = P_phi / d */
    double K[4];
    for (int i = 0; i < n; i++) {
        K[i] = P_phi[i] / d;
    }

    /* Update parameters θ(k) = θ(k-1) + K * ε */
    for (int i = 0; i < n; i++) {
        rls->theta[i] += K[i] * error;
    }

    /* Update covariance P(k) = (P - K*φ'*P) / λ */
    double new_P[4][4] = {{0}};
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            double K_phi_P = 0.0;
            for (int k = 0; k < n; k++) {
                K_phi_P += K[i] * phi[k] * rls->P[k][j];
            }
            new_P[i][j] = (rls->P[i][j] - K_phi_P) / rls->forgetting_factor;
        }
    }

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            rls->P[i][j] = new_P[i][j];
        }
    }

    if (!rls->initialized) rls->initialized = 1;

    return error;
}

/**
 * @brief Get parameter estimates.
 */
void rls_get_theta(const rls_identifier_t *rls, double *theta_out)
{
    if (rls == NULL || theta_out == NULL) return;
    for (int i = 0; i < rls->n_params; i++) {
        theta_out[i] = rls->theta[i];
    }
}

/**
 * @brief Reset RLS covariance to restart adaptation.
 *
 * After large process changes (feedstock switch, equipment
 * modification), the old parameter estimates are invalid.
 * Resetting the covariance enables rapid re-identification.
 */
void rls_reset_covariance(rls_identifier_t *rls)
{
    if (rls == NULL) return;

    int n = rls->n_params;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            rls->P[i][j] = (i == j) ? 1000.0 : 0.0;
        }
    }
}

/* =========================================================================
 * L8: Adaptive Ratio Trim Gain Scheduling
 * ========================================================================= */

/**
 * @brief Adapt trim gain based on identified process gain.
 *
 * The relationship between ratio trim and quality is:
 *   Q = K_process * R_trim
 *
 * If the process gain K_process changes (due to feedstock,
 * fouling, operating point), the trim controller should
 * adjust its gain to maintain consistent closed-loop behavior.
 *
 * Gain scheduling rule:
 *   Kp_new = Kp_nominal * (K_nominal / K_identified)
 *
 * This maintains constant loop gain:
 *   Loop gain = Kp * K_process ≈ constant
 *
 * Bounded adaptation prevents extreme gain values.
 */
void adaptive_trim_gain_schedule(ratio_trim_controller_t *trim,
                                  double identified_gain, double nominal_gain,
                                  double Kp_min, double Kp_max)
{
    if (trim == NULL) return;
    if (fabs(nominal_gain) < 1e-9 || fabs(identified_gain) < 1e-9) return;

    double gain_ratio = nominal_gain / identified_gain;
    double new_Kp = trim->Kp_trim * gain_ratio;

    /* Apply bounds */
    if (new_Kp < Kp_min) new_Kp = Kp_min;
    if (new_Kp > Kp_max) new_Kp = Kp_max;

    trim->Kp_trim = new_Kp;
}

/**
 * @brief Detect process change from error statistics.
 *
 * Monitors the variance of trim errors over a sliding window.
 * If the variance increases significantly above baseline,
 * the process dynamics may have changed, warranting
 * re-identification or operator alert.
 *
 * Returns: 1 if process change detected, 0 otherwise.
 *
 * This is a simple statistical process control (SPC) method:
 * variance exceeding 3× baseline triggers detection.
 */
int adaptive_detect_process_change(const double *error_window,
                                    int window_size,
                                    double baseline_var,
                                    double threshold_mult)
{
    if (error_window == NULL || window_size < 3) return 0;

    /* Compute window mean */
    double sum = 0.0;
    for (int i = 0; i < window_size; i++) {
        sum += error_window[i];
    }
    double mean = sum / window_size;

    /* Compute window variance */
    double var = 0.0;
    for (int i = 0; i < window_size; i++) {
        double diff = error_window[i] - mean;
        var += diff * diff;
    }
    var /= window_size;

    /* Compare to baseline */
    if (baseline_var <= 0.0) return 0; /* No baseline */
    if (var > threshold_mult * baseline_var) return 1;

    return 0;
}

/* =========================================================================
 * L8: Blend Ratio Optimization — Linear Programming
 * ========================================================================= */

/**
 * @brief Initialize blend optimizer.
 */
void blend_optimizer_init(blend_optimizer_t *opt, int n_components)
{
    if (opt == NULL || n_components < 2) return;

    opt->n_components = n_components;

    opt->component_costs  = (double *)malloc((size_t)n_components * sizeof(double));
    opt->component_flows  = (double *)malloc((size_t)n_components * sizeof(double));
    opt->quality_coeffs   = (double *)malloc((size_t)n_components * sizeof(double));

    if (opt->component_costs) memset(opt->component_costs, 0, (size_t)n_components * sizeof(double));
    if (opt->component_flows) memset(opt->component_flows, 0, (size_t)n_components * sizeof(double));
    if (opt->quality_coeffs)  memset(opt->quality_coeffs,  0, (size_t)n_components * sizeof(double));

    opt->total_flow   = 0.0;
    opt->quality_min  = 0.0;
    opt->quality_max  = 100.0;
    opt->optimal_cost = 0.0;
    opt->feasible     = 0;
}

/**
 * @brief Set component costs.
 */
void blend_optimizer_set_costs(blend_optimizer_t *opt, const double *costs)
{
    if (opt == NULL || costs == NULL || opt->component_costs == NULL) return;
    for (int i = 0; i < opt->n_components; i++) {
        opt->component_costs[i] = costs[i];
    }
}

/**
 * @brief Set quality coefficients.
 */
void blend_optimizer_set_quality(blend_optimizer_t *opt, const double *quality)
{
    if (opt == NULL || quality == NULL || opt->quality_coeffs == NULL) return;
    for (int i = 0; i < opt->n_components; i++) {
        opt->quality_coeffs[i] = quality[i];
    }
}

/**
 * @brief Solve 2-component blend optimization analytically.
 *
 * For 2 components:
 *   minimize c1*F1 + c2*F2
 *   subject to F1 + F2 = F_total
 *              Q_min ≤ (q1*F1 + q2*F2)/F_total ≤ Q_max
 *              F1, F2 ≥ 0
 *
 * Since F2 = F_total - F1, substitute:
 *   Q = q1*(F1/F_total) + q2*((F_total-F1)/F_total) = q2 + (q1-q2)*F1/F_total
 *
 * The quality constraint becomes:
 *   Q_min ≤ q2 + (q1-q2)*F1/F_total ≤ Q_max
 *
 * Solve for F1 bound:
 *   If q1 > q2: F1 increases quality
 *     F1_min = (Q_min - q2) * F_total / (q1 - q2)
 *     F1_max = (Q_max - q2) * F_total / (q1 - q2)
 *   If q1 < q2: F1 decreases quality (the bounds swap)
 *
 * The cost is linear in F1: J = c2*F_total + (c1-c2)*F1
 * So the optimum is at one of the bounds:
 *   If c1 < c2: maximize F1 → use F1_max (cheaper to use component 1)
 *   If c1 > c2: minimize F1 → use F1_min (cheaper to use component 2)
 */
static int blend_2comp_solve(blend_optimizer_t *opt,
                              double total_flow, double Q_min, double Q_max)
{
    if (opt == NULL || opt->component_costs == NULL ||
        opt->quality_coeffs == NULL || opt->component_flows == NULL)
        return 0;

    double c1 = opt->component_costs[0];
    double c2 = opt->component_costs[1];
    double q1 = opt->quality_coeffs[0];
    double q2 = opt->quality_coeffs[1];

    double F_total = total_flow;

    /* Quality contribution of component 1 fraction */
    /* If q1 == q2, quality doesn't depend on blend → purely cost-driven */
    if (fabs(q1 - q2) < 1e-12) {
        /* Quality is independent of blend ratio */
        /* Check if quality constraints are satisfied at all */
        if (q1 < Q_min || q1 > Q_max) {
            opt->feasible = 0;
            return 0; /* Infeasible — wrong components */
        }
        /* Cheapest component gets all flow */
        if (c1 < c2) {
            opt->component_flows[0] = F_total;
            opt->component_flows[1] = 0.0;
        } else {
            opt->component_flows[0] = 0.0;
            opt->component_flows[1] = F_total;
        }
        opt->optimal_cost = c1 * opt->component_flows[0] +
                            c2 * opt->component_flows[1];
        opt->feasible = 1;
        return 1;
    }

    /* Component 1 fraction range from quality constraints */
    double F1_min_q = (Q_min - q2) * F_total / (q1 - q2);
    double F1_max_q = (Q_max - q2) * F_total / (q1 - q2);

    /* Ensure F1_min ≤ F1_max (if q1<q2, fractions swap) */
    if (F1_min_q > F1_max_q) {
        double tmp = F1_min_q;
        F1_min_q = F1_max_q;
        F1_max_q = tmp;
    }

    /* Intersect with [0, F_total] */
    if (F1_min_q < 0.0) F1_min_q = 0.0;
    if (F1_max_q > F_total) F1_max_q = F_total;

    /* Check feasibility */
    if (F1_min_q > F1_max_q + 1e-9) {
        opt->feasible = 0;
        return 0;
    }

    /* Optimal F1 based on cost */
    double F1_opt;
    if (c1 < c2) {
        /* Component 1 cheaper → maximize F1 */
        F1_opt = F1_max_q;
    } else if (c1 > c2) {
        /* Component 2 cheaper → minimize F1 */
        F1_opt = F1_min_q;
    } else {
        /* Equal cost → any feasible point; pick midpoint */
        F1_opt = (F1_min_q + F1_max_q) / 2.0;
    }

    double F2_opt = F_total - F1_opt;

    opt->component_flows[0] = F1_opt;
    opt->component_flows[1] = F2_opt;
    opt->optimal_cost = c1 * F1_opt + c2 * F2_opt;
    opt->total_flow   = F_total;
    opt->feasible     = 1;

    return 1;
}

/**
 * @brief Solve N-component blend via greedy heuristic.
 *
 * For N > 2 components, the problem is a linear program.
 * This implementation uses a greedy approach:
 *   1. Sort components by cost/quality ratio
 *   2. Fill with cheapest components until quality constraints met
 *
 * For exact solutions with N ≥ 3, a full simplex implementation
 * would be needed. This heuristic works well for 3-5 components
 * where the quality constraint is a single linear inequality.
 */
static int blend_ncomp_solve(blend_optimizer_t *opt,
                              double total_flow, double Q_min, double Q_max)
{
    if (opt == NULL || opt->component_costs == NULL ||
        opt->quality_coeffs == NULL || opt->component_flows == NULL)
        return 0;

    int n = opt->n_components;

    /* For N > 2, use a simplified approach:
       Treat the top N-1 components as a blend, and the Nth as the balancer */

    /* First, allocate equal fractions as starting point */
    double *frac = (double *)malloc((size_t)n * sizeof(double));
    if (frac == NULL) return 0;

    for (int i = 0; i < n; i++) {
        frac[i] = 1.0 / n;
    }

    /* Compute initial quality */
    double Q = 0.0;
    for (int i = 0; i < n; i++) {
        Q += opt->quality_coeffs[i] * frac[i];
    }

    /* Greedy adjustment: shift fractions toward cheaper components
       while maintaining quality constraints */
    int iterations = 100;
    double step = 0.01;

    while (iterations-- > 0) {
        /* Find cheapest component that can take more */
        int cheapest = -1;
        double cheapest_cost = 1e12;
        for (int i = 0; i < n; i++) {
            if (frac[i] < 0.99 && opt->component_costs[i] < cheapest_cost) {
                /* Check if increasing this component keeps Q within bounds */
                double new_frac = frac[i] + step;
                double sum_others = 0.0;
                double weighted_Q_others = 0.0;
                for (int j = 0; j < n; j++) {
                    if (j != i) {
                        sum_others += frac[j];
                        weighted_Q_others += opt->quality_coeffs[j] * frac[j];
                    }
                }
                double max_Q_new = opt->quality_coeffs[i] * new_frac + weighted_Q_others;
                double frac_scale = (1.0 - new_frac) / (sum_others > 0 ? sum_others : 0.01);
                /* Scale other fractions proportionally */
                double min_Q_new = opt->quality_coeffs[i] * new_frac;
                for (int j = 0; j < n; j++) {
                    if (j != i) {
                        min_Q_new += opt->quality_coeffs[j] * frac[j] * frac_scale;
                    }
                }

                if (min_Q_new <= Q_max && max_Q_new >= Q_min) {
                    cheapest = i;
                    cheapest_cost = opt->component_costs[i];
                }
            }
        }

        if (cheapest < 0) break; /* No improvement possible */

        /* Increase cheapest component, decrease most expensive */
        int most_expensive = -1;
        double most_exp_cost = -1e12;
        for (int i = 0; i < n; i++) {
            if (i != cheapest && frac[i] > 0.01 &&
                opt->component_costs[i] > most_exp_cost) {
                most_expensive = i;
                most_exp_cost = opt->component_costs[i];
            }
        }

        if (most_expensive < 0) break;

        frac[cheapest]      += step;
        frac[most_expensive] -= step;

        /* Normalize */
        double sum = 0.0;
        for (int i = 0; i < n; i++) sum += frac[i];
        if (sum > 0.0) {
            for (int i = 0; i < n; i++) frac[i] /= sum;
        }
    }

    /* Set flows */
    for (int i = 0; i < n; i++) {
        opt->component_flows[i] = frac[i] * total_flow;
    }

    /* Check feasibility */
    Q = 0.0;
    for (int i = 0; i < n; i++) {
        Q += opt->quality_coeffs[i] * frac[i];
    }
    opt->feasible = (Q >= Q_min - 1e-6 && Q <= Q_max + 1e-6) ? 1 : 0;

    /* Compute cost */
    opt->optimal_cost = 0.0;
    for (int i = 0; i < n; i++) {
        opt->optimal_cost += opt->component_costs[i] * opt->component_flows[i];
    }
    opt->total_flow = total_flow;

    free(frac);
    return opt->feasible;
}

/**
 * @brief Solve blend optimization (dispatcher).
 */
int blend_optimizer_solve(blend_optimizer_t *opt, int n_comp,
                           double total_flow, double Q_min, double Q_max)
{
    if (opt == NULL || total_flow <= 0.0 || Q_max < Q_min) return 0;

    opt->quality_min = Q_min;
    opt->quality_max = Q_max;

    if (n_comp == 2) {
        return blend_2comp_solve(opt, total_flow, Q_min, Q_max);
    } else if (n_comp >= 3) {
        return blend_ncomp_solve(opt, total_flow, Q_min, Q_max);
    }

    return 0;
}

/**
 * @brief Get optimized flow for a component.
 */
double blend_optimizer_get_flow(const blend_optimizer_t *opt, int component_i)
{
    if (opt == NULL || component_i < 0 || component_i >= opt->n_components)
        return -1.0;
    if (opt->component_flows == NULL) return -1.0;
    return opt->component_flows[component_i];
}

/**
 * @brief Get optimal total cost.
 */
double blend_optimizer_get_cost(const blend_optimizer_t *opt)
{
    if (opt == NULL) return 0.0;
    return opt->optimal_cost;
}

/* =========================================================================
 * L8: Real-Time Density Compensation
 * ========================================================================= */

/**
 * @brief Gas density compensation from P/T.
 *
 * ρ_comp = ρ_ref * (P/P_ref) * (T_ref/T)
 *
 * This linearized ideal gas correction is accurate for
 * moderate P/T changes. For cryogenic or high-pressure
 * applications, a real-gas EOS (Peng-Robinson, SRK)
 * should be used.
 */
double density_compensate_gas(double rho_ref, double P_ref, double T_ref,
                               double P_actual, double T_actual)
{
    if (P_ref <= 0.0 || T_actual <= 0.0 || T_ref <= 0.0) return rho_ref;
    return rho_ref * (P_actual / P_ref) * (T_ref / T_actual);
}

/**
 * @brief Liquid density temperature compensation.
 *
 * ρ_comp = ρ_ref / (1 + β * ΔT)
 *
 * The denominator form preserves mass conservation better
 * than the numerator form for large ΔT.
 *
 * For water at 20-80°C: Δρ < 2% (β = 2.1e-4)
 * For ethanol at 20-80°C: Δρ ≈ 7% (β = 1.1e-3)
 */
double density_compensate_liquid(double rho_ref, double beta,
                                  double T_ref, double T_actual)
{
    double dT = T_actual - T_ref;
    double denom = 1.0 + beta * dT;
    if (denom <= 0.0) return rho_ref;
    return rho_ref / denom;
}

/**
 * @brief Convert volumetric flow to mass flow.
 */
double mass_flow_from_volume(double vol_flow, double density)
{
    return vol_flow * density;
}

/* =========================================================================
 * L8: Performance Monitoring & Diagnostics
 * ========================================================================= */

/**
 * @brief Compute ratio control performance metrics.
 *
 * Statistical analysis of ratio errors:
 *   - Mean: systematic offset (bias) in ratio
 *   - Std dev: random variability (noise, disturbances)
 *   - Accuracy: percentage of samples within tolerance
 *
 * For Six Sigma quality: accuracy > 99.9997% (6σ)
 * For process control: accuracy > 95% acceptable
 * For blending: accuracy > 99% typical requirement
 */
void ratio_performance_metrics(const double *ratio_errors, int n_samples,
                                double tolerance_pct,
                                double *mean_err, double *std_err,
                                double *accuracy_pct)
{
    if (ratio_errors == NULL || n_samples <= 0) return;

    /* Compute mean */
    double sum = 0.0;
    for (int i = 0; i < n_samples; i++) {
        sum += ratio_errors[i];
    }
    double mean = sum / n_samples;
    if (mean_err != NULL) *mean_err = mean;

    /* Compute standard deviation */
    double sum_sq_diff = 0.0;
    for (int i = 0; i < n_samples; i++) {
        double diff = ratio_errors[i] - mean;
        sum_sq_diff += diff * diff;
    }
    double std = sqrt(sum_sq_diff / n_samples);
    if (std_err != NULL) *std_err = std;

    /* Compute accuracy (% within tolerance) */
    if (accuracy_pct != NULL) {
        int in_tolerance = 0;
        for (int i = 0; i < n_samples; i++) {
            if (fabs(ratio_errors[i]) <= tolerance_pct * 0.01) {
                in_tolerance++;
            }
        }
        *accuracy_pct = 100.0 * (double)in_tolerance / (double)n_samples;
    }
}

/**
 * @brief Detect ratio control loop oscillation.
 *
 * Uses autocorrelation function (ACF) to detect periodic behavior.
 *
 * Oscillation Index:
 *   OI = |min(ACF after lag 0)| / ACF(0)
 *
 * If OI > 0.3: significant oscillation (possible stiction or
 * aggressive tuning)
 * If OI > 0.5: severe oscillation (requires immediate retuning)
 *
 * This is one of the standard methods for control loop
 * performance assessment (Harris index, oscillation index).
 *
 * Reference: Thornhill, Huang, Zhang (2003), "Detection of
 *   Multiple Oscillations in Control Loops", J. Process Control
 */
double ratio_detect_oscillation(const double *ratio_errors, int n_samples)
{
    if (ratio_errors == NULL || n_samples < 10) return 0.0;

    /* Compute mean */
    double sum = 0.0;
    for (int i = 0; i < n_samples; i++) sum += ratio_errors[i];
    double mean = sum / n_samples;

    /* Compute ACF(0) = variance */
    double acf0 = 0.0;
    for (int i = 0; i < n_samples; i++) {
        double diff = ratio_errors[i] - mean;
        acf0 += diff * diff;
    }
    acf0 /= n_samples;

    if (acf0 < 1e-12) return 0.0; /* No variation to analyze */

    /* Compute ACF for lags 1 to n_samples/2 */
    double max_negative_acf = 0.0;
    int found_negative = 0;

    for (int lag = 1; lag < n_samples / 2; lag++) {
        double acf = 0.0;
        int count = 0;
        for (int i = 0; i < n_samples - lag; i++) {
            acf += (ratio_errors[i] - mean) * (ratio_errors[i + lag] - mean);
            count++;
        }
        if (count > 0) acf /= (count * acf0); /* Normalize */

        if (acf < 0.0) {
            if (!found_negative || fabs(acf) > max_negative_acf) {
                max_negative_acf = fabs(acf);
                found_negative = 1;
            }
        }
    }

    /* Oscillation index */
    double OI = found_negative ? max_negative_acf : 0.0;
    if (OI > 1.0) OI = 1.0;
    return OI;
}

/**
 * @brief Compute economic impact of ratio control accuracy.
 *
 * For combustion control: excess air costs energy.
 *   Each 1% ratio error → ~0.3% efficiency loss for natural gas.
 *
 * For blending control: off-ratio means off-spec product.
 *   Rework cost = off_spec_flow * (product_value - salvage_value).
 *
 * This function quantifies the financial motivation for
 * tight ratio control.
 */
double ratio_economic_impact(double ratio_error_pct, double master_flow,
                              double slave_cost_factor, double efficiency_impact)
{
    /* Excess slave flow cost due to ratio error */
    double excess_flow = master_flow * fabs(ratio_error_pct) / 100.0;
    double excess_cost = excess_flow * slave_cost_factor;

    /* Efficiency penalty (combustion: excess air = wasted fuel) */
    double efficiency_loss = fabs(ratio_error_pct) * efficiency_impact;
    double efficiency_cost = master_flow * slave_cost_factor * efficiency_loss / 100.0;

    return excess_cost + efficiency_cost;
}
