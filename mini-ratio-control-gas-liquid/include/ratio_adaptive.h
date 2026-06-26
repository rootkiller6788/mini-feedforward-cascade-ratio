/**
 * @file ratio_adaptive.h
 * @brief Adaptive ratio control — online identification and optimization.
 *
 * Level: L8 Advanced Topics
 * Reference: Ljung, "System Identification" (1999), Ch.11 Recursive Methods
 *            Astrom & Wittenmark, "Adaptive Control" (2013), Ch.3
 *            Edgar, Himmelblau, Lasdon, "Optimization of Chemical Processes" (2001)
 *
 * Course mapping:
 *   Stanford EE392: Industrial AI — adaptive/learning control
 *   MIT 6.302: Feedback Systems — adaptive feedforward
 *   Berkeley ME233: Advanced Control — system identification
 *   CMU 24-677: Advanced Control — stochastic/adaptive methods
 *
 * This module provides:
 *   - Recursive Least Squares (RLS) for online ratio-to-quality gain identification
 *   - Adaptive ratio trim gain scheduling
 *   - Blend ratio cost optimization via linear programming
 *   - Real-time density compensation with temperature/pressure tracking
 *   - Performance monitoring for ratio control loops
 */

#ifndef RATIO_ADAPTIVE_H
#define RATIO_ADAPTIVE_H

#include "ratio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L8: Recursive Least Squares (RLS) — Online Identification
 *===========================================================================*/

/**
 * @brief Initialize the RLS identifier.
 *
 * Sets up the covariance matrix P as diagonal with large initial values
 * (indicating high uncertainty), zeros the parameter vector θ,
 * and sets the forgetting factor λ.
 *
 * Forgetting factor selection:
 *   λ = 0.99 → slow forgetting, smooth estimates (stable processes)
 *   λ = 0.95 → moderate forgetting (typical process control)
 *   λ = 0.90 → fast forgetting (rapidly changing processes, higher noise)
 *
 * @param rls               RLS identifier state
 * @param forgetting_factor λ (0.9 to 0.999)
 * @param n_params          Number of parameters to identify (1-4)
 *
 * Complexity: O(n²) where n = n_params
 * References: Ljung (1999), Eq. 11.31-11.33
 */
void rls_init(rls_identifier_t *rls, double forgetting_factor, int n_params);

/**
 * @brief Perform one RLS update step.
 *
 * Updates parameter estimates θ based on new measurement:
 *
 *   prediction = φ' * θ(k-1)
 *   error     = y(k) - prediction
 *   K         = P(k-1) * φ / (λ + φ' * P(k-1) * φ)
 *   θ(k)      = θ(k-1) + K * error
 *   P(k)      = (P(k-1) - K * φ' * P(k-1)) / λ
 *
 * @param rls   RLS identifier state
 * @param y     New measurement (scalar output)
 * @param phi   Regressor vector (length = n_params)
 * @return      Prediction error (y - φ'·θ)
 *
 * Complexity: O(n²) where n = n_params
 * References: Ljung (1999), Eq. 11.34-11.37
 */
double rls_update(rls_identifier_t *rls, double y, const double *phi);

/**
 * @brief Get the current parameter estimate.
 *
 * @param rls        RLS identifier state
 * @param theta_out  Output vector (must be at least n_params elements)
 *
 * Complexity: O(n)
 */
void rls_get_theta(const rls_identifier_t *rls, double *theta_out);

/**
 * @brief Reset RLS covariance matrix (re-enables fast adaptation).
 *
 * Resets P to diagonal with large values. Use after large process
 * changes that invalidate the previous estimate.
 *
 * @param rls  RLS identifier state
 *
 * Complexity: O(n²)
 */
void rls_reset_covariance(rls_identifier_t *rls);

/*===========================================================================
 * L8: Adaptive Ratio Trim — Gain Scheduling
 *===========================================================================*/

/**
 * @brief Adapt ratio trim gain based on identified process gain.
 *
 * When the process gain (ratio → quality) changes (e.g., due to
 * feedstock changes, fouling, or operating point shifts), the
 * ratio trim controller should adapt its gain.
 *
 * Simple gain scheduling:
 *   Kp_trim_new = Kp_trim_nominal / identified_gain_ratio
 *
 * where identified_gain_ratio = identified_gain / nominal_gain.
 *
 * A more conservative strategy uses bounded adaptation:
 *   Kp_trim_new = clamp(Kp_nominal / gain_ratio, Kp_min, Kp_max)
 *
 * @param trim                Trim controller to adapt
 * @param identified_gain     Currently identified process gain (from RLS)
 * @param nominal_gain        Nominal (design) process gain
 * @param Kp_min              Minimum allowed Kp
 * @param Kp_max              Maximum allowed Kp
 *
 * Complexity: O(1)
 */
void adaptive_trim_gain_schedule(ratio_trim_controller_t *trim,
                                  double identified_gain, double nominal_gain,
                                  double Kp_min, double Kp_max);

/**
 * @brief Detect if the process has changed enough to warrant re-identification.
 *
 * Monitors the ratio trim error statistics. If the error variance
 * increases significantly (e.g., 3x above baseline), returns 1 to
 * signal that process dynamics may have changed.
 *
 * @param error_window    Array of recent trim errors
 * @param window_size     Number of samples in window
 * @param baseline_var    Baseline error variance (from commissioning)
 * @param threshold_mult  Threshold multiplier (e.g., 3.0)
 * @return                1 if process change detected, 0 otherwise
 *
 * Complexity: O(n) where n = window_size
 */
int adaptive_detect_process_change(const double *error_window,
                                    int window_size,
                                    double baseline_var,
                                    double threshold_mult);

/*===========================================================================
 * L8: Blend Ratio Optimization — Linear Programming
 *===========================================================================*/

/**
 * @brief Initialize the blend optimizer.
 *
 * Allocates internal arrays for component costs, flows, and quality
 * coefficients. Must be called before any optimization.
 *
 * @param opt           Blend optimizer state
 * @param n_components  Number of blend components (≥ 2)
 *
 * Complexity: O(n)
 */
void blend_optimizer_init(blend_optimizer_t *opt, int n_components);

/**
 * @brief Set component cost coefficients for optimization.
 *
 * The objective is to minimize total cost:
 *   J = sum(c_i * F_i)
 *
 * where c_i is the cost per unit of component i
 * and F_i is the flow rate of component i.
 *
 * @param opt      Blend optimizer
 * @param costs    Array of n_components costs (per unit)
 *
 * Complexity: O(n)
 */
void blend_optimizer_set_costs(blend_optimizer_t *opt, const double *costs);

/**
 * @brief Set quality contribution coefficients.
 *
 * Assumes linear mixing rule for quality Q:
 *   Q = sum(q_i * x_i)  where x_i = F_i / F_total
 *
 * where q_i is the quality contribution per unit fraction of component i.
 *
 * @param opt      Blend optimizer
 * @param quality  Array of n_components quality coefficients
 *
 * Complexity: O(n)
 */
void blend_optimizer_set_quality(blend_optimizer_t *opt, const double *quality);

/**
 * @brief Solve the blend ratio optimization problem.
 *
 * Formulation:
 *   minimize    J = sum(c_i * F_i)
 *   subject to  sum(F_i) = total_flow
 *               F_i ≥ 0                               (non-negativity)
 *               Q_min ≤ sum(q_i * F_i)/total_flow ≤ Q_max  (quality)
 *
 * For 2-component blends, solved analytically (no LP solver needed).
 * For 3+ components, uses a simplified simplex-like algorithm.
 *
 * The solution gives the optimal flow for each component that
 * minimizes cost while meeting quality constraints.
 *
 * @param opt        Blend optimizer (updated with optimal flows)
 * @param n_comp     Number of components
 * @param total_flow Total required product flow
 * @param Q_min      Minimum acceptable quality
 * @param Q_max      Maximum acceptable quality
 * @return           1 if feasible solution found, 0 otherwise
 *
 * Complexity: O(n³) worst-case for simplex, O(1) for 2-component analytical
 * References: Edgar, Himmelblau, Lasdon (2001), Ch. 7.3-7.4
 */
int blend_optimizer_solve(blend_optimizer_t *opt, int n_comp,
                           double total_flow, double Q_min, double Q_max);

/**
 * @brief Get the optimized flow for a specific component.
 *
 * @param opt          Blend optimizer (after successful solve)
 * @param component_i  Component index (0-based)
 * @return             Optimized flow, or -1 if component_i invalid
 *
 * Complexity: O(1)
 */
double blend_optimizer_get_flow(const blend_optimizer_t *opt, int component_i);

/**
 * @brief Get the optimal total cost.
 *
 * @param opt  Blend optimizer (after successful solve)
 * @return     Optimal cost per unit time
 *
 * Complexity: O(1)
 */
double blend_optimizer_get_cost(const blend_optimizer_t *opt);

/*===========================================================================
 * L8: Real-Time Density Compensation
 *===========================================================================*/

/**
 * @brief Apply real-time temperature/pressure compensation to gas density.
 *
 * Combines ideal gas law with real-time P/T measurements:
 *   ρ_comp = ρ_ref * (P_actual / P_ref) * (T_ref / T_actual)
 *
 * This is used to correct gas flow measurements from actual to
 * reference conditions for accurate ratio control.
 *
 * @param rho_ref       Reference density (kg/m³) at P_ref, T_ref
 * @param P_ref         Reference pressure (kPa_abs)
 * @param T_ref         Reference temperature (K)
 * @param P_actual      Actual pressure (kPa_abs)
 * @param T_actual      Actual temperature (K)
 * @return              Compensated density (kg/m³)
 *
 * Complexity: O(1)
 */
double density_compensate_gas(double rho_ref, double P_ref, double T_ref,
                               double P_actual, double T_actual);

/**
 * @brief Apply temperature compensation to liquid density.
 *
 * Linear expansion model:
 *   ρ_comp = ρ_ref / (1 + β * (T_actual - T_ref))
 *
 * The denominator form is more accurate than the numerator form
 * for large temperature changes, as it preserves mass conservation.
 *
 * @param rho_ref       Reference density (kg/m³) at T_ref
 * @param beta          Thermal expansion coefficient (1/K)
 * @param T_ref         Reference temperature (°C)
 * @param T_actual      Actual temperature (°C)
 * @return              Compensated density (kg/m³)
 *
 * Complexity: O(1)
 */
double density_compensate_liquid(double rho_ref, double beta,
                                  double T_ref, double T_actual);

/**
 * @brief Compute mass flow from volumetric flow with density compensation.
 *
 * Mass flow = volumetric_flow * density_compensated
 *
 * This is the fundamental conversion needed for mass-based ratio control
 * when flow meters measure volumetric flow.
 *
 * @param vol_flow      Volumetric flow in engineering units
 * @param density       Compensated density (kg/m³)
 * @return              Mass flow (kg/s or kg/h)
 *
 * Complexity: O(1)
 */
double mass_flow_from_volume(double vol_flow, double density);

/*===========================================================================
 * L8: Performance Monitoring & Diagnostics
 *===========================================================================*/

/**
 * @brief Compute ratio control loop performance metrics.
 *
 * Calculates:
 *   - Mean ratio error (bias)
 *   - Ratio error standard deviation
 *   - Ratio settling time (after setpoint change)
 *   - Ratio control accuracy (% of time within tolerance)
 *
 * @param ratio_errors      Array of ratio error samples
 * @param n_samples         Number of samples
 * @param tolerance_pct     Acceptable tolerance band (% of R_sp)
 * @param mean_err          [out] Mean ratio error
 * @param std_err           [out] Standard deviation of ratio error
 * @param accuracy_pct      [out] % of samples within tolerance
 *
 * Complexity: O(n)
 */
void ratio_performance_metrics(const double *ratio_errors, int n_samples,
                                double tolerance_pct,
                                double *mean_err, double *std_err,
                                double *accuracy_pct);

/**
 * @brief Detect ratio control loop oscillation.
 *
 * Uses autocorrelation method: if the first autocorrelation zero-crossing
 * occurs at a consistent lag, the loop is oscillating.
 *
 * The oscillation index:
 *   OI = |ACF at first negative peak| / ACF(0)
 *
 * OI > 0.3 → significant oscillation detected
 * OI > 0.5 → severe oscillation
 *
 * @param ratio_errors  Array of ratio error samples
 * @param n_samples     Number of samples
 * @return              Oscillation index (0 to 1)
 *
 * Complexity: O(n²) for full ACF computation
 */
double ratio_detect_oscillation(const double *ratio_errors, int n_samples);

/**
 * @brief Compute the economic impact of ratio control accuracy.
 *
 * For combustion: excess air costs fuel efficiency.
 *   Each 1% excess O2 ≈ 0.3-0.5% fuel efficiency loss.
 *
 * For blending: ratio error means off-spec product.
 *   Off-spec cost = (product_value - rework_value) * off_spec_flow.
 *
 * @param ratio_error_pct    Ratio error as % of setpoint
 * @param master_flow        Master flow (kg/h or similar)
 * @param slave_cost_factor  Cost factor for slave stream ($/kg)
 * @param efficiency_impact  Impact of ratio error on efficiency (fraction/%)
 * @return                   Economic impact ($/h)
 *
 * Complexity: O(1)
 */
double ratio_economic_impact(double ratio_error_pct, double master_flow,
                              double slave_cost_factor, double efficiency_impact);

#ifdef __cplusplus
}
#endif

#endif /* RATIO_ADAPTIVE_H */
