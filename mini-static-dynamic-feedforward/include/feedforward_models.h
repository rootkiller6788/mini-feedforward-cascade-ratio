#ifndef FEEDFORWARD_MODELS_H
#define FEEDFORWARD_MODELS_H

#include "feedforward_defs.h"

/**
 * @file feedforward_models.h
 * @brief Process and disturbance model identification and validation
 *
 * Knowledge: L3 Engineering Structures, L4 Engineering Laws, L5 Algorithms
 *
 * Covers model identification methods used in feedforward controller design:
 * - Step response analysis for FOPDT/SOPDT parameter estimation
 * - Frequency response from transfer functions
 * - Model validation and comparison
 * - Pade approximation for dead time
 * - Discretization methods (Tustin, backward Euler, zero-order hold)
 */

/* ============================================================================
 * L3/L5: Model initialization and parameter setting
 * ============================================================================ */

void fopdt_init(fopdt_t *m, double Kp, double tau, double theta);
void sopdt_init(sopdt_t *m, double Kp, double tau1, double tau2, double theta);
void ipdt_init(ipdt_t *m, double Kp, double theta);
void dist_model_init(dist_model_t *m, double Kd, double tau_d, double theta_d);
void disturbance_meas_init(disturbance_meas_t *dm, double range_min, double range_max,
                           double rate_limit);

/* ============================================================================
 * L5: Step response identification methods
 * ============================================================================ */

/**
 * @brief Identify FOPDT model from step response data
 *
 * Uses the Sundaresan-Krishnaswamy (1978) method:
 * - Fit time to reach 35.3% and 85.3% of steady state
 * - tau = 0.67*(t2 - t1), theta = 1.3*t1 - 0.29*t2
 *
 * @param t         Time array [s]
 * @param y         Output response array
 * @param n         Number of data points
 * @param u_step    Input step magnitude
 * @param model     Output: identified FOPDT model
 * @return 0 on success, -1 on insufficient data
 */
int fopdt_identify_step(const double *t, const double *y, int n,
                        double u_step, fopdt_t *model);

/**
 * @brief Identify FOPDT model using the two-point method (Åström-Hägglund)
 *
 * Based on times t28% and t63% (times to reach 28.3% and 63.2%):
 *   tau = t63% - t28%
 *   theta = t63% - tau
 *   Kp = delta_y / delta_u
 *
 * @param t         Time array [s]
 * @param y         Output response array
 * @param n         Number of data points
 * @param u_step    Input step magnitude
 * @param model     Output: identified FOPDT model
 * @return 0 on success
 */
int fopdt_identify_two_point(const double *t, const double *y, int n,
                             double u_step, fopdt_t *model);

/**
 * @brief Identify FOPDT using area method (integral method)
 *
 * Based on computing the area between step response and steady state:
 *   tau = (A1 / Kp), theta = t0 - tau
 * where A1 is the integral of (y_inf - y(t)) from 0 to infinity.
 *
 * Reference: Åström & Hägglund (1995) §2.4
 *
 * @param t         Time array [s]
 * @param y         Output response array
 * @param n         Number of data points
 * @param u_step    Input step magnitude
 * @param model     Output: identified FOPDT model
 * @return 0 on success
 */
int fopdt_identify_area(const double *t, const double *y, int n,
                        double u_step, fopdt_t *model);

/**
 * @brief Identify SOPDT from step response using Smith's method
 *
 * For overdamped second-order systems, fits two time constants
 * from the inflection point of the step response.
 *
 * Reference: C.L. Smith (1972) "Digital Computer Process Control"
 *
 * @param t         Time array [s]
 * @param y         Output response array
 * @param n         Number of data points
 * @param u_step    Input step magnitude
 * @param model     Output: identified SOPDT model
 * @return 0 on success
 */
int sopdt_identify_step(const double *t, const double *y, int n,
                        double u_step, sopdt_t *model);

/* ============================================================================
 * L4: Engineering laws — Step response, transfer function evaluation
 * ============================================================================ */

/**
 * @brief Evaluate FOPDT step response at time t
 *
 * y(t) = Kp * u_step * (1 - exp(-(t-theta)/tau))   for t >= theta
 * y(t) = 0                                         for t < theta
 *
 * @param model     FOPDT model parameters
 * @param u_step    Input step magnitude
 * @param t         Time to evaluate [s]
 * @return Process output at time t
 */
double fopdt_step_response(const fopdt_t *model, double u_step, double t);

/**
 * @brief Evaluate SOPDT step response at time t
 *
 * y(t) = Kp * u_step * (1 + (tau1*exp(-(t-theta)/tau1) - tau2*exp(-(t-theta)/tau2))/(tau2-tau1))
 *        for t >= theta; y(t) = 0 for t < theta
 *
 * @param model     SOPDT model parameters
 * @param u_step    Input step magnitude
 * @param t         Time to evaluate [s]
 * @return Process output at time t
 */
double sopdt_step_response(const sopdt_t *model, double u_step, double t);

/**
 * @brief Evaluate frequency response of a transfer function
 *
 * G(jw) = K * e^(-j*w*theta) * N(jw) / D(jw)
 *
 * @param tf_model  Transfer function
 * @param omega     Angular frequency [rad/s]
 * @param magnitude Output: |G(jw)|
 * @param phase     Output: arg(G(jw)) [rad]
 */
void tf_frequency_response(const tf_t *tf_model, double omega,
                           double *magnitude, double *phase);

/* ============================================================================
 * L5: Pade approximation for dead time
 * ============================================================================ */

/**
 * @brief Compute first-order Pade approximation of dead time
 *
 * e^(-theta*s) ~ (1 - theta*s/2) / (1 + theta*s/2)
 *
 * Returns num and den coefficients for the rational approximation.
 *
 * @param theta     Dead time [s]
 * @param num       Output: numerator coefficients [b1, b0] = [-theta/2, 1]
 * @param den       Output: denominator coefficients [a1, a0] = [theta/2, 1]
 */
void pade_first_order(double theta, double num[2], double den[2]);

/**
 * @brief Compute second-order Pade approximation of dead time
 *
 * e^(-theta*s) ~ (1 - theta*s/2 + theta^2*s^2/12) / (1 + theta*s/2 + theta^2*s^2/12)
 *
 * More accurate than first-order, especially for larger theta*omega products.
 *
 * @param theta     Dead time [s]
 * @param num       Output: numerator [b2, b1, b0]
 * @param den       Output: denominator [a2, a1, a0]
 */
void pade_second_order(double theta, double num[3], double den[3]);

/* ============================================================================
 * L3: Discretization methods
 * ============================================================================ */

/**
 * @brief Discretize continuous FOPDT using zero-order hold (ZOH)
 *
 * G(z) = (1 - z^(-1)) * Z{ G(s)/s }
 *
 * For FOPDT with integer delay steps d = round(theta/Ts):
 *   G(z) = Kp * (1 - exp(-Ts/tau)) * z^(-d-1) / (1 - exp(-Ts/tau) * z^(-1))
 *
 * @param model     FOPDT continuous model
 * @param Ts        Sample time [s]
 * @param dtf       Output: discrete transfer function
 */
void fopdt_to_discrete_zoh(const fopdt_t *model, double Ts, tf_discrete_t *dtf);

/**
 * @brief Discretize using bilinear (Tustin) transformation
 *
 * s ~ (2/Ts)*(z-1)/(z+1)
 *
 * Applied to FOPDT: G(s) = Kp/(tau*s+1)
 * After Tustin: G(z) = Kp * Ts * (z+1) / ((2*tau+Ts)*z + (Ts-2*tau))
 *
 * @param model     FOPDT continuous model (dead time ignored — use Pade first)
 * @param Ts        Sample time [s]
 * @param dtf       Output: discrete transfer function
 */
void fopdt_to_discrete_tustin(const fopdt_t *model, double Ts, tf_discrete_t *dtf);

/**
 * @brief Discretize using backward Euler (first-order)
 *
 * s ~ (z-1)/(Ts*z)  or equivalently  s ~ (1 - z^(-1))/Ts
 *
 * G(z) = Kp * (Ts/(tau+Ts)) * z^(-1) / (1 - (tau/(tau+Ts))*z^(-1))
 *
 * @param model     FOPDT continuous model
 * @param Ts        Sample time [s]
 * @param dtf       Output: discrete transfer function
 */
void fopdt_to_discrete_euler(const fopdt_t *model, double Ts, tf_discrete_t *dtf);

/**
 * @brief General transfer function discretization via Tustin
 *
 * Converts continuous TF to discrete using bilinear transformation.
 * Handles up to 4th order numerator and denominator.
 *
 * @param ct        Continuous transfer function
 * @param Ts        Sample time [s]
 * @param dt        Output: discrete transfer function
 * @return 0 on success, -1 if order too high
 */
int tf_to_discrete_tustin(const tf_t *ct, double Ts, tf_discrete_t *dt);

/* ============================================================================
 * L5: Model validation
 * ============================================================================ */

/**
 * @brief Compute R-squared (coefficient of determination) for model fit
 *
 * R^2 = 1 - SS_res / SS_tot
 * where SS_res = sum((y_actual - y_model)^2), SS_tot = sum((y_actual - y_mean)^2)
 *
 * @param y_actual  Actual data
 * @param y_model   Model predictions
 * @param n         Number of data points
 * @return R^2 value (0..1, 1 = perfect fit)
 */
double model_r_squared(const double *y_actual, const double *y_model, int n);

/**
 * @brief Compute Root Mean Square Error (RMSE)
 *
 * @param y_actual  Actual data
 * @param y_model   Model predictions
 * @param n         Number of data points
 * @return RMSE
 */
double model_rmse(const double *y_actual, const double *y_model, int n);

/**
 * @brief Compute Mean Absolute Error (MAE)
 *
 * @param y_actual  Actual data
 * @param y_model   Model predictions
 * @param n         Number of data points
 * @return MAE
 */
double model_mae(const double *y_actual, const double *y_model, int n);

/**
 * @brief Validate disturbance measurement signal quality
 *
 * Checks: overrange, underrange, stale (no update within timeout),
 * rate-of-change violation, NaN/Inf detection.
 *
 * @param meas          Disturbance measurement
 * @param prev_value    Previous valid value
 * @param prev_time     Previous timestamp [s]
 * @param stale_timeout Max time before stale [s]
 * @return Signal quality status
 */
signal_status_t disturbance_validate(const disturbance_meas_t *meas,
                                     double prev_value, double prev_time,
                                     double stale_timeout);

#endif