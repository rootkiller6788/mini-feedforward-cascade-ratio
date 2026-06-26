#ifndef FEEDFORWARD_ADVANCED_H
#define FEEDFORWARD_ADVANCED_H

#include "feedforward_defs.h"

/**
 * @file feedforward_advanced.h
 * @brief Advanced feedforward control topics
 *
 * Knowledge: L7 Industrial Applications, L8 Advanced Topics, L9 Industry Frontiers
 *
 * Covers:
 * - Adaptive/gain-scheduled feedforward
 * - Feedforward for non-minimum-phase systems
 * - Stochastic disturbance feedforward
 * - Feedforward with actuator constraints
 * - Multi-variable (MIMO) feedforward
 * - Iterative learning feedforward
 */

/* ============================================================================
 * L7: Industrial applications — Gain-scheduled feedforward
 * ============================================================================ */

/**
 * @brief Gain-scheduled feedforward for nonlinear processes
 *
 * Many industrial processes have operating-point-dependent gains.
 * Gain scheduling adjusts Kff based on the current operating point.
 *
 * Common scheduling variables:
 * - Production rate (throughput)
 * - Feed composition
 * - Equipment fouling factor
 *
 * Schedule structure: (operating_point, Kff) pairs with linear interpolation.
 */
typedef struct {
    double *schedule_x;     /**< Scheduling variable values (monotonic increasing) */
    double *schedule_Kff;   /**< Corresponding feedforward gains */
    int     n_points;       /**< Number of scheduling points */
    double  x_min;          /**< Minimum scheduling variable */
    double  x_max;          /**< Maximum scheduling variable */
} ff_gain_schedule_t;

/**
 * @brief Initialize gain schedule
 *
 * @param gs           Gain schedule structure
 * @param x            Scheduling variable values (n_points elements)
 * @param Kff          Corresponding gains (n_points elements)
 * @param n_points     Number of points (>= 2)
 */
void ff_gain_schedule_init(ff_gain_schedule_t *gs, const double *x,
                           const double *Kff, int n_points);

/**
 * @brief Look up scheduled feedforward gain
 *
 * Uses linear interpolation between schedule points.
 * Clamps to nearest value if x is outside schedule range.
 *
 * @param gs     Gain schedule
 * @param x      Current scheduling variable value
 * @return Interpolated feedforward gain
 */
double ff_gain_schedule_lookup(const ff_gain_schedule_t *gs, double x);

/**
 * @brief Free gain schedule resources
 *
 * @param gs     Gain schedule to free
 */
void ff_gain_schedule_free(ff_gain_schedule_t *gs);

/* ============================================================================
 * L8: Advanced — Non-minimum-phase feedforward
 * ============================================================================ */

/**
 * @brief Detect non-minimum-phase (NMP) behavior in transfer function
 *
 * A transfer function is NMP if it has:
 * - Right-half-plane zeros (inverse response)
 * - Dead time (infinite-dimensional NMP)
 *
 * For NMP systems, Gff(s) = Gp^(-1)(s) * Gd(s) would be unstable.
 * Approximate inversion methods:
 * 1. Ignore dead time: use static + lead-lag
 * 2. All-pass factorization: Gp(s) = Gp_mp(s) * Gp_nmp(s), invert only Gp_mp
 * 3. Limited authority: clip FF output to safe range
 *
 * @param Gp     Process transfer function
 * @return 1 if NMP, 0 if minimum-phase
 */
int ff_is_non_minimum_phase(const tf_t *Gp);

/**
 * @brief Minimum-phase factorization for feedforward
 *
 * Separates Gp(s) = Gp_mp(s) * Gp_nmp(s) where Gp_mp contains
 * all LHP poles/zeros and Gp_nmp contains RHP zeros and dead time.
 *
 * The implementable feedforward inverts only Gp_mp:
 *   Gff(s) = -Gp_mp^(-1)(s) * Gd(s)
 *
 * @param Gp          Input: process transfer function
 * @param Gp_mp       Output: minimum-phase factor
 * @param Gp_nmp      Output: non-minimum-phase factor
 * @return 0 on success
 */
int ff_factor_minimum_phase(const tf_t *Gp, tf_t *Gp_mp, tf_t *Gp_nmp);

/* ============================================================================
 * L8: Advanced — Feedforward with actuator limits
 * ============================================================================ */

/**
 * @brief Feedforward allocation with actuator rate and range limits
 *
 * Industrial actuators have:
 * - Position limits (physical stops)
 * - Rate limits (how fast the actuator can move)
 * - Deadband/hysteresis
 *
 * This function computes the achievable feedforward output considering
 * all actuator constraints. If the ideal FF exceeds limits, the excess
 * is recorded for the feedback controller to handle.
 *
 * @param ff             Feedforward controller
 * @param d_meas         Disturbance measurement
 * @param u_fb           Feedback contribution
 * @param rate_limit     Max actuator rate of change [units/s]
 * @param sat_upper      Upper saturation limit
 * @param sat_lower      Lower saturation limit
 * @param u_out          Output: constrained combined output
 * @param ff_unused      Output: portion of FF that could not be applied
 * @return 0 if within limits, -1 if saturated
 */
int feedforward_with_limits(feedforward_t *ff, double d_meas, double u_fb,
                            double rate_limit, double sat_upper, double sat_lower,
                            double *u_out, double *ff_unused);

/* ============================================================================
 * L8: Stochastic disturbance feedforward
 * ============================================================================ */

/**
 * @brief Kalman filter for disturbance estimation
 *
 * When disturbances are not directly measurable, a Kalman filter can
 * estimate the disturbance from process output measurements.
 *
 * State: x = [PV, disturbance]^T
 * Measurement: y = PV
 *
 * This enables "inferential feedforward" — feeding forward on an
 * estimated rather than measured disturbance.
 *
 * Reference: Kalman (1960); Åström & Wittenmark (2008) §9.3
 */
typedef struct {
    double A[4];      /**< 2x2 state transition matrix (row-major) */
    double B[2];      /**< 2x1 input matrix */
    double C[2];      /**< 1x2 output matrix */
    double Q[4];      /**< 2x2 process noise covariance */
    double R;         /**< Measurement noise variance (scalar) */
    double P[4];      /**< 2x2 error covariance */
    double x_hat[2];  /**< Estimated state [PV_hat, d_hat] */
    double K[2];      /**< Kalman gain [K_pv, K_d] */
    double Ts;        /**< Sample time [s] */
    int    initialized;
} ff_kalman_dist_t;

/**
 * @brief Initialize Kalman filter for disturbance estimation
 *
 * @param kf           Kalman filter structure
 * @param A_model      2x2 state transition matrix
 * @param C_model      1x2 output matrix
 * @param Q_noise      2x2 process noise covariance
 * @param R_noise      Measurement noise variance
 * @param x0           Initial state estimate
 * @param P0           Initial error covariance
 * @param Ts           Sample time [s]
 */
void ff_kalman_dist_init(ff_kalman_dist_t *kf, const double A_model[4],
                         const double C_model[2], const double Q_noise[4],
                         double R_noise, const double x0[2],
                         const double P0[4], double Ts);

/**
 * @brief Execute one Kalman filter step
 *
 * Predict: x_hat = A*x_hat + B*u
 * Update:  K = P*C'/(C*P*C' + R)
 *          x_hat = x_hat + K*(y - C*x_hat)
 *          P = (I - K*C)*P
 *
 * @param kf     Kalman filter
 * @param u      Control input (manipulated variable)
 * @param y      Measurement (process variable)
 */
void ff_kalman_dist_step(ff_kalman_dist_t *kf, double u, double y);

/**
 * @brief Get estimated disturbance from Kalman filter
 *
 * @param kf     Kalman filter
 * @return Estimated disturbance value
 */
double ff_kalman_dist_get(const ff_kalman_dist_t *kf);

/* ============================================================================
 * L9: Industry frontiers — Iterative learning feedforward
 * ============================================================================ */

/**
 * @brief Iterative Learning Control (ILC) for repetitive processes
 *
 * ILC improves feedforward control over repeated batches/cycles by
 * learning from previous cycle errors.
 *
 * Update law (P-type ILC):
 *   u_ff[k+1](t) = u_ff[k](t) + gamma * e[k](t+delta)
 *
 * Where k is the batch index and gamma is the learning rate.
 *
 * Common in: injection molding, robotic assembly, chemical batch processes.
 *
 * Reference: Bristow, Tharayil, Alleyne (2006) "A Survey of ILC"
 */
typedef struct {
    double *u_ff_cycle;     /**< Feedforward profile for one cycle */
    double *e_prev_cycle;   /**< Error profile from previous cycle */
    int     n_samples;      /**< Samples per cycle */
    double  gamma;          /**< Learning rate (0..1, smaller = more cautious) */
    double  q_filter;       /**< Q-filter bandwidth (robustness filter) */
    int     cycle_count;    /**< Number of completed cycles */
    double  e_rms;          /**< RMS error of last cycle */
    double  e_rms_prev;     /**< RMS error of previous cycle */
} ff_ilc_t;

/**
 * @brief Initialize ILC feedforward
 *
 * @param ilc         ILC structure
 * @param n_samples   Samples per batch/cycle
 * @param gamma       Learning rate (0 < gamma <= 1)
 * @param q_filter    Q-filter cutoff (0..1, 1 = no filter)
 */
void ff_ilc_init(ff_ilc_t *ilc, int n_samples, double gamma, double q_filter);

/**
 * @brief Record error from one cycle (store for next cycle's update)
 *
 * @param ilc       ILC structure
 * @param error     Error array for one complete cycle (n_samples)
 */
void ff_ilc_record_error(ff_ilc_t *ilc, const double *error);

/**
 * @brief Update feedforward profile for next cycle
 *
 * Applies P-type ILC update law with Q-filter for robustness:
 *   u_ff_new[i] = Q_filter * (u_ff_old[i] + gamma * e[i+1])
 *
 * @param ilc       ILC structure
 * @param u_ff_new  Output: updated feedforward profile
 */
void ff_ilc_update(ff_ilc_t *ilc, double *u_ff_new);

/**
 * @brief Free ILC resources
 */
void ff_ilc_free(ff_ilc_t *ilc);

#endif