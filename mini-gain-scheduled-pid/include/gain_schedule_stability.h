#ifndef GAIN_SCHEDULE_STABILITY_H
#define GAIN_SCHEDULE_STABILITY_H

#include "gain_schedule_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file    gain_schedule_stability.h
 * @brief   Stability Analysis for Gain-Scheduled Systems L4/L5/L8
 *
 * Provides tools for analyzing the stability of gain-scheduled control
 * systems. Covers frozen-time stability, slow-variation analysis,
 * Lyapunov-based methods, and small-gain theorem applications.
 *
 * Key Theorems:
 *   1. Frozen-Time Stability: If the linearized system is stable at each
 *      frozen operating point, the nonlinear scheduled system may still
 *      be unstable if scheduling is too fast (Shamma & Athans 1990).
 *   2. Slow-Variation Condition: If gain variation is sufficiently slow
 *      relative to closed-loop dynamics, frozen-time stability implies
 *      global stability (Desoer 1969).
 *   3. Quadratic Lyapunov Stability: Existence of a common Lyapunov
 *      matrix P > 0 for all frozen operating points guarantees stability
 *      (Apkarian & Gahinet 1995).
 *   4. Small-Gain Theorem: Stability under unmodeled dynamics and
 *      scheduling errors.
 *
 * References:
 *   Shamma & Athans, "Analysis of gain scheduled control...", IEEE TAC, 1990.
 *   Apkarian & Gahinet, "Self-scheduled H-infinity control", IEEE TAC, 1995.
 *   Rugh & Shamma, "Research on gain scheduling", Automatica, 2000.
 *   Stillwell, "Stability Preserving Interpolation...", Ph.D. CMU, 2000.
 */

/**
 * Frozen-time stability check: Verify all frozen PID controllers
 * in the schedule are stable (poles in LHP for continuous-time).
 * Uses the Routh-Hurwitz criterion for third-order polynomials.
 *
 * For a PI controller: a0*s^2 + a1*s + a2 = 0, stable iff all a_i > 0.
 * For a PID controller: a0*s^3 + a1*s^2 + a2*s + a3 = 0,
 *   stable iff all a_i > 0 AND a1*a2 > a0*a3.
 */
bool gs_stability_frozen_time_check(const gain_schedule_table_t *table,
                                     const double *K_array,
                                     const double *tau_array,
                                     const double *L_array,
                                     uint32_t n,
                                     char *errmsg, size_t errmsg_size);

/**
 * Compute the maximum eigenvalue of the closed-loop A-matrix
 * at a specific operating point (1st-order Pade for dead time).
 * Returns the real part of the dominant eigenvalue.
 * Negative => stable; positive => unstable.
 */
double gs_stability_spectral_abscissa(double Kp, double Ki, double Kd,
                                       double K, double tau, double L);

/**
 * Slow-variation check: Verify that the rate of gain change
 * is below the threshold required for frozen-time stability to
 * imply global stability.
 *
 * Condition: |dKp/dt| / Kp < lambda_min / alpha
 * where lambda_min is the smallest stability margin across the schedule
 * and alpha is a safety factor (typically 0.5-1.0).
 */
bool gs_stability_slow_variation_check(const gain_schedule_table_t *table,
                                        double max_slew_rate,
                                        double alpha,
                                        char *errmsg, size_t errmsg_size);

/**
 * Compute the stability margin (distance to instability) for the
 * worst-case operating point in the schedule.
 * Returns the minimum distance in parameter space.
 */
double gs_stability_min_margin(const gain_schedule_table_t *table,
                                const double *K_array,
                                const double *tau_array,
                                const double *L_array,
                                uint32_t n);

/**
 * Small-gain analysis: Check if the gain scheduling error can
 * destabilize the system when viewed as additive uncertainty.
 *
 * Condition: ||Delta(s)|| * ||T(s)|| < 1
 * where Delta(s) is the multiplicative gain error between
 * interpolated and true optimal gains, and T(s) is the
 * complementary sensitivity function.
 */
bool gs_stability_small_gain_check(const pid_gain_set_t *nominal,
                                    const pid_gain_set_t *scheduled,
                                    double process_gain,
                                    double process_tau,
                                    double process_delay);

/**
 * Routh-Hurwitz stability criterion for a cubic polynomial:
 *   a0*s^3 + a1*s^2 + a2*s + a3
 * Returns true if all roots have negative real parts.
 * Conditions: a0>0, a1>0, a2>0, a3>0, a1*a2 > a0*a3
 */
bool gs_stability_routh_hurwitz_cubic(double a0, double a1,
                                       double a2, double a3);

/**
 * Routh-Hurwitz for quadratic: a0*s^2 + a1*s + a2
 * Conditions: a0>0, a1>0, a2>0 (all coefficients same sign)
 */
bool gs_stability_routh_hurwitz_quadratic(double a0, double a1, double a2);

/**
 * Compute gain margin [dB] and phase margin [degrees] for a
 * PID-controlled FOPDT process using frequency-domain analysis.
 * Uses numerical search of the Nyquist plot.
 */
bool gs_stability_margins_fopdt(double Kp, double Ki, double Kd,
                                 double K, double tau, double L,
                                 double *gm_dB, double *pm_deg);

/**
 * Compute the maximum allowable scheduling rate for stability.
 * Based on the slow-variation theorem (Desoer 1969).
 *
 * max_rate = (stability_margin * bandwidth) / (peak_sensitivity)
 */
double gs_stability_max_scheduling_rate(double stability_margin,
                                         double bandwidth,
                                         double peak_sensitivity);

/**
 * Lyapunov-based stability: Check if there exists a common quadratic
 * Lyapunov function V(x) = x'*P*x for all operating points.
 * Uses a simplified check based on the frozen closed-loop poles.
 * Returns the Lyapunov matrix condition number (lower is better).
 * Returns -1 if no common function found.
 */
double gs_stability_lyapunov_condition(const gain_schedule_table_t *table,
                                        const double *K_array,
                                        const double *tau_array,
                                        const double *L_array,
                                        uint32_t n);

double gs_stability_delay_margin(double phase_margin_deg,
                                  double crossover_freq);

double gs_stability_modulus_margin(double sensitivity_peak);

int32_t gs_stability_closed_loop_poles(double Kp, double Ki, double Kd,
                                        double K, double tau, double L,
                                        double *real_poles);

#ifdef __cplusplus
}
#endif
#endif /* GAIN_SCHEDULE_STABILITY_H */
