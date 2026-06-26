#ifndef GAIN_SCHEDULE_ADAPTIVE_H
#define GAIN_SCHEDULE_ADAPTIVE_H

#include "gain_schedule_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file    gain_schedule_adaptive.h
 * @brief   Adaptive Gain Scheduling Extensions L5/L7/L8
 *
 * Extends classical gain scheduling with adaptive mechanisms:
 *   - Online schedule update via recursive identification
 *   - Performance-based schedule adjustment
 *   - Fuzzy logic gain interpolation
 *   - Neural network gain mapping
 *   - Multi-model blending (weighted combination)
 *
 * L7 Applications:
 *   - Siemens S7-1200/1500 PID with adaptive gain scheduling
 *   - Rockwell CompactLogix PIDE with scheduled gains
 *   - Temperature control with varying load (HVAC/Building automation)
 *   - pH neutralization with titration curve adaptation
 *   - Flight control envelope protection
 *
 * L8 Advanced Topics:
 *   - Time-varying parameter estimation
 *   - Stochastic gain adaptation (Bayesian updating)
 *   - Monte Carlo robustness validation
 *   - Lyapunov-based adaptation law
 *
 * References:
 *   Landau et al., "Adaptive Control", 2nd Ed., Springer, 2011.
 *   Ioannou & Sun, "Robust Adaptive Control", Prentice Hall, 1996.
 *   Takagi & Sugeno, "Fuzzy identification of systems...", IEEE TSMC, 1985.
 */

/**
 * Recursive Least Squares (RLS) for online process identification.
 * Estimates local FOPDT parameters (K, tau) from I/O data.
 * RLS with exponential forgetting factor.
 *
 * Forgetting factor lambda in (0.95, 0.999):
 *   - Close to 1: slow adaptation, good noise rejection
 *   - Close to 0.95: fast adaptation, more noise-sensitive
 *
 * Returns true if parameter estimates have converged.
 */
typedef struct {
    double theta[3];       /* [K, tau, bias] */
    double P[3][3];        /* Covariance matrix */
    double lambda;         /* Forgetting factor */
    double phi[3];         /* Regression vector [u(k), y(k-1), 1] */
    double y_hat;          /* Predicted output */
    double residual;       /* Prediction error */
    uint32_t n_updates;
    bool     converged;
} gs_rls_estimator_t;

void gs_adaptive_rls_init(gs_rls_estimator_t *est, double lambda);
void gs_adaptive_rls_update(gs_rls_estimator_t *est,
                             double u, double y);
bool gs_adaptive_rls_get_params(const gs_rls_estimator_t *est,
                                 double *K, double *tau);

/**
 * Online adaptation of the gain schedule table.
 * Monitors control performance (IAE, ISE, settling time) and adjusts
 * gains when performance degrades beyond a threshold.
 *
 * Strategy:
 *   1. Monitor recent performance metric
 *   2. Compare with expected performance
 *   3. If degradation detected, perturb gains using gradient search
 *   4. Update schedule entry if improvement achieved
 */
typedef struct {
    double  performance_window[64];
    uint32_t perf_index;
    double  baseline_performance;
    double  current_performance;
    double  performance_threshold;
    double  adaptation_rate;
    double  gradient_step[3];
    bool    adapting;
    uint32_t successful_updates;
    uint32_t failed_updates;
} gs_adaptive_performance_t;

void gs_adaptive_perf_init(gs_adaptive_performance_t *perf,
                            double threshold, double rate);

/**
 * Evaluate current closed-loop performance from error history.
 * Uses ITAE (Integral of Time-weighted Absolute Error).
 * Returns normalized performance (0=terrible, 1=perfect).
 */
double gs_adaptive_evaluate_itae(const double *error_history,
                                  const double *time,
                                  uint32_t n);

/**
 * Evaluate using IAE (Integral of Absolute Error).
 */
double gs_adaptive_evaluate_iae(const double *error_history, uint32_t n);

/**
 * Evaluate using ISE (Integral of Squared Error).
 */
double gs_adaptive_evaluate_ise(const double *error_history, uint32_t n);

/**
 * Gradient-based gain adaptation.
 * Perturbs Kp, Ki, Kd individually, measures performance change,
 * and adjusts in the direction of improvement.
 *
 * Uses simultaneous perturbation stochastic approximation (SPSA)
 * for gradient estimation when noise is present.
 *
 * Returns true if gains were adjusted (improvement found).
 */
bool gs_adaptive_gradient_update(
    gs_pid_state_t *state,
    gain_schedule_table_t *table,
    const gs_adaptive_performance_t *perf,
    uint32_t sched_index);

/**
 * Fuzzy logic gain scheduling.
 * Maps scheduling variable to gain adjustments using fuzzy rules.
 * Membership functions: triangular with configurable spread.
 *
 * Typical rules:
 *   IF error IS large AND rate IS fast THEN increase Kp
 *   IF error IS small AND rate IS slow THEN decrease Kd
 */
typedef struct {
    double center;
    double spread;
} gs_fuzzy_mf_t;

typedef struct {
    gs_fuzzy_mf_t error_mf[5];    /* NL NS ZE PS PL */
    gs_fuzzy_mf_t rate_mf[5];
    double rule_base[5][5][3];    /* [e_idx][de_idx][Kp_delta, Ki_delta, Kd_delta] */
} gs_fuzzy_schedule_t;

void gs_adaptive_fuzzy_init(gs_fuzzy_schedule_t *fs);

/**
 * Compute gain adjustments from fuzzy inference.
 * Inputs: normalized error and error rate in [-1, 1].
 * Outputs: Kp_adj, Ki_adj, Kd_adj as multiplicative factors.
 */
void gs_adaptive_fuzzy_infer(const gs_fuzzy_schedule_t *fs,
                              double error_norm,
                              double error_rate_norm,
                              double *Kp_adj,
                              double *Ki_adj,
                              double *Kd_adj);

/**
 * Multi-model blending: Use a bank of pre-designed PID controllers
 * and blend their outputs based on the scheduling variable.
 *
 * This implements the "soft switching" approach to gain scheduling,
 * where multiple controllers run in parallel and their outputs are
 * linearly combined based on operating region membership.
 *
 * n_models: number of PID models in the bank
 * weights: array of blending weights [n_models] (must sum to 1.0)
 * outputs: array of individual controller outputs [n_models]
 *
 * Returns blended output.
 */
double gs_adaptive_blend_outputs(const double *weights,
                                  const double *outputs,
                                  uint32_t n_models);

/**
 * Compute blending weights from scheduling variable using
 * Gaussian membership functions centered at each breakpoint.
 */
void gs_adaptive_gaussian_weights(double sched_val,
                                   const double *centers,
                                   const double *sigmas,
                                   uint32_t n,
                                   double *weights);

#ifdef __cplusplus
}
#endif
#endif /* GAIN_SCHEDULE_ADAPTIVE_H */
