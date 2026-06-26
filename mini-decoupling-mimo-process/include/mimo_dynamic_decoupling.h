/**
 * @file mimo_dynamic_decoupling.h
 * @brief Dynamic Decoupling Design — Ideal, Simplified, and Partial methods.
 *
 * Knowledge Coverage:
 *   L1: DynamicDecoupler struct
 *   L2: Dynamic decoupling concept (canceling off-diagonal dynamics)
 *   L3: Discrete-time implementation with FIR/IIR filters
 *   L5: Ideal decoupling algorithm, Simplified decoupling algorithm
 *   L8: Robustness to model mismatch, Lyapunov-based stability analysis
 *
 * References:
 *   - Wang, Q.-G. "Decoupling Control" (2003), Lecture Notes in Control
 *     and Information Sciences, Vol.285
 *   - Skogestad & Postlethwaite (2005), Ch.10
 *   - CMU 24-677, Chapter 14 (Multivariable Controller Design)
 *   - Princeton MAE 546, Lecture 18 (MIMO Design)
 */

#ifndef MIMO_DYNAMIC_DECOUPLING_H
#define MIMO_DYNAMIC_DECOUPLING_H

#include "mimo_decoupling_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * L1 — Dynamic Decoupler Types
 * ========================================================================== */

/**
 * @brief Full dynamic decoupler configuration
 */
typedef struct {
    Decoupler base;   /**< base decoupler with element matrix D_{ij}(s) */
    double bandwidth;  /**< design bandwidth for decoupling (rad/s) */
    int method;        /**< 0=ideal, 1=simplified, 2=partial */
    double tolerance;  /**< magnitude tolerance for which off-diagonals to cancel */
    bool use_fir;      /**< if true, use FIR approximation for implementation */
    int fir_length;    /**< FIR filter length for discrete-time implementation */
} DynamicDecoupler;

/* ==========================================================================
 * L5 — Design Algorithms
 * ========================================================================== */

/**
 * @brief Design ideal dynamic decoupler
 *
 * Ideal decoupling: D(s) = G^{-1}(s) * diag(G(s))
 *
 * This completely eliminates interaction at all frequencies.
 * However, G^{-1}(s) often contains unstable poles or is non-proper,
 * making it unrealizable in practice.
 *
 * The resulting apparent process is:
 *   G_a(s) = G(s) * D(s) = diag(G_{11}(s), G_{22}(s), ..., G_{pp}(s))
 *
 * Algorithm (for 2x2 case):
 *   D_{11}(s) = 1
 *   D_{12}(s) = -G_{12}(s) / G_{11}(s)
 *   D_{21}(s) = -G_{21}(s) / G_{22}(s)
 *   D_{22}(s) = 1
 *
 * @param model  MIMO process model
 * @param dd     output dynamic decoupler (pre-allocated)
 * @return       0 on success, -1 if any G_{ii} is non-minimum phase
 *
 * Reference: Wang (2003), §3.2
 */
int mimo_ideal_dynamic_decoupler(const MIMOModel *model, DynamicDecoupler *dd);

/**
 * @brief Design simplified dynamic decoupler
 *
 * Simplified decoupling uses the diagonal elements of G(s) to cancel
 * off-diagonal effects, but with additional lead-lag elements to ensure
 * realizability.
 *
 * For 2x2:
 *   D_{11}(s) = D_{22}(s) = 1
 *   D_{12}(s) = -G_{12}(s) / G_{11}(s) * F_{12}(s)
 *   D_{21}(s) = -G_{21}(s) / G_{22}(s) * F_{21}(s)
 *
 * where F_{ij}(s) are lead-lag filters to ensure properness.
 *
 * @param model  MIMO process model
 * @param dd     output dynamic decoupler
 * @return       0 on success
 *
 * Reference: Luyben (1990), Ch.13; Seborg (2016), §18.5.2
 */
int mimo_simplified_dynamic_decoupler(const MIMOModel *model, DynamicDecoupler *dd);

/**
 * @brief Design partial (band-limited) dynamic decoupler
 *
 * Decoupling is applied only up to a specified bandwidth omega_bw.
 * This avoids amplifying high-frequency noise and improves robustness.
 *
 * D(s) = [low-pass filtered version of ideal decoupler]
 *
 * @param model     MIMO model
 * @param dd        output decoupler, dd->bandwidth must be set
 * @return          0 on success
 *
 * Reference: Hovd & Skogestad (1994), "Pairing Criteria for
 *            Decentralized Control", Automatica 30(6)
 */
int mimo_partial_dynamic_decoupler(const MIMOModel *model, DynamicDecoupler *dd);

/**
 * @brief Apply dynamic decoupler to controller output signals in time domain
 *
 * Implements: u[k] = D(z) * u_c[k] using discrete-time state-space realization.
 *
 * Each decoupler element D_{ij}(z) is realized as a state-space system.
 *
 * @param dd           dynamic decoupler with discrete-time elements
 * @param u_controller controller output vector [m] at current time step
 * @param u_process    process input vector [m] (output)
 * @param states       state vectors for each decoupler element (persistent,
 *                     size [m][m][max_order], caller-managed memory)
 */
void mimo_dynamic_decoupler_step(const DynamicDecoupler *dd,
                                  const double *u_controller,
                                  double *u_process,
                                  double ***states);

/**
 * @brief Convert continuous-time decoupler elements to discrete-time
 *        using Tustin's bilinear transform for DSP implementation
 *
 * @param dd  dynamic decoupler (modified in-place, D(s) -> D(z))
 * @param Ts  sample time (seconds)
 * @return    0 on success
 */
int mimo_decoupler_c2d(DynamicDecoupler *dd, double Ts);

/**
 * @brief Compute frequency response magnitude of the decoupled system
 *
 * Returns |G_a(j*omega)| = |G(j*omega) * D(j*omega)| at each frequency.
 * Useful for assessing decoupling quality across the frequency range.
 *
 * @param model       MIMO model
 * @param dd          dynamic decoupler
 * @param freqs       frequency array (rad/s)
 * @param n_freqs     number of frequency points
 * @param magnitudes  output magnitude matrix [n_freqs][p][m], pre-allocated
 */
void mimo_decoupled_freqresp(const MIMOModel *model, const DynamicDecoupler *dd,
                              const double *freqs, int n_freqs,
                              double *magnitudes);

/* ==========================================================================
 * L8 — Robustness Analysis
 * ========================================================================== */

/**
 * @brief Compute structured singular value (mu) for robustness analysis
 *
 * mu measures the smallest perturbation that destabilizes the decoupled system.
 * Lower bound via power iteration on the spectral radius.
 *
 * @param model MIMO model
 * @param dd    dynamic decoupler
 * @return      mu lower bound (higher = less robust)
 *
 * Reference: Doyle (1982), "Analysis of Feedback Systems with Structured
 *            Uncertainties", IEE Proc. 129(6)
 */
double mimo_mu_analysis(const MIMOModel *model, const DynamicDecoupler *dd);

/**
 * @brief Lyapunov function verification for decoupled system stability (L8)
 *
 * For the apparent process G_a(s) = G(s)*D(s), verify stability by
 * checking if there exists P = P^T > 0 such that A^T P + P A < 0
 * (Lyapunov equation for continuous-time state-space).
 *
 * Solves the Lyapunov equation via Bartels-Stewart algorithm.
 *
 * @param ss_A  state matrix A of the apparent process
 * @param n     dimension
 * @param P     output Lyapunov matrix (pre-allocated n×n)
 * @return      true if P > 0 (stable), false otherwise
 *
 * Reference: Lyapunov (1892), "The General Problem of the Stability of Motion"
 *            + Bartels & Stewart (1972), "Solution of the Matrix Equation
 *            AX + XB = C", Comm. ACM 15(9)
 */
bool mimo_lyapunov_stability(const double *ss_A, int n, double *P);

/**
 * @brief Monte Carlo robustness test: fraction of perturbed systems stable.
 * @param model      nominal MIMO model
 * @param Kc         controller gain vector [n]
 * @param n_trials   number of Monte Carlo trials
 * @param rel_uncert relative uncertainty (e.g., 0.20 = 20%)
 * @return           fraction of stable trials [0, 1]
 */
double mimo_monte_carlo_robustness(const MIMOModel *model, const double *Kc,
                                    int n_trials, double rel_uncert);

/**
 * @brief Compute stability margin det(I + G(0)*diag(Kc))
 */
double mimo_stability_margin(const MIMOModel *model, const double *Kc);

/**
 * @brief Compute sensitivity matrix S(0) = (I + G(0)*diag(Kc))^{-1}
 */
int mimo_sensitivity_dc(const MIMOModel *model, const double *Kc, double *S);

#ifdef __cplusplus
}
#endif

#endif /* MIMO_DYNAMIC_DECOUPLING_H */
