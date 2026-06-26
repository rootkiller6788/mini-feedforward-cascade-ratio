/**
 * @file mimo_svd_decoupling.h
 * @brief SVD-Based Decoupling — uses Singular Value Decomposition to design
 *        a decoupling pre/post-compensator that achieves diagonal dominance
 *        across the frequency range of interest.
 *
 * Knowledge Coverage:
 *   L1: SVDDecoupler struct
 *   L2: SVD decoupling concept, principal directions
 *   L4: SVD theorem (Eckart-Young-Mirsky), maximum singular value sensitivity
 *   L5: SVD decomposition (Golub-Reinsch), H∞-loop shaping with SVD
 *   L8: Structured SVD, worst-case disturbance direction analysis
 *
 * References:
 *   - Skogestad & Postlethwaite (2005), Ch.3.3, Ch.10.8
 *   - Golub & Van Loan, "Matrix Computations" (2013), §8.6
 *   - Stanford ENGR205, Lecture 18 (SVD and MIMO Robustness)
 *   - MIT 2.171, Lecture 12 (SVD in Control Design)
 */

#ifndef MIMO_SVD_DECOUPLING_H
#define MIMO_SVD_DECOUPLING_H

#include "mimo_decoupling_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SVD decoupler configuration
 *
 * G(j*omega) = U * Sigma * V^H
 *
 * Decoupling compensator:
 *   Pre-compensator:   K_pre  = V
 *   Post-compensator:  K_post = U^H
 *
 * The apparent plant becomes: K_post * G * K_pre = Sigma (diagonal!)
 */
typedef struct {
    Decoupler base;
    double U[MIMO_MAX_DIM][MIMO_MAX_DIM];       /**< left singular vectors */
    double Sigma[MIMO_MAX_DIM];                  /**< singular values (sorted descending) */
    double V[MIMO_MAX_DIM][MIMO_MAX_DIM];       /**< right singular vectors */
    int n;                                        /**< dimension */
    double condition_number;                      /**< sigma_max / sigma_min */
    double effective_rank;                        /**< rank based on singular value drop-off */
    double frequency;                             /**< frequency at which SVD was computed */
} SVDDecoupler;

/**
 * @brief Compute SVD of steady-state gain matrix K
 *
 * K = U * Sigma * V^T  (real SVD, working with ℝ^{n×n})
 *
 * Uses the Golub-Reinsch algorithm (one-sided Jacobi for small n <= MIMO_MAX_DIM):
 *   1. Bidiagonalize K
 *   2. Diagonalize bidiagonal matrix via implicit QR
 *   3. Accumulate U and V
 *
 * @param K   input matrix, row-major, n×n
 * @param n   dimension
 * @param sd  output SVD decomposer (pre-allocated, filled with U, Sigma, V)
 * @return    0 on success
 *
 * Complexity: O(n^3) for full SVD
 *
 * Reference: Golub & Reinsch (1970), "Singular Value Decomposition and
 *            Least Squares Solutions", Numerische Mathematik 14:403-420
 */
int mimo_svd_decompose(const double *K, int n, SVDDecoupler *sd);

/**
 * @brief Design SVD-based static decoupler
 *
 * Decoupler D = V * Sigma^{-1} * U^T
 *
 * This is a special case: the pre/post compensator pair (V, U^T) makes
 * the apparent process diagonal with gains = singular values.
 *
 * For square systems, this reduces to D = K^{-1} (same as static decoupling
 * with improved numerical properties through SVD).
 *
 * @param model  MIMO model
 * @param sd     output SVD decoupler
 * @return       0 on success
 *
 * Reference: Skogestad & Postlethwaite (2005), §10.8
 */
int mimo_svd_static_decoupler(const MIMOModel *model, SVDDecoupler *sd);

/**
 * @brief Design frequency-dependent SVD decoupler
 *
 * Computes SVD of G(j*omega) at multiple frequencies and fits
 * the singular vector matrices U(omega), V(omega) with rational
 * transfer functions.
 *
 * @param model     MIMO model
 * @param sd        output SVD decoupler (at DC, for initialization)
 * @param freqs     frequency grid (rad/s)
 * @param n_freqs   number of frequency points
 * @return          0 on success
 *
 * Reference: Hung & MacFarlane (1982), "Multivariable Feedback: A
 *            Quasi-Classical Approach", Lecture Notes in Control 40
 */
int mimo_svd_dynamic_decoupler(const MIMOModel *model, SVDDecoupler *sd,
                                const double *freqs, int n_freqs);

/**
 * @brief Apply SVD decoupler: u_process = V * Sigma_inv * U^T * u_controller
 *
 * For the frequency-domain design, this applies the static (DC) approximation.
 *
 * @param sd            SVD decoupler
 * @param u_controller  controller output vector
 * @param u_process     process input vector (output)
 */
void mimo_svd_decoupler_apply(const SVDDecoupler *sd,
                               const double *u_controller,
                               double *u_process);

/**
 * @brief Compute the minimum singular value over frequency grid
 *
 * sigma_min(G(j*omega)) is the MIMO generalization of gain margin.
 * A small sigma_min indicates poor controllability at that frequency.
 *
 * @param model   MIMO model
 * @param freqs   frequency grid
 * @param n_freqs number of frequency points
 * @param sig_min output array of minimum singular values [n_freqs]
 *
 * Reference: Skogestad & Postlethwaite (2005), §6.5
 */
void mimo_min_singular_value(const MIMOModel *model, const double *freqs,
                              int n_freqs, double *sig_min);

/**
 * @brief Align input/output directions for maximal decoupling (L8)
 *
 * Using the principal gains framework, find the frequency where
 * the misalignment between input and output principal directions
 * is minimized, indicating the optimal operating bandwidth for
 * SVD-based decoupling.
 *
 * @param model         MIMO model
 * @param freqs         frequency grid
 * @param n_freqs       number of frequency points
 * @param best_freq     output: best frequency for decoupling
 * @param misalignment  output: misalignment angle at best frequency (rad)
 * @return              0 on success
 *
 * Reference: MacFarlane & Kouvaritakis (1977), "A Design Technique for
 *            Linear Multivariable Feedback Systems", IJC 25(6)
 */
int mimo_principal_gains_alignment(const MIMOModel *model, const double *freqs,
                                    int n_freqs, double *best_freq,
                                    double *misalignment);

#ifdef __cplusplus
}
#endif

#endif /* MIMO_SVD_DECOUPLING_H */
