/**
 * @file mimo_static_decoupling.h
 * @brief Static (Steady-State) Decoupling Design
 *
 * D = K^{-1} — the simplest decoupling method.
 * The decoupler is a constant matrix that cancels steady-state interaction.
 *
 * Knowledge Coverage:
 *   L1: StaticDecoupler struct
 *   L2: Steady-state decoupling concept
 *   L4: Invertibility criterion, singular value sensitivity
 *   L5: SVD-based pseudo-inverse for non-square plants
 *
 * References:
 *   - Skogestad & Postlethwaite (2005), Ch.10.8
 *   - Seborg et al. (2016), Ch.18.5
 *   - Berkeley ME233, Lecture 16 (Decoupling Control)
 */

#ifndef MIMO_STATIC_DECOUPLING_H
#define MIMO_STATIC_DECOUPLING_H

#include "mimo_decoupling_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Static decoupler configuration
 */
typedef struct {
    Decoupler base;                                  /**< base decoupler struct */
    double K_inv[MIMO_MAX_DIM][MIMO_MAX_DIM];       /**< inverse gain matrix K^{-1} */
    double sv[MIMO_MAX_DIM];                         /**< singular values of K */
    double condition_number;                         /**< condition number of K */
    bool is_square;                                  /**< true if p == m */
    bool uses_pseudoinverse;                         /**< true if SVD pseudoinverse used */
    double pseudoinv_tolerance;                      /**< SVD truncation tolerance */
} StaticDecoupler;

/**
 * @brief Design static decoupler D = K^{-1} for square MIMO system
 *
 * For m=p (square plant):
 *   D = K^{-1}
 *   G_a = G(0) * K^{-1} = I (at steady state)
 *
 * Requirements:
 *   - Plant must be square (p == m)
 *   - K must be non-singular: det(K) != 0
 *
 * @param model  MIMO model (provides K = G(0))
 * @param sd     output static decoupler (pre-allocated)
 * @return       0 on success, -1 if K is singular, -2 if not square
 *
 * Algorithm:
 *   1. Extract K = G(0) from model
 *   2. Compute K^{-1} via LU decomposition with partial pivoting
 *   3. Set D = K^{-1}
 *   4. Compute condition number for quality assessment
 *
 * Reference: Luyben (1990), "Process Modeling, Simulation and Control
 *            for Chemical Engineers", Ch.13
 */
int mimo_static_decoupler_design(const MIMOModel *model, StaticDecoupler *sd);

/**
 * @brief Design static decoupler for non-square MIMO system using
 *        Moore-Penrose pseudoinverse via SVD
 *
 * For p != m:
 *   If p > m (more outputs): D = (K^T K)^{-1} K^T  (left inverse)
 *   If p < m (more inputs):  D = K^T (K K^T)^{-1}   (right inverse)
 *
 * Singular values below tolerance are zeroed to handle ill-conditioning.
 *
 * @param model      MIMO model
 * @param sd         output static decoupler
 * @param tolerance  SVD truncation tolerance (e.g., 1e-6)
 * @return           0 on success
 *
 * Reference: Golub & Van Loan (2013), §5.5
 */
int mimo_static_decoupler_pseudoinv(const MIMOModel *model, StaticDecoupler *sd,
                                     double tolerance);

/**
 * @brief Apply static decoupler to control signals
 *
 * u_process = D * u_controller
 *
 * where D ∈ ℝ^{m×m} is the static decoupling matrix,
 * u_controller ∈ ℝ^m is the decentralized controller output,
 * u_process ∈ ℝ^m is the actual process input.
 *
 * @param sd            static decoupler
 * @param u_controller  controller output vector (length m)
 * @param u_process     process input vector (length m), output
 */
void mimo_static_decoupler_apply(const StaticDecoupler *sd,
                                  const double *u_controller,
                                  double *u_process);

/**
 * @brief Compute apparent steady-state gain after static decoupling
 *
 * K_a = K * D
 *
 * Should be close to identity for good decoupling.
 *
 * @param model original model
 * @param sd    static decoupler
 * @param Ka    apparent gain matrix (output, pre-allocated n×n)
 */
void mimo_static_apparent_gain(const MIMOModel *model, const StaticDecoupler *sd,
                                double *Ka);

/**
 * @brief Analyze sensitivity of decoupling to gain errors
 *
 * If the true gain K_true differs from the design gain K_design,
 * the residual interaction is |K_true * K_design^{-1} - I|.
 *
 * This function computes the worst-case residual for a given
 * fractional gain uncertainty.
 *
 * @param model      MIMO model with nominal gains
 * @param sd         designed static decoupler
 * @param rel_error  relative error in each gain element (e.g., 0.1 = 10%)
 * @return           worst-case residual interaction norm (Frobenius)
 *
 * Reference: Skogestad & Postlethwaite (2005), §6.4 (Robustness)
 */
double mimo_static_sensitivity(const MIMOModel *model, const StaticDecoupler *sd,
                                double rel_error);

#ifdef __cplusplus
}
#endif

#endif /* MIMO_STATIC_DECOUPLING_H */
