/**
 * @file mimo_inverted_decoupling.h
 * @brief Inverted Decoupling — feedforward-based decoupling using the
 *        inverted process model structure.
 *
 * Unlike conventional decoupling (D before process), inverted decoupling
 * uses a feedback-like structure where decoupling signals are injected
 * as feedforward compensations, improving robustness to model errors.
 *
 * Knowledge Coverage:
 *   L2: Inverted decoupling concept
 *   L5: Inverted decoupler design algorithm
 *   L6: Distillation column application (Wood-Berry column)
 *   L8: Robust inverted decoupling with H-infinity shaping
 *
 * References:
 *   - Wade (1997), "Inverted Decoupling: A Neglected Technique"
 *     ISA Transactions, 36(1):3-10
 *   - Garrido et al. (2012), "Inverted Decoupling Internal Model Control"
 *     IEC Research, 51(14):5307-5316
 *   - Purdue ME 575, Lecture 19 (Decoupling Control)
 *   - Cambridge 4F3, Lecture 11 (MIMO Control Design)
 */

#ifndef MIMO_INVERTED_DECOUPLING_H
#define MIMO_INVERTED_DECOUPLING_H

#include "mimo_decoupling_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inverted decoupler configuration
 *
 * In the inverted structure, the controller output u_ci is modified by
 * feedforward terms from other controller outputs before being applied
 * to the process:
 *
 *   u_i = u_ci + sum_{j != i} d_{ij}(s) * u_j
 *
 * where d_{ij}(s) = -G_{ij}(s) / G_{ii}(s)  (for the 2x2 case).
 */
typedef struct {
    Decoupler base;                                    /**< base decoupler */
    DecouplerElement feedforward[MIMO_MAX_DIM][MIMO_MAX_DIM]; /**< d_{ij}(s) */
    double alpha[MIMO_MAX_DIM];                        /**< robustness tuning parameters */
    bool use_imc_structure;                            /**< use IMC-based inverted decoupling */
    double filter_time_const[MIMO_MAX_DIM];            /**< IMC filter time constants */
} InvertedDecoupler;

/**
 * @brief Design standard inverted decoupler
 *
 * For 2×2 system:
 *   d_{12}(s) = -G_{12}(s) / G_{11}(s)  (feedforward from u_2 to u_1 path)
 *   d_{21}(s) = -G_{21}(s) / G_{22}(s)  (feedforward from u_1 to u_2 path)
 *
 * The apparent process becomes:
 *   y_1 = G_{11}(s) * u_{c1}
 *   y_2 = G_{22}(s) * u_{c2}
 *
 * Key advantage: the decoupler uses only process model ratios,
 * not inverses, making it more robust to model errors.
 *
 * @param model  MIMO process model
 * @param id     output inverted decoupler (pre-allocated)
 * @return       0 on success, -1 if any diagonal G_ii has RHP zero
 *
 * Reference: Wade (1997)
 */
int mimo_inverted_decoupler_design(const MIMOModel *model, InvertedDecoupler *id);

/**
 * @brief Design inverted decoupler with IMC (Internal Model Control) structure
 *
 * Combines inverted decoupling with IMC filtering for improved robustness.
 * Each feedforward path includes a low-pass filter:
 *
 *   d_{ij}(s) = -G_{ij}(s) / G_{ii}(s) * 1/(lambda_{ij}*s + 1)^{n_{ij}}
 *
 * where lambda_{ij} is the IMC filter constant and n_{ij} ensures properness.
 *
 * @param model   MIMO process model
 * @param id      output inverted decoupler (id->filter_time_const must be set)
 * @param lambda  IMC filter time constant for each input
 * @return        0 on success
 *
 * Reference: Garrido et al. (2012)
 */
int mimo_inverted_imc_decoupler(const MIMOModel *model, InvertedDecoupler *id,
                                 const double *lambda);

/**
 * @brief Execute inverted decoupling in real-time (time-domain step)
 *
 * u_i(t) = u_ci(t) + sum_{j != i} d_{ij} * u_j(t)
 *
 * This requires solving the implicit equation since u_j depends on u_i.
 * The algebraic loop is resolved by direct substitution (for 2×2) or
 * by fixed-point iteration (for higher dimensions).
 *
 * @param id           inverted decoupler
 * @param u_controller controller output vector [m]
 * @param u_process    process input vector [m] (output)
 * @return             number of iterations for algebraic loop resolution
 */
int mimo_inverted_decoupler_step(InvertedDecoupler *id,
                                  const double *u_controller,
                                  double *u_process);

/**
 * @brief Compute robustness margin of inverted decoupler
 *
 * Analyzes sensitivity to multiplicative uncertainty in G_{ii}(s).
 * Returns the maximum relative error in G_{ii} that can be tolerated
 * before the decoupled system becomes unstable.
 *
 * @param model  MIMO process model
 * @param id     designed inverted decoupler
 * @return       robustness margin (0 to 1, higher = more robust)
 */
double mimo_inverted_robustness(const MIMOModel *model, const InvertedDecoupler *id);

/**
 * @brief Verify the inverted decoupler does not introduce algebraic loops
 *        for n > 2 by checking the directed graph of feedforward connections
 *
 * An algebraic loop occurs if the feedforward graph has a cycle.
 * Uses depth-first search for cycle detection.
 *
 * @param id  inverted decoupler
 * @return    true if no cycles (valid), false if algebraic loop exists
 */
bool mimo_inverted_no_algebraic_loop(const InvertedDecoupler *id);

#ifdef __cplusplus
}
#endif

#endif /* MIMO_INVERTED_DECOUPLING_H */
