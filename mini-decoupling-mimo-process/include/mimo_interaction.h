/**
 * @file mimo_interaction.h
 * @brief MIMO Interaction Analysis — RGA, Niederlinski Index, Condition Number,
 *        Dynamic RGA, and pairing selection.
 *
 * Knowledge Coverage:
 *   L1 Definitions:  RGA matrix, NI, condition number, DRGA
 *   L2 Core Concepts: interaction measure, pairing rule, integrity
 *   L4 Engineering Laws: Bristol's RGA theorem, Niederlinski stability theorem
 *   L5 Algorithms: RGA computation, NI computation, optimal pairing search
 *
 * References:
 *   - Bristol, E.H. (1966). "On a New Measure of Interaction for
 *     Multivariable Process Control." IEEE Trans. Auto. Control, AC-11(1).
 *   - Niederlinski, A. (1971). "A Heuristic Approach to the Design of
 *     Linear Multivariable Interacting Control Systems." Automatica, 7(6).
 *   - Skogestad & Postlethwaite (2005), Ch.3, Ch.10
 *   - CMU 24-677, Lecture 12 (Interaction & Decoupling)
 *   - Georgia Tech ECE 6550, Ch.7 (Multivariable Frequency Response)
 */

#ifndef MIMO_INTERACTION_H
#define MIMO_INTERACTION_H

#include "mimo_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * L1 — RGA (Relative Gain Array) structure
 * ========================================================================== */

/**
 * @brief RGA (Relative Gain Array) for static interaction analysis
 *
 * RGA = K .* (K^{-1})^T   where .* is the Hadamard (element-wise) product
 *
 * Properties (Bristol, 1966):
 *   - Each row and column sums to 1
 *   - lambda_ij close to 1 => weak interaction, good pairing
 *   - lambda_ij < 0     => sign reversal, avoid pairing
 *   - lambda_ij >> 1    => severe interaction, decoupling needed
 */
typedef struct {
    double rga[MIMO_MAX_DIM][MIMO_MAX_DIM]; /**< RGA matrix: lambda_ij */
    int n;                                    /**< matrix dimension (square) */
    double cn;                                /**< condition number of K */
    double ni;                                /**< Niederlinski index */
    char interpretation[256];                 /**< human-readable interpretation */
} RGAMatrix;

/* ==========================================================================
 * L1 — Dynamic RGA (DRGA)
 * ========================================================================== */

/**
 * @brief Dynamic RGA evaluated at a specific frequency
 *
 * DRGA(omega) = G(j*omega) .* (G(j*omega)^{-1})^T
 *
 * Reveals how interaction changes with frequency.
 */
typedef struct {
    double rga_magnitude[MIMO_MAX_DIM][MIMO_MAX_DIM];  /**< |DRGA_ij(j*omega)| */
    double rga_phase[MIMO_MAX_DIM][MIMO_MAX_DIM];      /**< phase of DRGA_ij in radians */
    int n;                                               /**< dimension */
    double frequency;                                    /**< frequency omega in rad/s */
} DynamicRGA;

/* ==========================================================================
 * L1 — Pairing Recommendation
 * ========================================================================== */

/** Maximum number of candidate pairings to enumerate */
#define MIMO_MAX_PAIRINGS 64

/**
 * @brief A set of candidate input-output pairings with quality metrics
 */
typedef struct {
    MIMOPairing candidates[MIMO_MAX_PAIRINGS]; /**< candidate pairings sorted by quality */
    int n_candidates;                           /**< number of valid candidates found */
    int best_index;                             /**< index of best pairing in candidates[] */
} PairingSet;

/* ==========================================================================
 * L2/L4 — Core Interaction Functions
 * ========================================================================== */

/**
 * @brief Compute the RGA matrix from steady-state gain matrix K
 *
 * RGA = K .* (K^{-1})^T  (Hadamard product of K and its inverse transpose)
 *
 * Steps:
 *   1. Compute K^{-1} via Gaussian elimination with partial pivoting
 *   2. Transpose K^{-1}
 *   3. Element-wise multiply with K
 *
 * @param K    steady-state gain matrix, row-major, size n×n
 * @param n    dimension of square matrix
 * @param rga  output RGA matrix (pre-allocated)
 * @return     0 on success, -1 if K is singular
 *
 * Theorem (Bristol): Sum of each RGA row = 1, sum of each RGA column = 1.
 *
 * Reference: Bristol (1966), IEEE TAC-11(1):133-134
 */
int mimo_rga_compute(const double *K, int n, RGAMatrix *rga);

/**
 * @brief Compute Niederlinski Index
 *
 * NI = det(K) / prod_i K_{ii}
 *
 * Niederlinski Theorem (1971):
 *   If NI < 0, then the closed-loop system with integral action and
 *   diagonal pairing is unstable (necessary condition for integrity).
 *
 * @param K       steady-state gain matrix, row-major, size n×n
 * @param n       dimension
 * @param pairing pairing vector (pairing[i]=j: output i -> input j), or NULL for diagonal
 * @return        NI value; NaN if denominator zero
 *
 * Reference: Niederlinski (1971), Automatica 7(6):691-701
 */
double mimo_niederlinski_index(const double *K, int n, const int *pairing);

/**
 * @brief Compute condition number of the gain matrix K
 *
 * kappa(K) = sigma_max(K) / sigma_min(K)
 *
 * A large condition number (> 10) indicates the system is ill-conditioned
 * and sensitive to model errors. Strong interaction is expected.
 *
 * Uses power iteration for sigma_max and inverse power iteration for sigma_min.
 *
 * @param K  gain matrix, row-major, size n×n
 * @param n  dimension
 * @return   condition number kappa(K)
 *
 * Reference: Golub & Van Loan, "Matrix Computations" (2013), §2.6
 */
double mimo_condition_number(const double *K, int n);

/**
 * @brief Compute Dynamic RGA at frequency omega
 *
 * DRGA(omega) = G(j*omega) .* (G(j*omega)^{-1})^T
 *
 * Reveals frequency-dependent interaction: a pair may be well-decoupled
 * at low frequency but strongly coupled near crossover.
 *
 * @param model  MIMO model
 * @param omega  frequency in rad/s
 * @param drga   output DRGA (pre-allocated)
 * @return       0 on success, -1 if G(j*omega) is singular
 *
 * Reference: Witcher & McAvoy (1977), "Interaction Control Systems:
 *            Steady-State and Dynamic Treatment"
 */
int mimo_dynamic_rga(const MIMOModel *model, double omega, DynamicRGA *drga);

/**
 * @brief Enumerate all feasible pairings and rank them by quality
 *
 * Uses RGA rules (Bristol, 1966):
 *   1. Prefer RGA_ij close to 1
 *   2. Avoid RGA_ij < 0 (integrity violation)
 *   3. Avoid RGA_ij >> 1 (severe interaction)
 *   4. Check NI > 0 condition
 *
 * For n <= 4, enumerates all n! permutations.
 * For n > 4, uses a heuristic greedy search.
 *
 * @param K       steady-state gain matrix, row-major, n×n
 * @param n       number of inputs/outputs
 * @param pset    output pairing set (pre-allocated)
 * @return        number of feasible pairings found
 *
 * Reference: Bristol (1966) + Seborg et al. (2016), Ch.18.3
 */
int mimo_enumerate_pairings(const double *K, int n, PairingSet *pset);

/**
 * @brief Compute the effective RGA (ERGA) — RGA weighted by bandwidth
 *
 * ERGA_ij = RGA_ij * (omega_cij / omega_cb)
 *
 * where omega_cij is the crossover frequency of G_ij and omega_cb is
 * the target closed-loop bandwidth. This accounts for dynamic effects
 * in pairing selection without full dynamic RGA computation.
 *
 * @param model        MIMO model
 * @param bandwidth    target closed-loop bandwidth (rad/s)
 * @param erga_output  output ERGA matrix [n][n], pre-allocated
 * @return             0 on success
 *
 * Reference: Xiong et al. (2005), "Effective RGA for Control Structure
 *            Design", J. Process Control
 */
int mimo_effective_rga(const MIMOModel *model, double bandwidth, double *erga_output);

/**
 * @brief Check if a pairing has integrity against loop failures
 *
 * Integrity: the system remains stable when any single loop is taken out
 * of automatic mode (sensor/actuator failure). The Niederlinski Index
 * must be > 0 for all principal sub-determinants.
 *
 * @param K       steady-state gain matrix, row-major, n×n
 * @param n       dimension
 * @param pairing pairing vector
 * @return        true if pairing has integrity, false otherwise
 *
 * Reference: Grosdidier, Morari, Holt (1985), "Closed-Loop Properties
 *            from Steady-State Gain Information", IEC Fund., 24(2)
 */
bool mimo_check_integrity(const double *K, int n, const int *pairing);

/**
 * @brief Compute the Interaction Quotient (IQ)
 *
 * IQ = 1 - (sum of RGA elements farthest from 1) / n
 *
 * IQ = 1 means perfect decoupling; IQ near 0 means severe interaction.
 *
 * @param rga  RGA matrix
 * @return     IQ in [0, 1]
 */
double mimo_interaction_quotient(const RGAMatrix *rga);

/**
 * @brief Print RGA matrix and interpretation
 */
void mimo_rga_print(const RGAMatrix *rga);

/**
 * @brief Print pairing recommendations
 */
void mimo_pairing_print(const PairingSet *pset);

#ifdef __cplusplus
}
#endif

#endif /* MIMO_INTERACTION_H */
