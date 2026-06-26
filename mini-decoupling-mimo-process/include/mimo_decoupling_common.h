/**
 * @file mimo_decoupling_common.h
 * @brief Common types and utilities shared across decoupling design methods.
 *
 * Covers L1: Decoupler struct definition, L2: decoupling concept definitions.
 */

#ifndef MIMO_DECOUPLING_COMMON_H
#define MIMO_DECOUPLING_COMMON_H

#include "mimo_model.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * L1 — Decoupler Types
 * ========================================================================== */

/**
 * @brief Decoupler type enumeration
 */
typedef enum {
    DECOUPLER_NONE = 0,           /**< no decoupling */
    DECOUPLER_STATIC = 1,         /**< steady-state decoupling: D(s) = K^{-1} */
    DECOUPLER_IDEAL = 2,          /**< ideal dynamic decoupling */
    DECOUPLER_SIMPLIFIED = 3,     /**< simplified dynamic decoupling */
    DECOUPLER_INVERTED = 4,       /**< inverted decoupling */
    DECOUPLER_SVD = 5             /**< SVD-based decoupling */
} DecouplerType;

/**
 * @brief Decoupler transfer function element D_{ij}(s)
 *
 * Each decoupler element is a proper rational transfer function.
 * Industrial implementations typically use lead-lag + deadtime forms.
 */
typedef struct {
    double gain;                       /**< steady-state gain of decoupler */
    double num[MIMO_MAX_ORDER + 1];   /**< numerator coefficients (b_0..b_n) */
    double den[MIMO_MAX_ORDER + 1];   /**< denominator coefficients (a_0..a_m) */
    int num_order;                     /**< numerator order */
    int den_order;                     /**< denominator order */
    double time_delay;                 /**< dead time compensation */
    bool is_active;                    /**< whether this decoupler element is used */
} DecouplerElement;

/**
 * @brief Full decoupler matrix D(s) ∈ ℂ^{m×m}
 *
 * Placed between the controller outputs and process inputs:
 * U(s) = D(s) * U_c(s), where U_c are the controller signals.
 *
 * The apparent process becomes: G_a(s) = G(s) * D(s)
 * which should be diagonal (or nearly diagonal) for effective decoupling.
 */
typedef struct {
    DecouplerElement elements[MIMO_MAX_DIM][MIMO_MAX_DIM]; /**< D_{ij}(s) */
    int n_inputs;           /**< number of controller outputs (decoupler inputs) */
    int n_outputs;          /**< number of process inputs (decoupler outputs) */
    DecouplerType type;     /**< decoupling method used */
    double condition;       /**< quality metric (e.g., relative gain after decoupling) */
    bool is_causal;         /**< whether the decoupler is realizable */
    bool is_stable;         /**< whether the decoupler itself is stable */
} Decoupler;

/* ==========================================================================
 * L1 — Interaction Analysis Result
 * ========================================================================== */

/**
 * @brief Interaction metric for a decoupled system
 *
 * Measures how much interaction remains after decoupling.
 */
typedef struct {
    double rga_max_off_diagonal;  /**< maximum off-diagonal |RGA_ij| after decoupling */
    double rga_mean_deviation;    /**< mean deviation of RGA from identity */
    double cn_apparent;           /**< condition number of apparent process G*D */
    double ni_apparent;           /**< Niederlinski index of apparent process */
    bool is_acceptable;           /**< whether interaction is within tolerance */
    char summary[256];            /**< human-readable summary */
} InteractionMetric;

/* ==========================================================================
 * L2 — Decoupler Quality Functions
 * ========================================================================== */

/**
 * @brief Initialize an empty decoupler
 */
void decoupler_init(Decoupler *D, int n_inputs, int n_outputs, DecouplerType type);

/**
 * @brief Set a decoupler element D_{i,j}(s) as a FOPDT transfer function
 */
void decoupler_set_fopdt(Decoupler *D, int i, int j,
                          double K, double tau, double theta);

/**
 * @brief Set a decoupler element as a lead-lag compensator
 * D(s) = K * (T_lead*s + 1) / (T_lag*s + 1)
 */
void decoupler_set_leadlag(Decoupler *D, int i, int j,
                            double K, double T_lead, double T_lag);

/**
 * @brief Evaluate decoupler matrix at complex frequency s
 */
void decoupler_evaluate(const Decoupler *D, double complex s,
                         double complex **D_matrix);

/**
 * @brief Compute the apparent (decoupled) process G_a(s) = G(s) * D(s)
 *
 * G_a = G * D where G is p×m and D is m×m.
 * For ideal decoupling, G_a should be diagonal.
 *
 * @param model    original MIMO model G(s)
 * @param D        decoupler D(s)
 * @param s        complex frequency point
 * @param Ga       output apparent process matrix (pre-allocated p×m)
 */
void decoupler_apparent_process(const MIMOModel *model, const Decoupler *D,
                                 double complex s, double complex **Ga);

/**
 * @brief Evaluate interaction remaining after decoupling
 *
 * Computes RGA of the apparent process at steady-state (s=0)
 * and measures deviation from the identity matrix.
 *
 * @param model   original MIMO model
 * @param D       decoupler
 * @param metric  output interaction metric
 * @return        0 on success
 */
int decoupler_interaction_metric(const MIMOModel *model, const Decoupler *D,
                                  InteractionMetric *metric);

/**
 * @brief Check if decoupler is proper (num_order <= den_order for all elements)
 *
 * A non-proper decoupler cannot be realized physically.
 * Returns true if all elements are proper or strictly proper.
 */
bool decoupler_is_proper(const Decoupler *D);

/**
 * @brief Check if decoupler is stable
 *
 * All decoupler poles (roots of denominator polynomial of each element)
 * must lie in the open left half-plane for continuous time.
 * Uses Routh-Hurwitz criterion (L4).
 */
bool decoupler_is_stable(const Decoupler *D);

/**
 * @brief Compute Routh array for stability check (L4 — Routh-Hurwitz criterion)
 *
 * @param den       denominator coefficients [a_0, ..., a_n]
 * @param order     polynomial order
 * @param routh     output Routh array, (order+1) rows pre-allocated
 * @return          number of sign changes (unstable poles if > 0)
 *
 * Reference: Routh (1877), "A Treatise on the Stability of a Given
 *            State of Motion"
 */
int mimo_routh_hurwitz(const double *den, int order, double *routh);

/**
 * @brief Compute poles of a transfer function by finding roots of denominator
 *        polynomial using the companion matrix eigenvalue method
 *
 * @param den       denominator coefficients [a_0, ..., a_n], a_n = 1
 * @param order     polynomial order
 * @param poles     output complex poles array (pre-allocated, order elements)
 * @return          number of real poles found
 */
int mimo_find_poles(const double *den, int order, double complex *poles);

#ifdef __cplusplus
}
#endif

#endif /* MIMO_DECOUPLING_COMMON_H */
