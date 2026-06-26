/**
 * @file mimo_model.h
 * @brief MIMO Process Model — transfer function matrices, state-space models,
 *        and multivariable system representations for decoupling control.
 *
 * Knowledge Coverage:
 *   L1 Definitions:  MIMOModel, TFRow, SSModel, MIMOPairing structs
 *   L2 Core Concepts: MIMO transfer function matrix, state-space, pairing
 *   L3 Engineering Structures: discretized continuous-time models, Tustin's method
 *   L4 Engineering Laws: model order, McMillan degree, controllability/observability
 *
 * References:
 *   - Skogestad & Postlethwaite, "Multivariable Feedback Control" (2005), Ch.3
 *   - Seborg, Edgar, Mellichamp, "Process Dynamics and Control" (2016), Ch.18
 *   - MIT 6.302 Feedback Systems, Lecture 18 (MIMO Systems)
 *   - Stanford ENGR205, Lecture 14 (Multivariable Processes)
 */

#ifndef MIMO_MODEL_H
#define MIMO_MODEL_H

#include <stddef.h>
#include <stdbool.h>
#include <complex.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * L1 — Core Type Definitions
 * ========================================================================== */

/** Maximum order of any individual transfer function */
#define MIMO_MAX_ORDER 10
/** Maximum number of inputs/outputs */
#define MIMO_MAX_DIM 8
/** Default tolerance for numerical comparisons */
#define MIMO_EPSILON 1e-10

/**
 * @brief Single transfer function element G_ij(s) = b(s)/a(s)
 *
 * Represents one element of the MIMO transfer function matrix.
 * Numerator:  b_n s^n + ... + b_1 s + b_0
 * Denominator: a_m s^m + ... + a_1 s + a_0  (monic, a_m = 1.0)
 */
typedef struct {
    double num[MIMO_MAX_ORDER + 1];   /**< numerator coefficients (b_0..b_n) */
    double den[MIMO_MAX_ORDER + 1];   /**< denominator coefficients (a_0..a_m), a_m = 1 */
    int num_order;                     /**< order of numerator polynomial */
    int den_order;                     /**< order of denominator polynomial */
    double gain;                       /**< steady-state gain K_ij */
    double time_delay;                 /**< pure time delay (dead time) theta */
    double time_constant;              /**< dominant time constant (first-order + delay approx) */
} MIMOTransferFunction;

/**
 * @brief One row of a MIMO transfer function matrix G(s)
 * Each row corresponds to one output y_i = sum_j G_ij(s) * u_j
 */
typedef struct {
    MIMOTransferFunction elements[MIMO_MAX_DIM]; /**< G_i1, G_i2, ..., G_im */
    int num_inputs;                              /**< number of columns (inputs) */
} MIMOTFRow;

/**
 * @brief Full MIMO transfer function matrix G(s) in C^{p x m}
 *
 * G(s) describes the dynamic relationship: Y(s) = G(s) * U(s)
 * where Y in C^p is the output vector, U in C^m is the input vector.
 */
typedef struct {
    MIMOTFRow rows[MIMO_MAX_DIM];  /**< p rows of the transfer function matrix */
    int num_outputs;               /**< p — number of outputs */
    int num_inputs;                /**< m — number of inputs */
    double sample_time;            /**< sample time Ts for discrete approximation (seconds) */
    char name[128];                /**< process name / tag */
} MIMOModel;

/**
 * @brief Discrete-time state-space model (L3 Engineering Structure)
 *
 * x[k+1] = A x[k] + B u[k]
 * y[k]   = C x[k] + D u[k]
 *
 * For continuous-time: dx/dt = A_c x + B_c u, y = C x + D u
 * Discretization via Tustin/bilinear: s -> 2/Ts * (z-1)/(z+1)
 */
typedef struct {
    double A[MIMO_MAX_DIM][MIMO_MAX_DIM]; /**< state matrix (n x n) */
    double B[MIMO_MAX_DIM][MIMO_MAX_DIM]; /**< input matrix (n x m) */
    double C[MIMO_MAX_DIM][MIMO_MAX_DIM]; /**< output matrix (p x n) */
    double D[MIMO_MAX_DIM][MIMO_MAX_DIM]; /**< feedthrough matrix (p x m) */
    int n_states;                          /**< number of states n */
    int n_inputs;                          /**< number of inputs m */
    int n_outputs;                         /**< number of outputs p */
    double sample_time;                    /**< for discrete-time models, 0 for continuous */
    bool is_discrete;                      /**< true if discrete-time, false if continuous */
} MIMOStateSpace;

/**
 * @brief Input-output pairing configuration (L4 — Bristol's method)
 *
 * Defines which manipulated variable (MV) controls which controlled variable (CV).
 * A pairing is a bijection: {outputs} <-> {inputs}.
 */
typedef struct {
    int pairing[MIMO_MAX_DIM];     /**< pairing[i] = j means output i paired with input j */
    int n_pairs;                    /**< number of paired variables */
    double rga_value;               /**< RGA element value for this pair */
    bool is_feasible;               /**< whether pairing passes stability checks */
    double niederlinski_index;      /**< NI value for this pairing */
} MIMOPairing;

/* ==========================================================================
 * L2 — Core Model Operations
 * ========================================================================== */

/**
 * @brief Initialize a MIMO model with given dimensions
 * @param model      pointer to model (output)
 * @param n_outputs  number of controlled variables (CVs)
 * @param n_inputs   number of manipulated variables (MVs)
 * @param name       process identifier string
 */
void mimo_model_init(MIMOModel *model, int n_outputs, int n_inputs, const char *name);

/**
 * @brief Set a single transfer function element G_{i,j}(s)
 * @param model   MIMO model to modify
 * @param i       output index (0-based)
 * @param j       input index (0-based)
 * @param num     numerator coefficients array [b_0, b_1, ..., b_n]
 * @param num_ord order of numerator
 * @param den     denominator coefficients array [a_0, a_1, ..., a_m]
 * @param den_ord order of denominator
 *
 * Note: denominator is normalized so a_m = 1.0
 */
void mimo_model_set_tf(MIMOModel *model, int i, int j,
                        const double *num, int num_ord,
                        const double *den, int den_ord);

/**
 * @brief Set FOPDT (First-Order Plus Dead Time) element
 * G(s) = K * exp(-theta*s) / (tau*s + 1)
 *
 * This is the most common industrial model form.
 *
 * @param model MIMO model
 * @param i     output index
 * @param j     input index
 * @param K     steady-state gain
 * @param tau   time constant (seconds)
 * @param theta dead time (seconds)
 */
void mimo_model_set_fopdt(MIMOModel *model, int i, int j,
                           double K, double tau, double theta);

/**
 * @brief Set SOPDT (Second-Order Plus Dead Time) element
 * G(s) = K * exp(-theta*s) / (tau^2*s^2 + 2*zeta*tau*s + 1)
 *
 * @param model MIMO model
 * @param i     output index
 * @param j     input index
 * @param K     steady-state gain
 * @param tau   natural period (seconds)
 * @param zeta  damping ratio
 * @param theta dead time (seconds)
 */
void mimo_model_set_sopdt(MIMOModel *model, int i, int j,
                           double K, double tau, double zeta, double theta);

/**
 * @brief Evaluate one transfer function element G_{i,j}(s) at complex frequency s
 *
 * Computes G(s) = (b_n s^n + ... + b_0) / (a_m s^m + ... + a_0)
 * using Horner's method for numerical stability.
 *
 * @param tf  transfer function element
 * @param s   complex frequency point
 * @return    G(s) in C
 *
 * Complexity: O(max(n,m)) arithmetic operations
 */
double complex mimo_tf_evaluate(const MIMOTransferFunction *tf, double complex s);

/**
 * @brief Evaluate the entire MIMO transfer function matrix at frequency s
 * @param model MIMO model
 * @param s     complex frequency
 * @param G     output matrix G(s) in C^{p x m}, size [p][m] pre-allocated
 */
void mimo_model_evaluate(const MIMOModel *model, double complex s, double complex **G);

/**
 * @brief Compute steady-state gain matrix K = G(0) in R^{p x m}
 *
 * K_{ij} = G_{ij}(0) — the static gain from input j to output i.
 * Fundamental for RGA analysis and static decoupling design.
 *
 * @param model MIMO model
 * @param K     output gain matrix, size [p][m] pre-allocated, row-major
 *
 * Reference: Bristol (1966), "On a New Measure of Interaction for
 *            Multivariable Process Control"
 */
void mimo_model_steady_state_gain(const MIMOModel *model, double *K);

/**
 * @brief Convert MIMO transfer function model to state-space representation
 *
 * Uses a block-diagonal controllable canonical form for each SISO element.
 * The total state dimension n = sum of denominator orders of all elements.
 *
 * @param model  MIMO transfer function model (input)
 * @param ss     state-space model (output, pre-allocated)
 *
 * Reference: Kailath, "Linear Systems" (1980), Ch.6
 */
void mimo_model_to_state_space(const MIMOModel *model, MIMOStateSpace *ss);

/**
 * @brief Discretize continuous state-space model using Tustin's bilinear transform
 *
 * s -> (2/Ts) * (z-1)/(z+1)
 *
 * A_d = (I + A*h/2) * inv(I - A*h/2)
 * B_d = inv(I - A*h/2) * B * h
 * C_d = C * inv(I - A*h/2)
 * D_d = D + C * inv(I - A*h/2) * B * h/2
 *
 * where h = sample_time.
 *
 * @param continuous  continuous-time state-space model
 * @param discrete    output discrete-time model (pre-allocated)
 * @param Ts          sample time (seconds)
 *
 * Reference: Franklin, Powell, Workman, "Digital Control of Dynamic Systems"
 *            (1998), Section 11.3
 */
void mimo_ss_c2d_tustin(const MIMOStateSpace *continuous, MIMOStateSpace *discrete, double Ts);

/**
 * @brief Check if a MIMO state-space model is controllable
 *
 * Controllability matrix: C = [B, AB, A^2 B, ..., A^{n-1} B]
 * System is controllable iff rank(C) = n.
 *
 * Uses numerical rank via SVD tolerance MIMO_EPSILON.
 *
 * @param ss  state-space model
 * @return    true if controllable, false otherwise
 *
 * Reference: Kalman (1960), "On the General Theory of Control Systems"
 */
bool mimo_ss_is_controllable(const MIMOStateSpace *ss);

/**
 * @brief Check if a MIMO state-space model is observable
 *
 * Observability matrix: O = [C; CA; CA^2; ...; CA^{n-1}]
 * System is observable iff rank(O) = n.
 *
 * @param ss  state-space model
 * @return    true if observable, false otherwise
 */
bool mimo_ss_is_observable(const MIMOStateSpace *ss);

/**
 * @brief Compute McMillan degree of MIMO model
 *
 * The McMillan degree is the sum of the orders of the denominator polynomials
 * in the Smith-McMillan form, which equals the minimal number of states needed.
 *
 * @param model  MIMO model
 * @return       McMillan degree
 */
int mimo_model_mcmillan_degree(const MIMOModel *model);

/**
 * @brief Print a MIMO model summary (for diagnostics)
 */
void mimo_model_print(const MIMOModel *model);

/* ==========================================================================
 * L6 — Canonical Process Models (Benchmark Problems)
 * ========================================================================== */

/**
 * @brief Initialize Wood-Berry distillation column model (2x2).
 * Reference: Wood & Berry (1973), Chem. Eng. Sci., 28:1707-1717
 */
void mimo_wood_berry_model(MIMOModel *model);

/**
 * @brief Initialize blending process MIMO model (2x2).
 * Reference: Seborg, Edgar, Mellichamp (2016), Ch.18
 */
void mimo_blending_process_model(MIMOModel *model);

/**
 * @brief Compute condition number of steady-state gain matrix.
 */
double mimo_model_condition_number(const MIMOModel *model);

/**
 * @brief Compute minimum singular value of steady-state gain matrix.
 */
double mimo_model_min_singular_value(const MIMOModel *model);

#ifdef __cplusplus
}
#endif

#endif /* MIMO_MODEL_H */
