/**
 * @file mimo_model.c
 * @brief Implementation of MIMO process model operations.
 *
 * Each function implements a distinct knowledge point from L1-L4.
 */

#include "mimo_model.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ==========================================================================
 * L2 — Model Initialization
 * ========================================================================== */

void mimo_model_init(MIMOModel *model, int n_outputs, int n_inputs, const char *name) {
    if (!model) return;
    if (n_outputs <= 0 || n_outputs > MIMO_MAX_DIM) n_outputs = 1;
    if (n_inputs <= 0 || n_inputs > MIMO_MAX_DIM) n_inputs = 1;

    model->num_outputs = n_outputs;
    model->num_inputs = n_inputs;
    model->sample_time = 0.01; /* default 10ms */

    if (name) {
        strncpy(model->name, name, sizeof(model->name) - 1);
        model->name[sizeof(model->name) - 1] = '\0';
    } else {
        model->name[0] = '\0';
    }

    /* Initialize all transfer function elements to zero */
    for (int i = 0; i < n_outputs; i++) {
        model->rows[i].num_inputs = n_inputs;
        for (int j = 0; j < n_inputs; j++) {
            MIMOTransferFunction *tf = &model->rows[i].elements[j];
            memset(tf->num, 0, sizeof(tf->num));
            memset(tf->den, 0, sizeof(tf->den));
            tf->num_order = 0;
            tf->den_order = 0;
            tf->num[0] = 0.0;
            tf->den[0] = 1.0;
            tf->gain = 0.0;
            tf->time_delay = 0.0;
            tf->time_constant = 0.0;
        }
    }
}

/* ==========================================================================
 * L2 — Setting Transfer Function Elements
 * ========================================================================== */

void mimo_model_set_tf(MIMOModel *model, int i, int j,
                        const double *num, int num_ord,
                        const double *den, int den_ord) {
    if (!model || !num || !den) return;
    if (i < 0 || i >= model->num_outputs || j < 0 || j >= model->num_inputs) return;

    MIMOTransferFunction *tf = &model->rows[i].elements[j];

    /* Clamp orders */
    if (num_ord > MIMO_MAX_ORDER) num_ord = MIMO_MAX_ORDER;
    if (den_ord > MIMO_MAX_ORDER) den_ord = MIMO_MAX_ORDER;
    if (num_ord < 0) num_ord = 0;
    if (den_ord < 0) den_ord = 0;

    tf->num_order = num_ord;
    tf->den_order = den_ord;

    /* Copy numerator */
    memset(tf->num, 0, sizeof(tf->num));
    for (int k = 0; k <= num_ord; k++) {
        tf->num[k] = num[k];
    }

    /* Copy denominator and normalize to monic.
     * Both numerator and denominator are divided by the leading
     * denominator coefficient to preserve G(s) = N(s)/D(s). */
    memset(tf->den, 0, sizeof(tf->den));
    double leading = den[den_ord];
    if (fabs(leading) < MIMO_EPSILON) leading = 1.0; /* safety */

    for (int k = 0; k <= den_ord; k++) {
        tf->den[k] = den[k] / leading;
    }
    tf->den[den_ord] = 1.0; /* enforce monic */

    /* Also divide numerator by leading to preserve transfer function */
    for (int k = 0; k <= num_ord; k++) {
        tf->num[k] = num[k] / leading;
    }

    /* Compute steady-state gain: G(0) = num[0] / den[0] */
    if (fabs(tf->den[0]) > MIMO_EPSILON) {
        tf->gain = tf->num[0] / tf->den[0];
    } else {
        tf->gain = 0.0;
    }

    /* Approximate dominant time constant as -1/pole (for 1st-order systems) */
    if (den_ord >= 1) {
        tf->time_constant = 1.0 / fabs(tf->den[0]); /* rough estimate for a1*s + a0 = 0 -> pole = -a0/a1 */
    }
}

/* ==========================================================================
 * L2 — FOPDT and SOPDT Convenience Functions
 * ========================================================================== */

void mimo_model_set_fopdt(MIMOModel *model, int i, int j,
                           double K, double tau, double theta) {
    if (!model) return;
    if (i < 0 || i >= model->num_outputs || j < 0 || j >= model->num_inputs) return;

    MIMOTransferFunction *tf = &model->rows[i].elements[j];

    /* G(s) = K * exp(-theta*s) / (tau*s + 1)
     * num = [K], den = [1, tau] (i.e., a0=1, a1=tau) */
    double num[] = { K };
    double den[] = { 1.0, tau };
    mimo_model_set_tf(model, i, j, num, 0, den, 1);
    tf->time_delay = theta;
    tf->time_constant = tau;
    tf->gain = K;
}

void mimo_model_set_sopdt(MIMOModel *model, int i, int j,
                           double K, double tau, double zeta, double theta) {
    if (!model) return;
    if (i < 0 || i >= model->num_outputs || j < 0 || j >= model->num_inputs) return;

    MIMOTransferFunction *tf = &model->rows[i].elements[j];

    /* G(s) = K * exp(-theta*s) / (tau^2*s^2 + 2*zeta*tau*s + 1)
     * num = [K], den = [1, 2*zeta*tau, tau^2] */
    double num[] = { K };
    double den[] = { 1.0, 2.0 * zeta * tau, tau * tau };
    mimo_model_set_tf(model, i, j, num, 0, den, 2);
    tf->time_delay = theta;
    tf->time_constant = tau;
    tf->gain = K;
}

/* ==========================================================================
 * L3 — Transfer Function Evaluation (Horner's Method)
 * ========================================================================== */

double complex mimo_tf_evaluate(const MIMOTransferFunction *tf, double complex s) {
    if (!tf) return 0.0;

    /* Evaluate numerator via Horner's method:
     * N(s) = b_0 + s*(b_1 + s*(b_2 + ... + s*b_n)) */
    double complex num_val = 0.0;
    for (int k = tf->num_order; k >= 0; k--) {
        num_val = num_val * s + tf->num[k];
    }

    /* Evaluate denominator via Horner's method:
     * D(s) = a_0 + s*(a_1 + s*(a_2 + ... + s*a_m)) */
    double complex den_val = 0.0;
    for (int k = tf->den_order; k >= 0; k--) {
        den_val = den_val * s + tf->den[k];
    }

    /* Avoid division by zero */
    if (cabs(den_val) < MIMO_EPSILON) {
        return INFINITY;
    }

    /* Incorporate pure time delay: exp(-theta * s) */
    double complex result = num_val / den_val;
    if (tf->time_delay > MIMO_EPSILON) {
        result *= cexp(-tf->time_delay * s);
    }

    return result;
}

/* ==========================================================================
 * L3 — Full Matrix Evaluation
 * ========================================================================== */

void mimo_model_evaluate(const MIMOModel *model, double complex s, double complex **G) {
    if (!model || !G) return;

    for (int i = 0; i < model->num_outputs; i++) {
        for (int j = 0; j < model->num_inputs; j++) {
            G[i][j] = mimo_tf_evaluate(&model->rows[i].elements[j], s);
        }
    }
}

/* ==========================================================================
 * L4 — Steady-State Gain Matrix K = G(0)
 * ========================================================================== */

void mimo_model_steady_state_gain(const MIMOModel *model, double *K) {
    if (!model || !K) return;

    /* K_{ij} = G_{ij}(0) — no need to evaluate full TF at s=0
     * Just use the gain field */
    for (int i = 0; i < model->num_outputs; i++) {
        for (int j = 0; j < model->num_inputs; j++) {
            K[i * model->num_inputs + j] = model->rows[i].elements[j].gain;
        }
    }
}

/* ==========================================================================
 * L3 — Transfer Function to State-Space Conversion
 * ========================================================================== */

/* Helper: convert single SISO TF to controllable canonical form.
 * Available for standalone use or embedding into larger state-space models. */
__attribute__((unused))
static void siso_tf_to_ss(const MIMOTransferFunction *tf,
                           double *A, double *B, double *C,
                           int *n) {
    int order = tf->den_order;
    *n = order;

    if (order == 0) {
        /* Pure gain, no states */
        *C = tf->gain;
        return;
    }

    /* Controllable canonical form:
     * A = [0       1       0      ...  0    ]
     *     [0       0       1      ...  0    ]
     *     [...                         ...  ]
     *     [-a_0  -a_1    -a_2    ... -a_{n-1}]
     *
     * B = [0, 0, ..., 0, 1]^T
     * C = [b_0, b_1, ..., b_{n-1}] (for strictly proper, D=0)
     *
     * Note: den is monic, so a_n = 1.
     * The denominator is: s^n + a_{n-1}*s^{n-1} + ... + a_1*s + a_0
     */

    /* Fill A matrix */
    for (int r = 0; r < order; r++) {
        for (int c = 0; c < order; c++) {
            if (c == r + 1) {
                A[r * order + c] = 1.0; /* super-diagonal ones */
            } else if (r == order - 1) {
                A[r * order + c] = -tf->den[c]; /* bottom row: -a_0, -a_1, ... */
            } else {
                A[r * order + c] = 0.0;
            }
        }
    }

    /* B = [0, 0, ..., 1]^T */
    for (int r = 0; r < order; r++) {
        B[r] = 0.0;
    }
    B[order - 1] = 1.0;

    /* C = [b_0, b_1, ..., b_{n-1}] — note: b_n contributes to D */
    for (int c = 0; c < order; c++) {
        if (c <= tf->num_order) {
            C[c] = tf->num[c];
        } else {
            C[c] = 0.0;
        }
    }
}

void mimo_model_to_state_space(const MIMOModel *model, MIMOStateSpace *ss) {
    if (!model || !ss) return;

    memset(ss, 0, sizeof(*ss));
    int p = model->num_outputs;
    int m = model->num_inputs;

    ss->n_states = 0;
    ss->n_inputs = m;
    ss->n_outputs = p;

    /* Build block-diagonal state-space:
     * Each SISO element adds its denominator order to total states.
     * The output y_i = sum_j C_{ij} * x_{ij} + D_{ij} * u_j
     *
     * For simplicity, create one big block-diagonal system.
     * Total states = sum of all (i,j) denominator orders.
     */

    int total_states = 0;

    for (int i = 0; i < p; i++) {
        for (int j = 0; j < m; j++) {
            const MIMOTransferFunction *tf = &model->rows[i].elements[j];
            int order = tf->den_order;
            total_states += order;
        }
    }

    if (total_states > MIMO_MAX_DIM) {
        /* Truncate to max dimension; use minimal realization instead */
        total_states = MIMO_MAX_DIM;
    }

    ss->n_states = total_states;

    /* Accumulate state contributions */
    int offset = 0;
    for (int i = 0; i < p; i++) {
        for (int j = 0; j < m; j++) {
            const MIMOTransferFunction *tf = &model->rows[i].elements[j];
            int order = tf->den_order;
            if (order == 0 || offset + order > total_states) continue;

            /* Place controllable canonical form at block (offset, offset) */
            for (int r = 0; r < order; r++) {
                /* A block */
                for (int c = 0; c < order; c++) {
                    if (c == r + 1) {
                        ss->A[offset + r][offset + c] = 1.0;
                    } else if (r == order - 1) {
                        ss->A[offset + r][offset + c] = -tf->den[c];
                    }
                }
                /* B block: j-th input column */
                if (j < m && r == order - 1) {
                    ss->B[offset + r][j] = 1.0;
                }
                /* C block: i-th output row */
                if (i < p) {
                    for (int c = 0; c < order; c++) {
                        if (c <= tf->num_order) {
                            ss->C[i][offset + c] += tf->num[c];
                        }
                    }
                }
            }
            /* D block */
            if (i < p && j < m && tf->num_order >= tf->den_order) {
                ss->D[i][j] += tf->num[tf->den_order]; /* relative degree */
            }
            offset += order;
        }
    }
}

/* ==========================================================================
 * L3 — Tustin's Bilinear Discretization
 * ========================================================================== */

/* Helper: solve linear system Ax = b by Gaussian elimination with partial pivoting */
static int gauss_solve(int n, double *A, double *b, double *x) {
    /* Allocate augmented matrix */
    double *aug = (double *)malloc(n * (n + 1) * sizeof(double));
    if (!aug) return -1;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i * (n + 1) + j] = A[i * n + j];
        }
        aug[i * (n + 1) + n] = b[i];
    }

    /* Forward elimination with partial pivoting */
    for (int k = 0; k < n; k++) {
        /* Find pivot */
        int max_row = k;
        double max_val = fabs(aug[k * (n + 1) + k]);
        for (int i = k + 1; i < n; i++) {
            double val = fabs(aug[i * (n + 1) + k]);
            if (val > max_val) {
                max_val = val;
                max_row = i;
            }
        }

        if (max_val < MIMO_EPSILON) {
            free(aug);
            return -1; /* singular */
        }

        /* Swap rows */
        if (max_row != k) {
            for (int j = k; j <= n; j++) {
                double tmp = aug[k * (n + 1) + j];
                aug[k * (n + 1) + j] = aug[max_row * (n + 1) + j];
                aug[max_row * (n + 1) + j] = tmp;
            }
        }

        /* Eliminate */
        for (int i = k + 1; i < n; i++) {
            double factor = aug[i * (n + 1) + k] / aug[k * (n + 1) + k];
            for (int j = k; j <= n; j++) {
                aug[i * (n + 1) + j] -= factor * aug[k * (n + 1) + j];
            }
        }
    }

    /* Back substitution */
    for (int i = n - 1; i >= 0; i--) {
        double sum = aug[i * (n + 1) + n];
        for (int j = i + 1; j < n; j++) {
            sum -= aug[i * (n + 1) + j] * x[j];
        }
        x[i] = sum / aug[i * (n + 1) + i];
    }

    free(aug);
    return 0;
}

/* Helper: matrix multiply C = A * B, all n×n row-major */
static void mat_mult(int n, const double *A, const double *B, double *C) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            double sum = 0.0;
            for (int k = 0; k < n; k++) {
                sum += A[i * n + k] * B[k * n + j];
            }
            C[i * n + j] = sum;
        }
    }
}

void mimo_ss_c2d_tustin(const MIMOStateSpace *continuous, MIMOStateSpace *discrete, double Ts) {
    if (!continuous || !discrete || Ts <= 0) return;

    memcpy(discrete, continuous, sizeof(*discrete));
    int n = continuous->n_states;

    if (n == 0) return;

    discrete->is_discrete = true;
    discrete->sample_time = Ts;

    double h2 = Ts / 2.0;

    /* Build M = I - A*h/2,  solve M * X = (I + A*h/2) for A_d */
    double *M = (double *)malloc(n * n * sizeof(double));
    double *I_plus_Ah2 = (double *)malloc(n * n * sizeof(double));
    double *M_inv = (double *)malloc(n * n * sizeof(double));
    double *A_d = (double *)malloc(n * n * sizeof(double));

    if (!M || !I_plus_Ah2 || !M_inv || !A_d) {
        free(M); free(I_plus_Ah2); free(M_inv); free(A_d);
        return;
    }

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            M[i * n + j] = ((i == j) ? 1.0 : 0.0) - continuous->A[i][j] * h2;
            I_plus_Ah2[i * n + j] = ((i == j) ? 1.0 : 0.0) + continuous->A[i][j] * h2;
        }
    }

    /* Compute M^{-1} column by column, then A_d = M^{-1} * (I + A*h/2) */
    double *M_inv_col = (double *)malloc(n * sizeof(double));
    double *b_vec = (double *)malloc(n * sizeof(double));
    double *x_vec = (double *)malloc(n * sizeof(double));

    if (!M_inv_col || !b_vec || !x_vec) {
        free(M_inv_col); free(b_vec); free(x_vec);
        free(M); free(I_plus_Ah2); free(M_inv); free(A_d);
        return;
    }

    for (int col = 0; col < n; col++) {
        for (int r = 0; r < n; r++) {
            b_vec[r] = (r == col) ? 1.0 : 0.0;
        }
        if (gauss_solve(n, M, b_vec, x_vec) == 0) {
            for (int r = 0; r < n; r++) {
                M_inv[r * n + col] = x_vec[r];
            }
        } else {
            /* Singular — set to identity */
            for (int r = 0; r < n; r++) {
                M_inv[r * n + col] = (r == col) ? 1.0 : 0.0;
            }
        }
    }

    /* A_d = M^{-1} * (I + A*h/2) */
    mat_mult(n, M_inv, I_plus_Ah2, A_d);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            discrete->A[i][j] = A_d[i * n + j];
        }
    }

    free(M); free(I_plus_Ah2); free(M_inv); free(A_d);
    free(M_inv_col); free(b_vec); free(x_vec);
}

/* ==========================================================================
 * L4 — Controllability and Observability (Kalman Rank Tests)
 * ========================================================================== */

bool mimo_ss_is_controllable(const MIMOStateSpace *ss) {
    if (!ss || ss->n_states == 0) return true; /* trivial system */

    int n = ss->n_states;
    int m = ss->n_inputs;

    /* Build controllability matrix C = [B, AB, A^2B, ..., A^{n-1}B]
     * C is n × (n*m). Check rank via column norm */
    double *Cmat = (double *)calloc(n * n * m, sizeof(double));
    if (!Cmat) return false;

    /* First block: B */
    for (int r = 0; r < n; r++) {
        for (int j = 0; j < m; j++) {
            Cmat[r * (n * m) + j] = ss->B[r][j];
        }
    }

    /* Compute successive powers */
    double *A_pow_B = (double *)calloc(n * m, sizeof(double));
    if (!A_pow_B) { free(Cmat); return false; }

    for (int k = 1; k < n; k++) {
        /* Compute A^k * B */
        for (int r = 0; r < n; r++) {
            for (int j = 0; j < m; j++) {
                double sum = 0.0;
                for (int l = 0; l < n; l++) {
                    sum += ss->A[r][l] * Cmat[l * (n * m) + ((k-1) * m + j)];
                }
                Cmat[r * (n * m) + (k * m + j)] = sum;
            }
        }
    }

    /* Check rank: compute sum of column norms, count non-negligible columns */
    int rank = 0;
    for (int col = 0; col < n * m; col++) {
        double norm = 0.0;
        for (int r = 0; r < n; r++) {
            norm += Cmat[r * (n * m) + col] * Cmat[r * (n * m) + col];
        }
        if (sqrt(norm) > MIMO_EPSILON) rank++;
    }

    free(Cmat);
    free(A_pow_B);

    return (rank >= n);
}

bool mimo_ss_is_observable(const MIMOStateSpace *ss) {
    if (!ss || ss->n_states == 0) return true;

    int n = ss->n_states;
    int p = ss->n_outputs;

    /* Build observability matrix O = [C; CA; CA^2; ...; CA^{n-1}]
     * O is (n*p) × n. Check row rank via QR-like norm check. */
    double *Omat = (double *)calloc(n * p * n, sizeof(double));
    if (!Omat) return false;

    /* First block: C */
    for (int i = 0; i < p; i++) {
        for (int c = 0; c < n; c++) {
            Omat[i * n + c] = ss->C[i][c];
        }
    }

    /* Successive blocks: C * A^k */
    for (int k = 1; k < n; k++) {
        for (int i = 0; i < p; i++) {
            for (int c = 0; c < n; c++) {
                double sum = 0.0;
                for (int l = 0; l < n; l++) {
                    sum += Omat[((k-1) * p + i) * n + l] * ss->A[l][c];
                }
                Omat[(k * p + i) * n + c] = sum;
            }
        }
    }

    /* Check row rank via column norms of O^T */
    int rank = 0;
    for (int col = 0; col < n; col++) {
        double norm = 0.0;
        for (int row = 0; row < n * p; row++) {
            norm += Omat[row * n + col] * Omat[row * n + col];
        }
        if (sqrt(norm) > MIMO_EPSILON) rank++;
    }

    free(Omat);
    return (rank >= n);
}

/* ==========================================================================
 * L4 — McMillan Degree
 * ========================================================================== */

int mimo_model_mcmillan_degree(const MIMOModel *model) {
    if (!model) return 0;

    int total_order = 0;
    for (int i = 0; i < model->num_outputs; i++) {
        for (int j = 0; j < model->num_inputs; j++) {
            total_order += model->rows[i].elements[j].den_order;
        }
    }

    /* The McMillan degree is the sum of denominator orders in the
     * Smith-McMillan form. For a matrix of rational functions, this
     * equals the minimal state dimension. Our block-diagonal form
     * is an upper bound; the true McMillan degree may be lower due
     * to pole-zero cancellations. */
    return total_order;
}

/* ==========================================================================
 * L2 — Model Printing
 * ========================================================================== */

void mimo_model_print(const MIMOModel *model) {
    if (!model) return;

    printf("=== MIMO Model: %s ===\n", model->name[0] ? model->name : "(unnamed)");
    printf("Outputs: %d, Inputs: %d, Sample Time: %.4f s\n",
           model->num_outputs, model->num_inputs, model->sample_time);

    for (int i = 0; i < model->num_outputs; i++) {
        printf("  Output y%d:\n", i + 1);
        for (int j = 0; j < model->num_inputs; j++) {
            const MIMOTransferFunction *tf = &model->rows[i].elements[j];
            printf("    G[%d,%d]: K=%.4f, tau=%.4f, theta=%.4f, num_ord=%d, den_ord=%d\n",
                   i, j, tf->gain, tf->time_constant, tf->time_delay,
                   tf->num_order, tf->den_order);
        }
    }
    printf("McMillan degree: %d\n", mimo_model_mcmillan_degree(model));
}
