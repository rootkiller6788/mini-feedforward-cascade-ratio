/**
 * @file mimo_static_decoupling.c
 * @brief Static (DC) decoupling design: D = K^{-1} and pseudoinverse variants.
 *
 * Knowledge:
 *   L2: Static decoupling concept - cancels steady-state interaction
 *   L4: Invertibility criterion (det(K) != 0), condition number analysis
 *   L5: LU decomposition with partial pivoting for matrix inversion
 *   L5: Moore-Penrose pseudoinverse via SVD for non-square plants
 *   L6: Sensitivity analysis to gain uncertainty
 *
 * References:
 *   - Luyben (1990), "Process Modeling, Simulation and Control", Ch.13
 *   - Skogestad & Postlethwaite (2005), Ch.10.8
 *   - Golub & Van Loan (2013), "Matrix Computations"
 */

#include "mimo_static_decoupling.h"
#include "mimo_model.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ==========================================================================
 * L5 — LU Decomposition + Linear Solve + Matrix Inverse
 * ========================================================================== */

/**
 * @brief LU decomposition with partial pivoting (Doolittle algorithm).
 * @param A   input n×n matrix (row-major), overwritten with LU
 * @param n   dimension
 * @param piv pivot vector (output, size n)
 * @return    0 on success, -1 if singular
 */
static int lu_decompose(double *A, int n, int *piv) {
    for (int i = 0; i < n; i++) piv[i] = i;
    for (int k = 0; k < n; k++) {
        int max_row = k;
        double max_val = fabs(A[k * n + k]);
        for (int r = k + 1; r < n; r++) {
            if (fabs(A[r * n + k]) > max_val) {
                max_val = fabs(A[r * n + k]);
                max_row = r;
            }
        }
        if (max_val < MIMO_EPSILON) return -1;
        if (max_row != k) {
            int tp = piv[k]; piv[k] = piv[max_row]; piv[max_row] = tp;
            for (int c = 0; c < n; c++) {
                double tmp = A[k * n + c];
                A[k * n + c] = A[max_row * n + c];
                A[max_row * n + c] = tmp;
            }
        }
        for (int r = k + 1; r < n; r++) {
            A[r * n + k] /= A[k * n + k];
            for (int c = k + 1; c < n; c++) {
                A[r * n + c] -= A[r * n + k] * A[k * n + c];
            }
        }
    }
    return 0;
}

/**
 * @brief Solve LU * x = b (b is permuted by piv).
 *        Forward + back substitution. O(n^2)
 */
static void lu_solve(const double *LU, int n, const int *piv,
                      const double *b, double *x) {
    /* Forward substitution: solve L*y = P*b */
    double *y = (double *)malloc(n * sizeof(double));
    for (int i = 0; i < n; i++) {
        double sum = b[piv[i]];
        for (int j = 0; j < i; j++)
            sum -= LU[i * n + j] * y[j];
        y[i] = sum;
    }
    /* Back substitution: solve U*x = y */
    for (int i = n - 1; i >= 0; i--) {
        double sum = y[i];
        for (int j = i + 1; j < n; j++)
            sum -= LU[i * n + j] * x[j];
        x[i] = sum / LU[i * n + i];
    }
    free(y);
}

/**
 * @brief Matrix inverse via LU decomposition.
 *        Solve A*A_inv = I column by column.
 */
static int matrix_inv_lu(const double *A, int n, double *A_inv) {
    double *LU = (double *)malloc(n * n * sizeof(double));
    int *piv = (int *)malloc(n * sizeof(int));
    double *b = (double *)calloc(n, sizeof(double));
    double *x = (double *)calloc(n, sizeof(double));
    if (!LU || !piv || !b || !x) {
        free(LU); free(piv); free(b); free(x); return -1;
    }
    memcpy(LU, A, n * n * sizeof(double));
    if (lu_decompose(LU, n, piv) != 0) {
        free(LU); free(piv); free(b); free(x); return -1;
    }
    for (int col = 0; col < n; col++) {
        memset(b, 0, n * sizeof(double));
        b[col] = 1.0;
        lu_solve(LU, n, piv, b, x);
        for (int r = 0; r < n; r++)
            A_inv[r * n + col] = x[r];
    }
    free(LU); free(piv); free(b); free(x);
    return 0;
}

/* ==========================================================================
 * L2/L5 — Static Decoupler Design: D = K^{-1}
 * ========================================================================== */

int mimo_static_decoupler_design(const MIMOModel *model, StaticDecoupler *sd) {
    if (!model || !sd) return -1;
    int n = model->num_outputs;
    if (n != model->num_inputs) return -2;
    if (n <= 0 || n > MIMO_MAX_DIM) return -1;

    memset(sd, 0, sizeof(StaticDecoupler));
    decoupler_init(&sd->base, n, n, DECOUPLER_STATIC);
    sd->is_square = true;
    sd->uses_pseudoinverse = false;
    sd->pseudoinv_tolerance = MIMO_EPSILON;

    /* Extract K = G(0) */
    double *K = (double *)calloc(n * n, sizeof(double));
    if (!K) return -1;
    mimo_model_steady_state_gain(model, K);

    /* Compute K^{-1} via LU into a flat temporary array */
    double *Kinv_flat = (double *)calloc(n * n, sizeof(double));
    if (!Kinv_flat) { free(K); return -1; }
    int ret = matrix_inv_lu(K, n, Kinv_flat);
    free(K);
    if (ret != 0) { free(Kinv_flat); return -1; }

    /* Copy K^{-1} from flat array into 2D sd->K_inv and decoupler base */
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            sd->K_inv[i][j] = Kinv_flat[i * n + j];
            sd->base.elements[i][j].gain = Kinv_flat[i * n + j];
            sd->base.elements[i][j].num[0] = Kinv_flat[i * n + j];
            sd->base.elements[i][j].den[0] = 1.0;
            sd->base.elements[i][j].num_order = 0;
            sd->base.elements[i][j].den_order = 0;
            sd->base.elements[i][j].is_active = true;
        }
    }
    free(Kinv_flat);

    /* Compute condition number of K via power iteration for sigma_max,
     * determinant estimate for sigma_min */
    {
        double *Kcpy = (double *)malloc(n * n * sizeof(double));
        mimo_model_steady_state_gain(model, Kcpy);

        /* Sigma_max via power iteration on K^T*K */
        double *v = (double *)calloc(n, sizeof(double));
        v[0] = 1.0;
        double sig_max_sq = 0.0;
        for (int it = 0; it < 30; it++) {
            double *w = (double *)calloc(n, sizeof(double));
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    for (int l = 0; l < n; l++)
                        w[i] += Kcpy[l*n+i] * Kcpy[l*n+j] * v[j];
            double nrm = 0.0;
            for (int i = 0; i < n; i++) nrm += w[i]*w[i];
            nrm = sqrt(nrm);
            if (nrm < MIMO_EPSILON) { free(w); break; }
            for (int i = 0; i < n; i++) v[i] = w[i] / nrm;
            free(w);
        }
        for (int i = 0; i < n; i++) {
            double dot = 0.0;
            for (int j = 0; j < n; j++) dot += Kcpy[i*n+j] * v[j];
            sig_max_sq += dot * dot; /* approx sigma_max^2 */
        }
        double sig_max = sqrt(sig_max_sq);

        /* Sigma_min via determinant estimate for small n */
        double *LUd = (double *)malloc(n*n*sizeof(double));
        memcpy(LUd, Kcpy, n*n*sizeof(double));
        double det = 1.0;
        int sgn = 1;
        for (int k = 0; k < n; k++) {
            int mr = k;
            double mv = fabs(LUd[k*n+k]);
            for (int r = k+1; r < n; r++)
                if (fabs(LUd[r*n+k]) > mv) { mv = fabs(LUd[r*n+k]); mr = r; }
            if (mv < MIMO_EPSILON) { det = 0.0; break; }
            if (mr != k) {
                for (int c = 0; c < n; c++) {
                    double t = LUd[k*n+c]; LUd[k*n+c] = LUd[mr*n+c]; LUd[mr*n+c] = t;
                }
                sgn = -sgn;
            }
            det *= LUd[k*n+k];
            for (int r = k+1; r < n; r++) {
                double f = LUd[r*n+k] / LUd[k*n+k];
                for (int c = k; c < n; c++) LUd[r*n+c] -= f * LUd[k*n+c];
            }
        }
        det *= sgn;
        free(LUd);

        double sig_min = fabs(det) / pow(sig_max, n > 1 ? n-1 : 1);
        if (sig_min < MIMO_EPSILON) sig_min = MIMO_EPSILON;
        sd->condition_number = sig_max / sig_min;

        /* Store singular values estimate */
        sd->sv[0] = sig_max;
        if (n > 1) sd->sv[n-1] = sig_min;

        free(Kcpy); free(v);
    }

    sd->base.condition = sd->condition_number;
    return 0;
}

/* ==========================================================================
 * L5 — Pseudoinverse for Non-Square Plants (More-Penrose via SVD)
 * ========================================================================== */

/**
 * @brief Two-sided Jacobi SVD for small matrices (n <= MIMO_MAX_DIM).
 *        Decomposes A = U * S * V^T where U, V orthogonal, S diagonal.
 *
 * Reference: Golub & Van Loan (2013), Algorithm 8.5.1 (Jacobi SVD)
 */
static int svd_jacobi(int m, int n, double *A, double *U, double *S, double *V) {
    int k = (m < n) ? m : n;

    /* Initialize V = I, U = empty */
    for (int i = 0; i < k; i++) {
        for (int j = 0; j < k; j++) {
            V[i*k + j] = (i == j) ? 1.0 : 0.0;
            U[i*k + j] = (i == j) ? 1.0 : 0.0;
        }
    }

    /* Copy A into working matrix (m x n), we work on k x k portion */
    double *B = (double *)calloc(k * k, sizeof(double));
    for (int i = 0; i < k; i++)
        for (int j = 0; j < k; j++)
            B[i*k + j] = A[i*n + j]; /* use first k columns */

    /* Jacobi iteration: sweep until off-diagonal < tolerance */
    for (int sweep = 0; sweep < 20; sweep++) {
        double off_norm = 0.0;
        for (int i = 0; i < k; i++)
            for (int j = 0; j < k; j++)
                if (i != j) off_norm += B[i*k + j] * B[i*k + j];

        if (sqrt(off_norm) < 1e-10) break;

        for (int p = 0; p < k - 1; p++) {
            for (int q = p + 1; q < k; q++) {
                double app = B[p*k+p], aqq = B[q*k+q];
                double apq = B[p*k+q], aqp = B[q*k+p];

                /* Compute Jacobi rotation to zero out (p,q) and (q,p) */
                double theta = (aqq - app) / (2.0 * apq);
                double t = 1.0 / (fabs(theta) + sqrt(theta*theta + 1.0));
                if (theta < 0) t = -t;
                double c_val = 1.0 / sqrt(1.0 + t*t);
                double s_val = c_val * t;

                /* Apply rotation: B = J^T * B * J */
                for (int i = 0; i < k; i++) {
                    double bip = B[i*k+p], biq = B[i*k+q];
                    B[i*k+p] = c_val*bip - s_val*biq;
                    B[i*k+q] = s_val*bip + c_val*biq;
                }
                for (int j = 0; j < k; j++) {
                    double bpj = B[p*k+j], bqj = B[q*k+j];
                    B[p*k+j] = c_val*bpj - s_val*bqj;
                    B[q*k+j] = s_val*bpj + c_val*bqj;
                }

                /* Accumulate V */
                for (int i = 0; i < k; i++) {
                    double vip = V[i*k+p], viq = V[i*k+q];
                    V[i*k+p] = c_val*vip - s_val*viq;
                    V[i*k+q] = s_val*vip + c_val*viq;
                }
                /* Accumulate U similarly */
                for (int i = 0; i < k; i++) {
                    double uip = U[i*k+p], uiq = U[i*k+q];
                    U[i*k+p] = c_val*uip - s_val*uiq;
                    U[i*k+q] = s_val*uip + c_val*uiq;
                }
            }
        }
    }

    /* Extract singular values (diagonal = sorted by magnitude) */
    for (int i = 0; i < k; i++) {
        S[i] = B[i*k + i];
        if (S[i] < 0) {
            S[i] = -S[i];
            for (int r = 0; r < k; r++) V[r*k + i] = -V[r*k + i];
        }
    }

    /* Sort by descending */
    for (int i = 0; i < k - 1; i++) {
        int best = i;
        for (int j = i + 1; j < k; j++)
            if (S[j] > S[best]) best = j;
        if (best != i) {
            double tmp = S[i]; S[i] = S[best]; S[best] = tmp;
            for (int r = 0; r < k; r++) {
                double tv = V[r*k+i]; V[r*k+i] = V[r*k+best]; V[r*k+best] = tv;
                double tu = U[r*k+i]; U[r*k+i] = U[r*k+best]; U[r*k+best] = tu;
            }
        }
    }

    free(B);
    return 0;
}

int mimo_static_decoupler_pseudoinv(const MIMOModel *model, StaticDecoupler *sd,
                                     double tolerance) {
    if (!model || !sd) return -1;
    int p = model->num_outputs;
    int m = model->num_inputs;
    if (p <= 0 || m <= 0 || p > MIMO_MAX_DIM || m > MIMO_MAX_DIM) return -1;

    memset(sd, 0, sizeof(StaticDecoupler));
    sd->is_square = (p == m);
    sd->uses_pseudoinverse = true;
    sd->pseudoinv_tolerance = tolerance;

    /* Extract K */
    double *K = (double *)calloc(p * m, sizeof(double));
    mimo_model_steady_state_gain(model, K);

    int k = (p < m) ? p : m;
    double *U = (double *)calloc(k * k, sizeof(double));
    double *S = (double *)calloc(k, sizeof(double));
    double *V = (double *)calloc(k * k, sizeof(double));

    svd_jacobi(p, m, K, U, S, V);

    /* Compute pseudoinverse: K^+ = V * S^+ * U^T
     * S^+ = diag(1/s_1, ..., 1/s_r, 0, ..., 0) where s_i > tolerance */
    double *Sp = (double *)calloc(k * k, sizeof(double));
    for (int i = 0; i < k; i++) {
        if (S[i] > tolerance) {
            Sp[i*k + i] = 1.0 / S[i];
        }
    }

    /* K_inv (m x p) = V (m x k) * S^+ (k x k) * U^T (k x p) */
    for (int i = 0; i < m && i < MIMO_MAX_DIM; i++) {
        for (int j = 0; j < p && j < MIMO_MAX_DIM; j++) {
            double sum = 0.0;
            for (int r = 0; r < k; r++) {
                double vs = 0.0;
                for (int c = 0; c < k; c++) {
                    vs += V[i*k + c] * Sp[c*k + r];
                }
                sum += vs * U[j*k + r];
            }
            sd->K_inv[i][j] = sum;
            sd->base.elements[i][j].gain = sum;
            sd->base.elements[i][j].num[0] = sum;
            sd->base.elements[i][j].den[0] = 1.0;
            sd->base.elements[i][j].is_active = true;
        }
    }

    sd->condition_number = (S[k-1] > tolerance) ? S[0] / S[k-1] : INFINITY;
    for (int i = 0; i < k; i++) sd->sv[i] = S[i];

    free(K); free(U); free(S); free(V); free(Sp);
    decoupler_init(&sd->base, m, m, DECOUPLER_SVD);
    return 0;
}

/* ==========================================================================
 * L2 — Apply Static Decoupler in Real-Time
 * ========================================================================== */

void mimo_static_decoupler_apply(const StaticDecoupler *sd,
                                  const double *u_controller,
                                  double *u_process) {
    if (!sd || !u_controller || !u_process) return;
    int m = sd->base.n_inputs;
    for (int i = 0; i < m; i++) {
        double sum = 0.0;
        for (int j = 0; j < m; j++) {
            sum += sd->K_inv[i][j] * u_controller[j];
        }
        u_process[i] = sum;
    }
}

/* ==========================================================================
 * L4 — Apparent Gain: K_a = K * D
 * ========================================================================== */

void mimo_static_apparent_gain(const MIMOModel *model, const StaticDecoupler *sd,
                                double *Ka) {
    if (!model || !sd || !Ka) return;
    int n = model->num_outputs;
    double *Kmat = (double *)calloc(n * n, sizeof(double));
    mimo_model_steady_state_gain(model, Kmat);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            double sum = 0.0;
            for (int k = 0; k < n; k++)
                sum += Kmat[i*n + k] * sd->K_inv[k][j];
            Ka[i*n + j] = sum;
        }
    free(Kmat);
}

/* ==========================================================================
 * L6 — Sensitivity to Gain Uncertainty
 * ========================================================================== */

double mimo_static_sensitivity(const MIMOModel *model, const StaticDecoupler *sd,
                                double rel_error) {
    if (!model || !sd) return 0.0;
    int n = model->num_outputs;

    double *K = (double *)calloc(n * n, sizeof(double));
    mimo_model_steady_state_gain(model, K);

    /* Worst-case: perturb K by +/- rel_error, compute residual norm */
    double worst_norm = 0.0;

    /* For each element (i,j), try +rel and -rel */
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            double orig = K[i*n + j];
            /* Perturb K[i][j] */
            for (int sign = -1; sign <= 1; sign += 2) {
                K[i*n + j] = orig * (1.0 + sign * rel_error);

                /* Compute residual: K_perturbed * K_inv - I */
                double frob2 = 0.0;
                for (int r = 0; r < n; r++) {
                    for (int c = 0; c < n; c++) {
                        double val = 0.0;
                        for (int l = 0; l < n; l++)
                            val += K[r*n + l] * sd->K_inv[l][c];
                        val -= (r == c) ? 1.0 : 0.0;
                        frob2 += val * val;
                    }
                }
                double frob = sqrt(frob2);
                if (frob > worst_norm) worst_norm = frob;

                K[i*n + j] = orig; /* restore */
            }
        }
    }

    free(K);
    return worst_norm;
}
