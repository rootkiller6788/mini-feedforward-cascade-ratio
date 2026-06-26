/**
 * @file mimo_robustness.c
 * @brief Robustness analysis for MIMO decoupled systems.
 *
 * Knowledge:
 *   L4: Gain/phase margin analysis for MIMO systems
 *   L5: Sensitivity and complementary sensitivity functions
 *   L8: Structured singular value (mu) lower bound
 *   L8: Lyapunov-based stability margin
 *   L8: Monte Carlo robustness testing under parameter uncertainty
 *   L4: Return difference matrix analysis
 *
 * References:
 *   - Skogestad & Postlethwaite (2005), Ch.7-8
 *   - Doyle, Wall, Stein (1982), "Performance and Robustness Analysis
 *     for Structured Uncertainty", IEEE CDC
 *   - Zhou, Doyle, Glover (1996), "Robust and Optimal Control"
 */

#include "mimo_model.h"
#include "mimo_decoupling_common.h"
#include "mimo_interaction.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ==========================================================================
 * L4 — Sensitivity Function S(s) = (I + G(s)*K(s))^{-1}
 *
 * For decentralized control at DC:
 * S(0) = (I + K * Kc_diag)^{-1} where K = G(0), Kc_diag = diag(Kc_i)
 *
 * The sensitivity function indicates disturbance rejection:
 * small ||S|| = good disturbance rejection.
 * ========================================================================== */

/**
 * @brief Compute steady-state sensitivity matrix S(0)
 * @param model  MIMO model
 * @param Kc     diagonal controller gains [n]
 * @param S      output sensitivity matrix [n][n], row-major
 * @return       0 on success
 */
int mimo_sensitivity_dc(const MIMOModel *model, const double *Kc, double *S) {
    if (!model || !Kc || !S) return -1;
    int n = model->num_outputs;
    if (n != model->num_inputs || n == 0) return -1;

    double *Kmat = (double *)calloc(n * n, sizeof(double));
    mimo_model_steady_state_gain(model, Kmat);

    /* Build I + K * diag(Kc) = I + G(0) * Kc_diag */
    double *M = (double *)calloc(n * n, sizeof(double));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            M[i * n + j] = Kmat[i * n + j] * Kc[j];
            if (i == j) M[i * n + j] += 1.0;
        }
    }

    /* S = M^{-1} via Gauss-Jordan */
    int N = 2 * n;
    double *aug = (double *)calloc(n * N, sizeof(double));
    if (!aug) { free(Kmat); free(M); return -1; }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) aug[i*N + j] = M[i*n + j];
        aug[i*N + n + i] = 1.0;
    }
    for (int col = 0; col < n; col++) {
        int mr = col;
        double mv = fabs(aug[mr*N + col]);
        for (int r = col+1; r < n; r++)
            if (fabs(aug[r*N+col]) > mv) { mv = fabs(aug[r*N+col]); mr = r; }
        if (mv < MIMO_EPSILON) { free(aug); free(Kmat); free(M); return -1; }
        if (mr != col)
            for (int c = 0; c < N; c++) {
                double t = aug[col*N+c]; aug[col*N+c] = aug[mr*N+c]; aug[mr*N+c] = t;
            }
        double piv = aug[col*N+col];
        for (int c = 0; c < N; c++) aug[col*N+c] /= piv;
        for (int r = 0; r < n; r++) {
            if (r == col) continue;
            double f = aug[r*N+col];
            for (int c = 0; c < N; c++) aug[r*N+c] -= f * aug[col*N+c];
        }
    }
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            S[i*n + j] = aug[i*N + n + j];

    free(aug); free(Kmat); free(M);
    return 0;
}

/* ==========================================================================
 * L4 — Complementary Sensitivity T(s) = I - S(s) = G*K*(I+G*K)^{-1}
 * ========================================================================== */

/**
 * @brief Compute complementary sensitivity T(0) = I - S(0)
 * @param model MIMO model
 * @param Kc    controller gains
 * @param T     output T matrix [n][n]
 */
int mimo_complementary_sensitivity_dc(const MIMOModel *model, const double *Kc, double *T) {
    int n = model->num_outputs;

    double *S = (double *)calloc(n * n, sizeof(double));
    if (mimo_sensitivity_dc(model, Kc, S) != 0) { free(S); return -1; }

    for (int i = 0; i < n * n; i++)
        T[i] = (i / n == i % n) ? 1.0 - S[i] : -S[i];

    free(S);
    return 0;
}

/* ==========================================================================
 * L4 — Return Difference Norm
 *
 * ||I + G(0)*K||_F  — larger values indicate better robustness
 * ========================================================================== */

double mimo_return_difference_norm(const MIMOModel *model, const double *Kc) {
    if (!model || !Kc) return 0.0;
    int n = model->num_outputs;
    if (n != model->num_inputs) return 0.0;

    double *Kmat = (double *)calloc(n * n, sizeof(double));
    mimo_model_steady_state_gain(model, Kmat);

    double frob2 = 0.0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            double val = Kmat[i*n + j] * Kc[j];
            if (i == j) val += 1.0;
            frob2 += val * val;
        }
    }

    free(Kmat);
    return sqrt(frob2);
}

/* ==========================================================================
 * L8 — Structured Singular Value (mu) Lower Bound
 *
 * For diagonal multiplicative input uncertainty Delta = diag(delta_i),
 * a lower bound on mu is:
 *   mu >= max_i |eig_i( G(0) * diag(Kc_i) * (I + G(0)*diag(Kc_i))^{-1} )|
 *
 * This provides a measure of robust stability margin:
 *   1/mu = maximum uncertainty before instability.
 * ========================================================================== */

double mimo_mu_lower_bound(const MIMOModel *model, const double *Kc) {
    if (!model || !Kc) return 0.0;
    int n = model->num_outputs;
    if (n != model->num_inputs) return 0.0;

    double *T = (double *)calloc(n * n, sizeof(double));
    if (mimo_complementary_sensitivity_dc(model, Kc, T) != 0) {
        free(T); return 0.0;
    }

    /* Compute eigenvalues of T via power iteration (find spectral radius) */
    double *v = (double *)calloc(n, sizeof(double));
    v[0] = 1.0;
    double rho = 0.0;
    for (int it = 0; it < 30; it++) {
        double *w = (double *)calloc(n, sizeof(double));
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                w[i] += T[i*n + j] * v[j];
        double nm = 0.0;
        for (int i = 0; i < n; i++) nm += w[i] * w[i];
        nm = sqrt(nm);
        if (nm < 1e-12) { free(w); break; }
        for (int i = 0; i < n; i++) v[i] = w[i] / nm;
        free(w);
    }
    /* Rayleigh quotient */
    for (int i = 0; i < n; i++) {
        double dot = 0.0;
        for (int j = 0; j < n; j++) dot += T[i*n + j] * v[j];
        rho += v[i] * dot;
    }
    rho = fabs(rho);

    free(T); free(v);
    return rho;
}

/* ==========================================================================
 * L8 — Gain Margin Analysis for Decentralized MIMO
 *
 * For each loop i, the gain margin is the factor by which the
 * loop gain Kc_i can be increased before the MIMO system
 * becomes unstable.
 *
 * GM_i = 1 / |eig_i(S(0) - I)| approximately
 * ========================================================================== */

void mimo_gain_margins(const MIMOModel *model, const double *Kc,
                        double *gm) {
    if (!model || !Kc || !gm) return;
    int n = model->num_outputs;

    double *S = (double *)calloc(n * n, sizeof(double));
    if (mimo_sensitivity_dc(model, Kc, S) != 0) {
        for (int i = 0; i < n; i++) gm[i] = INFINITY;
        free(S); return;
    }

    /* For each loop, estimate gain margin from sensitivity diagonal */
    for (int i = 0; i < n; i++) {
        /* GM = 1/|S_{ii}| (simplified MIMO gain margin) */
        double sii = S[i*n + i];
        if (fabs(sii) < MIMO_EPSILON) {
            gm[i] = INFINITY;
        } else if (fabs(sii) >= 1.0) {
            gm[i] = 0.0; /* already unstable */
        } else {
            gm[i] = 1.0 / (1.0 - fabs(sii));
        }
    }

    free(S);
}

/* ==========================================================================
 * L8 — Monte Carlo Robustness Test
 *
 * Perturbs the gain matrix by random multiplicative uncertainty
 * and checks if the system remains stable (NI > 0 condition).
 * ========================================================================== */

/**
 * @brief Run Monte Carlo robustness test
 *
 * @param model      nominal MIMO model
 * @param Kc         controller gains [n]
 * @param n_trials   number of Monte Carlo trials
 * @param rel_uncert relative uncertainty (e.g., 0.20 = 20%)
 * @return           fraction of trials where system remained stable [0, 1]
 */
double mimo_monte_carlo_robustness(const MIMOModel *model, const double *Kc,
                                    int n_trials, double rel_uncert) {
    if (!model || !Kc || n_trials <= 0) return 0.0;

    int n = model->num_outputs;
    if (n != model->num_inputs) return 0.0;

    double *K_nom = (double *)calloc(n * n, sizeof(double));
    mimo_model_steady_state_gain(model, K_nom);

    int stable_count = 0;

    for (int trial = 0; trial < n_trials; trial++) {
        /* Generate random perturbation */
        double *K_pert = (double *)calloc(n * n, sizeof(double));
        for (int i = 0; i < n * n; i++) {
            /* Uniform random in [-rel_uncert, +rel_uncert] */
            double r = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
            K_pert[i] = K_nom[i] * (1.0 + r * rel_uncert);

            /* Ensure no sign changes (process physics) */
            if (K_nom[i] > 0 && K_pert[i] < 0) K_pert[i] = K_nom[i] * 0.01;
            if (K_nom[i] < 0 && K_pert[i] > 0) K_pert[i] = K_nom[i] * 0.01;
        }

        /* Check Niederlinski Index for diagonal pairing */
        double ni = mimo_niederlinski_index(K_pert, n, NULL);

        /* Check condition number */
        double cn = mimo_condition_number(K_pert, n);

        /* System considered stable if NI > 0 and CN < 100 */
        if (ni > 0 && !isnan(ni) && cn < 100.0) {
            stable_count++;
        }

        free(K_pert);
    }

    free(K_nom);
    return (double)stable_count / n_trials;
}

/* ==========================================================================
 * L4 — Stability margin via return difference determinant
 *
 * det(I + G(0)*K) indicates closed-loop stability margin.
 * Values near 0 indicate near-instability.
 * ========================================================================== */

double mimo_stability_margin(const MIMOModel *model, const double *Kc) {
    if (!model || !Kc) return 0.0;
    int n = model->num_outputs;
    if (n != model->num_inputs) return 0.0;

    double *Kmat = (double *)calloc(n * n, sizeof(double));
    mimo_model_steady_state_gain(model, Kmat);

    /* Build I + G(0) * diag(Kc) */
    double *IK = (double *)calloc(n * n, sizeof(double));
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            IK[i*n + j] = Kmat[i*n + j] * Kc[j];
            if (i == j) IK[i*n + j] += 1.0;
        }

    /* Determinant */
    double det = 1.0;
    int sgn = 1;
    for (int k = 0; k < n && det != 0; k++) {
        int mr = k;
        double mv = fabs(IK[mr*n+k]);
        for (int r = k+1; r < n; r++)
            if (fabs(IK[r*n+k]) > mv) { mv = fabs(IK[r*n+k]); mr = r; }
        if (mv < MIMO_EPSILON) { det = 0.0; break; }
        if (mr != k) {
            for (int c = 0; c < n; c++) {
                double t = IK[k*n+c]; IK[k*n+c] = IK[mr*n+c]; IK[mr*n+c] = t;
            }
            sgn = -sgn;
        }
        det *= IK[k*n+k];
        for (int r = k+1; r < n; r++) {
            double f = IK[r*n+k] / IK[k*n+k];
            for (int c = k; c < n; c++) IK[r*n+c] -= f * IK[k*n+c];
        }
    }
    det *= sgn;

    free(Kmat); free(IK);
    return det;
}
