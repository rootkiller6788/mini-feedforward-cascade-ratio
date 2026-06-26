/**
 * @file mimo_interaction.c
 * @brief MIMO interaction analysis: RGA, Niederlinski Index, condition number,
 *        dynamic RGA, pairing enumeration, integrity, effective RGA.
 *
 * Knowledge:
 *   L4: RGA computation (Bristol 1966 theorem)
 *   L4: Niederlinski Index (Niederlinski 1971 stability theorem)
 *   L5: Condition number via power iteration
 *   L5: Dynamic RGA at frequency (Witcher & McAvoy 1977)
 *   L5: Optimal pairing via RGA rules (Bristol + Seborg)
 *   L4: Integrity checking (Grosdidier, Morari, Holt 1985)
 *   L5: Effective RGA (Xiong et al. 2005)
 *
 * References:
 *   - Bristol (1966), IEEE TAC-11(1):133-134
 *   - Niederlinski (1971), Automatica 7(6):691-701
 *   - Skogestad & Postlethwaite (2005), Ch.10
 */

#include "mimo_interaction.h"
#include "mimo_model.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>

/* ==========================================================================
 * Helper: Matrix inverse via Gauss-Jordan with partial pivoting
 * ========================================================================== */

static int mat_inv(const double *A, int n, double *Ainv) {
    if (!A || !Ainv || n <= 0 || n > MIMO_MAX_DIM) return -1;
    int N = 2 * n;
    double *aug = (double *)calloc(n * N, sizeof(double));
    if (!aug) return -1;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) aug[i*N + j] = A[i*n + j];
        aug[i*N + n + i] = 1.0;
    }
    for (int col = 0; col < n; col++) {
        int mr = col;
        double mv = fabs(aug[mr*N + col]);
        for (int r = col+1; r < n; r++)
            if (fabs(aug[r*N+col]) > mv) { mv = fabs(aug[r*N+col]); mr = r; }
        if (mv < MIMO_EPSILON) { free(aug); return -1; }
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
            Ainv[i*n+j] = aug[i*N + n + j];
    free(aug);
    return 0;
}

/* ==========================================================================
 * L4 — RGA Computation (Bristol, 1966)
 *
 * RGA = K .* (K^{-1})^T  (Hadamard product)
 *
 * Properties: each row/col sums to 1; lambda_ij near 1 = good pairing;
 * lambda_ij < 0 = integrity violation; lambda_ij >> 1 = severe interaction.
 * ========================================================================== */

int mimo_rga_compute(const double *K, int n, RGAMatrix *rga) {
    if (!K || !rga || n <= 0 || n > MIMO_MAX_DIM) return -1;
    memset(rga, 0, sizeof(RGAMatrix));
    rga->n = n;

    double *Kinv = (double *)calloc(n * n, sizeof(double));
    if (!Kinv) return -1;
    if (mat_inv(K, n, Kinv) != 0) { free(Kinv); return -1; }

    /* RGA_{ij} = K_{ij} * (K^{-1})_{ji} */
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            rga->rga[i][j] = K[i*n + j] * Kinv[j*n + i];

    /* Condition number via power iteration */
    double *v = (double *)calloc(n, sizeof(double));
    v[0] = 1.0;
    double sig_max_sq = 0.0;
    for (int it = 0; it < 30; it++) {
        double *w = (double *)calloc(n, sizeof(double));
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                for (int l = 0; l < n; l++)
                    w[i] += K[l*n+i] * K[l*n+j] * v[j];
        double nm = 0.0;
        for (int i = 0; i < n; i++) nm += w[i]*w[i];
        nm = sqrt(nm);
        if (nm < MIMO_EPSILON) { free(w); break; }
        double rq = 0.0;
        for (int i = 0; i < n; i++) { v[i] = w[i]/nm; rq += v[i]*w[i]; }
        sig_max_sq = rq / nm;
        free(w);
    }
    double sig_max = sqrt(sig_max_sq);
    free(v);

    /* Determinant estimate for sigma_min */
    double *LU = (double *)malloc(n*n*sizeof(double));
    memcpy(LU, K, n*n*sizeof(double));
    double det = 1.0;
    int sgn = 1;
    for (int k = 0; k < n; k++) {
        int mr = k;
        double mv = fabs(LU[mr*n+k]);
        for (int r = k+1; r < n; r++)
            if (fabs(LU[r*n+k]) > mv) { mv = fabs(LU[r*n+k]); mr = r; }
        if (mv < MIMO_EPSILON) { det = 0.0; break; }
        if (mr != k) {
            for (int c = 0; c < n; c++) {
                double t = LU[k*n+c]; LU[k*n+c] = LU[mr*n+c]; LU[mr*n+c] = t;
            }
            sgn = -sgn;
        }
        det *= LU[k*n+k];
        for (int r = k+1; r < n; r++) {
            double f = LU[r*n+k] / LU[k*n+k];
            for (int c = k; c < n; c++) LU[r*n+c] -= f * LU[k*n+c];
        }
    }
    det *= sgn;
    free(LU);
    double sig_min = fabs(det) / pow(sig_max, n > 1 ? n-1 : 1);
    if (sig_min < MIMO_EPSILON) sig_min = MIMO_EPSILON;
    rga->cn = sig_max / sig_min;

    /* NI and interpretation */
    rga->ni = mimo_niederlinski_index(K, n, NULL);

    double max_rga = 0.0, min_rga = INFINITY;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            if (rga->rga[i][j] > max_rga) max_rga = rga->rga[i][j];
            if (rga->rga[i][j] < min_rga) min_rga = rga->rga[i][j];
        }
    if (min_rga < 0)
        snprintf(rga->interpretation, sizeof(rga->interpretation),
                 "NEGATIVE RGA: integrity violation. Range [%.3f,%.3f] CN=%.1f",
                 min_rga, max_rga, rga->cn);
    else if (max_rga > 5.0)
        snprintf(rga->interpretation, sizeof(rga->interpretation),
                 "STRONG interaction (max=%.2f). Decoupling needed. CN=%.1f",
                 max_rga, rga->cn);
    else if (max_rga < 2.0)
        snprintf(rga->interpretation, sizeof(rga->interpretation),
                 "WEAK interaction (max=%.2f). Decentralized OK. CN=%.1f",
                 max_rga, rga->cn);
    else
        snprintf(rga->interpretation, sizeof(rga->interpretation),
                 "MODERATE interaction (max=%.2f). Consider decoupling. CN=%.1f",
                 max_rga, rga->cn);

    free(Kinv);
    return 0;
}

/* ==========================================================================
 * L4 — Niederlinski Index (Niederlinski, 1971)
 * NI = det(K) / prod_i K_{i,pairing[i]}
 * NI < 0 => unstable with integral action on diagonal pairing.
 * ========================================================================== */

double mimo_niederlinski_index(const double *K, int n, const int *pairing) {
    if (!K || n <= 0 || n > MIMO_MAX_DIM) return NAN;
    double *LU = (double *)malloc(n*n*sizeof(double));
    memcpy(LU, K, n*n*sizeof(double));
    double det = 1.0;
    int sgn = 1;
    for (int k = 0; k < n; k++) {
        int mr = k;
        double mv = fabs(LU[mr*n+k]);
        for (int r = k+1; r < n; r++)
            if (fabs(LU[r*n+k]) > mv) { mv = fabs(LU[r*n+k]); mr = r; }
        if (mv < MIMO_EPSILON) { free(LU); return NAN; }
        if (mr != k) {
            for (int c = 0; c < n; c++) {
                double t = LU[k*n+c]; LU[k*n+c] = LU[mr*n+c]; LU[mr*n+c] = t;
            }
            sgn = -sgn;
        }
        det *= LU[k*n+k];
        for (int r = k+1; r < n; r++) {
            double f = LU[r*n+k] / LU[k*n+k];
            for (int c = k; c < n; c++) LU[r*n+c] -= f * LU[k*n+c];
        }
    }
    det *= sgn;
    free(LU);

    double prod = 1.0;
    for (int i = 0; i < n; i++) {
        int j = pairing ? pairing[i] : i;
        prod *= K[i*n + j];
    }
    if (fabs(prod) < MIMO_EPSILON) return NAN;
    return det / prod;
}

/* ==========================================================================
 * L5 — Condition Number via Power Iteration
 * kappa(K) = sigma_max / sigma_min
 * ========================================================================== */

double mimo_condition_number(const double *K, int n) {
    if (!K || n <= 0) return 0.0;
    double *v = (double *)calloc(n, sizeof(double));
    v[0] = 1.0;
    double sig2 = 0.0;
    for (int it = 0; it < 30; it++) {
        double *w = (double *)calloc(n, sizeof(double));
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                for (int l = 0; l < n; l++)
                    w[i] += K[l*n+i] * K[l*n+j] * v[j];
        double nm = 0.0;
        for (int i = 0; i < n; i++) nm += w[i]*w[i];
        nm = sqrt(nm);
        if (nm < 1e-15) { free(w); break; }
        double rq = 0.0;
        for (int i = 0; i < n; i++) { v[i] = w[i]/nm; rq += v[i]*w[i]; }
        sig2 = rq / nm;
        free(w);
    }
    double smax = sqrt(sig2);
    free(v);
    if (smax < 1e-15) return INFINITY;

    /* sigma_min from determinant (n <= 3) */
    if (n <= 3) {
        double *LU = (double *)malloc(n*n*sizeof(double));
        memcpy(LU, K, n*n*sizeof(double));
        double det = 1.0;
        for (int k = 0; k < n && det != 0; k++) {
            int mr = k;
            double mv = fabs(LU[mr*n+k]);
            for (int r = k+1; r < n; r++)
                if (fabs(LU[r*n+k]) > mv) { mv = fabs(LU[r*n+k]); mr = r; }
            if (mv < 1e-15) { det = 0.0; break; }
            if (mr != k)
                for (int c = 0; c < n; c++) {
                    double t = LU[k*n+c]; LU[k*n+c] = LU[mr*n+c]; LU[mr*n+c] = t;
                }
            det *= LU[k*n+k];
            for (int r = k+1; r < n; r++) {
                double f = LU[r*n+k] / LU[k*n+k];
                for (int c = k; c < n; c++) LU[r*n+c] -= f*LU[k*n+c];
            }
        }
        free(LU);
        double smin = fabs(det)/pow(smax, n-1);
        if (smin > 1e-15) return smax/smin;
    }
    return smax / 1e-10; /* rough bound */
}

/* ==========================================================================
 * L5 — Dynamic RGA at frequency omega
 * ========================================================================== */

int mimo_dynamic_rga(const MIMOModel *model, double omega, DynamicRGA *drga) {
    if (!model || !drga) return -1;
    int n = model->num_outputs;
    if (n != model->num_inputs || n == 0) return -1;
    memset(drga, 0, sizeof(DynamicRGA));
    drga->n = n;
    drga->frequency = omega;

    /* Evaluate G(j*omega) */
    double complex s = omega * I;
    double complex *Gd = (double complex *)calloc(n*n, sizeof(double complex));
    double complex **G = (double complex **)malloc(n*sizeof(double complex*));
    if (!Gd || !G) { free(Gd); free(G); return -1; }
    for (int i = 0; i < n; i++) G[i] = &Gd[i*n];
    mimo_model_evaluate(model, s, G);

    /* Build 2n x 2n real system for complex matrix inversion */
    int n2 = 2*n;
    double *aug = (double *)calloc(n2 * (2*n2), sizeof(double));
    if (!aug) { free(Gd); free(G); return -1; }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i*(2*n2) + j] = creal(G[i][j]);
            aug[i*(2*n2) + n + j] = -cimag(G[i][j]);
            aug[n+i*(2*n2) + j] = cimag(G[i][j]);
            aug[n+i*(2*n2) + n + j] = creal(G[i][j]);
        }
        aug[i*(2*n2) + 2*n + i] = 1.0;
        aug[n+i*(2*n2) + 2*n + n + i] = 1.0;
    }

    /* Gauss-Jordan */
    for (int col = 0; col < n2; col++) {
        int mr = col;
        double mv = fabs(aug[mr*(2*n2)+col]);
        for (int r = col+1; r < n2; r++)
            if (fabs(aug[r*(2*n2)+col]) > mv) { mv = fabs(aug[r*(2*n2)+col]); mr = r; }
        if (mv < MIMO_EPSILON) { free(aug); free(Gd); free(G); return -1; }
        if (mr != col)
            for (int c = 0; c < 2*n2; c++) {
                double t = aug[col*(2*n2)+c]; aug[col*(2*n2)+c] = aug[mr*(2*n2)+c];
                aug[mr*(2*n2)+c] = t;
            }
        double piv = aug[col*(2*n2)+col];
        for (int c = 0; c < 2*n2; c++) aug[col*(2*n2)+c] /= piv;
        for (int r = 0; r < n2; r++) {
            if (r == col) continue;
            double f = aug[r*(2*n2)+col];
            for (int c = 0; c < 2*n2; c++) aug[r*(2*n2)+c] -= f * aug[col*(2*n2)+c];
        }
    }

    /* Extract DRGA: G .* (G^{-1})^T */
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            /* (G^{-1})^T[i][j] = G^{-1}[j][i] which is
               aug[j*(2*n2) + 2*n + i] + j * aug[j*(2*n2) + 2*n + n + i] */
            double complex GinvT = aug[j*(2*n2) + 2*n + i]
                                 + aug[j*(2*n2) + 2*n + n + i] * I;
            double complex dr = G[i][j] * GinvT;
            drga->rga_magnitude[i][j] = cabs(dr);
            drga->rga_phase[i][j] = carg(dr);
        }
    }

    free(aug); free(Gd); free(G);
    return 0;
}

/* ==========================================================================
 * L5 — Pairing Enumeration
 * ========================================================================== */

static int next_perm(int *a, int n) {
    int i = n - 2;
    while (i >= 0 && a[i] >= a[i+1]) i--;
    if (i < 0) return 0;
    int j = n - 1;
    while (a[j] <= a[i]) j--;
    int t = a[i]; a[i] = a[j]; a[j] = t;
    for (int l = i+1, r = n-1; l < r; l++, r--)
        { t = a[l]; a[l] = a[r]; a[r] = t; }
    return 1;
}

int mimo_enumerate_pairings(const double *K, int n, PairingSet *pset) {
    if (!K || !pset || n <= 0 || n > MIMO_MAX_DIM) return 0;
    memset(pset, 0, sizeof(PairingSet));

    RGAMatrix rga;
    if (mimo_rga_compute(K, n, &rga) != 0) return 0;

    int perm[MIMO_MAX_DIM];
    for (int i = 0; i < n; i++) perm[i] = i;

    int cnt = 0;
    do {
        if (cnt >= MIMO_MAX_PAIRINGS) break;
        double score = 0.0;
        bool valid = true;
        for (int i = 0; i < n; i++) {
            double rv = rga.rga[i][perm[i]];
            if (rv < -MIMO_EPSILON) { valid = false; break; }
            score += 1.0/(1.0 + fabs(rv - 1.0));
            if (rv > 10.0) score -= 0.5;
        }
        if (!valid) continue;
        double ni = mimo_niederlinski_index(K, n, perm);
        MIMOPairing *c = &pset->candidates[cnt];
        for (int i = 0; i < n; i++) c->pairing[i] = perm[i];
        c->n_pairs = n;
        c->rga_value = score;
        c->niederlinski_index = ni;
        c->is_feasible = (ni > 0) && !isnan(ni);
        cnt++;
    } while (next_perm(perm, n));

    pset->n_candidates = cnt;
    /* Sort by score descending */
    for (int i = 0; i < cnt-1; i++)
        for (int j = i+1; j < cnt; j++)
            if (pset->candidates[j].rga_value > pset->candidates[i].rga_value) {
                MIMOPairing t = pset->candidates[i];
                pset->candidates[i] = pset->candidates[j];
                pset->candidates[j] = t;
            }
    pset->best_index = (cnt > 0) ? 0 : -1;
    return cnt;
}

/* ==========================================================================
 * L4 — Integrity Check (Grosdidier, Morari, Holt 1985)
 * ========================================================================== */

bool mimo_check_integrity(const double *K, int n, const int *pairing) {
    if (!K || !pairing || n <= 0 || n > MIMO_MAX_DIM) return false;
    double *Kp = (double *)calloc(n*n, sizeof(double));
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            Kp[i*n + pairing[j]] = K[i*n + j];

    for (int k = 1; k <= n; k++) {
        /* Determinant of kxk leading submatrix */
        double *sub = (double *)calloc(k*k, sizeof(double));
        for (int i = 0; i < k; i++)
            for (int j = 0; j < k; j++)
                sub[i*k + j] = Kp[i*n + j];
        double det = 1.0;
        for (int p = 0; p < k && det != 0; p++) {
            int mr = p;
            double mv = fabs(sub[mr*k+p]);
            for (int r = p+1; r < k; r++)
                if (fabs(sub[r*k+p]) > mv) { mv = fabs(sub[r*k+p]); mr = r; }
            if (mv < MIMO_EPSILON) { det = 0.0; break; }
            if (mr != p)
                for (int c = 0; c < k; c++) {
                    double t = sub[p*k+c]; sub[p*k+c] = sub[mr*k+c]; sub[mr*k+c] = t;
                }
            det *= sub[p*k+p];
            for (int r = p+1; r < k; r++) {
                double f = sub[r*k+p] / sub[p*k+p];
                for (int c = p; c < k; c++) sub[r*k+c] -= f*sub[p*k+c];
            }
        }
        free(sub);
        if (det < -MIMO_EPSILON) { free(Kp); return false; }
    }
    free(Kp);
    return true;
}

/* ==========================================================================
 * L5 — Interaction Quotient
 * ========================================================================== */

double mimo_interaction_quotient(const RGAMatrix *rga) {
    if (!rga || rga->n == 0) return 0.0;
    int n = rga->n;
    double sum = 0.0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            double tgt = (i == j) ? 1.0 : 0.0;
            sum += fabs(rga->rga[i][j] - tgt);
        }
    double iq = 1.0 - sum / (n*n);
    return (iq > 1.0) ? 1.0 : ((iq < 0.0) ? 0.0 : iq);
}

/* ==========================================================================
 * L5 — Effective RGA (Xiong et al. 2005)
 * ========================================================================== */

int mimo_effective_rga(const MIMOModel *model, double bandwidth, double *erga_output) {
    if (!model || !erga_output) return -1;
    int n = model->num_outputs;
    if (n != model->num_inputs || n == 0) return -1;
    double *K = (double *)calloc(n*n, sizeof(double));
    mimo_model_steady_state_gain(model, K);
    RGAMatrix rga;
    if (mimo_rga_compute(K, n, &rga) != 0) { free(K); return -1; }
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            double tau = model->rows[i].elements[j].time_constant;
            double wc = (tau > MIMO_EPSILON) ? 1.0/tau : bandwidth;
            double wt = wc / (bandwidth + MIMO_EPSILON);
            erga_output[i*n+j] = rga.rga[i][j] * wt;
        }
    free(K);
    return 0;
}

/* ==========================================================================
 * Printing
 * ========================================================================== */

void mimo_rga_print(const RGAMatrix *rga) {
    if (!rga) return;
    printf("=== RGA (n=%d) CN=%.2f NI=%.4f ===\n", rga->n, rga->cn, rga->ni);
    for (int i = 0; i < rga->n; i++) {
        printf("  ");
        for (int j = 0; j < rga->n; j++) printf("%8.4f ", rga->rga[i][j]);
        printf("\n");
    }
    printf("%s\n", rga->interpretation);
}

void mimo_pairing_print(const PairingSet *pset) {
    if (!pset) return;
    printf("=== Pairings (%d candidates) ===\n", pset->n_candidates);
    for (int k = 0; k < pset->n_candidates; k++) {
        const MIMOPairing *p = &pset->candidates[k];
        printf("  #%d: [", k+1);
        for (int i = 0; i < p->n_pairs; i++)
            printf("%d%s", p->pairing[i], i < p->n_pairs-1 ? "," : "");
        printf("] NI=%.4f feasible=%s %s\n",
               p->niederlinski_index,
               p->is_feasible ? "yes" : "no",
               k == pset->best_index ? "*** BEST" : "");
    }
}
