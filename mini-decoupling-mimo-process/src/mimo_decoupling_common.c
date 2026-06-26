/**
 * @file mimo_decoupling_common.c
 * @brief Common decoupling utilities: init, evaluation, Routh-Hurwitz stability,
 *        pole finding via companion matrix QR, properness checks.
 *
 * Knowledge: L1 (struct operations), L2 (properness/stability), L4 (Routh-Hurwitz),
 *             L5 (pole finding via QR iteration), L3 (complex evaluation).
 *
 * References:
 *   - Routh (1877), "A Treatise on the Stability of a Given State of Motion"
 *   - Golub & Van Loan (2013), "Matrix Computations", Ch.7
 *   - Francis (1961), "The QR Transformation", Computer J. 4
 */

#include "mimo_decoupling_common.h"
#include "mimo_model.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>

/* ==========================================================================
 * L1/L2 — Decoupler Initialization
 * ========================================================================== */

void decoupler_init(Decoupler *D, int n_inputs, int n_outputs, DecouplerType type) {
    if (!D) return;
    if (n_inputs < 1 || n_inputs > MIMO_MAX_DIM) n_inputs = 1;
    if (n_outputs < 1 || n_outputs > MIMO_MAX_DIM) n_outputs = 1;
    memset(D, 0, sizeof(Decoupler));
    D->n_inputs = n_inputs;
    D->n_outputs = n_outputs;
    D->type = type;
    D->is_causal = true;
    D->is_stable = true;
    D->condition = 1.0;
    for (int i = 0; i < n_outputs; i++) {
        for (int j = 0; j < n_inputs; j++) {
            DecouplerElement *el = &D->elements[i][j];
            memset(el->num, 0, sizeof(el->num));
            memset(el->den, 0, sizeof(el->den));
            el->num_order = 0;
            el->den_order = 0;
            el->gain = (i == j) ? 1.0 : 0.0;
            el->num[0] = el->gain;
            el->den[0] = 1.0;
            el->time_delay = 0.0;
            el->is_active = (i == j);
        }
    }
}

void decoupler_set_fopdt(Decoupler *D, int i, int j,
                          double K, double tau, double theta) {
    if (!D) return;
    if (i < 0 || i >= D->n_outputs || j < 0 || j >= D->n_inputs) return;
    DecouplerElement *el = &D->elements[i][j];
    memset(el->num, 0, sizeof(el->num));
    memset(el->den, 0, sizeof(el->den));
    el->num[0] = K;
    el->num_order = 0;
    el->den[0] = 1.0;
    el->den[1] = tau;
    el->den_order = 1;
    el->gain = K;
    el->time_delay = theta;
    el->is_active = true;
}

void decoupler_set_leadlag(Decoupler *D, int i, int j,
                            double K, double T_lead, double T_lag) {
    if (!D) return;
    if (i < 0 || i >= D->n_outputs || j < 0 || j >= D->n_inputs) return;
    DecouplerElement *el = &D->elements[i][j];
    memset(el->num, 0, sizeof(el->num));
    memset(el->den, 0, sizeof(el->den));
    el->num[0] = K;
    el->num[1] = K * T_lead;
    el->num_order = 1;
    el->den[0] = 1.0;
    el->den[1] = T_lag;
    el->den_order = 1;
    el->gain = K;
    el->time_delay = 0.0;
    el->is_active = true;
}

/* ==========================================================================
 * L3 — Evaluate decoupler element D_{ij}(s) at complex s via Horner's method.
 * Complexity: O(max(num_order, den_order))
 * ========================================================================== */

static double complex dcpl_eval(const DecouplerElement *el, double complex s) {
    double complex num_val = el->num[el->num_order];
    for (int k = el->num_order - 1; k >= 0; k--)
        num_val = num_val * s + el->num[k];
    double complex den_val = el->den[el->den_order];
    for (int k = el->den_order - 1; k >= 0; k--)
        den_val = den_val * s + el->den[k];
    if (cabs(den_val) < MIMO_EPSILON)
        return 1e15 * num_val;
    double complex result = num_val / den_val;
    if (el->time_delay > MIMO_EPSILON)
        result *= cexp(-el->time_delay * s);
    return result;
}

void decoupler_evaluate(const Decoupler *D, double complex s,
                         double complex **D_matrix) {
    if (!D || !D_matrix) return;
    for (int i = 0; i < D->n_outputs; i++)
        for (int j = 0; j < D->n_inputs; j++)
            D_matrix[i][j] = dcpl_eval(&D->elements[i][j], s);
}

/* ==========================================================================
 * L2 — Apparent process G_a(s) = G(s) * D(s)
 * ========================================================================== */

void decoupler_apparent_process(const MIMOModel *model, const Decoupler *D,
                                 double complex s, double complex **Ga) {
    if (!model || !D || !Ga) return;
    int p = model->num_outputs, m = model->num_inputs;
    double complex *Gd = (double complex *)calloc(p * m, sizeof(double complex));
    double complex *Dd = (double complex *)calloc(m * m, sizeof(double complex));
    double complex **G = (double complex **)malloc(p * sizeof(double complex *));
    double complex **Dm = (double complex **)malloc(m * sizeof(double complex *));
    if (!Gd || !Dd || !G || !Dm) {
        free(Gd); free(Dd); free(G); free(Dm); return;
    }
    for (int i = 0; i < p; i++) G[i] = &Gd[i * m];
    for (int i = 0; i < m; i++) Dm[i] = &Dd[i * m];
    mimo_model_evaluate(model, s, G);
    decoupler_evaluate(D, s, Dm);
    for (int i = 0; i < p; i++)
        for (int j = 0; j < m; j++) {
            double complex sum = 0.0;
            for (int k = 0; k < m; k++) sum += G[i][k] * Dm[k][j];
            Ga[i][j] = sum;
        }
    free(Gd); free(Dd); free(G); free(Dm);
}

/* ==========================================================================
 * L2 — Interaction residual after decoupling (at s=0)
 * ========================================================================== */

int decoupler_interaction_metric(const MIMOModel *model, const Decoupler *D,
                                  InteractionMetric *metric) {
    if (!model || !D || !metric) return -1;
    memset(metric, 0, sizeof(InteractionMetric));
    int n = model->num_outputs;
    if (n != model->num_inputs || n == 0) return -1;
    double complex *Gd = (double complex *)calloc(n * n, sizeof(double complex));
    double complex **Ga = (double complex **)malloc(n * sizeof(double complex *));
    if (!Gd || !Ga) { free(Gd); free(Ga); return -1; }
    for (int i = 0; i < n; i++) Ga[i] = &Gd[i * n];
    decoupler_apparent_process(model, D, 0.0 + 0.0*I, Ga);
    double off_max = 0.0, diag_min = INFINITY, off_sum = 0.0;
    int off_cnt = 0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            double mag = cabs(Ga[i][j]);
            if (i == j) { if (mag < diag_min) diag_min = mag; }
            else { off_sum += mag; off_cnt++; if (mag > off_max) off_max = mag; }
        }
    if (diag_min < MIMO_EPSILON) diag_min = 1.0;
    metric->rga_max_off_diagonal = off_max / diag_min;
    metric->rga_mean_deviation = off_cnt > 0 ? off_sum / (off_cnt * diag_min) : 0.0;
    metric->is_acceptable = (metric->rga_max_off_diagonal < 0.3);
    snprintf(metric->summary, sizeof(metric->summary),
             "Max off-diag/diag=%.4f acceptable=%s",
             metric->rga_max_off_diagonal,
             metric->is_acceptable ? "yes" : "no");
    free(Gd); free(Ga);
    return 0;
}

/* ==========================================================================
 * L2 — Properness check: all elements num_order <= den_order
 * ========================================================================== */

bool decoupler_is_proper(const Decoupler *D) {
    if (!D) return false;
    for (int i = 0; i < D->n_outputs; i++)
        for (int j = 0; j < D->n_inputs; j++) {
            const DecouplerElement *el = &D->elements[i][j];
            if (!el->is_active) continue;
            if (el->num_order > el->den_order) return false;
        }
    return true;
}

/* ==========================================================================
 * L4 — Routh-Hurwitz Stability Criterion (Routh, 1877)
 *
 * For polynomial a_n*s^n + ... + a_1*s + a_0, the Routh array counts
 * sign changes in column 1 = number of RHP roots.
 *
 * Routh array construction:
 *   Row 0: a_n,   a_{n-2}, a_{n-4}, ...
 *   Row 1: a_{n-1}, a_{n-3}, a_{n-5}, ...
 *   Row k: b1 = (row_{k-1}[0]*row_{k-2}[c+1] - row_{k-2}[0]*row_{k-1}[c+1]) / row_{k-1}[0]
 * ========================================================================== */

int mimo_routh_hurwitz(const double *den, int order, double *routh) {
    if (!den || !routh || order <= 0) return -1;
    int n = order;
    int cols = (n + 2) / 2;

    /* First two rows: alternating coefficients */
    for (int c = 0; c < cols; c++) {
        int i0 = n - 2*c, i1 = n - 1 - 2*c;
        routh[0*cols + c] = (i0 >= 0) ? den[i0] : 0.0;
        routh[1*cols + c] = (i1 >= 0) ? den[i1] : 0.0;
    }

    /* Count sign changes in first column */
    int sign_chg = 0;
    double prev = routh[0*cols];

    for (int r = 0; r <= n; r++) {
        double cur = routh[r*cols];
        if (prev > MIMO_EPSILON && cur < -MIMO_EPSILON) sign_chg++;
        if (prev < -MIMO_EPSILON && cur > MIMO_EPSILON) sign_chg++;
        if (fabs(cur) > MIMO_EPSILON) prev = cur;

        if (r >= 2) {
            double pivot = routh[(r-1)*cols];
            if (fabs(pivot) < MIMO_EPSILON) pivot = MIMO_EPSILON;
            for (int c = 0; c < cols - 1; c++) {
                double num = routh[(r-1)*cols] * routh[(r-2)*cols + c + 1]
                           - routh[(r-2)*cols] * routh[(r-1)*cols + c + 1];
                routh[r*cols + c] = num / pivot;
            }
            routh[r*cols + cols - 1] = 0.0;
        }
    }
    return sign_chg;
}

bool decoupler_is_stable(const Decoupler *D) {
    if (!D) return false;
    int maxord = 0;
    for (int i = 0; i < D->n_outputs; i++)
        for (int j = 0; j < D->n_inputs; j++)
            if (D->elements[i][j].den_order > maxord)
                maxord = D->elements[i][j].den_order;
    int cols = (maxord + 2) / 2;
    double *routh = (double *)calloc((maxord + 1) * cols, sizeof(double));
    if (!routh) return false;
    bool stable = true;
    for (int i = 0; i < D->n_outputs && stable; i++)
        for (int j = 0; j < D->n_inputs && stable; j++) {
            const DecouplerElement *el = &D->elements[i][j];
            if (!el->is_active || el->den_order <= 0) continue;
            if (mimo_routh_hurwitz(el->den, el->den_order, routh) != 0)
                stable = false;
        }
    free(routh);
    return stable;
}

/* ==========================================================================
 * L5 — Pole Finding via Companion Matrix + QR Iteration
 *
 * For monic polynomial p(s) = s^n + a_{n-1}*s^{n-1} + ... + a_0,
 * the companion matrix C has p(s) as its characteristic polynomial.
 * Its eigenvalues = roots of p(s).
 *
 * Denom coefficients: den[0]=a_0, den[1]=a_1, ..., den[n]=a_n=1 (monic).
 *
 * Algorithm: Build Hessenberg companion form, then Francis QR iteration
 * with Wilkinson shift for rapid convergence to Schur form.
 *
 * Reference: Golub & Van Loan (2013), Algorithm 7.5.1
 * ========================================================================== */

int mimo_find_poles(const double *den, int order, double complex *poles) {
    if (!den || !poles || order <= 0) return 0;

    /* Linear: as + b = 0 => s = -b/a */
    if (order == 1) {
        if (fabs(den[1]) < MIMO_EPSILON) { poles[0] = INFINITY; return 0; }
        poles[0] = -den[0] / den[1] + 0.0 * I;
        return 1;
    }

    /* Quadratic: formula */
    if (order == 2) {
        double a = den[2], b = den[1], c = den[0];
        double disc = b*b - 4.0*a*c;
        if (disc >= 0.0) {
            double sd = sqrt(disc);
            poles[0] = (-b + sd)/(2.0*a) + 0.0*I;
            poles[1] = (-b - sd)/(2.0*a) + 0.0*I;
            return 2;
        } else {
            double re = -b/(2.0*a), im = sqrt(-disc)/(2.0*a);
            poles[0] = re + im*I;
            poles[1] = re - im*I;
            return 0;
        }
    }

    /* Order >= 3: Companion matrix + QR */
    int n = order;
    double *H = (double *)calloc(n * n, sizeof(double));
    if (!H) return 0;

    for (int i = 0; i < n - 1; i++) H[i*n + i + 1] = 1.0;
    for (int j = 0; j < n; j++) H[(n-1)*n + j] = -den[j];

    /* QR iteration (max 200 iter) */
    for (int iter = 0; iter < 200; iter++) {
        /* Wilkinson shift */
        double shift = H[(n-1)*n + (n-1)];
        if (n >= 2) {
            double a = H[(n-2)*n+(n-2)], b = H[(n-2)*n+(n-1)];
            double c2 = H[(n-1)*n+(n-2)], d = H[(n-1)*n+(n-1)];
            double tr = a + d, disc = tr*tr - 4.0*(a*d - b*c2);
            double e1 = (tr + sqrt(fmax(disc, 0.0)))/2.0;
            double e2 = (tr - sqrt(fmax(disc, 0.0)))/2.0;
            shift = (fabs(e1 - d) < fabs(e2 - d)) ? e1 : e2;
        }
        for (int i = 0; i < n; i++) H[i*n+i] -= shift;

        /* QR via Givens */
        for (int k = 0; k < n - 1; k++) {
            double x = H[k*n+k], y = H[(k+1)*n+k];
            double r = hypot(x, y);
            if (r < MIMO_EPSILON) continue;
            double cv = x/r, sv = -y/r;
            for (int j = k; j < n; j++) {
                double h1 = H[k*n+j], h2 = H[(k+1)*n+j];
                H[k*n+j] = cv*h1 - sv*h2;
                H[(k+1)*n+j] = sv*h1 + cv*h2;
            }
        }
        for (int k = 0; k < n - 1; k++) {
            double x = H[k*n+k], y = H[(k+1)*n+k];
            double r = hypot(x, y);
            if (r < MIMO_EPSILON) continue;
            double cv = x/r, sv = -y/r;
            for (int i = 0; i <= k+1 && i < n; i++) {
                double h1 = H[i*n+k], h2 = H[i*n+(k+1)];
                H[i*n+k] = cv*h1 - sv*h2;
                H[i*n+(k+1)] = sv*h1 + cv*h2;
            }
        }
        for (int i = 0; i < n; i++) H[i*n+i] += shift;

        bool done = true;
        for (int i = 1; i < n; i++)
            if (fabs(H[i*n+(i-1)]) > 1e-10) { done = false; break; }
        if (done) break;
    }

    /* Extract eigenvalues */
    int rc = 0, i = 0;
    while (i < n) {
        if (i < n - 1 && fabs(H[(i+1)*n+i]) > 1e-8) {
            double a = H[i*n+i], b2 = H[i*n+(i+1)];
            double c2 = H[(i+1)*n+i], d = H[(i+1)*n+(i+1)];
            double tr = a + d, dt = a*d - b2*c2;
            double disc = tr*tr - 4.0*dt;
            double re = tr/2.0;
            double im = (disc < 0) ? sqrt(-disc)/2.0 : 0.0;
            if (im < MIMO_EPSILON) {
                double sdd = sqrt(fmax(disc, 0.0));
                poles[i] = (tr + sdd)/2.0 + 0.0*I;
                poles[i+1] = (tr - sdd)/2.0 + 0.0*I;
                rc += 2;
            } else {
                poles[i] = re + im*I;
                poles[i+1] = re - im*I;
            }
            i += 2;
        } else {
            poles[i] = H[i*n+i] + 0.0*I;
            rc++; i++;
        }
    }

    /* Sort by real part */
    for (int a = 0; a < n - 1; a++)
        for (int b = a + 1; b < n; b++)
            if (creal(poles[a]) > creal(poles[b])) {
                double complex t = poles[a];
                poles[a] = poles[b]; poles[b] = t;
            }

    free(H);
    return rc;
}
