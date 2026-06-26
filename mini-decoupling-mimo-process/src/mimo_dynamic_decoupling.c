/**
 * @file mimo_dynamic_decoupling.c
 * @brief Dynamic decoupling design: ideal, simplified, and partial (band-limited).
 *
 * Knowledge points:
 *   L5: Ideal dynamic decoupler D(s) = G^{-1}(s) * diag(G(s))
 *   L5: Simplified dynamic decoupler with realizability filters
 *   L5: Partial (band-limited) decoupler with low-pass shaping
 *   L3: Discrete-time implementation via Tustin transform
 *   L8: Structured singular value (mu) robustness analysis
 *   L8: Lyapunov stability verification (Bartels-Stewart algorithm)
 */

#include "mimo_dynamic_decoupling.h"
#include "mimo_static_decoupling.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ==========================================================================
 * Helper: convert single TF to FOPDT approximation
 * ========================================================================== */

static void tf_to_fopdt(const MIMOTransferFunction *tf,
                         double *K, double *tau, double *theta) {
    *K = tf->gain;
    *theta = tf->time_delay;

    /* For higher-order systems, use the half-rule (Skogestad, 2003):
     * tau_eff = tau_0 + tau_2/2 + tau_3 + ...
     * where tau_i are the time constants sorted by magnitude */
    if (tf->den_order == 1) {
        *tau = tf->time_constant;
    } else if (tf->den_order == 2) {
        /* Denominator: tau^2*s^2 + 2*zeta*tau*s + 1 */
        double a2 = tf->den[2]; /* tau^2 */
        /* Half-rule: effective tau = tau_dominant + tau_secondary/2 */
        *tau = sqrt(a2); /* simplified */
    } else {
        /* Higher order: use sum of time constants approximation */
        *tau = tf->time_constant;
        if (*tau < MIMO_EPSILON) *tau = 1.0;
    }
}

/* ==========================================================================
 * L5 — Ideal Dynamic Decoupler
 * ========================================================================== */

int mimo_ideal_dynamic_decoupler(const MIMOModel *model, DynamicDecoupler *dd) {
    if (!model || !dd) return -1;

    int n = model->num_outputs;
    if (n != model->num_inputs || n > MIMO_MAX_DIM) return -1;

    dd->method = 0;
    dd->bandwidth = 10.0; /* default */
    dd->tolerance = 0.01;
    dd->use_fir = false;
    dd->fir_length = 0;

    decoupler_init(&dd->base, n, n, DECOUPLER_IDEAL);

    if (n == 2) {
        /* 2×2 case: explicit formula
         * D_{11} = 1,    D_{12} = -G_{12}/G_{11}
         * D_{21} = -G_{21}/G_{22},  D_{22} = 1
         */
        const MIMOTransferFunction *G11 = &model->rows[0].elements[0];
        const MIMOTransferFunction *G12 = &model->rows[0].elements[1];
        const MIMOTransferFunction *G21 = &model->rows[1].elements[0];
        const MIMOTransferFunction *G22 = &model->rows[1].elements[1];

        /* Check diagonal elements are non-zero */
        if (fabs(G11->gain) < MIMO_EPSILON || fabs(G22->gain) < MIMO_EPSILON) {
            return -1;
        }

        /* Diagonal: unity */
        decoupler_set_fopdt(&dd->base, 0, 0, 1.0, 0.0, 0.0);
        decoupler_set_fopdt(&dd->base, 1, 1, 1.0, 0.0, 0.0);

        /* D_{12} = -G_{12}/G_{11}
         * Approximate as FOPDT: K_d = -K_{12}/K_{11}
         * tau_d = tau_{12}, theta_d = theta_{12} - theta_{11} */
        double K1, tau1, theta1, K2, tau2, theta2;
        tf_to_fopdt(G11, &K1, &tau1, &theta1);
        tf_to_fopdt(G12, &K2, &tau2, &theta2);

        double K_d12 = -K2 / K1;
        double tau_d12 = tau2;
        double theta_d12 = theta2 - theta1;
        if (theta_d12 < 0) theta_d12 = 0; /* non-causal deadtime; implies zero in numerator */

        decoupler_set_fopdt(&dd->base, 0, 1, K_d12, tau_d12, theta_d12);

        /* D_{21} = -G_{21}/G_{22} */
        tf_to_fopdt(G21, &K1, &tau1, &theta1);
        tf_to_fopdt(G22, &K2, &tau2, &theta2);

        double K_d21 = -K1 / K2;
        double tau_d21 = tau1;
        double theta_d21 = theta1 - theta2;
        if (theta_d21 < 0) theta_d21 = 0;

        decoupler_set_fopdt(&dd->base, 1, 0, K_d21, tau_d21, theta_d21);

    } else if (n == 3) {
        /* 3×3 ideal decoupler:
         * D = G^{-1} * diag(G_{11}, G_{22}, G_{33})
         *
         * Compute G^{-1} symbolically:
         * G^{-1} = adj(G) / det(G)
         *
         * D_{ij} = (cofactor_{ji} / det(G)) * G_{jj}
         *
         * For simplicity, use static approximation at DC:
         * D_static = K^{-1}
         */
        StaticDecoupler sd_static;
        if (mimo_static_decoupler_design(model, &sd_static) == 0) {
            /* Copy static gains */
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    dd->base.elements[i][j].gain = sd_static.K_inv[i][j];
                    dd->base.elements[i][j].num[0] = sd_static.K_inv[i][j];
                    dd->base.elements[i][j].den[0] = 1.0;
                    dd->base.elements[i][j].is_active = true;
                }
            }
        }

        /* Add dynamic terms: lead-lag for off-diagonal elements */
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i == j) continue;
                const MIMOTransferFunction *Gij = &model->rows[i].elements[j];
                const MIMOTransferFunction *Gii = &model->rows[i].elements[i];

                /* D_{ij} has dynamics: ~ -G_{ij}/G_{ii} */
                double Kii = Gii->gain;
                double tau_ii = Gii->time_constant;
                double tau_ij = Gij->time_constant;

                if (fabs(Kii) > MIMO_EPSILON && tau_ii > MIMO_EPSILON) {
                    /* Lead-lag: (tau_ii*s + 1) / (tau_ij*s + 1) * static_gain */
                    dd->base.elements[i][j].gain *= 1.0; /* already set from static */

                    /* Add pole at 1/tau_ij and zero at 1/tau_ii */
                    double T_lead = tau_ii;
                    double T_lag = tau_ij;
                    if (T_lag < MIMO_EPSILON) T_lag = T_lead;

                    dd->base.elements[i][j].num[1] = dd->base.elements[i][j].num[0] * T_lead;
                    dd->base.elements[i][j].den[1] = T_lag;
                    dd->base.elements[i][j].num_order = 1;
                    dd->base.elements[i][j].den_order = 1;
                }
            }
        }
    } else {
        /* n > 3: use static approximation + dynamic correction */
        StaticDecoupler sd_static;
        mimo_static_decoupler_design(model, &sd_static);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                dd->base.elements[i][j].gain = sd_static.K_inv[i][j];
                dd->base.elements[i][j].num[0] = sd_static.K_inv[i][j];
                dd->base.elements[i][j].den[0] = 1.0;
                dd->base.elements[i][j].is_active = true;
            }
        }
    }

    /* Verify properness and stability */
    dd->base.is_causal = decoupler_is_proper(&dd->base);
    dd->base.is_stable = decoupler_is_stable(&dd->base);

    return 0;
}

/* ==========================================================================
 * L5 — Simplified Dynamic Decoupler
 * ========================================================================== */

int mimo_simplified_dynamic_decoupler(const MIMOModel *model, DynamicDecoupler *dd) {
    if (!model || !dd) return -1;

    int n = model->num_outputs;
    if (n != model->num_inputs || n > MIMO_MAX_DIM) return -1;

    dd->method = 1;
    dd->bandwidth = 10.0;
    dd->tolerance = 0.01;
    dd->use_fir = false;
    dd->fir_length = 0;

    decoupler_init(&dd->base, n, n, DECOUPLER_SIMPLIFIED);

    /* Simplified decoupling: off-diagonal elements are scaled versions
     * of -G_{ij}/G_{ii} with additional lag to ensure properness.
     *
     * D_{ii}(s) = 1
     * D_{ij}(s) = -G_{ij}(s)/G_{ii}(s) * F_{ij}(s)
     *
     * where F_{ij}(s) = 1/(T_f*s + 1) ensures D_{ij} is proper
     * when relative degree(G_{ij}) = relative degree(G_{ii}).
     */

    /* Set diagonal to unity */
    for (int i = 0; i < n; i++) {
        decoupler_set_fopdt(&dd->base, i, i, 1.0, 0.0, 0.0);
    }

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) continue;

            const MIMOTransferFunction *Gij = &model->rows[i].elements[j];
            const MIMOTransferFunction *Gii = &model->rows[i].elements[i];

            double K_ij = Gij->gain;
            double K_ii = Gii->gain;
            double tau_ij = Gij->time_constant;
            double tau_ii = Gii->time_constant;

            if (fabs(K_ii) < MIMO_EPSILON) {
                dd->base.elements[i][j].is_active = false;
                continue;
            }

            /* Static gain */
            double K_d = -K_ij / K_ii;
            double tau_d = tau_ij;
            double theta_d = Gij->time_delay - Gii->time_delay;
            if (theta_d < 0) theta_d = 0;

            /* Relative degree adjustment:
             * If num_order of Gij > den_order of Gij (improper ratio),
             * add low-pass filter 1/(T_f*s+1) to make proper */
            int rel_deg_ij = Gij->den_order - Gij->num_order;
            int rel_deg_ii = Gii->den_order - Gii->num_order;
            int extra_lag = rel_deg_ii - rel_deg_ij;
            if (extra_lag < 0) extra_lag = 0;

            /* Build denominator: add extra_lag poles at tau_ii/10 */
            double T_f = tau_ii * 0.1; /* filter time constant */
            if (T_f < MIMO_EPSILON) T_f = 0.1;

            int ord = 1 + extra_lag; /* base order is 1 (FOPDT), plus extra lag */
            if (ord > MIMO_MAX_ORDER) ord = MIMO_MAX_ORDER;

            /* Set decoupler element */
            dd->base.elements[i][j].gain = K_d;
            dd->base.elements[i][j].num[0] = K_d;
            dd->base.elements[i][j].num_order = 0;
            dd->base.elements[i][j].den[0] = 1.0;

            /* Denominator: (tau_d*s + 1) * (T_f*s + 1)^extra_lag
             * Approximate as single FOPDT for simplicity */
            dd->base.elements[i][j].den[1] = tau_d + extra_lag * T_f;
            dd->base.elements[i][j].den_order = 1;
            dd->base.elements[i][j].time_delay = theta_d;
            dd->base.elements[i][j].is_active = true;
        }
    }

    dd->base.is_causal = decoupler_is_proper(&dd->base);
    dd->base.is_stable = decoupler_is_stable(&dd->base);

    return 0;
}

/* ==========================================================================
 * L5 — Partial (Band-Limited) Dynamic Decoupler
 * ========================================================================== */

int mimo_partial_dynamic_decoupler(const MIMOModel *model, DynamicDecoupler *dd) {
    if (!model || !dd) return -1;

    /* First, design the ideal decoupler */
    int ret = mimo_ideal_dynamic_decoupler(model, dd);
    if (ret != 0) return ret;

    dd->method = 2;
    dd->base.type = DECOUPLER_IDEAL; /* using it as a base */

    /* Apply low-pass filter to all off-diagonal elements:
     * D_{ij}^{partial}(s) = D_{ij}^{ideal}(s) * (omega_bw / (s + omega_bw))
     *
     * This limits decoupling action to frequencies below omega_bw.
     */
    double omega_bw = dd->bandwidth;
    if (omega_bw < MIMO_EPSILON) omega_bw = 1.0;

    int n = model->num_outputs;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            if (!dd->base.elements[i][j].is_active) continue;

            DecouplerElement *e = &dd->base.elements[i][j];

            /* Multiply by 1/(s/omega_bw + 1) = omega_bw/(s + omega_bw)
             * This adds one pole at -omega_bw.
             *
             * If e = num(s)/den(s), new e = [num(s)*omega_bw] / [den(s)*(s+omega_bw)]
             */
            /* Shift numerator coefficients: multiply by omega_bw */
            for (int k = 0; k <= e->num_order; k++) {
                e->num[k] *= omega_bw;
            }

            /* Add (s + omega_bw) factor to denominator:
             * New den(s) = old_den(s) * (s + omega_bw)
             * = old_den(s) * omega_bw + old_den(s) * s
             */
            double new_den[MIMO_MAX_ORDER + 2];
            memset(new_den, 0, sizeof(new_den));

            /* new_den[0] = old_den[0] * omega_bw */
            /* new_den[k] = old_den[k] * omega_bw + old_den[k-1] */
            for (int k = 0; k <= e->den_order + 1 && k < MIMO_MAX_ORDER; k++) {
                double term1 = (k <= e->den_order) ? e->den[k] * omega_bw : 0.0;
                double term2 = (k >= 1 && (k - 1) <= e->den_order) ? e->den[k - 1] : 0.0;
                new_den[k] = term1 + term2;
            }

            int new_den_order = e->den_order + 1;
            if (new_den_order > MIMO_MAX_ORDER) new_den_order = MIMO_MAX_ORDER;

            /* Normalize to monic */
            double lead_den = new_den[new_den_order];
            if (fabs(lead_den) < MIMO_EPSILON) lead_den = 1.0;

            e->den_order = new_den_order;
            for (int k = 0; k <= new_den_order; k++) {
                e->den[k] = new_den[k] / lead_den;
            }
        }
    }

    return 0;
}

/* ==========================================================================
 * L3 — Discrete-Time Step (FIR Filter for Implementation)
 * ========================================================================== */

void mimo_dynamic_decoupler_step(const DynamicDecoupler *dd,
                                  const double *u_controller,
                                  double *u_process,
                                  double ***states) {
    if (!dd || !u_controller || !u_process) return;

    int m = dd->base.n_inputs;

    /* Simple implementation: use static gain at each step
     * u_process = D(0) * u_controller (approximate)
     *
     * Full dynamic implementation would maintain states for each
     * decoupler element as a discrete filter.
     */
    for (int i = 0; i < m; i++) {
        double sum = 0.0;
        for (int j = 0; j < m; j++) {
            sum += dd->base.elements[i][j].gain * u_controller[j];
        }
        u_process[i] = sum;
    }

    /* If states pointer provided, update FIR filter states */
    if (states && dd->use_fir && dd->fir_length > 0) {
        /* Shift and update for each (i,j) element */
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < m; j++) {
                /* Shift delay line */
                for (int k = dd->fir_length - 1; k > 0; k--) {
                    states[i][j][k] = states[i][j][k - 1];
                }
                states[i][j][0] = u_controller[j];
            }
        }
    }
}

/* ==========================================================================
 * L3 — Continuous-to-Discrete Conversion (Tustin)
 * ========================================================================== */

int mimo_decoupler_c2d(DynamicDecoupler *dd, double Ts) {
    if (!dd || Ts <= 0) return -1;

    /* Apply Tustin transform to each decoupler element:
     * s -> (2/Ts) * (z-1)/(z+1)
     *
     * For D(s) = num(s)/den(s):
     * D(z) = num((2/Ts)*(z-1)/(z+1)) / den((2/Ts)*(z-1)/(z+1))
     *
     * For simplicity, use Euler's forward difference for implementation:
     * s = (z-1)/Ts  => H(z) = H(s)|_{s=(z-1)/Ts}
     */

    int n = dd->base.n_inputs;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            DecouplerElement *e = &dd->base.elements[i][j];
            if (!e->is_active || e->den_order == 0) continue;

            /* Tustin: H(z) = H(s) where s = (2/Ts) * (z-1)/(z+1)
             *
             * For first-order: (K) / (tau*s + 1) ->
             *   K * (z+1) / ((2*tau/Ts + 1)*z + (1 - 2*tau/Ts))
             *
             * Implementation: use bilinear coefficients
             */
            if (e->den_order == 1 && e->num_order == 0) {
                double K = e->num[0];
                double tau = e->den[1]; /* den = [1, tau] */
                double h2 = 2.0 / Ts;

                /* D(s) = K / (tau*s + 1)
                 * D(z) = K * (z+1) / ((tau*h2 + 1)*z + (1 - tau*h2))
                 *       = [K/(tau*h2+1) * z + K/(tau*h2+1)] / [z + (1-tau*h2)/(tau*h2+1)]
                 */
                double a = tau * h2;
                double b0 = K / (a + 1.0);
                double b1 = b0;
                double a1 = (1.0 - a) / (a + 1.0);

                e->num[0] = b0;
                e->num[1] = b1;
                e->num_order = 1;
                e->den[0] = a1;
                e->den[1] = 1.0; /* monic in z */
                e->den_order = 1;
            }
            /* Higher-order elements use zero-order hold approximation:
             * retain as continuous for now (indicated by is_active flag) */
        }
    }

    return 0;
}

/* ==========================================================================
 * L5 — Frequency Response of Decoupled System
 * ========================================================================== */

void mimo_decoupled_freqresp(const MIMOModel *model, const DynamicDecoupler *dd,
                              const double *freqs, int n_freqs,
                              double *magnitudes) {
    if (!model || !dd || !freqs || !magnitudes || n_freqs <= 0) return;

    int p = model->num_outputs;
    int m = model->num_inputs;

    for (int k = 0; k < n_freqs; k++) {
        double omega = freqs[k];
        double complex s = omega * I;

        /* Evaluate G(s) * D(s) = Ga(s) */
        double complex **G_mat = (double complex **)malloc(p * sizeof(double complex *));
        double complex **D_mat = (double complex **)malloc(m * sizeof(double complex *));
        if (!G_mat || !D_mat) { free(G_mat); free(D_mat); return; }

        for (int i = 0; i < p; i++) {
            G_mat[i] = (double complex *)malloc(m * sizeof(double complex));
            D_mat[i] = (double complex *)malloc(m * sizeof(double complex));
            if (!G_mat[i] || !D_mat[i]) goto cleanup;
        }

        mimo_model_evaluate(model, s, G_mat);
        decoupler_evaluate(&dd->base, s, D_mat);

        /* Ga = G * D, store magnitudes */
        for (int i = 0; i < p; i++) {
            for (int j = 0; j < m; j++) {
                double complex sum = 0.0;
                for (int l = 0; l < m; l++) {
                    sum += G_mat[i][l] * D_mat[l][j];
                }
                int idx = (k * p + i) * m + j;
                magnitudes[idx] = cabs(sum);
            }
        }

    cleanup:
        if (G_mat) { for (int i = 0; i < p; i++) free(G_mat[i]); free(G_mat); }
        if (D_mat) { for (int i = 0; i < m; i++) free(D_mat[i]); free(D_mat); }
    }
}

/* ==========================================================================
 * L8 — Structured Singular Value (mu) Analysis
 * ========================================================================== */

double mimo_mu_analysis(const MIMOModel *model, const DynamicDecoupler *dd) {
    if (!model || !dd) return 0.0;

    /* mu for decoupling: measure how much multiplicative uncertainty
     * in G_{ii} can be tolerated.
     *
     * For diagonal uncertainty Delta = diag(delta_1, ..., delta_n),
     * mu(omega) = max_i |D_{ij}(j*omega) * G_{jj}(j*omega) / G_{ii}(j*omega)|
     *
     * Simplified: use steady-state analysis for lower bound.
     */

    int n = model->num_outputs;
    if (n != model->num_inputs) return 0.0;

    double *K = (double *)malloc(n * n * sizeof(double));
    if (!K) return 0.0;
    mimo_model_steady_state_gain(model, K);

    /* For off-diagonal element (i,j), the perturbation in G_{ii}
     * is coupled through D_{ji}. The mu bound is:
     *
     * mu >= max_{i != j} |K_{ij} * D_{ji}| / |K_{ii}|
     */
    double mu_max = 0.0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            double Kii = K[i * n + i];
            double Kij = K[i * n + j];
            double Dji = dd->base.elements[j][i].gain;

            if (fabs(Kii) > MIMO_EPSILON) {
                double mu_ij = fabs(Kij * Dji / Kii);
                if (mu_ij > mu_max) mu_max = mu_ij;
            }
        }
    }

    free(K);
    return mu_max;
}

/* ==========================================================================
 * L8 — Lyapunov Stability (Bartels-Stewart Algorithm)
 * ========================================================================== */

bool mimo_lyapunov_stability(const double *ss_A, int n, double *P) {
    if (!ss_A || !P || n <= 0 || n > MIMO_MAX_DIM) return false;

    /* Solve Lyapunov equation: A^T P + P A = -Q, with Q = I
     * If P > 0 (all eigenvalues positive), the system is stable.
     *
     * Bartels-Stewart algorithm:
     * 1. Reduce A to real Schur form: A = U T U^T
     * 2. Transform: T^T Y + Y T = -U^T Q U
     * 3. Solve for Y via back-substitution (T is quasi-upper-triangular)
     * 4. P = U Y U^T
     *
     * Simplified for small n: direct solve of the Sylvester equation.
     */

    /* For n <= 4, solve directly via Kronecker product:
     * (I ⊗ A^T + A^T ⊗ I) * vec(P) = -vec(Q)
     */

    /* Build right-hand side: -I (vectorized, Q=I) */
    double *rhs = (double *)calloc(n * n, sizeof(double));
    double *Kron = (double *)calloc(n * n * n * n, sizeof(double));
    if (!rhs || !Kron) { free(rhs); free(Kron); return false; }

    /* Q = I, so vec(Q) has 1 at diagonal positions */
    for (int i = 0; i < n; i++) {
        rhs[i * n + i] = -1.0;
    }

    /* Kronecker: (I ⊗ A^T + A^T ⊗ I)
     * Entry ((i,j), (k,l)) = I_{ik} * A^T_{jl} + A^T_{ik} * I_{jl}
     *                       = delta_{ik} * A_{lj} + A_{ki} * delta_{jl}
     */

    for (int i = 0; i < n; i++) {          /* row of P */
        for (int j = 0; j < n; j++) {      /* col of P */
            int row = i * n + j;
            for (int k = 0; k < n; k++) {  /* row of result */
                for (int l = 0; l < n; l++) { /* col of result */
                    int col = k * n + l;
                    double val = 0.0;

                    /* delta_{ik} * A_{lj} */
                    if (i == k) val += ss_A[l * n + j];

                    /* A_{ki} * delta_{jl} */
                    if (j == l) val += ss_A[i * n + k]; /* A^T_ik = A_ki */

                    Kron[row * (n * n) + col] = val;
                }
            }
        }
    }

    /* Solve Kron * vec(P) = rhs via Gaussian elimination */
    int N = n * n;
    /* Augmented matrix */
    double *aug = (double *)malloc(N * (N + 1) * sizeof(double));
    if (!aug) { free(rhs); free(Kron); return false; }

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            aug[i * (N + 1) + j] = Kron[i * N + j];
        }
        aug[i * (N + 1) + N] = rhs[i];
    }

    /* Gaussian elimination with partial pivoting */
    for (int k = 0; k < N; k++) {
        int max_row = k;
        double max_val = fabs(aug[k * (N + 1) + k]);
        for (int i = k + 1; i < N; i++) {
            if (fabs(aug[i * (N + 1) + k]) > max_val) {
                max_val = fabs(aug[i * (N + 1) + k]);
                max_row = i;
            }
        }
        if (max_val < MIMO_EPSILON) { free(aug); free(rhs); free(Kron); return false; }

        if (max_row != k) {
            for (int j = 0; j <= N; j++) {
                double tmp = aug[k * (N + 1) + j];
                aug[k * (N + 1) + j] = aug[max_row * (N + 1) + j];
                aug[max_row * (N + 1) + j] = tmp;
            }
        }

        for (int i = k + 1; i < N; i++) {
            double factor = aug[i * (N + 1) + k] / aug[k * (N + 1) + k];
            for (int j = k; j <= N; j++) {
                aug[i * (N + 1) + j] -= factor * aug[k * (N + 1) + j];
            }
        }
    }

    /* Back substitution */
    double *vecP = (double *)malloc(N * sizeof(double));
    if (!vecP) { free(aug); free(rhs); free(Kron); return false; }

    for (int i = N - 1; i >= 0; i--) {
        double sum = aug[i * (N + 1) + N];
        for (int j = i + 1; j < N; j++) {
            sum -= aug[i * (N + 1) + j] * vecP[j];
        }
        vecP[i] = sum / aug[i * (N + 1) + i];
    }

    /* Reshape to P matrix */
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            P[i * n + j] = vecP[i * n + j];
        }
    }

    /* Check positive definiteness via Sylvester's criterion:
     * All leading principal minors > 0 */
    bool is_pd = true;
    for (int k = 0; k < n && is_pd; k++) {
        /* Extract k×k leading principal submatrix */
        double *sub = (double *)malloc(k * k * sizeof(double));
        if (!sub) { is_pd = false; break; }

        for (int r = 0; r < k; r++) {
            for (int c = 0; c < k; c++) {
                sub[r * k + c] = P[r * n + c];
            }
        }

        double det_sub = 0.0;
        if (k == 1) {
            det_sub = sub[0];
        } else if (k == 2) {
            det_sub = sub[0] * sub[3] - sub[1] * sub[2];
        } else {
            /* Compute determinant for k >= 3 */
            double *LU = (double *)malloc(k * k * sizeof(double));
            if (LU) {
                memcpy(LU, sub, k * k * sizeof(double));
                det_sub = 1.0;
                int sign = 1;
                for (int p = 0; p < k; p++) {
                    int max_row2 = p;
                    double max_val2 = fabs(LU[p * k + p]);
                    for (int rr = p + 1; rr < k; rr++) {
                        if (fabs(LU[rr * k + p]) > max_val2) {
                            max_val2 = fabs(LU[rr * k + p]);
                            max_row2 = rr;
                        }
                    }
                    if (max_val2 < MIMO_EPSILON) { det_sub = 0.0; break; }
                    if (max_row2 != p) {
                        for (int cc = 0; cc < k; cc++) {
                            double tmp = LU[p * k + cc];
                            LU[p * k + cc] = LU[max_row2 * k + cc];
                            LU[max_row2 * k + cc] = tmp;
                        }
                        sign = -sign;
                    }
                    det_sub *= LU[p * k + p];
                    for (int rr = p + 1; rr < k; rr++) {
                        double factor = LU[rr * k + p] / LU[p * k + p];
                        for (int cc = p; cc < k; cc++) {
                            LU[rr * k + cc] -= factor * LU[p * k + cc];
                        }
                    }
                }
                det_sub *= sign;
                free(LU);
            }
        }

        free(sub);
        if (det_sub <= MIMO_EPSILON) { is_pd = false; break; }
    }

    free(aug); free(rhs); free(Kron); free(vecP);
    return is_pd;
}
