/**
 * @file mimo_inverted_decoupling.c
 * @brief Inverted decoupling design — feedforward-based decoupling with
 *        implicit signal flow structure. More robust than conventional decoupling.
 *
 * Knowledge points:
 *   L5: Standard inverted decoupler design
 *   L5: IMC-based inverted decoupling
 *   L3: Algebraic loop resolution (fixed-point iteration)
 *   L8: Robustness margin analysis
 *   L5: Cycle detection in feedforward graph (DFS)
 */

#include "mimo_inverted_decoupling.h"
#include "mimo_interaction.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ==========================================================================
 * L5 — Standard Inverted Decoupler Design
 * ========================================================================== */

int mimo_inverted_decoupler_design(const MIMOModel *model, InvertedDecoupler *id) {
    if (!model || !id) return -1;

    int n = model->num_outputs;
    if (n != model->num_inputs || n > MIMO_MAX_DIM) return -1;

    memset(id, 0, sizeof(*id));
    id->use_imc_structure = false;
    for (int i = 0; i < n; i++) {
        id->alpha[i] = 1.0; /* default robustness */
        id->filter_time_const[i] = 0.0;
    }

    decoupler_init(&id->base, n, n, DECOUPLER_INVERTED);

    /* For each output i, design feedforward from other MVs:
     *
     * d_{ij}(s) = -G_{ij}(s) / G_{ii}(s)   for j != i
     *
     * Signal flow (2x2):
     *   u_1 = u_{c1} + d_{12} * u_2
     *   u_2 = u_{c2} + d_{21} * u_1
     *
     * This creates a loop that must be resolved.
     */

    for (int i = 0; i < n; i++) {
        /* Set diagonal: direct path */
        decoupler_set_fopdt(&id->base, i, i, 1.0, 0.0, 0.0);

        const MIMOTransferFunction *Gii = &model->rows[i].elements[i];
        if (fabs(Gii->gain) < MIMO_EPSILON) continue;

        for (int j = 0; j < n; j++) {
            if (i == j) continue;

            const MIMOTransferFunction *Gij = &model->rows[i].elements[j];

            /* d_{ij} = -G_{ij} / G_{ii} */
            double K_d = -Gij->gain / Gii->gain;
            double tau_d = Gij->time_constant;
            double theta_d = Gij->time_delay - Gii->time_delay;
            if (theta_d < 0) theta_d = 0;

            /* Apply robustness factor alpha */
            K_d *= id->alpha[i];

            /* Set decoupler element fields directly */
            id->feedforward[i][j].gain = K_d;
            id->feedforward[i][j].num[0] = K_d;
            id->feedforward[i][j].num_order = 0;
            id->feedforward[i][j].den[0] = 1.0;
            id->feedforward[i][j].den[1] = tau_d;
            id->feedforward[i][j].den_order = 1;
            id->feedforward[i][j].time_delay = theta_d;
            id->feedforward[i][j].is_active = true;

            /* Also store in base decoupler for evaluation */
            memcpy(&id->base.elements[i][j], &id->feedforward[i][j],
                   sizeof(DecouplerElement));
        }
    }

    /* Verify no algebraic loop for n > 2 */
    if (n > 2) {
        if (!mimo_inverted_no_algebraic_loop(id)) {
            /* Could flag as warning but not fatal */
        }
    }

    return 0;
}

/* ==========================================================================
 * L5 — IMC-Based Inverted Decoupler (Garrido et al., 2012)
 * ========================================================================== */

int mimo_inverted_imc_decoupler(const MIMOModel *model, InvertedDecoupler *id,
                                 const double *lambda) {
    if (!model || !id || !lambda) return -1;

    int n = model->num_outputs;
    if (n != model->num_inputs || n > MIMO_MAX_DIM) return -1;

    /* First, standard inverted decoupler */
    int ret = mimo_inverted_decoupler_design(model, id);
    if (ret != 0) return ret;

    id->use_imc_structure = true;

    /* Apply IMC low-pass filter to each feedforward path:
     *
     * d_{ij}^{IMC}(s) = d_{ij}(s) * 1/(lambda_i * s + 1)^{n_{ij}}
     *
     * where n_{ij} ensures properness.
     */

    for (int i = 0; i < n; i++) {
        id->filter_time_const[i] = lambda[i];

        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            if (!id->feedforward[i][j].is_active) continue;

            DecouplerElement *e = &id->feedforward[i][j];

            /* Add pole at -1/lambda_i
             * New den(s) = old_den(s) * (lambda_i * s + 1)
             */
            double lam = lambda[i];
            if (lam < MIMO_EPSILON) lam = 0.1;

            /* Multiply denominator by (lam*s + 1):
             * den_new[k] = den_old[k] * 1 + den_old[k-1] * lam */
            double new_den[MIMO_MAX_ORDER + 2];
            memset(new_den, 0, sizeof(new_den));

            for (int k = 0; k <= e->den_order + 1 && k < MIMO_MAX_ORDER; k++) {
                double term1 = (k <= e->den_order) ? e->den[k] : 0.0;
                double term2 = (k >= 1) ? e->den[k - 1] * lam : 0.0;
                new_den[k] = term1 + term2;
            }

            int new_ord = e->den_order + 1;
            if (new_ord > MIMO_MAX_ORDER) new_ord = MIMO_MAX_ORDER;

            /* Normalize to monic */
            double lead = new_den[new_ord];
            if (fabs(lead) < MIMO_EPSILON) lead = 1.0;

            e->den_order = new_ord;
            for (int k = 0; k <= new_ord; k++) {
                e->den[k] = new_den[k] / lead;
            }

            /* Copy to base */
            memcpy(&id->base.elements[i][j], e, sizeof(DecouplerElement));
        }
    }

    return 0;
}

/* ==========================================================================
 * L3 — Algebraic Loop Resolution (Fixed-Point Iteration)
 * ========================================================================== */

int mimo_inverted_decoupler_step(InvertedDecoupler *id,
                                  const double *u_controller,
                                  double *u_process) {
    if (!id || !u_controller || !u_process) return 0;

    int n = id->base.n_inputs;

    if (n == 1) {
        u_process[0] = u_controller[0];
        return 1;
    }

    if (n == 2) {
        /* 2×2: closed-form solution
         *
         * u_1 = u_{c1} + d_{12} * u_2
         * u_2 = u_{c2} + d_{21} * u_1
         *
         * Substitute:
         * u_1 = u_{c1} + d_{12} * (u_{c2} + d_{21} * u_1)
         * u_1 = u_{c1} + d_{12}*u_{c2} + d_{12}*d_{21}*u_1
         * u_1 * (1 - d_{12}*d_{21}) = u_{c1} + d_{12}*u_{c2}
         * u_1 = (u_{c1} + d_{12}*u_{c2}) / (1 - d_{12}*d_{21})
         *
         * Similarly:
         * u_2 = (u_{c2} + d_{21}*u_{c1}) / (1 - d_{12}*d_{21})
         */

        double d12 = id->feedforward[0][1].gain;
        double d21 = id->feedforward[1][0].gain;

        double denom = 1.0 - d12 * d21;
        if (fabs(denom) < MIMO_EPSILON) {
            /* Degenerate: use controller signal directly */
            u_process[0] = u_controller[0];
            u_process[1] = u_controller[1];
            return 1;
        }

        u_process[0] = (u_controller[0] + d12 * u_controller[1]) / denom;
        u_process[1] = (u_controller[1] + d21 * u_controller[0]) / denom;
        return 1;

    } else {
        /* n > 2: fixed-point iteration
         *
         * u^{(k+1)} = u_c + F * u^{(k)}
         * where F_{ij} = d_{ij} for i != j, F_{ii} = 0
         *
         * Converges if spectral radius of F < 1.
         */

        /* Initialize with controller signals */
        double u_prev[MIMO_MAX_DIM];
        double u_curr[MIMO_MAX_DIM];
        for (int i = 0; i < n; i++) {
            u_prev[i] = u_controller[i];
            u_curr[i] = u_controller[i];
        }

        int max_iter = 50;
        int iter;
        for (iter = 0; iter < max_iter; iter++) {
            for (int i = 0; i < n; i++) {
                double sum = u_controller[i];
                for (int j = 0; j < n; j++) {
                    if (i == j) continue;
                    if (id->feedforward[i][j].is_active) {
                        sum += id->feedforward[i][j].gain * u_prev[j];
                    }
                }
                u_curr[i] = sum;
            }

            /* Check convergence */
            double max_diff = 0.0;
            for (int i = 0; i < n; i++) {
                double diff = fabs(u_curr[i] - u_prev[i]);
                if (diff > max_diff) max_diff = diff;
            }

            if (max_diff < 1e-8) break;

            for (int i = 0; i < n; i++) {
                u_prev[i] = u_curr[i];
            }
        }

        for (int i = 0; i < n; i++) {
            u_process[i] = u_curr[i];
        }

        return iter;
    }
}

/* ==========================================================================
 * L8 — Robustness Margin for Inverted Decoupler
 * ========================================================================== */

double mimo_inverted_robustness(const MIMOModel *model, const InvertedDecoupler *id) {
    if (!model || !id) return 0.0;

    int n = model->num_outputs;
    if (n <= 1) return 1.0;

    /* Robustness margin: how much multiplicative error in G_{ii}
     * can be tolerated.
     *
     * For inverted decoupling, the condition for stability under
     * multiplicative uncertainty Delta_i in G_{ii} is:
     *
     * |Delta_i| < |1 - sum_{j != i} d_{ij} * d_{ji}| / |sum_{j != i} d_{ij} * d_{ji}|
     *
     * Return the minimum over i of this bound.
     */

    double *K = (double *)malloc(n * n * sizeof(double));
    if (!K) return 0.0;
    mimo_model_steady_state_gain(model, K);

    double min_margin = 1.0;

    for (int i = 0; i < n; i++) {
        double sum_loop = 0.0;
        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            sum_loop += fabs(id->feedforward[i][j].gain * id->feedforward[j][i].gain);
        }

        /* Margin = |1 - sum_loop| / sum_loop */
        double margin;
        if (sum_loop < MIMO_EPSILON) {
            margin = 1.0; /* no interaction, infinite margin */
        } else {
            margin = fabs(1.0 - sum_loop) / sum_loop;
            if (margin > 1.0) margin = 1.0; /* cap at 1 */
            if (margin < 0.0) margin = 0.0;
        }

        if (margin < min_margin) min_margin = margin;
    }

    free(K);

    /* Also account for condition number */
    double cn = 1.0;
    double *K_flat = (double *)malloc(n * n * sizeof(double));
    if (K_flat) {
        mimo_model_steady_state_gain(model, K_flat);
        /* Use singular values for condition number */
        cn = mimo_condition_number(K_flat, n) / 10.0; /* normalize */
        if (cn > 1.0) min_margin /= cn;
        free(K_flat);
    }

    return min_margin;
}

/* ==========================================================================
 * L5 — Algebraic Loop Detection (Cycle Detection via DFS)
 * ========================================================================== */

bool mimo_inverted_no_algebraic_loop(const InvertedDecoupler *id) {
    if (!id) return true;

    int n = id->base.n_inputs;
    if (n <= 2) return true; /* 2x2 is always resolvable */

    /* Build adjacency matrix: A_{ij} = 1 if d_{ij} is active */
    bool adj[MIMO_MAX_DIM][MIMO_MAX_DIM];
    memset(adj, 0, sizeof(adj));

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (id->feedforward[i][j].is_active) {
                adj[i][j] = true;
            }
        }
    }

    /* DFS cycle detection */
    /* Colors: 0 = white (unvisited), 1 = gray (in stack), 2 = black (done) */
    int color[MIMO_MAX_DIM];
    memset(color, 0, sizeof(color));

    /* Iterative DFS with explicit stack */
    typedef struct { int node; int next_child; } StackFrame;
    StackFrame stack[MIMO_MAX_DIM * MIMO_MAX_DIM];
    int stack_top = 0;

    for (int start = 0; start < n; start++) {
        if (color[start] != 0) continue;

        /* Push start node */
        stack[0].node = start;
        stack[0].next_child = 0;
        stack_top = 1;
        color[start] = 1; /* gray */

        while (stack_top > 0) {
            StackFrame *frame = &stack[stack_top - 1];
            int u = frame->node;
            bool found_child = false;

            for (int v = frame->next_child; v < n; v++) {
                if (adj[u][v]) {
                    if (color[v] == 1) {
                        return false; /* back edge = cycle found */
                    }
                    if (color[v] == 0) {
                        /* Push child */
                        frame->next_child = v + 1; /* resume here later */
                        stack[stack_top].node = v;
                        stack[stack_top].next_child = 0;
                        stack_top++;
                        color[v] = 1; /* gray */
                        found_child = true;
                        break;
                    }
                }
            }

            if (!found_child) {
                /* No more children; pop and mark black */
                color[u] = 2; /* black */
                stack_top--;
            }
        }
    }

    return true;
}
