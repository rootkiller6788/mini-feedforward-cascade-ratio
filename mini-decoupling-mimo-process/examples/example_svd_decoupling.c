/**
 * example_svd_decoupling.c — SVD-Based MIMO Decoupling
 *
 * Demonstrates SVD decomposition, SVD-based decoupler design,
 * and singular value analysis for MIMO robustness.
 * Reference: Skogestad & Postlethwaite (2005)
 */

#include "mimo_model.h"
#include "mimo_svd_decoupling.h"
#include "mimo_interaction.h"
#include <stdio.h>

int main(void) {
    printf("=== SVD-Based MIMO Decoupling ===\n\n");

    MIMOModel m;
    mimo_model_init(&m, 2, 2, "SVD Test");
    mimo_model_set_fopdt(&m, 0, 0, 3.0, 5.0, 1.0);
    mimo_model_set_fopdt(&m, 0, 1, 0.8, 3.0, 2.0);
    mimo_model_set_fopdt(&m, 1, 0, 0.5, 4.0, 1.5);
    mimo_model_set_fopdt(&m, 1, 1, 2.5, 6.0, 1.0);
    printf("2x2 system initialized.\n");

    double K[4];
    mimo_model_steady_state_gain(&m, K);
    printf("Gain: [%.2f %.2f; %.2f %.2f]\n\n", K[0], K[1], K[2], K[3]);

    SVDDecoupler svd;
    if (mimo_svd_decompose(K, 2, &svd) == 0) {
        printf("Singular values: %.3f, %.3f\n", svd.Sigma[0], svd.Sigma[1]);
        printf("Condition number: %.2f\n", svd.condition_number);
        printf("U = [%.4f %.4f; %.4f %.4f]\n",
               svd.U[0][0], svd.U[0][1], svd.U[1][0], svd.U[1][1]);
        printf("V = [%.4f %.4f; %.4f %.4f]\n",
               svd.V[0][0], svd.V[0][1], svd.V[1][0], svd.V[1][1]);
    }

    SVDDecoupler sd_dec;
    if (mimo_svd_static_decoupler(&m, &sd_dec) == 0) {
        printf("\nSVD decoupler D:\n");
        printf("  [%.4f %.4f]\n", sd_dec.base.elements[0][0].gain,
               sd_dec.base.elements[0][1].gain);
        printf("  [%.4f %.4f]\n", sd_dec.base.elements[1][0].gain,
               sd_dec.base.elements[1][1].gain);

        double uc[] = {1.0, 0.5}, up[2];
        mimo_svd_decoupler_apply(&sd_dec, uc, up);
        printf("Controller: [%.2f %.2f] -> Decoupled: [%.2f %.2f]\n",
               uc[0], uc[1], up[0], up[1]);
    }

    return 0;
}
