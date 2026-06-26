/**
 * @file example_reactor_decoupling.c
 * @brief Chemical Reactor Temperature/Level Control — 2×2 MIMO Decoupling
 *
 * L6 Canonical Problem: Continuous stirred-tank reactor (CSTR) with
 * interaction between cooling jacket temperature and reactor level.
 *
 * MV1: Feed flow rate        → CV1: Reactor level
 * MV2: Coolant flow rate     → CV2: Reactor temperature
 *
 * Transfer function (simplified model):
 *   G(s) = [   2.0e^{-0.5s}/(5.0s+1)     0.8/(3.0s+1)    ]
 *          [   0.5/(8.0s+1)             -3.0e^{-s}/(6.0s+1) ]
 *
 * This demonstrates inverted decoupling which is preferred for
 * chemical processes due to superior robustness.
 */

#include <stdio.h>
#include <math.h>
#include "../include/mimo_model.h"
#include "../include/mimo_interaction.h"
#include "../include/mimo_inverted_decoupling.h"
#include "../include/mimo_svd_decoupling.h"

int main(void) {
    printf("============================================================\n");
    printf("  CSTR Reactor — Inverted & SVD Decoupling Example\n");
    printf("============================================================\n\n");

    /* Create CSTR model */
    MIMOModel cstr;
    mimo_model_init(&cstr, 2, 2, "CSTR Reactor");

    mimo_model_set_fopdt(&cstr, 0, 0, 2.0, 5.0, 0.5);
    mimo_model_set_fopdt(&cstr, 0, 1, 0.8, 3.0, 0.0);
    mimo_model_set_fopdt(&cstr, 1, 0, 0.5, 8.0, 0.0);
    mimo_model_set_fopdt(&cstr, 1, 1, -3.0, 6.0, 1.0);

    printf("Model: CSTR Reactor\n");
    mimo_model_print(&cstr);
    printf("\n");

    /* RGA Analysis */
    double K[4];
    mimo_model_steady_state_gain(&cstr, K);

    RGAMatrix rga;
    mimo_rga_compute(K, 2, &rga);
    printf("--- RGA Analysis ---\n");
    mimo_rga_print(&rga);

    /* Inverted Decoupler Design */
    printf("\n--- Inverted Decoupler Design ---\n");
    InvertedDecoupler id;
    int ret = mimo_inverted_decoupler_design(&cstr, &id);
    if (ret == 0) {
        printf("Feedforward gains:\n");
        printf("  d12 = -G12/G11 = %.4f\n", id.feedforward[0][1].gain);
        printf("  d21 = -G21/G22 = %.4f\n", id.feedforward[1][0].gain);

        double robustness = mimo_inverted_robustness(&cstr, &id);
        printf("Robustness margin: %.4f (1.0 = perfectly robust)\n", robustness);

        /* Test algebraic loop resolution */
        printf("\n--- Signal Flow Test ---\n");
        double uc[] = { 0.05, -0.03 };
        double up[2];
        int iter = mimo_inverted_decoupler_step(&id, uc, up);
        printf("Controller outputs: [%.4f, %.4f]\n", uc[0], uc[1]);
        printf("Process inputs:     [%.4f, %.4f]\n", up[0], up[1]);
        printf("Iterations to resolve: %d\n", iter);
    }

    /* IMC-based Inverted Decoupler */
    printf("\n--- IMC-Based Inverted Decoupler ---\n");
    InvertedDecoupler id_imc;
    double lambda[] = { 2.0, 3.0 };
    ret = mimo_inverted_imc_decoupler(&cstr, &id_imc, lambda);
    if (ret == 0) {
        printf("IMC filter constants: lambda1=%.2f, lambda2=%.2f\n",
               lambda[0], lambda[1]);
        printf("Filtered feedforward gains:\n");
        printf("  d12_IMC = %.4f  d21_IMC = %.4f\n",
               id_imc.feedforward[0][1].gain,
               id_imc.feedforward[1][0].gain);
    }

    /* SVD Decoupler Design */
    printf("\n--- SVD Decoupler Design ---\n");
    SVDDecoupler svd_d;
    ret = mimo_svd_static_decoupler(&cstr, &svd_d);
    if (ret == 0) {
        printf("Singular values: sigma1=%.4f, sigma2=%.4f\n",
               svd_d.Sigma[0], svd_d.Sigma[1]);
        printf("Condition number: %.4f\n", svd_d.condition_number);
        printf("Effective rank: %.1f\n", svd_d.effective_rank);

        /* Apply SVD decoupler */
        double uc_svd[] = { 0.05, -0.03 };
        double up_svd[2];
        mimo_svd_decoupler_apply(&svd_d, uc_svd, up_svd);
        printf("SVD-decoupled process inputs: [%.4f, %.4f]\n",
               up_svd[0], up_svd[1]);
    }

    /* Principal Gains Alignment */
    printf("\n--- Principal Gains Alignment ---\n");
    double freqs[] = { 0.01, 0.1, 0.5, 1.0, 5.0 };
    double best_freq, misalignment;
    ret = mimo_principal_gains_alignment(&cstr, freqs, 5, &best_freq, &misalignment);
    if (ret == 0) {
        printf("Best decoupling frequency: %.4f rad/s\n", best_freq);
        printf("Misalignment angle: %.4f rad (%.1f deg)\n",
               misalignment, misalignment * 180.0 / M_PI);
        printf("(Closer to 0 = better decoupling alignment)\n");
    }

    printf("\n============================================================\n");
    printf("  Example complete.\n");
    printf("============================================================\n");

    return 0;
}
