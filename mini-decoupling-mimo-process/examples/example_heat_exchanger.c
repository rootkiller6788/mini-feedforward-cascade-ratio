/**
 * @file example_heat_exchanger.c
 * @brief Heat Exchanger Network — 3x3 MIMO Decoupling Example
 *
 * L6 Canonical Problem: A network of two coupled heat exchangers plus
 * a bypass stream. Demonstrates decoupling for n>2 systems.
 *
 * This example covers:
 *   1. 3x3 RGA analysis
 *   2. Static decoupler for higher dimensions
 *   3. Simplified dynamic decoupler
 *   4. Lyapunov stability verification
 */

#include <stdio.h>
#include <math.h>
#include "../include/mimo_model.h"
#include "../include/mimo_interaction.h"
#include "../include/mimo_static_decoupling.h"
#include "../include/mimo_dynamic_decoupling.h"

int main(void) {
    printf("============================================================\n");
    printf("  Heat Exchanger Network — 3x3 MIMO Decoupling Example\n");
    printf("============================================================\n\n");

    /* 3x3 heat exchanger network:
     *
     * MV1: Hot stream flow HX1       → CV1: Cold stream outlet temp T1
     * MV2: Hot stream flow HX2       → CV2: Cold stream outlet temp T2
     * MV3: Bypass valve position     → CV3: Mixed stream temperature T3
     *
     * G(s) = [  3.0/(4s+1)   0.5/(6s+1)   0.2/(8s+1)   ]
     *        [  0.4/(7s+1)   2.5/(5s+1)   0.3/(9s+1)   ]
     *        [  0.1/(10s+1)  0.2/(12s+1)  1.8/(3s+1)   ]
     */

    MIMOModel hx;
    mimo_model_init(&hx, 3, 3, "Heat Exchanger Network");

    mimo_model_set_fopdt(&hx, 0, 0, 3.0, 4.0, 0.0);
    mimo_model_set_fopdt(&hx, 0, 1, 0.5, 6.0, 0.0);
    mimo_model_set_fopdt(&hx, 0, 2, 0.2, 8.0, 0.0);

    mimo_model_set_fopdt(&hx, 1, 0, 0.4, 7.0, 0.0);
    mimo_model_set_fopdt(&hx, 1, 1, 2.5, 5.0, 0.0);
    mimo_model_set_fopdt(&hx, 1, 2, 0.3, 9.0, 0.0);

    mimo_model_set_fopdt(&hx, 2, 0, 0.1, 10.0, 0.0);
    mimo_model_set_fopdt(&hx, 2, 1, 0.2, 12.0, 0.0);
    mimo_model_set_fopdt(&hx, 2, 2, 1.8, 3.0, 0.0);

    printf("Model: 3x3 Heat Exchanger Network\n");
    mimo_model_print(&hx);
    printf("McMillan degree: %d\n\n", mimo_model_mcmillan_degree(&hx));

    /* RGA Analysis */
    double K[9];
    mimo_model_steady_state_gain(&hx, K);
    printf("Steady-state gain matrix K:\n");
    for (int i = 0; i < 3; i++) {
        printf("  [ %8.4f  %8.4f  %8.4f ]\n",
               K[i*3], K[i*3+1], K[i*3+2]);
    }

    RGAMatrix rga;
    if (mimo_rga_compute(K, 3, &rga) == 0) {
        printf("\n");
        mimo_rga_print(&rga);

        double iq = mimo_interaction_quotient(&rga);
        printf("Interaction Quotient: %.4f\n", iq);
    }

    /* Pairing Recommendation */
    PairingSet pset;
    int n = mimo_enumerate_pairings(K, 3, &pset);
    printf("\n");
    mimo_pairing_print(&pset);

    /* Static Decoupler */
    printf("\n--- Static Decoupler (3x3) ---\n");
    StaticDecoupler sd;
    if (mimo_static_decoupler_design(&hx, &sd) == 0) {
        printf("Static decoupler D = K^{-1}:\n");
        for (int i = 0; i < 3; i++) {
            printf("  [ %8.4f  %8.4f  %8.4f ]\n",
                   sd.K_inv[i][0], sd.K_inv[i][1], sd.K_inv[i][2]);
        }
        printf("Condition number: %.4f\n", sd.condition_number);

        /* Apparent gain after decoupling */
        double Ka[9];
        mimo_static_apparent_gain(&hx, &sd, Ka);
        printf("\nApparent gain K_a = K * D:\n");
        for (int i = 0; i < 3; i++) {
            printf("  [ %8.4f  %8.4f  %8.4f ]\n",
                   Ka[i*3], Ka[i*3+1], Ka[i*3+2]);
        }
        printf("(Should be close to identity for good decoupling)\n");
    }

    /* Simplified Dynamic Decoupler */
    printf("\n--- Simplified Dynamic Decoupler ---\n");
    DynamicDecoupler dd;
    if (mimo_simplified_dynamic_decoupler(&hx, &dd) == 0) {
        printf("Decoupler gain matrix:\n");
        for (int i = 0; i < 3; i++) {
            printf("  [ %8.4f  %8.4f  %8.4f ]\n",
                   dd.base.elements[i][0].gain,
                   dd.base.elements[i][1].gain,
                   dd.base.elements[i][2].gain);
        }
        printf("Is causal: %s\n", dd.base.is_causal ? "YES" : "NO");

        /* Interaction metric */
        InteractionMetric metric;
        decoupler_interaction_metric(&hx, &dd.base, &metric);
        printf("After decoupling: %s\n", metric.summary);
    }

    /* Sensitivity Analysis */
    printf("\n--- Robustness to Gain Uncertainty ---\n");
    double sensitivity_10pct = mimo_static_sensitivity(&hx, &sd, 0.10);
    double sensitivity_20pct = mimo_static_sensitivity(&hx, &sd, 0.20);
    printf("Residual with 10%% gain error: %.6f\n", sensitivity_10pct);
    printf("Residual with 20%% gain error: %.6f\n", sensitivity_20pct);

    /* Lyapunov Stability for Apparent Process */
    printf("\n--- Lyapunov Stability Verification ---\n");
    /* Build approximate state matrix for decoupled system */
    double A_dec[9] = {
        -0.25,  0.0,   0.0,
         0.0,  -0.20,  0.0,
         0.0,   0.0,  -0.33
    };
    double P[9];
    bool stable = mimo_lyapunov_stability(A_dec, 3, P);
    printf("Decoupled system stable (Lyapunov): %s\n", stable ? "YES" : "NO");
    if (stable) {
        printf("Lyapunov matrix P diagonal entries: [%.4f, %.4f, %.4f]\n",
               P[0], P[4], P[8]);
    }

    /* Integrity check */
    printf("\n--- Integrity Against Loop Failures ---\n");
    int diag_pairing[] = { 0, 1, 2 };
    bool has_integrity = mimo_check_integrity(K, 3, diag_pairing);
    printf("Diagonal pairing has integrity: %s\n",
           has_integrity ? "YES" : "NO");

    printf("\n============================================================\n");
    printf("  Example complete.\n");
    printf("============================================================\n");

    return 0;
}
