/**
 * @file example_distillation_column.c
 * @brief Wood-Berry Distillation Column — Classic 2×2 MIMO Decoupling Example
 *
 * L6 Canonical Problem: Distillation column top/bottom composition control.
 *
 * The Wood-Berry column is the most famous MIMO control benchmark.
 * It separates methanol and water in a distillation column.
 *
 * Process:
 *   MV1: Reflux flow rate       → CV1: Overhead methanol composition (XD)
 *   MV2: Steam flow rate        → CV2: Bottom methanol composition (XB)
 *
 * Transfer function (Wood & Berry, 1973):
 *   G(s) = [ 12.8e^{-s}/(16.7s+1)   -18.9e^{-3s}/(21.0s+1)  ]
 *          [  6.6e^{-7s}/(10.9s+1)  -19.4e^{-3s}/(14.4s+1) ]
 *
 * This example demonstrates:
 *   1. RGA analysis for pairing selection
 *   2. Static decoupler design
 *   3. Ideal dynamic decoupler design
 *   4. Interaction metric comparison
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../include/mimo_model.h"
#include "../include/mimo_interaction.h"
#include "../include/mimo_static_decoupling.h"
#include "../include/mimo_dynamic_decoupling.h"

int main(void) {
    printf("============================================================\n");
    printf("  Wood-Berry Distillation Column — MIMO Decoupling Example\n");
    printf("============================================================\n\n");

    /* Step 1: Create the Wood-Berry model */
    MIMOModel wb;
    mimo_model_init(&wb, 2, 2, "Wood-Berry Distillation Column");

    mimo_model_set_fopdt(&wb, 0, 0, 12.8, 16.7, 1.0);
    mimo_model_set_fopdt(&wb, 0, 1, -18.9, 21.0, 3.0);
    mimo_model_set_fopdt(&wb, 1, 0, 6.6, 10.9, 7.0);
    mimo_model_set_fopdt(&wb, 1, 1, -19.4, 14.4, 3.0);

    printf("Model: Wood-Berry Distillation Column\n");
    mimo_model_print(&wb);
    printf("\n");

    /* Step 2: RGA Analysis */
    printf("--- RGA Analysis ---\n");
    double K[4];
    mimo_model_steady_state_gain(&wb, K);
    printf("Steady-state gain matrix K:\n");
    printf("  [ %8.4f  %8.4f ]\n", K[0], K[1]);
    printf("  [ %8.4f  %8.4f ]\n", K[2], K[3]);

    RGAMatrix rga;
    mimo_rga_compute(K, 2, &rga);
    mimo_rga_print(&rga);

    /* Step 3: Pairing Recommendation */
    printf("\n--- Pairing Recommendation ---\n");
    PairingSet pset;
    mimo_enumerate_pairings(K, 2, &pset);
    mimo_pairing_print(&pset);

    /* Step 4: Static Decoupler Design */
    printf("\n--- Static Decoupler Design ---\n");
    StaticDecoupler sd;
    int ret = mimo_static_decoupler_design(&wb, &sd);
    if (ret == 0) {
        printf("Static decoupler D = K^{-1}:\n");
        printf("  [ %8.4f  %8.4f ]\n", sd.K_inv[0][0], sd.K_inv[0][1]);
        printf("  [ %8.4f  %8.4f ]\n", sd.K_inv[1][0], sd.K_inv[1][1]);
        printf("Condition number of K: %.4f\n", sd.condition_number);
    } else {
        printf("K is singular — static decoupler not feasible.\n");
    }

    /* Step 5: Apply Static Decoupler */
    printf("\n--- Applying Static Decoupler ---\n");
    double uc_test[] = { 0.1, -0.05 };
    double up_test[2];
    mimo_static_decoupler_apply(&sd, uc_test, up_test);
    printf("Controller output uc = [%.4f, %.4f]\n", uc_test[0], uc_test[1]);
    printf("Process input   up = [%.4f, %.4f]\n", up_test[0], up_test[1]);

    /* Step 6: Dynamic Decoupler */
    printf("\n--- Ideal Dynamic Decoupler ---\n");
    DynamicDecoupler dd;
    ret = mimo_ideal_dynamic_decoupler(&wb, &dd);
    if (ret == 0) {
        printf("Dynamic decoupler gain matrix:\n");
        printf("  D[0,0]=%.4f  D[0,1]=%.4f\n",
               dd.base.elements[0][0].gain, dd.base.elements[0][1].gain);
        printf("  D[1,0]=%.4f  D[1,1]=%.4f\n",
               dd.base.elements[1][0].gain, dd.base.elements[1][1].gain);
        printf("Is proper: %s\n", dd.base.is_causal ? "YES" : "NO");
        printf("Is stable: %s\n", dd.base.is_stable ? "YES" : "NO");
    }

    /* Step 7: Interaction Metric */
    printf("\n--- Interaction After Decoupling ---\n");
    InteractionMetric metric;
    decoupler_interaction_metric(&wb, &dd.base, &metric);
    printf("%s\n", metric.summary);

    /* Step 8: Simulate closed-loop step response (open-loop approximation) */
    printf("\n--- Open-Loop Step Response Simulation ---\n");
    printf("Step in reflux (+0.1) with decoupling:\n");
    double uc_step[] = { 0.1, 0.0 };
    double up_dec[2];
    mimo_static_decoupler_apply(&sd, uc_step, up_dec);
    printf("  Process input: reflux = %.4f, steam = %.4f\n", up_dec[0], up_dec[1]);

    /* Approximate steady-state change */
    double dy1 = K[0] * up_dec[0] + K[1] * up_dec[1];
    double dy2 = K[2] * up_dec[0] + K[3] * up_dec[1];
    printf("  Predicted delta Y (steady state): XD = %.4f, XB = %.4f\n", dy1, dy2);
    printf("  (With perfect decoupling: XB should be approximately 0)\n");

    /* Without decoupling */
    double dy1_no = K[0] * 0.1 + K[1] * 0.0;
    double dy2_no = K[2] * 0.1 + K[3] * 0.0;
    printf("  Without decoupling: XD = %.4f, XB = %.4f\n", dy1_no, dy2_no);
    printf("  Interaction reduction: XB from %.4f to %.4f\n",
           fabs(dy2_no), fabs(dy2));

    printf("\n============================================================\n");
    printf("  Example complete.\n");
    printf("============================================================\n");

    return 0;
}
