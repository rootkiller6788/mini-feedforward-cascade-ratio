/**
 * example_wood_berry.c ¡ª Wood-Berry Distillation Column Decoupling
 *
 * Demonstrates: RGA analysis, pairing selection, static decoupling,
 *               dynamic decoupling, and closed-loop performance comparison
 *               on the canonical Wood-Berry 2x2 distillation model.
 *
 * Reference: Wood & Berry (1973), Chem. Eng. Sci., 28:1707-1717
 *
 * Build: gcc -I../include -o wb_example example_wood_berry.c -L../build -lmimo_decoupling -lm
 */

#include "mimo_model.h"
#include "mimo_interaction.h"
#include "mimo_static_decoupling.h"
#include "mimo_dynamic_decoupling.h"
#include "mimo_inverted_decoupling.h"
#include "mimo_decoupling_common.h"
#include <stdio.h>
#include <math.h>

int main(void) {
    printf("=== Wood-Berry Distillation Column Decoupling Example ===

");

    /* Step 1: Build the Wood-Berry model */
    MIMOModel wb;
    mimo_wood_berry_model(&wb);
    printf("Step 1: Wood-Berry model initialized.
");
    mimo_model_print(&wb);

    /* Step 2: Compute RGA and analyze interaction */
    double K[4];
    mimo_model_steady_state_gain(&wb, K);
    printf("
Step 2: Steady-state gain matrix K =
");
    printf("  [ %8.4f  %8.4f ]
", K[0], K[1]);
    printf("  [ %8.4f  %8.4f ]
", K[2], K[3]);

    RGAMatrix rga;
    mimo_rga_compute(K, 2, &rga);
    printf("
");
    mimo_rga_print(&rga);
    printf("  Interpretation: Strong interaction ¡ª decoupling is essential.
");

    /* Step 3: Find best input-output pairing */
    PairingSet pset;
    mimo_enumerate_pairings(K, 2, &pset);
    mimo_pairing_print(&pset);

    /* Step 4: Design static decoupler */
    StaticDecoupler sd;
    if (mimo_static_decoupler_design(&wb, &sd) == 0) {
        printf("
Step 4: Static decoupler D = K^{-1}:
");
        printf("  Condition number: %.2f
", sd.condition_number);
        printf("  D = [ %8.4f  %8.4f ]
", sd.K_inv[0][0], sd.K_inv[0][1]);
        printf("      [ %8.4f  %8.4f ]
", sd.K_inv[1][0], sd.K_inv[1][1]);

        /* Verify K * D = I */
        double Ka[4];
        mimo_static_apparent_gain(&wb, &sd, Ka);
        printf("  K * D (should be I) = [ %8.4f  %8.4f ]
", Ka[0], Ka[1]);
        printf("                       [ %8.4f  %8.4f ]
", Ka[2], Ka[3]);
    }

    /* Step 5: Design ideal dynamic decoupler */
    DynamicDecoupler dd;
    if (mimo_ideal_dynamic_decoupler(&wb, &dd) == 0) {
        printf("
Step 5: Ideal dynamic decoupler:
");
        printf("  D_{11} = %.4f, D_{12} = %.4f (gain)
",
               dd.base.elements[0][0].gain, dd.base.elements[0][1].gain);
        printf("  D_{21} = %.4f, D_{22} = %.4f (gain)
",
               dd.base.elements[1][0].gain, dd.base.elements[1][1].gain);
        printf("  Proper: %s, Stable: %s
",
               dd.base.is_causal ? "yes" : "no",
               dd.base.is_stable ? "yes" : "no");
    }

    /* Step 6: Inverted decoupler */
    InvertedDecoupler id;
    if (mimo_inverted_decoupler_design(&wb, &id) == 0) {
        printf("
Step 6: Inverted decoupler:
");
        printf("  d_{12} = %.4f (feedforward u2->u1)
",
               id.feedforward[0][1].gain);
        printf("  d_{21} = %.4f (feedforward u1->u2)
",
               id.feedforward[1][0].gain);

        double u_c[] = {1.0, 0.0};  /* step in setpoint 1 */
        double u_p[2];
        mimo_inverted_decoupler_step(&id, u_c, u_p);
        printf("  Step response: u_process = [%.4f, %.4f]
", u_p[0], u_p[1]);
    }

    /* Step 7: Interaction metric after decoupling */
    InteractionMetric metric;
    decoupler_interaction_metric(&wb, &sd.base, &metric);
    printf("
Step 7: After static decoupling:
");
    printf("  %s
", metric.summary);

    decoupler_interaction_metric(&wb, &dd.base, &metric);
    printf("  After dynamic decoupling:
");
    printf("  %s
", metric.summary);

    /* Step 8: NI-based integrity analysis */
    int pairing_diag[] = {0, 1};
    int pairing_off[] = {1, 0};
    double ni_diag = mimo_niederlinski_index(K, 2, pairing_diag);
    double ni_off = mimo_niederlinski_index(K, 2, pairing_off);
    printf("
Step 8: Niederlinski integrity analysis:
");
    printf("  Diagonal (y1-R, y2-S): NI = %.4f %s
",
           ni_diag, ni_diag > 0 ? "(stable)" : "(UNSTABLE!)");
    printf("  Off-diagonal (y1-S, y2-R): NI = %.4f
", ni_off);

    /* Step 9: Sensitivity to gain uncertainty */
    double sens = mimo_static_sensitivity(&wb, &sd, 0.20);
    printf("
Step 9: Worst-case residual with 20%% gain error: %.4f
", sens);
    printf("  (Small value = robust, large value = sensitive)
");

    /* Step 10: Monte Carlo robustness */
    double Kc[] = {0.5, -0.1};
    double mc = mimo_monte_carlo_robustness(&wb, Kc, 500, 0.15);
    printf("
Step 10: Monte Carlo robustness (500 trials, 15%% uncert):
");
    printf("  %.1f%% of perturbed systems remain stable
", mc * 100.0);

    printf("
=== Analysis Complete ===
");
    printf("Wood-Berry column requires decoupling for good performance.
");
    printf("Recommended pairing: diagonal (R->yD, S->yB) with dynamic decoupling.
");

    return 0;
}
