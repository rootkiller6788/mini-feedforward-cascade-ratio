/**
 * example_rga_analysis.c — RGA and Dynamic RGA Analysis
 *
 * Demonstrates static RGA, dynamic RGA, effective RGA,
 * pairing selection, integrity checks, and condition number.
 * Reference: Bristol (1966), Skogestad & Postlethwaite (2005)
 */

#include "mimo_model.h"
#include "mimo_interaction.h"
#include <stdio.h>
#include <math.h>

int main(void) {
    printf("=== RGA Interaction Analysis ===\n\n");

    MIMOModel sys;
    mimo_model_init(&sys, 2, 2, "RGA Example");
    mimo_model_set_fopdt(&sys, 0, 0, 2.0, 10.0, 2.0);
    mimo_model_set_fopdt(&sys, 0, 1, 0.8, 5.0, 1.0);
    mimo_model_set_fopdt(&sys, 1, 0, 0.5, 6.0, 2.0);
    mimo_model_set_fopdt(&sys, 1, 1, 1.5, 12.0, 1.0);
    printf("2x2 system with interaction.\n\n");

    double K[4];
    mimo_model_steady_state_gain(&sys, K);

    RGAMatrix rga;
    mimo_rga_compute(K, 2, &rga);
    mimo_rga_print(&rga);

    double iq = mimo_interaction_quotient(&rga);
    printf("\nInteraction Quotient = %.3f\n", iq);

    double cn = mimo_condition_number(K, 2);
    printf("Condition number = %.2f\n", cn);

    PairingSet ps;
    mimo_enumerate_pairings(K, 2, &ps);
    mimo_pairing_print(&ps);

    /* Dynamic RGA */
    printf("\nDynamic RGA analysis:\n");
    double freqs[] = {0.0, 0.01, 0.1, 1.0};
    for (int f = 0; f < 4; f++) {
        DynamicRGA drga;
        if (mimo_dynamic_rga(&sys, freqs[f], &drga) == 0) {
            printf("  w=%.2f: max|DRGA| = [", freqs[f]);
            double mx = 0.0;
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    if (drga.rga_magnitude[i][j] > mx)
                        mx = drga.rga_magnitude[i][j];
            printf("%.3f]\n", mx);
        }
    }

    /* Effective RGA */
    double erga[4];
    mimo_effective_rga(&sys, 0.1, erga);
    printf("\nEffective RGA (bw=0.1):\n");
    printf("  [%.4f %.4f]\n  [%.4f %.4f]\n",
           erga[0], erga[1], erga[2], erga[3]);

    /* Integrity */
    int pair_diag[] = {0, 1};
    double ni = mimo_niederlinski_index(K, 2, pair_diag);
    bool integ = mimo_check_integrity(K, 2, pair_diag);
    printf("\nDiagonal pairing: NI=%.4f, integrity=%s\n",
           ni, integ ? "PASS" : "FAIL");

    return 0;
}
