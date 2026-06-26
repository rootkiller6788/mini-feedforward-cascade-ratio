/**
 * example_blending.c — Blending Process with Static Decoupling
 *
 * Demonstrates RGA, pairing, static decoupling for a 2x2 blending process.
 * Reference: Seborg, Edgar, Mellichamp (2016), Ch.18
 */

#include "mimo_model.h"
#include "mimo_interaction.h"
#include "mimo_static_decoupling.h"
#include "mimo_decoupling_common.h"
#include <stdio.h>

int main(void) {
    printf("=== Blending Process Decoupling ===\n\n");

    MIMOModel blend;
    mimo_blending_process_model(&blend);
    mimo_model_print(&blend);

    double K[4];
    mimo_model_steady_state_gain(&blend, K);
    printf("\nGain K = [%.2f %.2f; %.2f %.2f]\n", K[0], K[1], K[2], K[3]);

    RGAMatrix rga;
    mimo_rga_compute(K, 2, &rga);
    mimo_rga_print(&rga);

    double iq = mimo_interaction_quotient(&rga);
    printf("Interaction quotient IQ = %.3f (1.0 = decoupled)\n", iq);

    PairingSet ps;
    mimo_enumerate_pairings(K, 2, &ps);
    mimo_pairing_print(&ps);

    StaticDecoupler sd;
    if (mimo_static_decoupler_design(&blend, &sd) == 0) {
        printf("\nStatic decoupler D = K^{-1}:\n");
        printf("  D = [%.4f %.4f; %.4f %.4f]\n",
               sd.K_inv[0][0], sd.K_inv[0][1],
               sd.K_inv[1][0], sd.K_inv[1][1]);
        printf("  Condition number: %.2f\n", sd.condition_number);

        InteractionMetric metric;
        decoupler_interaction_metric(&blend, &sd.base, &metric);
        printf("  After decoupling: %s\n", metric.summary);
    }

    printf("\nBlending process has moderate interaction.\n");
    printf("Static decoupling effectively reduces cross-coupling.\n");
    return 0;
}
