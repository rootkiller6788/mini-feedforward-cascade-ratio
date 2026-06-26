/**
 * @file example_compressor_surge.c
 * @brief L6 Example: Compressor Anti-Surge Override Control
 *
 * A centrifugal compressor surge protection system using high-select
 * override. The primary controller regulates pressure; an override
 * opens the recycle valve when approaching the surge line.
 *
 * Reference:
 *   Nisenfeld & Seemann (1981). ISA.
 *   Boyce, M.P. (2012). Gas Turbine Engineering Handbook.
 *
 * Knowledge Coverage: L6 — Compressor surge protection
 * Course: Purdue ME 575, Tsinghua process control
 */

#include <stdio.h>
#include <math.h>
#include "override_core.h"

int main(void) {
    printf("=== Compressor Anti-Surge Override Control ===\n\n");

    surge_control_t surge;
    surge_control_init(&surge, 0.5, 20.0, 15.0);
    printf("Surge line: dP = %.2f * Q^2 + %.2f\n",
           surge.surge_line_slope, surge.surge_line_intercept);
    printf("Minimum surge margin: %.1f%%\n\n", surge.min_surge_margin);

    double flows[] = {10.0, 8.0, 6.0, 5.0, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0};
    int n = 10;

    printf("%-8s %-10s %-10s %-12s %-12s\n",
           "Flow", "Surge-dP", "Oper-dP", "Margin%%", "Override");
    printf("-------- ---------- ---------- ------------ ------------\n");

    for (int i = 0; i < n; i++) {
        double q = flows[i];
        double surge_dp = 0.5 * q * q + 20.0;
        double oper_dp = 0.6 * q * q + 25.0 + (10.0 - q) * 5.0;
        double margin = (1.0 - oper_dp / surge_dp) * 100.0;
        int override = (margin < 15.0) ? 1 : 0;
        printf("%-8.1f %-10.1f %-10.1f %-12.1f %s\n",
               q, surge_dp, oper_dp, margin,
               override ? "[OPEN RECYCLE]" : "Normal");
    }

    printf("\nAs flow decreases, the operating point approaches the surge line.\n");
    printf("When margin drops below 15%%, the anti-surge override opens recycle valve.\n");
    printf("\n=== Example Complete ===\n");
    return 0;
}
