/**
 * @file example_blending_ratio.c
 * @brief Example: Multi-Component Blending with Cost Optimization
 *
 * Level: L6 Canonical Problems — Blending Ratio Control
 *         L8 Advanced Topics — Economic Blend Optimization
 *
 * Scenario: A gasoline blending operation mixes 3 components
 * (reformate, alkylate, butane) to meet octane specifications
 * at minimum cost. The ratio controller maintains component
 * ratios while the optimizer periodically adjusts targets
 * based on spot prices.
 *
 * Components:
 *   1. Reformate:  RON=98, cost=$2.50/gal
 *   2. Alkylate:   RON=93, cost=$2.20/gal
 *   3. Butane:     RON=92, cost=$1.80/gal
 *
 * Product requirement: RON ≥ 95, total flow = 1000 gal/h
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* Type definitions */
typedef struct { double *component_costs, *component_flows, *quality_coeffs; int n_components; double total_flow, quality_min, quality_max, optimal_cost; int feasible; } blend_optimizer_t;

/* External functions */
extern void   blend_optimizer_init(blend_optimizer_t *opt, int n_components);
extern void   blend_optimizer_set_costs(blend_optimizer_t *opt, const double *costs);
extern void   blend_optimizer_set_quality(blend_optimizer_t *opt, const double *quality);
extern int    blend_optimizer_solve(blend_optimizer_t *opt, int n_comp, double total_flow, double Q_min, double Q_max);
extern double blend_optimizer_get_flow(const blend_optimizer_t *opt, int component_i);
extern double blend_optimizer_get_cost(const blend_optimizer_t *opt);
extern int    blending_ratio_validate(const double *fractions, int n_components, double total_flow);
extern int    blending_ratio_setpoints(const double *fractions, int n_components, double total_flow, double *setpoints);

int main(void)
{
    printf("=== Gasoline Blending Ratio Optimization ===\n");
    printf("Reference: Toyota Production System (TPS) quality blending\n");
    printf("ISO 8217: Marine fuel blending specification\n\n");

    /* --- Scenario 1: Base Case --- */
    printf("--- Scenario 1: Base Case ---\n");
    printf("Components: Reformate ($2.50/gal, RON=98), Alkylate ($2.20/gal, RON=93), Butane ($1.80/gal, RON=92)\n");
    printf("Product: 1000 gal/h, RON >= 95\n\n");

    blend_optimizer_t opt;
    blend_optimizer_init(&opt, 3);

    double costs[3]  = {2.50, 2.20, 1.80};
    double rons[3]   = {98.0, 93.0, 92.0};
    blend_optimizer_set_costs(&opt, costs);
    blend_optimizer_set_quality(&opt, rons);

    int feasible = blend_optimizer_solve(&opt, 3, 1000.0, 95.0, 110.0);
    if (feasible) {
        printf("Optimal Blend Solution Found:\n");
        printf("  Reformate: %.0f gal/h (%.1f%%)\n",
               blend_optimizer_get_flow(&opt, 0),
               blend_optimizer_get_flow(&opt, 0) / 10.0);
        printf("  Alkylate:  %.0f gal/h (%.1f%%)\n",
               blend_optimizer_get_flow(&opt, 1),
               blend_optimizer_get_flow(&opt, 1) / 10.0);
        printf("  Butane:    %.0f gal/h (%.1f%%)\n",
               blend_optimizer_get_flow(&opt, 2),
               blend_optimizer_get_flow(&opt, 2) / 10.0);

        /* Verify quality */
        double actual_ron = (blend_optimizer_get_flow(&opt, 0) * 98.0 +
                             blend_optimizer_get_flow(&opt, 1) * 93.0 +
                             blend_optimizer_get_flow(&opt, 2) * 92.0) / 1000.0;
        printf("  Actual RON: %.1f (spec: >= 95.0)\n", actual_ron);
        printf("  Total Cost: $%.2f/h\n\n", blend_optimizer_get_cost(&opt));
    } else {
        printf("No feasible solution found!\n\n");
    }

    /* --- Scenario 2: Price Change --- */
    printf("--- Scenario 2: Alkylate Price Drop ($2.20 → $2.00) ---\n");

    double costs2[3] = {2.50, 2.00, 1.80};
    blend_optimizer_set_costs(&opt, costs2);
    feasible = blend_optimizer_solve(&opt, 3, 1000.0, 95.0, 110.0);

    if (feasible) {
        printf("New Optimal Blend:\n");
        printf("  Reformate: %.0f gal/h (%.1f%%)\n",
               blend_optimizer_get_flow(&opt, 0),
               blend_optimizer_get_flow(&opt, 0) / 10.0);
        printf("  Alkylate:  %.0f gal/h (%.1f%%)\n",
               blend_optimizer_get_flow(&opt, 1),
               blend_optimizer_get_flow(&opt, 1) / 10.0);
        printf("  Butane:    %.0f gal/h (%.1f%%)\n",
               blend_optimizer_get_flow(&opt, 2),
               blend_optimizer_get_flow(&opt, 2) / 10.0);
        printf("  Total Cost: $%.2f/h\n", blend_optimizer_get_cost(&opt));
        printf("  Savings vs Base: $%.2f/h ($%.0f/day)\n\n",
               250.0 * 24.0 - blend_optimizer_get_cost(&opt) * 24.0,
               250.0 * 24.0 - blend_optimizer_get_cost(&opt) * 24.0);
    }

    /* --- Scenario 3: Higher Octane Spec --- */
    printf("--- Scenario 3: Premium Grade (RON >= 98) ---\n");
    feasible = blend_optimizer_solve(&opt, 3, 1000.0, 98.0, 110.0);

    if (feasible) {
        printf("Premium Blend:\n");
        printf("  Reformate: %.0f gal/h (%.1f%%)\n",
               blend_optimizer_get_flow(&opt, 0),
               blend_optimizer_get_flow(&opt, 0) / 10.0);
        printf("  Alkylate:  %.0f gal/h (%.1f%%)\n",
               blend_optimizer_get_flow(&opt, 1),
               blend_optimizer_get_flow(&opt, 1) / 10.0);
        printf("  Butane:    %.0f gal/h (%.1f%%)\n",
               blend_optimizer_get_flow(&opt, 2),
               blend_optimizer_get_flow(&opt, 2) / 10.0);

        double ron = (blend_optimizer_get_flow(&opt, 0) * 98.0 +
                      blend_optimizer_get_flow(&opt, 1) * 93.0 +
                      blend_optimizer_get_flow(&opt, 2) * 92.0) / 1000.0;
        printf("  Actual RON: %.1f\n", ron);
        printf("  Total Cost: $%.2f/h\n", blend_optimizer_get_cost(&opt));

        /* Verify blending ratio validation */
        double fractions[3];
        for (int i = 0; i < 3; i++) {
            fractions[i] = blend_optimizer_get_flow(&opt, i) / 1000.0;
        }
        int valid = blending_ratio_validate(fractions, 3, 1000.0);
        printf("  Blend ratio valid: %s\n", valid ? "YES" : "NO");
    }

    /* Free allocated memory */
    if (opt.component_costs) free(opt.component_costs);
    if (opt.component_flows) free(opt.component_flows);
    if (opt.quality_coeffs)  free(opt.quality_coeffs);

    printf("\n=== Blending Optimization Complete ===\n");
    return 0;
}
