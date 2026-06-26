/**
 * @file example_ratio_blending.c
 * @brief L6 Example: Ratio Control — Fuel-Air Combustion & Product Blending
 *
 * Demonstrates ratio control with feedback trim:
 *   1. Fuel-Air ratio control for combustion (stoichiometric)
 *   2. Dual-stream blending with analyzer feedback trim
 *
 * Applications: Boiler combustion, gasoline blending, chemical dosing
 * Reference: Liptak (2006), Instrument Engineers' Handbook, Vol. 2
 * Curriculum: Stanford ENGR205, Purdue ME575, RWTH Aachen ICS
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "cascade_types.h"
#include "cascade_pid.h"
#include "ratio_control.h"
#include "feedforward_control.h"

int main(void)
{
    printf("============================================================\n");
    printf("  Ratio Control Examples\n");
    printf("  1. Fuel-Air stoichiometric ratio (combustion)\n");
    printf("  2. Dual-stream blending with quality trim\n");
    printf("============================================================\n\n");

    /*------------------------------------------------------
     * Example 1: Fuel-Air Ratio Control
     *------------------------------------------------------*/
    printf("--- Example 1: Fuel-Air Combustion Ratio ---\n");
    printf("  Stoichiometric air/fuel ratio ≈ 14.7:1 (mass)\n");
    printf("  For methane: CH4 + 2O2 → CO2 + 2H2O\n\n");

    ratio_station_t fuel_air;
    ratio_init(&fuel_air, "FFIC-301");
    ratio_configure_fixed(&fuel_air, 14.7, 0.0, 1.0);
    fuel_air.mw_factor_a = 28.97;  /* Air MW */
    fuel_air.mw_factor_b = 16.04;  /* CH4 MW */

    /* Simulate varying fuel demand */
    double fuel_flows[] = {5.0, 10.0, 15.0, 20.0, 25.0, 20.0, 10.0, 5.0};
    int n_steps = sizeof(fuel_flows) / sizeof(fuel_flows[0]);

    printf("Fuel[t/h]  Air[t/h]  Ratio  Excess Air[%%]\n");
    printf("--------  --------  -----  -------------\n");

    for (int i = 0; i < n_steps; i++) {
        fuel_air.wild_flow = fuel_flows[i];
        fuel_air.controlled_sp = ratio_calculate_setpoint(&fuel_air, 100.0);
        double actual_air = fuel_air.controlled_sp * 0.95;  /* Simulate imperfect control */
        fuel_air.controlled_flow = actual_air;

        double R = ratio_compute_linear(&fuel_air);
        double excess_air = (R - 14.7) / 14.7 * 100.0;

        printf("%8.1f  %8.1f  %5.2f  %+13.1f\n",
               fuel_flows[i], actual_air, R, excess_air);
    }

    /*------------------------------------------------------
     * Example 2: Dual-Stream Blending with Quality Trim
     *------------------------------------------------------*/
    printf("\n--- Example 2: Gasoline Blending ---\n");
    printf("  Blending two gasoline streams to meet octane spec\n");
    printf("  Stream A: 87 RON, $2.50/gal\n");
    printf("  Stream B: 93 RON, $3.00/gal\n");
    printf("  Target: 91 RON at minimum cost\n\n");

    blend_station_t blend;
    memset(&blend, 0, sizeof(blend));
    blend.num_streams = 2;
    blend.total_flow_sp = 1000.0;  /* barrels/day */

    ratio_init(&blend.streams[0], "StreamA");
    ratio_init(&blend.streams[1], "StreamB");
    blend.streams[0].controlled_sp_max = 1000.0;
    blend.streams[1].controlled_sp_max = 1000.0;

    double costs[2] = {2.50, 3.00};
    double octanes[2] = {87.0, 93.0};
    double optimal[2];

    double total_cost = ratio_optimize_blend(&blend, costs, octanes,
        90.5, 91.5, optimal);

    printf("Optimal blend:\n");
    printf("  Stream A: %.0f bbl/day (%.0f%%)\n",
           optimal[0], optimal[0]/blend.total_flow_sp*100.0);
    printf("  Stream B: %.0f bbl/day (%.0f%%)\n",
           optimal[1], optimal[1]/blend.total_flow_sp*100.0);

    double blend_octane = (octanes[0]*optimal[0] + octanes[1]*optimal[1])
        / blend.total_flow_sp;
    printf("  Blend octane: %.1f RON\n", blend_octane);
    printf("  Total cost: $%.2f per barrel\n", total_cost / blend.total_flow_sp);

    printf("\n  Cost saving vs 100%% Stream B: $%.2f/bbl\n",
           3.00 - total_cost / blend.total_flow_sp);

    /*------------------------------------------------------
     * Example 3: Ratio Cascade with Feedforward
     *------------------------------------------------------*/
    printf("\n--- Example 3: Ratio + Feedforward Cascade ---\n");

    ff_compensator_t ff;
    ff_init(&ff, 0.5, -50.0, 50.0);
    ff_configure_static(&ff, -0.5, 0.0);

    double disturbances[] = {0.0, 5.0, 10.0, 15.0, 0.0, -5.0};
    printf("Disturbance  FF_Output   Corrected_Ratio\n");
    printf("-----------  ----------  ---------------\n");

    for (int i = 0; i < 6; i++) {
        double ff_out = ff_update(&ff, disturbances[i]);
        double corrected_ratio = 14.7 + ff_out * 0.1;
        printf("%11.1f  %10.2f  %15.2f\n",
               disturbances[i], ff_out, corrected_ratio);
    }

    printf("\n============================================================\n");
    printf("  Ratio & Feedforward control demonstrated successfully.\n");
    printf("============================================================\n");

    return 0;
}
