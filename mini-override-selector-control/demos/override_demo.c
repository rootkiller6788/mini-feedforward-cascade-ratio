/**
 * @file override_demo.c
 * @brief Interactive demo of override selector control
 *
 * Demonstrates high-select, low-select, and median-select in action
 * with hysteresis and rate limiting. Users can see how override
 * control prevents constraint violations.
 */
#include <stdio.h>
#include <math.h>
#include "override_core.h"
#include "override_selector.h"
#include "override_pid.h"
#include "override_constraint.h"

int main(void) {
    printf("============================================\n");
    printf("  Override Selector Control — Live Demo\n");
    printf("============================================\n\n");

    /* Demo 1: High-select override (anti-surge) */
    printf("--- Demo 1: High-Select Override ---\n");
    printf("Scenario: Two controllers — Primary (P) and Anti-Surge (S)\n");
    printf("When S output > P output, S overrides via high-select.\n\n");

    double primary_outs[] = {50.0, 48.0, 45.0, 42.0, 40.0, 38.0};
    double surge_outs[]  = {30.0, 35.0, 40.0, 48.0, 55.0, 65.0};
    int valid[] = {1, 1, 1, 1, 1, 1};
    int n = 6;

    printf("Step  Primary  Surge   Selected  Mode\n");
    printf("----  -------  ------  --------  ---------\n");
    for (int i = 0; i < n; i++) {
        double vals[] = {primary_outs[i], surge_outs[i]};
        int idx;
        double sel = selector_high(vals, valid, 2, &idx);
        printf("%4d  %7.1f  %6.1f  %8.1f  %s\n",
               i, primary_outs[i], surge_outs[i], sel,
               (idx == 0) ? "PRIMARY" : "OVERRIDE (SURGE)");
    }

    /* Demo 2: Low-select override (valve position limit) */
    printf("\n--- Demo 2: Low-Select Override ---\n");
    printf("Scenario: Temp controller output vs Valve position limit\n\n");

    double temp_outs[] = {60.0, 65.0, 75.0, 85.0, 90.0, 95.0};
    double valve_limit = 80.0;

    printf("Step  TempOut  V-Limit  Selected  Mode\n");
    printf("----  -------  -------  --------  ---------\n");
    for (int i = 0; i < n; i++) {
        double vals[] = {temp_outs[i], valve_limit};
        int idx;
        double sel = selector_low(vals, valid, 2, &idx);
        printf("%4d  %7.1f  %7.1f  %8.1f  %s\n",
               i, temp_outs[i], valve_limit, sel,
               (idx == 0) ? "PRIMARY" : "OVERRIDE (LIMIT)");
    }

    /* Demo 3: Hysteresis prevents chattering */
    printf("\n--- Demo 3: Hysteresis Prevents Chatter ---\n");
    printf("Without hysteresis, small noise can cause rapid switching.\n\n");

    double noisy_vals[] = {50.0, 51.0, 49.5, 51.2, 50.3, 49.8};
    int prev_idx = -1;
    for (int i = 0; i < n; i++) {
        double vals[] = {noisy_vals[i], 50.0};
        int idx;
        double sel = selector_high_hysteresis(vals, valid, 2, prev_idx, 1.5, &idx);
        printf("  t=%d: vals=[%.1f, 50.0] selected=%.1f (idx=%d) prev_idx=%d\n",
               i, noisy_vals[i], sel, idx, prev_idx);
        prev_idx = idx;
    }
    printf("  Hysteresis band of 1.5 kept selection stable despite noise.\n");

    /* Demo 4: Constraint monitoring */
    printf("\n--- Demo 4: Constraint Approach Factor ---\n");
    constraint_def_t def;
    override_constraint_def_init(&def);
    constraint_def_set(&def, "TI-101.HI", "Reactor Temp High",
                       PRIORITY_CONSTRAINT, CONSTRAINT_HARD_ABS,
                       100.0, 0.0, 120.0, -10.0, 10.0, 5.0);
    constraint_state_t cs;
    override_constraint_state_init(&cs, &def);

    double test_vals[] = {50.0, 85.0, 92.0, 96.0, 98.0, 102.0, 110.0};
    for (int i = 0; i < 7; i++) {
        constraint_update(&cs, test_vals[i], 1.0);
        double af = constraint_approach_factor(&cs);
        printf("  Value=%.1f: AF=%.2f  %s\n",
               test_vals[i], af,
               cs.violating ? "[VIOLATED]" : cs.approaching ? "[APPROACHING]" : "[OK]");
    }

    printf("\n============================================\n");
    printf("  Demo Complete\n");
    printf("============================================\n");
    return 0;
}
