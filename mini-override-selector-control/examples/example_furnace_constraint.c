/**
 * @file example_furnace_constraint.c
 * @brief L6 Example: Furnace Tube Skin Temperature Override
 *
 * A fired heater (furnace) has a primary outlet temperature controller
 * that adjusts fuel gas flow. A high-select override controller monitors
 * tube skin temperatures and reduces firing when any tube approaches its
 * maximum allowable temperature, preventing coking and tube rupture.
 *
 * This is a classic multi-constraint override scheme where the most
 * critical constraint (highest approach factor) takes control.
 *
 * Reference:
 *   Shinskey, F.G. (1996). Process Control Systems. Ch. 9.
 *   API 530/560: Fired Heaters for General Refinery Service.
 *
 * Knowledge Coverage: L6 — Furnace constraint control
 * Course: Purdue ME 575, Tsinghua process control
 * Industrial: Shell, ExxonMobil, Sinopec fired heater control
 */

#include <stdio.h>
#include <math.h>
#include "override_core.h"
#include "override_pid.h"
#include "override_constraint.h"

#define NUM_TUBES 4

int main(void) {
    printf("=== Furnace Tube Skin Temperature Override Control ===\n\n");

    override_controller_t temp_ctrl;
    override_controller_init(&temp_ctrl, 0, "TIC-101",
                             "Furnace Outlet Temp", PRIORITY_PRIMARY);
    temp_ctrl.params.Kc = 1.0;
    temp_ctrl.params.Ti = 180.0;
    temp_ctrl.params.u_min = 0.0;
    temp_ctrl.params.u_max = 100.0;
    temp_ctrl.active = 1;

    printf("Primary: Outlet Temp Controller (Kc=%.1f Ti=%.1f)\n\n",
           temp_ctrl.params.Kc, temp_ctrl.params.Ti);

    constraint_def_t tube_defs[NUM_TUBES];
    constraint_state_t tube_states[NUM_TUBES];

    for (int i = 0; i < NUM_TUBES; i++) {
        char tag[16];
        snprintf(tag, sizeof(tag), "TSH-%03d.HI", 201 + i);
        override_constraint_def_init(&tube_defs[i]);
        constraint_def_set(&tube_defs[i], tag, "Tube Skin Temperature High",
                           PRIORITY_CONSTRAINT, CONSTRAINT_HARD_ABS,
                           850.0, -273.0, 950.0, -273.0, 50.0, 10.0);
        override_constraint_state_init(&tube_states[i], &tube_defs[i]);
    }

    printf("Constraints: %d tube skin temperature limits (850 C max)\n", NUM_TUBES);
    printf("Override engages when any tube exceeds 800 C (50 C margin)\n\n");

    double outlet_temp = 650.0, outlet_sp = 700.0;
    double tube_temps[NUM_TUBES] = {720.0, 730.0, 715.0, 725.0};
    double dt = 1.0;

    printf("Time  Outlet  T1     T2     T3     T4     Max-AF  Override  Fire\n");
    printf("----  ------  -----  -----  -----  -----  ------  --------  ----\n");

    for (int t = 0; t <= 40; t += 1) {
        temp_ctrl.pv = outlet_temp;
        double fire_cmd = override_pid_update(&temp_ctrl, outlet_sp, outlet_temp, dt);

        double max_af = -1.0;
        for (int i = 0; i < NUM_TUBES; i++) {
            constraint_update(&tube_states[i], tube_temps[i], dt);
            double af = constraint_approach_factor(&tube_states[i]);
            if (af > max_af) { max_af = af; }
        }

        int override_active = 0;
        if (max_af >= 0.5) {
            override_active = 1;
            double scale = 1.0 / (1.0 + max_af);
            fire_cmd *= scale;
        }

        printf("%4d  %6.0f  %5.0f  %5.0f  %5.0f  %5.0f  %6.2f  %s  %5.1f\n",
               t, outlet_temp,
               tube_temps[0], tube_temps[1], tube_temps[2], tube_temps[3],
               max_af,
               override_active ? "[OVERRIDE]" : "Normal   ",
               fire_cmd);

        double heating = fire_cmd * 2.0;
        outlet_temp += 0.02 * (heating - 5.0);
        if (t > 15) { tube_temps[1] += 3.0 + (t - 15) * 1.5; tube_temps[3] += 2.0; }
        for (int i = 0; i < NUM_TUBES; i++) {
            tube_temps[i] += 0.1 * fire_cmd - 0.05 * (tube_temps[i] - 200.0);
        }
    }

    printf("\nWhen Tube 2 approaches 850 C limit, override reduces firing\n");
    printf("to protect tube integrity and prevent coking/carbon deposition.\n");
    printf("\n=== Example Complete ===\n");
    return 0;
}
