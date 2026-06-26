/**
 * @file example_boiler_override.c
 * @brief L6 Example: Boiler Pressure Override with Drum Level Constraint
 *
 * A power boiler has a primary firing rate controller for steam pressure.
 * A high-select override controller prevents drum level from dropping too
 * low (risk of tube overheating). A low-select override limits firing rate
 * to prevent excessive pressure. This is a classic double-override (high+low
 * select) configuration commonly found in power generation.
 *
 * Reference:
 *   Dukelow, S.G. (1991). The Control of Boilers (2nd ed.). ISA.
 *   Liptak, B.G. (2006). Instrument Engineers' Handbook, Vol. 2.
 *
 * Knowledge Coverage: L6 — Boiler combustion override
 * Course: Purdue ME 575, Tsinghua process control
 */

#include <stdio.h>
#include <math.h>
#include "override_core.h"
#include "override_pid.h"
#include "override_selector.h"

int main(void) {
    printf("=== Boiler Firing Rate Override Control (Double Override) ===\n\n");

    override_controller_t press_ctrl, level_ctrl, fuel_ctrl;
    override_controller_init(&press_ctrl, 0, "PIC-101",
                             "Steam Pressure", PRIORITY_PRIMARY);
    override_controller_init(&level_ctrl, 1, "LIC-101",
                             "Drum Level Low", PRIORITY_CONSTRAINT);
    override_controller_init(&fuel_ctrl, 2, "FIC-101",
                             "Fuel Flow Max", PRIORITY_CONSTRAINT);

    press_ctrl.params.Kc = 1.5; press_ctrl.params.Ti = 60.0;
    press_ctrl.params.u_min = 0.0; press_ctrl.params.u_max = 100.0;
    press_ctrl.active = 1;

    level_ctrl.params.Kc = 3.0; level_ctrl.params.Ti = 10.0;
    level_ctrl.params.u_min = 0.0; level_ctrl.params.u_max = 100.0;

    fuel_ctrl.params.Kc = 1.0; fuel_ctrl.params.Ti = 5.0;
    fuel_ctrl.params.u_min = 0.0; fuel_ctrl.params.u_max = 100.0;

    printf("Controllers:\n");
    printf("  [0] Steam Pressure (Primary): Kc=%.1f Ti=%.1f\n",
           press_ctrl.params.Kc, press_ctrl.params.Ti);
    printf("  [1] Drum Level Low (High-Select Override): Kc=%.1f Ti=%.1f\n",
           level_ctrl.params.Kc, level_ctrl.params.Ti);
    printf("  [2] Fuel Flow Max (Low-Select Override): Kc=%.1f Ti=%.1f\n\n",
           fuel_ctrl.params.Kc, fuel_ctrl.params.Ti);

    double pressure = 40.0, pressure_sp = 42.0;
    double drum_level = 55.0, drum_level_sp = 50.0;
    double dt = 1.0;

    printf("Time  Press    Lvl     P-Out   L-Out   F-Max   HS Sel  LS Sel  Final\n");
    printf("----  -------  ------  ------  ------  ------  ------  ------  -----\n");

    for (int t = 0; t <= 25; t += 1) {
        press_ctrl.pv = pressure;
        double p_out = override_pid_update(&press_ctrl, pressure_sp, pressure, dt);

        level_ctrl.setpoint = drum_level_sp;
        level_ctrl.pv = drum_level;
        level_ctrl.active = 1;
        double l_out = override_pid_update(&level_ctrl, drum_level_sp, drum_level, dt);

        fuel_ctrl.pv = p_out;
        fuel_ctrl.active = 1;
        double f_out = override_pid_update(&fuel_ctrl, 90.0, p_out, dt);

        double hs = (p_out > l_out) ? p_out : l_out;
        double ls = (hs < f_out) ? hs : f_out;

        printf("%4d  %7.1f  %6.1f  %6.1f  %6.1f  %6.1f  %6.1f  %6.1f  %6.1f\n",
               t, pressure, drum_level, p_out, l_out, f_out, hs, ls, ls);

        pressure += 0.02 * (ls / 100.0) * 2.0;
        if (t > 10) drum_level -= 3.0;
        if (drum_level < 40.0) {
            printf("  >>> DRUM LEVEL LOW! High-select override activates <<<\n");
        }
    }

    printf("\nDouble-override scheme ensures:\n");
    printf("  1. High-select: firing rate maintains drum level\n");
    printf("  2. Low-select: firing rate stays within fuel system max\n");
    printf("\n=== Example Complete ===\n");
    return 0;
}
