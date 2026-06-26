/** @file example_temp_control.c
 * @brief Temperature Control with Gain-Scheduled PID (L6)
 *
 * Demonstrates gain-scheduled PID for a temperature control process
 * where the process gain and time constant vary significantly with
 * temperature. Uses Ziegler-Nichols design at each operating point
 * and linear interpolation between breakpoints.
 *
 * Process: Heat exchanger with nonlinear heat transfer.
 * Scheduling variable: Measured temperature (SCHED_VAR_TEMPERATURE)
 */
#include "../include/gain_schedule_core.h"
#include "../include/gain_schedule_interp.h"
#include "../include/gain_schedule_design.h"
#include "../include/gain_schedule_pid.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double simulate_process(double T, double u, double T_in, double dt) {
    double K = 0.8 + 0.004 * T;
    double tau = 15.0 - 0.05 * T;
    double L = 2.0;
    if (tau < 3.0) tau = 3.0;
    if (K < 0.5) K = 0.5;
    if (K > 2.0) K = 2.0;
    double a = exp(-dt / tau);
    static double T_prev = 25.0;
    static double u_delayed[40] = {0};
    static int delay_idx = 0;
    u_delayed[delay_idx] = u;
    int past_idx = (delay_idx - (int)(L/dt) + 40) % 40;
    if (past_idx < 0) past_idx += 40;
    double u_del = u_delayed[past_idx];
    delay_idx = (delay_idx + 1) % 40;
    double T_new = a * T_prev + K * (1.0 - a) * u_del + (1.0 - a) * T_in;
    T_prev = T_new;
    return T_new;
}

int main(void) {
    printf("\n========================================\n");
    printf("  Temperature Gain-Scheduled PID Demo\n");
    printf("========================================\n\n");
    
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_TEMPERATURE);
    table.interp_method = INTERP_CUBIC_HERMITE;
    
    double sv[] = {30.0, 50.0, 70.0, 90.0, 110.0};
    double K[]  = {0.92, 1.0, 1.08, 1.16, 1.24};
    double tau_arr[] = {13.5, 12.5, 11.5, 10.5, 9.5};
    double L[]  = {2.0, 2.0, 2.0, 2.0, 2.0};
    
    gs_design_frozen_parameter(&table, 4, sv, K, tau_arr, L, 5);
    gs_design_smooth_schedule(&table, 3);
    
    printf("Schedule Table (%u entries):\n", gs_table_count(&table));
    printf("  Temp [C]    Kp      Ki      Kd\n");
    printf("  ---------   ------  ------  ------\n");
    for (uint32_t i = 0; i < table.num_entries; i++) {
        printf("  %6.1f     %6.3f  %6.3f  %6.4f\n",
               table.entries[i].scheduling_value,
               table.entries[i].gains.Kp,
               table.entries[i].gains.Ki,
               table.entries[i].gains.Kd);
    }
    
    gs_pid_state_t pid;
    gs_pid_init(&pid, "TIC-HEX-101", GS_PID_ISA_STANDARD);
    gs_pid_set_dt(&pid, 0.5);
    gs_pid_set_saturation(&pid, 100.0, 0.0);
    
    double T = 25.0;
    double setpoint = 70.0;
    double T_in = 20.0;
    
    printf("\nSimulation: Setpoint=%.0fC, T_in=%.0fC\n\n", setpoint, T_in);
    printf("  Time[s]   T[C]      SP[C]     u[%%]     Kp      Ki\n");
    printf("  --------  --------  --------  --------  ------  ------\n");
    
    for (double t = 0.0; t <= 120.0; t += 0.5) {
        double u;
        gs_pid_update(&pid, &table, setpoint, T, T, &u);
        T = simulate_process(T, u, T_in, 0.5);
        
        if (fmod(t, 5.0) < 0.25) {
            printf("  %8.1f  %8.2f  %8.1f  %8.2f  %6.3f  %6.3f\n",
                   t, T, setpoint, u,
                   pid.Kp_current, pid.Ki_current);
        }
        
        if (t >= 60.0) setpoint = 90.0;
    }
    
    printf("\nFinal temperature: %.2f C\n", T);
    printf("Schedule switch count: %llu\n",
           (unsigned long long)gs_pid_get_switch_count(&pid));
    printf("Example complete.\n");
    return 0;
}
