/** @file example_ph_control.c
 * @brief pH Neutralization with Gain-Scheduled PID (L6/L7)
 *
 * Demonstrates gain-scheduled PID for pH neutralization control.
 * The pH process is highly nonlinear due to the logarithmic
 * relationship between pH and hydrogen ion concentration.
 * The titration curve has regions of high and low sensitivity.
 *
 * Process: Strong acid-strong base neutralization in a CSTR.
 * Scheduling variable: Measured pH value (SCHED_VAR_PH)
 *
 * References:
 *   McAvoy et al., "Dynamics of pH in CSTRs", IEC Res., 1972.
 *   Henson & Seborg, "Adaptive nonlinear control of a pH CSTR", CEP, 1994.
 */
#include "../include/gain_schedule_core.h"
#include "../include/gain_schedule_design.h"
#include "../include/gain_schedule_pid.h"
#include "../include/gain_schedule_stability.h"
#include <stdio.h>
#include <math.h>

static double ph_process(double pH, double u_base, double u_acid,
                          double dt, double *x_a, double *x_b) {
    (void)pH;
    double V = 10.0;
    double q = 0.1;
    double C_a_in = 0.1;
    double C_b_in = 0.1;
    double Kw = 1e-14;
    
    double xa = *x_a, xb = *x_b;
    
    double q_base = u_base;
    double q_acid = u_acid;
    
    double dxa = (q * C_a_in - (q + q_base + q_acid) * xa) / V;
    double dxb = (q_base * C_b_in - (q + q_base + q_acid) * xb) / V;
    
    xa += dxa * dt;
    xb += dxb * dt;
    
    if (xa < 0.0) xa = 0.0;
    if (xb < 0.0) xb = 0.0;
    
    double delta = xa - xb;
    double H = (delta + sqrt(delta*delta + 4.0*Kw)) / 2.0;
    double pH_new = -log10(H);
    if (pH_new < 0.0) pH_new = 0.0;
    if (pH_new > 14.0) pH_new = 14.0;
    
    *x_a = xa;
    *x_b = xb;
    return pH_new;
}

int main(void) {
    printf("\n========================================\n");
    printf("  pH Neutralization Gain-Scheduled PID\n");
    printf("========================================\n\n");
    
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_PH);
    table.interp_method = INTERP_AKIMA;
    
    /* Low gain near pH=7 (high sensitivity), high gain elsewhere */
    double sv[] = {2.0, 5.0, 7.0, 9.0, 12.0};
    double K_arr[]  = {0.05, 0.1, 0.5, 0.1, 0.05};
    double tau_arr[] = {50.0, 25.0, 10.0, 25.0, 50.0};
    double L_arr[] = {2.0, 1.0, 0.5, 1.0, 2.0};
    
    gs_design_frozen_parameter(&table, 0, sv, K_arr, tau_arr, L_arr, 5);
    
    printf("Schedule Table (%u entries):\n", gs_table_count(&table));
    printf("  pH     Kp      Ki      Kd\n");
    printf("  ----   ------  ------  ------\n");
    for (uint32_t i = 0; i < table.num_entries; i++) {
        printf("  %4.1f  %6.3f  %6.3f  %6.4f\n",
               table.entries[i].scheduling_value,
               table.entries[i].gains.Kp,
               table.entries[i].gains.Ki,
               table.entries[i].gains.Kd);
    }
    
    gs_pid_state_t pid;
    gs_pid_init(&pid, "AIC-pH-301", GS_PID_ISA_STANDARD);
    gs_pid_set_dt(&pid, 0.1);
    gs_pid_set_saturation(&pid, 0.5, 0.0);
    
    double xa = 0.01, xb = 0.0;
    double pH = 2.0;
    double setpoint = 7.0;
    
    printf("\nNeutralization: pH %.1f -> %.1f\n\n", pH, setpoint);
    printf("  Time[s]  pH      SP     Base[L/s]  Kp      Ki\n");
    printf("  -------  ------  -----  ---------  ------  ------\n");
    
    for (double t = 0.0; t <= 300.0; t += 0.1) {
        double u;
        gs_pid_update(&pid, &table, setpoint, pH, pH, &u);
        pH = ph_process(pH, u, 0.0, 0.1, &xa, &xb);
        
        if (fmod(t, 15.0) < 0.05) {
            printf("  %7.1f  %6.2f  %5.1f  %9.4f  %6.3f  %6.3f\n",
                   t, pH, setpoint, u,
                   pid.Kp_current, pid.Ki_current);
        }
    }
    
    printf("\nFinal pH: %.2f (target: %.1f)\n", pH, setpoint);
    printf("Schedule switches: %llu\n",
           (unsigned long long)gs_pid_get_switch_count(&pid));
    printf("Example complete.\n");
    return 0;
}
