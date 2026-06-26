/**
 * @file example_gas_liquid_reactor.c
 * @brief Example: Gas-Liquid CSTR Reactor Ratio Control
 *
 * Level: L6 Canonical Problems — Chemical Reactor Ratio Control
 *
 * Scenario: A gas-liquid CSTR reactor where gaseous reactant A
 * dissolves into the liquid phase and reacts (first-order kinetics).
 * The objective: maintain the liquid concentration of A at a
 * target setpoint by adjusting the gas flow rate (ratio control).
 *
 *   Gas A (Waste/feed) ──→ [CSTR Reactor] ──→ Product
 *   Liquid B ────────────→                  ──→ Effluent
 *
 * The gas-to-liquid ratio (G/L) is the manipulated variable.
 * The quality variable is C_A (concentration of A in liquid).
 * A ratio trim controller adjusts G/L based on C_A measurement.
 */

#include <stdio.h>
#include <math.h>

/* Type definitions */
typedef struct { double Kp_trim, Ti_trim, Ts, trim_min, trim_max, trim_dz, integrator, prev_error, output; int saturated; } ratio_trim_controller_t;

/* External functions */
extern void   gl_reactor_dynamic_step(double V_liquid, double V_gas, double Q_liquid_in, double Q_liquid_out, double Q_gas_in, double Q_gas_out, double C_A_in, double *C_A, double *P_A, double kLa, double kH, double k_reaction, double Ts);
extern double gl_reactor_steady_gas_flow(double C_A_target, double C_A_in, double kLa, double kH, double k_reaction, double V_liquid, double V_gas, double Q_liquid);
extern void   ratio_trim_init(ratio_trim_controller_t *trim, double Kp, double Ti, double Ts);
extern double ratio_trim_step(ratio_trim_controller_t *trim, double quality_sp, double quality_pv);

int main(void)
{
    printf("=== Gas-Liquid CSTR Ratio Control ===\n\n");

    /* Reactor parameters */
    double V_liquid = 100.0;     /* Liquid volume (L) */
    double V_gas    =  50.0;     /* Gas headspace volume (L) */
    double kLa      = 0.05;      /* Mass transfer coefficient (1/s) */
    double kH       = 0.034;     /* Henry's constant for CO2-like species */
    double k_reaction = 0.01;    /* First-order reaction rate (1/s) */
    double Q_liquid  = 10.0;     /* Liquid flow rate (L/s) */
    double C_A_in    = 0.02;     /* Inlet liquid concentration (mol/L) */

    printf("Reactor: V_L=%.0f L, V_G=%.0f L, kLa=%.3f 1/s, k=%.3f 1/s\n",
           V_liquid, V_gas, kLa, k_reaction);
    printf("Liquid flow: %.1f L/s, Feed conc: %.3f mol/L\n\n", Q_liquid, C_A_in);

    /* Target concentration */
    double C_A_target = 0.005;   /* mol/L */
    printf("Target C_A: %.4f mol/L\n", C_A_target);

    /* Compute steady-state gas flow for target concentration */
    double Q_gas_ss = gl_reactor_steady_gas_flow(C_A_target, C_A_in,
                                                   kLa, kH, k_reaction,
                                                   V_liquid, V_gas, Q_liquid);
    printf("Steady-state gas flow required: %.2f L/s\n", Q_gas_ss);
    printf("Gas/Liquid ratio (G/L): %.3f\n\n", Q_gas_ss / Q_liquid);

    /* Ratio trim controller: adjusts G/L based on C_A deviation */
    ratio_trim_controller_t trim;
    ratio_trim_init(&trim, 0.1, 30.0, 1.0);

    /* Simulation */
    double C_A = 0.01;  /* Initial concentration (above target) */
    double P_A = 0.1;   /* Initial partial pressure (atm) */
    double Q_gas = Q_gas_ss; /* Start at steady-state value */
    double Ts = 1.0;    /* 1 second time step */
    double sim_time = 0.0;

    printf("Time(s)   Q_gas    G/L_ratio  C_A(mol/L)  P_A(atm)  Trim\n");
    printf("------   ------    ---------  ----------  --------  -----\n");

    for (int step = 0; step <= 200; step++) {
        sim_time = step * Ts;

        /* Apply a disturbance at t=50s: reduce inlet concentration */
        double C_A_in_actual = C_A_in;
        if (sim_time >= 50.0 && sim_time < 100.0) {
            C_A_in_actual = 0.01; /* Feed dilution disturbance */
        }

        /* Reactor dynamics */
        gl_reactor_dynamic_step(V_liquid, V_gas,
                                 Q_liquid, Q_liquid,
                                 Q_gas, Q_gas,
                                 C_A_in_actual,
                                 &C_A, &P_A,
                                 kLa, kH, k_reaction, Ts);

        /* Ratio trim: adjust gas flow based on C_A error */
        double trim_output = ratio_trim_step(&trim, C_A_target, C_A);

        /* Apply trim to gas flow (±20% authority) */
        double trim_authority = 0.2 * Q_gas_ss;
        Q_gas = Q_gas_ss + trim_authority * (trim_output / 0.3);
        /* Clamp trim authority */
        if (Q_gas < 0.5 * Q_gas_ss) Q_gas = 0.5 * Q_gas_ss;
        if (Q_gas > 1.5 * Q_gas_ss) Q_gas = 1.5 * Q_gas_ss;

        /* Print every 20 steps */
        if (step % 20 == 0) {
            printf("%6.0f    %6.2f    %6.3f     %7.5f    %6.4f   %+.3f\n",
                   sim_time, Q_gas, Q_gas/Q_liquid, C_A, P_A, trim.output);
        }
    }

    /* Final state */
    printf("\n--- Final State ---\n");
    printf("C_A = %.5f mol/L (target: %.5f, error: %.5f)\n",
           C_A, C_A_target, C_A - C_A_target);
    printf("G/L ratio = %.3f\n", Q_gas / Q_liquid);

    double error_pct = fabs(C_A - C_A_target) / C_A_target * 100.0;
    printf("Concentration error: %.2f%%\n", error_pct);

    if (error_pct < 5.0)
        printf("STATUS: Tight ratio control achieved (<5%% error)\n");
    else
        printf("STATUS: Ratio trim needs tuning (error >5%%)\n");

    printf("\n=== Simulation Complete ===\n");
    return 0;
}
