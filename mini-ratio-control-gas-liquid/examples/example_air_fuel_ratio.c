/**
 * @file example_air_fuel_ratio.c
 * @brief Example: Air-Fuel Ratio Control for a Natural Gas Boiler
 *
 * Level: L6 Canonical Problems — Combustion Ratio Control
 *
 * This example demonstrates:
 *   1. Stoichiometric AFR computation for natural gas
 *   2. Cross-limiting for combustion safety during load changes
 *   3. Excess air control with O2 trim feedback
 *   4. Combustion efficiency monitoring
 *
 * Scenario: A natural gas boiler ramps from 50% to 100% load.
 * The cross-limiting controller ensures air always leads fuel,
 * maintaining safe (lean) combustion throughout the transient.
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>

/* Type definitions (same as headers) */
typedef enum { CROSS_LIMIT_NONE=0, CROSS_LIMIT_AIR_LEADS=1, CROSS_LIMIT_FUEL_LEADS=2, CROSS_LIMIT_DOUBLE=3 } cross_limit_mode_t;
typedef struct { cross_limit_mode_t mode; double afr_stoich, r_air_rich, r_fuel_rich, demand_air, demand_fuel, air_flow, fuel_flow, sp_air, sp_fuel; int air_high_selected, fuel_low_selected; uint64_t last_update_ms; } cross_limiting_t;
typedef struct { double lambda_excess_air, actual_afr, o2_flue_gas_pct, co_flue_gas_ppm, co2_flue_gas_pct, nox_flue_gas_ppm, stack_temp_c, combustion_efficiency_pct; int fuel_rich_alarm; } combustion_efficiency_t;

/* External functions */
extern double stoichiometric_afr_get(int fuel_code);
extern double lambda_from_afr(double AFR_actual, double AFR_stoich);
extern double excess_air_percent(double lambda);
extern double combustion_excess_air_from_o2(double target_o2_pct);
extern double combustion_afr_target(double afr_stoich, double excess_air_pct, double *lambda);
extern double combustion_efficiency_compute(const combustion_efficiency_t *eff, double T_ambient);
extern void   cross_limit_init(cross_limiting_t *cl, cross_limit_mode_t mode, double afr_stoich, double r_air_rich, double r_fuel_rich);
extern void   cross_limit_update_flows(cross_limiting_t *cl, double air_flow, double fuel_flow);
extern void   cross_limit_update_demands(cross_limiting_t *cl, double demand_air, double demand_fuel);
extern void   cross_limit_execute(cross_limiting_t *cl, double r_extra);
extern double cross_limit_current_afr(const cross_limiting_t *cl);
extern double cross_limit_lambda(const cross_limiting_t *cl);
extern int    cross_limit_check_safety(const cross_limiting_t *cl, double afr_actual);
extern void   cross_limit_margins(const cross_limiting_t *cl, double *margin_air, double *margin_fuel);

/* Simple process simulation: air and fuel flows track their setpoints
   with first-order dynamics */
static double process_update(double current, double setpoint, double tau, double Ts)
{
    double alpha = Ts / (tau + Ts);
    return current + alpha * (setpoint - current);
}

int main(void)
{
    printf("=== Air-Fuel Ratio Control: Natural Gas Boiler Load Ramp ===\n");
    printf("Reference: Tesla Gigafactory boiler control integration\n");
    printf("ISO 50001: Energy management systems — combustion optimization\n\n");

    /* --- Setup --- */
    double AFR_stoich = stoichiometric_afr_get(0); /* Natural gas: 17.2 */
    double excess_air_target_pct = 15.0; /* 15% excess air */
    double lambda_target;
    double AFR_target = combustion_afr_target(AFR_stoich, excess_air_target_pct, &lambda_target);

    printf("Fuel: Natural Gas (CH4)\n");
    printf("Stoichiometric AFR: %.1f kg_air / kg_fuel\n", AFR_stoich);
    printf("Target Excess Air:  %.1f%% (lambda = %.3f)\n", excess_air_target_pct, lambda_target);
    printf("Target AFR:         %.1f\n\n", AFR_target);

    /* --- Initialize cross-limiting --- */
    cross_limiting_t cl;
    cross_limit_init(&cl, CROSS_LIMIT_AIR_LEADS, AFR_stoich, 1.05, 0.95);

    /* Initial conditions at 50% load */
    double air_flow = 86.0;   /* kg/s (50% of full air) */
    double fuel_flow = 5.0;   /* kg/s (50% of full fuel, AFR=17.2) */
    double demand_air, demand_fuel;
    double tau_air = 2.0, tau_fuel = 1.0; /* air slower than fuel */
    double Ts = 0.5; /* 500 ms */
    double sim_time = 0.0;

    printf("Time(s)  Demand%%  Air_SP  Fuel_SP  Air_flow  Fuel_flow  AFR   Lambda  Safety\n");
    printf("------   -------  ------  -------  --------  ---------  ----  ------  ------\n");

    /* Simulate 60 seconds: ramp from 50% to 100% over 30s, hold */
    for (int step = 0; step <= 120; step++) {
        sim_time = step * Ts;

        /* Ramp load from 50% to 100% between t=5s and t=35s */
        double load_frac;
        if (sim_time < 5.0) {
            load_frac = 0.50;
        } else if (sim_time < 35.0) {
            load_frac = 0.50 + 0.50 * (sim_time - 5.0) / 30.0;
        } else {
            load_frac = 1.00;
        }

        /* Demand signals (unconstrained, from master pressure controller) */
        demand_air  = 172.0 * load_frac; /* Full-load air = 172 kg/s */
        demand_fuel =  10.0 * load_frac; /* Full-load fuel = 10 kg/s */

        /* Apply cross-limiting */
        cross_limit_update_flows(&cl, air_flow, fuel_flow);
        cross_limit_update_demands(&cl, demand_air, demand_fuel);
        cross_limit_execute(&cl, 1.05);

        /* Process simulation: flows track setpoints */
        air_flow  = process_update(air_flow,  cl.sp_air,  tau_air,  Ts);
        fuel_flow = process_update(fuel_flow, cl.sp_fuel, tau_fuel, Ts);

        /* Current AFR and lambda */
        double afr_actual = cross_limit_current_afr(&cl);
        double lambda_actual = cross_limit_lambda(&cl);
        int safety = cross_limit_check_safety(&cl, afr_actual);
        double m_air, m_fuel;
        cross_limit_margins(&cl, &m_air, &m_fuel);

        /* Print every 10 steps */
        if (step % 10 == 0) {
            printf("%6.1f   %6.0f%%  %6.1f  %6.2f   %7.1f   %7.2f    %5.1f  %6.3f    %d\n",
                   sim_time, load_frac*100,
                   cl.sp_air, cl.sp_fuel,
                   air_flow, fuel_flow,
                   afr_actual, lambda_actual, safety);
        }
    }

    /* --- Combustion efficiency analysis at final state --- */
    printf("\n--- Final State Efficiency Analysis ---\n");
    double lambda_final = cross_limit_lambda(&cl);
    double o2_pct = 21.0 * (1.0 - 1.0 / lambda_final); /* approximation */
    printf("Lambda: %.3f\n", lambda_final);
    printf("Estimated O2 in flue gas: %.1f%%\n", o2_pct);

    combustion_efficiency_t eff = {
        lambda_final, cross_limit_current_afr(&cl),
        o2_pct, 50.0, 10.0, 80.0, 180.0, 0.0, 0
    };
    double eta = combustion_efficiency_compute(&eff, 25.0);
    printf("Combustion Efficiency: %.1f%%\n", eta);

    double ea = combustion_excess_air_from_o2(o2_pct);
    printf("Excess Air: %.1f%%\n", ea);

    printf("\n=== Simulation Complete ===\n");
    return 0;
}
