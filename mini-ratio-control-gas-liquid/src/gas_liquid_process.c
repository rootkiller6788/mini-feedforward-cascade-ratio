/**
 * @file gas_liquid_process.c
 * @brief Gas-liquid process models for ratio control.
 *
 * Level: L3 Engineering Structures + L4 Engineering Laws + L6 Canonical Problems
 *
 * Implements physical models for gas-liquid systems encountered
 * in ratio control applications:
 *   - Gas density (ideal gas law + compressibility)
 *   - Liquid density temperature compensation
 *   - Henry's Law gas-liquid equilibrium
 *   - Gas absorption/scrubbing column dynamics
 *   - Gas-liquid CSTR reactor dynamics
 *   - Two-phase flow in pipelines
 *   - Combustion efficiency and exhaust gas analysis
 *   - Gas-liquid separator design equations
 *
 * References:
 *   - Treybal, "Mass-Transfer Operations" (1980)
 *   - Perry's Chemical Engineers' Handbook (9th Ed)
 *   - Turns, "An Introduction to Combustion" (2012)
 *   - Poling, Prausnitz, O'Connell, "Properties of Gases and Liquids" (2001)
 */

#include "ratio_types.h"
#include "gas_liquid_process.h"
#include <math.h>
#include <string.h>

/* Universal gas constant: 8.314462618 J/(mol·K) */
#define R_GAS 8.314462618

/* Standard reference conditions for gas */
#define T_NORMAL 273.15   /* K (0°C) */
#define P_NORMAL 101325.0 /* Pa (1 atm) */

/* =========================================================================
 * L3: Gas Density — Ideal Gas Law
 * ========================================================================= */

/**
 * @brief Ideal gas density computation.
 *
 * ρ = P * M / (R * T)
 *
 * where:
 *   P = absolute pressure (Pa)
 *   M = molar mass (kg/mol)
 *   R = 8.314462618 J/(mol·K)
 *   T = absolute temperature (K)
 *
 * This is the foundation of all gas flow measurement correction.
 * Without density compensation, gas flow measurements can be off
 * by ±20% due to temperature and pressure variations alone.
 */
double gas_density_ideal(const gas_state_t *gas)
{
    if (gas == NULL) return 0.0;
    if (gas->temperature_k <= 0.0) return 0.0;
    if (gas->molar_mass <= 0.0) return 0.0;

    return (gas->pressure_pa * gas->molar_mass) /
           (R_GAS * gas->temperature_k);
}

/**
 * @brief Real gas density with compressibility correction.
 *
 * ρ = P * M / (Z * R * T)
 *
 * Compressibility factor Z accounts for non-ideal behavior.
 * For most gases at moderate pressures (< 10 bar), Z ≈ 1.
 * For high pressure or low temperature, Z can deviate significantly.
 *
 * Example Z values at standard conditions:
 *   Air:     Z ≈ 0.9999
 *   CO2:     Z ≈ 0.9945
 *   Natural gas: Z ≈ 0.95-0.98 (at pipeline pressure)
 *   Hydrogen: Z ≈ 1.0006
 */
double gas_density_real(const gas_state_t *gas)
{
    if (gas == NULL) return 0.0;
    if (gas->temperature_k <= 0.0) return 0.0;
    if (gas->molar_mass <= 0.0) return 0.0;

    double Z = gas->compressibility;
    if (Z <= 0.0) Z = 1.0; /* fallback to ideal if Z not set */

    return (gas->pressure_pa * gas->molar_mass) /
           (Z * R_GAS * gas->temperature_k);
}

/**
 * @brief Gas density with automatic ideal/real selection.
 */
double gas_density_compute(const gas_state_t *gas)
{
    if (gas == NULL) return 0.0;
    return gas->use_real_gas ? gas_density_real(gas) : gas_density_ideal(gas);
}

/**
 * @brief Convert gas flow from normal to actual conditions.
 *
 * Q_actual = Q_normal * (P_n / P_actual) * (T_actual / T_n)
 *
 * This is essential when ratio setpoints are specified at normal
 * conditions (0°C, 1 atm) but flow meters read at actual process
 * conditions (e.g., 150°C, 500 kPa).
 */
double gas_flow_normal_to_actual(double flow_normal,
                                  double P_actual, double T_actual,
                                  double P_n, double T_n)
{
    if (P_actual <= 0.0 || T_n <= 0.0) return 0.0;
    return flow_normal * (P_n / P_actual) * (T_actual / T_n);
}

double gas_flow_actual_to_normal(double flow_actual,
                                  double P_actual, double T_actual,
                                  double P_n, double T_n)
{
    if (P_n <= 0.0 || T_actual <= 0.0) return 0.0;
    return flow_actual * (P_actual / P_n) * (T_n / T_actual);
}

/* =========================================================================
 * L3: Liquid Density Temperature Compensation
 * ========================================================================= */

/**
 * @brief Temperature-compensated liquid density.
 *
 * ρ(T) = ρ_ref * (1 - β * (T - T_ref))
 *
 * This linear model is accurate for moderate temperature ranges
 * (±50°C from reference). For wider ranges, polynomial or
 * modified Rackett equation is recommended.
 *
 * Water example:
 *   ρ(20°C) = 998.2 kg/m³
 *   ρ(80°C) = 998.2 * (1 - 2.1e-4 * 60) = 985.5 kg/m³
 *
 * The 13 kg/m³ difference (1.3%) matters for custody transfer
 * and precise ratio control.
 */
double liquid_density_compensate(const liquid_density_t *liq)
{
    if (liq == NULL) return 0.0;
    if (liq->density_ref <= 0.0) return 0.0;

    double dT = liq->temperature_c - liq->temp_ref;
    return liq->density_ref * (1.0 - liq->expansion_coeff * dT);
}

/**
 * @brief Get liquid thermal expansion coefficient β.
 *
 * Returns 1/K for common industrial liquids.
 * β values vary significantly:
 *   - Water: 2.1e-4 /K (very low — stable density)
 *   - Organics: ~1e-3 /K (5× more sensitive than water)
 *   - Cryogenics: higher still
 *
 * This lookup table enables ratio control systems to automatically
 * apply correct density compensation based on the fluid name.
 */
double liquid_expansion_coeff(const char *substance)
{
    if (substance == NULL) return -1.0;

    if (strcmp(substance, "water") == 0)      return 2.1e-4;
    if (strcmp(substance, "ethanol") == 0)    return 1.1e-3;
    if (strcmp(substance, "methanol") == 0)   return 1.2e-3;
    if (strcmp(substance, "benzene") == 0)    return 1.2e-3;
    if (strcmp(substance, "oil_light") == 0)  return 7.0e-4;
    if (strcmp(substance, "oil_heavy") == 0)  return 5.0e-4;
    if (strcmp(substance, "acetone") == 0)    return 1.4e-3;
    if (strcmp(substance, "glycerol") == 0)   return 4.8e-4;
    if (strcmp(substance, "ammonia") == 0)    return 2.4e-3;
    if (strcmp(substance, "sulfuric_acid") == 0) return 5.5e-4;

    return -1.0; /* substance not found */
}

/* =========================================================================
 * L4: Henry's Law — Gas-Liquid Equilibrium
 * ========================================================================= */

/**
 * @brief Compute Henry's Law equilibrium concentration.
 *
 * C_eq = kH * P_gas
 *
 * Henry's Law states that at constant temperature, the amount of
 * gas dissolved in a liquid is proportional to its partial pressure
 * above the liquid.
 *
 * This is the fundamental relationship for:
 *   - Gas absorption column design
 *   - Gas-liquid reactor mass transfer
 *   - Dissolved oxygen control in fermentation
 *   - Carbonation control in beverage production
 *
 * Temperature dependence of kH (van't Hoff equation):
 *   kH(T) = kH(T0) * exp( -ΔH_sol/R * (1/T - 1/T0) )
 *
 * where ΔH_sol is the enthalpy of dissolution.
 */
double henry_equilibrium(const gl_equilibrium_t *eq)
{
    if (eq == NULL) return 0.0;
    return eq->henry_constant * eq->gas_partial_pressure;
}

/**
 * @brief Compute mass transfer rate.
 *
 * N = kLa * V * (C_eq - C_bulk)
 *
 * The driving force for mass transfer is the difference between
 * equilibrium concentration (at the gas-liquid interface) and
 * bulk liquid concentration. kLa is the overall volumetric
 * mass transfer coefficient.
 *
 * kLa depends on:
 *   - Gas superficial velocity (higher → more turbulence → higher kLa)
 *   - Liquid properties (viscosity, surface tension)
 *   - Column internals (packing type, tray design)
 *   - Bubble size distribution
 *
 * Positive N → absorption (gas dissolving into liquid)
 * Negative N → desorption/stripping (gas leaving liquid)
 */
double mass_transfer_rate(const gl_equilibrium_t *eq, double C_bulk)
{
    if (eq == NULL) return 0.0;

    double C_eq = henry_equilibrium(eq);
    return eq->mass_transfer_coeff * eq->liquid_volume * (C_eq - C_bulk);
}

/**
 * @brief Compute required L/G ratio for target removal efficiency.
 *
 * For dilute countercurrent absorption:
 *   (L/G)_min = (y_in - y_out) / (x_eq(y_in) - x_in)
 *
 * where:
 *   y_in/y_out = gas-phase mole fractions (in/out)
 *   x_in       = liquid-phase mole fraction at inlet
 *   x_eq       = equilibrium liquid mole fraction at gas inlet
 *
 * Using Henry's Law: x_eq = y * P_total / (kH * C_total_liquid)
 *
 * This L/G is the minimum theoretical requirement (infinite column).
 * Actual columns operate at 1.2-2.0 × minimum L/G.
 */
double absorber_liquid_gas_ratio(double y_in, double y_out,
                                  double x_in, double kH, double P_total)
{
    if (kH <= 0.0 || P_total <= 0.0) return 0.0;

    /* Convert mole fractions to concentrations */
    double C_total_liquid = 55.5; /* mol/L for water at 25°C */
    double x_eq_in = y_in * P_total / (kH * C_total_liquid);

    double denominator = x_eq_in - x_in;
    if (denominator <= 0.0) return 0.0; /* driving force negative or zero */

    return (y_in - y_out) / denominator;
}

/**
 * @brief Compute Number of Transfer Units (NTU) for gas absorption.
 *
 * For dilute systems with linear equilibrium (Henry's Law):
 *   NTU = ln( (y_in - m*x_in) / (y_out - m*x_in) ) / (1 - m*G/L)
 *
 * where m = kH * P_total / C_total (equilibrium slope in mole fractions).
 *
 * For the special case m*G/L = 1 (pinch condition):
 *   NTU = (y_in - y_out) / (y_out - m*x_in)  [L'Hospital's rule limit]
 */
double absorber_ntu(double y_in, double y_out, double x_in,
                     double kH, double P_total, double L_G_ratio)
{
    if (L_G_ratio <= 0.0 || kH <= 0.0 || P_total <= 0.0) return 0.0;

    double C_total = 55.5; /* mol/L for water */
    double m = kH * P_total / C_total; /* equilibrium slope */

    double num = y_in - m * x_in;
    double den = y_out - m * x_in;

    if (num <= 0.0 || den <= 0.0) return 0.0; /* no driving force */

    double absorption_factor = m / L_G_ratio;

    /* Handle pinch condition: AF = 1 */
    if (fabs(absorption_factor - 1.0) < 1e-6) {
        return num / den - 1.0;
    }

    /* Standard NTU formula */
    return log(num / den) / (1.0 - absorption_factor);
}

/**
 * @brief Dynamic absorber step.
 *
 * Simplified lumped-parameter dynamic model for a gas absorber.
 *
 * Mass balance (liquid phase):
 *   dC_L/dt = (G/V)*(C_G_in - C_G_out) + kLa*(C_eq - C_L)
 *
 * Assuming gas-phase dynamics are fast (pseudo-steady state):
 *   C_G_out ≈ C_G_in - (L/G)*(C_L_out - C_L_in)
 *
 * Discretized with forward Euler:
 *   C_L(k+1) = C_L(k) + Ts * [terms]
 */
double absorber_dynamic_step(const gl_equilibrium_t *eq,
                              double Q_gas, double Q_liquid,
                              double C_gas_in, double *C_liquid, double Ts)
{
    (void)Q_liquid; /* reserved for future liquid-phase convection term */
    if (eq == NULL || C_liquid == NULL) return 0.0;
    if (eq->liquid_volume <= 0.0) return *C_liquid;

    double C_eq = henry_equilibrium(eq);

    /* Mass transfer term */
    double mt_term = eq->mass_transfer_coeff * (C_eq - *C_liquid);

    /* Convection term (gas inflow, liquid outflow carry) */
    double conv_term = 0.0;
    if (eq->liquid_volume > 0.0) {
        conv_term = (Q_gas / eq->liquid_volume) * C_gas_in;
    }

    /* Forward Euler update */
    double dC = (conv_term + mt_term) * Ts;
    *C_liquid += dC;

    /* Physical constraint: concentration cannot be negative */
    if (*C_liquid < 0.0) *C_liquid = 0.0;

    return *C_liquid;
}

/* =========================================================================
 * L6: Gas-Liquid CSTR Reactor Dynamics
 * ========================================================================= */

/**
 * @brief Dynamic step for gas-liquid CSTR.
 *
 * Models a continuous stirred-tank reactor with gas-liquid mass transfer
 * and first-order chemical reaction in the liquid phase.
 *
 * Liquid phase balance:
 *   dC_A/dt = (Q_in*C_Ain - Q_out*C_A)/V_L + kLa*(C_eq - C_A) - k*C_A
 *
 * Gas phase balance (assuming well-mixed gas, component A):
 *   dP_A/dt = (G_in*P_Ain - G_out*P_A)/V_G - kLa*(C_eq - C_A)*R*T
 *
 * where C_eq = kH * P_A (Henry's Law linking gas and liquid).
 *
 * The gas-liquid ratio directly affects reactor performance:
 *   - Higher G/L → more gas available → higher C_eq → faster reaction
 *   - Lower G/L → gas becomes limiting → reaction rate decreases
 */
void gl_reactor_dynamic_step(double V_liquid, double V_gas,
                              double Q_liquid_in, double Q_liquid_out,
                              double Q_gas_in, double Q_gas_out,
                              double C_A_in, double *C_A, double *P_A,
                              double kLa, double kH, double k_reaction,
                              double Ts)
{
    if (C_A == NULL || P_A == NULL) return;
    if (V_liquid <= 0.0 || V_gas <= 0.0) return;

    double C_eq = kH * (*P_A);

    /* Liquid phase: dC_A/dt */
    double inflow_term    = (Q_liquid_in > 0 && V_liquid > 0) ?
                             (Q_liquid_in * C_A_in) / V_liquid : 0.0;
    double outflow_term   = (Q_liquid_out > 0 && V_liquid > 0) ?
                             (Q_liquid_out * (*C_A)) / V_liquid : 0.0;
    double mass_transfer  = kLa * (C_eq - (*C_A));
    double reaction       = k_reaction * (*C_A);

    double dC_A_dt = inflow_term - outflow_term + mass_transfer - reaction;

    /* Gas phase: dP_A/dt */
    double gas_inflow     = (Q_gas_in > 0 && V_gas > 0) ?
                             (Q_gas_in * (*P_A)) / V_gas : 0.0;
    double gas_outflow    = (Q_gas_out > 0 && V_gas > 0) ?
                             (Q_gas_out * (*P_A)) / V_gas : 0.0;
    double gas_mt         = kLa * (C_eq - (*C_A)) * R_GAS * 298.15 / V_gas;

    double dP_A_dt = gas_inflow - gas_outflow - gas_mt;

    /* Euler integration */
    *C_A += dC_A_dt * Ts;
    *P_A += dP_A_dt * Ts;

    /* Clamp to physical bounds */
    if (*C_A < 0.0) *C_A = 0.0;
    if (*P_A < 0.0) *P_A = 0.0;
}

/**
 * @brief Compute steady-state gas flow for target concentration.
 *
 * Sets dC_A/dt = 0 and solves for Q_gas:
 *
 * At steady state:
 *   0 = (Q_l*C_Ain - Q_l*C_A)/V_L + kLa*(kH*P_A - C_A) - k*C_A
 *   0 = (Q_g*P_Ain - Q_g*P_A)/V_G - kLa*(kH*P_A - C_A)*R*T
 *
 * For the simple case (Q_in = Q_out, G_in = G_out, P_Ain = 0):
 *   Q_gas = (k*C_A_target * V_L) / (kH * P_A_target)
 *
 * where P_A_target = (C_A_target + (Q_l*(C_A_target - C_Ain)/(V_L*kLa) + k*C_A_target/kLa)) / kH
 *
 * This function computes the gas flow needed to maintain a target
 * liquid concentration, which is the primary ratio control objective
 * in gas-liquid reactors.
 */
double gl_reactor_steady_gas_flow(double C_A_target, double C_A_in,
                                   double kLa, double kH, double k_reaction,
                                   double V_liquid, double V_gas, double Q_liquid)
{
    if (kLa <= 0.0 || kH <= 0.0 || V_liquid <= 0.0) return 0.0;

    /* From liquid phase steady state, compute required C_eq */
    double tau = (Q_liquid > 0.0) ? V_liquid / Q_liquid : 1e6;
    double C_eq_required = C_A_target +
        (C_A_target * (1.0/tau + k_reaction) - C_A_in / tau) / kLa;

    if (C_eq_required <= 0.0) return 0.0;

    /* From C_eq = kH * P_A, compute required P_A */
    double P_A_required = C_eq_required / kH;

    /* From gas phase steady state, compute required gas flow */
    /* Simplified: G = V_G * kLa * (C_eq - C_A) / (kH * (P_Ain - P_A)) */
    /* Assumes fresh gas has P_Ain = 0 for the species */
    if (V_gas <= 0.0) V_gas = V_liquid; /* assume equal volumes if not specified */

    double mass_transfer = kLa * (C_eq_required - C_A_target);
    if (mass_transfer <= 0.0) return 0.0;

    /* Q_gas = mass_transfer * V_liquid / (kH * P_A_required) * (V_gas/V_liquid) */
    /* Simplified: Q_gas needed to maintain P_A_required */
    double Q_gas = mass_transfer * V_gas / (kH * P_A_required);

    return Q_gas > 0.0 ? Q_gas : 0.0;
}

/* =========================================================================
 * L6: Two-Phase Flow in Pipelines
 * ========================================================================= */

/**
 * @brief Compute void fraction (gas volume fraction).
 *
 * Homogeneous model (no slip between phases):
 *   α = β = Q_gas / (Q_gas + Q_liquid)
 *
 * This is the simplest two-phase flow model. It assumes both
 * phases travel at the same velocity. Valid only for:
 *   - Dispersed bubble flow (small bubbles well-mixed)
 *   - Mist flow (small droplets well-dispersed)
 */
double two_phase_void_fraction(const two_phase_flow_t *tf,
                                double C0, double U_drift, double pipe_area)
{
    if (tf == NULL) return 0.0;

    double Q_total = tf->gas_flow_vol + tf->liquid_flow_vol;
    if (Q_total <= 0.0) return 0.0;

    /* Drift-flux model (Zuber-Findlay, 1965) */
    double j_total = Q_total / pipe_area;
    if (j_total <= 0.0) return 0.0;

    double alpha = tf->gas_flow_vol / (C0 * Q_total + pipe_area * U_drift);

    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    return alpha;
}

/**
 * @brief Estimate two-phase flow regime.
 *
 * Uses simplified Taitel-Dukler (1976) criteria based on
 * gas and liquid superficial velocities.
 *
 * Superficial velocity = volumetric flow / pipe cross-sectional area.
 *
 * Regime transitions:
 *   Bubbly → Slug:    Gas bubbles coalesce → large Taylor bubbles
 *   Slug → Annular:   Gas core becomes continuous → liquid film on wall
 *   Annular → Mist:   Liquid film destabilizes → droplets entrained in gas
 */
int two_phase_flow_regime(const two_phase_flow_t *tf,
                           double pipe_diam, double rho_gas,
                           double rho_liquid, double sigma)
{
    if (tf == NULL || pipe_diam <= 0.0) return 0;

    double A = 3.14159265359 * pipe_diam * pipe_diam / 4.0;
    double j_gas = tf->gas_flow_vol / A;

    /* Simplified Taitel-Dukler regime transition criteria */
    double g = 9.81;

    /* Bubbly to slug transition: gas void fraction > 0.25 */
    double alpha_homogeneous = tf->gas_flow_vol /
        (tf->gas_flow_vol + tf->liquid_flow_vol);
    if (alpha_homogeneous < 0.25 && j_gas < 0.5) return 0; /* Bubbly */

    /* Slug to annular transition: gas velocity high enough to lift liquid */
    /* Kutateladze number criterion */
    double Ku = j_gas * sqrt(rho_gas / (g * sigma * (rho_liquid - rho_gas)));
    if (Ku < 3.1) return 1; /* Slug/Churn */

    /* Annular to mist transition */
    if (j_gas > 15.0) return 3; /* Mist */
    return 2; /* Annular */
}

/**
 * @brief Lockhart-Martinelli two-phase multiplier.
 *
 * Φ²_L = 1 + C/X + 1/X²
 *
 * where:
 *   X = sqrt(ΔP_liquid / ΔP_gas) = Lockhart-Martinelli parameter
 *   C = Chisholm coefficient (depends on flow regime)
 *
 * C values (Chisholm correlation):
 *   Gas turbulent, Liquid turbulent:  C = 20
 *   Gas turbulent, Liquid viscous:    C = 12
 *   Gas viscous,    Liquid turbulent: C = 10
 *   Gas viscous,    Liquid viscous:   C =  5
 *
 * Φ²_L is always ≥ 1, meaning two-phase pressure drop
 * is always greater than single-phase liquid pressure drop.
 */
double lockhart_martinelli_multiplier(double X, double C_coeff)
{
    if (X <= 0.0) return 1.0;

    double invX = 1.0 / X;
    return 1.0 + C_coeff * invX + invX * invX;
}

/* =========================================================================
 * L6: Combustion Efficiency Model
 * ========================================================================= */

/**
 * @brief Compute combustion efficiency from flue gas analysis.
 *
 * Uses the Siegert formula (simplified stack loss method):
 *
 *   stack_loss_pct = (T_stack - T_amb) * (A1 / CO2_pct + B1)
 *
 * where A1, B1 are fuel-dependent constants:
 *   Natural gas:  A1 = 0.37, B1 = 0.009
 *   Fuel oil:     A1 = 0.43, B1 = 0.010
 *   Coal:         A1 = 0.62, B1 = 0.011
 *
 * Then:
 *   η = 100 - stack_loss - radiation_loss(1-2%) - unburned_loss
 *
 * This formula is implemented in most handheld combustion analyzers
 * (e.g., Testo, Bacharach, Kane) and DCS combustion control blocks.
 *
 * Reference: ASME PTC 4-2013, "Fired Steam Generators"
 */
double combustion_efficiency_compute(const combustion_efficiency_t *eff,
                                      double T_ambient)
{
    if (eff == NULL) return 0.0;

    /* Determine A1, B1 based on fuel type from CO2 level */
    /* Natural gas: CO2 typically 9-11% */
    /* Oil: CO2 typically 12-14% */
    /* Coal: CO2 typically 14-16% */
    double A1, B1;

    if (eff->co2_flue_gas_pct > 13.0) {
        A1 = 0.62; B1 = 0.011; /* Coal */
    } else if (eff->co2_flue_gas_pct > 11.0) {
        A1 = 0.43; B1 = 0.010; /* Oil */
    } else {
        A1 = 0.37; B1 = 0.009; /* Natural gas */
    }

    double dT = eff->stack_temp_c - T_ambient;
    if (dT < 0.0) dT = 0.0; /* Efficiency cannot exceed 100% */

    double stack_loss;
    if (eff->co2_flue_gas_pct > 0.0) {
        stack_loss = dT * (A1 / eff->co2_flue_gas_pct + B1);
    } else {
        stack_loss = dT * 0.05; /* fallback estimate */
    }

    double radiation_loss = 1.5; /* typical 1-2% */
    double unburned_loss = 0.0;

    /* CO-based unburned fuel loss */
    /* Approx: each 1000 ppm CO ≈ 0.3% efficiency loss */
    if (eff->co_flue_gas_ppm > 100.0) {
        unburned_loss = (eff->co_flue_gas_ppm / 1000.0) * 0.3;
    }

    double efficiency = 100.0 - stack_loss - radiation_loss - unburned_loss;
    if (efficiency < 0.0)   efficiency = 0.0;
    if (efficiency > 99.9) efficiency = 99.9;

    return efficiency;
}

/**
 * @brief Compute excess air from O2 in dry flue gas.
 *
 * For complete combustion with dry air (21% O2, 79% N2):
 *   Excess air % = O2_pct_measured * 100 / (21 - O2_pct_measured)
 *
 * This assumes the O2 in flue gas comes entirely from excess air
 * (not from incomplete combustion or air infiltration).
 *
 * Example:
 *   O2 = 3%  → excess air = 3*100/(21-3) = 16.7%
 *   O2 = 6%  → excess air = 6*100/(21-6) = 40%
 *   O2 = 10% → excess air = 10*100/(21-10) = 91%
 *
 * High O2 → high excess air → low efficiency (heating extra N2).
 * This is why excess O2 is a key controlled variable in combustion ratio control.
 */
double combustion_excess_air_from_o2(double target_o2_pct)
{
    if (target_o2_pct <= 0.0) return 0.0;
    if (target_o2_pct >= 21.0) return 1e6; /* effectively infinite */

    return target_o2_pct * 100.0 / (21.0 - target_o2_pct);
}

/**
 * @brief Compute target AFR for combustion control.
 *
 * AFR_target = AFR_stoich * (1 + excess_air_pct/100)
 * λ = 1 + excess_air_pct/100
 *
 * This is the primary setpoint calculation for combustion
 * ratio control.
 */
double combustion_afr_target(double afr_stoich, double excess_air_pct,
                              double *lambda)
{
    if (afr_stoich <= 0.0) return 0.0;

    double lambda_val = 1.0 + excess_air_pct / 100.0;
    if (lambda != NULL) *lambda = lambda_val;

    return afr_stoich * lambda_val;
}

/* =========================================================================
 * L6: Gas-Liquid Separator Model
 * ========================================================================= */

/**
 * @brief Compute separator liquid residence time.
 *
 * t_res = V_liquid / Q_liquid
 *
 * The residence time determines how long the liquid has to
 * separate from the gas. Insufficient residence time leads
 * to liquid carry-over into the gas outlet.
 */
double separator_residence_time(double V_liquid, double Q_liquid)
{
    if (Q_liquid <= 0.0) return 1e6; /* infinite residence time */
    return V_liquid / Q_liquid;
}

/**
 * @brief Maximum gas velocity to avoid liquid carry-over.
 *
 * Souders-Brown equation:
 *   v_max = K * sqrt((ρ_liquid - ρ_gas) / ρ_gas)
 *
 * K factor design values:
 *   Vertical separator with mesh pad: K = 0.11 m/s
 *   Vertical separator without mesh:  K = 0.05-0.07 m/s
 *   Horizontal separator:            K = 0.12-0.15 m/s
 *
 * The higher the density difference, the higher the allowable
 * gas velocity (easier gravity separation).
 */
double souders_brown_velocity(double rho_liquid, double rho_gas, double K_value)
{
    if (rho_gas <= 0.0) return 0.0;
    if (rho_liquid <= rho_gas) return 0.0;

    return K_value * sqrt((rho_liquid - rho_gas) / rho_gas);
}

/**
 * @brief Gas-liquid separation efficiency.
 *
 * η = 1 - exp(-t_res * v_settle / H)
 *
 * where:
 *   t_res     = gas residence time (s)
 *   v_settle  = droplet settling velocity (m/s, from Stokes' law)
 *   H         = effective separation height (m)
 *
 * Stokes settling velocity for small droplets (Re < 1):
 *   v_settle = g * d² * (ρ_liquid - ρ_gas) / (18 * μ)
 *
 * Efficiency → 1 as residence time increases.
 * This relationship guides separator sizing and L/G ratio control
 * for downstream processing.
 */
double separation_efficiency(double v_settling, double t_residence, double H)
{
    if (H <= 0.0) return 0.0;

    double exponent = -t_residence * v_settling / H;
    return 1.0 - exp(exponent);
}
