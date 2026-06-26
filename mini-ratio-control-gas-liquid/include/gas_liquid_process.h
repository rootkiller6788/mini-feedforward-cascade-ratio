/**
 * @file gas_liquid_process.h
 * @brief Gas-liquid process models for ratio control applications.
 *
 * Level: L3 Engineering Structures + L6 Canonical Problems
 * Reference: Treybal, "Mass-Transfer Operations" (1980), Ch.6-8
 *            Perry's Chemical Engineers' Handbook (9th Ed), Sec.14
 *            Seader, Henley, Roper, "Separation Process Principles" (2011)
 *            Froment, Bischoff, De Wilde, "Chemical Reactor Analysis & Design" (2010)
 *
 * Course mapping:
 *   Stanford ENGR205: Process Control — reactor & mass transfer models
 *   MIT 2.171: Digital Control — physical process modeling
 *   CMU 24-677: Advanced Control — nonlinear process models
 *   Tsinghua: Process Control Engineering — industrial process dynamics
 *
 * This module provides dynamic and steady-state models for gas-liquid
 * systems commonly encountered in ratio control applications:
 *   - Gas absorption/scrubbing columns
 *   - Gas-liquid CSTR reactors
 *   - Two-phase pipeline flow
 *   - Boiler/furnace combustion
 *   - Gas-liquid separators
 */

#ifndef GAS_LIQUID_PROCESS_H
#define GAS_LIQUID_PROCESS_H

#include "ratio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L3: Gas Density & Ideal Gas Law Computations
 *===========================================================================*/

/**
 * @brief Compute gas density using the ideal gas law.
 *
 * Ideal Gas Law: ρ = P * M / (R * T)
 *
 * @param gas            Gas state (pressure, temperature, molar mass)
 * @return               Density in kg/m³
 *
 * Complexity: O(1)
 * References: Poling, Prausnitz, O'Connell (2001), Eq. 4.2.1
 */
double gas_density_ideal(const gas_state_t *gas);

/**
 * @brief Compute gas density with real-gas compressibility correction.
 *
 * Real gas density: ρ = P * M / (Z * R * T)
 *
 * The compressibility factor Z accounts for non-ideal behavior.
 * Z < 1: attractive forces dominate (compressible)
 * Z > 1: repulsive forces dominate
 * Z = 1: ideal gas
 *
 * @param gas            Gas state with compressibility factor
 * @return               Density in kg/m³
 *
 * Complexity: O(1)
 * References: Poling, Prausnitz, O'Connell (2001), Eq. 4.3.1
 */
double gas_density_real(const gas_state_t *gas);

/**
 * @brief Compute gas density with automatic mode selection.
 *
 * Uses ideal gas law if use_real_gas = 0,
 * otherwise applies compressibility correction.
 *
 * @param gas            Gas state
 * @return               Density in kg/m³
 *
 * Complexity: O(1)
 */
double gas_density_compute(const gas_state_t *gas);

/**
 * @brief Convert gas flow from actual to normal conditions.
 *
 * Normal conditions: T_n = 273.15 K (0°C), P_n = 101325 Pa (1 atm)
 *
 * Conversion: Q_normal = Q_actual * (P_actual/P_n) * (T_n/T_actual)
 *
 * This is essential for ratio control when gas flow meters measure
 * at actual conditions but ratio is specified at normal conditions.
 *
 * @param flow_actual    Actual volumetric flow (m³/s)
 * @param P_actual       Actual absolute pressure (Pa)
 * @param T_actual       Actual temperature (K)
 * @param P_n            Normal pressure (Pa, default 101325)
 * @param T_n            Normal temperature (K, default 273.15)
 * @return               Normalized flow (Nm³/s)
 *
 * Complexity: O(1)
 * References: ISO 5024: Petroleum liquids and gases — standard reference conditions
 */
double gas_flow_normal_to_actual(double flow_normal,
                                  double P_actual, double T_actual,
                                  double P_n, double T_n);

double gas_flow_actual_to_normal(double flow_actual,
                                  double P_actual, double T_actual,
                                  double P_n, double T_n);

/*===========================================================================
 * L3: Liquid Density Temperature Compensation
 *===========================================================================*/

/**
 * @brief Compute temperature-compensated liquid density.
 *
 * Linear thermal expansion model:
 *   ρ(T) = ρ_ref * (1 - β * (T - T_ref))
 *
 * Valid for moderate temperature ranges (±50°C from T_ref).
 * For wider ranges, polynomial models are recommended.
 *
 * @param liq            Liquid density state
 * @return               Compensated density (kg/m³)
 *
 * Complexity: O(1)
 * References: Perry's Handbook (9th Ed), Sec. 2-134
 */
double liquid_density_compensate(const liquid_density_t *liq);

/**
 * @brief Get thermal expansion coefficient for common liquids.
 *
 * Returns β in 1/K for specified substance:
 *   "water"  → 2.1e-4
 *   "ethanol" → 1.1e-3
 *   "methanol" → 1.2e-3
 *   "benzene" → 1.2e-3
 *   "oil_light" → 7.0e-4
 *   "oil_heavy" → 5.0e-4
 *
 * @param substance     Substance name string
 * @return              β coefficient (1/K), -1 if substance not found
 *
 * Complexity: O(1)
 */
double liquid_expansion_coeff(const char *substance);

/*===========================================================================
 * L4: Gas Absorption / Scrubbing Column Model
 *===========================================================================*/

/**
 * @brief Compute Henry's Law equilibrium concentration.
 *
 * Henry's Law:
 *   C_eq = kH * P_gas
 *
 * where:
 *   C_eq = equilibrium concentration of dissolved gas (mol/L)
 *   kH   = Henry's law constant (mol/(L·atm))
 *   P_gas = partial pressure of gas above liquid (atm)
 *
 * Common Henry's constants at 25°C:
 *   CO2: 3.4e-2 mol/(L·atm)
 *   O2:  1.3e-3 mol/(L·atm)
 *   NH3: 5.8e+1 mol/(L·atm)
 *   SO2: 1.2e+0 mol/(L·atm)
 *
 * @param eq             Gas-liquid equilibrium parameters
 * @return               Equilibrium concentration (mol/L)
 *
 * Complexity: O(1)
 */
double henry_equilibrium(const gl_equilibrium_t *eq);

/**
 * @brief Compute mass transfer rate in a gas-liquid contactor.
 *
 * Mass transfer rate:
 *   N = kLa * V * (C_eq - C_bulk)
 *
 * where:
 *   N     = molar transfer rate (mol/s)
 *   kLa   = volumetric mass transfer coefficient (1/s)
 *   V     = liquid volume (L)
 *   C_eq  = equilibrium concentration (mol/L)
 *   C_bulk = bulk liquid concentration (mol/L)
 *
 * @param eq             Equilibrium parameters
 * @param C_bulk         Bulk liquid concentration (mol/L)
 * @return               Mass transfer rate (mol/s)
 *
 * Complexity: O(1)
 * References: Treybal (1980), Eq. 6.17
 */
double mass_transfer_rate(const gl_equilibrium_t *eq, double C_bulk);

/**
 * @brief Compute the required liquid-to-gas ratio for a target removal efficiency.
 *
 * For a counter-current absorber with dilute species:
 *   L/G = (y_in - y_out) / (x_out - x_in)
 *
 * where:
 *   L = liquid molar flow (mol/s)
 *   G = gas molar flow (mol/s)
 *   y_in, y_out = gas-phase mole fractions
 *   x_in, x_out = liquid-phase mole fractions
 *
 * The minimum L/G for 100% removal (infinite column):
 *   (L/G)_min = (y_in - y_out) / (x_eq(y_in) - x_in)
 *
 * @param y_in          Inlet gas mole fraction
 * @param y_out         Outlet gas mole fraction (target)
 * @param x_in          Inlet liquid mole fraction (typically 0 for fresh solvent)
 * @param kH            Henry's law constant for the species
 * @param P_total       Total pressure (atm)
 * @return              Required L/G ratio (dimensionless)
 *
 * Complexity: O(1)
 */
double absorber_liquid_gas_ratio(double y_in, double y_out,
                                  double x_in, double kH, double P_total);

/**
 * @brief Compute NTU (Number of Transfer Units) for a gas absorber.
 *
 * For dilute systems with constant L/G:
 *   NTU = ln( (y_in - m*x_in) / (y_out - m*x_in) ) / (1 - m*G/L)
 *
 * where m = kH * P_total (equilibrium line slope in mole fraction coordinates).
 *
 * @param y_in          Inlet gas mole fraction
 * @param y_out         Outlet gas mole fraction
 * @param x_in          Inlet liquid mole fraction
 * @param kH            Henry's law constant (mol/(L·atm))
 * @param P_total       Total pressure (atm)
 * @param L_G_ratio     Liquid-to-gas molar flow ratio
 * @return              Number of transfer units
 *
 * Complexity: O(1)
 */
double absorber_ntu(double y_in, double y_out, double x_in,
                     double kH, double P_total, double L_G_ratio);

/**
 * @brief Dynamic absorber model — one time step.
 *
 * Simplified lumped-parameter absorber dynamics:
 *   dC_liquid/dt = (Q_gas/V)*(C_gas_in - C_gas_out) + kLa*(C_eq - C_liquid)
 *
 * Discretized using forward Euler:
 *   C_liquid(k+1) = C_liquid(k) + Ts * [ ... ]
 *
 * @param eq             Equilibrium parameters
 * @param Q_gas          Gas volumetric flow (L/s)
 * @param Q_liquid       Liquid volumetric flow (L/s)
 * @param C_gas_in       Inlet gas concentration (mol/L)
 * @param C_liquid       Current liquid concentration (mol/L) — updated in place
 * @param Ts             Time step (s)
 * @return               Updated C_liquid (mol/L)
 *
 * Complexity: O(1)
 */
double absorber_dynamic_step(const gl_equilibrium_t *eq,
                              double Q_gas, double Q_liquid,
                              double C_gas_in, double *C_liquid, double Ts);

/*===========================================================================
 * L6: Gas-Liquid CSTR Reactor Model
 *===========================================================================*/

/**
 * @brief Dynamic gas-liquid CSTR (Continuous Stirred Tank Reactor) model.
 *
 * Mass balance for species A in liquid phase:
 *   dC_A/dt = (F_in*C_Ain - F_out*C_A)/V + kLa*(C_A_eq - C_A) + r_A
 *
 * where r_A is the reaction rate (typically -k*C_A for first-order).
 *
 * Gas phase (assuming well-mixed):
 *   dP_A/dt = (G_in*P_Ain - G_out*P_A)/V_gas - kLa*(C_A_eq - C_A)*R*T
 *
 * @param V_liquid      Liquid volume (L)
 * @param V_gas         Gas volume (L)
 * @param Q_liquid_in   Inlet liquid flow (L/s)
 * @param Q_liquid_out  Outlet liquid flow (L/s)
 * @param Q_gas_in      Inlet gas flow (L/s)
 * @param Q_gas_out     Outlet gas flow (L/s)
 * @param C_A_in        Inlet liquid concentration (mol/L)
 * @param C_A           Current liquid concentration [updated] (mol/L)
 * @param P_A           Partial pressure of A in gas [updated] (atm)
 * @param kLa           Mass transfer coefficient (1/s)
 * @param kH            Henry's constant (mol/(L·atm))
 * @param k_reaction    Reaction rate constant (1/s for 1st-order)
 * @param Ts            Time step (s)
 *
 * Complexity: O(1)
 * References: Froment, Bischoff, De Wilde (2010), Ch.7
 */
void gl_reactor_dynamic_step(double V_liquid, double V_gas,
                              double Q_liquid_in, double Q_liquid_out,
                              double Q_gas_in, double Q_gas_out,
                              double C_A_in, double *C_A, double *P_A,
                              double kLa, double kH, double k_reaction,
                              double Ts);

/**
 * @brief Compute steady-state gas-liquid ratio for a CSTR.
 *
 * At steady state: dC_A/dt = 0, dP_A/dt = 0
 *
 * The required gas-to-liquid ratio GLR = Q_gas / Q_liquid
 * to maintain a target concentration C_A_target can be computed
 * by solving the steady-state mass balance.
 *
 * @param C_A_target    Target liquid concentration (mol/L)
 * @param C_A_in        Inlet liquid concentration (mol/L)
 * @param kLa           Mass transfer coefficient (1/s)
 * @param kH            Henry's constant (mol/(L·atm))
 * @param k_reaction    Reaction rate constant (1/s)
 * @param V_liquid      Liquid volume (L)
 * @param V_gas         Gas volume (L)
 * @param Q_liquid      Liquid flow rate (L/s)
 * @return              Required gas flow rate (L/s) to maintain C_A_target
 *
 * Complexity: O(1)
 */
double gl_reactor_steady_gas_flow(double C_A_target, double C_A_in,
                                   double kLa, double kH, double k_reaction,
                                   double V_liquid, double V_gas, double Q_liquid);

/*===========================================================================
 * L6: Two-Phase Flow in Pipelines
 *===========================================================================*/

/**
 * @brief Compute void fraction for two-phase gas-liquid flow.
 *
 * Homogeneous model (no slip):
 *   α = β = Q_gas / (Q_gas + Q_liquid)
 *
 * Slip-corrected (Zuber-Findlay drift-flux model):
 *   α = Q_gas / (C0 * (Q_gas + Q_liquid) + A * U_drift)
 *
 * where C0 is the distribution parameter (≈1.2 for turbulent flow)
 * and U_drift is the drift velocity.
 *
 * @param tf             Two-phase flow state (gas/liquid flow rates)
 * @param C0             Distribution parameter
 * @param U_drift        Drift velocity (m/s)
 * @param pipe_area      Pipe cross-sectional area (m²)
 * @return               Void fraction α (dimensionless)
 *
 * Complexity: O(1)
 * References: Zuber & Findlay (1965), "Average Volumetric Concentration
 *             in Two-Phase Flow Systems", J. Heat Transfer, 87, 453-468
 */
double two_phase_void_fraction(const two_phase_flow_t *tf,
                                double C0, double U_drift, double pipe_area);

/**
 * @brief Estimate two-phase flow regime.
 *
 * Returns regime index based on gas and liquid superficial velocities:
 *   0 = bubbly flow (low gas, continuous liquid)
 *   1 = slug/churn flow (intermediate)
 *   2 = annular flow (high gas, liquid film on wall)
 *   3 = mist flow (very high gas, liquid droplets)
 *
 * Uses Taitel-Dukler (1976) flow regime map approximation.
 *
 * @param tf             Two-phase flow state
 * @param pipe_diam      Pipe diameter (m)
 * @param rho_gas        Gas density (kg/m³)
 * @param rho_liquid     Liquid density (kg/m³)
 * @param sigma          Surface tension (N/m)
 * @return               Flow regime index (0-3)
 *
 * Complexity: O(1)
 * References: Taitel & Dukler (1976), AIChE Journal, 22(1), 47-55
 */
int two_phase_flow_regime(const two_phase_flow_t *tf,
                           double pipe_diam, double rho_gas,
                           double rho_liquid, double sigma);

/**
 * @brief Compute Lockhart-Martinelli two-phase multiplier.
 *
 * The two-phase multiplier Φ²_L relates single-phase pressure drop
 * to two-phase pressure drop:
 *   (ΔP/L)_tp = Φ²_L * (ΔP/L)_L
 *
 * where (ΔP/L)_L is the pressure drop if liquid flowed alone.
 *
 * Approximate correlation (Chisholm, 1967):
 *   Φ²_L = 1 + C/X + 1/X²
 *
 * where C depends on flow regime (gas/liquid both turbulent: C=20).
 *
 * @param X              Lockhart-Martinelli parameter
 * @param C_coeff        Chisholm coefficient
 * @return               Two-phase multiplier Φ²_L
 *
 * Complexity: O(1)
 * References: Chisholm (1967), Int. J. Heat Mass Transfer, 10, 1767-1778
 */
double lockhart_martinelli_multiplier(double X, double C_coeff);

/*===========================================================================
 * L6: Combustion Efficiency Model
 *===========================================================================*/

/**
 * @brief Compute combustion efficiency from flue gas analysis.
 *
 * Efficiency based on stack loss method (ASME PTC 4):
 *   η = 100% - stack_loss - radiation_loss - unburned_loss
 *
 * Stack loss (dry gas method):
 *   stack_loss = k * (T_stack - T_ambient) / CO2_pct
 *
 * Simplified Siegert formula:
 *   η = 100 - (T_stack - T_ambient) * (A1/CO2 + B1)
 *
 * where A1, B1 depend on fuel type.
 *
 * @param eff            Combustion efficiency parameters
 * @param T_ambient      Ambient temperature (°C)
 * @return               Efficiency (%)
 *
 * Complexity: O(1)
 * References: ASME PTC 4-2013, "Fired Steam Generators"
 */
double combustion_efficiency_compute(const combustion_efficiency_t *eff,
                                      double T_ambient);

/**
 * @brief Compute required excess air for target O2 in flue gas.
 *
 * For complete combustion:
 *   Excess air % = O2_pct * 100 / (21 - O2_pct)
 *
 * This assumes dry air with 21% O2.
 *
 * @param target_o2_pct  Target O2 in dry flue gas (%)
 * @return               Required excess air (%)
 *
 * Complexity: O(1)
 */
double combustion_excess_air_from_o2(double target_o2_pct);

/**
 * @brief Compute target air-fuel ratio for combustion control.
 *
 * AFR_target = AFR_stoich * (1 + excess_air_pct / 100)
 *
 * where AFR_stoich is the stoichiometric air-fuel ratio for the fuel.
 *
 * @param afr_stoich     Stoichiometric AFR (mass basis)
 * @param excess_air_pct Excess air percentage
 * @param lambda         [out] Excess air ratio λ
 * @return               Target AFR for control
 *
 * Complexity: O(1)
 */
double combustion_afr_target(double afr_stoich, double excess_air_pct,
                              double *lambda);

/*===========================================================================
 * L6: Gas-Liquid Separator Model
 *===========================================================================*/

/**
 * @brief Compute gas-liquid separator residence time.
 *
 * Separator sizing criterion:
 *   t_residence = V_liquid / Q_liquid
 *
 * Typical residence times:
 *   Vertical separator: 2-5 minutes
 *   Horizontal separator: 3-10 minutes
 *
 * @param V_liquid       Liquid volume (m³)
 * @param Q_liquid       Liquid outflow (m³/s)
 * @return               Residence time (seconds)
 *
 * Complexity: O(1)
 */
double separator_residence_time(double V_liquid, double Q_liquid);

/**
 * @brief Compute maximum gas velocity to avoid liquid carry-over.
 *
 * Souders-Brown equation:
 *   v_max = K * sqrt((ρ_liquid - ρ_gas) / ρ_gas)
 *
 * where K is a design constant (typically 0.03-0.11 m/s for vertical separators).
 *
 * @param rho_liquid     Liquid density (kg/m³)
 * @param rho_gas        Gas density (kg/m³)
 * @param K_value        Souders-Brown K factor (m/s)
 * @return               Maximum gas velocity (m/s)
 *
 * Complexity: O(1)
 * References: Souders & Brown (1934), Ind. Eng. Chem., 26(1), 98-103
 */
double souders_brown_velocity(double rho_liquid, double rho_gas, double K_value);

/**
 * @brief Compute gas-liquid separation efficiency.
 *
 * Simplified model:
 *   η = 1 - exp(-t_residence * v_settling / H)
 *
 * where v_settling is the droplet settling velocity (Stokes' law)
 * and H is the effective separation height.
 *
 * @param v_settling     Droplet settling velocity (m/s)
 * @param t_residence    Gas residence time (s)
 * @param H              Effective separation height (m)
 * @return               Separation efficiency (0-1)
 *
 * Complexity: O(1)
 */
double separation_efficiency(double v_settling, double t_residence, double H);

#ifdef __cplusplus
}
#endif

#endif /* GAS_LIQUID_PROCESS_H */
