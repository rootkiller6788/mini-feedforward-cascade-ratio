/**
 * @file ratio_cross_limiting.h
 * @brief Cross-limiting ratio control for combustion safety.
 *
 * Level: L3 Engineering Structures + L5 Algorithms/Methods
 * Reference: Shinskey, "Process Control Systems" (1996), Ch.7.6 Cross-Limiting
 *            Liptak, "Instrument Engineers' Handbook" (2005), Vol.2, Sec.8.18
 *            ISA-77.41.01, "Fossil Fuel Power Plant Boiler Combustion Controls"
 *
 * Course mapping:
 *   Stanford ENGR205: Process Control — cross-limiting combustion control
 *   Purdue ME 575: Industrial Control — boiler combustion safety
 *   RWTH Aachen: Industrial Control Systems — power plant automation
 *   Tsinghua: Process Control Engineering — combustion optimization
 *
 * Cross-limiting prevents dangerous fuel-rich conditions during load changes
 * by constraining air and fuel flow setpoints relative to each other.
 *
 * The key insight: during a load INCREASE, air must lead fuel (increase air first).
 * During a load DECREASE, fuel must lead air (decrease fuel first).
 *
 * This is implemented with high/low selectors:
 *
 *   Air-leads logic (safe for all changes):
 *     SP_air  = max(demand_air,  fuel_flow * AFR_stoich / r_air_rich)
 *     SP_fuel = min(demand_fuel, air_flow  * r_fuel_rich / AFR_stoich)
 *
 *   Double cross-limit (both directions safe, conventional standard):
 *     SP_air  = max(min(demand_air,  fuel_flow * r_extra_air), fuel_flow * AFR_stoich / r_air)
 *     SP_fuel = min(max(demand_fuel, air_flow  * r_extra_fuel), air_flow  * r_fuel / AFR_stoich)
 */

#ifndef RATIO_CROSS_LIMITING_H
#define RATIO_CROSS_LIMITING_H

#include "ratio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L3: Cross-Limiting Initialization & Configuration
 *===========================================================================*/

/**
 * @brief Initialize a cross-limiting control block.
 *
 * Sets up the cross-limiting parameters for combustion control.
 * Typical configuration:
 *   afr_stoich = 17.2 (natural gas), 14.7 (gasoline), etc.
 *   r_air_rich = 1.05 (5% excess air margin)
 *   r_fuel_rich = 0.95 (5% fuel-rich margin, for fuel-leads direction)
 *
 * The mode determines the safety logic:
 *   AIR_LEADS — safest for all load changes (air leads fuel)
 *   FUEL_LEADS — appropriate when gas supply is limiting
 *   DOUBLE — maximum safety, uses both chains
 *
 * @param cl            Cross-limiting block (zero-initialized)
 * @param mode          Cross-limiting mode
 * @param afr_stoich    Stoichiometric air-fuel ratio (mass basis)
 * @param r_air_rich    Air-rich margin (e.g., 1.05)
 * @param r_fuel_rich   Fuel-rich margin (e.g., 0.95)
 *
 * Complexity: O(1)
 */
void cross_limit_init(cross_limiting_t *cl, cross_limit_mode_t mode,
                       double afr_stoich, double r_air_rich, double r_fuel_rich);

/**
 * @brief Update measured process flows in the cross-limiting block.
 *
 * Must be called every control cycle with current air and fuel flow
 * measurements before computing setpoints.
 *
 * @param cl         Cross-limiting block
 * @param air_flow   Measured air mass flow (kg/s or normalized)
 * @param fuel_flow  Measured fuel mass flow (kg/s or normalized)
 *
 * Complexity: O(1)
 */
void cross_limit_update_flows(cross_limiting_t *cl,
                               double air_flow, double fuel_flow);

/**
 * @brief Update unconstrained demand signals.
 *
 * The demand signals come from the master controller (typically
 * a steam pressure or temperature controller in boiler applications).
 * These represent the desired air and fuel flows before safety
 * cross-limiting is applied.
 *
 * @param cl           Cross-limiting block
 * @param demand_air   Unconstrained air demand
 * @param demand_fuel  Unconstrained fuel demand
 *
 * Complexity: O(1)
 */
void cross_limit_update_demands(cross_limiting_t *cl,
                                 double demand_air, double demand_fuel);

/*===========================================================================
 * L5: Cross-Limiting Computation Algorithms
 *===========================================================================*/

/**
 * @brief Execute air-leads cross-limiting.
 *
 * In air-leads mode, the air setpoint can increase ahead of fuel,
 * but fuel can only increase if sufficient air is already present.
 *
 * Algorithm:
 *   1. SP_air = max(demand_air, fuel_flow * AFR_stoich / r_air_rich)
 *      — Air never goes below what's needed for current fuel flow + margin
 *   2. SP_fuel = min(demand_fuel, air_flow * r_air_rich / AFR_stoich)
 *      — Fuel cannot exceed what current air flow can support
 *
 * This ensures: fuel_flow ≤ air_flow * r_air_rich / AFR_stoich
 *
 * I.e., the actual AFR never goes below AFR_stoich / r_air_rich
 * (a lean-biased margin that prevents CO formation).
 *
 * @param cl  Cross-limiting block (updated with SP_air, SP_fuel)
 *
 * Complexity: O(1)
 * References: Shinskey (1996), Sec. 7.6
 */
void cross_limit_air_leads(cross_limiting_t *cl);

/**
 * @brief Execute fuel-leads cross-limiting.
 *
 * In fuel-leads mode (used when gas supply is the bottleneck):
 *   1. SP_fuel = max(demand_fuel, air_flow * r_fuel_rich / AFR_stoich)
 *      — Fuel never drops below what air can safely combust
 *   2. SP_air = min(demand_air, fuel_flow * AFR_stoich / r_fuel_rich)
 *      — Air cannot exceed what fuel is available (prevents lean blowout)
 *
 * @param cl  Cross-limiting block (updated with SP_air, SP_fuel)
 *
 * Complexity: O(1)
 */
void cross_limit_fuel_leads(cross_limiting_t *cl);

/**
 * @brief Execute double cross-limiting (full safety).
 *
 * Double cross-limiting combines both air-leads and fuel-leads
 * for maximum combustion safety during all load transitions.
 *
 * This is the industry standard for large utility boilers and
 * fired heaters (ISA-77.41.01).
 *
 * Algorithm for each direction:
 *
 * Load INCREASE (demand up):
 *   It must be: SP_fuel ≤ SP_air * r_fuel / AFR_stoich  (fuel constrained by air)
 *   and:        SP_air  ≥ fuel_flow * AFR_stoich / r_air (air leads fuel)
 *
 * Load DECREASE (demand down):
 *   It must be: SP_air  ≤ fuel_flow * AFR_stoich * r_extra / r_fuel (air constrained by fuel)
 *   and:        SP_fuel ≥ SP_air * r_fuel / AFR_stoich  (fuel leads air decrease)
 *
 * Implementation with selector chain:
 *   SP_air  = max(min(demand_air,  fuel_flow * AFR_stoich * r_extra),
 *                 fuel_flow * AFR_stoich / r_air)
 *
 *   SP_fuel = min(max(demand_fuel, air_flow  * r_fuel / AFR_stoich),
 *                 air_flow  * r_extra / AFR_stoich)
 *
 * where r_extra is the extra margin for the opposite direction (typically 1.03-1.10).
 *
 * @param cl       Cross-limiting block (updated with SP_air, SP_fuel)
 * @param r_extra  Extra margin factor (e.g., 1.05 for 5%)
 *
 * Complexity: O(1)
 */
void cross_limit_double(cross_limiting_t *cl, double r_extra);

/**
 * @brief Execute cross-limiting based on configured mode.
 *
 * Dispatches to the appropriate cross-limiting algorithm
 * based on cl->mode. This is the primary runtime function.
 *
 * @param cl       Cross-limiting block (updated with SP_air, SP_fuel)
 * @param r_extra  Extra margin for double cross-limit (ignored for other modes)
 *
 * Complexity: O(1)
 */
void cross_limit_execute(cross_limiting_t *cl, double r_extra);

/*===========================================================================
 * L4: Safety Checks & Diagnostics
 *===========================================================================*/

/**
 * @brief Check if the current combustion state is safe.
 *
 * Returns:
 *   0 → Safe: AFR > AFR_stoich (lean side, excess air present)
 *   1 → Warning: AFR slightly below stoich (marginal)
 *   2 → Danger: AFR significantly below stoich (CO hazard)
 *  -1 → Error: invalid flow measurements
 *
 * @param cl           Cross-limiting block
 * @param afr_actual   Actual measured AFR
 * @return             Safety status code
 *
 * Complexity: O(1)
 */
int cross_limit_check_safety(const cross_limiting_t *cl, double afr_actual);

/**
 * @brief Compute the current air-fuel ratio from cross-limiting state.
 *
 * AFR = air_flow / fuel_flow (mass basis)
 *
 * @param cl  Cross-limiting block
 * @return    Current AFR, or 0.0 if fuel_flow ≤ 0
 *
 * Complexity: O(1)
 */
double cross_limit_current_afr(const cross_limiting_t *cl);

/**
 * @brief Compute the excess air ratio (lambda).
 *
 * λ = actual_AFR / AFR_stoich
 *
 * λ = 1.0 → stoichiometric
 * λ > 1.0 → lean (excess air)
 * λ < 1.0 → rich (excess fuel, CO hazard)
 *
 * @param cl  Cross-limiting block
 * @return    Lambda value, or 0.0 if invalid
 *
 * Complexity: O(1)
 */
double cross_limit_lambda(const cross_limiting_t *cl);

/**
 * @brief Compute the air and fuel setpoint margins.
 *
 * Air margin: how far SP_air is above the minimum safe air flow
 *   = SP_air - fuel_flow * AFR_stoich / r_air_rich
 *
 * Fuel margin: how far SP_fuel is below the maximum safe fuel flow
 *   = air_flow * r_fuel_rich / AFR_stoich - SP_fuel
 *
 * Positive margins indicate safe operating envelope.
 * Negative margins indicate constraint violation.
 *
 * @param cl           Cross-limiting block
 * @param margin_air   [out] Air margin (positive = safe)
 * @param margin_fuel  [out] Fuel margin (positive = safe)
 *
 * Complexity: O(1)
 */
void cross_limit_margins(const cross_limiting_t *cl,
                          double *margin_air, double *margin_fuel);

/**
 * @brief Get cross-limiting diagnostics as a formatted string.
 *
 * @param cl     Cross-limiting block
 * @param buf    Output buffer (at least 512 bytes)
 * @param bufsz  Buffer size
 * @return       Number of characters written
 *
 * Complexity: O(1)
 */
int cross_limit_diagnostics(const cross_limiting_t *cl, char *buf, size_t bufsz);

#ifdef __cplusplus
}
#endif

#endif /* RATIO_CROSS_LIMITING_H */
