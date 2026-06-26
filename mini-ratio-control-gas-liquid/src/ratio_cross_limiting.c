/**
 * @file ratio_cross_limiting.c
 * @brief Cross-limiting ratio control for combustion safety.
 *
 * Level: L3 Engineering Structures + L5 Algorithms/Methods + L4 Engineering Laws
 *
 * Implements the cross-limiting algorithms that prevent dangerous
 * fuel-rich conditions during boiler/furnace load changes.
 *
 * The cross-limiting logic ensures that:
 *   - On load INCREASE: air flow must increase BEFORE fuel flow
 *   - On load DECREASE: fuel flow must decrease BEFORE air flow
 *
 * This guarantees that the combustion zone always has excess air,
 * preventing CO formation (incomplete combustion hazard).
 *
 * References:
 *   - Shinskey, "Process Control Systems" (1996), Ch.7.6
 *   - Liptak, "Instrument Engineers' Handbook" (2005), Vol.2, Sec.8.18
 *   - ISA-77.41.01 "Fossil Fuel Power Plant Boiler Combustion Controls"
 */

#include "ratio_types.h"
#include "ratio_cross_limiting.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* =========================================================================
 * L3: Cross-Limiting Initialization
 * ========================================================================= */

/**
 * @brief Initialize cross-limiting block.
 *
 * Sets up the safety margins:
 *   - r_air_rich (e.g., 1.05): air must exceed stoichiometric by 5%
 *   - r_fuel_rich (e.g., 0.95): fuel limited to 95% of what current air supports
 *
 * These margins create a safe operating band around the stoichiometric ratio.
 * The band width is the trade-off:
 *   - Wider band → safer but less efficient (more excess air)
 *   - Narrower band → more efficient but less safe
 *
 * For natural gas: r_air_rich=1.03 provides adequate safety
 * For oil:         r_air_rich=1.05 recommended
 * For coal:        r_air_rich=1.08 recommended (slower combustion)
 */
void cross_limit_init(cross_limiting_t *cl, cross_limit_mode_t mode,
                       double afr_stoich, double r_air_rich, double r_fuel_rich)
{
    if (cl == NULL) return;

    memset(cl, 0, sizeof(*cl));

    cl->mode          = mode;
    cl->afr_stoich    = afr_stoich > 0.0 ? afr_stoich : 17.2;
    cl->r_air_rich    = r_air_rich > 0.0 ? r_air_rich : 1.05;
    cl->r_fuel_rich   = r_fuel_rich > 0.0 ? r_fuel_rich : 0.95;

    /* Initialize all measurements and demands to zero */
    cl->demand_air  = 0.0;
    cl->demand_fuel = 0.0;
    cl->air_flow    = 0.0;
    cl->fuel_flow   = 0.0;
    cl->sp_air      = 0.0;
    cl->sp_fuel     = 0.0;

    cl->air_high_selected  = 0;
    cl->fuel_low_selected  = 0;
    cl->last_update_ms     = 0;
}

/**
 * @brief Update measured flows.
 */
void cross_limit_update_flows(cross_limiting_t *cl,
                               double air_flow, double fuel_flow)
{
    if (cl == NULL) return;
    cl->air_flow  = air_flow;
    cl->fuel_flow = fuel_flow;
}

/**
 * @brief Update demand signals from master controller.
 */
void cross_limit_update_demands(cross_limiting_t *cl,
                                 double demand_air, double demand_fuel)
{
    if (cl == NULL) return;
    cl->demand_air  = demand_air;
    cl->demand_fuel = demand_fuel;
}

/* =========================================================================
 * L5: Air-Leads Cross-Limiting Algorithm
 *
 * This is the most common industrial cross-limiting configuration.
 * Air always leads fuel — during load changes, air is increased
 * first and decreased last, ensuring a lean-bias at all times.
 *
 * Selector logic:
 *
 *   Air channel  [HIGH SELECT]:
 *     SP_air = max(demand_air, fuel_flow * AFR_stoich / r_air_rich)
 *
 *     - demand_air: what the master controller wants
 *     - fuel_flow * AFR_stoich / r_air_rich: minimum air to support
 *       current fuel flow with r_air_rich margin
 *
 *   Fuel channel [LOW SELECT]:
 *     SP_fuel = min(demand_fuel, air_flow * r_air_rich / AFR_stoich)
 *
 *     - demand_fuel: what the master controller wants
 *     - air_flow * r_air_rich / AFR_stoich: maximum fuel that current
 *       air flow can support with r_air_rich margin
 *
 * During a load INCREASE:
 *   - Air demand increases → air SP increases (high select: demand > minimum)
 *   - Air flow increases (process response)
 *   - Air flow increase enables fuel SP increase (low select limit rises)
 *   - Fuel flow increases
 *
 * During a load DECREASE:
 *   - Fuel demand decreases → fuel SP decreases (low select: demand < maximum)
 *   - Fuel flow decreases (process response)
 *   - Fuel flow decrease enables air SP decrease (high select: minimum drops)
 *   - Air flow decreases
 * ========================================================================= */

void cross_limit_air_leads(cross_limiting_t *cl)
{
    if (cl == NULL) return;
    if (cl->afr_stoich <= 0.0) return;

    /* --- Air channel: HIGH SELECT --- */
    double air_minimum = 0.0;
    if (cl->fuel_flow > 0.0) {
        /* Minimum air required to support current fuel flow with margin */
        air_minimum = cl->fuel_flow * cl->afr_stoich / cl->r_air_rich;
    }

    /* High select: use the larger of demand and safety minimum */
    double sp_air = cl->demand_air;
    if (air_minimum > sp_air) {
        sp_air = air_minimum;
        cl->air_high_selected = 1; /* Safety override active */
    } else {
        cl->air_high_selected = 0;
    }

    /* --- Fuel channel: LOW SELECT --- */
    double fuel_maximum = 1e12; /* effectively unlimited */
    if (cl->air_flow > 0.0) {
        /* Maximum fuel that current air flow can support with margin */
        fuel_maximum = cl->air_flow * cl->r_air_rich / cl->afr_stoich;
    }

    /* Low select: use the smaller of demand and safety maximum */
    double sp_fuel = cl->demand_fuel;
    if (fuel_maximum < sp_fuel) {
        sp_fuel = fuel_maximum;
        cl->fuel_low_selected = 1; /* Safety override active */
    } else {
        cl->fuel_low_selected = 0;
    }

    cl->sp_air  = sp_air;
    cl->sp_fuel = sp_fuel;
}

/* =========================================================================
 * L5: Fuel-Leads Cross-Limiting Algorithm
 *
 * Used when gas/fuel supply is the bottleneck (e.g., waste gas firing,
 * limited fuel gas availability).
 *
 * Fuel-leads ensures that when fuel supply changes, air adjusts
 * afterward to avoid lean blowout (flame extinction from too much air).
 *
 * Selector logic:
 *   Fuel channel [HIGH SELECT]:
 *     SP_fuel = max(demand_fuel, air_flow * r_fuel_rich / AFR_stoich)
 *
 *   Air channel [LOW SELECT]:
 *     SP_air = min(demand_air, fuel_flow * AFR_stoich / r_fuel_rich)
 * ========================================================================= */

void cross_limit_fuel_leads(cross_limiting_t *cl)
{
    if (cl == NULL) return;
    if (cl->afr_stoich <= 0.0) return;

    /* --- Fuel channel: HIGH SELECT --- */
    double fuel_minimum = 0.0;
    if (cl->air_flow > 0.0) {
        /* Minimum fuel to avoid excessive lean (blowout prevention) */
        fuel_minimum = cl->air_flow * cl->r_fuel_rich / cl->afr_stoich;
    }

    double sp_fuel = cl->demand_fuel;
    if (fuel_minimum > sp_fuel) {
        sp_fuel = fuel_minimum;
    }

    /* --- Air channel: LOW SELECT --- */
    double air_maximum = 1e12;
    if (cl->fuel_flow > 0.0) {
        /* Maximum air to avoid excessive lean */
        air_maximum = cl->fuel_flow * cl->afr_stoich / cl->r_fuel_rich;
    }

    double sp_air = cl->demand_air;
    if (air_maximum < sp_air) {
        sp_air = air_maximum;
    }

    cl->sp_air  = sp_air;
    cl->sp_fuel = sp_fuel;
}

/* =========================================================================
 * L5: Double Cross-Limiting Algorithm (Full Safety)
 *
 * This is the industry standard for large utility boilers and
 * refinery fired heaters. It combines both air-leads and fuel-leads
 * logic using a chain of high/low selectors.
 *
 * For each direction:
 *   - Load INCREASE: air leads fuel (air can increase freely,
 *     fuel constrained by measured air flow)
 *   - Load DECREASE: fuel leads air (fuel can decrease freely,
 *     air constrained by measured fuel flow)
 *
 * Algorithm with selector chains:
 *
 *                                          ┌─ max ─┐
 *   demand_air ──┬──→ min ─────────────────┤       ├──→ SP_air
 *                │         ┌─────────────┐  │       │
 *                │         │ fuel_flow * │  └───────┘
 *                │         │ AFR_stoich  │      ↑
 *                │         │ * r_extra   │      │
 *                │         └─────────────┘      │
 *                │                              │
 *                └──→ fuel_flow * AFR_stoich / r_air_rich ──┘
 *
 *   demand_fuel ──┬──→ max ──────────────────┬──→ min ──→ SP_fuel
 *                 │         ┌───────────────┐ │
 *                 │         │ air_flow *    │ │
 *                 │         │ r_fuel_rich / │ │
 *                 │         │ AFR_stoich    │ │
 *                 │         └───────────────┘ │
 *                 │                           │
 *                 └──→ air_flow * r_extra / AFR_stoich ──┘
 *
 * This is a 4-selector configuration (2 high, 2 low) that provides
 * cross-limiting in both directions simultaneously.
 *
 * Reference: ISA-77.41.01 standard for boiler combustion control.
 * ========================================================================= */

void cross_limit_double(cross_limiting_t *cl, double r_extra)
{
    if (cl == NULL) return;
    if (cl->afr_stoich <= 0.0) return;
    if (r_extra <= 0.0) r_extra = 1.05;

    double afr = cl->afr_stoich;

    /* === Air SP: max(min(demand_air, fuel_flow*AFR*r_extra), fuel_flow*AFR/r_air) === */

    double air_max_limit = 1e12;
    double air_min_limit = 0.0;

    if (cl->fuel_flow > 0.0) {
        /* Upper constraint: air cannot exceed what fuel can support (with r_extra margin) */
        air_max_limit = cl->fuel_flow * afr * r_extra;
        /* Lower constraint: air must be at least enough for fuel (with r_air margin) */
        air_min_limit = cl->fuel_flow * afr / cl->r_air_rich;
    }

    double sp_air_intermediate = cl->demand_air;
    if (air_max_limit < sp_air_intermediate) {
        sp_air_intermediate = air_max_limit;
    }
    if (air_min_limit > sp_air_intermediate) {
        sp_air_intermediate = air_min_limit;
        cl->air_high_selected = 1;
    } else {
        cl->air_high_selected = 0;
    }
    cl->sp_air = sp_air_intermediate;

    /* === Fuel SP: min(max(demand_fuel, air_flow*r_fuel/afr), air_flow*r_extra/afr) === */

    double fuel_max_limit = 1e12;
    double fuel_min_limit = 0.0;

    if (cl->air_flow > 0.0) {
        /* Upper constraint: fuel cannot exceed what air can support (with r_extra margin) */
        fuel_max_limit = cl->air_flow * r_extra / afr;
        /* Lower constraint: fuel must be at least enough for air (with r_fuel margin) */
        fuel_min_limit = cl->air_flow * cl->r_fuel_rich / afr;
    }

    double sp_fuel_intermediate = cl->demand_fuel;
    if (fuel_min_limit > sp_fuel_intermediate) {
        sp_fuel_intermediate = fuel_min_limit;
    }
    if (fuel_max_limit < sp_fuel_intermediate) {
        sp_fuel_intermediate = fuel_max_limit;
        cl->fuel_low_selected = 1;
    } else {
        cl->fuel_low_selected = 0;
    }
    cl->sp_fuel = sp_fuel_intermediate;
}

/* =========================================================================
 * L5: Cross-Limiting Execution (Mode Dispatch)
 * ========================================================================= */

/**
 * @brief Execute cross-limiting based on configured mode.
 *
 * Dispatches to the appropriate algorithm. This is the primary
 * runtime entry point called from the DCS/PLC control logic.
 */
void cross_limit_execute(cross_limiting_t *cl, double r_extra)
{
    if (cl == NULL) return;

    switch (cl->mode) {
        case CROSS_LIMIT_AIR_LEADS:
            cross_limit_air_leads(cl);
            break;
        case CROSS_LIMIT_FUEL_LEADS:
            cross_limit_fuel_leads(cl);
            break;
        case CROSS_LIMIT_DOUBLE:
            cross_limit_double(cl, r_extra);
            break;
        case CROSS_LIMIT_NONE:
        default:
            /* No cross-limiting: pass demands straight through */
            cl->sp_air  = cl->demand_air;
            cl->sp_fuel = cl->demand_fuel;
            break;
    }
}

/* =========================================================================
 * L4: Safety Checks & Diagnostics
 * ========================================================================= */

/**
 * @brief Check combustion safety status.
 *
 * Returns safety code based on current AFR:
 *   0 → Safe (lean side, AFR > AFR_stoich)
 *   1 → Warning (AFR within 95-100% of stoich, marginal)
 *   2 → Danger (AFR < 95% of stoich, CO hazard)
 *  -1 → Invalid (no flow measurements)
 *
 * Combustion safety standards:
 *   - NFPA 86: Standard for Ovens and Furnaces
 *   - ISA-77.41.01: Boiler combustion controls
 *   - EN 746-2: Industrial thermoprocessing equipment
 */
int cross_limit_check_safety(const cross_limiting_t *cl, double afr_actual)
{
    if (cl == NULL) return -1;
    if (cl->afr_stoich <= 0.0 || afr_actual <= 0.0) return -1;

    double ratio_to_stoich = afr_actual / cl->afr_stoich;

    if (ratio_to_stoich >= 1.0) return 0;           /* Safe — lean */
    if (ratio_to_stoich >= 0.95) return 1;          /* Warning — marginal */
    return 2;                                        /* Danger — rich */
}

/**
 * @brief Compute current AFR from measured flows.
 */
double cross_limit_current_afr(const cross_limiting_t *cl)
{
    if (cl == NULL) return 0.0;
    if (cl->fuel_flow <= 0.0) return 0.0;
    return cl->air_flow / cl->fuel_flow;
}

/**
 * @brief Compute excess air ratio lambda.
 */
double cross_limit_lambda(const cross_limiting_t *cl)
{
    if (cl == NULL) return 0.0;
    if (cl->afr_stoich <= 0.0) return 0.0;

    double afr = cross_limit_current_afr(cl);
    if (afr <= 0.0) return 0.0;

    return afr / cl->afr_stoich;
}

/**
 * @brief Compute cross-limiting safety margins.
 *
 * Positive margins indicate the setpoints are within the
 * safe operating envelope. Negative margins indicate that
 * a constraint is binding (cross-limiting is active).
 */
void cross_limit_margins(const cross_limiting_t *cl,
                          double *margin_air, double *margin_fuel)
{
    if (cl == NULL) return;

    if (margin_air != NULL) {
        /* Air margin: how much SP_air exceeds the safety minimum */
        if (cl->fuel_flow > 0.0 && cl->afr_stoich > 0.0) {
            double air_safety_min = cl->fuel_flow * cl->afr_stoich / cl->r_air_rich;
            *margin_air = cl->sp_air - air_safety_min;
        } else {
            *margin_air = cl->sp_air; /* Safe by default (no fuel) */
        }
    }

    if (margin_fuel != NULL) {
        /* Fuel margin: how much the safety maximum exceeds SP_fuel */
        if (cl->air_flow > 0.0 && cl->afr_stoich > 0.0) {
            double fuel_safety_max = cl->air_flow * cl->r_air_rich / cl->afr_stoich;
            *margin_fuel = fuel_safety_max - cl->sp_fuel;
        } else {
            *margin_fuel = 0.0; /* No air flow → cannot determine */
        }
    }
}

/**
 * @brief Generate cross-limiting diagnostic summary.
 */
int cross_limit_diagnostics(const cross_limiting_t *cl, char *buf, size_t bufsz)
{
    if (cl == NULL || buf == NULL || bufsz == 0) return 0;

    double margin_air, margin_fuel;
    cross_limit_margins(cl, &margin_air, &margin_fuel);

    double afr_actual = cross_limit_current_afr(cl);
    double lambda = cross_limit_lambda(cl);

    return snprintf(buf, bufsz,
        "Cross-Limiting Status:\n"
        "  Mode:            %d (0=none,1=air_leads,2=fuel_leads,3=double)\n"
        "  AFR_stoich:      %.2f\n"
        "  r_air_rich:      %.3f\n"
        "  r_fuel_rich:     %.3f\n"
        "  Air flow:        %.4f\n"
        "  Fuel flow:       %.4f\n"
        "  AFR actual:      %.4f\n"
        "  Lambda:          %.4f\n"
        "  Demand air:      %.4f\n"
        "  Demand fuel:     %.4f\n"
        "  SP air:          %.4f\n"
        "  SP fuel:         %.4f\n"
        "  Air margin:      %.4f\n"
        "  Fuel margin:     %.4f\n"
        "  Air HS active:   %d\n"
        "  Fuel LS active:  %d\n",
        cl->mode, cl->afr_stoich, cl->r_air_rich, cl->r_fuel_rich,
        cl->air_flow, cl->fuel_flow, afr_actual, lambda,
        cl->demand_air, cl->demand_fuel, cl->sp_air, cl->sp_fuel,
        margin_air, margin_fuel,
        cl->air_high_selected, cl->fuel_low_selected);
}
