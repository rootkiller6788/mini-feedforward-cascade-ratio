/**
 * @file override_selector.h
 * @brief Override/Selector Control — Constraint Handling & Auctioneering
 *
 * Module: mini-cascade-control-primary-secondary
 * Knowledge Coverage: L1 Definitions, L2 Core Concepts, L3 Engineering Structures
 *
 * Override control (also called constraint control or auctioneering) allows
 * multiple controllers to compete for control of a single manipulated variable.
 * The controller with the most urgent constraint "wins" and takes control.
 *
 * Common architectures:
 *   - Low Select: Protects equipment from minimum limits (e.g., pump suction
 *     pressure, compressor surge, minimum flow)
 *   - High Select: Protects equipment from maximum limits (e.g., max pressure,
 *     max temperature, max motor current)
 *   - Median Select: Safety with 2oo3 voting (redundant sensors)
 *
 * Override is a form of multi-loop control closely related to cascade:
 * both involve multiple controllers, but override controllers are in
 * PARALLEL (competing) rather than SERIES (cascading).
 *
 * Reference: Liptak (2006) Instrument Engineers' Handbook, Vol. 2, Ch. 8.12
 *            Shinskey (1996) Process Control Systems, Ch. 7
 * Curriculum: MIT 6.302, Stanford ENGR205, Purdue ME575, RWTH Aachen ICS
 */

#ifndef OVERRIDE_SELECTOR_H
#define OVERRIDE_SELECTOR_H

#include "cascade_types.h"
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Override/Selector Type Definitions
 * ========================================================================= */

#define OVERRIDE_MAX_CONTROLLERS 6

/** Selector function type */
typedef enum {
    SEL_LOW       = 0,    /**< Minimum selects (auctioneering low)        */
    SEL_HIGH      = 1,    /**< Maximum selects (auctioneering high)       */
    SEL_MEDIAN    = 2,    /**< Median selects (for 3 inputs)              */
    SEL_AVERAGE   = 3,    /**< Average of valid inputs                    */
    SEL_MID_OF_3  = 4,    /**< Middle of three (2oo3 voting)              */
    SEL_FIRST_VALID = 5   /**< First valid (fallback chain)               */
} selector_function_t;

/** Override controller initialization mode */
typedef enum {
    OVERRIDE_INIT_NORMAL  = 0,   /**< Normal initialization                */
    OVERRIDE_INIT_LAST_MV = 1,   /**< Initialize using last selected MV    */
    OVERRIDE_INIT_MIDPOINT = 2   /**< Initialize to midpoint of range      */
} override_init_t;

/** Constraint type for override controller */
typedef enum {
    CONSTRAINT_MINIMUM = 0,      /**< Process low limit (e.g., min flow)   */
    CONSTRAINT_MAXIMUM = 1,      /**< Process high limit (e.g., max press) */
    CONSTRAINT_RATE    = 2,      /**< Rate-of-change constraint            */
    CONSTRAINT_VALVE   = 3       /**< Valve position constraint            */
} constraint_type_t;

/** Individual override constraint definition */
typedef struct {
    const char            *tag;          /**< Constraint identifier tag    */
    constraint_type_t      type;         /**< Min, max, rate, or valve     */
    cascade_pid_controller_t pid;        /**< PID controller for constraint*/
    double                 limit;        /**< Constraint limit value       */
    bool                   active;       /**< Currently selected/active    */
    bool                   enabled;      /**< Constraint is enabled        */
    double                 hysteresis;   /**< Hysteresis for selection     */
    double                 pv;           /**< Process variable value       */
    double                 output;       /**< Controller output            */
    uint32_t               priority;     /**< Priority (lowest = highest)  */
    uint64_t               activation_count; /**< # times activated        */
} override_slot_t;

/** Override selector — manages multiple competing controllers */
typedef struct {
    override_slot_t        slots[OVERRIDE_MAX_CONTROLLERS];
    uint32_t               num_slots;
    selector_function_t    selector_func;
    double                 final_output;     /**< Selected MV output       */
    uint32_t               active_slot_index;/**< Currently selected index  */
    double                 hysteresis_global;/**< Global hysteresis band   */
    bool                   bumpless_enabled; /**< Bumpless transfer active */
    double                 tracking_gain;    /**< Tracking gain for inactive controllers */

    /* Primary (normal) controller */
    cascade_pid_controller_t primary_controller;
    bool                   primary_active;   /**< Primary has control     */

    /* Statistics */
    uint64_t               selection_count;
    double                 last_selection_time;
    uint32_t               max_active_duration; /**< Max time one constraint held */
} override_selector_t;

/* =========================================================================
 * L2: Override Selector Initialization & Configuration
 * ========================================================================= */

/**
 * override_init: Initialize an override selector.
 *
 * Configures the selector function, hysteresis band, and bumpless
 * transfer settings. All constraint slots are cleared.
 *
 * @param selector         Override selector to initialize
 * @param func             Selection function (low/high/median)
 * @param hysteresis       Global hysteresis band for chattering prevention
 * @param output_min       Output lower limit
 * @param output_max       Output upper limit
 *
 * Complexity: O(1)
 */
void override_init(override_selector_t *selector,
                    selector_function_t func,
                    double hysteresis,
                    double output_min, double output_max);

/**
 * override_add_constraint: Add a constraint controller.
 *
 * Each constraint has its own PID controller that computes an output
 * based on how far the process variable is from the constraint limit.
 * All controller outputs are fed to the selector, and the most
 * conservative (or aggressive) one is chosen.
 *
 * For LOW SELECT (protection):
 *   - Constraints are high-limit controllers (direct acting)
 *   - Selector picks the MINIMUM output
 *   - The controller with the most severe violation has lowest output
 *
 * For HIGH SELECT:
 *   - Constraints are low-limit controllers (reverse acting)
 *   - Selector picks the MAXIMUM output
 *
 * @param selector     Override selector
 * @param tag          Constraint identifier
 * @param type         Constraint type (MINIMUM or MAXIMUM)
 * @param limit        Constraint limit value
 * @param kp, ti, td   PID parameters for the constraint
 * @param priority     Priority level (0 = highest)
 * @return             Slot index on success, -1 if full
 *
 * Complexity: O(1)
 */
int override_add_constraint(override_selector_t *selector,
                             const char *tag,
                             constraint_type_t type,
                             double limit,
                             double kp, double ti, double td,
                             uint32_t priority);

/**
 * override_set_primary: Set the primary (normal) controller.
 *
 * The primary controller runs under normal conditions when no
 * constraints are active. Its output is also fed to the selector.
 *
 * @param selector     Override selector
 * @param kp, ti, td   PID parameters for primary controller
 */
void override_set_primary(override_selector_t *selector,
                           double kp, double ti, double td);

/* =========================================================================
 * L3: Override Selection Algorithm
 * ========================================================================= */

/**
 * override_select: Execute override selection.
 *
 * Algorithm:
 *   1. Update each enabled constraint's PID with its PV and limit
 *   2. Update primary controller with its SP and PV
 *   3. Apply selector function to all outputs
 *   4. Apply hysteresis to prevent chattering
 *   5. Set active flag on selected controller
 *   6. Apply bumpless tracking to inactive controllers
 *
 * Hysteresis logic:
 *   If new_selection != current_selection:
 *     If |current_output - new_output| > hysteresis:
 *       Switch to new selection
 *     Else:
 *       Maintain current selection
 *
 * This prevents rapid switching between controllers when outputs are close.
 *
 * @param selector        Override selector with all constraints configured
 * @param primary_sp      Primary (normal) controller setpoint
 * @param primary_pv      Primary (normal) controller process variable
 * @param constraint_pvs  Array of PVs for each constraint slot (NULL = no update)
 * @param Ts              Sample time [seconds]
 * @return                Selected final output value
 *
 * Complexity: O(n) where n = num_slots
 */
double override_select(override_selector_t *selector,
                        double primary_sp, double primary_pv,
                        const double *constraint_pvs,
                        double Ts);

/**
 * override_get_active_tag: Get the tag of the currently active controller.
 *
 * Useful for operator displays to show which constraint or controller
 * is currently in control of the manipulated variable.
 *
 * @param selector  Override selector
 * @return          Tag string of active controller or "PRIMARY"
 *
 * Complexity: O(1)
 */
const char *override_get_active_tag(const override_selector_t *selector);

/**
 * override_is_constraint_active: Check if a specific constraint has control.
 *
 * @param selector   Override selector
 * @param slot_index Constraint slot index
 * @return           true if this constraint is currently selected
 *
 * Complexity: O(1)
 */
bool override_is_constraint_active(const override_selector_t *selector,
                                    uint32_t slot_index);

/* =========================================================================
 * L5: Median Select & Voting Logic
 * ========================================================================= */

/**
 * override_median_select: Median/mid-of-3 selection.
 *
 * Used in safety-critical applications with triplicated sensors
 * (2oo3 voting). The median of three measurements is selected,
 * providing fault tolerance against a single sensor failure.
 *
 * Algorithm:
 *   Given three sorted inputs a ≤ b ≤ c:
 *     median = b
 *   If any input has deviation > tolerance from median:
 *     Flag as suspect (but still include in median if 2oo3)
 *
 * @param values     Array of 3 input values
 * @param tolerance  Maximum allowed deviation from median
 * @param suspect    Output: bitmask of suspect inputs
 * @return           Median value
 *
 * Complexity: O(1)
 */
double override_median_select(const double values[3],
                               double tolerance,
                               uint32_t *suspect);

/**
 * override_mid_of_3_select: 2oo3 voting for redundant sensors.
 *
 * Selects the middle value of three. If one sensor has failed
 * (detected by deviation > tolerance), uses the remaining two
 * valid sensors' average. If two sensors have failed, uses
 * the single remaining valid sensor.
 *
 * Fault detection: |sensor_i - median| > tolerance → sensor_i is faulty
 *
 * @param values     Array of 3 input values
 * @param tolerance  Fault detection tolerance
 * @param faults     Output: number of detected faults
 * @return           Voted value, or NaN if all 3 faulty
 *
 * Complexity: O(1)
 * Ref: IEC 61508 functional safety, 2oo3 voting architecture
 */
double override_mid_of_3_select(const double values[3],
                                 double tolerance,
                                 uint32_t *faults);

/* =========================================================================
 * L5: Auctioneering Performance Analysis
 * ========================================================================= */

/**
 * override_selection_frequency: Analyze selection switching frequency.
 *
 * Excessive switching between controllers indicates:
 *   - Insufficient hysteresis
 *   - Poorly tuned constraint controllers
 *   - Process oscillating near constraint boundary
 *
 * @param selector         Override selector
 * @param window_seconds   Analysis window
 * @param switches_per_hour Output: switching rate
 * @return                 0 if acceptable, 1 if excessive
 *
 * Complexity: O(1)
 */
int override_selection_frequency(const override_selector_t *selector,
                                  double window_seconds,
                                  double *switches_per_hour);

/**
 * override_reset_statistics: Reset all selection statistics.
 *
 * Zeroes activation counts, switching history, and timing data.
 *
 * Complexity: O(n) where n = num_slots
 */
void override_reset_statistics(override_selector_t *selector);

#ifdef __cplusplus
}
#endif

#endif /* OVERRIDE_SELECTOR_H */
