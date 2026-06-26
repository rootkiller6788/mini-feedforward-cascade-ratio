/**
 * @file override_constraint.h
 * @brief Override/Selector Control — Constraint Management
 *
 * Implements constraint variable monitoring, violation detection,
 * and priority-based constraint handling. Constraints form the
 * foundation of override control — when a constraint approaches
 * its limit, the override system transfers control from the
 * primary controller to a constraint controller.
 *
 * Reference:
 *   Seborg, D.E., Edgar, T.F. & Mellichamp, D.A. (2016).
 *   Process Dynamics and Control (4th ed.), Wiley. Chapter 16:
 *   Enhanced Single-Loop Control Strategies.
 *
 *   Marlin, T.E. (2000). Process Control: Designing Processes
 *   and Control Systems for Dynamic Performance (2nd ed.).
 *   McGraw-Hill. Chapter 22: Constraint Control.
 *
 * Knowledge Coverage:
 *   L2 — Core Concepts: Constraint handling, approach factor
 *   L3 — Engineering Structures: Constraint monitoring, violation logic
 *   L5 — Algorithms: Priority-based override selection
 *
 * Course Alignment:
 *   Purdue ME 575: Constraint control in process industries
 *   Tsinghua: 过程约束控制
 */

#ifndef OVERRIDE_CONSTRAINT_H
#define OVERRIDE_CONSTRAINT_H

#include "override_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L2 — Constraint Evaluation Functions
 * ========================================================================= */

/**
 * Update a single constraint state with a new measurement
 *
 * Evaluates whether the constraint is approaching or violating
 * its high and low limits, and computes the approach factor.
 *
 * Approach factor = max(approach_hi, approach_lo), where:
 *   approach_hi = (value - (hi_limit - margin)) / margin
 *   approach_lo = ((lo_limit + margin) - value) / margin
 *
 * A factor >= 1.0 means the constraint is violated.
 * A factor >= 0.0 means the constraint is in the margin zone.
 *
 * @param state  Constraint state to update
 * @param value  New measured value
 * @param dt     Time step [s]
 * @return 0 if no issue, 1 if limiting, 2 if violating
 */
int constraint_update(constraint_state_t *state, double value, double dt);

/**
 * Evaluate all constraints and return the most critical one
 *
 * Scans through all constraints, updates their approach factors,
 * and identifies the constraint that is most urgently approaching
 * or violating its limit. Priority ordering:
 *   PRIORITY_EMERGENCY > PRIORITY_SAFETY > PRIORITY_CONSTRAINT
 *
 * @param constraints Array of constraint states
 * @param n_constraints Number of constraints
 * @param active_idx Output: index of most critical constraint (-1 if none)
 * @return Maximum approach factor across all constraints
 */
double constraint_evaluate_all(constraint_state_t *constraints,
                               int n_constraints,
                               int *active_idx);

/**
 * Check if a specific constraint is currently being violated
 *
 * @param state Constraint state
 * @return 1 if violated (hi or lo), 0 otherwise
 */
int constraint_is_violated(const constraint_state_t *state);

/**
 * Determine if override action is needed based on constraint evaluation
 *
 * Override is needed when:
 * 1. Any constraint approach factor >= 1.0 (violation)
 * 2. Any constraint approach factor >= 0.5 with rapid approach rate
 *
 * @param constraints Array of constraint states
 * @param n_constraints Number of constraints
 * @return 1 if override needed, 0 otherwise
 */
int constraint_override_needed(const constraint_state_t *constraints,
                               int n_constraints);

/**
 * Compute the approach factor for a constraint
 *
 * The approach factor quantifies how close a measurement is to
 * its limit, accounting for the safety margin.
 *
 * For high limit:  factor = (value - (hi_limit - margin)) / margin
 * For low limit:   factor = ((lo_limit + margin) - value) / margin
 *
 * @param state Constraint state
 * @return Approach factor (<0 = safe, 0..=1 = margin zone, >1 = violated)
 */
double constraint_approach_factor(const constraint_state_t *state);

/* =========================================================================
 * L2 — Constraint Initialization Functions
 * ========================================================================= */

/**
 * Set constraint definition with all parameters
 *
 * @param def         Pointer to constraint definition
 * @param tag         Tag name
 * @param description Human-readable description
 * @param priority    Priority level
 * @param ctype       Constraint type
 * @param hi_limit    High limit
 * @param lo_limit    Low limit
 * @param hi_hi_limit High-high limit
 * @param lo_lo_limit Low-low limit
 * @param margin      Safety margin
 * @param rate_limit  Rate-of-change limit
 */
void constraint_def_set(constraint_def_t *def,
                        const char *tag,
                        const char *description,
                        override_priority_t priority,
                        constraint_type_t ctype,
                        double hi_limit,
                        double lo_limit,
                        double hi_hi_limit,
                        double lo_lo_limit,
                        double margin,
                        double rate_limit);

/**
 * Enable or disable a constraint
 *
 * @param def     Constraint definition
 * @param enabled 1 to enable, 0 to disable
 */
void constraint_def_set_enabled(constraint_def_t *def, int enabled);

/**
 * Set constraint latching behavior
 *
 * When latched, a constraint violation persists until manually reset.
 * This is used for safety-critical constraints where auto-recovery
 * is not allowed.
 *
 * @param def     Constraint definition
 * @param latched 1 to latch, 0 for auto-reset
 */
void constraint_def_set_latched(constraint_def_t *def, int latched);

/**
 * Reset a latched constraint after manual verification
 *
 * @param state Constraint state to reset
 */
void constraint_reset_latch(constraint_state_t *state);

/**
 * Calculate the rate of change of a constraint value
 *
 * Uses a first-order difference approximation:
 *   roc = (current_value - prev_value) / dt
 *
 * @param state      Constraint state (updated with new roc)
 * @param new_value  Current measurement
 * @param dt         Time step [s]
 * @return Rate of change [units/s]
 */
double constraint_calc_rate(constraint_state_t *state,
                            double new_value, double dt);

/* =========================================================================
 * L3 — Constraint Prediction Functions
 * ========================================================================= */

/**
 * Predict time to constraint violation based on current rate
 *
 * Assuming constant rate of change, estimate the time until
 * the constraint value reaches its limit.
 *
 * For high limit:  t = (hi_limit - value) / roc  if roc > 0
 * For low limit:   t = (value - lo_limit) / |roc| if roc < 0
 *
 * @param state Constraint state
 * @return Time to violation [s], or INFINITY if approaching away from limit
 */
double constraint_time_to_violation(const constraint_state_t *state);

/**
 * Predict constraint value after a given time horizon
 *
 * Using linear extrapolation: v(t+horizon) = v(t) + roc * horizon
 *
 * @param state   Constraint state
 * @param horizon Time horizon [s]
 * @return Predicted value
 */
double constraint_predict_value(const constraint_state_t *state,
                                double horizon);

/**
 * Determine constraint severity index (0-100)
 *
 * Combines approach factor and rate of change to produce a
 * normalized severity score:
 *   severity = 50 * (approach_factor + |roc|/rate_limit)
 * capped at 100.
 *
 * @param state Constraint state
 * @return Severity index [0..100]
 */
double constraint_severity(const constraint_state_t *state);

/* =========================================================================
 * L5 — Priority-Based Override Logic
 * ========================================================================= */

/**
 * Determine which constraint takes precedence in an override situation
 *
 * When multiple constraints are violated simultaneously, the one
 * with the highest priority (lowest priority enum value) takes
 * precedence. If two constraints have the same priority, the one
 * with the higher approach factor takes precedence.
 *
 * @param constraints Array of constraint states
 * @param n_constraints Number of constraints
 * @param violated_idx Output: array of violated constraint indices
 * @param max_violated Maximum number to return
 * @return Number of violated constraints found
 */
int constraint_priority_sort(constraint_state_t *constraints,
                             int n_constraints,
                             int *violated_idx,
                             int max_violated);

/**
 * Generate a human-readable constraint status summary
 *
 * @param state  Constraint state
 * @param buffer Output buffer
 * @param bufsz  Buffer size
 * @return Number of characters written (not including null)
 */
int constraint_status_string(const constraint_state_t *state,
                             char *buffer, int bufsz);

#ifdef __cplusplus
}
#endif

#endif /* OVERRIDE_CONSTRAINT_H */
