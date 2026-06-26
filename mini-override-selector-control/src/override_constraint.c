/**
 * @file override_constraint.c
 * @brief Override/Selector Control — Constraint Management Implementation
 *
 * Implements constraint variable monitoring, evaluation, and
 * violation detection for override control systems. Provides
 * priority-based override decision logic and constraint
 * severity assessment.
 *
 * Knowledge Coverage:
 *   L2: Constraint handling, approach factor computation
 *   L3: Constraint state machine, monitoring infrastructure
 *   L5: Priority-based override selection, severity assessment
 *   L6: Time-to-violation prediction
 *
 * Reference:
 *   Seborg, Edgar & Mellichamp (2016). Process Dynamics and Control.
 *   Marlin (2000). Process Control (2nd ed.), Ch. 22: Constraint Control.
 */

#include "override_constraint.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef INFINITY
#define INFINITY (1.0/0.0)
#endif

/* =========================================================================
 * L2 — Constraint Update and Evaluation
 * ========================================================================= */

int constraint_update(constraint_state_t *state, double value, double dt) {
    if (state == NULL || state->def == NULL) return 0;

    double prev_value = state->current_value;
    state->current_value = value;

    /* Update rate of change */
    if (dt > 0.0) {
        state->rate_of_change = (value - prev_value) / dt;
    }

    /* Re-evaluate constraint status */
    const constraint_def_t *def = state->def;

    if (!def->enabled) {
        state->violating = 0;
        state->approaching = 0;
        state->approach_factor = -1.0;
        return 0;
    }

    /* Check hi_hi and lo_lo (emergency) limits */
    int emergency = 0;
    if (value >= def->hi_hi_limit || value <= def->lo_lo_limit) {
        emergency = 1;
    }

    /* Check hi and lo limits */
    int hi_viol = (value >= def->hi_limit) ? 1 : 0;
    int lo_viol = (value <= def->lo_limit) ? 1 : 0;

    state->violating = hi_viol || lo_viol || emergency;

    /* Check if approaching limit (within margin zone) */
    int hi_approaching = 0;
    int lo_approaching = 0;

    if (!hi_viol && def->hi_limit < INFINITY) {
        double hi_margin_start = def->hi_limit - def->margin;
        if (value >= hi_margin_start && value < def->hi_limit) {
            hi_approaching = 1;
        }
    }

    if (!lo_viol && def->lo_limit > -INFINITY) {
        double lo_margin_start = def->lo_limit + def->margin;
        if (value <= lo_margin_start && value > def->lo_limit) {
            lo_approaching = 1;
        }
    }

    state->approaching = hi_approaching || lo_approaching;

    /* Compute approach factor */
    state->approach_factor = constraint_approach_factor(state);

    /* Accumulate violation time */
    if (state->violating) {
        state->time_in_violation += dt;
    }

    /* Return status */
    if (emergency) return 2;
    if (hi_viol || lo_viol) return 1;
    if (hi_approaching || lo_approaching) return -1;
    return 0;
}

double constraint_evaluate_all(constraint_state_t *constraints,
                               int n_constraints,
                               int *active_idx) {
    if (active_idx) *active_idx = -1;

    if (constraints == NULL || n_constraints <= 0) return -1.0;

    /* Find the constraint with the highest priority violation */
    /* Lower priority enum value = higher priority */
    int worst_priority_value = PRIORITY_COUNT + 1;
    double worst_approach = -INFINITY;
    int worst_idx = -1;

    for (int i = 0; i < n_constraints; i++) {
        constraint_state_t *cs = &constraints[i];
        if (cs->def == NULL || !cs->def->enabled) continue;

        double af = constraint_approach_factor(cs);

        /* Consider a constraint "active" if its approach factor > 0.5 */
        if (af > 0.5) {
            int pri = (int)cs->def->priority;
            if (pri < worst_priority_value ||
                (pri == worst_priority_value && af > worst_approach)) {
                worst_priority_value = pri;
                worst_approach = af;
                worst_idx = i;
            }
        }
    }

    if (active_idx && worst_idx >= 0) *active_idx = worst_idx;
    return (worst_idx >= 0) ? worst_approach : -1.0;
}

int constraint_is_violated(const constraint_state_t *state) {
    if (state == NULL || state->def == NULL) return 0;
    return state->violating;
}

int constraint_override_needed(const constraint_state_t *constraints,
                               int n_constraints) {
    if (constraints == NULL || n_constraints <= 0) return 0;

    for (int i = 0; i < n_constraints; i++) {
        const constraint_state_t *cs = &constraints[i];
        if (cs->def == NULL || !cs->def->enabled) continue;

        /* Override needed if any constraint is violated */
        if (cs->violating) return 1;

        /* Or if approaching rapidly */
        if (cs->approach_factor >= 0.5 &&
            cs->rate_of_change > 0.1 * cs->def->margin) {
            /* Rate of approach is significant */
            return 1;
        }
    }

    return 0;
}

/* =========================================================================
 * L2 — Approach Factor Computation
 * ========================================================================= */

double constraint_approach_factor(const constraint_state_t *state) {
    if (state == NULL || state->def == NULL) return -1.0;
    if (!state->def->enabled) return -1.0;

    const constraint_def_t *def = state->def;
    double value = state->current_value;

    double af_hi = -1.0;
    double af_lo = -1.0;

    /* High limit approach factor */
    if (def->hi_limit < INFINITY && def->margin > 0.0) {
        double margin_start = def->hi_limit - def->margin;
        af_hi = (value - margin_start) / def->margin;
        /* Cap at a reasonable max */
        if (af_hi > 10.0) af_hi = 10.0;
    } else if (def->hi_limit < INFINITY && value >= def->hi_limit) {
        af_hi = 1.0; /* Violated with no margin defined */
    }

    /* Low limit approach factor */
    if (def->lo_limit > -INFINITY && def->margin > 0.0) {
        double margin_start = def->lo_limit + def->margin;
        af_lo = (margin_start - value) / def->margin;
        if (af_lo > 10.0) af_lo = 10.0;
    } else if (def->lo_limit > -INFINITY && value <= def->lo_limit) {
        af_lo = 1.0; /* Violated with no margin defined */
    }

    return (af_hi > af_lo) ? af_hi : af_lo;
}

/* =========================================================================
 * L2 — Constraint Initialization Helpers
 * ========================================================================= */

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
                        double rate_limit) {
    if (def == NULL) return;

    def->tag = tag;
    def->description = description;
    def->priority = priority;
    def->ctype = ctype;
    def->hi_limit = hi_limit;
    def->lo_limit = lo_limit;
    def->hi_hi_limit = hi_hi_limit;
    def->lo_lo_limit = lo_lo_limit;
    def->margin = margin;
    def->rate_limit = rate_limit;
    def->enabled = 1;
    def->latched = 0;
    def->degrade_on_fault = 1;
}

void constraint_def_set_enabled(constraint_def_t *def, int enabled) {
    if (def == NULL) return;
    def->enabled = enabled ? 1 : 0;
}

void constraint_def_set_latched(constraint_def_t *def, int latched) {
    if (def == NULL) return;
    def->latched = latched ? 1 : 0;
}

void constraint_reset_latch(constraint_state_t *state) {
    if (state == NULL) return;
    state->overridden = 0;
    state->time_in_violation = 0.0;
}

double constraint_calc_rate(constraint_state_t *state,
                            double new_value, double dt) {
    if (state == NULL || dt <= 0.0) return 0.0;

    double prev = state->current_value;
    double roc = (new_value - prev) / dt;
    state->rate_of_change = roc;
    return roc;
}

/* =========================================================================
 * L3 — Constraint Prediction Functions
 * ========================================================================= */

double constraint_time_to_violation(const constraint_state_t *state) {
    if (state == NULL || state->def == NULL) return INFINITY;

    const constraint_def_t *def = state->def;
    double value = state->current_value;
    double roc = state->rate_of_change;

    /* Determine which limit is relevant based on direction */
    double t_hi = INFINITY;
    double t_lo = INFINITY;

    if (roc > 1e-9 && def->hi_limit < INFINITY) {
        /* Approaching high limit */
        t_hi = (def->hi_limit - value) / roc;
    }

    if (roc < -1e-9 && def->lo_limit > -INFINITY) {
        /* Approaching low limit */
        t_lo = (value - def->lo_limit) / (-roc);
    }

    return (t_hi < t_lo) ? t_hi : t_lo;
}

double constraint_predict_value(const constraint_state_t *state,
                                double horizon) {
    if (state == NULL) return 0.0;

    return state->current_value + state->rate_of_change * horizon;
}

double constraint_severity(const constraint_state_t *state) {
    if (state == NULL || state->def == NULL) return 0.0;

    double af = constraint_approach_factor(state);
    double roc_ratio = 0.0;

    if (state->def->rate_limit > 0.0) {
        roc_ratio = fabs(state->rate_of_change) / state->def->rate_limit;
    }

    /* Severity = 50 * (approach_factor + roc_ratio), capped at 100 */
    double severity = 50.0 * (af + roc_ratio);
    if (severity < 0.0) severity = 0.0;
    if (severity > 100.0) severity = 100.0;
    return severity;
}

/* =========================================================================
 * L5 — Priority-Based Override Logic
 * ========================================================================= */

int constraint_priority_sort(constraint_state_t *constraints,
                             int n_constraints,
                             int *violated_idx,
                             int max_violated) {
    if (constraints == NULL || violated_idx == NULL || n_constraints <= 0) {
        return 0;
    }

    /* Collect violated constraints */
    typedef struct {
        int idx;
        int priority;
        double approach;
    } violated_entry_t;

    violated_entry_t *entries = (violated_entry_t*)malloc(
        (size_t)n_constraints * sizeof(violated_entry_t));
    if (entries == NULL) return 0;

    int nv = 0;
    for (int i = 0; i < n_constraints; i++) {
        constraint_state_t *cs = &constraints[i];
        if (cs->def == NULL || !cs->def->enabled) continue;
        if (cs->violating || cs->approach_factor > 0.5) {
            entries[nv].idx = i;
            entries[nv].priority = (int)cs->def->priority;
            entries[nv].approach = cs->approach_factor;
            nv++;
        }
    }

    /* Sort by priority (ascending) then approach factor (descending) */
    for (int i = 1; i < nv; i++) {
        violated_entry_t key = entries[i];
        int j = i - 1;
        while (j >= 0 &&
               (entries[j].priority > key.priority ||
                (entries[j].priority == key.priority &&
                 entries[j].approach < key.approach))) {
            entries[j + 1] = entries[j];
            j--;
        }
        entries[j + 1] = key;
    }

    /* Fill output with sorted indices */
    int count = (nv < max_violated) ? nv : max_violated;
    for (int i = 0; i < count; i++) {
        violated_idx[i] = entries[i].idx;
    }

    free(entries);
    return count;
}

int constraint_status_string(const constraint_state_t *state,
                             char *buffer, int bufsz) {
    if (state == NULL || buffer == NULL || bufsz <= 0) return 0;

    const char *tag = (state->def && state->def->tag) ? state->def->tag : "?";
    const char *status;
    if (state->violating) {
        status = "VIOLATED";
    } else if (state->approaching) {
        status = "APPROACHING";
    } else {
        status = "OK";
    }

    return snprintf(buffer, (size_t)bufsz,
                    "%s: value=%.3f af=%.2f roc=%.3f [%s]",
                    tag, state->current_value,
                    state->approach_factor,
                    state->rate_of_change,
                    status);
}
