/**
 * @file override_core.c
 * @brief Override/Selector Control — Core Implementation
 *
 * Implements initialization, validation, string conversion, and
 * core utility functions for the override selector control system.
 *
 * Knowledge Coverage:
 *   L1: Data validation, initialization, string converters
 *   L2: Core override logic (selector execution, constraint evaluation,
 *       transfer handling, tracking updates)
 *   L3: Runtime state management
 */

#include "override_core.h"
#include "override_selector.h"
#include "override_constraint.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifndef INFINITY
#define INFINITY (1.0/0.0)
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =========================================================================
 * L1 — Enumeration String Converters
 * ========================================================================= */

const char* override_mode_name(override_mode_t mode) {
    switch (mode) {
        case OVERRIDE_MODE_DISABLED:    return "Disabled";
        case OVERRIDE_MODE_PRIMARY:     return "Primary Controller Active";
        case OVERRIDE_MODE_OVERRIDE:    return "Override Controller Active";
        case OVERRIDE_MODE_MANUAL:      return "Manual";
        case OVERRIDE_MODE_INITIALIZE:  return "Initializing";
        case OVERRIDE_MODE_FAULT:       return "Fault";
        default:                        return "Unknown Mode";
    }
}

const char* selector_type_name(selector_type_t stype) {
    switch (stype) {
        case SELECTOR_HIGH:       return "High Select";
        case SELECTOR_LOW:        return "Low Select";
        case SELECTOR_MEDIAN:     return "Median Select";
        case SELECTOR_WEIGHTED:   return "Weighted Select";
        case SELECTOR_AUCTIONEER: return "Auctioneering";
        case SELECTOR_AVERAGE:    return "Average Select";
        case SELECTOR_CUSTOM:     return "Custom Selector";
        default:                  return "Unknown Selector";
    }
}

const char* constraint_type_name(constraint_type_t ctype) {
    switch (ctype) {
        case CONSTRAINT_NONE:      return "None";
        case CONSTRAINT_HARD_ABS:  return "Hard Absolute Limit";
        case CONSTRAINT_HARD_RATE: return "Hard Rate Limit";
        case CONSTRAINT_SOFT_ABS:  return "Soft Absolute Limit";
        case CONSTRAINT_SOFT_RATE: return "Soft Rate Limit";
        case CONSTRAINT_QUALITY:   return "Quality Constraint";
        case CONSTRAINT_ECONOMIC:  return "Economic Constraint";
        default:                   return "Unknown Constraint";
    }
}

const char* tracking_mode_name(tracking_mode_t tmode) {
    switch (tmode) {
        case TRACK_NONE:           return "No Tracking";
        case TRACK_PV:             return "Track PV";
        case TRACK_OUTPUT:         return "Track Output";
        case TRACK_EXTERNAL_RESET: return "External Reset (IEC 61131-3)";
        case TRACK_PREDICTIVE:     return "Predictive Tracking";
        default:                   return "Unknown Tracking";
    }
}

const char* override_priority_name(override_priority_t priority) {
    switch (priority) {
        case PRIORITY_EMERGENCY:    return "Emergency (ESD)";
        case PRIORITY_SAFETY:       return "Safety (SIS)";
        case PRIORITY_CONSTRAINT:   return "Process Constraint";
        case PRIORITY_PRIMARY:      return "Primary Control";
        case PRIORITY_OPTIMIZATION: return "Economic Optimization";
        case PRIORITY_DIAGNOSTIC:   return "Diagnostic/Monitoring";
        default:                    return "Unknown Priority";
    }
}

const char* xfer_mode_name(xfer_mode_t xmode) {
    switch (xmode) {
        case XFER_BUMPLESS:     return "Bumpless Transfer";
        case XFER_BUMPED:       return "Bumped Transfer";
        case XFER_RATE_LIMITED: return "Rate-Limited Transfer";
        case XFER_RAMP:         return "Ramped Transfer";
        default:                return "Unknown Transfer Mode";
    }
}

/* =========================================================================
 * L1 — Initialization Functions
 * ========================================================================= */

int override_state_init(override_state_t *state,
                        const override_config_t *config,
                        int n_controllers,
                        int n_constraints) {
    if (state == NULL || config == NULL) return -1;
    if (n_controllers < 1 || n_controllers > config->max_controllers) return -1;
    if (n_constraints < 0 || n_constraints > config->max_constraints) return -1;

    memset(state, 0, sizeof(override_state_t));

    state->mode = OVERRIDE_MODE_INITIALIZE;
    state->prev_mode = OVERRIDE_MODE_DISABLED;
    state->selector_type = SELECTOR_LOW;
    state->xfer_mode = XFER_BUMPLESS;

    state->active_controller = -1;
    state->prev_controller = -1;
    state->num_controllers = n_controllers;
    state->num_constraints = n_constraints;

    state->selector_output = 0.0;
    state->prev_output = 0.0;

    /* Allocate arrays */
    state->controller_outputs = (double*)calloc((size_t)n_controllers, sizeof(double));
    state->controller_tracking = (double*)calloc((size_t)n_controllers, sizeof(double));
    state->controller_enabled = (int*)calloc((size_t)n_controllers, sizeof(int));
    state->controller_faulted = (int*)calloc((size_t)n_controllers, sizeof(int));

    if (n_constraints > 0) {
        state->constraints = (constraint_state_t*)calloc((size_t)n_constraints,
                                                          sizeof(constraint_state_t));
    } else {
        state->constraints = NULL;
    }

    /* Check allocation success */
    if (state->controller_outputs == NULL ||
        state->controller_tracking == NULL ||
        state->controller_enabled == NULL ||
        state->controller_faulted == NULL) {
        override_state_free(state);
        return -1;
    }
    if (n_constraints > 0 && state->constraints == NULL) {
        override_state_free(state);
        return -1;
    }

    state->hysteresis = config->default_hysteresis;
    state->output_rate_limit = config->default_rate_limit;
    state->sample_time = 0.0;
    state->iteration_count = 0;
    state->initialized = 1;
    state->fault_latch = 0;
    state->standby_active = 0;

    return 0;
}

void override_state_free(override_state_t *state) {
    if (state == NULL) return;
    free(state->controller_outputs);
    free(state->controller_tracking);
    free(state->controller_enabled);
    free(state->controller_faulted);
    free(state->constraints);
    state->controller_outputs = NULL;
    state->controller_tracking = NULL;
    state->controller_enabled = NULL;
    state->controller_faulted = NULL;
    state->constraints = NULL;
    state->initialized = 0;
}

void override_pid_params_init(override_pid_params_t *params) {
    if (params == NULL) return;
    params->Kc = 1.0;
    params->Ti = 60.0;
    params->Td = 0.0;
    params->N = 10.0;
    params->Ts = 0.1;
    params->b = 1.0;
    params->c = 0.0;
    params->u_min = 0.0;
    params->u_max = 100.0;
    params->tracking_gain = 1.0;
}

void override_controller_init(override_controller_t *ctrl,
                              int id,
                              const char *tag,
                              const char *desc,
                              override_priority_t priority) {
    if (ctrl == NULL) return;
    memset(ctrl, 0, sizeof(override_controller_t));
    ctrl->id = id;
    ctrl->tag = tag;
    ctrl->description = desc;
    ctrl->priority = priority;
    override_pid_params_init(&ctrl->params);
    ctrl->setpoint = 0.0;
    ctrl->pv = 0.0;
    ctrl->output = 0.0;
    ctrl->tracking_value = 0.0;
    ctrl->integral = 0.0;
    ctrl->last_error = 0.0;
    ctrl->last_pv = 0.0;
    ctrl->last_output = 0.0;
    ctrl->active = 0;
    ctrl->enabled = 1;
    ctrl->faulted = 0;
    ctrl->in_manual = 0;
    ctrl->manual_output = 0.0;
    ctrl->saturation_count = 0.0;
    ctrl->windup_limited = 0;
}

void override_constraint_def_init(constraint_def_t *def) {
    if (def == NULL) return;
    memset((void*)def, 0, sizeof(constraint_def_t));
    def->priority = PRIORITY_CONSTRAINT;
    def->ctype = CONSTRAINT_HARD_ABS;
    def->hi_limit = INFINITY;
    def->lo_limit = -INFINITY;
    def->hi_hi_limit = INFINITY;
    def->lo_lo_limit = -INFINITY;
    def->margin = 0.0;
    def->rate_limit = 0.0;
    def->enabled = 1;
    def->latched = 0;
    def->degrade_on_fault = 1;
}

int override_constraint_def_is_valid(const constraint_def_t *def) {
    if (def == NULL) return 0;
    if (def->tag == NULL || def->tag[0] == '\0') return 0;
    if (def->hi_limit <= def->lo_limit) return 0;
    if (def->hi_hi_limit < def->hi_limit) return 0;
    if (def->lo_lo_limit > def->lo_limit) return 0;
    if (def->margin < 0.0) return 0;
    if (def->rate_limit < 0.0) return 0;
    return 1;
}

void override_constraint_state_init(constraint_state_t *state,
                                    const constraint_def_t *def) {
    if (state == NULL) return;
    memset(state, 0, sizeof(constraint_state_t));
    state->def = def;
    state->current_value = 0.0;
    state->rate_of_change = 0.0;
    state->approach_factor = 0.0;
    state->violating = 0;
    state->approaching = 0;
    state->faulted = 0;
    state->overridden = 0;
    state->time_in_violation = 0.0;
}

void override_state_reset(override_state_t *state) {
    if (state == NULL) return;
    state->mode = OVERRIDE_MODE_INITIALIZE;
    state->prev_mode = OVERRIDE_MODE_DISABLED;
    state->active_controller = -1;
    state->prev_controller = -1;
    state->selector_output = 0.0;
    state->prev_output = 0.0;
    state->iteration_count = 0;
    state->fault_latch = 0;

    if (state->controller_outputs && state->num_controllers > 0) {
        memset(state->controller_outputs, 0,
               (size_t)state->num_controllers * sizeof(double));
    }
    if (state->controller_tracking && state->num_controllers > 0) {
        memset(state->controller_tracking, 0,
               (size_t)state->num_controllers * sizeof(double));
    }
    if (state->controller_faulted && state->num_controllers > 0) {
        memset(state->controller_faulted, 0,
               (size_t)state->num_controllers * sizeof(int));
    }
}

void surge_control_init(surge_control_t *surge,
                        double slope,
                        double intercept,
                        double min_margin) {
    if (surge == NULL) return;
    memset(surge, 0, sizeof(surge_control_t));
    surge->surge_line_slope = slope;
    surge->surge_line_intercept = intercept;
    surge->min_surge_margin = min_margin;
    surge->surge_margin = 100.0;
    surge->surge_override_active = 0;
    surge->surge_detected = 0;
}

void vpc_state_init(vpc_state_t *vpc,
                    double setpoint,
                    double vpc_min,
                    double vpc_max) {
    if (vpc == NULL) return;
    memset(vpc, 0, sizeof(vpc_state_t));
    vpc->enabled = 1;
    vpc->vpc_setpoint = setpoint;
    vpc->vpc_min = vpc_min;
    vpc->vpc_max = vpc_max;
    vpc->main_valve_position = setpoint;
    vpc->vpc_valve_position = 0.0;
    override_pid_params_init(&vpc->vpc_pid);
    vpc->vpc_pid.Kc = 0.5;   /* VPC default: gentle P gain */
    vpc->vpc_pid.Ti = 60.0;  /* VPC default: slow integral */
    vpc->vpc_output = 0.0;
    vpc->vpc_active = 0;
    vpc->integral = 0.0;
    vpc->last_error = 0.0;
}

/* =========================================================================
 * L1 — Validation Functions
 * ========================================================================= */

int override_pid_params_is_valid(const override_pid_params_t *params) {
    if (params == NULL) return 0;
    if (params->Kc <= 0.0) return 0;
    if (params->Ti <= 0.0) return 0;
    if (params->Td < 0.0) return 0;
    if (params->N < 2.0 || params->N > 50.0) return 0;
    if (params->Ts <= 0.0) return 0;
    if (params->b < 0.0 || params->b > 1.0) return 0;
    if (params->c < 0.0 || params->c > 1.0) return 0;
    if (params->u_min >= params->u_max) return 0;
    if (params->tracking_gain <= 0.0) return 0;
    return 1;
}

int override_state_is_valid(const override_state_t *state) {
    if (state == NULL) return 0;
    if (!state->initialized) return 0;
    if (state->num_controllers < 1) return 0;
    if (state->num_constraints < 0) return 0;
    if (state->controller_outputs == NULL) return 0;
    if (state->controller_tracking == NULL) return 0;
    if (state->controller_enabled == NULL) return 0;
    if (state->hysteresis < 0.0) return 0;
    if (state->output_rate_limit < 0.0) return 0;
    return 1;
}

/* =========================================================================
 * L2 — Core Override Logic Implementation
 * ========================================================================= */

/**
 * Execute one cycle of override selector logic.
 *
 * Algorithm:
 * 1. Evaluate all constraints → determine if override needed
 * 2. For each enabled controller, collect its output
 * 3. Apply selector logic (HS/LS/MS) to choose active controller
 * 4. If active controller changed, perform transfer
 * 5. Update tracking signals for inactive controllers
 * 6. Apply output rate limiting
 * 7. Update diagnostic state
 */
double override_execute(override_state_t *state,
                        override_controller_t *controllers,
                        int n_controllers) {
    if (state == NULL || controllers == NULL) return -1.0;
    if (n_controllers < 1) return -1.0;

    double dt = state->sample_time;
    state->iteration_count++;

    /* Step 1: Evaluate constraints */
    int constraint_idx = -1;
    double max_approach = 0.0;
    if (state->num_constraints > 0 && state->constraints != NULL) {
        max_approach = constraint_evaluate_all(state->constraints,
                                                state->num_constraints,
                                                &constraint_idx);
    }

    /* Step 2: Collect controller outputs and compute selector */
    int selected = -1;
    for (int i = 0; i < n_controllers && i < state->num_controllers; i++) {
        state->controller_outputs[i] = controllers[i].output;
        state->controller_enabled[i] = controllers[i].enabled && !controllers[i].faulted;
        state->controller_faulted[i] = controllers[i].faulted;
    }

    /* Step 3: Apply selector logic */
    selected = override_select_controller(state, controllers, n_controllers);

    /* Step 4: Handle transfer if active controller changed */
    if (selected >= 0 && selected != state->active_controller) {
        override_transfer(state, controllers,
                          state->active_controller, selected);
        state->prev_controller = state->active_controller;
        state->active_controller = selected;
    }

    /* Step 5: Update tracking */
    override_update_tracking(state, controllers, n_controllers);

    /* Step 6: Rate-limit selector output */
    if (selected >= 0) {
        state->selector_output = selector_rate_limit(
            state->controller_outputs[selected],
            state->prev_output,
            state->output_rate_limit,
            dt > 0.0 ? dt : 1.0
        );
    }
    state->prev_output = state->selector_output;

    /* Step 7: Determine final mode */
    if (state->fault_latch) {
        state->prev_mode = state->mode;
        state->mode = OVERRIDE_MODE_FAULT;
    } else if (constraint_idx >= 0 && max_approach >= 1.0) {
        state->prev_mode = state->mode;
        state->mode = OVERRIDE_MODE_OVERRIDE;
    } else {
        state->prev_mode = state->mode;
        state->mode = OVERRIDE_MODE_PRIMARY;
    }

    return state->selector_output;
}

int override_evaluate_constraints(override_state_t *state) {
    if (state == NULL || state->constraints == NULL) return -1;

    int worst_idx = -1;
    double worst_approach = -INFINITY;

    for (int i = 0; i < state->num_constraints; i++) {
        constraint_state_t *cs = &state->constraints[i];
        if (cs->def == NULL || !cs->def->enabled) continue;

        double af = constraint_approach_factor(cs);
        if (af > worst_approach) {
            worst_approach = af;
            worst_idx = i;
        }
    }

    return worst_idx;
}

int override_transfer(override_state_t *state,
                      override_controller_t *controllers,
                      int from_idx,
                      int to_idx) {
    if (state == NULL || controllers == NULL) return -1;
    if (to_idx < 0 || to_idx >= state->num_controllers) return -1;

    /* Mark inactive controllers */
    for (int i = 0; i < state->num_controllers; i++) {
        controllers[i].active = (i == to_idx) ? 1 : 0;
    }

    /* Initialize new active controller with current output for bumpless */
    if (state->xfer_mode == XFER_BUMPLESS || state->xfer_mode == XFER_RATE_LIMITED) {
        double current_out = state->selector_output;
        /* Set tracking value for the newly active controller so it starts
           from the current output without a bump */
        controllers[to_idx].last_output = current_out;
        /* The PID update will handle bumpless initialization via tracking */
    }

    return 0;
}

void override_update_tracking(override_state_t *state,
                              override_controller_t *controllers,
                              int n_controllers) {
    if (state == NULL || controllers == NULL) return;

    double active_output = 0.0;
    if (state->active_controller >= 0 &&
        state->active_controller < state->num_controllers) {
        active_output = state->selector_output;
    }

    for (int i = 0; i < n_controllers && i < state->num_controllers; i++) {
        if (i != state->active_controller) {
            /* Inactive controller tracks the active output */
            controllers[i].tracking_value = active_output;
            state->controller_tracking[i] = active_output;
        } else {
            state->controller_tracking[i] = controllers[i].output;
        }
    }
}

int override_select_controller(override_state_t *state,
                               override_controller_t *controllers,
                               int n_controllers) {
    if (state == NULL || controllers == NULL || n_controllers < 1) return -1;

    /* Only consider enabled, non-faulted controllers */
    double *vals = state->controller_outputs;
    int *valid = state->controller_enabled;
    int nc = (n_controllers < state->num_controllers) ? n_controllers
                                                       : state->num_controllers;
    int selected = -1;

    switch (state->selector_type) {
        case SELECTOR_HIGH:
            selector_high_hysteresis(vals, valid, nc,
                                     state->active_controller,
                                     state->hysteresis, &selected);
            break;
        case SELECTOR_LOW:
            selector_low_hysteresis(vals, valid, nc,
                                    state->active_controller,
                                    state->hysteresis, &selected);
            break;
        case SELECTOR_MEDIAN:
            selector_median_hysteresis(vals, valid, nc,
                                       state->active_controller,
                                       state->hysteresis, &selected);
            break;
        case SELECTOR_WEIGHTED:
        case SELECTOR_AVERAGE:
            /* For weighted/average, just pick the max for override
               (these are for blending, not override) */
            selector_high(vals, valid, nc, &selected);
            break;
        case SELECTOR_AUCTIONEER:
            if (nc >= 3) {
                selector_auctioneer_3(vals[0], valid[0],
                                      vals[1], valid[1],
                                      vals[2], valid[2], &selected);
            } else {
                selector_median(vals, valid, nc, &selected);
            }
            break;
        default:
            selected = 0;
            break;
    }

    return selected;
}
