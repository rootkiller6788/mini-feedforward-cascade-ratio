/**
 * @file override_diagnostics.c
 * @brief Override/Selector Control — Diagnostics Implementation
 *
 * Implements diagnostic monitoring, event logging, health checks,
 * and alarm integration for override selector systems.
 *
 * Knowledge Coverage:
 *   L2: Diagnostic monitoring concepts
 *   L3: Event logging ring buffer, health check state machine
 *   L7: ISA-18.2 alarm management, industrial audit trail
 *
 * Reference:
 *   ISA-18.2 (2016). Management of Alarm Systems.
 *   IEC 62682 (2014). Management of Alarms Systems.
 */

#include "override_diagnostics.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifndef INFINITY
#define INFINITY (1.0/0.0)
#endif

/* =========================================================================
 * L2 — Diagnostic Initialization
 * ========================================================================= */

void override_diag_init(override_diag_t *diag) {
    if (diag == NULL) return;
    memset(diag, 0, sizeof(override_diag_t));
}

void override_diag_reset(override_diag_t *diag) {
    if (diag == NULL) return;
    int saved_activation[PRIORITY_COUNT];
    memcpy(saved_activation, diag->controller_activation, sizeof(saved_activation));
    memset(diag, 0, sizeof(override_diag_t));
    memcpy(diag->controller_activation, saved_activation, sizeof(saved_activation));
}

/* =========================================================================
 * L2 — Event Logging (Ring Buffer)
 * ========================================================================= */

void override_event_log_init(override_event_log_t *log) {
    if (log == NULL) return;
    memset(log, 0, sizeof(override_event_log_t));
    log->event_id_counter = 1;
}

void override_event_log_add(override_event_log_t *log,
                            double timestamp,
                            override_mode_t old_mode,
                            override_mode_t new_mode,
                            int old_ctrl, int new_ctrl,
                            double output,
                            const char *reason) {
    if (log == NULL) return;

    override_event_t *evt = &log->events[log->head];
    evt->event_id = log->event_id_counter++;
    evt->timestamp = timestamp;
    evt->old_mode = old_mode;
    evt->new_mode = new_mode;
    evt->old_controller = old_ctrl;
    evt->new_controller = new_ctrl;
    evt->selected_output = output;
    evt->reason = reason;

    log->head = (log->head + 1) % OVERRIDE_EVENT_BUFFER_SIZE;
    if (log->head == 0) log->wrapped = 1;
    log->count++;
}

const override_event_t* override_event_get(const override_event_log_t *log,
                                           int n) {
    if (log == NULL || n < 0 || n >= log->count) return NULL;

    int total = log->wrapped ? OVERRIDE_EVENT_BUFFER_SIZE : log->count;
    int idx = log->wrapped ?
              (log->head - 1 - n + OVERRIDE_EVENT_BUFFER_SIZE) % OVERRIDE_EVENT_BUFFER_SIZE :
              (total - 1 - n);

    if (idx < 0 || idx >= OVERRIDE_EVENT_BUFFER_SIZE) return NULL;
    return &log->events[idx];
}

int override_event_count(const override_event_log_t *log) {
    if (log == NULL) return 0;
    return log->wrapped ? OVERRIDE_EVENT_BUFFER_SIZE : log->count;
}

void override_event_log_print(const override_event_log_t *log, FILE *stream) {
    if (log == NULL || stream == NULL) return;

    int n = override_event_count(log);
    fprintf(stream, "=== Override Event Log (%d events) ===\n", n);
    fprintf(stream, "ID   | Timestamp | From          | To            | OldCtrl | NewCtrl | Output  | Reason\n");
    fprintf(stream, "-----|-----------|---------------|---------------|---------|---------|---------|-------\n");

    for (int i = 0; i < n; i++) {
        const override_event_t *evt = override_event_get(log, i);
        if (evt == NULL) continue;

        fprintf(stream, "%-4d | %9.3f | %-13s | %-13s | %7d | %7d | %7.2f | %s\n",
                evt->event_id,
                evt->timestamp,
                override_mode_name(evt->old_mode),
                override_mode_name(evt->new_mode),
                evt->old_controller,
                evt->new_controller,
                evt->selected_output,
                evt->reason ? evt->reason : "");
    }
}

/* =========================================================================
 * L3 — Health Monitoring
 * ========================================================================= */

int override_health_check(const override_state_t *state,
                          const override_controller_t *controllers,
                          int n_controllers) {
    if (state == NULL || controllers == NULL) return -2;

    /* Check 1: Is the system initialized? */
    if (!state->initialized) return -2;

    /* Check 2: Are there any enabled controllers? */
    int enabled_count = 0;
    for (int i = 0; i < n_controllers; i++) {
        if (controllers[i].enabled && !controllers[i].faulted) {
            enabled_count++;
        }
    }
    if (enabled_count == 0) return -2;

    /* Check 3: Are there faulted controllers? */
    int faulted = override_count_faults(controllers, n_controllers);
    if (faulted > 0 && faulted == n_controllers) return -2;  /* All faulted */
    if (faulted > 0) return -1;  /* Some faulted — degraded */

    /* Check 4: Is selector output within reasonable range? */
    if (state->active_controller >= 0) {
        double out = state->selector_output;
        /* Output should be non-NaN and finite */
        if (isnan(out) || isinf(out)) return -1;
    }

    return 0; /* Healthy */
}

int override_count_faults(const override_controller_t *controllers, int n) {
    if (controllers == NULL) return 0;
    int count = 0;
    for (int i = 0; i < n; i++) {
        if (controllers[i].faulted) count++;
    }
    return count;
}

int override_detect_chatter(const override_state_t *state,
                            double window_s, int threshold) {
    if (state == NULL) return 0;

    /* Simplified chatter detection based on mode change frequency.
       In a full implementation, this would track the timestamp
       of each mode change and count changes within the window. */
    if (state->iteration_count > 0) {
        double avg_period = state->sample_time;
        if (avg_period > 0.0) {
            double switches_per_window = window_s / avg_period;
            /* If the mode changed in more than threshold of last few cycles */
            (void)switches_per_window;
        }
    }

    /* Simplified: check if active controller keeps changing */
    if (state->active_controller != state->prev_controller &&
        state->iteration_count > 3) {
        /* Quick change detected — could be chatter */
        return 1;
    }

    return 0;
}

void override_controller_usage(const override_state_t *state,
                               const override_controller_t *controllers,
                               int n_controllers,
                               double *fractions) {
    if (state == NULL || controllers == NULL || fractions == NULL) return;
    if (n_controllers <= 0 || state->iteration_count <= 0) {
        for (int i = 0; i < n_controllers; i++) fractions[i] = 0.0;
        return;
    }

    /* In a complete implementation, we would track per-controller
       active time. Here we use a simplified fraction based on
       the current active controller. */
    for (int i = 0; i < n_controllers; i++) {
        fractions[i] = 0.0;
    }

    if (state->active_controller >= 0 &&
        state->active_controller < n_controllers) {
        fractions[state->active_controller] = 1.0;
    }
}

/* =========================================================================
 * L3 — Diagnostic Reporting
 * ========================================================================= */

int override_diag_summary(const override_state_t *state,
                          const override_diag_t *diag,
                          char *buffer, int bufsz) {
    if (state == NULL || diag == NULL || buffer == NULL || bufsz <= 0) return 0;

    return snprintf(buffer, (size_t)bufsz,
                    "Override Status: Mode=%s, ActiveCtrl=%d, "
                    "Switches=%d, Violations=%d, Output=%.2f",
                    override_mode_name(state->mode),
                    state->active_controller,
                    diag->total_mode_switches,
                    diag->constraint_violations,
                    state->selector_output);
}

void override_status_print(const override_state_t *state,
                           const override_controller_t *controllers,
                           int n_controllers,
                           FILE *stream) {
    if (state == NULL || controllers == NULL || stream == NULL) return;

    fprintf(stream, "=== Override Selector Status ===\n");
    fprintf(stream, "Mode:              %s\n", override_mode_name(state->mode));
    fprintf(stream, "Selector:          %s\n", selector_type_name(state->selector_type));
    fprintf(stream, "Active Controller: %d\n", state->active_controller);
    fprintf(stream, "Output:            %.3f\n", state->selector_output);
    fprintf(stream, "Iterations:        %d\n", state->iteration_count);
    fprintf(stream, "\n--- Controllers ---\n");
    for (int i = 0; i < n_controllers; i++) {
        fprintf(stream, "  [%d] %s: SP=%.2f PV=%.2f OUT=%.2f %s %s\n",
                i,
                controllers[i].tag ? controllers[i].tag : "?",
                controllers[i].setpoint,
                controllers[i].pv,
                controllers[i].output,
                controllers[i].active ? "[ACTIVE]" : "",
                controllers[i].faulted ? "[FAULT]" : "");
    }
}

void override_constraint_report(const override_state_t *state, FILE *stream) {
    if (state == NULL || stream == NULL) return;

    fprintf(stream, "=== Constraint Report ===\n");
    if (state->constraints == NULL || state->num_constraints <= 0) {
        fprintf(stream, "  No constraints configured.\n");
        return;
    }

    for (int i = 0; i < state->num_constraints; i++) {
        const constraint_state_t *cs = &state->constraints[i];
        if (cs->def == NULL) continue;

        fprintf(stream, "  [%d] %-15s: val=%8.3f af=%5.2f roc=%8.3f %s %s %s\n",
                i,
                cs->def->tag ? cs->def->tag : "?",
                cs->current_value,
                cs->approach_factor,
                cs->rate_of_change,
                cs->violating ? "[VIOLATED]" : "",
                cs->approaching ? "[APPROACHING]" : "",
                cs->faulted ? "[FAULT]" : "");
    }
}

double override_mtbo(const override_event_log_t *log, double total_time_s) {
    if (log == NULL || total_time_s <= 0.0) return INFINITY;

    int n = override_event_count(log);
    if (n == 0) return INFINITY;

    return total_time_s / (double)n;
}

/* =========================================================================
 * L7 — ISA-18.2 Alarm Integration
 * ========================================================================= */

int override_alarm_priority(const override_state_t *state) {
    if (state == NULL) return 0;

    switch (state->mode) {
        case OVERRIDE_MODE_FAULT:
            return 4;  /* Critical */
        case OVERRIDE_MODE_OVERRIDE:
            /* Determine based on constraint priority */
            if (state->constraints && state->num_constraints > 0) {
                for (int i = 0; i < state->num_constraints; i++) {
                    constraint_state_t *cs = &state->constraints[i];
                    if (cs->violating) {
                        if (cs->def && cs->def->priority <= PRIORITY_SAFETY) return 4;
                        if (cs->def && cs->def->priority == PRIORITY_CONSTRAINT) return 3;
                    }
                }
            }
            return 2;  /* Medium by default */
        case OVERRIDE_MODE_PRIMARY:
            return 0;  /* Normal — no alarm */
        default:
            return 0;
    }
}

int override_should_alarm(const override_state_t *state, double suppress_s) {
    if (state == NULL) return 0;

    /* Per ISA-18.2: alarms should only be raised for events
       requiring operator action. Transient overrides that resolve
       quickly should be suppressed. */

    int priority = override_alarm_priority(state);

    if (priority >= 4) {
        /* Critical alarm — always raise regardless of duration */
        return 1;
    }

    if (priority >= 3) {
        /* High priority — alarm unless suppressed */
        /* In full implementation, check if the condition has
           persisted longer than suppress_s seconds.
           For now, always alarm for priority >= 3. */
        (void)suppress_s;
        return 1;
    }

    if (priority >= 2) {
        /* Medium — alarm only if persistent */
        /* Would check time_in_mode > suppress_s */
        return 1;
    }

    /* Low or no alarm */
    return 0;
}
