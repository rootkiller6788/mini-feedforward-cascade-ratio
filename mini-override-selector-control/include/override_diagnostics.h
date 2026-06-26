/**
 * @file override_diagnostics.h
 * @brief Override/Selector Control — Diagnostics and Monitoring
 *
 * Provides runtime diagnostics for override selector systems,
 * including event logging, performance monitoring, health checks,
 * and fault detection. Essential for industrial applications
 * where override events must be audited and analyzed.
 *
 * Reference:
 *   ISA-18.2 (2016). "Management of Alarm Systems for the
 *   Process Industries."
 *
 *   IEC 62682 (2014). "Management of Alarms Systems for the
 *   Process Industries."
 *
 * Knowledge Coverage:
 *   L2 — Core Concepts: Event logging, health monitoring
 *   L3 — Engineering Structures: Diagnostic counters, event ring buffer
 *   L7 — Industrial Applications: Alarm management, audit trail
 *
 * Course Alignment:
 *   RWTH Aachen: Industrial diagnostics and alarm management
 *   Tsinghua: 工业过程监控与诊断
 */

#ifndef OVERRIDE_DIAGNOSTICS_H
#define OVERRIDE_DIAGNOSTICS_H

#include "override_core.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L2 — Diagnostic Initialization and Reset
 * ========================================================================= */

/**
 * Initialize diagnostic counter structure
 *
 * @param diag Pointer to diagnostic structure
 */
void override_diag_init(override_diag_t *diag);

/**
 * Reset all diagnostic counters to zero
 *
 * @param diag Pointer to diagnostic structure
 */
void override_diag_reset(override_diag_t *diag);

/* =========================================================================
 * L2 — Event Logging
 * ========================================================================= */

/** Maximum number of events in circular buffer */
#define OVERRIDE_EVENT_BUFFER_SIZE 128

/**
 * Override event logger structure
 *
 * Maintains a circular buffer of recent override events
 * for post-mortem analysis and audit trail.
 */
typedef struct {
    override_event_t events[OVERRIDE_EVENT_BUFFER_SIZE];
    int head;               /**< Insertion index */
    int count;              /**< Total events logged (may exceed buffer) */
    int wrapped;            /**< 1 = buffer has wrapped */
    int event_id_counter;   /**< Monotonically increasing event ID */
} override_event_log_t;

/**
 * Initialize event log
 *
 * @param log Pointer to event log structure
 */
void override_event_log_init(override_event_log_t *log);

/**
 * Log a new override event
 *
 * @param log         Event log
 * @param timestamp   Event time [s]
 * @param old_mode    Previous mode
 * @param new_mode    New mode
 * @param old_ctrl    Previous active controller
 * @param new_ctrl    New active controller
 * @param output      Selected output value
 * @param reason      Human-readable reason string
 */
void override_event_log_add(override_event_log_t *log,
                            double timestamp,
                            override_mode_t old_mode,
                            override_mode_t new_mode,
                            int old_ctrl, int new_ctrl,
                            double output,
                            const char *reason);

/**
 * Get the Nth most recent event (0 = most recent)
 *
 * @param log Event log
 * @param n   Event index (0 = newest)
 * @return Pointer to event, or NULL if n >= count
 */
const override_event_t* override_event_get(const override_event_log_t *log,
                                           int n);

/**
 * Get total number of logged events
 *
 * @param log Event log
 * @return Total event count
 */
int override_event_count(const override_event_log_t *log);

/**
 * Print all events in the log to a file stream
 *
 * @param log    Event log
 * @param stream Output stream (e.g., stdout, file)
 */
void override_event_log_print(const override_event_log_t *log, FILE *stream);

/* =========================================================================
 * L3 — Health Monitoring Functions
 * ========================================================================= */

/**
 * Perform a health check on the override system
 *
 * Checks:
 * 1. All controllers respond (not faulted)
 * 2. At least one controller is enabled
 * 3. Selector output is within valid range
 * 4. No constraint is permanently violated
 *
 * @param state        Override state
 * @param controllers  Array of controllers
 * @param n_controllers Number of controllers
 * @return 0 if healthy, -1 if degraded, -2 if critical
 */
int override_health_check(const override_state_t *state,
                          const override_controller_t *controllers,
                          int n_controllers);

/**
 * Check if any controller is faulted
 *
 * @param controllers Array of controllers
 * @param n           Number of controllers
 * @return Number of faulted controllers
 */
int override_count_faults(const override_controller_t *controllers, int n);

/**
 * Check for rapid switching (chatter) in selector output
 *
 * Chatter detection: count how many times the active controller
 * changed in the last N seconds. If the count exceeds a threshold,
 * the selector may be chattering due to noise or poor tuning.
 *
 * @param state     Override state
 * @param window_s  Time window for detection [s]
 * @param threshold Maximum allowed switches in window
 * @return 1 if chattering, 0 otherwise
 */
int override_detect_chatter(const override_state_t *state,
                            double window_s, int threshold);

/**
 * Compute the fraction of time each controller was active
 *
 * @param state        Override state
 * @param controllers  Array of controllers
 * @param n_controllers Number of controllers
 * @param fractions     Output array: fraction[0..1] per controller
 */
void override_controller_usage(const override_state_t *state,
                               const override_controller_t *controllers,
                               int n_controllers,
                               double *fractions);

/* =========================================================================
 * L3 — Diagnostic Reporting
 * ========================================================================= */

/**
 * Generate a diagnostic summary string
 *
 * @param state  Override state
 * @param diag   Diagnostic counters
 * @param buffer Output buffer
 * @param bufsz  Buffer size
 * @return Number of characters written
 */
int override_diag_summary(const override_state_t *state,
                          const override_diag_t *diag,
                          char *buffer, int bufsz);

/**
 * Print override system status to file stream
 *
 * @param state        Override state
 * @param controllers  Array of controllers
 * @param n_controllers Number of controllers
 * @param stream       Output stream
 */
void override_status_print(const override_state_t *state,
                           const override_controller_t *controllers,
                           int n_controllers,
                           FILE *stream);

/**
 * Generate a constraint violation report
 *
 * Lists all constraints with their current status and
 * approach factors.
 *
 * @param state       Override state
 * @param stream      Output stream
 */
void override_constraint_report(const override_state_t *state, FILE *stream);

/**
 * Estimate mean time between override events (MTBO)
 *
 * MTBO is a reliability metric: total operating time divided by
 * the number of override events. Higher MTBO indicates a more
 * stable process (fewer constraint violations).
 *
 * @param log          Event log
 * @param total_time_s Total operating time [s]
 * @return MTBO [s], or INFINITY if no events
 */
double override_mtbo(const override_event_log_t *log, double total_time_s);

/* =========================================================================
 * L7 — Alarm Integration (ISA-18.2 / IEC 62682)
 * ========================================================================= */

/**
 * Alarm severity assessment based on override condition
 *
 * Maps override conditions to ISA-18.2 alarm priorities:
 *   Critical: Emergency/Safety priority override active
 *   High:     Constraint priority override active
 *   Medium:   Multiple constraints approaching
 *   Low:      Single constraint approaching
 *
 * @param state Override state
 * @return Alarm priority: 0=no alarm, 1=Low, 2=Medium, 3=High, 4=Critical
 */
int override_alarm_priority(const override_state_t *state);

/**
 * Check if override condition should generate an alarm
 *
 * Per ISA-18.2, not every override event should generate an alarm.
 * Alarms should be reserved for events requiring operator action.
 * Transient overrides that self-correct within a configured time
 * do not need to alarm.
 *
 * @param state       Override state
 * @param suppress_s  Suppression time [s] (transients shorter than this are suppressed)
 * @return 1 if alarm should be raised, 0 otherwise
 */
int override_should_alarm(const override_state_t *state, double suppress_s);

#ifdef __cplusplus
}
#endif

#endif /* OVERRIDE_DIAGNOSTICS_H */
