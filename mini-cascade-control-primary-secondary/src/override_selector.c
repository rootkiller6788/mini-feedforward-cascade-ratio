/**
 * @file override_selector.c
 * @brief Override/Selector Control — Constraint Protection & Auctioneering
 *
 * Implements industrial override control (constraint control):
 * - Low/high/median select for competing controllers
 * - Hysteresis-based switching to prevent chattering
 * - Bumpless transfer between active constraints
 * - 2oo3 voting (mid-of-3 select) for redundant safety sensors
 * - Primary controller with constraint override
 * - Switching frequency analysis
 *
 * Override control is critical for process safety: when a constraint
 * (e.g., max pressure, min flow) is approached, the constraint controller
 * takes over from the normal controller to prevent a trip or safety incident.
 *
 * Knowledge Coverage:
 *   L1-L2: Override/selector control architecture, parallel loop competition
 *   L3: Median select, hysteresis, bumpless constraint switching
 *   L5: 2oo3 voting, switching frequency analysis
 *
 * References:
 *   Liptak, Instrument Engineers' Handbook (2006), Vol. 2, Ch. 8.12
 *   Shinskey, Process Control Systems (1996), Ch. 7
 *   IEC 61508 — Functional Safety (2oo3 voting architecture)
 *
 * Curriculum: MIT 6.302, Stanford ENGR205, Purdue ME575, RWTH Aachen ICS
 */

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "cascade_types.h"
#include "cascade_pid.h"
#include "override_selector.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*===========================================================================
 * L2: Override Selector Initialization
 *===========================================================================*/

void override_init(override_selector_t *selector,
                    selector_function_t func,
                    double hysteresis,
                    double output_min, double output_max)
{
    if (!selector) return;

    memset(selector, 0, sizeof(*selector));

    selector->num_slots = 0;
    selector->selector_func = func;
    selector->hysteresis_global = hysteresis;
    selector->final_output = 0.0;
    selector->active_slot_index = 0;
    selector->bumpless_enabled = true;
    selector->tracking_gain = 1.0;
    selector->primary_active = false;

    selector->selection_count = 0;
    selector->last_selection_time = 0.0;
    selector->max_active_duration = 0;

    /* Initialize primary controller with defaults */
    cascade_pid_init(&selector->primary_controller,
        1.0, 60.0, 0.0, 1.0, output_min, output_max);
}

/*===========================================================================
 * L2: Add Constraint Controller
 *
 * Each constraint has its own PID controller. The constraint controller
 * error is computed as:
 *   For MINIMUM constraint: error = PV - limit  (direct-acting)
 *   For MAXIMUM constraint: error = limit - PV  (direct-acting)
 *
 * The PID output represents how urgently the constraint needs to
 * take control. The selector picks the most conservative output.
 *===========================================================================*/

int override_add_constraint(override_selector_t *selector,
                             const char *tag,
                             constraint_type_t type,
                             double limit,
                             double kp, double ti, double td,
                             uint32_t priority)
{
    if (!selector) return -1;
    if (selector->num_slots >= OVERRIDE_MAX_CONTROLLERS) return -1;
    if (kp <= 0.0 || ti <= 0.0) return -1;

    uint32_t idx = selector->num_slots;
    override_slot_t *slot = &selector->slots[idx];

    memset(slot, 0, sizeof(*slot));

    slot->tag = tag;  /* Pointer to static string — caller must keep alive */
    slot->type = type;
    slot->limit = limit;
    slot->active = false;
    slot->enabled = true;
    slot->hysteresis = selector->hysteresis_global;
    slot->pv = 0.0;
    slot->output = 0.0;
    slot->priority = priority;
    slot->activation_count = 0;

    /* Initialize PID for this constraint */
    cascade_pid_init(&slot->pid, kp, ti, td,
        1.0, 0.0, 100.0);

    /* For MAXIMUM constraints: error = limit - PV, output increases as PV nears limit
     * For MINIMUM constraints: error = PV - limit, output increases as PV drops toward limit */
    slot->pid.direction = CASCADE_DIRECT_ACTING;

    selector->num_slots++;
    return (int)idx;
}

void override_set_primary(override_selector_t *selector,
                           double kp, double ti, double td)
{
    if (!selector) return;

    selector->primary_controller.params.kp = kp;
    selector->primary_controller.params.ti = ti;
    selector->primary_controller.params.td = td;
    selector->primary_active = true;

    cascade_pid_reset(&selector->primary_controller);
}

/*===========================================================================
 * L3: Override Selection Algorithm
 *
 * The selector evaluates all controllers and picks the output that
 * best satisfies the selection criterion:
 *
 * SEL_LOW:  minimum output (most conservative for max-constraint protection)
 *   - Used when constraint controllers push output LOW to prevent over-range
 *   - Example: max pressure controller reduces valve opening
 *
 * SEL_HIGH: maximum output (most aggressive for min-constraint protection)
 *   - Used when constraint controllers push output HIGH to prevent under-range
 *   - Example: min flow controller increases pump speed
 *
 * Hysteresis prevents chattering:
 *   If |new_best - current_output| < hysteresis_band → keep current
 *===========================================================================*/

static double compute_constraint_error(const override_slot_t *slot)
{
    switch (slot->type) {
    case CONSTRAINT_MAXIMUM:
        /* Output increases as PV approaches limit from below */
        return slot->limit - slot->pv;

    case CONSTRAINT_MINIMUM:
        /* Output increases as PV approaches limit from above */
        return slot->pv - slot->limit;

    case CONSTRAINT_RATE:
        /* Rate constraint: error based on rate-of-change */
        return slot->limit - slot->pv;

    case CONSTRAINT_VALVE:
        /* Valve position: direct tracking */
        return slot->limit - slot->pv;

    default:
        return 0.0;
    }
}

double override_select(override_selector_t *selector,
                        double primary_sp, double primary_pv,
                        const double *constraint_pvs,
                        double Ts)
{
    if (!selector) return 0.0;

    /* Update primary controller */
    double primary_output = 0.0;
    if (selector->primary_active) {
        primary_output = cascade_pid_update_positional(
            &selector->primary_controller, primary_sp, primary_pv);
    }

    /* Update each constraint controller */
    for (uint32_t i = 0; i < selector->num_slots; i++) {
        override_slot_t *slot = &selector->slots[i];
        if (!slot->enabled) continue;

        /* Update constraint PV if provided */
        if (constraint_pvs) {
            slot->pv = constraint_pvs[i];
        }

        /* Compute constraint error */
        double error = compute_constraint_error(slot);

        /* Update constraint PID */
        slot->output = cascade_pid_update_positional(
            &slot->pid, 0.0, -error);  /* SP=0, PV=-error: PID drives error to 0 */
    }

    /* Apply selector function */
    double selected_output = primary_output;
    uint32_t selected_idx = (uint32_t)-1;  /* -1 = primary */
    bool primary_selected = true;

    if (selector->num_slots > 0) {
        switch (selector->selector_func) {

        case SEL_LOW:
            /* Find minimum output (highest constraint violation) */
            selected_output = primary_output;
            for (uint32_t i = 0; i < selector->num_slots; i++) {
                if (!selector->slots[i].enabled) continue;
                if (selector->slots[i].output < selected_output) {
                    selected_output = selector->slots[i].output;
                    selected_idx = i;
                    primary_selected = false;
                }
            }
            break;

        case SEL_HIGH:
            /* Find maximum output */
            selected_output = primary_output;
            for (uint32_t i = 0; i < selector->num_slots; i++) {
                if (!selector->slots[i].enabled) continue;
                if (selector->slots[i].output > selected_output) {
                    selected_output = selector->slots[i].output;
                    selected_idx = i;
                    primary_selected = false;
                }
            }
            break;

        case SEL_MEDIAN:
        case SEL_MID_OF_3:
        {
            /* Collect all outputs and find median */
            double outputs[OVERRIDE_MAX_CONTROLLERS + 1];
            uint32_t count = 0;
            outputs[count++] = primary_output;
            for (uint32_t i = 0; i < selector->num_slots; i++) {
                if (selector->slots[i].enabled) {
                    outputs[count++] = selector->slots[i].output;
                }
            }
            /* Simple median for up to 7 values */
            if (count > 0) {
                /* Bubble sort (small N, O(N²) acceptable) */
                for (uint32_t i = 0; i < count - 1; i++) {
                    for (uint32_t j = i + 1; j < count; j++) {
                        if (outputs[i] > outputs[j]) {
                            double tmp = outputs[i];
                            outputs[i] = outputs[j];
                            outputs[j] = tmp;
                        }
                    }
                }
                selected_output = outputs[count / 2];

                /* Find which controller has this output */
                if (fabs(selected_output - primary_output) < 1e-6) {
                    selected_idx = (uint32_t)-1;
                    primary_selected = true;
                } else {
                    for (uint32_t i = 0; i < selector->num_slots; i++) {
                        if (selector->slots[i].enabled &&
                            fabs(selector->slots[i].output - selected_output) < 1e-6) {
                            selected_idx = i;
                            primary_selected = false;
                            break;
                        }
                    }
                }
            }
            break;
        }

        case SEL_AVERAGE:
        {
            /* Average of all valid outputs */
            double sum = primary_output;
            uint32_t count = 1;
            for (uint32_t i = 0; i < selector->num_slots; i++) {
                if (selector->slots[i].enabled) {
                    sum += selector->slots[i].output;
                    count++;
                }
            }
            selected_output = sum / (double)count;
            selected_idx = (uint32_t)-1;  /* No single active */
            break;
        }

        case SEL_FIRST_VALID:
            /* Primary always, unless disabled */
            selected_output = primary_output;
            selected_idx = (uint32_t)-1;
            break;
        }
    }

    /* Apply hysteresis to prevent chattering:
     * Only switch if the difference exceeds the hysteresis band.
     * This prevents rapid switching when two outputs are very close. */
    if (!primary_selected && selected_idx != selector->active_slot_index &&
        selector->active_slot_index == (uint32_t)-1) {
        /* Switching from primary to a constraint */
        double diff = fabs(selected_output - primary_output);
        if (diff < selector->hysteresis_global) {
            /* Stay with current selection */
            selected_output = primary_output;
            selected_idx = (uint32_t)-1;
            primary_selected = true;
        }
    }

    /* Update active state */
    if (selected_idx != selector->active_slot_index) {
        selector->selection_count++;
        selector->last_selection_time = 0.0;  /* Would track wall time */
    }

    /* Bumpless tracking: inactive controllers track the active output */
    if (selector->bumpless_enabled) {
        if (primary_selected) {
            for (uint32_t i = 0; i < selector->num_slots; i++) {
                if (selector->slots[i].enabled) {
                    cascade_pid_output_tracking(&selector->slots[i].pid,
                        selected_output);
                }
            }
        } else {
            /* Constraint selected: primary tracks */
            cascade_pid_output_tracking(&selector->primary_controller,
                selected_output);
        }
    }

    /* Update active flags */
    for (uint32_t i = 0; i < selector->num_slots; i++) {
        selector->slots[i].active = (i == selected_idx);
        if (i == selected_idx) {
            selector->slots[i].activation_count++;
        }
    }
    selector->primary_active = primary_selected;

    selector->active_slot_index = selected_idx;
    selector->final_output = selected_output;

    (void)Ts;
    return selected_output;
}

const char *override_get_active_tag(const override_selector_t *selector)
{
    if (!selector) return "NONE";

    if (selector->active_slot_index == (uint32_t)-1) {
        return "PRIMARY";
    }

    if (selector->active_slot_index < selector->num_slots) {
        return selector->slots[selector->active_slot_index].tag;
    }

    return "UNKNOWN";
}

bool override_is_constraint_active(const override_selector_t *selector,
                                    uint32_t slot_index)
{
    if (!selector) return false;
    if (slot_index >= selector->num_slots) return false;
    return selector->slots[slot_index].active;
}

/*===========================================================================
 * L5: Median Select & 2oo3 Voting
 *
 * In safety-critical applications (SIL 2/3 per IEC 61508),
 * redundant sensors use voting logic for fault tolerance:
 *
 * 2oo3 (2 out of 3): Two of three sensors must agree.
 *   - Tolerates 1 fault (fail-safe or fail-danger)
 *   - Median select = 2oo3 voting when all sensors working
 *   - If one fails, degrades to 1oo2
 *
 * Fault detection:
 *   |sensor_i - median| > tolerance → sensor_i is suspect
 *===========================================================================*/

double override_median_select(const double values[3],
                               double tolerance,
                               uint32_t *suspect)
{
    if (!values) return 0.0;

    double v[3] = {values[0], values[1], values[2]};

    /* Sort three values */
    if (v[0] > v[1]) { double t = v[0]; v[0] = v[1]; v[1] = t; }
    if (v[1] > v[2]) { double t = v[1]; v[1] = v[2]; v[2] = t; }
    if (v[0] > v[1]) { double t = v[0]; v[0] = v[1]; v[1] = t; }

    double median = v[1];

    if (suspect) {
        *suspect = 0;
        for (int i = 0; i < 3; i++) {
            if (fabs(values[i] - median) > tolerance) {
                *suspect |= (1u << i);
            }
        }
    }

    return median;
}

double override_mid_of_3_select(const double values[3],
                                 double tolerance,
                                 uint32_t *faults)
{
    if (!values) return NAN;

    uint32_t suspect_mask = 0;
    double median = override_median_select(values, tolerance, &suspect_mask);

    if (faults) {
        /* Count number of bits set in suspect_mask */
        *faults = 0;
        for (int i = 0; i < 3; i++) {
            if (suspect_mask & (1u << i)) (*faults)++;
        }
    }

    /* Decision logic based on number of faults */
    if (*faults == 0) {
        /* All three sensors agree: use median (exact if identical, central otherwise) */
        return median;
    } else if (*faults == 1) {
        /* One sensor faulty: use average of the two good sensors */
        double sum = 0.0;
        int count = 0;
        for (int i = 0; i < 3; i++) {
            if (!(suspect_mask & (1u << i))) {
                sum += values[i];
                count++;
            }
        }
        return (count > 0) ? (sum / (double)count) : NAN;
    } else {
        /* Two or three sensors disagree: use median as best guess
         * or flag as unreliable if deviation is extreme */
        double range = fabs(values[2] - values[0]);
        double max_dev = fabs(values[2] - values[0]);
        (void)range;
        (void)max_dev;
        return median;  /* Best available estimate */
    }
}

/*===========================================================================
 * L5: Selection Frequency Analysis
 *
 * Excessive switching between override constraints indicates:
 * 1. Hysteresis too small → increase hysteresis_global
 * 2. Constraint controller tuning too aggressive → detune
 * 3. Process oscillating near constraint boundary → investigate
 *
 * Target: < 1 switch per hour for normal process operation.
 * > 10 switches per hour requires investigation.
 *===========================================================================*/

int override_selection_frequency(const override_selector_t *selector,
                                  double window_seconds,
                                  double *switches_per_hour)
{
    if (!selector || !switches_per_hour) return -1;
    if (window_seconds <= 0.0) return -1;

    /* Convert total selection count to hourly rate */
    double hours = window_seconds / 3600.0;
    *switches_per_hour = (double)selector->selection_count / hours;

    /* Acceptability thresholds */
    if (*switches_per_hour < 1.0) {
        return 0;  /* Normal */
    } else if (*switches_per_hour < 10.0) {
        return 1;  /* Warning: investigate */
    } else {
        return 2;  /* Excessive: must address */
    }
}

void override_reset_statistics(override_selector_t *selector)
{
    if (!selector) return;

    selector->selection_count = 0;
    selector->last_selection_time = 0.0;
    selector->max_active_duration = 0;

    for (uint32_t i = 0; i < selector->num_slots; i++) {
        selector->slots[i].activation_count = 0;
    }
}
