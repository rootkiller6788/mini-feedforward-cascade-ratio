/**
 * @file split_range_control.c
 * @brief Unified split-range control interface implementation
 *
 * Module: mini-split-range-control-heat-cool
 * Knowledge Coverage: L2 Core Concepts, L6 Canonical Problems
 *
 * Top-level control functions for reactor temperature control,
 * pH neutralization, and pressure control using split-range schemes.
 */

#include "split_range_control.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/* =========================================================================
 * split_control_create_reactor — L6
 *
 * Factory function for reactor temperature split-range control.
 *
 * Design rationale (Myke King, 2016, Ch. 9):
 *   - Heating valve: equal-percentage character to compensate for
 *     heat exchanger nonlinearity (UA varies with flow)
 *   - Cooling valve: linear character because cooling water flow
 *     is roughly linear with heat removal
 *   - Conservative PID: Kc=2.0, Ti=120s, Td=30s
 *     (based on typical 200L pilot reactor time constant ~60-120s)
 *   - Deadband of 2% to prevent simultaneous heating/cooling
 *   - Split point at 50% for symmetric heating/cooling capability
 *
 * Reference: Myke King (2016) Process Control: A Practical Approach
 *   Section 9.3 — "Reactor Temperature Control"
 * ========================================================================= */
split_range_controller_t split_control_create_reactor(void) {
    split_range_controller_t ctrl;
    memset(&ctrl, 0, sizeof(ctrl));

    /* Initialize split scheme for heat/cool */
    split_init_heat_cool_scheme(&ctrl.scheme);

    /* Conservative PID for reactor temperature */
    ctrl.pid_params.kc = 2.0;
    ctrl.pid_params.ti = 120.0;
    ctrl.pid_params.td = 30.0;
    ctrl.pid_params.tf = 0.5;
    ctrl.pid_params.derivative_filter_N = 8.0;
    ctrl.pid_params.beta = 1.0;
    ctrl.pid_params.gamma = 0.0;
    ctrl.pid_params.sample_time_sec = 1.0;
    ctrl.pid_params.bumpless_gain = 1.0;

    /* Initialize PID state */
    split_pid_reset_state(&ctrl.pid_state);

    /* PV context: temperature in degC, typical range -10 to 200 degC */
    ctrl.pv_context.pv_scale_min = -10.0;
    ctrl.pv_context.pv_scale_max = 200.0;
    snprintf(ctrl.pv_context.pv_tag, 32, "TIC-REACTOR");
    snprintf(ctrl.pv_context.pv_units, 16, "degC");

    ctrl.enabled = false;
    ctrl.cascade_mode = false;
    ctrl.remote_sp_active = false;
    ctrl.overall_health = SPLIT_HEALTH_OK;
    ctrl.controller_id = 100;

    return ctrl;
}

/* =========================================================================
 * split_control_create_ph — L6
 *
 * Factory function for pH neutralization split-range control.
 *
 * pH control is challenging because:
 *   1. The titration curve is extremely steep (high process gain near pH 7)
 *   2. Small reagent additions cause large pH changes near neutral
 *   3. The process gain varies by orders of magnitude
 *
 * Design choices:
 *   - Overlap (not deadband) to smooth the acid/base transition
 *   - Equal-percentage valves for fine control at low flow
 *   - High gain with moderate integral to respond to steep curve
 *   - Cubic spline transition for smoother valve movement
 *
 * Reference: McMillan (2005) "pH Measurement and Control", 3rd Ed.
 *   Seborg et al. (2016) Ch. 16 — pH Control
 * ========================================================================= */
split_range_controller_t split_control_create_ph(void) {
    split_range_controller_t ctrl;
    memset(&ctrl, 0, sizeof(ctrl));

    /* Initialize pH split scheme with overlap */
    split_init_ph_scheme(&ctrl.scheme);

    /* Aggressive PID for pH (high process gain near setpoint) */
    ctrl.pid_params.kc = 5.0;
    ctrl.pid_params.ti = 300.0;
    ctrl.pid_params.td = 60.0;
    ctrl.pid_params.tf = 1.0;
    ctrl.pid_params.derivative_filter_N = 10.0;
    ctrl.pid_params.beta = 0.7;   /* reduced SP weight to prevent overshoot */
    ctrl.pid_params.gamma = 0.0;
    ctrl.pid_params.sample_time_sec = 0.5;
    ctrl.pid_params.bumpless_gain = 1.0;

    split_pid_reset_state(&ctrl.pid_state);

    ctrl.pv_context.pv_scale_min = 0.0;
    ctrl.pv_context.pv_scale_max = 14.0;
    snprintf(ctrl.pv_context.pv_tag, 32, "AIC-PH");
    snprintf(ctrl.pv_context.pv_units, 16, "pH");

    ctrl.enabled = false;
    ctrl.overall_health = SPLIT_HEALTH_OK;
    ctrl.controller_id = 200;

    return ctrl;
}

/* =========================================================================
 * split_control_create_pressure — L6
 *
 * Factory function for pressure control with vent and inert gas.
 *
 * Common applications:
 *   - Reactor headspace pressure control
 *   - Distillation column pressure control
 *   - Tank blanketing systems
 *
 * Design:
 *   - Vent valve (fail-open for safety): opens with increasing CO
 *     to release pressure (reverse acting on pressure: more CO → less pressure)
 *   - Inert gas valve (fail-closed): opens with increasing CO
 *     to add pressure (direct acting on pressure)
 *
 * Reference: Seborg et al. (2016) Ch. 13 — Pressure and Flow Control
 *   ISA-77.44.01 — Fossil Fuel Plant Steam Temperature Controls
 * ========================================================================= */
split_range_controller_t split_control_create_pressure(void) {
    split_range_controller_t ctrl;
    memset(&ctrl, 0, sizeof(ctrl));

    /* Custom split scheme for pressure control */
    ctrl.scheme.mode = SPLIT_MODE_COMPLEMENTARY;
    ctrl.scheme.split_point = 50.0;
    ctrl.scheme.deadband_width = 3.0;  /* wider deadband for pressure */
    ctrl.scheme.transition_type = SPLIT_TRANSITION_LINEAR;
    ctrl.scheme.bumpless_transfer = true;

    /* Channel 0: Vent valve — CO 0->50%, valve 100->0% (open on low CO/high pressure) */
    ctrl.scheme.channels[0].channel_id = 0;
    ctrl.scheme.channels[0].action = SPLIT_ACTION_DECREASING;
    ctrl.scheme.channels[0].characteristic = SPLIT_VALVE_QUICK_OPENING;
    ctrl.scheme.channels[0].co_range_start = 0.0;
    ctrl.scheme.channels[0].co_range_end = 48.5;
    ctrl.scheme.channels[0].valve_range_start = 100.0;
    ctrl.scheme.channels[0].valve_range_end = 0.0;
    ctrl.scheme.channels[0].slew_rate_limit = 50.0;
    ctrl.scheme.channels[0].health = SPLIT_HEALTH_OK;
    ctrl.scheme.channels[0].Cv_rated = 30.0;
    snprintf(ctrl.scheme.channels[0].tag_name, 32, "PV-VENT");
    snprintf(ctrl.scheme.channels[0].service_description, 64, "Vent Valve");

    /* Channel 1: Inert gas valve — CO 50->100%, valve 0->100% */
    ctrl.scheme.channels[1].channel_id = 1;
    ctrl.scheme.channels[1].action = SPLIT_ACTION_INCREASING;
    ctrl.scheme.channels[1].characteristic = SPLIT_VALVE_LINEAR;
    ctrl.scheme.channels[1].co_range_start = 51.5;
    ctrl.scheme.channels[1].co_range_end = 100.0;
    ctrl.scheme.channels[1].valve_range_start = 0.0;
    ctrl.scheme.channels[1].valve_range_end = 100.0;
    ctrl.scheme.channels[1].slew_rate_limit = 50.0;
    ctrl.scheme.channels[1].health = SPLIT_HEALTH_OK;
    ctrl.scheme.channels[1].Cv_rated = 20.0;
    snprintf(ctrl.scheme.channels[1].tag_name, 32, "PV-INERT");
    snprintf(ctrl.scheme.channels[1].service_description, 64, "Inert Gas Valve");

    ctrl.scheme.num_channels = 2;

    /* Moderate PID for pressure (fast dynamics) */
    ctrl.pid_params.kc = 1.5;
    ctrl.pid_params.ti = 30.0;
    ctrl.pid_params.td = 5.0;
    ctrl.pid_params.tf = 0.2;
    ctrl.pid_params.derivative_filter_N = 8.0;
    ctrl.pid_params.beta = 1.0;
    ctrl.pid_params.gamma = 0.0;
    ctrl.pid_params.sample_time_sec = 0.2;
    ctrl.pid_params.bumpless_gain = 1.0;

    split_pid_reset_state(&ctrl.pid_state);

    ctrl.pv_context.pv_scale_min = 0.0;
    ctrl.pv_context.pv_scale_max = 10.0;
    snprintf(ctrl.pv_context.pv_tag, 32, "PIC-REACTOR");
    snprintf(ctrl.pv_context.pv_units, 16, "barg");

    ctrl.enabled = false;
    ctrl.overall_health = SPLIT_HEALTH_OK;
    ctrl.controller_id = 300;

    return ctrl;
}

/* =========================================================================
 * split_control_init — L2
 *
 * Initialize a controller with custom settings.
 * ========================================================================= */
void split_control_init(split_range_controller_t *ctrl,
                          const split_range_scheme_t *scheme,
                          double kc, double ti, double td, double ts,
                          double pv_min, double pv_max) {
    if (!ctrl) return;

    memset(ctrl, 0, sizeof(*ctrl));

    if (scheme) {
        memcpy(&ctrl->scheme, scheme, sizeof(*scheme));
    } else {
        split_init_heat_cool_scheme(&ctrl->scheme);
    }

    ctrl->pid_params.kc = kc;
    ctrl->pid_params.ti = ti;
    ctrl->pid_params.td = td;
    ctrl->pid_params.tf = 0.5;
    ctrl->pid_params.derivative_filter_N = 8.0;
    ctrl->pid_params.beta = 1.0;
    ctrl->pid_params.gamma = 0.0;
    ctrl->pid_params.sample_time_sec = ts > 0.0 ? ts : 1.0;
    ctrl->pid_params.bumpless_gain = 1.0;

    split_pid_reset_state(&ctrl->pid_state);

    ctrl->pv_context.pv_scale_min = pv_min;
    ctrl->pv_context.pv_scale_max = pv_max;

    ctrl->enabled = false;
    ctrl->overall_health = SPLIT_HEALTH_OK;
    ctrl->controller_id = 0;
}

/* =========================================================================
 * split_control_set_pv — L2
 *
 * Update process variable with filtering.
 *
 * Also computes PV rate of change for feedforward and diagnostic use.
 * ========================================================================= */
void split_control_set_pv(split_range_controller_t *ctrl,
                            double pv, double dt_sec) {
    if (!ctrl) return;

    ctrl->pv_context.previous_pv = ctrl->pv_context.process_variable;
    ctrl->pv_context.process_variable = pv;

    /* Apply PV filter */
    ctrl->pv_context.pv_filtered = split_pid_pv_filter(
        &ctrl->pid_state, pv, 0.7);

    /* Compute rate of change */
    if (dt_sec > 0.0) {
        ctrl->pv_context.pv_rate_of_change =
            (ctrl->pv_context.process_variable
             - ctrl->pv_context.previous_pv) / dt_sec;
    }
}

/* =========================================================================
 * split_control_set_sp — L2
 * ========================================================================= */
void split_control_set_sp(split_range_controller_t *ctrl, double sp) {
    if (!ctrl) return;
    ctrl->pv_context.previous_sp = ctrl->pv_context.setpoint;
    ctrl->pv_context.setpoint = sp;
}

/* =========================================================================
 * split_control_execute — L2
 *
 * Execute one complete control cycle.
 *
 * Steps:
 *   1. If NOT enabled, skip PID computation (manual mode)
 *      — but still apply manual valve positions
 *   2. If enabled, run PID computation
 *   3. Distribute PID output to valve channels
 *   4. Apply slew rate limits and hysteresis
 *   5. Update performance tracking
 *
 * This function is designed to be called at a fixed sample rate
 * (e.g., every 1 second for temperature, every 0.2s for pressure).
 * ========================================================================= */
void split_control_execute(split_range_controller_t *ctrl, double dt_sec) {
    if (!ctrl) return;
    if (dt_sec <= 0.0) dt_sec = 1.0;

    if (ctrl->enabled) {
        /* Active control: compute PID output */
        double sp = ctrl->cascade_mode && ctrl->remote_sp_active
                    ? ctrl->remote_setpoint
                    : ctrl->pv_context.setpoint;

        double pv = ctrl->pv_context.pv_filtered;
        if (pv == 0.0 && ctrl->pid_state.sample_index == 0) {
            pv = ctrl->pv_context.process_variable;
        }

        double pid_out = split_pid_incremental(
            &ctrl->pid_params, &ctrl->pid_state, sp, pv);

        /* Add feedforward */
        if (ctrl->pv_context.feedforward_enabled) {
            pid_out += ctrl->pv_context.feedforward_signal;
            if (pid_out > SPLIT_CO_MAX) pid_out = SPLIT_CO_MAX;
            if (pid_out < SPLIT_CO_MIN) pid_out = SPLIT_CO_MIN;
        }

        ctrl->previous_controller_output = ctrl->controller_output;
        ctrl->controller_output = pid_out;

        /* Distribute to channels */
        split_distribute_output(&ctrl->scheme, pid_out, ctrl->split_outputs);
    } else {
        /* Manual mode: maintain last controller output, apply manual positions */
        for (uint32_t i = 0; i < ctrl->scheme.num_channels && i < SPLIT_MAX_CHANNELS; i++) {
            if (ctrl->scheme.channels[i].manual_mode
                || ctrl->scheme.channels[i].maintenance_override) {
                ctrl->split_outputs[i] = ctrl->scheme.channels[i].manual_position;
            } else {
                /* Hold last valve position in manual mode */
                ctrl->split_outputs[i] = ctrl->scheme.channels[i].current_position;
            }
        }
    }

    /* Per-channel slew limiting */
    for (uint32_t i = 0; i < ctrl->scheme.num_channels && i < SPLIT_MAX_CHANNELS; i++) {
        split_range_channel_t *ch = &ctrl->scheme.channels[i];

        double pos_limited = split_slew_rate_limit(
            ch->current_position, ctrl->split_outputs[i],
            ch->slew_rate_limit, dt_sec);

        double pos_final = split_hysteresis_compensate(ch, pos_limited);

        /* Update valve state */
        ch->target_position = ctrl->split_outputs[i];
        ch->current_position = pos_final;
    }

    /* Update health */
    for (uint32_t i = 0; i < ctrl->scheme.num_channels && i < SPLIT_MAX_CHANNELS; i++) {
        if (ctrl->scheme.channels[i].health > ctrl->overall_health) {
            ctrl->overall_health = ctrl->scheme.channels[i].health;
        }
    }
}

/* =========================================================================
 * split_control_evaluate — L2
 *
 * Computes performance metrics incrementally.
 * In a real implementation, these integrals would be accumulated over
 * time. Here we compute snapshot values based on current state.
 * ========================================================================= */
void split_control_evaluate(const split_range_controller_t *ctrl,
                              split_range_performance_t *perf) {
    if (!ctrl || !perf) return;

    memset(perf, 0, sizeof(*perf));

    double error = ctrl->pv_context.setpoint - ctrl->pv_context.process_variable;
    double abs_error = fabs(error);

    /* Snap-shot metrics (would normally be accumulated) */
    perf->iae = abs_error;
    perf->ise = error * error;
    perf->itae = abs_error;   /* simplified: no time weight in snapshot */
    perf->itse = error * error;

    /* Overshoot */
    if (error < 0.0) {
        perf->overshoot_pct = -error / ctrl->pv_context.setpoint * 100.0;
    }

    perf->steady_state_error = error;

    /* Valve travel totals */
    for (uint32_t i = 0; i < ctrl->scheme.num_channels && i < SPLIT_MAX_CHANNELS; i++) {
        perf->total_valve_travel[i] = ctrl->scheme.channels[i].current_position;
    }

    perf->output_variance = ctrl->controller_output / 100.0;
    perf->pv_variance = ctrl->pv_context.process_variable / 100.0;

    /* Cross-coupling */
    perf->cross_coupling_index = split_cross_coupling_analysis(ctrl);

    /* Energy estimates */
    double heat_pow, cool_pow;
    split_valve_energy_consumption(ctrl->controller_output,
                                    ctrl->scheme.split_point,
                                    ctrl->split_outputs,
                                    (int)ctrl->scheme.num_channels,
                                    &heat_pow, &cool_pow);
    perf->energy_consumption_heating = heat_pow;
    perf->energy_consumption_cooling = cool_pow;

    /* Split efficiency */
    perf->split_efficiency_index = 1.0 - perf->cross_coupling_index;
}

/* =========================================================================
 * split_control_set_feedforward — L2
 * ========================================================================= */
void split_control_set_feedforward(split_range_controller_t *ctrl, double ff) {
    if (!ctrl) return;
    ctrl->pv_context.feedforward_signal = ff;
    ctrl->pv_context.feedforward_enabled = true;
}

/* =========================================================================
 * split_control_health_check — L2
 *
 * Aggregates per-channel health per NAMUR NE107.
 * The overall health is the worst status of any channel.
 * ========================================================================= */
split_range_health_t split_control_health_check(
    const split_range_controller_t *ctrl) {
    if (!ctrl) return SPLIT_HEALTH_FAILURE;

    split_range_health_t worst = SPLIT_HEALTH_OK;

    for (uint32_t i = 0; i < ctrl->scheme.num_channels && i < SPLIT_MAX_CHANNELS; i++) {
        if (ctrl->scheme.channels[i].health > worst) {
            worst = ctrl->scheme.channels[i].health;
        }
    }

    return worst;
}

/* =========================================================================
 * split_control_validate_scheme — L2
 *
 * Validates a split-range scheme for consistency:
 *   1. num_channels must be in [1, SPLIT_MAX_CHANNELS]
 *   2. Channel CO ranges must not leave gaps
 *   3. Split point must be in [SPLIT_POINT_MIN, SPLIT_POINT_MAX]
 *   4. Overlapping ranges only allowed in OVERLAP mode
 *   5. Channel CO ranges must be within [0, 100]
 *
 * Algorithm:
 *   1. Check num_channels validity
 *   2. Sort channels by co_range_start
 *   3. Verify coverage: first starts at 0, last ends at 100
 *   4. Check for gaps between consecutive channels
 *   5. Check for overlaps in non-overlap mode
 *   6. Verify all valve ranges within [0, 100]
 *
 * Returns 0 if valid, negative error code otherwise.
 * ========================================================================= */
int split_control_validate_scheme(const split_range_scheme_t *scheme) {
    if (!scheme) return -1;

    /* Check 1: valid number of channels */
    if (scheme->num_channels < 1
        || scheme->num_channels > SPLIT_MAX_CHANNELS) {
        return -1;
    }

    /* Check 2: split point range */
    if (scheme->split_point < SPLIT_POINT_MIN
        || scheme->split_point > SPLIT_POINT_MAX) {
        return -3;
    }

    /* Sort channels by co_range_start and check coverage */
    /* Simple O(n^2) sort for n <= 6 */
    double starts[6], ends[6];
    for (uint32_t i = 0; i < scheme->num_channels; i++) {
        starts[i] = scheme->channels[i].co_range_start;
        ends[i] = scheme->channels[i].co_range_end;

        /* Validate individual channel ranges */
        if (starts[i] < 0.0 || starts[i] > 100.0
            || ends[i] < 0.0 || ends[i] > 100.0) {
            return -5;
        }
        if (starts[i] >= ends[i]) {
            return -5;
        }
        if (scheme->channels[i].valve_range_start < 0.0
            || scheme->channels[i].valve_range_start > 100.0
            || scheme->channels[i].valve_range_end < 0.0
            || scheme->channels[i].valve_range_end > 100.0) {
            return -5;
        }
    }

    /* Sort (bubble sort sufficient for n <= 6) */
    for (uint32_t i = 0; i < scheme->num_channels - 1; i++) {
        for (uint32_t j = i + 1; j < scheme->num_channels; j++) {
            if (starts[j] < starts[i]) {
                double tmp = starts[i]; starts[i] = starts[j]; starts[j] = tmp;
                tmp = ends[i]; ends[i] = ends[j]; ends[j] = tmp;
            }
        }
    }

    /* Check coverage: first channel must start at 0 */
    /* Allow small gap at start due to deadband (max gap = deadband_width/2) */
    if (starts[0] > scheme->deadband_width / 2.0 + 0.5) {
        return -2; /* gap at beginning */
    }

    /* Check coverage: last channel must end at 100 */
    /* Allow small gap at end due to deadband */
    if (ends[scheme->num_channels - 1] < 100.0 - scheme->deadband_width / 2.0 - 0.5) {
        return -2; /* gap at end */
    }

    /* Check for gaps and overlaps */
    for (uint32_t i = 0; i < scheme->num_channels - 1; i++) {
        double gap = starts[i + 1] - ends[i];
        /* Allow gap up to deadband_width (intentional dead zone) */
        if (gap > scheme->deadband_width + 1.0) {
            return -2; /* excessive gap detected */
        }
        if (gap < -0.01 && scheme->mode != SPLIT_MODE_OVERLAP
            && scheme->overlap_width <= 0.0) {
            return -4; /* overlap without overlap mode */
        }
    }

    return 0;
}
