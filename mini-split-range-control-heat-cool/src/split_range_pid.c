/**
 * @file split_range_pid.c
 * @brief PID controller with split-range output mapping — implementation
 *
 * Module: mini-split-range-control-heat-cool
 * Knowledge Coverage: L3 Engineering Structures, L5 Algorithms
 *
 * Implements both incremental (velocity) and positional PID forms,
 * optimized for split-range control applications.  The velocity form
 * is preferred because it inherently provides bumpless transfer.
 *
 * Theoretical foundation:
 *
 * Continuous PID (parallel form):
 *   u(t) = Kc * [ e(t) + 1/Ti * integral(e(tau), 0..t) + Td * de(t)/dt ]
 *
 * Discretization (backward Euler):
 *   Integral:  sum e(k) * Ts
 *   Derivative: (e(k) - e(k-1)) / Ts
 *
 * Velocity form (differencing):
 *   du(k) = Kc * [ e(k) - e(k-1)
 *                + (Ts/Ti) * e(k)
 *                + (Td/Ts) * (e(k) - 2*e(k-1) + e(k-2)) ]
 *   u(k) = u(k-1) + du(k)
 *
 * 2-DOF Setpoint weighting:
 *   Error: e(k) = beta*SP - PV  (beta in [0,1], default 1)
 *   Derivative: d(k) = gamma*SP - PV (gamma in [0,1], default 0 = deriv on PV only)
 *
 * Anti-windup (conditional integration):
 *   If u(k-1) is saturated AND error has same sign as output:
 *     do not accumulate integral (hold integrator)
 *
 * Anti-windup (back-calculation, for positional form only):
 *   du_aw = (u_saturated - u_unsaturated) / Tt
 *   where Tt = sqrt(Ti * Td) (tracking time constant)
 *
 * Reference:
 *   Astrom & Hagglund (1995) PID Controllers: Theory, Design, and Tuning
 *   Åström & Hägglund (2006) Advanced PID Control
 *   Peng, Vrancic, Hanus (1996) "Anti-windup, bumpless, and conditioned
 *     transfer techniques for PID controllers"
 */

#include "split_range_pid.h"
#include "split_range_core.h"
#include <math.h>
#include <string.h>
#include <float.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =========================================================================
 * Internal: Check for NaN or Infinity
 * ========================================================================= */

static int is_bad_double(double x) {
    return isnan(x) || isinf(x);
}

/* =========================================================================
 * split_pid_init_params — L3
 *
 * Initialize PID parameters with the given values.
 * Validates and sets sensible defaults for invalid inputs.
 *
 * Guard conditions:
 *   kc < 0 → set to 0 (no proportional action)
 *   ti <= 0 → disable integral (set Ti to INF sentinel, treated as disabled)
 *   td <= 0 → disable derivative
 *   ts <= 0 → set to 1.0 (reasonable default for process control)
 * ========================================================================= */
void split_pid_init_params(split_range_pid_params_t *params,
                            double kc, double ti, double td, double ts) {
    if (!params) return;

    memset(params, 0, sizeof(*params));

    params->kc = (kc >= 0.0) ? kc : 0.0;
    params->ti = (ti > 0.0) ? ti : 0.0;      /* 0.0 = integral disabled */
    params->td = (td >= 0.0) ? td : 0.0;
    params->tf = 0.0;                          /* no setpoint filter by default */
    params->derivative_filter_N = 8.0;         /* standard N = 8 for deriv filter */
    params->beta = 1.0;                        /* full SP on proportional */
    params->gamma = 0.0;                       /* deriv on PV only (no deriv kick) */
    params->sample_time_sec = (ts > 0.0) ? ts : 1.0;
    params->bumpless_gain = 1.0;
}

/* =========================================================================
 * split_pid_reset_state — L3
 *
 * Clears all PID state, preparing for a fresh start or mode transition.
 * Crucially resets integral_accum to prevent windup carryover.
 * ========================================================================= */
void split_pid_reset_state(split_range_pid_state_t *state) {
    if (!state) return;

    memset(state, 0, sizeof(*state));
    state->integrator_hold = false;
    state->derivative_kick_handled = true; /* start with kick suppressed */
}

/* =========================================================================
 * split_pid_incremental — L3, L5
 *
 * Velocity-form (incremental) PID algorithm.
 *
 * This is the RECOMMENDED algorithm for split-range control because:
 *   1. Inherent bumpless transfer: mode changes only affect du, not u
 *   2. No explicit integrator reset needed
 *   3. Anti-windup: simply don't add du when output is saturated
 *   4. Natural handling of output limits
 *
 * Algorithm steps:
 *   1. Filter PV (exponential moving average)
 *   2. Compute error with 2-DOF setpoint weighting
 *   3. Compute proportional increment
 *   4. Compute integral increment (with anti-windup hold)
 *   5. Compute derivative increment (with filtering)
 *   6. Sum increments, clamp output
 *
 * Edge cases handled:
 *   - NULL params/state → return 0.0
 *   - NaN/Inf PV or SP → return last valid output
 *   - Ti == 0 → integral disabled
 *   - Td == 0 → derivative disabled
 *   - sample_time_sec == 0 → treated as 1.0
 * ========================================================================= */
double split_pid_incremental(split_range_pid_params_t *params,
                              split_range_pid_state_t *state,
                              double sp, double pv) {
    if (!params || !state) return 0.0;

    /* Validate inputs */
    if (is_bad_double(pv) || is_bad_double(sp)) {
        return state->last_output;
    }

    double Ts = params->sample_time_sec;
    if (Ts <= 0.0) Ts = 1.0;

    /* 2-DOF error computation:
     *   e_p = beta*SP - PV  (proportional error)
     *   e_i = SP - PV        (integral error, always full)
     *   e_d = gamma*SP - PV  (derivative error) */
    double e_p = params->beta * sp - pv;
    double e_i = sp - pv;
    double e_d = params->gamma * sp - pv;

    /* Previous errors for derivative */
    double e_d_prev = params->gamma * state->prev_error;

    /* --- Proportional increment --- */
    double dP = params->kc * (e_p - state->last_error);

    /* --- Integral increment ---
     * du_i = Kc * (Ts/Ti) * e_i(k)
     * Conditional integration: hold integrator when output saturated
     * and error pushes further into saturation. */
    double dI = 0.0;
    if (params->ti > 0.0) {
        if (!state->integrator_hold) {
            dI = params->kc * (Ts / params->ti) * e_i;
        }
    }

    /* --- Derivative increment ---
     * Filtered derivative on PV to avoid derivative kick.
     * Using backward difference approximation:
     *   de/dt ≈ (e_d(k) - e_d(k-1)) / Ts
     *
     * With first-order filter (N = derivative_filter_N):
     *   Td_filtered = Td / (1 + Td/(N*Ts) * (1 - z^-1))
     * Simplified: use filtered derivative difference. */
    double dD = 0.0;
    if (params->td > 0.0 && Ts > 0.0) {
        /* Derivative on PV (gamma=0 means dD is from PV change only) */
        if (state->sample_index > 0) {
            /* 3-point derivative: (e_d(k) - e_d(k-1)) / Ts */
            double deriv_term = (e_d - e_d_prev) / Ts;
            /* First-order filter on derivative */
            double N = params->derivative_filter_N;
            double alpha = Ts / (Ts + params->td / N);
            state->filtered_derivative = (1.0 - alpha) * state->filtered_derivative
                                         + alpha * deriv_term;
            dD = params->kc * params->td * state->filtered_derivative;
        }
    }

    /* --- Sum increments --- */
    double du = dP + dI + dD;

    /* Check for NaN in du */
    if (is_bad_double(du)) {
        du = 0.0;
    }

    /* --- Anti-windup: conditional integration flag ---
     * If output was at limit AND du pushes in same direction, hold integrator */
    double u_new = state->last_output + du;
    if (u_new >= SPLIT_CO_MAX && du > 0.0) {
        du = 0.0; /* prevent further increase */
        state->integrator_hold = true;
        u_new = state->last_output;
    } else if (u_new <= SPLIT_CO_MIN && du < 0.0) {
        du = 0.0; /* prevent further decrease */
        state->integrator_hold = true;
        u_new = state->last_output;
    } else {
        u_new = state->last_output + du;
        state->integrator_hold = false;
    }

    /* Clamp output */
    if (u_new > SPLIT_CO_MAX) u_new = SPLIT_CO_MAX;
    if (u_new < SPLIT_CO_MIN) u_new = SPLIT_CO_MIN;

    /* --- Update state for next iteration --- */
    state->prev_error = state->last_error;
    state->last_error = e_p;
    state->p_term = dP;
    state->i_term = dI;
    state->d_term = dD;
    state->integral_accum += dI; /* accumulate for monitoring */
    state->last_output = u_new;
    state->sample_index++;

    return u_new;
}

/* =========================================================================
 * split_pid_positional — L3
 *
 * Positional (absolute) PID algorithm.
 * Less preferred for split-range but included for legacy DCS compatibility
 * and comparison with the recommended velocity form.
 *
 * The positional form computes absolute output:
 *   u(k) = Kc * [ e(k) + (Ts/Ti) * sum(e, k) + (Td/Ts) * (e(k) - e(k-1)) ]
 *
 * Anti-windup via back-calculation:
 *   When output saturates, the integral term is adjusted by:
 *     I(k) = I(k-1) + Kc*Ts/Ti*e(k) + (u_sat - u_unsat)/Tt * Ts
 *   where Tt = sqrt(Ti*Td) is the tracking time constant.
 *
 * Theorem (Astrom & Hagglund, 1995, Sec. 3.5):
 *   For a first-order process, choosing Tt = sqrt(Ti*Td) guarantees
 *   that the anti-windup feedback loop is at least as fast as the
 *   closed-loop system, preventing performance degradation from windup.
 * ========================================================================= */
double split_pid_positional(split_range_pid_params_t *params,
                             split_range_pid_state_t *state,
                             double sp, double pv) {
    if (!params || !state) return 0.0;
    if (is_bad_double(pv) || is_bad_double(sp)) return state->last_output;

    double Ts = params->sample_time_sec;
    if (Ts <= 0.0) Ts = 1.0;

    /* 2-DOF errors */
    double e_p = params->beta * sp - pv;
    double e_i = sp - pv;   /* integral uses full error */
    double e_d = params->gamma * sp - pv;

    /* Proportional term */
    double P = params->kc * e_p;

    /* Integral term with anti-windup back-calculation */
    double I;
    if (params->ti > 0.0) {
        /* Increment integral */
        double dI = params->kc * (Ts / params->ti) * e_i;

        /* Back-calculation anti-windup tracking time */
        double Tt = (params->ti > 0.0 && params->td > 0.0)
            ? sqrt(params->ti * params->td)
            : params->ti; /* fallback */

        if (Tt < Ts) Tt = Ts; /* tracking must not be faster than sampling */

        /* Add anti-windup correction */
        double u_sat = state->last_output; /* previous saturated output */
        double u_unsat = P + state->integral_accum + dI; /* hypothetical unsaturated */

        /* tracking correction: (u_sat - u_unsat) / Tt * Ts */
        double aw_correction = (u_sat - u_unsat) * Ts / Tt;

        state->integral_accum += dI + aw_correction;
        I = state->integral_accum;
    } else {
        I = 0.0;
    }

    /* Derivative term with filter */
    double D = 0.0;
    if (params->td > 0.0 && Ts > 0.0 && state->sample_index > 0) {
        double raw_deriv = (e_d - state->prev_d_term) / Ts;
        double N = params->derivative_filter_N;
        if (N > 0.0) {
            double alpha = Ts / (Ts + params->td / N);
            state->filtered_derivative = (1.0 - alpha) * state->filtered_derivative
                                         + alpha * raw_deriv;
        } else {
            state->filtered_derivative = raw_deriv;
        }
        D = params->kc * params->td * state->filtered_derivative;
    }

    /* Sum for unsaturated output */
    double u_unsat = P + I + D;

    /* Clamp */
    double u;
    if (u_unsat > SPLIT_CO_MAX) {
        u = SPLIT_CO_MAX;
    } else if (u_unsat < SPLIT_CO_MIN) {
        u = SPLIT_CO_MIN;
    } else {
        u = u_unsat;
    }

    if (is_bad_double(u)) u = state->last_output;

    /* Update state */
    state->prev_error = state->last_error;
    state->last_error = e_p;
    state->prev_d_term = e_d;
    state->p_term = P;
    state->i_term = I;
    state->d_term = D;
    state->last_output = u;
    state->sample_index++;

    return u;
}

/* =========================================================================
 * split_pid_external_reset — L3, L5
 *
 * External reset feedback for anti-windup in cascade configurations.
 * When the split-range controller is a secondary loop in cascade control,
 * the primary controller output provides an external reset signal.
 *
 * The external reset forces the integral term to track:
 *   I_new = I_old + tracking_gain * (external_reset - u_old) * Ts / Tt
 *
 * This ensures that when the primary loop saturates the secondary SP,
 * the secondary integrator does not wind up.
 *
 * Reference: ISA-77.44.01 — Steam Temperature Controls
 *   Shinskey (1996) Process Control Systems, Ch. 7
 * ========================================================================= */
void split_pid_external_reset(split_range_pid_state_t *state,
                               double external_reset, double tracking_gain) {
    if (!state) return;
    if (is_bad_double(external_reset)) return;

    state->external_reset = external_reset;
    state->tracking_error = external_reset - state->last_output;

    /* Adjust integral accumulation to track external reset */
    if (tracking_gain > 0.0) {
        double correction = state->tracking_error * tracking_gain;
        state->integral_accum += correction;

        /* Also update output to track */
        state->last_output += correction;
        if (state->last_output > SPLIT_CO_MAX)
            state->last_output = SPLIT_CO_MAX;
        if (state->last_output < SPLIT_CO_MIN)
            state->last_output = SPLIT_CO_MIN;
    }
}

/* =========================================================================
 * split_pid_setpoint_filter — L3
 *
 * First-order low-pass filter on the setpoint to prevent abrupt
 * setpoint changes from causing large derivative kicks.
 *
 * Filter: Y(k) = Y(k-1) + (Ts/Tf) * (X(k) - Y(k-1))
 *
 * where Tf is the filter time constant. Typical Tf = 0.1 to 5 seconds.
 * ========================================================================= */
double split_pid_setpoint_filter(split_range_pid_state_t *state,
                                  double raw_setpoint,
                                  double filter_tau_sec, double dt_sec) {
    if (!state) return raw_setpoint;
    if (is_bad_double(raw_setpoint)) return state->filtered_pv;

    if (filter_tau_sec <= 0.0 || dt_sec <= 0.0) {
        return raw_setpoint; /* no filtering */
    }

    double alpha = dt_sec / filter_tau_sec;
    if (alpha > 1.0) alpha = 1.0; /* clamp for large dt */

    double filtered = state->filtered_pv
                      + alpha * (raw_setpoint - state->filtered_pv);

    return filtered;
}

/* =========================================================================
 * split_pid_pv_filter — L3
 *
 * Exponential moving average filter on the process variable.
 *   Y(k) = alpha * Y(k-1) + (1-alpha) * X(k)
 *
 * alpha close to 1 → heavy filtering (slow response)
 * alpha = 0 → no filtering (raw signal)
 *
 * Also computes rate-of-change for diagnostic purposes.
 * ========================================================================= */
double split_pid_pv_filter(split_range_pid_state_t *state,
                            double raw_pv, double alpha) {
    if (!state) return raw_pv;
    if (is_bad_double(raw_pv)) return state->filtered_pv;

    if (alpha < 0.0) alpha = 0.0;
    if (alpha >= 1.0) alpha = 0.999;

    state->prev_filtered_pv = state->filtered_pv;

    if (state->sample_index == 0) {
        /* First sample: initialize filter */
        state->filtered_pv = raw_pv;
        state->prev_filtered_pv = raw_pv;
    } else {
        state->filtered_pv = alpha * state->filtered_pv
                             + (1.0 - alpha) * raw_pv;
    }

    return state->filtered_pv;
}

/* =========================================================================
 * split_pid_control_cycle — L3
 *
 * Complete control cycle: read PV, compute PID, distribute to valves.
 * This is the main execution function called at each sample period.
 *
 * Steps:
 *   1. Store previous PV and SP
 *   2. Apply PV filtering
 *   3. Apply feedforward (if enabled)
 *   4. Compute PID output (velocity form)
 *   5. Add feedforward to PID output
 *   6. Distribute via split scheme
 *   7. Apply per-channel slew rate limits
 *   8. Apply per-channel hysteresis
 *   9. Update valve positions
 * ========================================================================= */
split_range_health_t split_pid_control_cycle(split_range_controller_t *ctrl,
                                              double dt_sec) {
    if (!ctrl || !ctrl->enabled) {
        return SPLIT_HEALTH_FAILURE;
    }
    if (dt_sec <= 0.0) dt_sec = 1.0;

    /* Step 1: Store previous values */
    ctrl->previous_controller_output = ctrl->controller_output;
    ctrl->pv_context.previous_pv = ctrl->pv_context.process_variable;
    ctrl->pv_context.previous_sp = ctrl->pv_context.setpoint;

    /* Step 2: Filter PV */
    double pv_filtered = split_pid_pv_filter(
        &ctrl->pid_state,
        ctrl->pv_context.process_variable,
        0.7); /* alpha = 0.7 moderate filtering */

    ctrl->pv_context.pv_filtered = pv_filtered;

    /* Step 3 & 4: Compute PID output */
    double sp = ctrl->cascade_mode && ctrl->remote_sp_active
                ? ctrl->remote_setpoint
                : ctrl->pv_context.setpoint;

    double pid_out = split_pid_incremental(
        &ctrl->pid_params,
        &ctrl->pid_state,
        sp, pv_filtered);

    /* Step 5: Add feedforward */
    if (ctrl->pv_context.feedforward_enabled) {
        pid_out += ctrl->pv_context.feedforward_signal;
        if (pid_out > SPLIT_CO_MAX) pid_out = SPLIT_CO_MAX;
        if (pid_out < SPLIT_CO_MIN) pid_out = SPLIT_CO_MIN;
    }

    ctrl->controller_output = pid_out;

    /* Step 6: Distribute to channel positions */
    split_distribute_output(&ctrl->scheme, pid_out, ctrl->split_outputs);

    /* Steps 7-9: Per-channel slew limiting, hysteresis, and position update */
    for (uint32_t i = 0; i < ctrl->scheme.num_channels && i < SPLIT_MAX_CHANNELS; i++) {
        split_range_channel_t *ch = &ctrl->scheme.channels[i];

        /* Slew rate limit */
        double pos_limited = split_slew_rate_limit(
            ch->current_position, ctrl->split_outputs[i],
            ch->slew_rate_limit, dt_sec);

        /* Hysteresis compensation */
        double pos_final = split_hysteresis_compensate(ch, pos_limited);

        /* Update valve state */
        ch->target_position = ctrl->split_outputs[i];
        ch->current_position = pos_final;
    }

    /* Compute overall health */
    ctrl->overall_health = SPLIT_HEALTH_OK;
    for (uint32_t i = 0; i < ctrl->scheme.num_channels && i < SPLIT_MAX_CHANNELS; i++) {
        if (ctrl->scheme.channels[i].health > ctrl->overall_health) {
            ctrl->overall_health = ctrl->scheme.channels[i].health;
        }
    }

    return ctrl->overall_health;
}

/* =========================================================================
 * split_pid_mode_transition — L3
 *
 * Bumpless transition between MANUAL / AUTO / CASCADE modes.
 *
 * In velocity-form PID, mode transitions are naturally bumpless because
 * the output accumulator u(k-1) retains the last value regardless of
 * how it was set.  However, we reset the integral accumulator to the
 * current output to avoid a transient when returning to AUTO.
 *
 * For MANUAL → AUTO transition:
 *   integral_accum = current_output (so P+I+D starts from there)
 *   last_error = beta*SP - PV (initialize error tracking)
 * ========================================================================= */
void split_pid_mode_transition(split_range_controller_t *ctrl,
                                bool enable, bool cascade) {
    if (!ctrl) return;

    if (enable && !ctrl->enabled) {
        /* MANUAL → AUTO: re-initialize state for bumpless */
        ctrl->pid_state.integral_accum = ctrl->controller_output;
        ctrl->pid_state.last_output = ctrl->controller_output;
        ctrl->pid_state.last_error = ctrl->pid_params.beta
                                     * ctrl->pv_context.setpoint
                                     - ctrl->pv_context.process_variable;
        ctrl->pid_state.prev_error = ctrl->pid_state.last_error;
        ctrl->pid_state.integrator_hold = false;
    }

    ctrl->enabled = enable;
    ctrl->cascade_mode = cascade;

    if (cascade) {
        ctrl->pid_state.integrator_hold = false; /* release hold for cascade */
    }
}

/* =========================================================================
 * split_pid_zn_tuning — L5
 *
 * Ziegler-Nichols open-loop step response method (Process Reaction Curve).
 *
 * From a step test on the process:
 *   - Process gain K = delta_PV / delta_CO
 *   - Time constant tau (time to 63.2% of final value)
 *   - Apparent dead time theta (intercept of max-slope tangent)
 *
 * Tuning rules (ZN):
 *   P:   Kc = 1.0 / (K * theta/tau)         (no Ti, Td)
 *   PI:  Kc = 0.9 / (K * theta/tau), Ti = 3.33*theta
 *   PID: Kc = 1.2 / (K * theta/tau), Ti = 2.0*theta, Td = 0.5*theta
 *
 * Tyreus-Luyben (more conservative, better for process control):
 *   PI:  Kc = 0.45 / (K * theta/tau), Ti = 7.33*theta (actually 2.2*Pu, Pu=4*theta)
 *   PID: Kc = 0.45 / (K * theta/tau), Ti = 2.2*Pu, Td = Pu/6.3
 *
 * Cohen-Coon (for FOPDT, minimizes various error integrals):
 *   PID: Kc = (1/K)*(tau/theta)*(4/3 + theta/(4*tau))
 *        Ti = theta*(32 + 6*theta/tau) / (13 + 8*theta/tau)
 *        Td = theta*4 / (11 + 2*theta/tau)
 *
 * Reference:
 *   Ziegler & Nichols (1942) "Optimum Settings for Automatic Controllers"
 *   Tyreus & Luyben (1992) "Tuning PI controllers for integrator/deadtime processes"
 * ========================================================================= */
void split_pid_zn_tuning(double process_gain, double process_tau,
                          double process_theta,
                          split_range_tuning_result_t *result, int method) {
    if (!result) return;
    memset(result, 0, sizeof(*result));

    /* Guard against invalid parameters */
    if (process_gain <= 0.0 || process_tau <= 0.0 || process_theta <= 0.0) {
        /* Set conservative defaults */
        result->pid_params.kc = 1.0;
        result->pid_params.ti = 60.0;
        result->pid_params.td = 15.0;
        result->pid_params.sample_time_sec = 1.0;
        snprintf(result->method_name, 48, "ZN-Fallback (invalid process params)");
        return;
    }

    double K = process_gain;
    double tau = process_tau;
    double theta = process_theta;
    double R = theta / tau; /* dead-time-to-time-constant ratio */

    switch (method) {
        case 0: /* Ziegler-Nichols */
            result->pid_params.kc = 1.2 / (K * R);
            result->pid_params.ti = 2.0 * theta;
            result->pid_params.td = 0.5 * theta;
            snprintf(result->method_name, 48, "Ziegler-Nichols Open-Loop (1942)");
            break;

        case 1: /* Tyreus-Luyben */
            /* Pu = 4*theta (approximation for FOPDT),
             * then Kc = Ku/2.2 = (4/pi * 1/A) / 2.2 ≈ 0.45/K * tau/theta */
            result->pid_params.kc = 0.45 / (K * R);
            result->pid_params.ti = 2.2 * 4.0 * theta;  /* 2.2*Pu */
            result->pid_params.td = (4.0 * theta) / 6.3; /* Pu/6.3 */
            snprintf(result->method_name, 48, "Tyreus-Luyben (1992)");
            break;

        case 2: /* Cohen-Coon */
            result->pid_params.kc = (1.0 / K) * (tau / theta)
                                     * (1.333 + theta / (4.0 * tau));
            result->pid_params.ti = theta * (32.0 + 6.0 * R) / (13.0 + 8.0 * R);
            result->pid_params.td = theta * 4.0 / (11.0 + 2.0 * R);
            snprintf(result->method_name, 48, "Cohen-Coon (1953)");
            break;

        default:
            result->pid_params.kc = 1.2 / (K * R);
            result->pid_params.ti = 2.0 * theta;
            result->pid_params.td = 0.5 * theta;
            snprintf(result->method_name, 48, "ZN (default)");
            break;
    }

    /* Apply practical limits */
    if (result->pid_params.kc < 0.01) result->pid_params.kc = 0.01;
    if (result->pid_params.ti < 0.1) result->pid_params.ti = 0.1;
    if (result->pid_params.td < 0.0) result->pid_params.td = 0.0;

    /* Derivative filter, sample time, setpoint weighting defaults */
    result->pid_params.derivative_filter_N = 8.0;
    result->pid_params.sample_time_sec = theta / 10.0; /* 10 samples per dead time */
    if (result->pid_params.sample_time_sec < 0.1)
        result->pid_params.sample_time_sec = 0.1;
    if (result->pid_params.sample_time_sec > 10.0)
        result->pid_params.sample_time_sec = 10.0;

    result->pid_params.beta = 1.0;
    result->pid_params.gamma = 0.0; /* derivative on PV */
    result->pid_params.bumpless_gain = 1.0;
}

/* =========================================================================
 * split_pid_closed_loop_poles — L4
 *
 * Computes the closed-loop poles of the PID-controlled FOPDT system.
 * Uses Pade(1,1) approximation for the dead time term:
 *   exp(-theta*s) ≈ (1 - theta*s/2) / (1 + theta*s/2)
 *
 * The closed-loop characteristic equation for
 *   Gc(s) = Kc*(1 + 1/(Ti*s) + Td*s)  and  Gp(s) = K*exp(-theta*s)/(tau*s+1)
 *
 * After substitution and simplification:
 *   a3*s^3 + a2*s^2 + a1*s + a0 = 0
 *
 * The cubic is solved using Cardano's formula.
 * ========================================================================= */
int split_pid_closed_loop_poles(double K, double tau, double theta,
                                 const split_range_pid_params_t *params,
                                 double poles[3][2]) {
    if (!params || !poles) return 0;

    double Kc = params->kc;
    double Ti = params->ti > 0.0 ? params->ti : 1e10; /* large Ti = no integral */
    double Td = params->td;

    /* Pade approximation coefficients */
    double p1 = -theta / 2.0;
    double p2 = theta / 2.0;

    /* Denominator of Pade + process: (1 + p2*s)*(tau*s+1) = p2*tau*s^2 + (p2+tau)*s + 1 */
    double da2 = p2 * tau;
    double da1 = p2 + tau;
    double da0 = 1.0;

    /* PID numerator: Kc*(1 + 1/(Ti*s) + Td*s) = (Kc*Td*s^2 + Kc*s + Kc/Ti) / s */
    /* Pade numerator: K*(1 + p1*s) = K + K*p1*s */

    /* Closed-loop characteristic: den_PID_open * den_process + num_PID_open * num_process = 0 */

    /* num_PID * num_process: (Kc*Td*s^2 + Kc*s + Kc/Ti) * K*(1 + p1*s) */
    /* = K*Kc*Td*p1*s^3 + K*(Kc*Td + Kc*p1)*s^2 + K*(Kc + Kc*p1/Ti)*s + K*Kc/Ti */
    double nb3 = K * Kc * Td * p1;
    double nb2 = K * (Kc * Td + Kc * p1);
    double nb1 = K * (Kc + Kc * p1 / Ti);
    double nb0 = K * Kc / Ti;

    /* den_PID * den_process: s * (p2*tau*s^2 + (p2+tau)*s + 1) = p2*tau*s^3 + (p2+tau)*s^2 + 1*s */
    double dc3 = da2;
    double dc2 = da1;
    double dc1 = da0;
    double dc0 = 0.0;

    /* Sum: polynomial coefficients (a3, a2, a1, a0) */
    double a3 = dc3 + nb3;
    double a2 = dc2 + nb2;
    double a1 = dc1 + nb1;
    double a0 = dc0 + nb0;

    if (fabs(a3) < 1e-15) {
        /* Degenerate: not actually cubic */
        poles[0][0] = -a0 / (a2 > 1e-15 ? a2 : 1.0);
        poles[0][1] = 0.0;
        poles[1][0] = poles[0][0];
        poles[1][1] = 0.0;
        poles[2][0] = poles[0][0];
        poles[2][1] = 0.0;
        return 3;
    }

    /* Normalize to s^3 + b2*s^2 + b1*s + b0 = 0 */
    double b2 = a2 / a3;
    double b1 = a1 / a3;
    double b0 = a0 / a3;

    /* Depressed cubic: let s = y - b2/3, gives y^3 + p*y + q = 0 */
    double p = b1 - b2 * b2 / 3.0;
    double q = b0 - b2 * b1 / 3.0 + 2.0 * b2 * b2 * b2 / 27.0;

    /* Discriminant: D = (q/2)^2 + (p/3)^3 */
    double discriminant = q * q / 4.0 + p * p * p / 27.0;

    double shift = b2 / 3.0;

    if (discriminant > 0) {
        /* One real root, two complex conjugates */
        double sqrt_D = sqrt(discriminant);
        double u = cbrt(-q / 2.0 + sqrt_D);
        double v = cbrt(-q / 2.0 - sqrt_D);
        double y1 = u + v;

        poles[0][0] = y1 - shift;
        poles[0][1] = 0.0;

        double real_part = -(u + v) / 2.0 - shift;
        double imag_part = sqrt(3.0) * (u - v) / 2.0;
        poles[1][0] = real_part;
        poles[1][1] = imag_part;
        poles[2][0] = real_part;
        poles[2][1] = -imag_part;
    } else if (fabs(discriminant) < 1e-12) {
        /* Triple or double root */
        double y1 = 2.0 * cbrt(-q / 2.0);
        poles[0][0] = y1 - shift;
        poles[0][1] = 0.0;
        poles[1][0] = -y1 / 2.0 - shift;
        poles[1][1] = 0.0;
        poles[2][0] = poles[1][0];
        poles[2][1] = 0.0;
    } else {
        /* Three real roots (casus irreducibilis) */
        double r = sqrt(-p * p * p / 27.0);
        double phi = acos(-q / (2.0 * r));
        double y1 = 2.0 * cbrt(r) * cos(phi / 3.0);
        double y2 = 2.0 * cbrt(r) * cos((phi + 2.0 * M_PI) / 3.0);
        double y3 = 2.0 * cbrt(r) * cos((phi + 4.0 * M_PI) / 3.0);

        poles[0][0] = y1 - shift;
        poles[0][1] = 0.0;
        poles[1][0] = y2 - shift;
        poles[1][1] = 0.0;
        poles[2][0] = y3 - shift;
        poles[2][1] = 0.0;
    }

    return 3;
}
