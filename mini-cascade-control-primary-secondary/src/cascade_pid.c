/**
 * @file cascade_pid.c
 * @brief Cascade PID Controller — Core Algorithm Implementation
 *
 * Implements the PID controller for cascade control systems:
 * - Positional and velocity PID forms
 * - Bumpless transfer (manual↔auto, cascade↔local)
 * - Anti-windup strategies (clamping, back-calculation, conditional integration)
 * - Setpoint filtering and weighting (2-DOF PID)
 * - Output tracking for cascade integration
 *
 * Knowledge Coverage:
 *   L3: PID discretization (backward Euler, Tustin), scan cycle timing
 *   L5: Anti-windup algorithms, 2-DOF setpoint handling
 *
 * References:
 *   Åström & Hägglund, PID Controllers: Theory, Design, and Tuning (1995)
 *   IEC 61131-3 Function Block PID specification
 *   Seborg et al., Process Dynamics and Control (2016), Ch. 8
 *
 * Curriculum: MIT 6.302, Stanford ENGR205, Berkeley ME233, Purdue ME575
 */

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "cascade_types.h"
#include "cascade_pid.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*---------------------------------------------------------------------------
 * L2: PID Controller Initialization
 *---------------------------------------------------------------------------*/

void cascade_pid_init(cascade_pid_controller_t *pid,
                      double kp, double ti, double td,
                      double ts, double out_min, double out_max)
{
    if (!pid) return;

    /* Parameter initialization */
    pid->params.kp = kp;
    pid->params.ti = ti;
    pid->params.td = td;
    pid->params.tf = (td > 1e-12) ? td / 8.0 : 0.1 * ts;
    pid->params.beta = 1.0;
    pid->params.gamma = 0.0;
    pid->params.output_min = out_min;
    pid->params.output_max = out_max;
    pid->params.rate_limit = 100.0;

    /* Controller configuration defaults */
    pid->sample_time = ts;
    pid->form = CASCADE_PID_PARALLEL;
    pid->direction = CASCADE_DIRECT_REVERSE;
    pid->aw_strategy = CASCADE_AW_CLAMPING;
    pid->aw_gain = 1.0;

    /* Scaling defaults: 0-100% */
    pid->pv_scale_min = 0.0;
    pid->pv_scale_max = 100.0;
    pid->co_scale_min = out_min;
    pid->co_scale_max = out_max;

    pid->controller_id = 0;

    /* Reset all dynamic state */
    cascade_pid_reset(pid);
}

void cascade_pid_reset(cascade_pid_controller_t *pid)
{
    if (!pid) return;

    pid->state.p_term = 0.0;
    pid->state.i_term = 0.0;
    pid->state.d_term = 0.0;
    pid->state.last_error = 0.0;
    pid->state.last_pv = 0.0;
    pid->state.integral = 0.0;
    pid->state.prev_output = 0.0;
    pid->state.prev_setpoint = 0.0;
    pid->state.filtered_setpoint = 0.0;
    pid->state.filtered_pv = 0.0;
    pid->state.last_derivative = 0.0;
    pid->state.integrator_active = true;
    pid->state.sample_count = 0;

    /* Initialize filtered PV to avoid derivative kick on first sample */
    /* Will be set properly on first update */
}

void cascade_pid_set_params(cascade_pid_controller_t *pid,
                            double kp, double ti, double td)
{
    if (!pid) return;
    if (kp < 0.0 || ti < 0.0 || td < 0.0) return;

    /* For velocity form, parameter changes are inherently bumpless.
     * For positional form, changing Ti while integral is non-zero
     * may cause a bump. The integral is not rescaled intentionally:
     * in industrial practice the new Ti takes effect over time. */
    pid->params.kp = kp;
    pid->params.ti = ti;
    pid->params.td = td;

    /* Update derivative filter time constant: Tf = Td / N_default */
    pid->params.tf = (td > 1e-12) ? td / 8.0 : pid->params.tf;
}

/*---------------------------------------------------------------------------
 * L3: Positional-Form PID Algorithm
 *
 * Discretization:
 *   Integral: backward Euler (I = I_prev + e * Ts)
 *   Derivative: backward difference on PV ((PV - PV_prev) / Ts)
 *
 * Reference: IEC 61131-3 PID function block, §2.2.3
 *---------------------------------------------------------------------------*/

double cascade_pid_update_positional(cascade_pid_controller_t *pid,
                                      double setpoint, double pv)
{
    if (!pid) return 0.0;

    double Ts = pid->sample_time;
    if (Ts < 1e-12) return pid->state.prev_output;

    /* Error computation with direction correction */
    double error = setpoint - pv;
    if (pid->direction == CASCADE_DIRECT_ACTING) {
        error = pv - setpoint;
    }

    /* On first call, initialize state to avoid derivative kick */
    if (pid->state.sample_count == 0) {
        pid->state.last_pv = pv;
        pid->state.prev_setpoint = setpoint;
    }

    /* --- Proportional term --- */
    /* With setpoint weighting: P = Kp * (beta * SP - PV) */
    pid->state.p_term = pid->params.kp *
        (pid->params.beta * setpoint - pv);

    /* --- Integral term with anti-windup --- */
    if (pid->state.integrator_active && pid->params.ti > 1e-12) {
        pid->state.integral += error * Ts;
        pid->state.i_term = (pid->params.kp / pid->params.ti) *
            pid->state.integral;
    } else {
        pid->state.i_term = 0.0;
    }

    /* --- Derivative term (on PV, not error, to avoid derivative kick) ---
     * Filtered derivative: D_f = Tf/(Tf+Ts) * D_prev + 1/(Tf+Ts) * (Td*(PV-PV_prev)) */
    if (pid->params.td > 1e-12) {
        double raw_deriv = pid->params.td *
            (pid->state.last_pv - pv) / Ts;
        double alpha = pid->params.tf / (pid->params.tf + Ts);
        pid->state.last_derivative = alpha * pid->state.last_derivative
            + (1.0 - alpha) * raw_deriv;
        pid->state.d_term = pid->params.kp * pid->state.last_derivative;
    } else {
        pid->state.d_term = 0.0;
        pid->state.last_derivative = 0.0;
    }

    /* --- Unconstrained output --- */
    double u_unsat = pid->state.p_term + pid->state.i_term +
        pid->state.d_term;

    /* --- Output saturation --- */
    double u_sat = u_unsat;
    if (u_sat > pid->params.output_max) u_sat = pid->params.output_max;
    if (u_sat < pid->params.output_min) u_sat = pid->params.output_min;

    /* --- Rate limiting --- */
    if (pid->params.rate_limit > 0.0) {
        double max_step = pid->params.rate_limit * Ts;
        double prev = pid->state.prev_output;
        if (u_sat - prev > max_step)  u_sat = prev + max_step;
        if (prev - u_sat > max_step)  u_sat = prev - max_step;
    }

    /* --- Anti-windup: clamp integral when saturated --- */
    if (pid->aw_strategy == CASCADE_AW_CLAMPING) {
        if (u_sat == pid->params.output_max && error > 0.0) {
            pid->state.integrator_active = false;
        } else if (u_sat == pid->params.output_min && error < 0.0) {
            pid->state.integrator_active = false;
        } else {
            pid->state.integrator_active = true;
        }

        /* Back-calculate integral term to match saturated output */
        if (pid->aw_strategy == CASCADE_AW_BACK_CALCULATION) {
            double tracking_term = (u_sat - u_unsat) * Ts /
                (pid->params.ti > 1e-12 ? pid->params.ti : Ts);
            pid->state.integral += pid->aw_gain * tracking_term;
        }
    }

    /* --- Update state for next iteration --- */
    pid->state.last_error = error;
    pid->state.last_pv = pv;
    pid->state.prev_output = u_sat;
    pid->state.sample_count++;

    return u_sat;
}

/*---------------------------------------------------------------------------
 * L3: Velocity-Form (Incremental) PID Algorithm
 *
 * du(k) = Kp * (e_k - e_{k-1}) + (Kp*Ts/Ti) * e_k
 *       + (Kp*Td/Ts) * (2*PV_{k-1} - PV_k - PV_{k-2})
 *
 * The velocity form is inherently bumpless:
 * - No integrator accumulator to wind up
 * - Mode changes produce zero initial delta-u
 * - Ideal for cascade secondary loops
 *
 * Reference: Åström & Hägglund (1995), Section 3.4
 *---------------------------------------------------------------------------*/

double cascade_pid_update_velocity(cascade_pid_controller_t *pid,
                                    double setpoint, double pv)
{
    if (!pid) return 0.0;

    double Ts = pid->sample_time;
    if (Ts < 1e-12) return pid->state.prev_output;

    double error = setpoint - pv;
    if (pid->direction == CASCADE_DIRECT_ACTING) {
        error = pv - setpoint;
    }

    /* Delta-P (proportional increment) */
    double dP = pid->params.kp * (error - pid->state.last_error);

    /* Delta-I (integral increment) */
    double dI = 0.0;
    if (pid->params.ti > 1e-12) {
        dI = (pid->params.kp * Ts / pid->params.ti) * error;
    }

    /* Delta-D (derivative increment, on PV to avoid setpoint kick) */
    double dD = 0.0;
    if (pid->params.td > 1e-12) {
        /* Two-sample backward: ΔD = Kp*Td/Ts * (2*PV_{k-1} - PV_k - PV_{k-2}) */
        double pv_prev = pid->state.last_pv;
        double pv_prev2 = pid->state.filtered_pv; /* Reuse for PV_{k-2} storage */
        dD = (pid->params.kp * pid->params.td / Ts) *
            (2.0 * pv_prev - pv - pv_prev2);
    }

    /* Total increment */
    double du = dP + dI + dD;

    /* Anti-windup for velocity form:
     * If du would drive output beyond limits, reduce du */
    double new_output = pid->state.prev_output + du;
    if (new_output > pid->params.output_max) {
        du = pid->params.output_max - pid->state.prev_output;
        new_output = pid->params.output_max;
    }
    if (new_output < pid->params.output_min) {
        du = pid->params.output_min - pid->state.prev_output;
        new_output = pid->params.output_min;
    }

    /* Rate limiting on delta-u */
    if (pid->params.rate_limit > 0.0) {
        double max_du = pid->params.rate_limit * Ts;
        double max_du_neg = -max_du;
        if (du > max_du) du = max_du;
        if (du < max_du_neg) du = max_du_neg;
        new_output = pid->state.prev_output + du;
    }

    /* Update state */
    pid->state.last_error = error;
    pid->state.filtered_pv = pid->state.last_pv; /* Shift PV_{k-1} → PV_{k-2} */
    pid->state.last_pv = pv;
    pid->state.prev_output = new_output;
    pid->state.last_derivative = du; /* Store increment for diagnostics */
    pid->state.sample_count++;

    return new_output;
}

/*---------------------------------------------------------------------------
 * L3: Bumpless Transfer Mechanisms
 *
 * Bumpless transfer ensures no sudden change in controller output when
 * switching operating modes. This is critical in cascade control where
 * the primary output becomes the secondary setpoint.
 *---------------------------------------------------------------------------*/

void cascade_pid_bumpless_manual_to_auto(cascade_pid_controller_t *pid,
                                          double manual_output,
                                          double setpoint, double pv)
{
    if (!pid) return;

    /* Back-calculate integral so that u_auto = manual_output at transition */
    double error = setpoint - pv;
    if (pid->direction == CASCADE_DIRECT_ACTING) {
        error = pv - setpoint;
    }

    double p_term = pid->params.kp *
        (pid->params.beta * setpoint - pv);
    double d_term = 0.0;

    /* Compute current D term for alignment */
    if (pid->params.td > 1e-12 && pid->sample_time > 1e-12) {
        double raw_deriv = pid->params.td *
            (pid->state.last_pv - pv) / pid->sample_time;
        d_term = pid->params.kp * raw_deriv;
    }

    /* Integral = manual_output - P - D (positional form) */
    if (pid->params.ti > 1e-12) {
        pid->state.integral = (manual_output - p_term - d_term) *
            pid->params.ti / pid->params.kp;
        pid->state.i_term = (pid->params.kp / pid->params.ti) *
            pid->state.integral;
    }

    /* Set initial state */
    pid->state.prev_output = manual_output;
    pid->state.last_pv = pv;
    pid->state.filtered_pv = pv;
    pid->state.last_error = error;
    pid->state.integrator_active = true;
}

double cascade_pid_bumpless_auto_to_manual(cascade_pid_controller_t *pid)
{
    if (!pid) return 0.0;
    return pid->state.prev_output;
}

/*---------------------------------------------------------------------------
 * L3: Setpoint Filtering — First-Order Exponential Filter
 *
 * SP_f(k) = SP_f(k-1) + (Ts/Tf) * (SP_raw - SP_f(k-1))
 *
 * This reduces derivative kick and provides smooth transitions.
 * Tf = 0 means no filtering (step change).
 *---------------------------------------------------------------------------*/

double cascade_pid_setpoint_filter(cascade_pid_controller_t *pid,
                                    double raw_sp, double filter_tau)
{
    if (!pid) return raw_sp;
    if (filter_tau <= 0.0 || pid->sample_time <= 0.0) return raw_sp;

    double alpha = pid->sample_time / (filter_tau + pid->sample_time);
    pid->state.filtered_setpoint = alpha * raw_sp +
        (1.0 - alpha) * pid->state.filtered_setpoint;

    return pid->state.filtered_setpoint;
}

double cascade_pid_setpoint_ramp(double current_sp, double target_sp,
                                  double max_rate, double ts)
{
    if (ts <= 0.0) return target_sp;

    double delta = target_sp - current_sp;
    double max_delta = max_rate * ts;

    if (fabs(delta) <= max_delta) {
        return target_sp;
    }

    if (delta > 0.0) {
        return current_sp + max_delta;
    } else {
        return current_sp - max_delta;
    }
}

/*---------------------------------------------------------------------------
 * L5: Anti-Windup Methods
 *
 * Three classic approaches:
 * 1. Clamping — freeze integrator when output saturates
 * 2. Back-calculation — feed saturation error back to integrator
 * 3. Conditional integration — only integrate when error is small
 *
 * Reference: Åström & Hägglund (1995), Ch. 3.5
 *---------------------------------------------------------------------------*/

void cascade_pid_anti_windup_clamping(cascade_pid_controller_t *pid,
                                       double error)
{
    if (!pid) return;

    double u_cur = pid->state.prev_output;
    double u_max = pid->params.output_max;
    double u_min = pid->params.output_min;

    /* Windup condition: output saturated AND integrator driving further out */
    if (u_cur >= u_max && error > 0.0) {
        pid->state.integrator_active = false;  /* Windup: stop integrating */
    } else if (u_cur <= u_min && error < 0.0) {
        pid->state.integrator_active = false;  /* Windup: stop integrating */
    } else {
        pid->state.integrator_active = true;   /* Normal: integrate freely */
    }
}

void cascade_pid_anti_windup_back_calc(cascade_pid_controller_t *pid,
                                        double u_unsat, double u_sat,
                                        double tracking_time)
{
    if (!pid || tracking_time < 1e-12) return;

    /* Tracking error: difference between saturated and unsaturated output */
    double track_err = u_sat - u_unsat;

    if (fabs(track_err) < 1e-12) return;

    /* Adjust integral using tracking time constant Tt:
     *   dI/dt = Kp/Tt * (u_sat - u_unsat)
     *   I(k+1) = I(k) + (Ts/Tt) * (u_sat - u_unsat) * (Ti/Kp)
     *
     * In discrete form with Ti normalized:
     *   delta_integral = (Ts/Tt) * (u_sat - u_unsat) * (Ti/Kp)
     */
    double delta_I = (pid->sample_time / tracking_time) * track_err *
        (pid->params.ti / pid->params.kp);
    pid->state.integral += pid->aw_gain * delta_I;
}

void cascade_pid_anti_windup_conditional(cascade_pid_controller_t *pid,
                                          double error, double threshold,
                                          bool output_saturated)
{
    if (!pid) return;

    /* Conditional integration logic:
     * - If |error| > threshold: error is large, likely saturation ahead → stop
     * - If output is saturated: stop integrating regardless
     * - Otherwise: integrate normally */
    if (fabs(error) > threshold || output_saturated) {
        pid->state.integrator_active = false;
    } else {
        pid->state.integrator_active = true;
    }
}

/*---------------------------------------------------------------------------
 * L5: Two-Degree-of-Freedom PID (Setpoint Weighting)
 *
 * u = Kp * [beta*SP - PV + 1/Ti*I + Td*d(gamma*SP - PV)/dt]
 *
 * beta = 0: no proportional kick from SP change
 * gamma = 0: no derivative kick from SP change (standard industrial practice)
 *
 * This is distinct from regular PID because it separates the response
 * to setpoint changes (servo) from disturbance rejection (regulator).
 *---------------------------------------------------------------------------*/

double cascade_pid_setpoint_weighting(cascade_pid_controller_t *pid,
                                       double setpoint, double pv,
                                       double beta, double gamma)
{
    if (!pid) return 0.0;

    double Ts = pid->sample_time;
    if (Ts < 1e-12) return pid->state.prev_output;

    double error = setpoint - pv;
    if (pid->direction == CASCADE_DIRECT_ACTING) {
        error = pv - setpoint;
    }

    /* On first call, initialize state to avoid derivative kick */
    if (pid->state.sample_count == 0) {
        pid->state.last_pv = pv;
        pid->state.prev_setpoint = setpoint;
    }

    /* P: with setpoint weighting beta */
    double p_term = pid->params.kp * (beta * setpoint - pv);

    /* I: standard integral on error (not affected by beta/gamma) */
    double i_term = 0.0;
    if (pid->state.integrator_active && pid->params.ti > 1e-12) {
        pid->state.integral += error * Ts;
        i_term = (pid->params.kp / pid->params.ti) * pid->state.integral;
    }

    /* D: derivative on (gamma*SP - PV) to control derivative kick */
    double d_term = 0.0;
    if (pid->params.td > 1e-12) {
        double deriv_input = gamma * setpoint - pv;
        double prev_deriv_input = gamma * pid->state.prev_setpoint -
            pid->state.last_pv;
        double raw_deriv = pid->params.td *
            (prev_deriv_input - deriv_input) / Ts;
        double alpha = pid->params.tf / (pid->params.tf + Ts);
        pid->state.last_derivative =
            alpha * pid->state.last_derivative + (1.0 - alpha) * raw_deriv;
        d_term = pid->params.kp * pid->state.last_derivative;
    }

    /* Sum and clamp */
    double u = p_term + i_term + d_term;
    if (u > pid->params.output_max) u = pid->params.output_max;
    if (u < pid->params.output_min) u = pid->params.output_min;

    /* State update */
    pid->state.last_pv = pv;
    pid->state.prev_setpoint = setpoint;
    pid->state.prev_output = u;
    pid->state.last_error = error;
    pid->state.p_term = p_term;
    pid->state.i_term = i_term;
    pid->state.d_term = d_term;
    pid->state.sample_count++;

    return u;
}

/*---------------------------------------------------------------------------
 * L5: Ideal and Series PID Forms
 *
 * The ISA ideal form and the interacting series form are commonly found
 * in DCS implementations. This function provides both forms.
 *---------------------------------------------------------------------------*/

double cascade_pid_compute_ideal(cascade_pid_controller_t *pid,
                                  double setpoint, double pv)
{
    if (!pid) return 0.0;

    double Ts = pid->sample_time;
    if (Ts < 1e-12) return pid->state.prev_output;

    double error = setpoint - pv;
    if (pid->direction == CASCADE_DIRECT_ACTING) {
        error = pv - setpoint;
    }

    /* P term */
    double P = pid->params.kp * error;

    /* I term: I = Kp/Ti * ∫ e dt */
    double I = 0.0;
    if (pid->params.ti > 1e-12 && pid->state.integrator_active) {
        pid->state.integral += error * Ts;
        I = (pid->params.kp / pid->params.ti) * pid->state.integral;
    }

    /* D term: D = Kp * Td * de/dt (filtered) */
    double D = 0.0;
    if (pid->params.td > 1e-12) {
        double raw_deriv = pid->params.td *
            (error - pid->state.last_error) / Ts;
        double alpha = pid->params.tf / (pid->params.tf + Ts);
        pid->state.last_derivative =
            alpha * pid->state.last_derivative + (1.0 - alpha) * raw_deriv;
        D = pid->params.kp * pid->state.last_derivative;
    }

    double u = P + I + D;

    /* Saturation */
    if (u > pid->params.output_max) u = pid->params.output_max;
    if (u < pid->params.output_min) u = pid->params.output_min;

    /* Anti-windup clamping */
    if ((u >= pid->params.output_max && error > 0.0) ||
        (u <= pid->params.output_min && error < 0.0)) {
        pid->state.integrator_active = false;
    } else {
        pid->state.integrator_active = true;
    }

    pid->state.last_error = error;
    pid->state.prev_output = u;
    pid->state.last_pv = pv;
    pid->state.sample_count++;

    return u;
}

double cascade_pid_compute_series(cascade_pid_controller_t *pid,
                                   double setpoint, double pv)
{
    if (!pid) return 0.0;

    double Ts = pid->sample_time;
    if (Ts < 1e-12) return pid->state.prev_output;

    double error = setpoint - pv;
    if (pid->direction == CASCADE_DIRECT_ACTING) {
        error = pv - setpoint;
    }

    /* Series PID: Gc(s) = Kc * (1 + 1/(Ti*s)) * (Td*s + 1)
     *
     * In discrete form:
     *   u_PI = Kc * (error + I)
     *   u = u_PI + Td * d(u_PI)/dt
     *
     * This means the derivative acts on the PI output, not on PV directly.
     */

    /* PI stage */
    double u_pi = pid->params.kp * error;
    if (pid->params.ti > 1e-12 && pid->state.integrator_active) {
        pid->state.integral += error * Ts;
        u_pi += (pid->params.kp / pid->params.ti) * pid->state.integral;
    }

    /* D stage: u = u_pi + Td * du_pi/dt */
    double u = u_pi;
    if (pid->params.td > 1e-12) {
        double du_pi = (u_pi - pid->state.filtered_setpoint) / Ts;
        u += pid->params.td * du_pi;
    }

    /* Store PI output for next derivative */
    pid->state.filtered_setpoint = u_pi;

    /* Saturation */
    if (u > pid->params.output_max) u = pid->params.output_max;
    if (u < pid->params.output_min) u = pid->params.output_min;

    /* Anti-windup */
    if ((u >= pid->params.output_max && error > 0.0) ||
        (u <= pid->params.output_min && error < 0.0)) {
        pid->state.integrator_active = false;
    } else {
        pid->state.integrator_active = true;
    }

    pid->state.last_error = error;
    pid->state.prev_output = u;
    pid->state.last_pv = pv;
    pid->state.sample_count++;

    return u;
}

/*---------------------------------------------------------------------------
 * L5: Output Tracking for Cascade Integration
 *
 * When the inner (secondary) loop is in manual or local mode,
 * the outer (primary) loop must track the secondary's PV
 * to allow bumpless reconnection of the cascade.
 *---------------------------------------------------------------------------*/

void cascade_pid_output_tracking(cascade_pid_controller_t *pid,
                                  double tracking_value)
{
    if (!pid) return;

    /* Force the output to track the specified value.
     * Back-calculate the integral to maintain smooth transition. */
    double u_desired = tracking_value;

    /* Clamp to output limits */
    if (u_desired > pid->params.output_max)
        u_desired = pid->params.output_max;
    if (u_desired < pid->params.output_min)
        u_desired = pid->params.output_min;

    /* Back-calculate integral term */
    double p_term = pid->state.p_term;
    double d_term = pid->state.d_term;

    if (pid->params.ti > 1e-12 && pid->params.kp > 1e-12) {
        pid->state.integral = (u_desired - p_term - d_term) *
            pid->params.ti / pid->params.kp;
    }

    pid->state.prev_output = u_desired;
    pid->state.integrator_active = true;
}
