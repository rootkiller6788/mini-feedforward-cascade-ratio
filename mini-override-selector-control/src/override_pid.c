/**
 * @file override_pid.c
 * @brief Override/Selector Control — PID Controller Implementation
 *
 * Implements PID controllers specifically designed for override
 * selector schemes. Supports positional and velocity form updates,
 * external reset tracking (IEC 61131-3), multiple anti-windup
 * methods, and bumpless transfer initialization.
 *
 * Knowledge Coverage:
 *   L2: Anti-windup (3 methods), external reset, bumpless transfer
 *   L3: Positional vs velocity PID, discretization, tracking
 *   L4: IEC 61131-3 PID function block, ISA form conversion
 *   L5: PID update algorithms with constraints
 *
 * Reference:
 *   Astrom & Hagglund (1995). PID Controllers: Theory, Design, and Tuning.
 *   IEC 61131-3 (2013). Section on PID Function Block.
 */

#include "override_pid.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef INFINITY
#define INFINITY (1.0/0.0)
#endif

/* =========================================================================
 * L2 — PID Initialization
 * ========================================================================= */

void override_pid_init(override_controller_t *ctrl,
                       double Kc, double Ti, double Td,
                       double N, double Ts,
                       double u_min, double u_max,
                       double tr_gain) {
    if (ctrl == NULL) return;

    memset(ctrl, 0, sizeof(override_controller_t));

    ctrl->params.Kc = Kc;
    ctrl->params.Ti = Ti;
    ctrl->params.Td = Td;
    ctrl->params.N = N;
    ctrl->params.Ts = Ts;
    ctrl->params.b = 1.0;
    ctrl->params.c = 0.0;
    ctrl->params.u_min = u_min;
    ctrl->params.u_max = u_max;
    ctrl->params.tracking_gain = tr_gain;

    ctrl->enabled = 1;
    ctrl->faulted = 0;
    ctrl->active = 0;
    ctrl->windup_limited = 0;

    ctrl->integral = 0.0;
    ctrl->last_error = 0.0;
    ctrl->last_pv = 0.0;
    ctrl->last_output = 0.0;
}

void override_pid_init_from_params(override_controller_t *ctrl,
                                   const override_pid_params_t *params) {
    if (ctrl == NULL || params == NULL) return;

    memcpy(&ctrl->params, params, sizeof(override_pid_params_t));

    ctrl->enabled = 1;
    ctrl->faulted = 0;
    ctrl->active = 0;
    ctrl->windup_limited = 0;

    ctrl->integral = 0.0;
    ctrl->last_error = 0.0;
    ctrl->last_pv = 0.0;
    ctrl->last_output = 0.0;
}

/* =========================================================================
 * L3 — Positional PID Update with Tracking and Anti-Windup
 * ========================================================================= */

/**
 * Compute the ISA standard positional PID output.
 *
 * The PID equation (ISA Standard form):
 *   u(t) = Kc * [b*r(t) - y(t)] + Kc/Ti * ∫(r-y)dt + Kc*Td * d(c*r - y)/dt
 *
 * Discretized using:
 *   - Backward Euler for integral: I(k) = I(k-1) + (Ts/Ti) * e(k)
 *   - Backward difference for derivative (on PV): D(k) = -Td/(Ts/N + Ts) * (y(k) - y(k-1) - D(k-1))
 *     (filtered derivative with N: derivative acts on -dy/dt)
 *   - Setpoint weighting: b for P, c for D
 *
 * External Reset (Tracking):
 *   When tracking is active (controller.output should follow tracking_value):
 *     I = tracking_value / Kc - b*r + y - D   [back-calculate integral]
 *
 * Anti-Windup:
 *   Back-calculation: I = I - (Ts/Tt)*(u_clamped - u_raw)
 *   where Tt ≈ sqrt(Ti*Td) or Ti (tracking time constant)
 */
double override_pid_update(override_controller_t *ctrl,
                           double sp, double pv, double dt) {
    if (ctrl == NULL || !ctrl->enabled) return 0.0;
    if (dt <= 0.0) dt = ctrl->params.Ts;
    if (dt <= 0.0) dt = 0.1; /* Sanity fallback */

    double Kc = ctrl->params.Kc;
    double Ti = ctrl->params.Ti;
    double Td = ctrl->params.Td;
    double N  = ctrl->params.N;
    double b  = ctrl->params.b;
    double c  = ctrl->params.c;
    double u_min = ctrl->params.u_min;
    double u_max = ctrl->params.u_max;
    double tr_gain = ctrl->params.tracking_gain;

    double error = sp - pv;

    /* --- D-term (derivative on PV, filtered) --- */
    /* The filtered derivative on measurement:
       D_f = Td/(Ts/N + Ts) * (D_f_prev - (pv - pv_prev)) where Ts_filter = Ts/N
       Actually using standard form: D(s) = Kc*Td*s / (1 + s*Td/N) * (-PV)
       Discretization: D(k) = a_d * D(k-1) - b_d * (PV(k) - PV(k-1))
       where a_d = Td/(N*Ts + Td), b_d = Kc*Td*N/(N*Ts + Td)
    */
    double Tf = Td / N;  /* Filter time constant */
    double a_d = 0.0;
    double b_d = 0.0;

    if (Td > 0.0 && dt > 0.0) {
        a_d = Td / (N * dt + Td);
        b_d = Kc * Td * N / (N * dt + Td);
    }

    /* D term: previous derivative contribution (stored in last_error for D) */
    double d_term = 0.0;
    if (Td > 0.0) {
        /* Compute new D output */
        d_term = a_d * ctrl->last_error - b_d * (pv - ctrl->last_pv);
    }

    /* --- P-term with setpoint weighting --- */
    double p_term = Kc * (b * sp - pv);

    /* --- I-term --- */
    /* When active (not tracking), normal integral update.
       When tracking, set I to make output = tracking_value. */
    double i_term = ctrl->integral;

    if (!ctrl->active) {
        /* Inactive — tracking mode: back-calculate integral so that
           P + I + D = tracking_value
           => I = tracking_value - P - D
           But we cap I to keep output within limits */
        i_term = ctrl->tracking_value - p_term - d_term;
        /* Clamp I to reasonable range */
        double i_min = u_min - p_term - d_term;
        double i_max = u_max - p_term - d_term;
        if (i_term < i_min) i_term = i_min;
        if (i_term > i_max) i_term = i_max;
    } else {
        /* Active — integrate error */
        if (Ti > 0.0 && Ti < INFINITY) {
            double di = Kc * (dt / Ti) * error;
            i_term += di;
        }
    }

    ctrl->integral = i_term;

    /* --- Summation --- */
    double u_raw = p_term + i_term + d_term;

    /* --- Output clamping (saturation) --- */
    double u_clamped = u_raw;
    if (u_clamped < u_min) u_clamped = u_min;
    if (u_clamped > u_max) u_clamped = u_max;

    /* --- Anti-Windup: Back-Calculation --- */
    if (ctrl->active && u_raw != u_clamped && tr_gain > 0.0) {
        override_pid_back_calc(ctrl, u_raw, u_clamped, dt);
        ctrl->windup_limited = 1;
    } else {
        ctrl->windup_limited = 0;
    }

    /* Store state for next iteration */
    ctrl->last_error = error;
    ctrl->last_pv = pv;

    /* Store D term contribution for next cycle (reuse last_error slot for filtered D) */
    ctrl->last_output = u_clamped;

    ctrl->output = u_clamped;
    ctrl->setpoint = sp;
    ctrl->pv = pv;

    return u_clamped;
}

/* =========================================================================
 * L3 — Velocity (Incremental) PID Update
 * ========================================================================= */

double override_pid_update_velocity(override_controller_t *ctrl,
                                    double sp, double pv, double dt) {
    if (ctrl == NULL || !ctrl->enabled) return 0.0;
    if (dt <= 0.0) dt = ctrl->params.Ts;
    if (dt <= 0.0) dt = 0.1;

    double Kc = ctrl->params.Kc;
    double Ti = ctrl->params.Ti;
    double Td = ctrl->params.Td;
    double N  = ctrl->params.N;

    double error = sp - pv;
    double error_prev = ctrl->last_error;
    double pv_prev = ctrl->last_pv;

    /* Velocity form (incremental):
       Δu = Kc * [ (e(k) - e(k-1)) + (Ts/Ti)*e(k) + (Td/Ts)*(Δe(k) - Δe(k-1)) ]
       with derivative on PV:
       Δu = Kc * [ -(pv(k) - pv(k-1)) + (Ts/Ti)*e(k) - (Td/Ts)*(pv(k) - 2*pv(k-1) + pv(k-2)) ]
    */

    double dpv = pv - pv_prev;

    /* P increment (act on PV change for bumpless behavior) */
    double dp = -Kc * dpv;

    /* I increment */
    double di = 0.0;
    if (Ti > 0.0 && Ti < INFINITY) {
        di = Kc * (dt / Ti) * error;
    }

    /* D increment with filter — backward difference on PV change */
    double dd = 0.0;
    if (Td > 0.0) {
        double Tf = Td / N;
        double a_d = Td / (N * dt + Td);
        /* previous filtered derivative (stored in last_error in velocity mode) */
        double d_prev = ctrl->last_error; /* misuse: stores previous D contribution */
        dd = a_d * d_prev - Kc * Td * N / (N * dt + Td) * (pv - 2.0 * pv_prev + ctrl->last_output);
    }

    double du = dp + di + dd;

    /* Clamp delta to output limits */
    double u_min = ctrl->params.u_min;
    double u_max = ctrl->params.u_max;
    double u_new = ctrl->output + du;
    if (u_new < u_min) { du = u_min - ctrl->output; }
    if (u_new > u_max) { du = u_max - ctrl->output; }

    ctrl->output += du;
    ctrl->last_error = error;
    ctrl->last_pv = pv;
    ctrl->last_output = pv; /* stores pv(k-2) for velocity D */
    ctrl->setpoint = sp;
    ctrl->pv = pv;

    return du;
}

/* =========================================================================
 * L3 — 2-DOF PID Update (Setpoint Weighting)
 * ========================================================================= */

double override_pid_update_2dof(override_controller_t *ctrl,
                                double sp, double pv, double dt) {
    /* 2-DOF is the standard positional update with b and c weights.
       The base positional PID already implements 2-DOF via b and c.
       This provides an explicit entry point. */
    return override_pid_update(ctrl, sp, pv, dt);
}

/* =========================================================================
 * L3 — Rate-Limited Setpoint PID Update
 * ========================================================================= */

double override_pid_update_rate_limited_sp(override_controller_t *ctrl,
                                           double sp, double pv,
                                           double dt, double sp_rate_lim) {
    if (ctrl == NULL) return 0.0;
    if (dt <= 0.0) dt = ctrl->params.Ts;
    if (dt <= 0.0) dt = 0.1;

    /* Apply rate limit to setpoint */
    static double filtered_sp = 0.0; /* Note: in a real system, this should be
                                        per-controller state. For demonstration,
                                        we use a single static. In production,
                                        add a filtered_sp field to the struct. */

    double max_delta = sp_rate_lim * dt;
    double delta = sp - ctrl->setpoint;
    if (delta > max_delta) {
        filtered_sp = ctrl->setpoint + max_delta;
    } else if (delta < -max_delta) {
        filtered_sp = ctrl->setpoint - max_delta;
    } else {
        filtered_sp = sp;
    }

    return override_pid_update(ctrl, filtered_sp, pv, dt);
}

/* =========================================================================
 * L3 — Anti-Windup Implementations
 * ========================================================================= */

void override_pid_back_calc(override_controller_t *ctrl,
                            double u_raw, double u_clamped, double dt) {
    if (ctrl == NULL) return;
    if (dt <= 0.0) return;

    double Tt = ctrl->params.Ti;
    if (ctrl->params.Td > 0.0) {
        /* Default tracking time: Tt = sqrt(Ti * Td) per Astrom & Hagglund */
        double tt_sqrt = sqrt(ctrl->params.Ti * ctrl->params.Td);
        if (tt_sqrt > 0.0) Tt = tt_sqrt;
    }

    double tracking_correction = (dt / Tt) * (u_clamped - u_raw);
    ctrl->integral += tracking_correction;
}

void override_pid_cond_integration(override_controller_t *ctrl,
                                   double error, int u_sat) {
    if (ctrl == NULL) return;

    if (u_sat) {
        /* Freeze integral if error would push further into saturation */
        if ((error > 0.0 && ctrl->output >= ctrl->params.u_max) ||
            (error < 0.0 && ctrl->output <= ctrl->params.u_min)) {
            /* Do not update integral — hold previous value */
            ctrl->windup_limited = 1;
            return;
        }
    }
    ctrl->windup_limited = 0;
}

double override_pid_clamp_integral(override_controller_t *ctrl,
                                   double di_raw, double u_clamped,
                                   int u_limits) {
    if (ctrl == NULL) return 0.0;

    double Kc = ctrl->params.Kc;
    double new_i = ctrl->integral + di_raw;

    if (u_limits > 0) {
        /* Upper limit active: clamp integral */
        double i_max = ctrl->params.u_max -
                       Kc * (ctrl->params.b * ctrl->setpoint - ctrl->pv);
        if (new_i > i_max) new_i = i_max;
    } else if (u_limits < 0) {
        /* Lower limit active: clamp integral */
        double i_min = ctrl->params.u_min -
                       Kc * (ctrl->params.b * ctrl->setpoint - ctrl->pv);
        if (new_i < i_min) new_i = i_min;
    }

    double di_actual = new_i - ctrl->integral;
    ctrl->integral = new_i;
    return di_actual;
}

/* =========================================================================
 * L3 — Tracking and Bumpless Transfer
 * ========================================================================= */

void override_pid_set_tracking(override_controller_t *ctrl,
                               double tracking_value, double dt) {
    if (ctrl == NULL) return;

    ctrl->tracking_value = tracking_value;

    /* Back-calculate integral so output = tracking_value:
       I = tracking_value - Kc*(b*sp - pv) - D */
    double Kc = ctrl->params.Kc;
    double p_term = Kc * (ctrl->params.b * ctrl->setpoint - ctrl->pv);
    /* Approximate D as 0 during tracking initialization */
    ctrl->integral = tracking_value - p_term;

    /* Clamp integral to avoid excessive values */
    double i_min = ctrl->params.u_min - p_term;
    double i_max = ctrl->params.u_max - p_term;
    if (ctrl->integral < i_min) ctrl->integral = i_min;
    if (ctrl->integral > i_max) ctrl->integral = i_max;

    ctrl->output = tracking_value;
    ctrl->last_output = tracking_value;
}

void override_pid_bumpless_init(override_controller_t *ctrl,
                                double init_output,
                                double sp, double pv) {
    if (ctrl == NULL) return;

    ctrl->setpoint = sp;
    ctrl->pv = pv;
    ctrl->last_pv = pv;

    double Kc = ctrl->params.Kc;
    double p_term = Kc * (ctrl->params.b * sp - pv);
    double d_term = 0.0; /* Assume D=0 at initialization */

    /* Set integral so that P + I + D = init_output */
    ctrl->integral = init_output - p_term - d_term;

    /* Clamp */
    double i_min = ctrl->params.u_min - p_term;
    double i_max = ctrl->params.u_max - p_term;
    if (ctrl->integral < i_min) ctrl->integral = i_min;
    if (ctrl->integral > i_max) ctrl->integral = i_max;

    ctrl->output = init_output;
    ctrl->last_output = init_output;
    ctrl->last_error = sp - pv;
}

int override_pid_is_saturated(const override_controller_t *ctrl) {
    if (ctrl == NULL) return 0;
    return (ctrl->output <= ctrl->params.u_min ||
            ctrl->output >= ctrl->params.u_max) ? 1 : 0;
}

double override_pid_tracking_error(const override_controller_t *ctrl) {
    if (ctrl == NULL) return 0.0;
    return ctrl->output - ctrl->tracking_value;
}

/* =========================================================================
 * L4 — PID Form Conversion
 * ========================================================================= */

void override_pid_isa_to_parallel(const override_pid_params_t *src,
                                  double *Kp, double *Ki, double *Kd) {
    if (src == NULL || Kp == NULL || Ki == NULL || Kd == NULL) return;

    *Kp = src->Kc;
    *Ki = (src->Ti > 0.0 && src->Ti < INFINITY) ? src->Kc / src->Ti : 0.0;
    *Kd = src->Kc * src->Td;
}

int override_pid_parallel_to_isa(double Kp, double Ki, double Kd,
                                 override_pid_params_t *dst) {
    if (dst == NULL) return -1;
    if (Kp <= 0.0) return -1;

    /* Special case: Ki = 0 (PD controller) */
    if (Ki <= 0.0) {
        dst->Kc = Kp;
        dst->Ti = INFINITY;  /* Integral disabled */
        dst->Td = Kd / Kp;
    } else {
        dst->Kc = Kp;
        dst->Ti = Kp / Ki;
        dst->Td = Kd / Kp;
    }

    return 0;
}

void override_pid_isa_to_series(const override_pid_params_t *src,
                                double *Kc_s, double *Ti_s, double *Td_s) {
    if (src == NULL || Kc_s == NULL || Ti_s == NULL || Td_s == NULL) return;

    /* Series (interacting) form conversion from ISA standard:
       Kc_s = Kc * (Ti + Td) / Ti
       Ti_s = Ti + Td
       Td_s = Ti * Td / (Ti + Td)
    */
    double Ti = src->Ti;
    double Td = src->Td;

    if (Ti <= 0.0 || Ti >= INFINITY) {
        /* PI not defined, fallback */
        *Kc_s = src->Kc;
        *Ti_s = Ti;
        *Td_s = 0.0;
        return;
    }

    double sum = Ti + Td;
    double prod = Ti * Td;

    *Kc_s = src->Kc * sum / Ti;
    *Ti_s = sum;
    *Td_s = (sum > 0.0) ? prod / sum : 0.0;
}
