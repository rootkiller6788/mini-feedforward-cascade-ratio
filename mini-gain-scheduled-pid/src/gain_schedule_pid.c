#include "gain_schedule_pid.h"
#include "gain_schedule_interp.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void gs_pid_init(gs_pid_state_t *state,
                 const char *tag,
                 gs_pid_form_t form) {
    if (!state) return;
    memset(state, 0, sizeof(gs_pid_state_t));
    
    if (tag) {
        strncpy(state->tag, tag, sizeof(state->tag) - 1);
        state->tag[sizeof(state->tag) - 1] = '\0';
    } else {
        strcpy(state->tag, "GS-PID-000");
    }
    strcpy(state->description, "Gain-Scheduled PID Controller");
    state->pid_form = form;
    
    state->Kp_current = 1.0;
    state->Ki_current = 0.1;
    state->Kd_current = 0.0;
    state->Ti_current = 10.0;
    state->Td_current = 0.0;
    state->N_current  = 10.0;
    state->b_current  = 1.0;
    state->c_current  = 0.0;
    
    state->saturation_high = +1e300;
    state->saturation_low  = -1e300;
    state->saturated = false;
    state->dt = 0.1;
    state->step_count = 0;
}

void gs_pid_set_saturation(gs_pid_state_t *state,
                            double high, double low) {
    if (!state) return;
    state->saturation_high = high;
    state->saturation_low  = low;
}

void gs_pid_set_dt(gs_pid_state_t *state, double dt) {
    if (!state || dt <= 0.0) return;
    state->dt = dt;
}

void gs_pid_reset_integral(gs_pid_state_t *state) {
    if (!state) return;
    state->integral = 0.0;
}

void gs_pid_set_integral(gs_pid_state_t *state, double value) {
    if (!state) return;
    state->integral = value;
}

void gs_pid_tracking_mode(gs_pid_state_t *state,
                           double tracking_input) {
    if (!state) return;
    state->tracking_input = tracking_input;
    state->control_output = tracking_input;
    state->prev_control_output = tracking_input;
    state->error = 0.0;
    state->prev_error = 0.0;
    state->derivative = 0.0;
    state->integral = tracking_input;
}

void gs_pid_update(gs_pid_state_t *state,
                   const gain_schedule_table_t *table,
                   double setpoint,
                   double pv,
                   double sched_val,
                   double *output) {
    if (!state || !output) return;
    
    /* Filter scheduling variable */
    double alpha_sched = state->sched_filter_state;
    double Ts = state->dt;
    double tau_s = 1.0;
    if (state->Kp_current > 0.0) tau_s = 1.0 / state->Kp_current;
    if (tau_s < Ts) tau_s = Ts;
    alpha_sched = Ts / (Ts + tau_s);
    state->sched_filter_state = alpha_sched;
    
    double sched_filt = (1.0 - alpha_sched) * state->scheduling_variable
                         + alpha_sched * sched_val;
    state->scheduling_variable = sched_filt;
    
    /* Interpolate gains from schedule */
    pid_gain_set_t interp = gs_table_interpolate_gains(table, sched_filt);
    
    /* Apply gain smoothing to avoid abrupt changes */
    double sm = 0.3;
    state->Kp_current = sm * interp.Kp + (1.0 - sm) * state->Kp_current;
    state->Ki_current = sm * interp.Ki + (1.0 - sm) * state->Ki_current;
    state->Kd_current = sm * interp.Kd + (1.0 - sm) * state->Kd_current;
    state->Ti_current = sm * interp.Ti + (1.0 - sm) * state->Ti_current;
    state->Td_current = sm * interp.Td + (1.0 - sm) * state->Td_current;
    state->N_current  = sm * interp.N  + (1.0 - sm) * state->N_current;
    state->b_current  = sm * interp.b  + (1.0 - sm) * state->b_current;
    state->c_current  = sm * interp.c  + (1.0 - sm) * state->c_current;
    
    /* Detect schedule switch */
    double Kp_diff = state->Kp_current - state->last_Kp_change;
    if (fabs(Kp_diff) > 0.001 * state->Kp_current) {
        state->schedule_switch_count++;
    }
    state->last_Kp_change = state->Kp_current;
    state->last_Ki_change = state->Ki_current;
    state->last_Kd_change = state->Kd_current;
    
    /* Compute 2-DOF error */
    state->setpoint = setpoint;
    state->process_variable = pv;
    double err_p = state->b_current * setpoint - pv;
    (void)(state->c_current * setpoint - pv); /* err_d for 2-DOF; not active in this form */
    state->error = setpoint - pv;
    
    /* Derivative with first-order filter (derivative on PV) */
    double alpha_d = Ts / (Ts + 1.0 / state->N_current);
    if (state->N_current <= 0.0) alpha_d = 1.0;
    state->deriv_filter_state = (1.0 - alpha_d) * state->deriv_filter_state
                                 + alpha_d * (pv - state->prev_pv) / Ts;
    state->derivative = state->deriv_filter_state;
    
    double Kp = state->Kp_current;
    double Ki = state->Ki_current;
    double Kd = state->Kd_current;
    
    double P_term = Kp * err_p;
    double I_term = state->integral + Ki * Ts * state->error;
    double D_term = 0.0;
    
    if (state->Kd_current > 0.0) {
        D_term = Kd * state->derivative;
    }
    
    double u_unsat = P_term + I_term + D_term;
    
    /* Output saturation */
    double u_sat = clamp(u_unsat, state->saturation_low, state->saturation_high);
    
    /* Anti-windup: back-calculation */
    if (u_sat != u_unsat) {
        state->saturated = true;
        double Kb = interp.Kb;
        if (Kb <= 0.0) Kb = 0.3;
        I_term = state->integral;  /* Don't integrate further */
    } else {
        state->saturated = false;
    }
    
    /* Update integrator */
    state->integral = I_term;
    
    /* Store for next cycle */
    state->prev_error = state->error;
    state->prev_pv = pv;
    state->prev_control_output = u_sat;
    state->control_output = u_sat;
    state->prev_sched_variable = state->scheduling_variable;
    state->elapsed_time += Ts;
    state->step_count++;
    
    *output = u_sat;
}

void gs_pid_update_direct(gs_pid_state_t *state,
                           double Kp, double Ki, double Kd,
                           double setpoint, double pv,
                           double *output) {
    if (!state || !output) return;
    
    state->Kp_current = Kp;
    state->Ki_current = Ki;
    state->Kd_current = Kd;
    
    state->setpoint = setpoint;
    state->process_variable = pv;
    state->error = setpoint - pv;
    
    double Ts = state->dt;
    
    /* Derivative on PV with filtering */
    double alpha_d = Ts / (Ts + 0.1);
    state->deriv_filter_state = (1.0 - alpha_d) * state->deriv_filter_state
                                 + alpha_d * (pv - state->prev_pv) / Ts;
    state->derivative = state->deriv_filter_state;
    
    double P_term = Kp * state->error;
    double I_term = state->integral + Ki * Ts * state->error;
    double D_term = Kd * state->derivative;
    
    double u_unsat = P_term + I_term + D_term;
    double u_sat = clamp(u_unsat, state->saturation_low, state->saturation_high);
    
    if (u_sat != u_unsat) {
        state->saturated = true;
    } else {
        state->saturated = false;
        state->integral = I_term;
    }
    
    state->prev_error = state->error;
    state->prev_pv = pv;
    state->prev_control_output = u_sat;
    state->control_output = u_sat;
    state->elapsed_time += Ts;
    state->step_count++;
    
    *output = u_sat;
}

pid_gain_set_t gs_pid_compute_gains(
    const gain_schedule_table_t *table,
    double sched_val) {
    if (!table || table->num_entries == 0) {
        pid_gain_set_t fallback;
        memset(&fallback, 0, sizeof(fallback));
        fallback.Kp = 1.0;
        fallback.Ki = 0.1;
        fallback.Kd = 0.0;
        fallback.N = 10.0;
        fallback.b = 1.0;
        fallback.Kb = 0.3;
        fallback.tracking_time = 1.0;
        return fallback;
    }
    return gs_table_interpolate_gains(table, sched_val);
}

void gs_pid_diagnostics(const gs_pid_state_t *state,
                         char *buffer, size_t bufsize) {
    if (!state || !buffer || bufsize == 0) return;
    snprintf(buffer, bufsize,
             "GS-PID[%s] form=%d Kp=%.4f Ki=%.4f Kd=%.4f "
             "u=%.4f e=%.4f I=%.4f sat=%d switches=%llu "
             "sched=%.4f dt=%.4f steps=%llu t=%.2f",
             state->tag, state->pid_form,
             state->Kp_current, state->Ki_current, state->Kd_current,
             state->control_output, state->error, state->integral,
             state->saturated ? 1 : 0,
             (unsigned long long)state->schedule_switch_count,
             state->scheduling_variable,
             state->dt,
             (unsigned long long)state->step_count,
             state->elapsed_time);
}

double gs_pid_estimate_bandwidth(const gs_pid_state_t *state,
                                  double process_gain,
                                  double process_tau) {
    if (!state || process_gain <= 0.0 || process_tau <= 0.0) return 0.0;
    double Kp = state->Kp_current;
    double open_loop_gain = Kp * process_gain;
    double wb = open_loop_gain / process_tau;
    return wb;
}

uint64_t gs_pid_get_switch_count(const gs_pid_state_t *state) {
    if (!state) return 0;
    return state->schedule_switch_count;
}
