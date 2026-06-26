/**
 * @file override_valve_pos.c
 * @brief Override/Selector Control — Valve Position Control Implementation
 *
 * Implements Valve Position Control (VPC), a specialized override
 * strategy that maintains the main control valve within its
 * effective operating range by manipulating a secondary valve.
 *
 * Knowledge Coverage:
 *   L2: VPC concept and operating principles
 *   L3: VPC PID implementation, split-range, deadband
 *   L5: VPC auto-tuning, performance metrics
 *   L6: Reactor temperature with VPC (classic problem)
 *
 * Reference:
 *   Shinskey (1996). Process Control Systems, Ch. 9.4.
 *   Luyben (2007). Chemical Reactor Design and Control, Ch. 8.
 */

#include "override_valve_pos.h"
#include <math.h>
#include <string.h>

#ifndef INFINITY
#define INFINITY (1.0/0.0)
#endif

/* =========================================================================
 * L2 — VPC Core Update
 * ========================================================================= */

double vpc_update(vpc_state_t *vpc, double main_valve_pos, double dt) {
    if (vpc == NULL || !vpc->enabled) return 0.0;
    if (dt <= 0.0) dt = vpc->vpc_pid.Ts;
    if (dt <= 0.0) dt = 1.0;

    vpc->main_valve_position = main_valve_pos;

    /* Determine if VPC should be active */
    int should_act = vpc_should_activate(vpc);
    vpc->vpc_active = should_act;

    if (!should_act) {
        /* Main valve is in range — ramp VPC output toward 0 slowly */
        /* Using a simple first-order decay: tau_decay = VPC_Ti * 5 */
        double tau_decay = vpc->vpc_pid.Ti * 5.0;
        if (tau_decay > 0.0 && tau_decay < INFINITY) {
            double alpha = dt / tau_decay;
            vpc->vpc_output = vpc->vpc_output * (1.0 - alpha);
        }
        /* Clamp to 0 minimum */
        if (vpc->vpc_output < 0.0) vpc->vpc_output = 0.0;
        return vpc->vpc_output;
    }

    /* VPC is active — compute PID action */
    double Kc = vpc->vpc_pid.Kc;
    double Ti = vpc->vpc_pid.Ti;
    double Td = vpc->vpc_pid.Td;
    double N  = vpc->vpc_pid.N;

    double error = vpc_error(vpc);

    /* --- P-term --- */
    double p_term = Kc * error;

    /* --- I-term --- */
    if (Ti > 0.0 && Ti < INFINITY) {
        vpc->integral += Kc * (dt / Ti) * error;
    }
    double i_term = vpc->integral;

    /* --- D-term (filtered, on error) --- */
    double d_term = 0.0;
    if (Td > 0.0 && N > 0.0) {
        double a_d = Td / (N * dt + Td);
        double b_d = Kc * Td * N / (N * dt + Td);
        d_term = a_d * vpc->last_output - b_d * (error - vpc->last_error);
    }

    double raw_output = p_term + i_term + d_term;

    /* Clamp to [vpc_pid.u_min, vpc_pid.u_max] */
    if (raw_output < vpc->vpc_pid.u_min) raw_output = vpc->vpc_pid.u_min;
    if (raw_output > vpc->vpc_pid.u_max) raw_output = vpc->vpc_pid.u_max;

    /* Anti-windup: if output saturates, freeze integral */
    if (raw_output <= vpc->vpc_pid.u_min || raw_output >= vpc->vpc_pid.u_max) {
        /* Restore previous integral */
        vpc->integral -= Kc * (dt / Ti) * error;
        /* Ensure integral stays within range */
        if (vpc->integral < vpc->vpc_pid.u_min) vpc->integral = vpc->vpc_pid.u_min;
        if (vpc->integral > vpc->vpc_pid.u_max) vpc->integral = vpc->vpc_pid.u_max;
    }

    vpc->vpc_output = raw_output;
    vpc->last_error = error;
    vpc->last_output = raw_output;

    return vpc->vpc_output;
}

/* =========================================================================
 * L2 — VPC Activation Logic
 * ========================================================================= */

int vpc_should_activate(const vpc_state_t *vpc) {
    if (vpc == NULL || !vpc->enabled) return 0;

    double pos = vpc->main_valve_position;
    double margin = 2.0; /* 2% deadband */

    if (pos > vpc->vpc_max + margin) return 1;
    if (pos < vpc->vpc_min - margin) return 1;

    return 0;
}

double vpc_error(const vpc_state_t *vpc) {
    if (vpc == NULL) return 0.0;

    double pos = vpc->main_valve_position;

    if (pos > vpc->vpc_max) {
        /* Valve too open — positive error to close it */
        return pos - vpc->vpc_setpoint;
    } else if (pos < vpc->vpc_min) {
        /* Valve too closed — negative error to open it */
        return pos - vpc->vpc_setpoint;
    } else {
        /* In range — zero error */
        return 0.0;
    }
}

/* =========================================================================
 * L3 — VPC Tuning Methods
 * ========================================================================= */

void vpc_set_tuning(vpc_state_t *vpc,
                    double Kc, double Ti, double Td, double N) {
    if (vpc == NULL) return;
    vpc->vpc_pid.Kc = Kc;
    vpc->vpc_pid.Ti = Ti;
    vpc->vpc_pid.Td = Td;
    vpc->vpc_pid.N = N;
    vpc->integral = 0.0;
    vpc->last_error = 0.0;
}

void vpc_autotune_from_main(vpc_state_t *vpc,
                            double main_Kc, double main_Ti) {
    if (vpc == NULL) return;

    /* Conservative VPC tuning: 5x slower and 5x less aggressive than main */
    vpc->vpc_pid.Kc = main_Kc * 0.2;
    vpc->vpc_pid.Ti = main_Ti * 5.0;
    vpc->vpc_pid.Td = 0.0;
    vpc->vpc_pid.N = 8.0;
    vpc->integral = 0.0;
    vpc->last_error = 0.0;
}

/* =========================================================================
 * L3 — VPC Anti-Windup and Saturation
 * ========================================================================= */

void vpc_antiwindup(vpc_state_t *vpc) {
    if (vpc == NULL) return;

    if (vpc->vpc_output <= vpc->vpc_pid.u_min &&
        vpc->last_error * vpc->integral < 0.0) {
        /* Saturating at min while integral pushing away from min */
        vpc->integral = vpc->vpc_pid.u_min - vpc->vpc_pid.Kc * vpc->last_error;
    }

    if (vpc->vpc_output >= vpc->vpc_pid.u_max &&
        vpc->last_error * vpc->integral > 0.0) {
        /* Saturating at max while integral pushing away from max */
        vpc->integral = vpc->vpc_pid.u_max - vpc->vpc_pid.Kc * vpc->last_error;
    }
}

int vpc_is_saturated(const vpc_state_t *vpc) {
    if (vpc == NULL) return 0;
    if (vpc->vpc_output >= vpc->vpc_pid.u_max) return 1;
    if (vpc->vpc_output <= vpc->vpc_pid.u_min) return -1;
    return 0;
}

void vpc_set_output_limits(vpc_state_t *vpc,
                           double vpc_out_min, double vpc_out_max) {
    if (vpc == NULL) return;
    if (vpc_out_min >= vpc_out_max) return;
    vpc->vpc_pid.u_min = vpc_out_min;
    vpc->vpc_pid.u_max = vpc_out_max;
}

/* =========================================================================
 * L5 — VPC Split-Range and Advanced Features
 * ========================================================================= */

void vpc_split_range(double vpc_output, double split_point,
                     double *valve_a_open, double *valve_b_open) {
    if (valve_a_open == NULL || valve_b_open == NULL) return;

    if (split_point <= 0.0) split_point = 50.0;
    if (split_point >= 100.0) split_point = 50.0;

    if (vpc_output <= split_point) {
        /* VPC output in Valve A range: 0..split_point → A: 0..100% */
        *valve_a_open = (vpc_output / split_point) * 100.0;
        *valve_b_open = 0.0;
    } else {
        /* VPC output in Valve B range: split_point..100% → B: 0..100% */
        *valve_a_open = 100.0;
        *valve_b_open = ((vpc_output - split_point) / (100.0 - split_point)) * 100.0;
    }

    /* Clamp */
    if (*valve_a_open < 0.0) *valve_a_open = 0.0;
    if (*valve_a_open > 100.0) *valve_a_open = 100.0;
    if (*valve_b_open < 0.0) *valve_b_open = 0.0;
    if (*valve_b_open > 100.0) *valve_b_open = 100.0;
}

double vpc_deadband_filter(double error, double deadband) {
    if (deadband <= 0.0) return error;

    if (fabs(error) < deadband) {
        return 0.0;
    } else if (error > 0.0) {
        return error - deadband;
    } else {
        return error + deadband;
    }
}

double vpc_rate_limit(vpc_state_t *vpc, double raw_output,
                      double rate_limit, double dt) {
    if (vpc == NULL || rate_limit <= 0.0 || dt <= 0.0) return raw_output;

    double max_delta = rate_limit * dt;
    double delta = raw_output - vpc->vpc_output;

    if (delta > max_delta) {
        return vpc->vpc_output + max_delta;
    } else if (delta < -max_delta) {
        return vpc->vpc_output - max_delta;
    } else {
        return raw_output;
    }
}

/* =========================================================================
 * L6 — VPC Process Simulation and Performance
 * ========================================================================= */

double vpc_simulate_main_valve(const vpc_state_t *vpc,
                               double main_valve_pos,
                               double process_gain, double dt) {
    if (vpc == NULL || dt <= 0.0) return main_valve_pos;

    /* First-order model of VPC effect on main valve:
       Increasing VPC output (opening bypass) reduces load on main valve,
       causing it to move toward its setpoint.
       d(pos)/dt = (desired_change - (pos - setpoint)) / tau
       where desired_change = -process_gain * VPC_output
    */

    double tau = 10.0; /* Default time constant [s] */
    double desired_pos = vpc->vpc_setpoint - process_gain * vpc->vpc_output;
    double alpha = dt / tau;

    double new_pos = main_valve_pos * (1.0 - alpha) + desired_pos * alpha;

    /* Simulate valve saturation */
    if (new_pos < 0.0) new_pos = 0.0;
    if (new_pos > 100.0) new_pos = 100.0;

    return new_pos;
}

void vpc_performance(const vpc_state_t *vpc,
                     double *time_in_range,
                     double *avg_deviation,
                     double *max_deviation) {
    if (vpc == NULL) {
        if (time_in_range) *time_in_range = 0.0;
        if (avg_deviation) *avg_deviation = 0.0;
        if (max_deviation) *max_deviation = 0.0;
        return;
    }

    double pos = vpc->main_valve_position;
    double dev = 0.0;

    if (pos > vpc->vpc_max) {
        dev = pos - vpc->vpc_max;
    } else if (pos < vpc->vpc_min) {
        dev = vpc->vpc_min - pos;
    }

    if (time_in_range) *time_in_range = (dev == 0.0) ? 1.0 : 0.0;
    if (avg_deviation) *avg_deviation = dev;
    if (max_deviation) *max_deviation = dev;
}
