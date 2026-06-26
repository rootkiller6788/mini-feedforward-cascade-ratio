/**
 * @file ratio_controller.c
 * @brief Ratio controller implementation — master-slave tracking,
 *        trim feedback, wild stream feedforward.
 *
 * Level: L2 Core Concepts + L3 Engineering Structures + L5 Algorithms
 *
 * Implements the full ratio controller state machine:
 *   1. Master flow update → slave SP computation
 *   2. Slave flow update → actual ratio computation
 *   3. Ratio trim from quality feedback
 *   4. Ratio error computation
 *   5. Safety monitoring
 *
 * References:
 *   - Shinskey, "Process Control Systems" (1996), Ch.7
 *   - Seborg et al. (2016), Ch.15
 *   - Liptak, "Instrument Engineers' Handbook" (2005), Vol.2
 */

#include "ratio_types.h"
#include "ratio_controller.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Forward declaration from ratio_core.c */
extern double ratio_compute_basic(double F_slave, double F_master);
extern double ratio_clamp(double R, double R_min, double R_max);
extern double ratio_station(double R, double F_master);
extern int ratio_validate_measurement(double flow, int quality);
extern double flow_ewma_filter(double x_raw, double y_prev, double Ts, double tau);

/* =========================================================================
 * L1-L2: Initialization & Configuration
 * ========================================================================= */

/**
 * @brief Initialize ratio control state with defaults.
 *
 * Sets up the ratio controller with safe initial values:
 *   - Ratio setpoint: as specified
 *   - All measurements: zero
 *   - Mode: FIXED_RATIO (safest startup mode)
 *   - Ratio limits: wide open (0.01 to 100.0)
 *
 * The operator must explicitly select WILD_STREAM or TRIMMED
 * mode after verifying flow measurements are valid.
 */
void ratio_control_init(ratio_control_state_t *rc, double R_sp,
                         int master_is_gas, int slave_is_gas)
{
    if (rc == NULL) return;

    memset(rc, 0, sizeof(*rc));

    rc->config.ratio_setpoint = R_sp > 0.0 ? R_sp : 1.0;
    rc->config.ratio_min      = 0.01;
    rc->config.ratio_max      = 100.0;
    rc->config.ratio_deadband = 0.01;
    rc->config.master_is_gas  = master_is_gas ? 1 : 0;
    rc->config.slave_is_gas   = slave_is_gas ? 1 : 0;
    rc->config.density_master = 1.0;
    rc->config.density_slave  = 1000.0; /* typical liquid density */
    rc->config.master_unit    = FLOW_UNIT_MASS;
    rc->config.slave_unit     = FLOW_UNIT_MASS;

    rc->mode           = RATIO_MODE_FIXED;
    rc->actual_ratio   = 0.0;
    rc->ratio_error    = 0.0;
    rc->slave_setpoint = 0.0;
    rc->ratio_trim     = 0.0;
    rc->trimmed_ratio  = R_sp;
    rc->ratio_valid    = 0;
    rc->slave_saturated = 0;
    rc->windup_active  = 0;
}

/**
 * @brief Set the ratio control mode.
 */
void ratio_set_mode(ratio_control_state_t *rc, ratio_mode_t mode)
{
    if (rc == NULL) return;
    rc->mode = mode;
}

/**
 * @brief Configure ratio safety limits.
 */
void ratio_set_limits(ratio_control_state_t *rc, double R_min, double R_max)
{
    if (rc == NULL) return;
    if (R_min > 0.0 && R_max > R_min) {
        rc->config.ratio_min = R_min;
        rc->config.ratio_max = R_max;
    }
}

/**
 * @brief Set flow measurement units and densities for ratio conversion.
 *
 * When master and slave are measured in different units (e.g., gas
 * in Nm³/h, liquid in kg/h), density values enable conversion to
 * a common mass-flow basis for ratio computation.
 */
void ratio_set_flow_units(ratio_control_state_t *rc,
                           flow_unit_t master_unit, flow_unit_t slave_unit,
                           double density_master, double density_slave)
{
    if (rc == NULL) return;
    rc->config.master_unit = master_unit;
    rc->config.slave_unit  = slave_unit;
    if (density_master > 0.0) rc->config.density_master = density_master;
    if (density_slave > 0.0)  rc->config.density_slave  = density_slave;
}

/* =========================================================================
 * L3: Runtime Measurement Update
 * ========================================================================= */

/**
 * @brief Update master (wild) flow measurement.
 *
 * The master flow is typically the uncontrolled stream
 * (e.g., production rate, fuel gas supply pressure-driven flow).
 *
 * When the master flow changes, the slave setpoint must be
 * recalculated to maintain the desired ratio.
 */
void ratio_update_master(ratio_control_state_t *rc, double flow, int quality)
{
    if (rc == NULL) return;

    rc->master.raw_flow  = flow;
    rc->master.signal_quality = quality;

    if (ratio_validate_measurement(flow, quality)) {
        /* Apply first-order filter to reduce noise */
        double prev_filt = rc->master.filtered_flow;
        double Ts = 0.5; /* default 500ms sample period */
        double tau = 2.0; /* 2-second filter time constant */
        rc->master.filtered_flow = flow_ewma_filter(flow, prev_filt, Ts, tau);

        /* Track rate of change for feedforward */
        if (rc->master.timestamp_ms > 0) {
            double dt = 0.001; /* approximate 1 second for rate calc */
            rc->master.flow_rate_of_change =
                (rc->master.filtered_flow - prev_filt) / dt;
        }
        rc->ratio_valid = 1;
    } else {
        rc->master.filtered_flow = 0.0;
        rc->master.flow_rate_of_change = 0.0;
        rc->ratio_valid = 0;
    }
}

/**
 * @brief Update slave flow measurement.
 *
 * The slave flow is the controlled stream. Its current value
 * is used to compute the actual ratio for feedback trim.
 */
void ratio_update_slave(ratio_control_state_t *rc, double flow, int quality)
{
    if (rc == NULL) return;

    rc->slave.raw_flow  = flow;
    rc->slave.signal_quality = quality;

    if (ratio_validate_measurement(flow, quality)) {
        rc->slave.filtered_flow = flow;
    } else {
        rc->slave.filtered_flow = 0.0;
        rc->ratio_valid = 0;
    }
}

/* =========================================================================
 * L2: Actual Ratio Computation
 * ========================================================================= */

/**
 * @brief Compute the measured (actual) ratio.
 *
 * R_actual = F_slave_filtered / F_master_filtered
 *
 * Returns 0.0 if either flow measurement is invalid.
 *
 * The actual ratio is compared against the effective ratio
 * (R_sp + R_trim) to compute the ratio error for trim control.
 */
double ratio_compute_actual(const ratio_control_state_t *rc)
{
    if (rc == NULL) return 0.0;
    if (!rc->ratio_valid) return 0.0;

    return ratio_compute_basic(rc->slave.filtered_flow,
                                rc->master.filtered_flow);
}

/**
 * @brief Compute the slave flow setpoint.
 *
 * Core ratio equation:
 *   SP_slave = R_effective * F_master
 *
 * where R_effective = clamp(R_sp + R_trim, R_min, R_max).
 *
 * The slave flow controller (typically a fast PID) then
 * adjusts the slave control valve to make F_slave = SP_slave.
 */
double ratio_compute_slave_setpoint(ratio_control_state_t *rc)
{
    if (rc == NULL) return 0.0;
    if (!rc->ratio_valid) return 0.0;

    /* Compute effective ratio with trim and clamping */
    double R_raw = rc->config.ratio_setpoint + rc->ratio_trim;
    double R_eff = ratio_clamp(R_raw,
                                rc->config.ratio_min,
                                rc->config.ratio_max);

    rc->trimmed_ratio = R_eff;

    /* Detect saturation of ratio trim */
    if (R_raw != R_eff) {
        rc->windup_active = 1;
    } else {
        rc->windup_active = 0;
    }

    /* Compute slave setpoint */
    rc->slave_setpoint = ratio_station(R_eff, rc->master.filtered_flow);
    return rc->slave_setpoint;
}

/**
 * @brief Compute ratio error.
 *
 * e_ratio = R_actual - R_effective
 *
 * This error is fed to the ratio trim controller which
 * adjusts R_trim to drive the error toward zero.
 */
double ratio_compute_error(ratio_control_state_t *rc)
{
    if (rc == NULL) return 0.0;

    double R_actual = ratio_compute_actual(rc);

    rc->actual_ratio = R_actual;
    rc->ratio_error  = R_actual - rc->trimmed_ratio;

    return rc->ratio_error;
}

/* =========================================================================
 * L5: Ratio Trim Controller
 * ========================================================================= */

/**
 * @brief Initialize the ratio trim controller.
 *
 * The trim controller is a slow PI loop that adjusts the
 * ratio setpoint based on quality or composition feedback.
 *
 * It should be tuned 5-10× slower than the slave flow controller
 * to avoid interaction:
 *   - Inner loop (flow): 1-5 second response
 *   - Outer loop (trim): 30-300 second response
 *
 * This time-scale separation is essential for cascade stability.
 */
void ratio_trim_init(ratio_trim_controller_t *trim,
                     double Kp, double Ti, double Ts)
{
    if (trim == NULL) return;

    trim->Kp_trim   = Kp > 0.0 ? Kp : 0.1;
    trim->Ti_trim   = Ti;
    trim->Ts        = Ts;
    trim->trim_min  = -0.3;  /* default: ±30% trim authority */
    trim->trim_max  =  0.3;
    trim->trim_dz   = 0.001; /* 0.1% dead zone */
    trim->integrator = 0.0;
    trim->prev_error = 0.0;
    trim->output     = 0.0;
    trim->saturated  = 0;
}

/**
 * @brief Set trim output limits.
 */
void ratio_trim_set_limits(ratio_trim_controller_t *trim,
                            double trim_min, double trim_max)
{
    if (trim == NULL) return;
    if (trim_max > trim_min) {
        trim->trim_min = trim_min;
        trim->trim_max = trim_max;
    }
}

/**
 * @brief Execute one ratio trim control step.
 *
 * The trim PI controller:
 *   e(k) = Q_sp - Q_pv
 *   P = Kp * e(k)
 *   I += (Kp * Ts / Ti) * e(k)  [with anti-windup]
 *   output = clamp(P + I, trim_min, trim_max)
 *
 * Dead zone: if |e| < deadband, the integrator is frozen.
 * This prevents small noise from causing continuous trim
 * adjustments that wear out the slave valve.
 *
 * Anti-windup: if output saturates, the integrator is
 * held at its current value (conditional integration).
 */
double ratio_trim_step(ratio_trim_controller_t *trim,
                        double quality_sp, double quality_pv)
{
    if (trim == NULL) return 0.0;

    double error = quality_sp - quality_pv;

    /* Dead zone check */
    if (fabs(error) < trim->trim_dz) {
        /* Freeze trim: small errors should not cause adjustment */
        return trim->output;
    }

    /* Proportional term */
    double P = trim->Kp_trim * error;

    /* Integral term with anti-windup */
    double I = trim->integrator;
    if (trim->Ti_trim > 0.0) {
        /* Compute new integrator value */
        double dI = (trim->Kp_trim * trim->Ts / trim->Ti_trim) * error;

        /* Anti-windup: only integrate if not saturated or
           integration would reduce saturation */
        double proposed = I + dI;
        double proposed_output = P + proposed;

        if (proposed_output > trim->trim_max && error > 0) {
            /* Saturated high, positive error would increase further — freeze I */
            dI = 0.0;
        } else if (proposed_output < trim->trim_min && error < 0) {
            /* Saturated low, negative error would decrease further — freeze I */
            dI = 0.0;
        }

        I += dI;
        trim->integrator = I;
    }

    /* Compute output with clamping */
    double output = P + I;
    if (output > trim->trim_max) {
        output = trim->trim_max;
        trim->saturated = 1;
    } else if (output < trim->trim_min) {
        output = trim->trim_min;
        trim->saturated = 1;
    } else {
        trim->saturated = 0;
    }

    trim->output     = output;
    trim->prev_error = error;

    return output;
}

/**
 * @brief Apply trim correction to the ratio control state.
 */
void ratio_apply_trim(ratio_control_state_t *rc, double R_trim)
{
    if (rc == NULL) return;
    rc->ratio_trim = R_trim;
}

/**
 * @brief Reset trim integrator to zero.
 */
void ratio_trim_reset(ratio_trim_controller_t *trim)
{
    if (trim == NULL) return;
    trim->integrator = 0.0;
    trim->output     = 0.0;
    trim->saturated  = 0;
}

/* =========================================================================
 * L4: Safety & Diagnostics
 * ========================================================================= */

/**
 * @brief Safety check on the current ratio.
 */
int ratio_check_safety(const ratio_control_state_t *rc)
{
    if (rc == NULL) return -1;
    if (!rc->ratio_valid) return -1;

    double R_actual = rc->actual_ratio;

    if (R_actual < rc->config.ratio_min) return 1;  /* ratio too low */
    if (R_actual > rc->config.ratio_max) return 2;  /* ratio too high */
    return 0;  /* safe */
}

/**
 * @brief Check if slave setpoint rate of change exceeds limits.
 */
int ratio_check_rate_limit(const ratio_control_state_t *rc,
                            double max_rate, double *out_rate)
{
    if (rc == NULL || out_rate == NULL) return 0;

    double rate = rc->master.flow_rate_of_change *
                  rc->trimmed_ratio;
    *out_rate = rate;

    if (max_rate > 0.0 && fabs(rate) > max_rate) {
        return 1;
    }
    return 0;
}

/**
 * @brief Generate diagnostic summary.
 */
int ratio_get_diagnostics(const ratio_control_state_t *rc,
                           char *buf, size_t bufsz)
{
    if (rc == NULL || buf == NULL || bufsz == 0) return 0;

    return snprintf(buf, bufsz,
        "Ratio Controller Status:\n"
        "  Mode:            %d\n"
        "  R_setpoint:      %.4f\n"
        "  R_trim:          %.4f\n"
        "  R_effective:     %.4f\n"
        "  R_actual:        %.4f\n"
        "  Ratio error:     %.4f\n"
        "  Master flow:     %.4f\n"
        "  Slave flow:      %.4f\n"
        "  Slave setpoint:  %.4f\n"
        "  Ratio valid:     %d\n"
        "  Slave saturated: %d\n"
        "  Windup active:   %d\n",
        rc->mode,
        rc->config.ratio_setpoint,
        rc->ratio_trim,
        rc->trimmed_ratio,
        rc->actual_ratio,
        rc->ratio_error,
        rc->master.filtered_flow,
        rc->slave.filtered_flow,
        rc->slave_setpoint,
        rc->ratio_valid,
        rc->slave_saturated,
        rc->windup_active);
}

/* =========================================================================
 * L5: Wild Stream Feedforward
 * ========================================================================= */

/**
 * @brief Compute feedforward correction from master flow change.
 *
 * When the master (wild) flow changes, a feedforward signal
 * can be added directly to the slave flow controller output
 * to provide immediate compensation, bypassing the ratio
 * feedback dynamics.
 *
 *   FF = K_ff * ΔF_master
 *
 * where K_ff = R_sp * (process_gain_inverse)
 *
 * For a simple flow loop:
 *   K_ff = R_sp / valve_gain
 *
 * Feedforward + feedback provides superior disturbance rejection
 * compared to feedback-only ratio control.
 */
double ratio_feedforward_correction(const ratio_control_state_t *rc,
                                     double K_ff, double prev_master)
{
    if (rc == NULL) return 0.0;

    double delta_master = rc->master.filtered_flow - prev_master;
    return K_ff * delta_master;
}

/**
 * @brief Apply low-pass filter to master flow measurements.
 *
 * EWMA filter specifically tuned for ratio control:
 *   α = Ts / (Ts + T_filter)
 *
 * The filter time constant should be:
 *   - Fast enough to track real process changes
 *   - Slow enough to reject flow turbulence noise
 *
 * For gas flow: T_filter = 1-5 seconds
 * For liquid flow: T_filter = 0.5-2 seconds
 */
double ratio_filter_master(double raw, double prev_filt,
                            double Ts, double T_filter)
{
    return flow_ewma_filter(raw, prev_filt, Ts, T_filter);
}

/* =========================================================================
 * L5: Ratio Blending Mode — Basic Implementation
 * ========================================================================= */

/**
 * @brief Compute component flow setpoints for blending ratio control.
 *
 * Given N components with target fractions x_i and total flow SP,
 * each component's flow setpoint is:
 *   F_i_sp = x_i * F_total
 *
 * where sum(x_i) = 1.
 *
 * The ratio controller maintains:
 *   R_i = F_i / F_total = x_i
 *
 * for each component i.
 *
 * @param fractions     Array of N target fractions (sum should be 1.0)
 * @param n_components  Number of components
 * @param total_flow    Total product flow setpoint
 * @param setpoints     [out] Array of N component flow setpoints
 * @return              1 if fractions sum to ~1.0, 0 otherwise
 */
int blending_ratio_setpoints(const double *fractions, int n_components,
                              double total_flow, double *setpoints)
{
    if (fractions == NULL || setpoints == NULL || n_components <= 0)
        return 0;

    /* Validate that fractions sum to approximately 1.0 */
    double sum = 0.0;
    for (int i = 0; i < n_components; i++) {
        if (fractions[i] < 0.0 || fractions[i] > 1.0) return 0;
        sum += fractions[i];
    }

    if (fabs(sum - 1.0) > 0.01) return 0; /* tolerance 1% */

    for (int i = 0; i < n_components; i++) {
        setpoints[i] = fractions[i] * total_flow;
    }

    return 1;
}

/**
 * @brief Validate blending ratio configuration.
 *
 * Checks:
 *   1. All fractions are non-negative
 *   2. Fractions sum to 1.0 (±1% tolerance)
 *   3. Total flow is positive
 *
 * Returns: 1 if valid, 0 if invalid
 */
int blending_ratio_validate(const double *fractions, int n_components,
                             double total_flow)
{
    if (fractions == NULL || n_components < 2) return 0;
    if (total_flow <= 0.0) return 0;

    double sum = 0.0;
    for (int i = 0; i < n_components; i++) {
        if (fractions[i] < 0.0) return 0;
        sum += fractions[i];
    }

    if (fabs(sum - 1.0) > 0.01) return 0;
    return 1;
}

/**
 * @brief Adjust blending fractions to compensate for a component shortfall.
 *
 * If one component's flow cannot meet its target (e.g., supply
 * constraint), the other components must be re-ratioed to maintain
 * total flow while keeping their relative proportions.
 *
 * Algorithm: redistribute the shortfall proportionally among
 * the remaining available components.
 *
 * @param fractions       Target fractions (modified in place)
 * @param n_components    Number of components
 * @param limited_index   Index of the constrained component
 * @param actual_flow     Actual achievable flow of constrained component
 * @param total_flow      Total product flow
 */
void blending_adjust_shortfall(double *fractions, int n_components,
                                int limited_index, double actual_flow,
                                double total_flow)
{
    if (fractions == NULL || n_components < 2) return;
    if (limited_index < 0 || limited_index >= n_components) return;
    if (total_flow <= 0.0) return;

    double limited_fraction = actual_flow / total_flow;
    double shortfall = fractions[limited_index] - limited_fraction;

    if (shortfall <= 0.0) return; /* no shortfall */

    /* Apply the constrained fraction */
    fractions[limited_index] = limited_fraction;

    /* Redistribute shortfall to remaining components proportionally */
    double sum_others = 0.0;
    for (int i = 0; i < n_components; i++) {
        if (i != limited_index) sum_others += fractions[i];
    }

    if (sum_others <= 0.0) return;

    for (int i = 0; i < n_components; i++) {
        if (i != limited_index) {
            fractions[i] += shortfall * (fractions[i] / sum_others);
        }
    }
}
