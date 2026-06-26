/**
 * @file ratio_controller.h
 * @brief Ratio controller — master-slave ratio tracking with trim feedback.
 *
 * Level: L2 Core Concepts + L5 Algorithms/Methods
 * Reference: Shinskey, "Process Control Systems" (1996), Ch.7 Ratio Control
 *            Seborg, Edgar, Mellichamp, "Process Dynamics and Control" (2016), Ch.15.3
 *            Liptak, "Instrument Engineers' Handbook" (2005), Vol.2, Sec.2.10
 *
 * Course mapping:
 *   Stanford ENGR205: Process Control — ratio & feedforward
 *   Berkeley ME233: Advanced Control — feedforward compensation
 *   Purdue ME 575: Industrial Control — master-slave flow ratio
 *   Tsinghua: Process Control Engineering — ratio control design
 *
 * The ratio controller maintains a desired ratio between a master (wild)
 * stream and a slave (controlled) stream. The core equation:
 *
 *   SP_slave = (R_sp + R_trim) * F_master
 *
 * where R_trim comes from a quality/composition feedback loop.
 */

#ifndef RATIO_CONTROLLER_H
#define RATIO_CONTROLLER_H

#include "ratio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * L1-L2: Initialization & Configuration
 *===========================================================================*/

/**
 * @brief Initialize a ratio control state.
 *
 * Sets default ratio setpoint and configuration, initializes all
 * measurement values to zero, and sets mode to FIXED_RATIO.
 *
 * @param rc         Pointer to uninitialized ratio control state
 * @param R_sp       Desired ratio setpoint (dimensionless, > 0)
 * @param master_is_gas  1 if master stream is gas, 0 if liquid
 * @param slave_is_gas   1 if slave stream is gas, 0 if liquid
 *
 * Complexity: O(1)
 */
void ratio_control_init(ratio_control_state_t *rc, double R_sp,
                         int master_is_gas, int slave_is_gas);

/**
 * @brief Set the ratio control mode.
 *
 * See ratio_mode_t for available modes:
 *   FIXED_RATIO → open-loop ratio
 *   WILD_STREAM → feedforward slave SP = R_sp * F_master
 *   TRIMMED     → ratio with quality feedback trim
 *   CROSS_LIMITED → combustion cross-limiting safety
 *   BLENDING    → multi-component blending
 *   CASCADE     → cascade ratio (outer loop adjusts R)
 *
 * @param rc    Ratio control state
 * @param mode  Desired operating mode
 */
void ratio_set_mode(ratio_control_state_t *rc, ratio_mode_t mode);

/**
 * @brief Configure ratio limits for safety.
 *
 * Ratio limits prevent the slave flow from going outside safe bounds:
 *   R_min ≤ R_effective ≤ R_max
 *
 * For combustion systems:
 *   R_min prevents excessively lean (flameout) conditions
 *   R_max prevents excessively rich (CO hazard) conditions
 *
 * @param rc       Ratio control state
 * @param R_min    Minimum safe ratio
 * @param R_max    Maximum safe ratio
 */
void ratio_set_limits(ratio_control_state_t *rc, double R_min, double R_max);

/**
 * @brief Set master and slave flow measurement units and densities.
 *
 * Density values are required when master and slave are in different
 * units (e.g., master is gas in Nm³/h, slave is liquid in kg/h).
 * The ratio controller converts to consistent units internally.
 *
 * @param rc              Ratio control state
 * @param master_unit     Master flow unit type
 * @param slave_unit      Slave flow unit type
 * @param density_master  Master stream density (kg/m³) at flow conditions
 * @param density_slave   Slave stream density (kg/m³) at flow conditions
 */
void ratio_set_flow_units(ratio_control_state_t *rc,
                           flow_unit_t master_unit, flow_unit_t slave_unit,
                           double density_master, double density_slave);

/*===========================================================================
 * L3: Runtime Ratio Computation
 *===========================================================================*/

/**
 * @brief Update master flow measurement.
 *
 * Stores master flow with timestamp and signal quality.
 * Triggers slave setpoint recalculation if in wild-stream mode.
 *
 * @param rc          Ratio control state
 * @param flow        Measured master flow (engineering units)
 * @param quality     Signal quality: 0=bad, 1=uncertain, 2=good
 *
 * Complexity: O(1)
 */
void ratio_update_master(ratio_control_state_t *rc, double flow, int quality);

/**
 * @brief Update slave flow measurement.
 *
 * Stores slave flow with timestamp. If signal quality is bad,
 * the ratio computation is invalidated (ratio_valid = 0).
 *
 * @param rc          Ratio control state
 * @param flow        Measured slave flow (engineering units)
 * @param quality     Signal quality: 0=bad, 1=uncertain, 2=good
 *
 * Complexity: O(1)
 */
void ratio_update_slave(ratio_control_state_t *rc, double flow, int quality);

/**
 * @brief Compute the current actual ratio.
 *
 * Ratio definition:
 *   R_actual = F_slave / F_master
 *
 * Valid only when both flows are > 0 and signal quality >= 1.
 * Returns 0.0 if ratio is invalid.
 *
 * Complexity: O(1)
 */
double ratio_compute_actual(const ratio_control_state_t *rc);

/**
 * @brief Compute the slave flow setpoint for ratio control.
 *
 * Core ratio equation:
 *   SP_slave = R_effective * F_master
 *
 * where R_effective = clamp(R_sp + R_trim, R_min, R_max)
 *
 * If R_sp + R_trim exceeds R_min/R_max, the effective ratio is clamped
 * and a saturation flag is set.
 *
 * @param rc          Ratio control state (updated with slave_setpoint)
 * @return            Computed slave flow setpoint
 *
 * Complexity: O(1)
 */
double ratio_compute_slave_setpoint(ratio_control_state_t *rc);

/**
 * @brief Compute ratio error for feedback/trim control.
 *
 * Ratio error:
 *   e_ratio = R_actual - R_effective
 *
 * A positive error means slave flow is higher than desired ratio.
 * A negative error means slave flow is lower than desired ratio.
 *
 * This error is used by the ratio trim controller.
 *
 * @param rc  Ratio control state (updated with ratio_error)
 * @return    Ratio error (dimensionless)
 *
 * Complexity: O(1)
 */
double ratio_compute_error(ratio_control_state_t *rc);

/*===========================================================================
 * L5: Ratio Trim & Advanced Control Algorithms
 *===========================================================================*/

/**
 * @brief Initialize a ratio trim PI controller.
 *
 * The trim controller adjusts the ratio based on quality feedback.
 * Typical tuning:
 *   Kp_trim: 0.05-0.5 (small, because ratio changes slowly)
 *   Ti_trim: 60-600 seconds (slow integral, avoids interaction with slave FC)
 *   trim_max: ±0.1 to ±0.3 of R_sp (limited authority)
 *
 * @param trim    Pointer to uninitialized trim controller
 * @param Kp      Proportional gain (> 0)
 * @param Ti      Integral time (seconds, 0 = I-disabled)
 * @param Ts      Sampling period (seconds, typically 1-10s)
 *
 * Complexity: O(1)
 */
void ratio_trim_init(ratio_trim_controller_t *trim,
                     double Kp, double Ti, double Ts);

/**
 * @brief Set trim output limits.
 *
 * Trim authority should be limited:
 *   trim_max = typically ±(0.1 * R_sp) to ±(0.3 * R_sp)
 *
 * Larger trim authority risks slave flow saturation or instability.
 *
 * @param trim      Trim controller
 * @param trim_min  Minimum trim correction (negative allowed)
 * @param trim_max  Maximum trim correction
 */
void ratio_trim_set_limits(ratio_trim_controller_t *trim,
                            double trim_min, double trim_max);

/**
 * @brief Execute one ratio trim control step.
 *
 * Quality-driven trim update:
 *   e_quality(k) = Q_sp - Q_measured
 *   P_trim = Kp_trim * e_quality(k)
 *   I_trim += (Kp_trim * Ts / Ti_trim) * e_quality(k)  [with anti-windup]
 *   R_trim = clamp(P_trim + I_trim, trim_min, trim_max)
 *
 * Dead zone: if |e_quality| < trim_dz, the trim output is frozen
 * to prevent dithering around the setpoint.
 *
 * @param trim        Trim controller state
 * @param quality_sp  Quality setpoint
 * @param quality_pv  Quality measurement (process variable)
 * @return            Current trim output R_trim
 *
 * Complexity: O(1)
 */
double ratio_trim_step(ratio_trim_controller_t *trim,
                        double quality_sp, double quality_pv);

/**
 * @brief Update the ratio trim value in the ratio control state.
 *
 * This function applies the trim controller output to the master
 * ratio controller. The effective ratio becomes:
 *
 *   R_effective = clamp(R_sp + R_trim, R_min, R_max)
 *
 * @param rc      Ratio control state
 * @param R_trim  Trim correction value from trim controller
 */
void ratio_apply_trim(ratio_control_state_t *rc, double R_trim);

/**
 * @brief Reset ratio trim integrator to zero.
 *
 * Used after large disturbances or manual ratio adjustments.
 *
 * @param trim  Trim controller
 */
void ratio_trim_reset(ratio_trim_controller_t *trim);

/*===========================================================================
 * L4: Engineering Laws — Safety Checks & Diagnostics
 *===========================================================================*/

/**
 * @brief Check if the ratio is within safe operating bounds.
 *
 * Returns:
 *   0 → ratio is in safe range [R_min, R_max]
 *   1 → ratio too low (slave insufficient relative to master)
 *   2 → ratio too high (excess slave flow, potential hazard)
 *  -1 → ratio invalid (measurement quality bad)
 *
 * @param rc  Ratio control state
 * @return    Safety status code
 */
int ratio_check_safety(const ratio_control_state_t *rc);

/**
 * @brief Compute the slave flow rate-of-change for actuator protection.
 *
 * Large sudden changes in slave setpoint can stress actuators
 * (valves, pumps, VFDs). This function detects if the rate-of-change
 * exceeds a configurable limit.
 *
 * @param rc             Ratio control state
 * @param max_rate       Maximum allowed rate of change (eng. units/s)
 * @param out_rate       [out] Current rate of change
 * @return               1 if rate limit exceeded, 0 otherwise
 */
int ratio_check_rate_limit(const ratio_control_state_t *rc,
                            double max_rate, double *out_rate);

/**
 * @brief Get the current ratio control status summary.
 *
 * Fills a human-readable diagnostic summary of the ratio controller:
 *   - Effective ratio
 *   - Master flow
 *   - Slave flow
 *   - Ratio error
 *   - Trim correction
 *   - Slave setpoint
 *
 * @param rc     Ratio control state
 * @param buf    Output buffer (must be at least 256 bytes)
 * @param bufsz  Buffer size
 * @return       Number of characters written
 */
int ratio_get_diagnostics(const ratio_control_state_t *rc,
                           char *buf, size_t bufsz);

/*===========================================================================
 * L5: Advanced — Wild Stream Feedforward
 *===========================================================================*/

/**
 * @brief Compute wild stream feedforward correction.
 *
 * When the master flow changes, the slave should respond immediately
 * via feedforward rather than waiting for ratio error feedback.
 *
 * Feedforward signal:
 *   FF = K_ff * (F_master(k) - F_master(k-1))
 *
 * where K_ff = R_sp * G_process_inv (approximate process inverse gain).
 *
 * The feedforward is added to the slave flow controller output:
 *   u_slave_effective = u_slave_PID + FF
 *
 * @param rc           Ratio control state
 * @param K_ff         Feedforward gain (typically ≈ R_sp * slave_gain_inv)
 * @param prev_master  Previous master flow value
 * @return             Feedforward correction signal
 *
 * Complexity: O(1)
 */
double ratio_feedforward_correction(const ratio_control_state_t *rc,
                                     double K_ff, double prev_master);

/**
 * @brief Apply low-pass filter to master flow for noise rejection.
 *
 * First-order exponential filter (EWMA):
 *   F_filtered(k) = α * F_raw(k) + (1-α) * F_filtered(k-1)
 *
 * where α = Ts / (Ts + T_filter) and T_filter is the filter time constant.
 *
 * Using filtered master flow prevents high-frequency noise from propagating
 * through the ratio calculation to the slave setpoint.
 *
 * @param raw        Raw master flow measurement
 * @param prev_filt  Previously filtered value
 * @param Ts         Sampling period (s)
 * @param T_filter   Filter time constant (s, typically 1-5s)
 * @return           Filtered flow value
 *
 * Complexity: O(1)
 */
double ratio_filter_master(double raw, double prev_filt,
                            double Ts, double T_filter);

#ifdef __cplusplus
}
#endif

#endif /* RATIO_CONTROLLER_H */
