#ifndef FEEDFORWARD_DYNAMIC_H
#define FEEDFORWARD_DYNAMIC_H

#include "feedforward_defs.h"

/**
 * @file feedforward_dynamic.h
 * @brief Dynamic feedforward control API — transient compensation
 *
 * Knowledge: L1 Definitions, L2 Core Concepts, L3 Engineering Structures,
 *            L5 Algorithms/Methods
 *
 * Dynamic feedforward compensates for differences in process and
 * disturbance dynamics (time constants, dead times).
 *
 * Perfect dynamic feedforward condition:
 *   Gff(s) = -Gd(s) / Gp(s)
 *
 * For FOPDT systems:
 *   Gff(s) = -(Kd/Kp) * (tau_p*s+1)/(tau_d*s+1) * e^((theta_p - theta_d)*s)
 *         = Kff_static * (lead)/(lag) * dead_time_comp
 *
 * The lead = tau_p, lag = tau_d. Dead time compensation uses:
 * - If theta_p > theta_d: implementable delay
 * - If theta_p < theta_d: prediction required (non-causal, approximate)
 *
 * References:
 *   Seborg et al. (2016) §15.4 — Dynamic feedforward design
 *   Åström & Hägglund (1995) §7.5 — Feedforward with dynamics
 */

/* ============================================================================
 * L5: Lead-Lag implementation (discrete-time difference equation)
 * ============================================================================ */

/**
 * @brief Initialize a lead-lag compensator
 *
 * Sets up the lead-lag for discrete-time execution using
 * bilinear (Tustin) discretization or backward Euler depending on configuration.
 *
 * Continuous: G(s) = K * (T_lead*s + 1) / (T_lag*s + 1)
 *
 * Bilinear (Tustin) discrete form:
 *   y[k] = (a1*y[k-1] + b0*x[k] + b1*x[k-1]) / a0
 * where coefficients are computed during init.
 *
 * @param ll          Lead-lag structure to initialize
 * @param K           Lead-lag gain
 * @param T_lead      Lead (zero) time constant [s]
 * @param T_lag       Lag (pole) time constant [s]
 * @param Ts          Sample time [s]
 */
void lead_lag_init(lead_lag_t *ll, double K, double T_lead, double T_lag, double Ts);

/**
 * @brief Execute one step of lead-lag compensation
 *
 * Uses bilinear (Tustin) transformation for robustness.
 *
 * Tustin: s ~ (2/Ts)*(z-1)/(z+1)
 * => G(z) = K * (2*T_lead*(z-1)/(z+1) + Ts) / (2*T_lag*(z-1)/(z+1) + Ts)
 *
 * @param ll     Lead-lag compensator
 * @param x      Current input sample
 * @return Compensated output
 */
double lead_lag_step(lead_lag_t *ll, double x);

/**
 * @brief Reset lead-lag internal state
 *
 * Clears history buffers. Useful after large disturbances or mode changes.
 *
 * @param ll     Lead-lag compensator to reset
 */
void lead_lag_reset(lead_lag_t *ll);

/**
 * @brief Compute lead-lag frequency response at a given angular frequency
 *
 * |G(jw)| = K * sqrt((w*T_lead)^2 + 1) / sqrt((w*T_lag)^2 + 1)
 * phase  = atan(w*T_lead) - atan(w*T_lag)
 *
 * @param ll         Lead-lag compensator
 * @param omega      Angular frequency [rad/s]
 * @param magnitude  Output: magnitude |G(jw)|
 * @param phase      Output: phase angle [rad]
 */
void lead_lag_freq_response(const lead_lag_t *ll, double omega,
                            double *magnitude, double *phase);

/* ============================================================================
 * L5: Second-order lead-lag
 * ============================================================================ */

/**
 * @brief Initialize second-order lead-lag compensator
 *
 * G(s) = K * (Tn^2*s^2 + 2*zeta_n*Tn*s + 1) / (Td^2*s^2 + 2*zeta_d*Td*s + 1)
 *
 * @param ll2         Second-order lead-lag structure
 * @param K           Gain
 * @param Tn          Numerator natural period [s]
 * @param Td          Denominator natural period [s]
 * @param zeta_n      Numerator damping ratio
 * @param zeta_d      Denominator damping ratio
 * @param Ts          Sample time [s]
 */
void lead_lag2_init(lead_lag2_t *ll2, double K, double Tn, double Td,
                    double zeta_n, double zeta_d, double Ts);

/**
 * @brief Execute one step of second-order lead-lag
 *
 * Uses bilinear discretization for second-order transfer function.
 *
 * @param ll2    Second-order lead-lag
 * @param x      Current input
 * @return Compensated output
 */
double lead_lag2_step(lead_lag2_t *ll2, double x);

/**
 * @brief Reset second-order lead-lag state
 *
 * @param ll2    Second-order lead-lag to reset
 */
void lead_lag2_reset(lead_lag2_t *ll2);

/* ============================================================================
 * L3: Discrete-time transfer function implementation
 * ============================================================================ */

/**
 * @brief Initialize discrete transfer function from coefficients
 *
 * G(z) = (b0 + b1*z^(-1) + ... + bn*z^(-n)) / (1 + a1*z^(-1) + ... + am*z^(-m))
 *
 * @param dtf        Discrete transfer function structure
 * @param num        Numerator coefficients [b0..bn]
 * @param den        Denominator coefficients [den[0]=1, a1..am]
 * @param n          Numerator order
 * @param m          Denominator order
 * @param Ts         Sample time [s]
 */
void dtf_init(tf_discrete_t *dtf, const double *num, const double *den,
              int n, int m, double Ts);

/**
 * @brief Execute one step of discrete transfer function
 *
 * y[k] = b0*x[k] + b1*x[k-1] + ... + bn*x[k-n]
 *        - a1*y[k-1] - a2*y[k-2] - ... - am*y[k-m]
 *
 * Maintains circular buffers for x and y histories.
 *
 * @param dtf    Discrete transfer function
 * @param x      Current input sample
 * @return Filtered/compensated output
 */
double dtf_step(tf_discrete_t *dtf, double x);

/**
 * @brief Reset discrete transfer function state
 */
void dtf_reset(tf_discrete_t *dtf);

/* ============================================================================
 * L5: Dynamic feedforward design methods
 * ============================================================================ */

/**
 * @brief Design dynamic feedforward for FOPDT process + FOPDT disturbance
 *
 * Computes the lead-lag parameters for perfect dynamic feedforward:
 *   T_lead = tau_p      (process time constant)
 *   T_lag  = tau_d      (disturbance time constant)
 *   K_ll   = -Kd/Kp     (static gain ratio)
 *
 * Dead time mismatch handling:
 *   dtheta = theta_d - theta_p
 *   If dtheta > 0: disturbance delay < process delay → lead-lag alone works
 *   If dtheta < 0: disturbance delay > process delay → extra delay needed
 *   If dtheta ~ 0: perfect dead time match
 *
 * @param process    Process model (FOPDT)
 * @param dist       Disturbance model (FOPDT)
 * @param action     Controller action direction
 * @param T_lead     Output: lead time constant [s]
 * @param T_lag      Output: lag time constant [s]
 * @param K_ll       Output: lead-lag gain
 * @return 0 on success, -1 if invalid parameters
 */
int ff_dynamic_design_fopdt(const fopdt_t *process, const dist_model_t *dist,
                            action_t action, double *T_lead, double *T_lag,
                            double *K_ll);

/**
 * @brief Design dynamic feedforward with SOPDT process model
 *
 * For SOPDT, the dynamic feedforward becomes a second-order compensator:
 *   Gff(s) = -(Kd/Kp) * ((tau1*s+1)*(tau2*s+1))/(tau_d*s+1) * e^((theta_p-theta_d)*s)
 *
 * This can be approximated as: second-order lead + first-order lag.
 *
 * @param process    SOPDT process model
 * @param dist       Disturbance model
 * @param action     Controller action direction
 * @param Tn         Output: second-order numerator natural period [s]
 * @param Td         Output: denominator natural period [s]
 * @param zeta_n     Output: numerator damping
 * @param zeta_d     Output: denominator damping
 * @param K          Output: gain
 * @return 0 on success, -1 if invalid
 */
int ff_dynamic_design_sopdt(const sopdt_t *process, const dist_model_t *dist,
                            action_t action, double *Tn, double *Td,
                            double *zeta_n, double *zeta_d, double *K);

/**
 * @brief Design feedforward from arbitrary transfer functions
 *
 * Gff(s) = -Gd(s) / Gp(s)
 *
 * Computed via polynomial division for the case where both are rational.
 * For non-minimum-phase Gp, the inverse may be unstable — this function
 * detects that case and falls back to static-only feedforward.
 *
 * @param Gp         Process transfer function
 * @param Gd         Disturbance transfer function
 * @param Gff_out    Output: designed feedforward transfer function
 * @return 0 on success, -1 if Gp is non-minimum-phase (unstable inverse)
 */
int ff_dynamic_design_tf(const tf_t *Gp, const tf_t *Gd, tf_t *Gff_out);

/**
 * @brief Initialize dynamic feedforward in a feedforward controller
 *
 * Configures the lead-lag component of the feedforward controller
 * based on designed dynamic parameters.
 *
 * @param ff         Feedforward controller
 * @param T_lead     Lead time constant [s]
 * @param T_lag      Lag time constant [s]
 * @param K_ll       Lead-lag gain
 * @param Ts         Sample time [s]
 */
void ff_dynamic_init(feedforward_t *ff, double T_lead, double T_lag,
                     double K_ll, double Ts);

/**
 * @brief Execute one step of dynamic feedforward
 *
 * Applies the lead-lag dynamic compensation to the disturbance measurement.
 *
 * @param ff         Feedforward controller
 * @param d_meas     Current disturbance measurement
 * @return Dynamic feedforward output
 */
double ff_dynamic_step(feedforward_t *ff, double d_meas);

/**
 * @brief Check if dynamic feedforward is implementable (causality check)
 *
 * Dynamic feedforward requires: theta_d >= theta_p  (disturbance delay <= process delay)
 * If disturbance reaches PV faster than the manipulated variable can affect PV,
 * perfect dynamic feedforward is non-causal.
 *
 * @param theta_p    Process dead time [s]
 * @param theta_d    Disturbance dead time [s]
 * @return 1 if implementable (causal), 0 if non-causal
 */
int ff_dynamic_is_causal(double theta_p, double theta_d);

/**
 * @brief Compute required additional delay for non-causal case
 *
 * If disturbance is faster than process (theta_d < theta_p),
 * the feedforward action should be delayed by dtheta = theta_p - theta_d
 * to synchronize the compensation with the disturbance effect.
 *
 * @param theta_p    Process dead time [s]
 * @param theta_d    Disturbance dead time [s]
 * @return Required delay [s] (>= 0)
 */
double ff_dynamic_required_delay(double theta_p, double theta_d);

#endif