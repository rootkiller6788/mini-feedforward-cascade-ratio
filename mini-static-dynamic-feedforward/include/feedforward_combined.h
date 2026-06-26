#ifndef FEEDFORWARD_COMBINED_H
#define FEEDFORWARD_COMBINED_H

#include "feedforward_defs.h"

/**
 * @file feedforward_combined.h
 * @brief Combined feedforward-feedback control
 *
 * Knowledge: L2 Core Concepts, L5 Algorithms, L6 Canonical Problems
 *
 * The most common industrial configuration: feedforward + PID feedback.
 *
 * Total control signal:
 *   u_total(t) = u_fb(t) + u_ff(t)
 *
 * Benefits over pure feedback:
 * - Disturbances are rejected before affecting the controlled variable
 * - Feedback only needs to correct residual errors (model mismatch, unmeasured dist.)
 * - Reduced control variance, improved product quality
 *
 * References:
 *   Seborg et al. (2016) §15.5 — Combined feedforward-feedback
 *   Myke King (2016) §8.4 — Feedforward implementation
 */

/* ============================================================================
 * L2: Feedforward controller lifecycle
 * ============================================================================ */

/**
 * @brief Initialize complete feedforward controller
 *
 * Sets all parameters to safe defaults, initializes internal state.
 * Mode defaults to FF_MODE_OFF (must be explicitly enabled).
 *
 * @param ff     Feedforward controller
 */
void feedforward_init(feedforward_t *ff);

/**
 * @brief Configure feedforward for static-only operation
 *
 * @param ff        Feedforward controller
 * @param Kff       Static feedforward gain
 * @param bias      Output bias
 * @param out_min   Output low clamp
 * @param out_max   Output high clamp
 * @param action    Control action direction
 * @param Ts        Sample time [s]
 */
void feedforward_configure_static(feedforward_t *ff, double Kff, double bias,
                                  double out_min, double out_max,
                                  action_t action, double Ts);

/**
 * @brief Configure feedforward for dynamic operation (lead-lag)
 *
 * @param ff        Feedforward controller
 * @param Kff       Static gain (used as base)
 * @param T_lead    Lead time constant [s]
 * @param T_lag     Lag time constant [s]
 * @param bias      Output bias
 * @param out_min   Output low clamp
 * @param out_max   Output high clamp
 * @param action    Control action direction
 * @param Ts        Sample time [s]
 */
void feedforward_configure_dynamic(feedforward_t *ff, double Kff,
                                   double T_lead, double T_lag,
                                   double bias, double out_min, double out_max,
                                   action_t action, double Ts);

/**
 * @brief Configure feedforward for combined static+dynamic mode
 *
 * Static: u_s = Kff_static * d(t) + bias
 * Dynamic: u_d = lead_lag(Kff_dynamic * d(t))
 * Total: u_ff = u_s + u_d (with optional blending factor)
 *
 * @param ff             Feedforward controller
 * @param Kff_static     Static feedforward gain
 * @param Kff_dynamic    Dynamic feedforward gain
 * @param T_lead         Lead time constant [s]
 * @param T_lag          Lag time constant [s]
 * @param bias           Output bias
 * @param blend_factor   Blend between static (0) and dynamic (1): u_ff = blend*u_d + (1-blend)*u_s
 * @param out_min        Output low clamp
 * @param out_max        Output high clamp
 * @param action         Control action direction
 * @param Ts             Sample time [s]
 */
void feedforward_configure_combined(feedforward_t *ff, double Kff_static,
                                    double Kff_dynamic, double T_lead, double T_lag,
                                    double bias, double blend_factor,
                                    double out_min, double out_max,
                                    action_t action, double Ts);

/* ============================================================================
 * L5: Combined feedforward-feedback step execution
 * ============================================================================ */

/**
 * @brief Execute one step of combined static + dynamic feedforward
 *
 * Computes both static and dynamic contributions and combines them.
 *
 * @param ff         Feedforward controller
 * @param d_meas     Current disturbance measurement
 * @return Total feedforward output
 */
double feedforward_step(feedforward_t *ff, double d_meas);

/**
 * @brief Execute feedforward with a feedback (PID) contribution
 *
 * u_combined = u_fb + u_ff_total
 *
 * Also handles anti-windup: if u_combined exceeds limits and ff output
 * is large, the feedback controller should be informed to prevent integral
 * windup (back-calculation method).
 *
 * @param ff         Feedforward controller
 * @param d_meas     Disturbance measurement
 * @param u_fb       Feedback (PID) controller output
 * @param u_combined Output: combined control signal (FB + FF)
 * @return 0 on success, -1 if clamping occurred
 */
int feedforward_step_with_feedback(feedforward_t *ff, double d_meas,
                                   double u_fb, double *u_combined);

/* ============================================================================
 * L5: Mode management and safety
 * ============================================================================ */

/**
 * @brief Enable/disable feedforward mode
 *
 * Smooth transition is essential: when enabling FF, initialize internal
 * state to avoid a bump in the control signal. When disabling, ramp the
 * FF contribution to zero over a configurable number of samples.
 *
 * @param ff         Feedforward controller
 * @param mode       Target mode
 * @param ramp_steps Number of steps to ramp FF contribution
 */
void feedforward_set_mode(feedforward_t *ff, ff_mode_t mode, int ramp_steps);

/**
 * @brief Force feedforward output to a specific value (tracking)
 *
 * Used when the control loop is in manual mode — the feedforward output
 * should track the actual manipulated variable to enable bumpless
 * transfer back to automatic.
 *
 * @param ff            Feedforward controller
 * @param track_value   Value to track
 */
void feedforward_track(feedforward_t *ff, double track_value);

/**
 * @brief Perform bumpless transfer: auto ↔ manual
 *
 * Reconfigures internal state so that switching between auto and manual
 * does not cause a control output bump.
 *
 * In manual: FF tracks actual output, FB tracks actual output minus FF
 * In auto: normal operation resumes from tracked values
 *
 * @param ff         Feedforward controller
 * @param to_auto    1 = switch to auto, 0 = switch to manual
 * @param actual_u   Current actual manipulated variable value
 */
void feedforward_bumpless_transfer(feedforward_t *ff, int to_auto, double actual_u);

/* ============================================================================
 * L6: Performance evaluation
 * ============================================================================ */

/**
 * @brief Initialize performance metrics structure
 *
 * @param perf     Performance structure to initialize
 */
void ff_performance_init(ff_performance_t *perf);

/**
 * @brief Update performance metrics with one sample
 *
 * Incrementally computes variance, peak error, ISE, settling time.
 *
 * @param perf       Performance accumulator
 * @param error      Current control error (SP - PV)
 * @param dt         Time step [s]
 * @param ff_active  1 if feedforward is active, 0 if not
 */
void ff_performance_update(ff_performance_t *perf, double error, double dt, int ff_active);

/**
 * @brief Finalize and compute derived performance metrics
 *
 * Computes variance reduction percentage, ISE reduction, etc.
 * Must be called after all samples have been processed.
 *
 * @param perf       Performance structure
 * @param n_samples  Total number of samples (with FF) used in computation
 */
void ff_performance_finalize(ff_performance_t *perf, int n_samples);

#endif