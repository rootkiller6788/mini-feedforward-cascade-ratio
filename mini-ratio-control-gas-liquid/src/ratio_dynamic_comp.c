/**
 * @file ratio_dynamic_comp.c
 * @brief Dynamic compensation for ratio control — lead-lag, deadtime,
 *        and feedforward dynamic alignment.
 *
 * Level: L3 Engineering Structures + L5 Algorithms/Methods
 *
 * When the master and slave flow paths have different dynamic
 * responses, the ratio controller must compensate to maintain
 * a consistent ratio during transients.
 *
 * Key dynamic elements:
 *   1. Lead-Lag compensator: aligns master/slave dynamics
 *   2. Deadtime compensation: accounts for transport delays
 *   3. Rate-of-change limiting: protects actuators
 *   4. Filter tuning: noise rejection vs responsiveness
 *
 * References:
 *   - Shinskey, "Process Control Systems" (1996), Ch.7
 *   - Seborg et al. (2016), Ch.15.4 Feedforward Control
 *   - Astrom & Hagglund (1995), Ch.5 Feedforward Design
 */

#include "ratio_types.h"
#include "ratio_controller.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* =========================================================================
 * L3: Lead-Lag Dynamic Compensator
 *
 * Transfer function:
 *   G_comp(s) = K * (T_lead * s + 1) / (T_lag * s + 1)
 *
 * Purpose: When the master flow changes, the slave flow responds
 * with its own dynamics (valve, pipe, sensor). If the slave is
 * slower than the master, the ratio will temporarily deviate.
 * A lead-lag compensator on the slave setpoint can cancel these
 * dynamics.
 *
 * Tustin discretization (bilinear transform):
 *   s = (2/Ts) * (1 - z^-1) / (1 + z^-1)
 *
 * Results in difference equation:
 *   y(k) = a1 * y(k-1) + b0 * x(k) + b1 * x(k-1)
 *
 * where:
 *   a1 = (2*T_lag - Ts) / (2*T_lag + Ts)
 *   b0 = (2*K*T_lead + K*Ts) / (2*T_lag + Ts)
 *   b1 = (-2*K*T_lead + K*Ts) / (2*T_lag + Ts)
 * ========================================================================= */

/**
 * @brief Initialize lead-lag compensator.
 *
 * Computes discrete-time coefficients from continuous-time parameters.
 *
 * Design guidance:
 *   - Lead (T_lead > T_lag): speeds up response (compensates slow slave)
 *   - Lag (T_lag > T_lead): slows down response (compensates fast slave)
 *   - Unity gain (K=1): steady-state ratio preserved
 *
 * Knowledge point: Lead-lag compensation — the standard dynamic
 * compensator in ratio control. Aligns master/slave dynamics to
 * maintain ratio during transients.
 */
void lead_lag_init(lead_lag_compensator_t *comp, double K,
                    double T_lead, double T_lag, double Ts)
{
    if (comp == NULL) return;

    comp->gain   = K;
    comp->T_lead = T_lead >= 0.0 ? T_lead : 0.0;
    comp->T_lag  = T_lag  >= 0.0 ? T_lag  : 0.0;
    comp->Ts     = Ts    >  0.0 ? Ts    : 0.1;

    /* Compute Tustin coefficients */
    double denom = 2.0 * comp->T_lag + comp->Ts;
    if (denom <= 0.0) {
        /* Pure lead (T_lag = 0): use forward Euler approximation */
        comp->a1 = 0.0;
        comp->b0 = K;
        comp->b1 = 0.0;
    } else {
        comp->a1 = (2.0 * comp->T_lag - comp->Ts) / denom;
        comp->b0 = (2.0 * K * T_lead + K * comp->Ts) / denom;
        comp->b1 = (-2.0 * K * T_lead + K * comp->Ts) / denom;
    }

    comp->prev_input  = 0.0;
    comp->prev_output = 0.0;
    comp->initialized = 0;
}

/**
 * @brief Execute one lead-lag compensator step.
 *
 * y(k) = a1 * y(k-1) + b0 * x(k) + b1 * x(k-1)
 *
 * On first call, initializes internal state to avoid
 * a transient jump (bumpless initialization).
 *
 * Knowledge point: Discrete-time lead-lag execution — applies
 * the compensator in real-time. First-call bumpless initialization
 * prevents output spikes at startup.
 */
double lead_lag_step(lead_lag_compensator_t *comp, double input)
{
    if (comp == NULL) return input;

    if (!comp->initialized) {
        /* Bumpless initialization: set internal state so output = input */
        comp->prev_input  = input;
        comp->prev_output = input;
        comp->initialized = 1;
        return input;
    }

    double output = comp->a1 * comp->prev_output
                  + comp->b0 * input
                  + comp->b1 * comp->prev_input;

    comp->prev_input  = input;
    comp->prev_output = output;

    return output;
}

/**
 * @brief Reset lead-lag compensator state.
 *
 * Forces re-initialization on next step. Use after:
 *   - Large process changes
 *   - Mode transitions (manual → auto)
 *   - Sensor replacement
 */
void lead_lag_reset(lead_lag_compensator_t *comp)
{
    if (comp == NULL) return;
    comp->initialized = 0;
    comp->prev_input  = 0.0;
    comp->prev_output = 0.0;
}

/**
 * @brief Compute lead-lag frequency response at a given frequency.
 *
 * G(jω) = K * (jω*T_lead + 1) / (jω*T_lag + 1)
 *
 * Returns magnitude ratio and phase shift:
 *   mag = |G(jω)|
 *   phase = arg(G(jω)) in degrees
 *
 * This is useful for designing the compensator:
 *   - At the dominant process frequency, tune T_lead to cancel
 *     the slave process lag
 *   - Set T_lag to filter high-frequency noise
 *
 * Knowledge point: Compensator frequency response — enables
 * design-time verification of the dynamic compensation.
 */
void lead_lag_frequency_response(double K, double T_lead, double T_lag,
                                  double omega, double *mag, double *phase_deg)
{
    /* G(jω) = K * (jωT_lead + 1) / (jωT_lag + 1) */

    double num_re = K;
    double num_im = K * omega * T_lead;

    double den_re = 1.0;
    double den_im = omega * T_lag;

    /* Magnitude */
    double num_mag = sqrt(num_re * num_re + num_im * num_im);
    double den_mag = sqrt(den_re * den_re + den_im * den_im);

    if (mag != NULL) {
        *mag = (den_mag > 0.0) ? (num_mag / den_mag) : num_mag;
    }

    /* Phase */
    if (phase_deg != NULL) {
        double num_phase = atan2(num_im, num_re);
        double den_phase = atan2(den_im, den_re);
        *phase_deg = (num_phase - den_phase) * 180.0 / 3.141592653589793;
    }
}

/* =========================================================================
 * L5: Feedforward Dynamic Alignment
 *
 * For ratio control with feedforward, the feedforward signal must
 * be dynamically aligned with the feedback path. If the feedforward
 * path is faster than the slave flow dynamics, excess correction
 * causes overshoot. If slower, the feedforward is ineffective.
 *
 * The ideal feedforward compensator:
 *   G_ff(s) = -G_d(s) / G_p(s)
 *
 * where:
 *   G_d(s) = disturbance transfer function (master flow → slave flow)
 *   G_p(s) = process transfer function (slave valve → slave flow)
 *
 * For first-order processes:
 *   G_ff(s) = -K_d * (τ_p * s + 1) / (τ_d * s + 1)
 *
 * This is exactly a lead-lag compensator.
 * ========================================================================= */

/**
 * @brief Design feedforward compensator for ratio control.
 *
 * Given the master-to-slave disturbance model and slave process model,
 * computes the optimal feedforward compensator parameters.
 *
 * For FOPDT (First-Order Plus Dead Time) models:
 *   Slave process:       G_p(s) = K_p * exp(-θ_p*s) / (τ_p*s + 1)
 *   Disturbance (master): G_d(s) = K_d * exp(-θ_d*s) / (τ_d*s + 1)
 *
 * The feedforward compensator:
 *   G_ff(s) = -(K_d/K_p) * (τ_p*s + 1) * exp(-(θ_d - θ_p)*s) / (τ_d*s + 1)
 *
 * This can be implemented as a lead-lag + deadtime.
 * The deadtime term can only be realized if θ_d >= θ_p.
 * If θ_d < θ_p (non-causal), the best we can do is cancel the lag only.
 *
 * Knowledge point: Feedforward design — systematic computation of
 * the optimal dynamic compensator for ratio control. Based on process
 * models (FOPDT), this achieves perfect disturbance rejection in theory.
 */
void feedforward_design(double K_p, double tau_p, double theta_p,
                         double K_d, double tau_d, double theta_d,
                         lead_lag_compensator_t *ff_comp, double Ts)
{
    if (ff_comp == NULL) return;

    if (K_p <= 0.0) {
        /* Invalid process gain — use default unity settings */
        lead_lag_init(ff_comp, 1.0, tau_d, tau_p, Ts);
        return;
    }

    /* Feedforward gain */
    double K_ff = -K_d / K_p;

    /* Lead-lag parameters to cancel process dynamics */
    double T_lead_ff = tau_p;   /* Cancel slave process lag */
    double T_lag_ff  = tau_d;   /* Match disturbance dynamics */

    /* Deadtime compensability check */
    /* If θ_d < θ_p, cannot fully compensate deadtime */
    /* The remaining deadtime θ_d - θ_p is handled by the lead-lag */
    if (theta_d < theta_p) {
        /* Non-causal: process delay > disturbance delay */
        /* Best achievable: compensate lag only, accept residual deadtime */
        T_lead_ff = tau_p;
        T_lag_ff  = tau_d;
    }

    lead_lag_init(ff_comp, fabs(K_ff), T_lead_ff, T_lag_ff, Ts);
}

/* =========================================================================
 * L3: Deadtime Handling in Ratio Control
 *
 * Transport delays (deadtime) in pipe runs cause the slave flow
 * measurement to lag the master flow change. This causes
 * temporary ratio errors during transients.
 *
 * Deadtime compensation approaches:
 *   1. Delay the master flow signal by θ before computing ratio
 *   2. Use Smith predictor structure on the slave flow loop
 *   3. Accept temporary ratio error (if within tolerances)
 * ========================================================================= */

/**
 * @brief Circular buffer deadtime delay.
 *
 * Implements a simple delay line using a circular buffer.
 * The output is the input from N samples ago, where
 * N = deadtime / Ts.
 *
 * This is used to synchronize the master and slave flow signals
 * when the slave measurement has a known transport delay.
 *
 * Knowledge point: Deadtime synchronization — delaying the master
 * signal by the slave's transport delay ensures that ratio
 * computations compare flows at the same "material time".
 *
 * Complexity: O(1) per update
 */
typedef struct {
    double *buffer;     /**< Circular buffer */
    int     size;       /**< Buffer capacity */
    int     head;       /**< Write index */
    int     filled;     /**< Number of samples stored (until full) */
} deadtime_delay_t;

int deadtime_delay_init(deadtime_delay_t *delay, int buffer_size)
{
    if (delay == NULL || buffer_size <= 0) return 0;

    delay->buffer = (double *)malloc((size_t)buffer_size * sizeof(double));
    if (delay->buffer == NULL) return 0;

    delay->size   = buffer_size;
    delay->head   = 0;
    delay->filled = 0;

    /* Initialize buffer to zero */
    for (int i = 0; i < buffer_size; i++) {
        delay->buffer[i] = 0.0;
    }

    return 1;
}

/**
 * @brief Push new sample, get delayed sample.
 *
 * Returns the sample from N steps ago, where N = buffer_size - 1.
 * If the buffer is not yet full, returns 0.0 (no valid delayed value).
 */
double deadtime_delay_step(deadtime_delay_t *delay, double input)
{
    if (delay == NULL || delay->buffer == NULL) return input;

    /* Write current input at head */
    delay->buffer[delay->head] = input;

    /* Read the oldest sample (next position after head) */
    int read_idx = delay->head + 1;
    if (read_idx >= delay->size) read_idx = 0;

    double delayed = delay->buffer[read_idx];

    /* Advance head */
    delay->head = read_idx;

    /* Track filling */
    if (delay->filled < delay->size) {
        delay->filled++;
    }

    /* If buffer not yet full, return 0 (no valid delayed value) */
    if (delay->filled < delay->size) {
        return 0.0;
    }

    return delayed;
}

void deadtime_delay_free(deadtime_delay_t *delay)
{
    if (delay == NULL) return;
    if (delay->buffer != NULL) {
        free(delay->buffer);
        delay->buffer = NULL;
    }
    delay->size   = 0;
    delay->head   = 0;
    delay->filled = 0;
}

/* =========================================================================
 * L5: Rate-of-Change Limiting for Slave Setpoint
 *
 * Actuator protection: large step changes in slave setpoint
 * cause valve/pump wear and process upset. Rate limiting
 * smooths setpoint changes.
 *
 * Rate limiter:
 *   if |ΔSP| > rate_max * Ts:
 *     SP(k) = SP(k-1) + sign(ΔSP) * rate_max * Ts
 *   else:
 *     SP(k) = SP_desired
 * ========================================================================= */

/**
 * @brief Apply rate-of-change limiting to a setpoint.
 *
 * Limits the rate at which a setpoint can change:
 *   rate = (SP_desired - SP_current) / Ts
 *   if |rate| > rate_max: clamp
 *
 * This protects downstream actuators and prevents process
 * upsets from sudden ratio changes.
 *
 * Knowledge point: Rate limiting — essential actuator protection
 * in ratio control. Large valves and VFDs cannot respond instantly;
 * rate limiting prevents windup and mechanical stress.
 */
double rate_limit_apply(double SP_desired, double SP_current,
                         double rate_max, double Ts)
{
    if (Ts <= 0.0) return SP_desired;

    double delta = SP_desired - SP_current;
    double max_delta = rate_max * Ts;

    if (fabs(delta) <= max_delta) {
        return SP_desired;
    }

    if (delta > 0.0) {
        return SP_current + max_delta;
    } else {
        return SP_current - max_delta;
    }
}

/**
 * @brief Compute rate-limited ramp trajectory.
 *
 * When the ratio setpoint changes, the slave setpoint ramps
 * linearly from its current value to the target over a
 * configurable ramp time.
 *
 *   SP(k) = SP_start + (SP_target - SP_start) * min(t_elapsed/ramp_time, 1)
 *
 * This is used for "bumpless" ratio setpoint changes in blending
 * and combustion control.
 *
 * Knowledge point: Setpoint ramping — smooth ratio transitions
 * implemented as linear ramps. The ramp time is a tuning parameter:
 * too fast → process upset; too slow → off-spec production extended.
 */
double ratio_ramp_compute(double SP_start, double SP_target,
                           double t_elapsed, double ramp_time)
{
    if (ramp_time <= 0.0) return SP_target;

    double frac = t_elapsed / ramp_time;
    if (frac >= 1.0) return SP_target;
    if (frac <= 0.0) return SP_start;

    return SP_start + (SP_target - SP_start) * frac;
}

/* =========================================================================
 * L5: Noise Rejection — Tunable Filters
 * ========================================================================= */

/**
 * @brief Second-order Butterworth low-pass filter for flow signals.
 *
 * The Butterworth filter provides a maximally flat passband
 * and a sharper rolloff than first-order EWMA.
 *
 * Discrete-time implementation (bilinear transform of 2nd order):
 *   y(k) = b0*x(k) + b1*x(k-1) + b2*x(k-2) - a1*y(k-1) - a2*y(k-2)
 *
 * For fs = 1/Ts and cutoff fc:
 *   ωc = 2π * fc
 *   pre-warped: ωc' = (2/Ts) * tan(ωc*Ts/2)
 *
 * Knowledge point: Butterworth filtering — provides superior noise
 * rejection for ratio control compared to simple EWMA, at the cost
 * of slightly more computation. Important when gas flow turbulence
 * creates high-frequency noise.
 */
typedef struct {
    double b0, b1, b2;    /**< Numerator coefficients */
    double a1, a2;        /**< Denominator coefficients */
    double x1, x2;        /**< Input history */
    double y1, y2;        /**< Output history */
    int    initialized;
} butterworth_filter_t;

void butterworth_init(butterworth_filter_t *filt, double fc, double Ts)
{
    if (filt == NULL) return;

    /* Pre-warped cutoff frequency */
    double omega = 2.0 * 3.141592653589793 * fc;
    double omega_w = (2.0 / Ts) * tan(omega * Ts / 2.0);

    /* Bilinear transform coefficients for 2nd-order low-pass */
    double C = 1.0 / tan(omega_w * Ts / 2.0); /* = omega_w_c */

    /* For normalized Butterworth (Q = 1/sqrt(2)): */
    double sqrt2 = 1.414213562373095;

    double denom = 1.0 + sqrt2 * C + C * C;

    filt->b0 = 1.0 / denom;
    filt->b1 = 2.0 / denom;
    filt->b2 = 1.0 / denom;
    filt->a1 = (2.0 - 2.0 * C * C) / denom;
    filt->a2 = (1.0 - sqrt2 * C + C * C) / denom;

    filt->x1 = 0.0; filt->x2 = 0.0;
    filt->y1 = 0.0; filt->y2 = 0.0;
    filt->initialized = 0;
}

double butterworth_step(butterworth_filter_t *filt, double input)
{
    if (filt == NULL) return input;

    if (!filt->initialized) {
        /* Bumpless initialization */
        filt->x1 = input; filt->x2 = input;
        filt->y1 = input; filt->y2 = input;
        filt->initialized = 1;
        return input;
    }

    double output = filt->b0 * input
                  + filt->b1 * filt->x1
                  + filt->b2 * filt->x2
                  - filt->a1 * filt->y1
                  - filt->a2 * filt->y2;

    /* Update history */
    filt->x2 = filt->x1;
    filt->x1 = input;
    filt->y2 = filt->y1;
    filt->y1 = output;

    return output;
}
