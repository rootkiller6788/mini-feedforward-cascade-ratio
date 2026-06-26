/**
 * @file feedforward_control.h
 * @brief Feedforward Control — Compensator Design & FF+FB Combination
 *
 * Module: mini-cascade-control-primary-secondary
 * Knowledge Coverage: L1 Definitions, L2 Core Concepts, L3 Engineering Structures, L5 Algorithms
 *
 * Feedforward control anticipates disturbances and applies corrective action
 * before the disturbance affects the process variable. Unlike feedback, which
 * reacts AFTER an error develops, feedforward acts in anticipation.
 *
 * Principle (L2): For a process with disturbance transfer function Gd(s) and
 * control transfer function Gp(s), the ideal feedforward compensator is:
 *
 *   Gff(s) = -Gd(s) / Gp(s)
 *
 * This complete disturbance rejection requires an exact process model and
 * a realizable (proper) compensator. In practice, simplified static or
 * lead-lag approximations are used.
 *
 * FF+FB Architecture (L3):
 *   u_total = u_fb (feedback PID) + u_ff (feedforward compensator)
 *
 * The feedback controller handles modeling errors and unmeasured disturbances,
 * while feedforward handles measured disturbances quickly.
 *
 * References:
 *   - Seborg, Edgar, Mellichamp (2016) Process Dynamics and Control, Ch. 15
 *   - Åström & Hägglund (1995), PID Controllers, Ch. 7
 *   - Brosilow & Joseph (2002), Techniques of Model-Based Control, Ch. 9
 *
 * Curriculum: MIT 6.302, Stanford ENGR205, Purdue ME575, RWTH Aachen ICS
 */

#ifndef FEEDFORWARD_CONTROL_H
#define FEEDFORWARD_CONTROL_H

#include "cascade_types.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Feedforward Compensator Types
 * ========================================================================= */

/** Feedforward compensator operating mode */
typedef enum {
    FF_MODE_OFF      = 0,       /**< Feedforward disabled                */
    FF_MODE_STATIC   = 1,       /**< Static gain only: u_ff = Kff * d   */
    FF_MODE_LEAD_LAG = 2,       /**< Dynamic lead-lag compensation       */
    FF_MODE_DEADTIME = 3,       /**< Deadtime + lead-lag compensation    */
    FF_MODE_RATIO    = 4,       /**< Ratio feedforward (flow ratio)      */
    FF_MODE_TABLE    = 5        /**< Lookup-table feedforward            */
} ff_mode_t;

/** Feedforward compensator runtime state */
typedef struct {
    ff_mode_t   mode;              /**< Operating mode                    */
    double      Kff;               /**< Static feedforward gain           */
    double      T_lead;            /**< Lead time constant [seconds]      */
    double      T_lag;             /**< Lag time constant [seconds]       */
    double      deadtime;          /**< Feedforward deadtime [seconds]    */
    double      bias;              /**< Static bias/offset                */
    double      ff_output;         /**< Current feedforward output        */
    /* Internal digital filter state */
    double      x_prev;            /**< Previous input sample             */
    double      y_prev;            /**< Previous output sample            */
    double      x_delayed[256];    /**< Delay-line buffer for deadtime    */
    uint32_t    delay_index;       /**< Current position in delay line    */
    uint32_t    delay_length;      /**< Delay line length (in samples)    */
    double      Ts;                /**< Sample time [seconds]             */
    double      output_min;        /**< Output lower clamp                */
    double      output_max;        /**< Output upper clamp                */
} ff_compensator_t;

/** Feedforward+Feedback combined controller */
typedef struct {
    ff_compensator_t       ff;           /**< Feedforward compensator     */
    cascade_pid_controller_t fb;          /**< Feedback PID controller     */
    double                 total_output;  /**< Combined FF+FB output       */
    double                 ff_contribution;/**< FF share of output         */
    double                 fb_contribution;/**< FB share of output         */
    bool                   ff_active;     /**< Feedforward active flag     */
    bool                   fb_active;     /**< Feedback active flag        */
    uint64_t               execution_count;
} ff_fb_controller_t;

/* =========================================================================
 * L2: Disturbance Model for Feedforward Design
 * ========================================================================= */

/** Disturbance model: G_d(s) = Kd * exp(-theta_d*s) / (tau_d*s + 1) */
typedef struct {
    double Kd;                /**< Disturbance gain                      */
    double tau_d;             /**< Disturbance lag time constant [s]     */
    double theta_d;           /**< Disturbance deadtime [s]              */
    char   name[32];          /**< Disturbance description               */
} ff_disturbance_model_t;

/** Feedforward design result: compensator parameters derived from models */
typedef struct {
    double Kff_ideal;         /**< Ideal static feedforward gain         */
    double T_lead_ideal;      /**< Ideal lead time constant              */
    double T_lag_ideal;       /**< Ideal lag time constant               */
    double deadtime_ideal;    /**< Ideal deadtime compensation           */
    bool   is_ideal_realizable; /**< True if ideal compensator is proper */
    double static_error_pct;  /**< Expected static error after FF [%]    */
    double bandwidth_ratio;   /**< FF vs FB bandwidth ratio              */
} ff_design_result_t;

/* =========================================================================
 * L2: Feedforward Initialization & Configuration
 * ========================================================================= */

/**
 * ff_init: Initialize a feedforward compensator.
 *
 * Sets default parameters: static mode with unity gain, no lead-lag.
 * All internal filter state is zeroed.
 *
 * Complexity: O(1)
 */
void ff_init(ff_compensator_t *ff, double ts,
             double out_min, double out_max);

/**
 * ff_configure_static: Configure as static gain compensator.
 *
 * u_ff(t) = Kff * d(t) + bias
 *
 * Simplest form, used when disturbance dynamics are negligible compared
 * to the feedback loop speed. Requires only steady-state gain knowledge.
 *
 * Complexity: O(1)
 */
void ff_configure_static(ff_compensator_t *ff,
                          double Kff, double bias);

/**
 * ff_configure_lead_lag: Configure dynamic lead-lag compensator.
 *
 * Gff(s) = Kff * (T_lead*s + 1) / (T_lag*s + 1)
 *
 * Lead-lag allows shaping the feedforward response to match process dynamics.
 * When T_lead > T_lag: anticipatory action (phase lead)
 * When T_lead < T_lag: delayed action (phase lag)
 * When T_lead = T_lag: static gain only
 *
 * Complexity: O(1)
 */
void ff_configure_lead_lag(ff_compensator_t *ff,
                            double Kff, double T_lead, double T_lag);

/**
 * ff_configure_deadtime: Configure deadtime + lead-lag compensator.
 *
 * Gff(s) = Kff * (T_lead*s + 1) / (T_lag*s + 1) * exp(-Lff*s)
 *
 * Deadtime compensation is implemented via a ring buffer delay line.
 * Maximum delay: 256 samples.
 *
 * Complexity: O(1) setup, O(1) per update
 */
void ff_configure_deadtime(ff_compensator_t *ff,
                            double Kff, double T_lead, double T_lag,
                            double deadtime);

/* =========================================================================
 * L3: Feedforward Update Algorithms
 * ========================================================================= */

/**
 * ff_update: Execute one feedforward compensator update.
 *
 * Processes the measured disturbance through the configured compensator
 * and returns the feedforward contribution.
 *
 * Algorithm (lead-lag, Tustin discretization):
 *   y(k) = a*y(k-1) + b*x(k) + c*x(k-1)
 *   where a = (2*T_lag - Ts)/(2*T_lag + Ts)
 *         b = Kff*(2*T_lead + Ts)/(2*T_lag + Ts)
 *         c = Kff*(Ts - 2*T_lead)/(2*T_lag + Ts)
 *
 * @param ff           Feedforward compensator state
 * @param disturbance  Current measured disturbance value
 * @return             Feedforward output contribution
 *
 * Complexity: O(1) without deadtime, O(1) with deadtime (ring buffer)
 */
double ff_update(ff_compensator_t *ff, double disturbance);

/**
 * ff_update_ff_fb: Combined feedforward + feedback update.
 *
 * Computes both contributions independently and sums them.
 * The feedback PID is updated with the disturbance-compensated error.
 * Feedforward acts on the measured disturbance directly.
 *
 * @param controller   FF+FB controller state
 * @param setpoint     Current setpoint
 * @param pv           Current process variable
 * @param disturbance  Current measured disturbance
 * @return             Total control output (FF + FB)
 *
 * Complexity: O(1)
 */
double ff_update_ff_fb(ff_fb_controller_t *controller,
                        double setpoint, double pv, double disturbance);

/**
 * ff_bumpless_enable: Enable feedforward without bump.
 *
 * When enabling feedforward while the controller is running, the
 * feedforward output is ramped from zero to full contribution over
 * a configurable time to avoid a step change in total output.
 *
 * Complexity: O(1) per call
 */
void ff_bumpless_enable(ff_fb_controller_t *controller,
                         double ramp_time_seconds);

/* =========================================================================
 * L5: Feedforward Compensator Design from Process Models
 * ========================================================================= */

/**
 * ff_design_from_models: Design optimal feedforward compensator.
 *
 * Given the process model Gp(s) and disturbance model Gd(s), computes
 * the ideal feedforward compensator:
 *
 *   Gff_ideal(s) = -Gd(s) / Gp(s)
 *
 * For FOPDT models:
 *   Gff(s) = -(Kd/K) * (tau*s + 1)/(tau_d*s + 1) * exp(-(theta_d-theta)*s)
 *
 * The ideal compensator is realizable iff theta_d >= theta (causal deadtime).
 * If not realizable, returns the best causal approximation.
 *
 * @param Gp          Process model (control channel)
 * @param Gd          Disturbance model (disturbance channel)
 * @param result      Output: ideal compensator parameters
 * @return            1 if ideal is realizable, 0 if approximation used, -1 on error
 *
 * Complexity: O(1)
 * Ref: Seborg et al. (2016), Section 15.2
 */
int ff_design_from_models(const cascade_fopdt_model_t *Gp,
                           const ff_disturbance_model_t *Gd,
                           ff_design_result_t *result);

/**
 * ff_design_static: Design static feedforward gain.
 *
 * Computes the steady-state feedforward gain that eliminates
 * the steady-state effect of a measured disturbance.
 *
 *   Kff = -Kd / K
 *
 * This is the minimal feedforward implementation. Effective when
 * disturbance dynamics are much slower than the feedback loop.
 *
 * Complexity: O(1)
 */
double ff_design_static(double process_gain, double disturbance_gain);

/**
 * ff_design_lead_lag_optimal: Design optimal lead-lag compensator.
 *
 * Fits a lead-lag compensator to the ideal feedforward response.
 * Uses frequency-domain matching at the crossover frequency.
 *
 * Complexity: O(1)
 * Ref: Brosilow & Joseph (2002), Section 9.4
 */
int ff_design_lead_lag_optimal(const cascade_fopdt_model_t *Gp,
                                const ff_disturbance_model_t *Gd,
                                double *Kff, double *T_lead, double *T_lag);

/* =========================================================================
 * L5: Cascade + Feedforward Integration
 * ========================================================================= */

/**
 * ff_cascade_update: Feedforward within a cascade structure.
 *
 * In cascade control with feedforward, the disturbance measurement
 * can feed forward to either the secondary loop (fastest response)
 * or the primary loop (if disturbance affects primary process directly).
 *
 * The preferred injection point is the secondary loop setpoint or
 * output, as this provides the fastest corrective action.
 *
 * @param cascade_pair   Primary-secondary cascade pair
 * @param disturbance    Measured disturbance value
 * @param ff             Configured feedforward compensator
 * @param inject_to_secondary  true = inject FF to secondary, false = to primary
 * @return               Modified secondary setpoint (with FF contribution)
 *
 * Complexity: O(1)
 * Ref: Seborg et al. (2016), Section 16.5
 */
double ff_cascade_update(cascade_config_t *cascade_pair,
                          double disturbance,
                          ff_compensator_t *ff,
                          bool inject_to_secondary);

/* =========================================================================
 * L5: Performance Analysis for Feedforward
 * ========================================================================= */

/**
 * ff_performance_evaluate: Evaluate feedforward performance.
 *
 * Compares feedback-only vs feedback+feedforward control by
 * computing the variance reduction ratio:
 *
 *   eta = 1 - var(PV_with_FF) / var(PV_without_FF)
 *
 * @param ff            Feedforward compensator
 * @param Gp            Process model
 * @param Gd            Disturbance model
 * @param disturbance_variance  Variance of disturbance signal
 * @return              Performance ratio [0..1], 1 = perfect rejection
 *
 * Complexity: O(1)
 */
double ff_performance_evaluate(const ff_compensator_t *ff,
                                const cascade_fopdt_model_t *Gp,
                                const ff_disturbance_model_t *Gd,
                                double disturbance_variance);

/**
 * ff_sensitivity_analysis: Sensitivity of FF to model errors.
 *
 * Computes how much degradation occurs when the actual process
 * gain differs from the modeled gain used for FF design.
 *
 *   degradation = |actual_gain - model_gain| / model_gain * 100%
 *
 * Feedforward is NOT robust to model errors — this is why feedback
 * is always used in combination with feedforward.
 *
 * Complexity: O(1)
 */
double ff_sensitivity_analysis(double modeled_gain, double actual_gain);

#ifdef __cplusplus
}
#endif

#endif /* FEEDFORWARD_CONTROL_H */
