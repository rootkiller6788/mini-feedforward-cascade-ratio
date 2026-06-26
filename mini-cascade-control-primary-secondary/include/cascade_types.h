/**
 * @file cascade_types.h
 * @brief Cascade Control — Core Type Definitions & Data Structures
 *
 * Module: mini-cascade-control-primary-secondary
 * Knowledge Coverage: L1 Definitions, L2 Core Concepts
 *
 * Cascade control is a multi-loop control strategy where the output of one
 * controller (primary/master) serves as the setpoint for another controller
 * (secondary/slave). This architecture provides superior disturbance rejection
 * compared to single-loop control.
 *
 * Architecture:
 *   Primary Loop (outer):  slow dynamics, tracks main PV to main SP
 *   Secondary Loop (inner): fast dynamics, rejects disturbances before they
 *                           affect the primary process variable
 *
 * Key advantage: The secondary loop absorbs disturbances locally, preventing
 * them from propagating to the primary process variable.
 *
 * Reference: Seborg, Edgar, Mellichamp (2016) Process Dynamics and Control
 *            Astrom & Hagglund, PID Controllers (1995)
 * Curriculum: MIT 6.302, Stanford ENGR205, Purdue ME575, RWTH Aachen ICS
 */

#ifndef CASCADE_TYPES_H
#define CASCADE_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Cascade Control Mode Definitions
 * ========================================================================= */

/** Cascade control operating modes */
typedef enum {
    CASCADE_MODE_OFF = 0,
    CASCADE_MODE_AUTO = 1,
    CASCADE_MODE_MANUAL = 2,
    CASCADE_MODE_CASCADE = 3,
    CASCADE_MODE_REMOTE_SP = 4,
    CASCADE_MODE_INITIALIZE = 5,
    CASCADE_MODE_FAILSAFE = 6
} cascade_mode_t;

/** Controller direction (direct or reverse acting) */
typedef enum {
    CASCADE_DIRECT_ACTING = 0,
    CASCADE_DIRECT_REVERSE = 1
} cascade_direction_t;

/** Anti-windup strategy for cascade PID */
typedef enum {
    CASCADE_AW_NONE = 0,
    CASCADE_AW_CLAMPING = 1,
    CASCADE_AW_BACK_CALCULATION = 2,
    CASCADE_AW_CONDITIONAL_INTEGRATION = 3,
    CASCADE_AW_EXTERNAL_RESET = 4
} cascade_anti_windup_t;

/** PID algorithm form used in cascade */
typedef enum {
    CASCADE_PID_PARALLEL = 0,
    CASCADE_PID_IDEAL = 1,
    CASCADE_PID_SERIES = 2,
    CASCADE_PID_PARALLEL_DERIV_ON_PV = 3
} cascade_pid_form_t;

/** Process model type for cascade loop analysis */
typedef enum {
    CASCADE_MODEL_FOPDT = 0,
    CASCADE_MODEL_SOPDT = 1,
    CASCADE_MODEL_INTEGRATING = 2,
    CASCADE_MODEL_INTEGRATING_PLUS_DT = 3,
    CASCADE_MODEL_SECOND_ORDER = 4
} cascade_model_type_t;

/** Cascade loop health states (per NAMUR NE107) */
typedef enum {
    CASCADE_HEALTH_OK = 0,
    CASCADE_HEALTH_MAINTENANCE = 1,
    CASCADE_HEALTH_OUT_OF_SPEC = 2,
    CASCADE_HEALTH_CHECK_FUNCTION = 3,
    CASCADE_HEALTH_FAILURE = 4
} cascade_health_t;

/* =========================================================================
 * L1: Constants & Physical Limits
 * ========================================================================= */

/** Rate-of-change limits for cascade output (%/sec) */
#define CASCADE_ROCLIM_MIN           0.1
#define CASCADE_ROCLIM_MAX           1000.0
#define CASCADE_ROCLIM_DEFAULT       100.0

/** Secondary-to-primary update rate ratio (typical: 4:1 to 10:1) */
#define CASCADE_UPDATE_RATIO_MIN     2
#define CASCADE_UPDATE_RATIO_MAX     50
#define CASCADE_UPDATE_RATIO_DEFAULT 5

/** Maximum number of cascade levels supported */
#define CASCADE_MAX_LEVELS           3

/** Anti-windup back-calculation gain default */
#define CASCADE_AW_KC_DEFAULT        1.0

/** Default setpoint filter time constant (seconds) */
#define CASCADE_SP_FILTER_TAU_DEFAULT 0.5

/* =========================================================================
 * L1: PID Parameters Structure
 * ========================================================================= */

typedef struct {
    double kp;
    double ti;
    double td;
    double tf;
    double beta;
    double gamma;
    double output_min;
    double output_max;
    double rate_limit;
} cascade_pid_params_t;

/* =========================================================================
 * L1: PID State Structure (Positional Form)
 * ========================================================================= */

typedef struct {
    double p_term;
    double i_term;
    double d_term;
    double last_error;
    double last_pv;
    double integral;
    double prev_output;
    double prev_setpoint;
    double filtered_setpoint;
    double filtered_pv;
    double last_derivative;
    bool   integrator_active;
    uint64_t sample_count;
} cascade_pid_state_t;

/* =========================================================================
 * L1: First-Order Plus Dead Time (FOPDT) Process Model
 *
 * The most common process model in chemical engineering:
 *   G(s) = K / (tau*s + 1) * exp(-theta*s)
 *
 * where K = process gain, tau = time constant, theta = dead time.
 * ========================================================================= */

typedef struct {
    cascade_model_type_t type;
    double K;
    double tau;
    double theta;
    char   name[32];
} cascade_fopdt_model_t;

/* =========================================================================
 * L1: Second-Order Plus Dead Time (SOPDT) Process Model
 *
 *   G(s) = K / ((tau1*s + 1)*(tau2*s + 1)) * exp(-theta*s)
 *
 * For underdamped second-order processes:
 *   G(s) = K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2) * exp(-theta*s)
 * ========================================================================= */

typedef struct {
    cascade_model_type_t type;
    double K;
    double tau1;
    double tau2;
    double theta;
    double zeta;
    double wn;
    char   name[32];
} cascade_sopdt_model_t;

/* =========================================================================
 * L1: Integrating Plus Dead Time Process Model
 *
 * Typical for level control in vessels:
 *   G(s) = K / s * exp(-theta*s)
 *
 * or with lag:
 *   G(s) = K / (s * (tau*s + 1)) * exp(-theta*s)
 * ========================================================================= */

typedef struct {
    cascade_model_type_t type;
    double K;
    double tau;
    double theta;
    char   name[32];
} cascade_integrating_model_t;

/* =========================================================================
 * L1: Single PID Controller Definition (used in each loop)
 * ========================================================================= */

typedef struct {
    cascade_pid_params_t params;
    cascade_pid_state_t  state;
    cascade_pid_form_t   form;
    cascade_direction_t  direction;
    cascade_anti_windup_t aw_strategy;
    double aw_gain;
    double sample_time;
    double pv_scale_min;
    double pv_scale_max;
    double co_scale_min;
    double co_scale_max;
    uint32_t controller_id;
} cascade_pid_controller_t;

/* =========================================================================
 * L2: Cascade Loop Configuration
 *
 * Primary (outer) loop and Secondary (inner) loop configuration.
 * The primary controller output becomes the secondary controller setpoint.
 * ========================================================================= */

typedef struct {
    cascade_pid_controller_t primary;
    cascade_pid_controller_t secondary;
    cascade_mode_t mode;
    double primary_sp;
    double primary_pv;
    double secondary_sp;
    double secondary_pv;
    double secondary_co;
    double primary_co;
    double secondary_sp_min;
    double secondary_sp_max;
    double primary_sp_min;
    double primary_sp_max;
    double update_ratio;
    uint64_t secondary_updates_per_primary;
    bool bumpless_enabled;
    bool sp_tracking_enabled;
    bool windup_protection;
    uint32_t cascade_id;
} cascade_config_t;

/* =========================================================================
 * L2: Cascade Loop System Model
 *
 * Contains the process models for both outer (primary) and inner (secondary)
 * loops. This is used for simulation, tuning, and stability analysis.
 * ========================================================================= */

typedef struct {
    cascade_fopdt_model_t primary_process;
    cascade_fopdt_model_t secondary_process;
    cascade_fopdt_model_t primary_disturbance;
    cascade_fopdt_model_t secondary_disturbance;
    double coupling_gain;
    bool has_interaction;
} cascade_system_model_t;

/* =========================================================================
 * L2: Cascade Performance Metrics
 *
 * Comprehensive set of control loop performance metrics per ISA and
 * industry best practices.
 * ========================================================================= */

typedef struct {
    double iae;
    double ise;
    double itae;
    double itse;
    double overshoot_pct;
    double settling_time_2pct;
    double rise_time;
    double decay_ratio;
    double steady_state_error;
    double peak_error;
    double output_variance;
    double pv_variance;
    double stiction_index;
    double oscillation_index;
    double minimum_gain_margin;
    double minimum_phase_margin;
} cascade_performance_t;

/* =========================================================================
 * L2: Cascade Tuning Result
 * ========================================================================= */

typedef struct {
    cascade_pid_params_t primary_params;
    cascade_pid_params_t secondary_params;
    double gain_margin;
    double phase_margin;
    double closed_loop_bandwidth;
    double recommended_update_ratio;
    char   method_name[32];
} cascade_tuning_result_t;

/* =========================================================================
 * L2: Cascade Stability Analysis Result
 * ========================================================================= */

typedef struct {
    double gain_margin_db;
    double phase_margin_deg;
    double delay_margin_sec;
    double sensitivity_peak;
    double complementary_sensitivity_peak;
    double modulus_margin;
    double crossover_freq_rad_s;
    double phase_crossover_freq_rad_s;
    bool   is_stable;
    double robustness_index;
    char   stability_verdict[64];
} cascade_stability_t;

/* =========================================================================
 * L2: Advanced Cascade State for Adaptive/Gain-Scheduled Control
 * ========================================================================= */

typedef struct {
    double scheduled_gain;
    double scheduled_ti;
    double scheduled_td;
    double current_operating_point;
    double gain_schedule[10][4];
    int    num_schedule_points;
    double adaptation_rate;
    double forgetting_factor;
    bool   is_adaptive;
    bool   is_gain_scheduled;
} cascade_adaptive_state_t;

/* =========================================================================
 * L2: Cascade Loop Interaction Measure (Relative Gain Array concept)
 * ========================================================================= */

typedef struct {
    double relative_gain;
    double niederlinski_index;
    double condition_number;
    double dynamic_rga[4][4];
    int    num_inputs;
    int    num_outputs;
} cascade_interaction_t;

/* =========================================================================
 * L2: Anti-Windup External Reset State for Cascade
 *
 * In cascade, the outer (primary) loop integrator should be held when
 * the inner (secondary) loop output saturates. This prevents primary
 * windup while the secondary loop is at its limit.
 * ========================================================================= */

typedef struct {
    double tracking_signal;
    double back_calc_gain;
    bool   primary_windup_flag;
    bool   secondary_windup_flag;
    double primary_i_term_saved;
    double secondary_i_term_saved;
    bool   bumpless_active;
} cascade_windup_state_t;

#ifdef __cplusplus
}
#endif

#endif /* CASCADE_TYPES_H */