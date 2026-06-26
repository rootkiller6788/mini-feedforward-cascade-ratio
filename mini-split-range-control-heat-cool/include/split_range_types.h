/**
 * @file split_range_types.h
 * @brief Split-Range Control — Core Type Definitions & Data Structures
 *
 * Module: mini-split-range-control-heat-cool
 * Knowledge Coverage: L1 Definitions, L2 Core Concepts
 *
 * Split-range control splits a single PID controller output across multiple
 * final control elements (valves, actuators). The classic application is
 * temperature control of a jacketed reactor where one valve admits steam
 * (heating) and another admits cooling water.
 *
 * Typical split scheme:
 *   Controller Output 0%–50%  → Heating Valve  0%–100% open (reverse: 100%→0%)
 *   Controller Output 50%–100% → Cooling Valve  0%–100% open
 *   Dead zone (48%–52%): both valves closed (configurable)
 *
 * Alternative schemes: sequenced, overlapped, complementary-characterized.
 *
 * Reference:
 *   Seborg, Edgar, Mellichamp (2016) Process Dynamics and Control, Ch. 16
 *   Myke King (2016) Process Control: A Practical Approach, Ch. 9
 *   ISA-75.01 — Control Valve Sizing Equations
 *   ISA-5.1 — Instrumentation Symbols and Identification
 *
 * Curriculum Mapping:
 *   MIT 6.302 — Feedback Systems (actuator saturation)
 *   Stanford ENGR205 — Process Control (split-range strategies)
 *   Purdue ME575 — Industrial Control (valve sequencing)
 *   RWTH Aachen — Industrial Control Systems (heating/cooling)
 *   Tsinghua — 过程控制工程 (分程控制)
 */

#ifndef SPLIT_RANGE_TYPES_H
#define SPLIT_RANGE_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1: Split-Range Mode Definitions
 * ========================================================================= */

typedef enum {
    SPLIT_MODE_SEQUENTIAL = 0,
    SPLIT_MODE_COMPLEMENTARY = 1,
    SPLIT_MODE_OVERLAP = 2,
    SPLIT_MODE_THREE_WAY = 3,
    SPLIT_MODE_CHARACTERIZED = 4,
    SPLIT_MODE_CUSTOM = 5
} split_range_mode_t;

typedef enum {
    SPLIT_ACTION_INCREASING = 0,
    SPLIT_ACTION_DECREASING = 1,
    SPLIT_ACTION_FIXED = 2
} split_range_action_t;

typedef enum {
    SPLIT_VALVE_LINEAR = 0,
    SPLIT_VALVE_EQUAL_PCT = 1,
    SPLIT_VALVE_QUICK_OPENING = 2,
    SPLIT_VALVE_MODIFIED_PARABOLIC = 3,
    SPLIT_VALVE_USER_TABLE = 4
} split_range_valve_char_t;

typedef enum {
    SPLIT_HEALTH_OK = 0,
    SPLIT_HEALTH_MAINTENANCE_REQUIRED = 1,
    SPLIT_HEALTH_OUT_OF_SPEC = 2,
    SPLIT_HEALTH_CHECK_FUNCTION = 3,
    SPLIT_HEALTH_FAILURE = 4,
    SPLIT_HEALTH_STICTION_DETECTED = 5,
    SPLIT_HEALTH_HYSTERESIS_EXCESSIVE = 6
} split_range_health_t;

typedef enum {
    SPLIT_TRANSITION_HARD = 0,
    SPLIT_TRANSITION_LINEAR = 1,
    SPLIT_TRANSITION_CUBIC_SPLINE = 2,
    SPLIT_TRANSITION_HYSTERESIS = 3
} split_range_transition_t;

/* =========================================================================
 * L1: Constants — ISA-75.01 Control Valve Sizing
 * ========================================================================= */

#define SPLIT_POINT_DEFAULT         50.0
#define SPLIT_POINT_MIN             1.0
#define SPLIT_POINT_MAX             99.0
#define SPLIT_DEADBAND_DEFAULT      2.0
#define SPLIT_DEADBAND_MIN          0.0
#define SPLIT_DEADBAND_MAX          20.0
#define SPLIT_OVERLAP_DEFAULT       0.0
#define SPLIT_OVERLAP_MAX           30.0
#define SPLIT_MAX_CHANNELS          6
#define SPLIT_VALVE_TABLE_MAX       64
#define SPLIT_RANGEABILITY_DEFAULT  50.0
#define SPLIT_CO_MIN                0.0
#define SPLIT_CO_MAX                100.0
#define SPLIT_VALVE_MIN             0.0
#define SPLIT_VALVE_MAX             100.0
#define SPLIT_SLEW_RATE_DEFAULT     25.0

/* =========================================================================
 * L1: Valve Characteristics Data Table
 * ========================================================================= */

typedef struct {
    double co_pct[64];
    double flow_pct[64];
    int    num_points;
} split_range_user_table_t;

/* =========================================================================
 * L1: Single Valve / Actuator Definition
 * Key parameters from ISA-75.01: Cv, FL, xT
 * ========================================================================= */

typedef struct {
    uint32_t                 channel_id;
    split_range_action_t     action;
    split_range_valve_char_t characteristic;

    double co_range_start;
    double co_range_end;
    double valve_range_start;
    double valve_range_end;

    double slew_rate_limit;
    double hysteresis_band;
    double stiction_threshold;

    double Cv_rated;
    double FL;
    double xT;
    double delta_P_design;

    double current_position;
    double target_position;
    double flow_rate;

    split_range_health_t health;
    bool   maintenance_override;
    bool   manual_mode;
    double manual_position;

    split_range_user_table_t user_table;
    char   tag_name[32];
    char   service_description[64];
} split_range_channel_t;

/* =========================================================================
 * L1: Split Scheme / Sequencing Definition
 *
 * Defines how the controller output 0–100% maps to each channel valve position.
 * Classic heat/cool scheme:
 *   Channel 0 (Heating): CO 0%→50% maps to Valve 100%→0% (reverse acting)
 *   Channel 1 (Cooling): CO 50%→100% maps to Valve 0%→100% (direct acting)
 *
 * Mathematical representation:
 *   For channel i in range [a_i, b_i]:
 *     v_i = g_i( clamp((co - a_i) / (b_i - a_i), 0, 1) )
 *   where g_i is the valve characteristic function.
 * ========================================================================= */

typedef struct {
    split_range_mode_t       mode;
    uint32_t                 num_channels;
    split_range_channel_t    channels[6];

    double split_point;
    double deadband_width;
    double overlap_width;
    split_range_transition_t transition_type;

    bool   bumpless_transfer;
    bool   track_manual_input;
    bool   balanced_heat_cool;

    double hysteresis_memory[6];
} split_range_scheme_t;

/* =========================================================================
 * L1: Process Variable / Setpoint Context
 * ========================================================================= */

typedef struct {
    double  process_variable;
    double  setpoint;
    double  previous_pv;
    double  previous_sp;
    double  pv_filtered;
    double  pv_rate_of_change;
    double  pv_scale_min;
    double  pv_scale_max;
    double  feedforward_signal;
    bool    feedforward_enabled;
    char    pv_tag[32];
    char    pv_units[16];
} split_range_pv_t;

/* =========================================================================
 * L1: PID Parameters for Split-Range Controller
 *
 * Standard parallel-form PID:
 *   CO(s) = Kc * (E(s) + 1/(Ti*s)*E(s) + Td*s*E(s))
 *
 * With filtering on derivative term:
 *   CO(s) = Kc * (E(s) + 1/(Ti*s)*E(s) + (Td*s)/(1 + Td/N*s)*E(s))
 * ========================================================================= */

typedef struct {
    double kc;
    double ti;
    double td;
    double tf;
    double derivative_filter_N;
    double beta;
    double gamma;
    double sample_time_sec;
    double bumpless_gain;
} split_range_pid_params_t;

/**
 * L1: PID State (Incremental / Velocity Form)
 *
 * The velocity form is preferred for split-range because it inherently
 * provides bumpless transfer. The integrator is implicitly handled by
 * the accumulator, and changing modes does not cause output bumps.
 *
 * Velocity form:
 *   Δu(k) = Kc * [Δe(k) + (Ts/Ti)*e(k) + (Td/Ts)*(Δe(k) - Δe(k-1))]
 *   u(k)  = u(k-1) + Δu(k)
 */
typedef struct {
    double  p_term;
    double  i_term;
    double  d_term;
    double  last_error;
    double  prev_error;
    double  integral_accum;
    double  last_output;
    double  filtered_derivative;
    double  filtered_pv;
    double  prev_filtered_pv;
    double  prev_d_term;
    bool    integrator_hold;
    bool    derivative_kick_handled;
    uint64_t sample_index;
    double  tracking_error;
    double  external_reset;
} split_range_pid_state_t;

/* =========================================================================
 * L1: Complete Split-Range Controller
 * ========================================================================= */

typedef struct {
    split_range_pid_params_t  pid_params;
    split_range_pid_state_t   pid_state;
    split_range_scheme_t      scheme;
    split_range_pv_t          pv_context;

    double controller_output;
    double split_outputs[6];
    double previous_controller_output;

    bool   enabled;
    bool   cascade_mode;
    double remote_setpoint;
    bool   remote_sp_active;

    split_range_health_t overall_health;
    uint32_t controller_id;
} split_range_controller_t;

/* =========================================================================
 * L1: Heat/Cool Process Model (Jacketed CSTR)
 *
 * Energy balance for a jacketed continuous stirred-tank reactor:
 *   V*rho*Cp * dT/dt = F*rho*Cp*(Tin - T) + U*A*(Tj - T)
 *                      + (-delta_Hr)*V*k0*exp(-Ea/(R_gas*T))*CA
 * ========================================================================= */

typedef struct {
    double V;
    double rho;
    double Cp;
    double U_times_A;
    double delta_Hr;
    double k0;
    double Ea;
    double R_gas;
    double CA;
    double F;
    double Tin;
    double Tj;
    double T;
    double T_ambient;
    double Tj_min;
    double Tj_max;
    double Q_heater_max;
    double Q_cooler_max;
    double inflection_temp;
    bool   has_runaway_risk;
} split_range_reactor_model_t;

/* =========================================================================
 * L2: Split-Range Performance Metrics
 * ========================================================================= */

typedef struct {
    double iae;
    double ise;
    double itae;
    double itse;
    double overshoot_pct;
    double settling_time_2pct;
    double rise_time_10_90;
    double decay_ratio;
    double steady_state_error;
    double peak_error;
    double output_variance;
    double pv_variance;
    double total_valve_travel[6];
    double valve_reversal_count[6];
    double energy_consumption_heating;
    double energy_consumption_cooling;
    double split_efficiency_index;
    double cross_coupling_index;
} split_range_performance_t;

/* =========================================================================
 * L2: Split-Range Tuning Result
 * ========================================================================= */

typedef struct {
    split_range_pid_params_t pid_params;
    double split_point_optimal;
    double deadband_optimal;
    double gain_margin_db;
    double phase_margin_deg;
    char   method_name[48];
    double recommended_update_rate_ms;
} split_range_tuning_result_t;

/* =========================================================================
 * L3: Split-Range Auto-Tuning State (Relay Feedback Method)
 *
 * Based on Astrom-Hagglund relay feedback for PID auto-tuning,
 * adapted for split-range systems where the relay amplitude must
 * account for both heating and cooling gains (often asymmetric).
 * ========================================================================= */

typedef struct {
    double relay_amplitude_heat;
    double relay_amplitude_cool;
    double relay_hysteresis;
    double ultimate_gain;
    double ultimate_period;
    double identified_gain_heat;
    double identified_gain_cool;
    double identified_tau_heat;
    double identified_tau_cool;
    double identified_theta_heat;
    double identified_theta_cool;
    int    cycle_count;
    bool   identification_complete;
    bool   asymmetric_gains_detected;
} split_range_autotune_t;

/* =========================================================================
 * L8: Adaptive Split-Range Gain Scheduling
 *
 * For processes where the heating and cooling gains differ significantly
 * (common in exothermic reactors), adaptive gain scheduling adjusts Kc
 * depending on whether the controller output is in the heating or cooling
 * region. Uses a Lyapunov-based stability guarantee for gain transitions.
 * Monte Carlo calibration is used for robust parameter estimation.
 * ========================================================================= */

typedef struct {
    double kc_heating;
    double kc_cooling;
    double kc_neutral;
    double ti_heating;
    double ti_cooling;
    double ti_neutral;
    double td_heating;
    double td_cooling;
    double td_neutral;
    double transition_rate;
    double current_effective_kc;
    double current_effective_ti;
    double current_effective_td;
    bool   adaptive_enabled;
    double lyapunov_margin;
    int    stagnation_counter;
    bool   monte_carlo_calibrated;
    double mcmc_acceptance_ratio;
} split_range_adaptive_t;

#ifdef __cplusplus
}
#endif

#endif /* SPLIT_RANGE_TYPES_H */
