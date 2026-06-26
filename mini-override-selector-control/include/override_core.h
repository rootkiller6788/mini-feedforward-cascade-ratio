/**
 * @file override_core.h
 * @brief Override/Selector Control — Core Definitions and Structures
 *
 * Reference:
 *   Buckley, P.S. (1964). "Override Controls for Distillation Columns."
 *   Instrumentation Technology.
 *
 *   Shinskey, F.G. (1996). Process Control Systems (4th ed.), McGraw-Hill.
 *   Chapter 9: Constraint Control and Override Systems.
 *
 *   Liptak, B.G. (2006). Instrument Engineers' Handbook (4th ed.), Vol. 2:
 *   Process Control and Optimization. Section 2.21: Override and Selector
 *   Control.
 *
 * Knowledge Coverage:
 *   L1 — Definitions: override, selector, constraint, valve position control
 *   L2 — Core Concepts: high/low/median select, auctioneering, tracking
 *   L3 — Engineering Structures: DCS function blocks, selector topology
 *   L4 — Engineering Laws: ISA-5.1, IEC 61131-3 function blocks
 *
 * Course Alignment:
 *   MIT 6.302: Multivariable constraint handling
 *   Stanford ENGR205: Industrial process constraints
 *   Purdue ME 575: Override/auctioneering control strategies
 *   Tsinghua: 过程控制工程 — 超驰控制/选择控制
 */

#ifndef OVERRIDE_CORE_H
#define OVERRIDE_CORE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * L1 — Override/Selector Type Enumerations
 * ========================================================================= */

/** Override control mode */
typedef enum {
    OVERRIDE_MODE_DISABLED    = 0,  /**< No override active */
    OVERRIDE_MODE_PRIMARY     = 1,  /**< Primary controller active */
    OVERRIDE_MODE_OVERRIDE    = 2,  /**< Override/constraint controller active */
    OVERRIDE_MODE_MANUAL      = 3,  /**< Manual override */
    OVERRIDE_MODE_INITIALIZE  = 4,  /**< Initialization/bumpless handoff */
    OVERRIDE_MODE_FAULT       = 5,  /**< Fault state */
    OVERRIDE_MODE_COUNT       = 6
} override_mode_t;

/** Selector logic type */
typedef enum {
    SELECTOR_HIGH       = 0,  /**< Select maximum output */
    SELECTOR_LOW        = 1,  /**< Select minimum output */
    SELECTOR_MEDIAN     = 2,  /**< Select median output */
    SELECTOR_WEIGHTED   = 3,  /**< Weighted combination */
    SELECTOR_AUCTIONEER = 4,  /**< Auctioneering (mid of 3) */
    SELECTOR_AVERAGE    = 5,  /**< Average of valid signals */
    SELECTOR_CUSTOM     = 6,  /**< User-defined logic */
    SELECTOR_COUNT      = 7
} selector_type_t;

/** Constraint type classification */
typedef enum {
    CONSTRAINT_NONE        = 0,  /**< No constraint */
    CONSTRAINT_HARD_ABS    = 1,  /**< Hard absolute limit (safety) */
    CONSTRAINT_HARD_RATE   = 2,  /**< Hard rate-of-change limit */
    CONSTRAINT_SOFT_ABS    = 3,  /**< Soft absolute limit with penalty */
    CONSTRAINT_SOFT_RATE   = 4,  /**< Soft rate limit with penalty */
    CONSTRAINT_QUALITY     = 5,  /**< Quality constraint (e.g., purity) */
    CONSTRAINT_ECONOMIC    = 6,  /**< Economic optimization constraint */
    CONSTRAINT_COUNT       = 7
} constraint_type_t;

/** Tracking/initialization mode for inactive controllers */
typedef enum {
    TRACK_NONE            = 0,  /**< No tracking */
    TRACK_PV              = 1,  /**< Track process variable */
    TRACK_OUTPUT          = 2,  /**< Track selected output */
    TRACK_EXTERNAL_RESET  = 3,  /**< External reset feedback (IEC 61131-3) */
    TRACK_PREDICTIVE      = 4,  /**< Predictive initialization */
    TRACK_COUNT           = 5
} tracking_mode_t;

/** Controller execution priority level */
typedef enum {
    PRIORITY_EMERGENCY    = 0,  /**< ESD interlocks (highest) */
    PRIORITY_SAFETY       = 1,  /**< SIS constraints */
    PRIORITY_CONSTRAINT   = 2,  /**< Process constraints */
    PRIORITY_PRIMARY      = 3,  /**< Primary control */
    PRIORITY_OPTIMIZATION = 4,  /**< Economic optimization */
    PRIORITY_DIAGNOSTIC   = 5,  /**< Diagnostic/monitoring (lowest) */
    PRIORITY_COUNT        = 6
} override_priority_t;

/** Selector output transition mode */
typedef enum {
    XFER_BUMPLESS      = 0,  /**< Bumpless transfer via tracking */
    XFER_BUMPED        = 1,  /**< Bumped transfer (immediate switch) */
    XFER_RATE_LIMITED  = 2,  /**< Rate-limited smooth transition */
    XFER_RAMP          = 3,  /**< Linear ramp over time */
    XFER_COUNT         = 4
} xfer_mode_t;

/* =========================================================================
 * L1 — Core Control Structures
 * ========================================================================= */

/**
 * PID controller parameters for override control
 *
 * Each controller in an override scheme has its own tuning parameters.
 * Inactive controllers must track the active controller's output to
 * prevent integral windup and enable bumpless transfer.
 */
typedef struct {
    double Kc;           /**< Proportional gain [(MV units)/(PV units)] */
    double Ti;           /**< Integral time [s] (HUGE_VAL = I-only off) */
    double Td;           /**< Derivative time [s] */
    double N;            /**< Derivative filter coefficient [2..50] */
    double Ts;           /**< Sampling period [s] */
    double b;            /**< Setpoint weight for P [0..1] */
    double c;            /**< Setpoint weight for D [0..1] */
    double u_min;        /**< Output lower limit */
    double u_max;        /**< Output upper limit */
    double tracking_gain;/**< External reset tracking gain (1/Tt) [1/s] */
} override_pid_params_t;

/**
 * Constraint variable definition
 *
 * Each constraint variable has high/low limits and a priority level.
 * The override system monitors constraint variables and switches
 * control when a constraint approaches violation.
 */
typedef struct {
    const char *tag;           /**< Tag name (e.g., "TI-101.HI") */
    const char *description;   /**< Human-readable description */
    override_priority_t priority; /**< Priority level */
    constraint_type_t ctype;   /**< Constraint type */
    double hi_limit;           /**< High limit value */
    double lo_limit;           /**< Low limit value */
    double hi_hi_limit;        /**< High-high limit (emergency) */
    double lo_lo_limit;        /**< Low-low limit (emergency) */
    double margin;             /**< Safety margin before override engages */
    double rate_limit;         /**< Rate limit [units/s] (0 = no limit) */
    int    enabled;            /**< 1 = constraint active */
    int    latched;            /**< 1 = latched until manual reset */
    int    degrade_on_fault;   /**< 1 = degrade to safe state on fault */
} constraint_def_t;

/**
 * Constraint variable runtime state
 */
typedef struct {
    const constraint_def_t *def; /**< Pointer to constraint definition */
    double current_value;        /**< Current measured value */
    double rate_of_change;       /**< Current rate of change */
    double approach_factor;      /**< 0=far, 1=at limit */
    int    violating;            /**< 1 = currently violating */
    int    approaching;          /**< 1 = approaching limit */
    int    faulted;              /**< 1 = sensor fault */
    int    overridden;           /**< 1 = being overridden */
    double time_in_violation;    /**< Accumulated violation time [s] */
} constraint_state_t;

/**
 * Override selector runtime state
 *
 * Manages the active controller selection and tracking of all
 * inactive controllers.
 */
typedef struct {
    override_mode_t mode;          /**< Current operating mode */
    override_mode_t prev_mode;     /**< Previous operating mode */
    selector_type_t selector_type; /**< Selector logic type */
    xfer_mode_t    xfer_mode;      /**< Transfer mode */

    int active_controller;         /**< Index of currently active controller */
    int prev_controller;           /**< Previously active controller index */
    int num_controllers;           /**< Total number of controllers */
    int num_constraints;           /**< Total number of constraint variables */

    double selector_output;        /**< Final selected output value */
    double prev_output;            /**< Previous output (for rate limiting) */

    double *controller_outputs;    /**< Array[n]: individual controller outputs */
    double *controller_tracking;   /**< Array[n]: tracking values for inactive */
    int    *controller_enabled;    /**< Array[n]: controller enable flags */
    int    *controller_faulted;    /**< Array[n]: controller fault flags */

    constraint_state_t *constraints; /**< Array: constraint variable states */

    double hysteresis;             /**< Selector hysteresis band */
    double output_rate_limit;      /**< Output rate limit [units/s] */

    double sample_time;            /**< Execution sample time [s] */
    int    iteration_count;        /**< Number of executions */

    int    initialized;            /**< 1 = fully initialized */
    int    fault_latch;            /**< 1 = latched fault active */
    int    standby_active;         /**< 1 = standby/shutdown active */
} override_state_t;

/**
 * Override system configuration
 */
typedef struct {
    int max_controllers;           /**< Maximum number of controllers */
    int max_constraints;           /**< Maximum number of constraints */
    double default_hysteresis;     /**< Default selector hysteresis */
    double default_rate_limit;     /**< Default output rate limit */
    double fault_timeout;          /**< Time to declare fault [s] */
    double tracking_threshold;     /**< Tracking error threshold */
    int    enable_diagnostics;     /**< 1 = enable diagnostic logging */
    int    enable_fault_latch;     /**< 1 = latch faults */
    int    enable_auto_recovery;   /**< 1 = auto-recover from non-latched faults */
} override_config_t;

/**
 * Controller instance within override system
 *
 * Each controller (primary and override) has its own PID parameters,
 * setpoint, process variable, and output. The tracking_value is used
 * for external reset when the controller is inactive.
 */
typedef struct {
    int id;                       /**< Controller ID/index */
    const char *tag;              /**< Controller tag name */
    const char *description;      /**< Human-readable description */
    override_priority_t priority; /**< Priority level */

    override_pid_params_t params; /**< PID tuning parameters */

    double setpoint;              /**< Current setpoint */
    double pv;                    /**< Current process variable */
    double output;                /**< Current computed output */
    double tracking_value;        /**< External tracking signal */

    double integral;              /**< Accumulated integral term */
    double last_error;            /**< Previous error (for D term) */
    double last_pv;               /**< Previous PV (derivative-on-PV) */
    double last_output;           /**< Previous output */

    int    active;                /**< 1 = currently active */
    int    enabled;               /**< 1 = controller enabled */
    int    faulted;               /**< 1 = fault detected */
    int    in_manual;             /**< 1 = manual mode */
    double manual_output;         /**< Manual mode output value */

    double saturation_count;      /**< Time spent in saturation [s] */
    int    windup_limited;        /**< 1 = currently anti-windup limited */
} override_controller_t;

/* =========================================================================
 * L1 — Diagnostic/Event Structures
 * ========================================================================= */

/** Override event record */
typedef struct {
    int    event_id;             /**< Sequential event ID */
    double timestamp;            /**< Event time [s] */
    override_mode_t old_mode;    /**< Previous mode */
    override_mode_t new_mode;    /**< New mode */
    int    old_controller;       /**< Previous active controller */
    int    new_controller;       /**< New active controller */
    double selected_output;      /**< Output value at switch */
    const char *reason;          /**< Human-readable reason */
} override_event_t;

/** Diagnostic counter structure */
typedef struct {
    int total_mode_switches;     /**< Total mode transitions */
    int constraint_violations;   /**< Total constraint violations */
    int selector_changes;        /**< Total selector output changes */
    int tracking_cycles;         /**< Total tracking execution cycles */
    int fault_events;            /**< Total fault detections */
    double max_constraint_margin; /**< Maximum violation margin */
    double accumulated_violation; /**< Integral of violation */
    int controller_activation[PRIORITY_COUNT]; /**< Activation count per priority */
} override_diag_t;

/* =========================================================================
 * L1 — Valve Position Control (VPC) Structures
 * ========================================================================= */

/**
 * VPC is a specialized override strategy where a valve position
 * controller manipulates a bypass or recycle valve to maintain
 * the main control valve within its effective range (typically
 * 10-90% open).
 */
typedef struct {
    int    enabled;              /**< 1 = VPC active */
    double vpc_setpoint;         /**< Desired valve position [%] */
    double vpc_min;              /**< Minimum desired valve position [%] */
    double vpc_max;              /**< Maximum desired valve position [%] */
    double main_valve_position;  /**< Current main valve position [%] */
    double vpc_valve_position;   /**< Current VPC valve position [%] */
    override_pid_params_t vpc_pid; /**< VPC PID parameters */
    double vpc_output;           /**< VPC PID output */
    int    vpc_active;           /**< 1 = VPC is currently active */
    double integral;             /**< VPC integral accumulator */
    double last_error;           /**< VPC last error */
    double last_output;          /**< VPC last filtered D output */
} vpc_state_t;

/**
 * Compressor surge control structure
 *
 * Surge is a catastrophic flow reversal in centrifugal compressors.
 * A high-select override opens a recycle/blow-off valve when the
 * operating point approaches the surge line.
 *
 * Reference: Nisenfeld, A.E. & Seemann, R.C. (1981). "Centrifugal
 *   Compressors — Understanding Their Control." ISA.
 */
typedef struct {
    double flow;                  /**< Current flow [kg/s or m³/s] */
    double delta_p;               /**< Differential pressure [kPa] */
    double suction_pressure;      /**< Suction pressure [kPa] */
    double discharge_pressure;    /**< Discharge pressure [kPa] */
    double surge_line_slope;      /**< Surge line slope parameter */
    double surge_line_intercept;  /**< Surge line intercept */
    double surge_margin;          /**< Current surge margin [%] */
    double min_surge_margin;      /**< Minimum surge margin before override [%] */
    double recycle_valve_output;  /**< Recycle valve command [%] */
    int    surge_override_active; /**< 1 = surge override active */
    int    surge_detected;        /**< 1 = surge condition detected */
    double surge_onset_time;      /**< Time when surge condition began [s] */
} surge_control_t;

/* =========================================================================
 * L1 — Function Declarations: Initialization & Validation
 * ========================================================================= */

int override_state_init(override_state_t *state,
                        const override_config_t *config,
                        int n_controllers,
                        int n_constraints);

void override_state_free(override_state_t *state);

void override_pid_params_init(override_pid_params_t *params);

void override_controller_init(override_controller_t *ctrl,
                              int id,
                              const char *tag,
                              const char *desc,
                              override_priority_t priority);

void override_constraint_def_init(constraint_def_t *def);

int override_constraint_def_is_valid(const constraint_def_t *def);

void override_constraint_state_init(constraint_state_t *state,
                                    const constraint_def_t *def);

int override_pid_params_is_valid(const override_pid_params_t *params);

int override_state_is_valid(const override_state_t *state);

void override_state_reset(override_state_t *state);

void surge_control_init(surge_control_t *surge,
                        double slope,
                        double intercept,
                        double min_margin);

void vpc_state_init(vpc_state_t *vpc,
                    double setpoint,
                    double vpc_min,
                    double vpc_max);

/* =========================================================================
 * L1 — Function Declarations: Enumeration String Converters
 * ========================================================================= */

const char* override_mode_name(override_mode_t mode);
const char* selector_type_name(selector_type_t stype);
const char* constraint_type_name(constraint_type_t ctype);
const char* tracking_mode_name(tracking_mode_t tmode);
const char* override_priority_name(override_priority_t priority);
const char* xfer_mode_name(xfer_mode_t xmode);

/* =========================================================================
 * L2 — Function Declarations: Core Override Logic
 * ========================================================================= */

double override_execute(override_state_t *state,
                        override_controller_t *controllers,
                        int n_controllers);

int override_evaluate_constraints(override_state_t *state);

int override_transfer(override_state_t *state,
                      override_controller_t *controllers,
                      int from_idx,
                      int to_idx);

void override_update_tracking(override_state_t *state,
                              override_controller_t *controllers,
                              int n_controllers);

int override_select_controller(override_state_t *state,
                               override_controller_t *controllers,
                               int n_controllers);

#ifdef __cplusplus
}
#endif

#endif /* OVERRIDE_CORE_H */
