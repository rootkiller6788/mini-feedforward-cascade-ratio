#ifndef FEEDFORWARD_DEFS_H
#define FEEDFORWARD_DEFS_H

#include <stddef.h>
#include <math.h>

/* ============================================================================
 * L1: Core Definitions — Fundamental types and constants
 * ============================================================================ */

/** Transfer function order enumeration */
typedef enum {
    TF_ORDER_ZERO    = 0,
    TF_ORDER_FIRST   = 1,
    TF_ORDER_SECOND  = 2,
    TF_ORDER_HIGH    = 3
} tf_order_t;

/** Feedforward mode enumeration (ISA-5.1 compatible labels) */
typedef enum {
    FF_MODE_OFF       = 0,
    FF_MODE_STATIC    = 1,
    FF_MODE_DYNAMIC   = 2,
    FF_MODE_COMBINED  = 3,
    FF_MODE_ADAPTIVE  = 4
} ff_mode_t;

/** Controller action direction */
typedef enum {
    ACTION_DIRECT    =  1,
    ACTION_REVERSE   = -1
} action_t;

#define FF_SIGNAL_MIN     0.0
#define FF_SIGNAL_MAX   100.0
#define FF_GAIN_MIN       0.001
#define FF_GAIN_MAX    1000.0
#define FF_TAU_MIN        0.001
#define FF_TAU_MAX    86400.0
#define FF_THETA_MAX   3600.0
#define FF_EPSILON       1e-12

/* ============================================================================
 * L1/L3: Transfer function structures
 * Represents: G(s) = K * e^(-theta*s) * N(s)/D(s)
 * ============================================================================ */

typedef struct {
    double  K;
    double  theta;
    double  num_coeffs[8];
    double  den_coeffs[8];
    int     order_num;
    int     order_den;
    tf_order_t type;
} tf_t;

typedef struct {
    double  num[16];
    double  den[16];
    int     n;
    int     m;
    double  Ts;
} tf_discrete_t;

/** Lead-Lag: G_ll(s) = K_ll * (T_lead*s+1) / (T_lag*s+1) */
typedef struct {
    double  K_ll;
    double  T_lead;
    double  T_lag;
    double  Ts;
    double  y_prev;
    double  x_prev;
    int     initialized;
} lead_lag_t;

/** Second-order lead-lag with damping */
typedef struct {
    double  K;
    double  Tn;
    double  Td;
    double  zeta_n;
    double  zeta_d;
    double  Ts;
    double  y1, y2;
    double  x1, x2;
    int     initialized;
} lead_lag2_t;

/* ============================================================================
 * L2: Process model and disturbance model definitions
 * ============================================================================ */

/** FOPDT: G(s) = Kp * e^(-theta*s) / (tau*s + 1) */
typedef struct {
    double  Kp;
    double  tau;
    double  theta;
} fopdt_t;

/** SOPDT: G(s) = Kp * e^(-theta*s) / ((tau1*s+1)*(tau2*s+1)) */
typedef struct {
    double  Kp;
    double  tau1;
    double  tau2;
    double  theta;
} sopdt_t;

/** IPDT: G(s) = Kp * e^(-theta*s) / s */
typedef struct {
    double  Kp;
    double  theta;
} ipdt_t;

/** Disturbance model: Gd(s) = Kd * e^(-theta_d*s) / (tau_d*s + 1) */
typedef struct {
    double  Kd;
    double  tau_d;
    double  theta_d;
} dist_model_t;

/** Signal status for robust operation */
typedef enum {
    SIG_VALID           = 0,
    SIG_OVERRANGE       = 1,
    SIG_UNDERRANGE      = 2,
    SIG_STALE           = 3,
    SIG_SENSOR_FAULT    = 4,
    SIG_BAD_QUALITY     = 5
} signal_status_t;

/** Disturbance measurement with quality metadata */
typedef struct {
    double          value;
    double          raw_value;
    double          timestamp;
    signal_status_t status;
    double          range_min;
    double          range_max;
    double          rate_limit;
} disturbance_meas_t;

/** Complete feedforward controller structure */
typedef struct {
    double      Kff;
    ff_mode_t   mode;
    action_t    action;
    double      bias;
    double      output_min;
    double      output_max;
    lead_lag_t  lead_lag;
    lead_lag2_t lead_lag2;
    tf_discrete_t dyn_tf;
    double      d_meas;
    double      d_prev;
    double      d_filtered;
    double      alpha_filter;
    double      u_ff_static;
    double      u_ff_dynamic;
    double      u_ff_total;
    double      u_combined;
    int         clamping;
    int         anti_windup;
    int         tracking;
    double      track_value;
    int         initialized;
    double      Ts;
} feedforward_t;

/** Feedforward performance metrics */
typedef struct {
    double  var_without_ff;
    double  var_with_ff;
    double  var_reduction_pct;
    double  peak_error_without;
    double  peak_error_with;
    double  ise_without;
    double  ise_with;
    double  ise_reduction_pct;
    double  settling_time_without;
    double  settling_time_with;
} ff_performance_t;

#endif