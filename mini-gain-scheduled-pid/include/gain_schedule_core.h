#ifndef GAIN_SCHEDULE_CORE_H
#define GAIN_SCHEDULE_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SCHED_VAR_MEASURED_OUTPUT    = 0,
    SCHED_VAR_REFERENCE_SETPOINT = 1,
    SCHED_VAR_CONTROL_SIGNAL     = 2,
    SCHED_VAR_EXTERNAL_MEASURED  = 3,
    SCHED_VAR_STATE_VECTOR       = 4,
    SCHED_VAR_TIME               = 5,
    SCHED_VAR_VELOCITY           = 6,
    SCHED_VAR_FLOW_RATE          = 7,
    SCHED_VAR_TEMPERATURE        = 8,
    SCHED_VAR_PRESSURE           = 9,
    SCHED_VAR_PH                 = 10
} sched_var_type_t;

typedef enum {
    INTERP_NEAREST_NEIGHBOR     = 0,
    INTERP_LINEAR               = 1,
    INTERP_CUBIC_HERMITE        = 2,
    INTERP_CUBIC_SPLINE         = 3,
    INTERP_POLYNOMIAL_LAGRANGE  = 4,
    INTERP_AKIMA                = 5,
    INTERP_GAUSSIAN_RBF         = 6
} interp_method_t;

typedef enum {
    GS_PID_PARALLEL_IDEAL       = 0,
    GS_PID_SERIES_INTERACTING   = 1,
    GS_PID_ISA_STANDARD         = 2,
    GS_PID_ACADEMIC_PARALLEL    = 3,
    GS_PID_TWO_DOF              = 4,
    GS_PID_INCREMENTAL_VELOCITY = 5
} gs_pid_form_t;

typedef enum {
    REGION_LOW_GAIN             = 0,
    REGION_NOMINAL              = 1,
    REGION_HIGH_GAIN            = 2,
    REGION_SATURATION           = 3,
    REGION_STARTUP              = 4,
    REGION_SHUTDOWN             = 5,
    REGION_TRANSITION_UP        = 6,
    REGION_TRANSITION_DOWN      = 7
} operating_region_t;

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double Ti;
    double Td;
    double N;
    double b;
    double c;
    double Kb;
    double tracking_time;
    double alpha;
} pid_gain_set_t;

typedef struct {
    double             scheduling_value;
    pid_gain_set_t     gains;
    operating_region_t region;
    double             bandwidth;
    double             gain_margin;
    double             phase_margin;
    double             sensitivity_peak;
    bool               validated;
    char               label[32];
} schedule_entry_t;

#define GS_MAX_BREAKPOINTS 128

typedef struct {
    schedule_entry_t entries[GS_MAX_BREAKPOINTS];
    uint32_t         num_entries;
    sched_var_type_t sched_var_type;
    interp_method_t  interp_method;
    double           sched_min;
    double           sched_max;
    double           hysteresis_band;
    bool             extrapolate_low;
    bool             extrapolate_high;
    double           default_Kp;
    double           default_Ki;
    double           default_Kd;
} gain_schedule_table_t;

#define GS_MAX_BREAKPOINTS_2D 32

typedef struct {
    double          sched_val_1;
    double          sched_val_2;
    pid_gain_set_t  gains;
    char            label[32];
} schedule_entry_2d_t;

typedef struct {
    schedule_entry_2d_t entries[GS_MAX_BREAKPOINTS_2D * GS_MAX_BREAKPOINTS_2D];
    uint32_t            rows;
    uint32_t            cols;
    sched_var_type_t    sched_var_1_type;
    sched_var_type_t    sched_var_2_type;
    interp_method_t     interp_method;
    bool                extrapolate;
    double              default_Kp;
    double              default_Ki;
    double              default_Kd;
} gain_schedule_table_2d_t;

typedef struct {
    char           tag[32];
    char           description[128];
    gs_pid_form_t  pid_form;
    double         Kp_current;
    double         Ki_current;
    double         Kd_current;
    double         Ti_current;
    double         Td_current;
    double         N_current;
    double         b_current;
    double         c_current;
    double         setpoint;
    double         process_variable;
    double         error;
    double         prev_error;
    double         integral;
    double         derivative;
    double         prev_pv;
    double         control_output;
    double         prev_control_output;
    double         scheduling_variable;
    double         prev_sched_variable;
    uint32_t       active_entry_index;
    double         saturation_high;
    double         saturation_low;
    bool           saturated;
    double         tracking_input;
    double         dt;
    double         elapsed_time;
    uint64_t       step_count;
    double         deriv_filter_state;
    double         sched_filter_state;
    double         last_Kp_change;
    double         last_Ki_change;
    double         last_Kd_change;
    uint64_t       schedule_switch_count;
} gs_pid_state_t;

typedef struct {
    gs_pid_form_t  pid_form;
    double         sample_time;
    double         max_integral;
    double         min_integral;
    double         deriv_filter_freq;
    double         sched_filter_freq;
    double         gain_smoothing_factor;
    double         min_sampling_ratio;
} gs_design_config_t;

void gs_table_init(gain_schedule_table_t *table, sched_var_type_t svt);

bool gs_table_add_entry(gain_schedule_table_t *table,
                        double sched_val,
                        const pid_gain_set_t *gains,
                        const char *label);

void gs_table_sort_entries(gain_schedule_table_t *table);

bool gs_table_find_bracket(const gain_schedule_table_t *table,
                           double sched_val,
                           uint32_t *idx_low,
                           uint32_t *idx_high);

bool gs_table_validate(const gain_schedule_table_t *table,
                       char *errmsg, size_t errmsg_size);

bool gs_table_remove_entry(gain_schedule_table_t *table, uint32_t index);

void gs_table_clear(gain_schedule_table_t *table);

uint32_t gs_table_count(const gain_schedule_table_t *table);

bool gs_table_clone(const gain_schedule_table_t *src, gain_schedule_table_t *dst);

const schedule_entry_t *gs_table_get_entry(const gain_schedule_table_t *table,
                                             uint32_t index);

int32_t gs_table_find_nearest(const gain_schedule_table_t *table, double sched_val);

void gs_table_set_defaults(gain_schedule_table_t *table, double Kp, double Ki, double Kd);

void gs_table_set_extrapolation(gain_schedule_table_t *table,
                                 bool extrap_low, bool extrap_high);

void gs_table_set_interp_method(gain_schedule_table_t *table, interp_method_t method);

#ifdef __cplusplus
}
#endif

#endif /* GAIN_SCHEDULE_CORE_H */
