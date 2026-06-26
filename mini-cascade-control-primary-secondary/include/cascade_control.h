/**
 * @file cascade_control.h
 * @brief Cascade Control Loop Management API
 *
 * Runtime management of cascade control pairs:
 * - Mode switching (Manual ↔ Auto ↔ Cascade)
 * - Primary-secondary execution coordination
 * - Bumpless transfer for cascade engagement/disengagement
 * - RGA decoupling, split-range, gain scheduling
 * - Performance assessment and open-loop detection
 *
 * Curriculum: MIT 6.302, Stanford ENGR205, Purdue ME575
 */

#ifndef CASCADE_CONTROL_H
#define CASCADE_CONTROL_H

#include "cascade_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Cascade pair lifecycle */
void cascade_pair_init(cascade_config_t *cascade,
                       const cascade_fopdt_model_t *primary_model,
                       const cascade_fopdt_model_t *secondary_model);

void cascade_set_mode(cascade_config_t *cascade, cascade_mode_t new_mode);

void cascade_execute_primary(cascade_config_t *cascade);

void cascade_execute_secondary(cascade_config_t *cascade);

/* Bumpless transfers */
void cascade_bumpless_manual_to_auto(cascade_config_t *cascade);

void cascade_bumpless_auto_to_cascade(cascade_config_t *cascade);

void cascade_bumpless_cascade_to_auto(cascade_config_t *cascade);

void cascade_bumpless_cascade_to_manual(cascade_config_t *cascade);

/* RGA analysis and decoupling */
void rga_compute_2x2(double K11, double K12, double K21, double K22,
                     double *lambda11, double *niederlinski, int *pairing_safe);

void decoupler_design_2x2(double K11, double K12, double K21, double K22,
                          double *d12, double *d21);

void decoupler_apply_2x2(double u1_raw, double u2_raw,
                          double d12, double d21,
                          double *u1_out, double *u2_out);

/* Performance assessment */
void cascade_performance_assess(const cascade_pid_controller_t *pid,
                                cascade_performance_t *perf,
                                double total_time,
                                double setpoint);

/* Split-range output */
void split_range_compute(double controller_output,
                          int split_type,
                          double split_point,
                          double *output_a, double *output_b);

/* Gain scheduling */
void gain_schedule_init(cascade_adaptive_state_t *gs);

void gain_schedule_add_point(cascade_adaptive_state_t *gs,
                              double operating_point,
                              double gain, double ti, double td);

void gain_schedule_update(cascade_adaptive_state_t *gs,
                           double operating_point,
                           cascade_pid_params_t *params);

/* L6: Canonical cascade patterns */
void temp_cascade_configure(cascade_config_t *cascade,
                             double reactor_tau, double jacket_tau,
                             double reactor_gain, double jacket_gain);

void level_flow_cascade_configure(cascade_config_t *cascade,
                                   double tank_area, double max_flow,
                                   double level_sp, double level_pv);

/* Open-loop detection and alarms */
int cascade_detect_open_loop(const cascade_pid_controller_t *pid,
                              double pv, double mv, double sp,
                              double pv_noise_threshold,
                              double mv_stuck_band);

int cascade_check_alarms(double pv,
                          double lo_lo, double lo,
                          double hi, double hi_hi);

#ifdef __cplusplus
}
#endif

#endif /* CASCADE_CONTROL_H */
