/**
 * @file cascade_stability.h
 * @brief Cascade Stability Analysis - Nyquist, Gain/Phase Margins, Robustness
 *
 * Module: mini-cascade-control-primary-secondary
 * Knowledge Coverage: L4 Engineering Laws, L5 Algorithms/Methods
 *
 * Cascade control introduces two feedback loops, creating a more complex
 * stability picture than single-loop control. The overall cascade system
 * is stable iff both the inner and outer loops are individually stable
 * AND the loops do not adversely interact.
 *
 * Characteristic equation for cascade:
 *   1 + Gc2(s)*Gp2(s) + Gc1(s)*Gc2(s)*Gp1(s)*Gp2(s) = 0
 *
 * Reference: Seborg et al. (2016) Chapter 16
 * Curriculum: MIT 6.302, Berkeley ME233, CMU 24-677
 */

#ifndef CASCADE_STABILITY_H
#define CASCADE_STABILITY_H

#include "cascade_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Frequency Response Computation */

void cascade_fopdt_frequency_response(const cascade_fopdt_model_t *model,
                                       double w, double *mag, double *phase_rad);

void cascade_pid_frequency_response(const cascade_pid_params_t *pid,
                                     double w, double ts,
                                     double *mag, double *phase_rad);

/* Open-Loop and Closed-Loop Cascade Response */

void cascade_open_loop_response(const cascade_pid_params_t *pri_pid,
                                 const cascade_pid_params_t *sec_pid,
                                 const cascade_fopdt_model_t *pri_proc,
                                 const cascade_fopdt_model_t *sec_proc,
                                 double w, double ts,
                                 double *mag, double *phase_rad);

void cascade_sensitivity_function(const cascade_pid_params_t *pri_pid,
                                   const cascade_pid_params_t *sec_pid,
                                   const cascade_fopdt_model_t *pri_proc,
                                   const cascade_fopdt_model_t *sec_proc,
                                   double w, double ts,
                                   double *mag, double *phase_rad);

/* Stability Margin Computation */

int cascade_compute_stability(const cascade_pid_params_t *pri_pid,
                               const cascade_pid_params_t *sec_pid,
                               const cascade_fopdt_model_t *pri_proc,
                               const cascade_fopdt_model_t *sec_proc,
                               double ts, int n_freq,
                               cascade_stability_t *result);

int cascade_check_nyquist_stability(const cascade_pid_params_t *pri_pid,
                                     const cascade_pid_params_t *sec_pid,
                                     const cascade_fopdt_model_t *pri_proc,
                                     const cascade_fopdt_model_t *sec_proc,
                                     double ts, int n_freq);

int cascade_check_stability_simple(const cascade_pid_params_t *pri_pid,
                                    const cascade_pid_params_t *sec_pid,
                                    const cascade_fopdt_model_t *pri_proc,
                                    const cascade_fopdt_model_t *sec_proc,
                                    double ts, int n_freq,
                                    cascade_stability_t *stability);

/* Robustness Analysis */

double cascade_robustness_index(const cascade_stability_t *stability);

double cascade_compute_modulus_margin(const cascade_pid_params_t *pri_pid,
                                       const cascade_pid_params_t *sec_pid,
                                       const cascade_fopdt_model_t *pri_proc,
                                       const cascade_fopdt_model_t *sec_proc,
                                       double ts, int n_freq);

int cascade_stability_delay_impact(double delay_sec,
                                    const cascade_pid_params_t *pri_pid,
                                    const cascade_pid_params_t *sec_pid,
                                    const cascade_fopdt_model_t *pri_proc,
                                    const cascade_fopdt_model_t *sec_proc,
                                    double ts, double *gm_db);

/* Secondary Loop Stability */

int cascade_secondary_stability(const cascade_pid_params_t *sec_pid,
                                 const cascade_fopdt_model_t *sec_proc,
                                 double ts, int n_freq,
                                 cascade_stability_t *result);

double cascade_max_allowed_gain(const cascade_pid_params_t *base_pid,
                                 const cascade_fopdt_model_t *process,
                                 double ts, double kp_min, double kp_max,
                                 int n_freq);

int cascade_min_phase_margin_primary(const double *gains,
                                      const double *ti_vals,
                                      int n_points,
                                      const cascade_fopdt_model_t *model,
                                      double ts, double *worst_pm);

#ifdef __cplusplus
}
#endif

#endif /* CASCADE_STABILITY_H */
