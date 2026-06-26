#ifndef GAIN_SCHEDULE_DESIGN_H
#define GAIN_SCHEDULE_DESIGN_H

#include "gain_schedule_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file    gain_schedule_design.h
 * @brief   Gain Schedule Design Rules L4/L5
 *
 * Implements systematic design procedures for gain-scheduled PID controllers.
 * The frozen-parameter method designs local PID controllers at each operating
 * point, then interpolates gains. Design rules include Ziegler-Nichols,
 * Cohen-Coon, IMC, SIMC, and AMIGO, adapted for each operating point.
 *
 * L4 Standards:
 *   - Ziegler-Nichols frequency-domain method (1942)
 *   - Cohen-Coon process reaction curve method (1953)
 *   - IMC-based tuning (Morari & Zafiriou 1989)
 *   - SIMC method (Skogestad 2003)
 *   - AMIGO method (Astrom & Hagglund 2004)
 *
 * L5 Algorithms:
 *   - Frozen-parameter linearization at each grid point
 *   - Operating-point-based process identification
 *   - Interpolation-aware gain design (smoothness constraints)
 *   - Stability margin preservation across schedule
 *
 * References:
 *   Astrom & Hagglund, "Revisiting the Ziegler-Nichols...", JPC, 2004.
 *   Skogestad, "Simple analytic rules...", JPC, 2003.
 *   Apkarian & Adams, "Advanced Gain-Scheduling...", 1998.
 */

typedef struct {
    double ultimate_gain;
    double ultimate_period;
    double dead_time;
    double time_constant;
    double static_gain;
    bool   identified;
} gs_process_info_t;

/**
 * Ziegler-Nichols PID design from ultimate gain and period.
 * PID: Kp=0.6*Ku, Ti=0.5*Pu, Td=0.125*Pu
 */
pid_gain_set_t gs_design_zn_pid(double Ku, double Pu);

/**
 * Ziegler-Nichols PI design from ultimate gain and period.
 * PI: Kp=0.45*Ku, Ti=0.85*Pu
 */
pid_gain_set_t gs_design_zn_pi(double Ku, double Pu);

/**
 * Tyreus-Luyben PID for chemical processes (conservative).
 * PID: Kp=0.45*Ku, Ti=2.2*Pu, Td=Pu/6.3
 */
pid_gain_set_t gs_design_tyreus_luyben_pid(double Ku, double Pu);

/**
 * Tyreus-Luyben PI for chemical processes.
 * PI: Kp=Ku/3.2, Ti=2.2*Pu
 */
pid_gain_set_t gs_design_tyreus_luyben_pi(double Ku, double Pu);

/**
 * Cohen-Coon PID from FOPDT model (K, tau, L).
 */
pid_gain_set_t gs_design_cohen_coon_pid(double K, double tau, double L);

/**
 * Cohen-Coon PI from FOPDT model.
 */
pid_gain_set_t gs_design_cohen_coon_pi(double K, double tau, double L);

/**
 * IMC-based PID for FOPDT process.
 * lambda: desired closed-loop time constant (lambda > 0.2*tau for robustness)
 */
pid_gain_set_t gs_design_imc_pid(double K, double tau, double L,
                                  double lambda);

/**
 * SIMC (Skogestad IMC) PI for FOPDT process.
 * tau_c: desired closed-loop time constant
 */
pid_gain_set_t gs_design_simc_pi(double K, double tau, double L,
                                  double tau_c);

/**
 * AMIGO PID (Approximate M-constrained Integral Gain Optimization).
 */
pid_gain_set_t gs_design_amigo_pid(double K, double tau, double L);

/**
 * AMIGO PI.
 */
pid_gain_set_t gs_design_amigo_pi(double K, double tau, double L);

/**
 * Design a complete 1D gain schedule using frozen-parameter method.
 * For each breakpoint:
 *   1. Linearize process model at operating point
 *   2. Design local PID using specified tuning rule
 *   3. Populate schedule entry
 *
 * tuning_rule: 0=ZN, 1=TL, 2=CC, 3=IMC, 4=SIMC, 5=AMIGO
 * process_params: array of (K, tau, L) for each operating point
 * sched_values: array of scheduling variable values
 * n: number of operating points
 */
bool gs_design_frozen_parameter(gain_schedule_table_t *table,
                                 int tuning_rule,
                                 const double *sched_values,
                                 const double *K_array,
                                 const double *tau_array,
                                 const double *L_array,
                                 uint32_t n);

/**
 * Validate that the designed schedule maintains stability margins
 * across all operating points. Checks:
 *   - Monotonicity of gain variation
 *   - Maximum gain ratio between adjacent points
 *   - Minimum phase margin across schedule
 */
bool gs_design_validate_margins(const gain_schedule_table_t *table,
                                double min_phase_margin,
                                double max_gain_ratio,
                                char *errmsg, size_t errmsg_size);

/**
 * Compute the gain ratio between adjacent schedule entries.
 * Returns the maximum ratio found, or 0.0 if table has < 2 entries.
 */
double gs_design_max_gain_ratio(const gain_schedule_table_t *table);

/**
 * Split a schedule region by inserting additional breakpoints
 * where gain variation exceeds a threshold. Improves smoothness.
 * Returns number of new entries added.
 */
uint32_t gs_design_refine_grid(gain_schedule_table_t *table,
                               double max_gain_step_ratio);

/**
 * Apply a smoothing filter to the gain schedule to reduce
 * abrupt changes. Uses moving average with configurable window.
 */
void gs_design_smooth_schedule(gain_schedule_table_t *table,
                                uint32_t window_size);

/**
 * Compute local stability margins (GM, PM, Ms) for each entry
 * in the schedule based on the local FOPDT model.
 */
void gs_design_compute_local_margins(gain_schedule_table_t *table,
                                      const double *K_array,
                                      const double *tau_array,
                                      const double *L_array,
                                      uint32_t n);

#ifdef __cplusplus
}
#endif
#endif /* GAIN_SCHEDULE_DESIGN_H */
