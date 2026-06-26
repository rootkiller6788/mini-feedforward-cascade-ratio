#include "feedforward_defs.h"
#include "feedforward_static.h"
#include "feedforward_dynamic.h"
#include "feedforward_models.h"
#include "feedforward_combined.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdlib.h>

/**
 * @file feedforward_tuning.c
 * @brief Feedforward design methods, tuning rules, and optimization
 *
 * Knowledge: L4 Engineering Laws, L5 Algorithms/Methods, L6 Canonical Problems
 *
 * Implements:
 * - Direct synthesis method for feedforward-feedback combination
 * - Feedforward parameter tuning by optimization
 * - Robustness analysis (gain/phase margins for FF+FB)
 * - Disturbance propagation analysis
 * - Combined FF-FB design tradeoff
 *
 * References:
 *   Seborg et al. (2016) §15.4-15.5
 *   Brosilow & Joseph (2002) "Techniques of Model-Based Control" §5
 *   Skogestad & Postlethwaite (2005) "Multivariable Feedback Control" §5
 */

/* ============================================================================
 * L5: Direct synthesis feedforward design
 * ============================================================================ */

/**
 * @brief Direct synthesis feedforward design for FOPDT process
 *
 * Given desired closed-loop response G_cl_des(s), compute the feedforward
 * controller transfer function:
 *
 * Gff(s) = (G_cl_des^(-1)(s) - 1) * Ga(s)  [feedforward path]
 *         where Ga holds Gp and Gd appropriately.
 *
 * For the disturbance rejection path:
 * Gd(s) + Gp(s) * Gff(s) = 0  (perfect rejection)
 * => Gff(s) = -Gd(s)/Gp(s)
 *
 * @param process     FOPDT process model
 * @param dist        Disturbance model (FOPDT)
 * @param lambda_cl   Desired closed-loop time constant [s]
 * @param action      Controller action direction
 * @param Kff_out     Output: static feedforward gain
 * @param T_lead_out  Output: lead time constant
 * @param T_lag_out   Output: lag time constant
 * @return 0 on success
 */
int ff_direct_synthesis_fopdt(const fopdt_t *process, const dist_model_t *dist,
                              double lambda_cl, action_t action,
                              double *Kff_out, double *T_lead_out, double *T_lag_out)
{
    if (!process || !dist || !Kff_out || !T_lead_out || !T_lag_out) return -1;
    if (fabs(process->Kp) < FF_EPSILON) return -1;
    if (lambda_cl < FF_TAU_MIN) lambda_cl = FF_TAU_MIN;

    /* Direct synthesis approach:
     *
     * Desired disturbance rejection: e(s)/d(s) = Gd(s) / (1 + Gp(s)*Gc(s))
     *
     * With feedforward: e(s)/d(s) = (Gd(s) + Gp(s)*Gff(s)) / (1 + Gp(s)*Gc(s))
     *
     * Setting numerator to zero for perfect rejection:
     * Gff(s) = -Gd(s)/Gp(s)
     *
     * But when Gp has dead time > Gd dead time, the inverse is non-causal.
     * In that case, use the internal model control (IMC) approach:
     * Gff(s) = -Gd(s) * Gp_inv(s) * F(s)
     * where F(s) is a filter that makes Gff(s) proper.
     *
     * Filter: F(s) = 1/(lambda_cl*s + 1)^n
     * where n is chosen to make the overall transfer function proper.
     */

    /* Gff(s) = -(Kd/Kp) * (tau_p*s+1)/(tau_d*s+1) *
     *          e^((theta_p-theta_d)*s) * 1/(lambda_cl*s+1)
     *
     * The filter adds extra lag to ensure properness.
     */

    /* Static gain */
    *Kff_out = -dist->Kd / process->Kp;
    *Kff_out *= (double)action;

    /* Lead = process time constant (to cancel process dynamics) */
    *T_lead_out = process->tau;

    /* Lag = disturbance time constant + desired closed-loop time constant
     * The extra lag from lambda_cl makes the controller proper and adds robustness */
    *T_lag_out = dist->tau_d + lambda_cl;

    if (*T_lag_out < FF_TAU_MIN) *T_lag_out = FF_TAU_MIN;
    if (*T_lead_out < 0.0) *T_lead_out = 0.0;

    return 0;
}

/* ============================================================================
 * L5: Feedforward tuning via optimization (pattern search)
 * ============================================================================ */

/**
 * @brief Simple cost function for feedforward tuning
 *
 * Cost = ISE + lambda * ISU (integral of squared control action)
 *
 * @param data        Time series data: [t0, y0, u0, d0, t1, y1, u1, d1, ...]
 * @param n_points    Number of data points
 * @param Kff         Static feedforward gain
 * @param T_lead      Lead time constant
 * @param T_lag       Lag time constant
 * @param lambda_u    Control effort penalty weight
 * @return Cost value (lower is better)
 */
static double ff_tuning_cost(const double *data, int n_points,
                             double Kff, double T_lead, double T_lag,
                             double lambda_u)
{
    if (!data || n_points < 4) return 1e12;

    /* Simulate combined FF+FB system over the data and compute ISE + lambda*ISU.
     * For simplicity, simulate static + dynamic FF and compare with
     * the "ideal" compensation.
     *
     * Data format per point: [t, y, u, d] (4 doubles per sample)
     */
    double ise = 0.0, isu = 0.0;
    lead_lag_t ll;
    lead_lag_init(&ll, Kff, T_lead, T_lag, 0.1); /* Assume Ts = 0.1s */

    for (int i = 0; i < n_points; i++) {
        double y_i = data[i * 4 + 1];  /* PV (error from setpoint) */
        double u_i = data[i * 4 + 2];  /* Manipulated variable */
        double d_i = data[i * 4 + 3];  /* Disturbance */

        /* Simulate what u_ff would be */
        double u_ff_dyn = lead_lag_step(&ll, d_i);

        ise += y_i * y_i;
        isu += (u_i - u_ff_dyn) * (u_i - u_ff_dyn);
    }

    return ise + lambda_u * isu;
}

/**
 * @brief Tune feedforward parameters via Hooke-Jeeves pattern search
 *
 * Direct search optimization for (Kff, T_lead, T_lag) that minimizes
 * the combined disturbance rejection + control effort cost.
 *
 * Algorithm: Hooke & Jeeves (1961) pattern search with
 * exploratory moves and pattern moves.
 *
 * @param data         Time series data (4 doubles per point: t, y, u, d)
 * @param n_points     Number of data points
 * @param Kff_init     Initial guess for static FF gain
 * @param T_lead_init  Initial guess for lead time constant
 * @param T_lag_init   Initial guess for lag time constant
 * @param lambda_u     Control effort penalty weight
 * @param Kff_opt      Output: optimized static FF gain
 * @param T_lead_opt   Output: optimized lead
 * @param T_lag_opt    Output: optimized lag
 * @param max_iter     Maximum iterations
 * @return Final cost
 */
double ff_tune_pattern_search(const double *data, int n_points,
                              double Kff_init, double T_lead_init, double T_lag_init,
                              double lambda_u,
                              double *Kff_opt, double *T_lead_opt, double *T_lag_opt,
                              int max_iter)
{
    if (!data || !Kff_opt || !T_lead_opt || !T_lag_opt || n_points < 4)
        return 1e12;
    if (max_iter < 1) max_iter = 50;

    /* Initial base point */
    double Kff_b = Kff_init;
    double Tl_b = T_lead_init;
    double Tg_b = T_lag_init;
    double cost_b = ff_tuning_cost(data, n_points, Kff_b, Tl_b, Tg_b, lambda_u);

    /* Step sizes */
    double dK = fabs(Kff_b) * 0.1;
    if (dK < 0.01) dK = 0.01;
    double dTl = fabs(Tl_b) * 0.1;
    if (dTl < FF_TAU_MIN) dTl = 0.1;
    double dTg = fabs(Tg_b) * 0.1;
    if (dTg < FF_TAU_MIN) dTg = 0.1;

    double tol = 1e-4;

    for (int iter = 0; iter < max_iter; iter++) {
        /* Exploratory move */
        double best_cost = cost_b;
        double best_K = Kff_b, best_Tl = Tl_b, best_Tg = Tg_b;

        /* Try Kff +/- dK */
        double c = ff_tuning_cost(data, n_points, Kff_b + dK, Tl_b, Tg_b, lambda_u);
        if (c < best_cost) { best_cost = c; best_K = Kff_b + dK; }

        c = ff_tuning_cost(data, n_points, Kff_b - dK, Tl_b, Tg_b, lambda_u);
        if (c < best_cost) { best_cost = c; best_K = Kff_b - dK; }

        /* Try T_lead +/- dTl */
        c = ff_tuning_cost(data, n_points, best_K, Tl_b + dTl, Tg_b, lambda_u);
        if (c < best_cost) { best_cost = c; best_Tl = Tl_b + dTl; }

        c = ff_tuning_cost(data, n_points, best_K, Tl_b - dTl, Tg_b, lambda_u);
        if (c < best_cost) { best_cost = c; best_Tl = Tl_b - dTl; }

        /* Try T_lag +/- dTg */
        c = ff_tuning_cost(data, n_points, best_K, best_Tl, Tg_b + dTg, lambda_u);
        if (c < best_cost) { best_cost = c; best_Tg = Tg_b + dTg; }

        c = ff_tuning_cost(data, n_points, best_K, best_Tl, Tg_b - dTg, lambda_u);
        if (c < best_cost) { best_cost = c; best_Tg = Tg_b - dTg; }

        /* If no improvement, reduce step size */
        if (best_cost >= cost_b - tol) {
            dK *= 0.5;
            dTl *= 0.5;
            dTg *= 0.5;
            if (dK < tol && dTl < tol && dTg < tol) break;
            continue;
        }

        /* Pattern move: extrapolate in the direction of improvement */
        double Kff_p = best_K + (best_K - Kff_b);
        double Tl_p = best_Tl + (best_Tl - Tl_b);
        double Tg_p = best_Tg + (best_Tg - Tg_b);

        /* Check bounds */
        if (Kff_p > FF_GAIN_MAX) Kff_p = FF_GAIN_MAX;
        if (Kff_p < -FF_GAIN_MAX) Kff_p = -FF_GAIN_MAX;
        if (Tl_p < 0.0) Tl_p = 0.0;
        if (Tg_p < FF_TAU_MIN) Tg_p = FF_TAU_MIN;

        double cost_p = ff_tuning_cost(data, n_points, Kff_p, Tl_p, Tg_p, lambda_u);

        if (cost_p < best_cost) {
            /* Pattern move successful */
            Kff_b = Kff_p; Tl_b = Tl_p; Tg_b = Tg_p;
            cost_b = cost_p;
        } else {
            /* Pattern move failed, use best from exploratory */
            Kff_b = best_K; Tl_b = best_Tl; Tg_b = best_Tg;
            cost_b = best_cost;
        }
    }

    *Kff_opt = Kff_b;
    *T_lead_opt = Tl_b;
    *T_lag_opt = Tg_b;

    return cost_b;
}

/* ============================================================================
 * L4: Robustness analysis for feedforward-feedback systems
 * ============================================================================ */

/**
 * @brief Compute the effect of feedforward gain error on closed-loop stability
 *
 * Feedforward does not affect closed-loop stability (it is outside the
 * feedback loop), BUT if the feedforward is too aggressive, it can
 * cause actuator saturation which indirectly affects stability.
 *
 * This function computes the closed-loop characteristic equation with
 * feedforward-feedback interaction to check for potential issues.
 *
 * Closed-loop characteristic equation:
 * 1 + Gp(s) * Gc(s) = 0  (unchanged by FF)
 *
 * However, the effective open-loop gain seen by the feedback controller
 * can be affected if FF output causes saturation.
 *
 * @param Kp          Process gain
 * @param Kc          Feedback controller gain
 * @param Kff         Feedforward gain
 * @param Kd          Disturbance gain
 * @param saturation_margin  Fraction of output range available for feedback
 * @return Gain margin factor (> 1 = safe, < 1 = risk of saturation)
 */
double ff_robustness_gain_margin(double Kp, double Kc, double Kff,
                                 double Kd, double saturation_margin)
{
    (void)saturation_margin; /* Reserved for future saturation analysis */
    /* Effective open-loop gain for feedback:
     * Without FF: L(s) = Gp(s) * Gc(s), DC gain = Kp * Kc
     * With FF: the feedback only needs to correct residual error.
     *
     * The residual gain mismatch factor:
     * residual_factor = |1 + Kp*Kff/Kd|
     *
     * If residual_factor is small (< 0.2), FF handles most of the
     * disturbance, and feedback gain can be reduced or left at original.
     *
     * If residual_factor is close to 1, FF provides little benefit.
     */

    if (fabs(Kd) < FF_EPSILON) return 1.0;

    double residual = fabs(1.0 + Kp * Kff / Kd);

    /* Available feedback authority considering FF output occupancy */
    double ff_fraction = fabs(Kff) / (fabs(Kff) + fabs(Kc) + FF_EPSILON);

    /* Gain margin factor: how much saturation margin is consumed */
    double margin_factor = 1.0 / (ff_fraction + FF_EPSILON);

    /* Scale by the residual factor */
    if (residual > FF_EPSILON) {
        margin_factor *= 1.0 / residual;
    }

    return margin_factor;
}

/**
 * @brief Compute the disturbance propagation transfer function
 *
 * With feedforward, the disturbance-to-output transfer function becomes:
 *
 * T_dy(s) = Gd(s) + Gp(s) * Gff(s)
 *
 * Without FF (Gff=0): T_dy(s) = Gd(s) — full disturbance effect
 * With perfect FF:  T_dy(s) = 0 — complete rejection
 *
 * This function evaluates |T_dy(jw)| at specified frequencies to
 * analyze residual disturbance sensitivity.
 *
 * @param process       FOPDT process model
 * @param dist          Disturbance model
 * @param Kff           Static feedforward gain
 * @param T_lead        Lead time constant [s]
 * @param T_lag         Lag time constant [s]
 * @param omega         Angular frequency [rad/s]
 * @return Magnitude of residual disturbance sensitivity |T_dy(jw)|
 */
double ff_disturbance_sensitivity(const fopdt_t *process, const dist_model_t *dist,
                                  double Kff, double T_lead, double T_lag,
                                  double omega)
{
    if (!process || !dist) return 1e12;

    /* Gd(jw) = Kd * e^(-j*w*theta_d) / (j*w*tau_d + 1) */
    double Gd_mag = fabs(dist->Kd) / sqrt(omega * omega * dist->tau_d * dist->tau_d + 1.0);

    /* Gp(jw) = Kp * e^(-j*w*theta_p) / (j*w*tau_p + 1) */
    double Gp_mag = fabs(process->Kp) / sqrt(omega * omega * process->tau * process->tau + 1.0);

    /* Gff(jw) = Kff * (j*w*T_lead + 1) / (j*w*T_lag + 1) */
    double Gff_mag = fabs(Kff) * sqrt(omega * omega * T_lead * T_lead + 1.0)
                     / sqrt(omega * omega * T_lag * T_lag + 1.0);

    /* Phase contributions are ignored for worst-case magnitude bound.
     * |T_dy| <= |Gd| + |Gp|*|Gff| (triangle inequality) */
    double worst_case = Gd_mag + Gp_mag * Gff_mag;

    /* Better estimate: use the actual vector addition with phase estimates.
     * For simplicity, assume worst-case alignment (phases add constructively). */
    return worst_case;
}

/* ============================================================================
 * L6: Feedforward-feedback interaction analysis
 * ============================================================================ */

/**
 * @brief Compute the effective disturbance rejection with combined FF+FB
 *
 * The closed-loop disturbance response for combined FF+FB:
 *
 * e(s)/d(s) = (Gd(s) + Gp(s)*Gff(s)) / (1 + Gp(s)*Gc(s))
 *
 * This quantifies how much the combined system rejects disturbances
 * compared to open-loop (no control) and feedback-only cases.
 *
 * @param process       FOPDT process model
 * @param dist          Disturbance model
 * @param Kff           FF static gain
 * @param T_lead        FF lead time constant
 * @param T_lag         FF lag time constant
 * @param Kc            Feedback controller gain (PI/PID)
 * @param Ti            Feedback integral time [s]
 * @param Ts            Sample time [s]
 * @param t_sim         Simulation duration [s]
 * @param d_step        Disturbance step magnitude
 * @param ise_out       Output: ISE for combined FF+FB
 * @param ise_fb_only   Output: ISE for FB only (no FF)
 * @return Disturbance rejection improvement factor (ISE_fb/ISE_combined)
 */
double ff_combined_rejection_sim(const fopdt_t *process, const dist_model_t *dist,
                                 double Kff, double T_lead, double T_lag,
                                 double Kc, double Ti, double Ts,
                                 double t_sim, double d_step,
                                 double *ise_out, double *ise_fb_only)
{
    if (!process || !dist) return 1.0;

    int n_steps = (int)(t_sim / Ts);
    if (n_steps < 10) n_steps = 10;
    if (n_steps > 10000) n_steps = 10000;

    /* Process state variables */
    double y_ff = 0.0;       /* PV with FF+FB */
    double y_fb = 0.0;       /* PV with FB only */
    double u_ff = 0.0, u_fb = 0.0;
    double e_int_ff = 0.0, e_int_fb = 0.0;

    /* Lead-lag for FF */
    lead_lag_t ll;
    lead_lag_init(&ll, Kff, T_lead, T_lag, Ts);

    double ise_combi = 0.0, ise_fbonly = 0.0;

    for (int k = 0; k < n_steps; k++) {
        double t = k * Ts;

        /* Disturbance: step at t=Ts (after one sample to simulate) */
        double d = 0.0;
        if (t >= Ts) d = d_step;

        /* --- FF+FB System --- */
        double e_ff = 0.0 - y_ff; /* Setpoint = 0 */
        double u_fb_ff = Kc * (e_ff + e_int_ff / Ti);
        double u_ff_dyn = lead_lag_step(&ll, d);
        double u_total_ff = u_fb_ff + u_ff_dyn;

        /* Process: dx/dt = (Kp*u - x)/tau */
        double dx_ff = (process->Kp * u_total_ff + dist->Kd * d - y_ff) / process->tau;
        y_ff += dx_ff * Ts;
        e_int_ff += e_ff * Ts;

        /* --- FB Only System --- */
        double e_fb = 0.0 - y_fb;
        double u_total_fb = Kc * (e_fb + e_int_fb / Ti);

        double dx_fb = (process->Kp * u_total_fb + dist->Kd * d - y_fb) / process->tau;
        y_fb += dx_fb * Ts;
        e_int_fb += e_fb * Ts;

        ise_combi += e_ff * e_ff * Ts;
        ise_fbonly += e_fb * e_fb * Ts;

        (void)u_ff;
        (void)u_fb;
    }

    if (ise_out) *ise_out = ise_combi;
    if (ise_fb_only) *ise_fb_only = ise_fbonly;

    if (ise_combi < FF_EPSILON) return 1e6;

    return ise_fbonly / ise_combi;
}

/* ============================================================================
 * L6: Step disturbance simulation for design verification
 * ============================================================================ */

/**
 * @brief Simulate closed-loop system response to step disturbance
 *
 * Full nonlinear simulation of FOPDT process with combined feedforward-feedback
 * control subjected to a step disturbance.
 *
 * @param process      FOPDT process model
 * @param dist         Disturbance model
 * @param ff           Configured feedforward controller
 * @param Kc           Feedback controller gain
 * @param Ti           Feedback integral time [s]
 * @param Td           Feedback derivative time [s]
 * @param d_step       Disturbance step magnitude
 * @param t_sim        Simulation duration [s]
 * @param Ts           Simulation time step [s]
 * @param t_out        Output: time array [n_steps]
 * @param y_out        Output: PV response [n_steps]
 * @param u_out        Output: control signal [n_steps]
 * @param max_steps    Maximum number of steps in output arrays
 * @return Number of simulation steps
 */
int ff_simulate_step_disturbance(const fopdt_t *process, const dist_model_t *dist,
                                 const feedforward_t *ff,
                                 double Kc, double Ti, double Td,
                                 double d_step, double t_sim, double Ts,
                                 double *t_out, double *y_out, double *u_out,
                                 int max_steps)
{
    if (!process || !dist || !ff || !t_out || !y_out || !u_out)
        return 0;

    int n_steps = (int)(t_sim / Ts) + 1;
    if (n_steps > max_steps) n_steps = max_steps;
    if (n_steps < 2) return 0;

    /* Initialize simulation */
    double y = 0.0;       /* Process variable (deviation from setpoint) */
    double e_prev = 0.0;
    double e_int = 0.0;   /* Integral of error */
    double u = 0.0;

    /* Copy FF for local modification */
    feedforward_t ff_local = *ff;
    ff_local.Ts = Ts;
    ff_local.initialized = 1;

    for (int k = 0; k < n_steps; k++) {
        double t = k * Ts;
        t_out[k] = t;

        /* Disturbance: step at t = 0.5*t_sim to observe response */
        double d = 0.0;
        if (t >= 0.5 * t_sim) d = d_step;

        /* Error */
        double e = 0.0 - y; /* Setpoint = 0 (deviation form) */

        /* Feedback control (PI) */
        double u_fb = Kc * (e + e_int / Ti + Td * (e - e_prev) / Ts);

        /* Feedforward control */
        double u_ff = feedforward_step(&ff_local, d);

        /* Total control signal */
        u = u_fb + u_ff;

        /* Process dynamics: y[k+1] = y[k] + Ts*(Kp*u + Kd*d - y)/tau
         * (Explicit Euler integration of FOPDT) */
        double dy = (process->Kp * u + dist->Kd * d - y) / process->tau;
        y += dy * Ts;

        /* Update integral and previous error */
        e_int += e * Ts;
        e_prev = e;

        y_out[k] = y;
        u_out[k] = u;
    }

    return n_steps;
}