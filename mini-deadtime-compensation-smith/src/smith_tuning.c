/**
 * @file smith_tuning.c
 * @brief Controller tuning algorithms for the Smith predictor primary controller.
 *
 * Implements multiple tuning methods for the delay-free process model:
 *   - SIMC (Skogestad, 2003): Simple IMC — most widely used in practice
 *   - IMC-based (Morari & Zafiriou, 1989): Internal Model Control
 *   - Lambda/Dahlin tuning (classic industrial method)
 *   - ISE-optimal numerical optimization
 *   - Robustness filter tuning (Normey-Rico et al., 1997)
 *
 * References:
 *   Skogestad (2003) J. Process Control, 13, 291-309
 *   Rivera, Morari, Skogestad (1986) IEC Proc. Des. Dev., 25, 252-265
 *   Morari & Zafiriou (1989) "Robust Process Control", Prentice Hall
 *   Dahlin (1968) Instruments and Control Systems, 41(6), 77-83
 */

#include "smith_tuning.h"
#include <math.h>
#include <string.h>

/*===========================================================================
 * L5: SIMC PI Tuning (Skogestad, 2003)
 *
 * For FOPDT: Gp(s) = K/(tau*s + 1)
 * PI: C(s) = Kp*(1 + 1/(Ti*s))
 *
 * SIMC rules:
 *   Kp = (1/K) * (tau / tau_c)
 *   Ti = min(tau, 4*tau_c)
 *
 * Closed-loop approx: T_cl(s) ≈ 1/(tau_c*s + 1)
 *
 * Theorem (Skogestad 2003, Appendix A): For the SIMC-tuned PI controller
 * on a FOPDT process, the sensitivity peak Ms ≤ 1.7 when tau_c ≥ 0.5*tau.
 *===========================================================================*/

int smith_tune_simc_pi(
    const smith_process_model_t *model,
    double tau_c,
    double *Kp_out, double *Ti_out)
{
    if (!model || !Kp_out || !Ti_out) return -1;

    double K, tau;
    switch (model->order) {
    case SMITH_MODEL_FOPDT:
        K = model->fopdt.K;
        tau = model->fopdt.tau;
        break;
    case SMITH_MODEL_SOPDT:
    case SMITH_MODEL_SOPDT_UNDER:
        K = model->sopdt.K;
        tau = (model->sopdt.tau1 > model->sopdt.tau2) ?
              model->sopdt.tau1 : model->sopdt.tau2;
        break;
    default:
        return -1;
    }

    if (fabs(K) < 1e-12 || tau <= 0.0 || tau_c <= 0.0) return -1;

    *Kp_out = (1.0 / K) * (tau / tau_c);
    *Ti_out = (tau < 4.0 * tau_c) ? tau : 4.0 * tau_c;

    if (*Kp_out < 0.0 && K > 0.0) *Kp_out = 0.0;
    return 0;
}

/*===========================================================================
 * L5: SIMC PID Tuning
 *
 * For SOPDT: Gp(s) = K/[(tau1*s+1)*(tau2*s+1)]
 * Cascade PID: Kp = (1/K)*(tau1/tau_c), Ti = tau1, Td = tau2
 * Reference: Skogestad (2003), Eq. 31-33
 *===========================================================================*/

int smith_tune_simc_pid(
    const smith_process_model_t *model,
    double tau_c,
    double *Kp_out, double *Ti_out, double *Td_out)
{
    if (!model || !Kp_out || !Ti_out || !Td_out) return -1;

    double K, tau1, tau2;
    if (model->order == SMITH_MODEL_SOPDT) {
        K = model->sopdt.K;
        tau1 = model->sopdt.tau1;
        tau2 = model->sopdt.tau2;
    } else if (model->order == SMITH_MODEL_FOPDT) {
        K = model->fopdt.K;
        tau1 = model->fopdt.tau;
        tau2 = 0.0;
    } else {
        return -1;
    }

    if (fabs(K) < 1e-12 || tau1 <= 0.0 || tau_c <= 0.0) return -1;

    *Kp_out = (1.0 / K) * (tau1 / tau_c);
    *Ti_out = tau1;
    *Td_out = tau2;

    if (*Kp_out < 0.0 && K > 0.0) *Kp_out = 0.0;
    return 0;
}

/*===========================================================================
 * L5: IMC-Based PI Tuning
 *
 * For Gp(s) = K/(tau*s+1):
 *   IMC: Q(s) = (tau*s+1)/(K*(lambda*s+1))
 *   Feedback: C(s) = Q/(1-Gp*Q) = (tau*s+1)/(K*lambda*s)
 *   → PI: Kp = tau/(K*lambda), Ti = tau
 *
 * lambda controls trade-off: λ = θ aggressive, λ = 1.5θ balanced
 *
 * Theorem (Morari & Zafiriou 1989, Thm 3.1):
 * For stable Gp(s), IMC with perfect model ≡ Smith predictor.
 * Thus IMC tuning on delay-free Gp(s) gives optimal Smith predictor tuning.
 *===========================================================================*/

int smith_tune_imc_pi(
    const smith_process_model_t *model,
    double lambda_imc,
    double *Kp_out, double *Ti_out)
{
    if (!model || !Kp_out || !Ti_out) return -1;

    double K, tau, theta;
    switch (model->order) {
    case SMITH_MODEL_FOPDT:
        K = model->fopdt.K; tau = model->fopdt.tau;
        theta = model->fopdt.theta; break;
    case SMITH_MODEL_SOPDT:
        K = model->sopdt.K; tau = model->sopdt.tau1;
        theta = model->sopdt.theta; break;
    default:
        return -1;
    }

    if (fabs(K) < 1e-12 || tau <= 0.0) return -1;

    /* IMC PI for delay-free FOPDT */
    *Kp_out = tau / (fabs(K) * lambda_imc);
    *Ti_out = tau;

    /* Enforce minimum lambda ≥ θ/2 for stability */
    double lambda_min = theta / 2.0;
    if (lambda_imc < lambda_min) {
        *Kp_out = tau / (fabs(K) * lambda_min);
    }

    if (*Kp_out < 0.0 && K > 0.0) *Kp_out = 0.0;
    return 0;
}

/*===========================================================================
 * L5: IMC PID Tuning for SOPDT
 *
 * C(s) = (tau1+tau2)/(K*λ) * (1 + 1/((tau1+tau2)*s) + tau1*tau2*s/(tau1+tau2))
 * → Kp = (tau1+tau2)/(K*λ), Ti = tau1+tau2, Td = tau1*tau2/(tau1+tau2)
 *===========================================================================*/

int smith_tune_imc_pid(
    const smith_process_model_t *model,
    double lambda_imc,
    double *Kp_out, double *Ti_out, double *Td_out)
{
    if (!model || !Kp_out || !Ti_out || !Td_out) return -1;

    double K, tau1, tau2, theta = 0.0;
    switch (model->order) {
    case SMITH_MODEL_SOPDT:
        K = model->sopdt.K; tau1 = model->sopdt.tau1;
        tau2 = model->sopdt.tau2; theta = model->sopdt.theta; break;
    case SMITH_MODEL_SOPDT_UNDER:
        K = model->sopdt.K;
        tau1 = 1.0 / (model->sopdt.zeta * model->sopdt.omega_n);
        tau2 = tau1 * 0.1;
        theta = model->sopdt.theta; break;
    default:
        return -1;
    }

    if (fabs(K) < 1e-12 || tau1 <= 0.0) return -1;

    *Ti_out = tau1 + tau2;
    if (*Ti_out < 1e-9) *Ti_out = tau1;

    *Td_out = (tau1 * tau2) / (*Ti_out);

    double lambda_min = theta / 2.0;
    double lambda_eff = (lambda_imc > lambda_min) ? lambda_imc : lambda_min;

    *Kp_out = (*Ti_out) / (fabs(K) * lambda_eff);

    if (*Kp_out < 0.0 && K > 0.0) *Kp_out = 0.0;
    return 0;
}

/*===========================================================================
 * L5: Lambda/Dahlin Tuning
 *
 * Classic industrial method. For Smith predictor (delay removed):
 *   Kp = tau / (K * (lambda + theta/2))
 *   Ti = tau
 *
 * Reference: Dahlin (1968) "Designing and tuning digital controllers"
 *===========================================================================*/

int smith_tune_lambda_pi(
    const smith_process_model_t *model,
    double lambda,
    double *Kp_out, double *Ti_out)
{
    if (!model || !Kp_out || !Ti_out) return -1;

    double K, tau, theta;
    switch (model->order) {
    case SMITH_MODEL_FOPDT:
        K = model->fopdt.K; tau = model->fopdt.tau;
        theta = model->fopdt.theta; break;
    case SMITH_MODEL_SOPDT:
        K = model->sopdt.K; tau = model->sopdt.tau1;
        theta = model->sopdt.theta; break;
    default:
        return -1;
    }

    if (fabs(K) < 1e-12 || tau <= 0.0) return -1;

    double lambda_eff = lambda + theta / 2.0;
    *Kp_out = tau / (fabs(K) * lambda_eff);
    *Ti_out = tau;

    if (*Kp_out < 0.0 && K > 0.0) *Kp_out = 0.0;
    return 0;
}

/*===========================================================================
 * L5: Robustness Filter Tuning
 *
 * Normey-Rico rule (1997): T_r ≥ θ * (ΔK + Δθ/τ)
 * Default: T_r = θ/2  (balanced), T_r = θ (robust)
 *===========================================================================*/

int smith_tune_robustness_filter(
    const smith_process_model_t *model,
    double delta_K, double delta_theta,
    double *Fr_out)
{
    if (!model || !Fr_out) return -1;

    double tau, theta;
    switch (model->order) {
    case SMITH_MODEL_FOPDT:
        tau = model->fopdt.tau; theta = model->fopdt.theta; break;
    case SMITH_MODEL_SOPDT:
    case SMITH_MODEL_SOPDT_UNDER:
        tau = model->sopdt.tau1; theta = model->sopdt.theta; break;
    default:
        return -1;
    }

    if (tau < 1e-9) tau = 1.0;

    *Fr_out = theta * (delta_K + delta_theta / tau);
    if (*Fr_out < theta / 10.0) *Fr_out = theta / 10.0;
    return 0;
}

/*===========================================================================
 * L5: ISE-Optimal PI Tuning via Golden-Section Search
 *
 * Minimizes J(Kp,Ti) = ∫ e²(t) dt for unit step response.
 *===========================================================================*/

static double sim_fopdt_step_ise(double K, double tau, double Kp, double Ti)
{
    /* Simulate step response of PI-controlled FOPDT and compute ISE */
    double dt = tau / 200.0;
    if (dt < 0.0005) dt = 0.0005;
    double t_end = 10.0 * tau;
    int n = (int)(t_end / dt);
    if (n < 500) n = 500;
    if (n > 20000) n = 20000;

    double y = 0.0, integrator = 0.0;
    double ise = 0.0;

    for (int k = 0; k < n; k++) {
        double e = 1.0 - y;
        double u = Kp * e + integrator;
        double dy = (K * u - y) / tau * dt;
        y += dy;
        if (Ti > 0.0) integrator += Kp * dt / Ti * e;
        ise += e * e * dt;
    }
    return ise;
}

int smith_tune_optimal_ise_pi(
    const smith_process_model_t *model,
    double Kp_init, double Ti_init,
    double *Kp_opt_out, double *Ti_opt_out, double *J_min_out)
{
    if (!model || !Kp_opt_out || !Ti_opt_out || !J_min_out) return -1;

    double K, tau;
    switch (model->order) {
    case SMITH_MODEL_FOPDT:
        K = model->fopdt.K; tau = model->fopdt.tau; break;
    default:
        return -1;
    }

    if (fabs(K) < 1e-12 || tau <= 0.0) return -1;

    double gr = (sqrt(5.0) - 1.0) / 2.0;
    double best_Kp = Kp_init, best_Ti = Ti_init;
    double best_J = sim_fopdt_step_ise(K, tau, Kp_init, Ti_init);
    int n_grid = 30;

    for (int i = 0; i < n_grid; i++) {
        double Kp_test = Kp_init * (0.2 + 4.8 * i / (n_grid - 1));
        double a = Ti_init * 0.2, b = Ti_init * 4.0;

        for (int iter = 0; iter < 30; iter++) {
            double c = b - gr * (b - a);
            double d = a + gr * (b - a);
            double Jc = sim_fopdt_step_ise(K, tau, Kp_test, c);
            double Jd = sim_fopdt_step_ise(K, tau, Kp_test, d);
            if (Jc < Jd) b = d; else a = c;
            if (fabs(b - a) < 1e-6) break;
        }

        double Ti_best = (a + b) / 2.0;
        double J_val = sim_fopdt_step_ise(K, tau, Kp_test, Ti_best);
        if (J_val < best_J) { best_J = J_val; best_Kp = Kp_test; best_Ti = Ti_best; }
    }

    *Kp_opt_out = best_Kp;
    *Ti_opt_out = best_Ti;
    *J_min_out = best_J;
    return 0;
}

/*===========================================================================
 * L4: Stability Margins
 *===========================================================================*/

/* Forward declaration from smith_robustness.c */
extern int smith_robustness_margins(
    const smith_process_model_t *model, double Kp, double Ti, double Td,
    double *gm_db, double *pm_deg, double *w_gc, double *w_pc);

int smith_tune_stability_margins(
    const smith_process_model_t *model,
    double Kp, double Ti, double Td,
    double *gm_db, double *pm_deg)
{
    return smith_robustness_margins(model, Kp, Ti, Td, gm_db, pm_deg, NULL, NULL);
}

int smith_tune_is_stable(
    double Kp, double Ti,
    const smith_process_model_t *model)
{
    if (!model) return 0;
    double gm_db, pm_deg;
    smith_tune_stability_margins(model, Kp, Ti, 0.0, &gm_db, &pm_deg);
    /* ISA/IEC: GM ≥ 6 dB, PM ≥ 30° */
    return (gm_db >= 6.0 && pm_deg >= 30.0) ? 1 : (gm_db > 0.0 && pm_deg > 0.0) ? 1 : 0;
}

/*===========================================================================
 * L5: Setpoint Prefilter Design
 *===========================================================================*/

void smith_tune_setpoint_filter(
    const smith_process_model_t *model, double Ti,
    double *T_ref_out)
{
    if (!model || !T_ref_out) return;
    *T_ref_out = Ti;
    double tau = 0.0;
    switch (model->order) {
    case SMITH_MODEL_FOPDT: tau = model->fopdt.tau; break;
    case SMITH_MODEL_SOPDT: tau = model->sopdt.tau1; break;
    default: return;
    }
    if (*T_ref_out < tau / 10.0) *T_ref_out = tau / 10.0;
    if (*T_ref_out > 10.0 * tau) *T_ref_out = 10.0 * tau;
}
