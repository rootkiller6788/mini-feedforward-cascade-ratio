/**
 * @file cascade_stability.c
 * @brief Cascade Stability Analysis
 * Module: mini-cascade-control-primary-secondary
 */

#include "cascade_stability.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void cascade_fopdt_frequency_response(const cascade_fopdt_model_t *model,
                                       double w, double *mag, double *phase_rad)
{
    if (model == NULL || mag == NULL || phase_rad == NULL) return;
    double K = model->K, tau = model->tau, theta = model->theta;
    if (w < 1e-12) { *mag = fabs(K); *phase_rad = 0.0; return; }
    *mag = K / sqrt(1.0 + w * w * tau * tau);
    *phase_rad = -w * theta - atan(w * tau);
    while (*phase_rad > M_PI)  *phase_rad -= 2.0 * M_PI;
    while (*phase_rad < -M_PI) *phase_rad += 2.0 * M_PI;
}

void cascade_pid_frequency_response(const cascade_pid_params_t *pid,
                                     double w, double ts,
                                     double *mag, double *phase_rad)
{
    (void)ts;
    if (pid == NULL || mag == NULL || phase_rad == NULL) return;
    double Kp = pid->kp, Ti = pid->ti, Td = pid->td, Tf = pid->tf;
    if (Tf < 1e-9 && Td > 1e-9) Tf = Td / 10.0;
    if (w < 1e-12) {
        *mag = (Ti > 1e-9) ? 1e9 : Kp;
        *phase_rad = (Ti > 1e-9) ? -M_PI/2.0 : 0.0;
        return;
    }
    double re = Kp, im = 0.0;
    if (Ti > 1e-9) im += -Kp / (w * Ti);
    if (Td > 1e-9 && Tf > 1e-9) {
        double d = 1.0 + w*w*Tf*Tf;
        re += Kp*w*w*Td*Tf / d;
        im += Kp*w*Td / d;
    }
    *mag = sqrt(re*re + im*im);
    *phase_rad = atan2(im, re);
}

void cascade_open_loop_response(const cascade_pid_params_t *pri_pid,
                                 const cascade_pid_params_t *sec_pid,
                                 const cascade_fopdt_model_t *pri_proc,
                                 const cascade_fopdt_model_t *sec_proc,
                                 double w, double ts,
                                 double *mag, double *phase_rad)
{
    if (!pri_pid||!sec_pid||!pri_proc||!sec_proc||!mag||!phase_rad) return;
    (void)ts;
    double cm, cp, pm_val, pp;
    cascade_pid_frequency_response(sec_pid, w, ts, &cm, &cp);
    cascade_fopdt_frequency_response(sec_proc, w, &pm_val, &pp);
    double om = cm * pm_val, op = cp + pp;
    double ore = om * cos(op), oim = om * sin(op);
    double dr = 1.0 + ore, di = oim, d2 = dr*dr + di*di;
    double hr = 0.0, hi = 0.0;
    if (d2 > 1e-15) { hr = (ore*dr + oim*di)/d2; hi = (oim*dr - ore*di)/d2; }
    double pgm, pgp, pppm, pppp;
    cascade_pid_frequency_response(pri_pid, w, ts, &pgm, &pgp);
    cascade_fopdt_frequency_response(pri_proc, w, &pppm, &pppp);
    double pr = pgm * cos(pgp), pi = pgm * sin(pgp);
    double ppr2 = pppm * cos(pppp), ppi2 = pppm * sin(pppp);
    double tr = pr*ppr2 - pi*ppi2, ti2 = pr*ppi2 + pi*ppr2;
    double Lr = tr*hr - ti2*hi, Li = tr*hi + ti2*hr;
    *mag = sqrt(Lr*Lr + Li*Li);
    *phase_rad = atan2(Li, Lr);
}

void cascade_sensitivity_function(const cascade_pid_params_t *pri_pid,
                                   const cascade_pid_params_t *sec_pid,
                                   const cascade_fopdt_model_t *pri_proc,
                                   const cascade_fopdt_model_t *sec_proc,
                                   double w, double ts,
                                   double *mag, double *phase_rad)
{
    if (!pri_pid||!sec_pid||!pri_proc||!sec_proc||!mag||!phase_rad) return;
    (void)ts;
    double Lm, Lp;
    cascade_open_loop_response(pri_pid, sec_pid, pri_proc, sec_proc, w, ts, &Lm, &Lp);
    double Lr = Lm * cos(Lp), Li = Lm * sin(Lp);
    double dr = 1.0 + Lr, di = Li, d2 = dr*dr + di*di;
    if (d2 < 1e-15) { *mag = 1e9; *phase_rad = 0.0; return; }
    *mag = 1.0 / sqrt(d2);
    *phase_rad = -atan2(di, dr);
}

int cascade_compute_stability(const cascade_pid_params_t *pri_pid,
                               const cascade_pid_params_t *sec_pid,
                               const cascade_fopdt_model_t *pri_proc,
                               const cascade_fopdt_model_t *sec_proc,
                               double ts, int n_freq,
                               cascade_stability_t *result)
{
    if (!pri_pid||!sec_pid||!pri_proc||!sec_proc||!result) return -1;
    if (n_freq < 20) n_freq = 200;
    memset(result, 0, sizeof(cascade_stability_t));
    double Ttot = pri_proc->tau + sec_proc->tau + pri_proc->theta + sec_proc->theta;
    if (Ttot < 1e-6) Ttot = 1.0;
    double ws = 0.0001/Ttot, we = 100.0/Ttot, wr = pow(we/ws, 1.0/(n_freq-1));
    double gm_db = 1e9, pm_deg = 1e9, ms_pk = 0.0, mt_pk = 0.0, wgc = 0.0;
    double pLm = 0.0, pLp = 0.0, pw = ws;
    cascade_open_loop_response(pri_pid, sec_pid, pri_proc, sec_proc, ws, ts, &pLm, &pLp);
    while (pLp > M_PI) pLp -= 2.0*M_PI;
    while (pLp < -M_PI) pLp += 2.0*M_PI;
    double sm0, sp0;
    cascade_sensitivity_function(pri_pid, sec_pid, pri_proc, sec_proc, ws, ts, &sm0, &sp0);
    ms_pk = sm0;
    { double Lr0 = pLm*cos(pLp), Li0 = pLm*sin(pLp), dr0 = 1.0+Lr0, di0 = Li0;
      double d20 = dr0*dr0+di0*di0;
      if (d20 > 1e-15) {
        double Tr = (Lr0*dr0+Li0*di0)/d20, Ti0 = (Li0*dr0-Lr0*di0)/d20;
        mt_pk = sqrt(Tr*Tr+Ti0*Ti0);
      }
    }
    double w = ws*wr;
    for (int i = 1; i < n_freq; i++) {
        double Lm, Lp;
        cascade_open_loop_response(pri_pid, sec_pid, pri_proc, sec_proc, w, ts, &Lm, &Lp);
        while (Lp > M_PI) Lp -= 2.0*M_PI;
        while (Lp < -M_PI) Lp += 2.0*M_PI;
        if ((pLm - 1.0)*(Lm - 1.0) < 0.0) {
            double a = (1.0 - pLm)/(Lm - pLm);
            if (a > 0.0 && a < 1.0) {
                double wc = pw + a*(w-pw), ph = pLp + a*(Lp-pLp);
                double pm = fabs(180.0 + ph*180.0/M_PI);
                if (pm < pm_deg) { pm_deg = pm; wgc = wc; }
            }
        }
        if ((pLp + M_PI)*(Lp + M_PI) < 0.0) {
            double a = fabs(pLp+M_PI)/(fabs(pLp+M_PI) + fabs(Lp+M_PI));
            if (a > 0.0 && a < 1.0) {
                double ml = pLm + a*(Lm - pLm);
                if (ml > 1e-12) { double gm = -20.0*log10(ml); if (gm < gm_db) gm_db = gm; }
            }
        }
        { double sm, sp;
          cascade_sensitivity_function(pri_pid, sec_pid, pri_proc, sec_proc, w, ts, &sm, &sp);
          if (sm > ms_pk) ms_pk = sm; }
        { double Lr = Lm*cos(Lp), Li = Lm*sin(Lp), dr = 1.0+Lr, di = Li;
          double d2 = dr*dr+di*di;
          if (d2 > 1e-15) {
            double Tr = (Lr*dr+Li*di)/d2, Ti0 = (Li*dr-Lr*di)/d2;
            double Tm = sqrt(Tr*Tr+Ti0*Ti0);
            if (Tm > mt_pk) mt_pk = Tm;
          }
        }
        pw = w; pLm = Lm; pLp = Lp; w *= wr;
    }
    result->gain_margin_db = gm_db;
    result->phase_margin_deg = pm_deg;
    result->crossover_freq_rad_s = wgc;
    result->sensitivity_peak = ms_pk;
    result->complementary_sensitivity_peak = mt_pk;
    result->delay_margin_sec = (wgc > 1e-12) ? (pm_deg*M_PI/180.0)/wgc : 1e6;
    result->modulus_margin = (ms_pk > 1e-12) ? (1.0/ms_pk) : 1.0;
    result->robustness_index = cascade_robustness_index(result);
    if (gm_db >= 6.0 && pm_deg >= 30.0 && ms_pk <= 2.0) {
        result->is_stable = true;
        snprintf(result->stability_verdict, 64, "%s", "Stable with good robustness");
    } else {
        result->is_stable = false;
        snprintf(result->stability_verdict, 64, "%s", "Margins insufficient");
    }
    return result->is_stable ? 0 : -1;
}

int cascade_check_nyquist_stability(const cascade_pid_params_t *pri_pid,
                                     const cascade_pid_params_t *sec_pid,
                                     const cascade_fopdt_model_t *pri_proc,
                                     const cascade_fopdt_model_t *sec_proc,
                                     double ts, int n_freq)
{
    cascade_stability_t s;
    memset(&s, 0, sizeof(s));
    cascade_compute_stability(pri_pid, sec_pid, pri_proc, sec_proc, ts, n_freq, &s);
    return s.is_stable ? 0 : -1;
}

int cascade_check_stability_simple(const cascade_pid_params_t *pri_pid,
                                    const cascade_pid_params_t *sec_pid,
                                    const cascade_fopdt_model_t *pri_proc,
                                    const cascade_fopdt_model_t *sec_proc,
                                    double ts, int n_freq,
                                    cascade_stability_t *stability)
{
    cascade_stability_t s;
    memset(&s, 0, sizeof(s));
    cascade_compute_stability(pri_pid, sec_pid, pri_proc, sec_proc, ts, n_freq, &s);
    if (stability) *stability = s;
    return s.is_stable ? 0 : -1;
}

double cascade_robustness_index(const cascade_stability_t *stability)
{
    if (!stability) return 0.0;
    return stability->modulus_margin;
}

double cascade_compute_modulus_margin(const cascade_pid_params_t *pri_pid,
                                       const cascade_pid_params_t *sec_pid,
                                       const cascade_fopdt_model_t *pri_proc,
                                       const cascade_fopdt_model_t *sec_proc,
                                       double ts, int n_freq)
{
    cascade_stability_t s;
    memset(&s, 0, sizeof(s));
    cascade_compute_stability(pri_pid, sec_pid, pri_proc, sec_proc, ts, n_freq, &s);
    return s.modulus_margin;
}

int cascade_stability_delay_impact(double delay_sec,
                                    const cascade_pid_params_t *pri_pid,
                                    const cascade_pid_params_t *sec_pid,
                                    const cascade_fopdt_model_t *pri_proc,
                                    const cascade_fopdt_model_t *sec_proc,
                                    double ts, double *gm_db)
{
    if (!pri_pid||!sec_pid||!pri_proc||!sec_proc||!gm_db) return -1;
    cascade_fopdt_model_t mp = *pri_proc;
    mp.theta += delay_sec;
    cascade_stability_t s;
    memset(&s, 0, sizeof(s));
    cascade_compute_stability(pri_pid, sec_pid, &mp, sec_proc, ts, 100, &s);
    *gm_db = s.gain_margin_db;
    return (s.is_stable && s.gain_margin_db > 2.0) ? 0 : -1;
}

int cascade_secondary_stability(const cascade_pid_params_t *sec_pid,
                                 const cascade_fopdt_model_t *sec_proc,
                                 double ts, int n_freq,
                                 cascade_stability_t *result)
{
    if (!sec_pid||!sec_proc) return -1;
    cascade_pid_params_t zp;
    memset(&zp, 0, sizeof(zp));
    zp.kp = 1.0; zp.ti = 1e9; zp.td = 0.0;
    cascade_fopdt_model_t pt;
    memset(&pt, 0, sizeof(pt));
    pt.K = 1.0; pt.tau = 1e-6; pt.theta = 0.0;
    cascade_stability_t local;
    cascade_stability_t *s = result ? result : &local;
    memset(s, 0, sizeof(*s));
    return cascade_compute_stability(&zp, sec_pid, &pt, sec_proc, ts, n_freq, s);
}

double cascade_max_allowed_gain(const cascade_pid_params_t *base_pid,
                                 const cascade_fopdt_model_t *process,
                                 double ts, double kp_min, double kp_max,
                                 int n_freq)
{
    if (!base_pid || !process) return kp_min;
    double lo = kp_min, hi = kp_max;
    for (int i = 0; i < 20; i++) {
        double mid = 0.5 * (lo + hi);
        cascade_pid_params_t tp = *base_pid;
        tp.kp = mid;
        cascade_stability_t s;
        memset(&s, 0, sizeof(s));
        cascade_secondary_stability(&tp, process, ts, n_freq, &s);
        if (s.is_stable && s.phase_margin_deg > 30.0) lo = mid;
        else hi = mid;
        if (fabs(hi - lo) < 1e-6) break;
    }
    return lo;
}

int cascade_min_phase_margin_primary(const double *gains,
                                      const double *ti_vals,
                                      int n_points,
                                      const cascade_fopdt_model_t *model,
                                      double ts, double *worst_pm)
{
    if (!gains || !ti_vals || !model || !worst_pm || n_points < 1) return -1;
    *worst_pm = 180.0;
    cascade_pid_params_t tp;
    memset(&tp, 0, sizeof(tp));
    tp.td = 0.0; tp.tf = 0.1;
    for (int i = 0; i < n_points; i++) {
        tp.kp = gains[i]; tp.ti = ti_vals[i];
        cascade_stability_t s;
        memset(&s, 0, sizeof(s));
        cascade_secondary_stability(&tp, model, ts, 100, &s);
        if (!s.is_stable) continue;
        if (s.phase_margin_deg < *worst_pm) *worst_pm = s.phase_margin_deg;
    }
    return (*worst_pm < 180.0) ? 0 : -1;
}
