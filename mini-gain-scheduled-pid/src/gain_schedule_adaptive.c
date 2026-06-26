#include "gain_schedule_adaptive.h"
#include "gain_schedule_pid.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* Local clamp for fuzzy inference */
static double clamp_adaptive_local(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void gs_adaptive_rls_init(gs_rls_estimator_t *est, double lambda) {
    if (!est) return;
    memset(est, 0, sizeof(gs_rls_estimator_t));
    est->lambda = lambda;
    if (est->lambda <= 0.9) est->lambda = 0.98;
    if (est->lambda > 1.0) est->lambda = 0.999;
    
    est->P[0][0] = 1000.0;
    est->P[1][1] = 1000.0;
    est->P[2][2] = 1000.0;
    est->theta[0] = 1.0;
    est->theta[1] = 0.5;
    est->theta[2] = 0.0;
    est->converged = false;
}

void gs_adaptive_rls_update(gs_rls_estimator_t *est,
                             double u, double y) {
    if (!est) return;
    
    double lambda = est->lambda;
    
    /* Build regression vector phi = [u(k-1), y(k-1), 1] */
    double u_prev = est->phi[0];
    double y_prev = est->phi[1];
    
    est->phi[0] = u;
    est->phi[1] = y;
    est->phi[2] = 1.0;
    
    /* Predicted output */
    est->y_hat = est->theta[0] * u_prev + est->theta[1] * y_prev
               + est->theta[2];
    est->residual = y - est->y_hat;
    
    /* RLS update: K = P*phi / (lambda + phi'*P*phi) */
    double phiP[3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            phiP[i] += est->phi[j] * est->P[j][i];
        }
    }
    
    double denom = lambda;
    for (int i = 0; i < 3; i++) {
        denom += phiP[i] * est->phi[i];
    }
    
    if (fabs(denom) < 1e-12) {
        est->n_updates++;
        return;
    }
    
    double K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = phiP[i] / denom;
    }
    
    /* Update theta */
    for (int i = 0; i < 3; i++) {
        est->theta[i] += K[i] * est->residual;
    }
    
    /* Update P: P = (I - K*phi')*P / lambda */
    double I_minus_Kphi[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            I_minus_Kphi[i][j] = -K[i] * est->phi[j];
            if (i == j) I_minus_Kphi[i][j] += 1.0;
        }
    }
    
    double P_new[3][3] = {{0}};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                P_new[i][j] += I_minus_Kphi[i][k] * est->P[k][j];
            }
            P_new[i][j] /= lambda;
        }
    }
    
    memcpy(est->P, P_new, sizeof(P_new));
    
    est->n_updates++;
    
    /* Check convergence: small residual over many updates */
    double trace_P = est->P[0][0] + est->P[1][1] + est->P[2][2];
    if (trace_P < 0.01 && est->n_updates > 100) {
        est->converged = true;
    }
}

bool gs_adaptive_rls_get_params(const gs_rls_estimator_t *est,
                                 double *K, double *tau) {
    if (!est || !K || !tau) return false;
    if (!est->converged && est->n_updates < 50) return false;
    
    *K = est->theta[0];
    double a = est->theta[1];
    if (fabs(*K) < 1e-9) *K = 1.0;
    if (a >= 1.0 || a <= -1.0) a = 0.5;
    *tau = 1.0 / (1.0 - a);
    if (*tau <= 0.0) *tau = 1.0;
    
    return true;
}

void gs_adaptive_perf_init(gs_adaptive_performance_t *perf,
                            double threshold, double rate) {
    if (!perf) return;
    memset(perf, 0, sizeof(gs_adaptive_performance_t));
    perf->performance_threshold = threshold;
    perf->adaptation_rate = rate;
    if (perf->adaptation_rate <= 0.0) perf->adaptation_rate = 0.1;
    if (perf->adaptation_rate > 1.0) perf->adaptation_rate = 1.0;
    perf->baseline_performance = 1.0;
}

double gs_adaptive_evaluate_iae(const double *error_history, uint32_t n) {
    if (!error_history || n == 0) return 0.0;
    double iae = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        iae += fabs(error_history[i]);
    }
    return iae;
}

double gs_adaptive_evaluate_ise(const double *error_history, uint32_t n) {
    if (!error_history || n == 0) return 0.0;
    double ise = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        ise += error_history[i] * error_history[i];
    }
    return ise;
}

double gs_adaptive_evaluate_itae(const double *error_history,
                                  const double *time,
                                  uint32_t n) {
    if (!error_history || !time || n == 0) return 0.0;
    double itae = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        itae += time[i] * fabs(error_history[i]);
    }
    return itae;
}

bool gs_adaptive_gradient_update(
    gs_pid_state_t *state,
    gain_schedule_table_t *table,
    const gs_adaptive_performance_t *perf,
    uint32_t sched_index) {
    if (!state || !table || !perf) return false;
    if (sched_index >= table->num_entries) return false;
    
    double rate = perf->adaptation_rate;
    double current_perf = perf->current_performance;
    double baseline = perf->baseline_performance;
    
    if (baseline <= 0.0) return false;
    
    double perf_ratio = current_perf / baseline;
    
    /* If performance degraded, adjust gains */
    if (perf_ratio > 1.0 + perf->performance_threshold) {
        pid_gain_set_t *g = &table->entries[sched_index].gains;
        
        g->Kp *= (1.0 + rate * (perf_ratio - 1.0));
        g->Ki *= (1.0 + rate * 0.5 * (perf_ratio - 1.0));
        g->Kd *= (1.0 - rate * 0.1 * (perf_ratio - 1.0));
        
        if (g->Kp < 0.01) g->Kp = 0.01;
        if (g->Ki < 0.0) g->Ki = 0.0;
        if (g->Kd < 0.0) g->Kd = 0.0;
        
        return true;
    }
    return false;
}

void gs_adaptive_fuzzy_init(gs_fuzzy_schedule_t *fs) {
    if (!fs) return;
    memset(fs, 0, sizeof(gs_fuzzy_schedule_t));
    
    /* Initialize membership function centers for error: NL=-1, NS=-0.5, ZE=0, PS=0.5, PL=1 */
    double centers[5] = {-1.0, -0.5, 0.0, 0.5, 1.0};
    double spreads[5]  = {0.4, 0.4, 0.4, 0.4, 0.4};
    
    for (int i = 0; i < 5; i++) {
        fs->error_mf[i].center = centers[i];
        fs->error_mf[i].spread = spreads[i];
        fs->rate_mf[i].center = centers[i];
        fs->rate_mf[i].spread = spreads[i];
    }
    
    /* Default rule base: symmetric */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            double abs_err = fabs(centers[i]);
            double abs_rate = fabs(centers[j]);
            
            fs->rule_base[i][j][0] = 0.1 * (abs_err + abs_rate);
            fs->rule_base[i][j][1] = 0.05 * abs_err;
            fs->rule_base[i][j][2] = 0.05 * abs_rate;
        }
    }
}

static double fuzzy_membership(double x, double center, double spread) {
    /* Triangular membership */
    double d = fabs(x - center);
    if (d >= spread) return 0.0;
    return 1.0 - d / spread;
}

void gs_adaptive_fuzzy_infer(const gs_fuzzy_schedule_t *fs,
                              double error_norm,
                              double error_rate_norm,
                              double *Kp_adj,
                              double *Ki_adj,
                              double *Kd_adj) {
    if (!fs || !Kp_adj || !Ki_adj || !Kd_adj) return;
    
    double e = clamp_adaptive_local(error_norm, -1.0, 1.0);
    double de = clamp_adaptive_local(error_rate_norm, -1.0, 1.0);
    
    double num_Kp = 0.0, num_Ki = 0.0, num_Kd = 0.0;
    double denom = 0.0;
    
    for (int i = 0; i < 5; i++) {
        double mu_e = fuzzy_membership(e, fs->error_mf[i].center,
                                        fs->error_mf[i].spread);
        if (mu_e < 1e-6) continue;
        
        for (int j = 0; j < 5; j++) {
            double mu_de = fuzzy_membership(de, fs->rate_mf[j].center,
                                             fs->rate_mf[j].spread);
            if (mu_de < 1e-6) continue;
            
            double weight = mu_e * mu_de;
            num_Kp += weight * fs->rule_base[i][j][0];
            num_Ki += weight * fs->rule_base[i][j][1];
            num_Kd += weight * fs->rule_base[i][j][2];
            denom += weight;
        }
    }
    
    if (denom > 1e-9) {
        *Kp_adj = 1.0 + num_Kp / denom;
        *Ki_adj = 1.0 + num_Ki / denom;
        *Kd_adj = 1.0 + num_Kd / denom;
    } else {
        *Kp_adj = 1.0;
        *Ki_adj = 1.0;
        *Kd_adj = 1.0;
    }
    
    if (*Kp_adj < 0.1) *Kp_adj = 0.1;
    if (*Kp_adj > 5.0) *Kp_adj = 5.0;
    if (*Ki_adj < 0.0) *Ki_adj = 0.0;
    if (*Ki_adj > 10.0) *Ki_adj = 10.0;
    if (*Kd_adj < 0.0) *Kd_adj = 0.0;
    if (*Kd_adj > 5.0) *Kd_adj = 5.0;
}

double gs_adaptive_blend_outputs(const double *weights,
                                  const double *outputs,
                                  uint32_t n_models) {
    if (!weights || !outputs || n_models == 0) return 0.0;
    
    double sum_w = 0.0;
    double blended = 0.0;
    for (uint32_t i = 0; i < n_models; i++) {
        blended += weights[i] * outputs[i];
        sum_w += weights[i];
    }
    
    if (sum_w < 1e-12) {
        for (uint32_t i = 0; i < n_models; i++) {
            blended += outputs[i];
        }
        return blended / (double)n_models;
    }
    return blended / sum_w;
}

void gs_adaptive_gaussian_weights(double sched_val,
                                   const double *centers,
                                   const double *sigmas,
                                   uint32_t n,
                                   double *weights) {
    if (!centers || !sigmas || !weights || n == 0) return;
    
    double sum = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        double sigma = sigmas[i];
        if (sigma <= 0.0) sigma = 1.0;
        double r2 = (sched_val - centers[i]) * (sched_val - centers[i]);
        weights[i] = exp(-r2 / (2.0 * sigma * sigma));
        sum += weights[i];
    }
    
    if (sum < 1e-12) {
        for (uint32_t i = 0; i < n; i++) {
            weights[i] = 1.0 / (double)n;
        }
    } else {
        for (uint32_t i = 0; i < n; i++) {
            weights[i] /= sum;
        }
    }
}
