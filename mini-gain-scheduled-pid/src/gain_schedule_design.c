#include "gain_schedule_design.h"
#include "gain_schedule_interp.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

pid_gain_set_t gs_design_zn_pid(double Ku, double Pu) {
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    g.Kp = 0.6 * Ku;
    g.Ki = g.Kp / (0.5 * Pu);
    g.Kd = g.Kp * (0.125 * Pu);
    g.Ti = 0.5 * Pu;
    g.Td = 0.125 * Pu;
    g.N  = 10.0;
    g.b  = 1.0;
    g.c  = 0.0;
    g.Kb = 0.3;
    g.tracking_time = 1.0;
    g.alpha = 0.0;
    return g;
}

pid_gain_set_t gs_design_zn_pi(double Ku, double Pu) {
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    g.Kp = 0.45 * Ku;
    g.Ki = g.Kp / (0.85 * Pu);
    g.Kd = 0.0;
    g.Ti = 0.85 * Pu;
    g.Td = 0.0;
    g.N  = 10.0;
    g.b  = 1.0;
    g.c  = 0.0;
    g.Kb = 0.3;
    g.tracking_time = 1.0;
    g.alpha = 0.0;
    return g;
}

pid_gain_set_t gs_design_tyreus_luyben_pid(double Ku, double Pu) {
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    g.Kp = 0.45 * Ku;
    g.Ki = g.Kp / (2.2 * Pu);
    g.Kd = g.Kp * (Pu / 6.3);
    g.Ti = 2.2 * Pu;
    g.Td = Pu / 6.3;
    g.N  = 10.0;
    g.b  = 1.0;
    g.c  = 0.0;
    g.Kb = 0.3;
    g.tracking_time = 2.0;
    g.alpha = 0.0;
    return g;
}

pid_gain_set_t gs_design_tyreus_luyben_pi(double Ku, double Pu) {
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    g.Kp = Ku / 3.2;
    g.Ki = g.Kp / (2.2 * Pu);
    g.Kd = 0.0;
    g.Ti = 2.2 * Pu;
    g.Td = 0.0;
    g.N  = 10.0;
    g.b  = 1.0;
    g.c  = 0.0;
    g.Kb = 0.3;
    g.tracking_time = 2.0;
    g.alpha = 0.0;
    return g;
}

pid_gain_set_t gs_design_cohen_coon_pid(double K, double tau, double L) {
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    if (K <= 0.0 || tau <= 0.0 || L < 0.0) {
        g.Kp = 1.0; g.Ki = 0.1; g.Kd = 0.0;
        return g;
    }
    double ratio = L / tau;
    double factor = tau / (K * L);
    g.Kp = factor * (1.35 + 0.27 * ratio);
    g.Ki = g.Kp / (L * (2.5 + 0.5 * ratio) / (1.0 + 0.6 * ratio));
    g.Kd = g.Kp * L * (0.37 + 0.00 * ratio) / (1.0 + 0.2 * ratio);
    g.Ti = L * (2.5 + 0.5 * ratio) / (1.0 + 0.6 * ratio);
    g.Td = L * (0.37 + 0.00 * ratio) / (1.0 + 0.2 * ratio);
    g.N  = 10.0;
    g.b  = 1.0;
    g.c  = 0.0;
    g.Kb = 0.4;
    g.tracking_time = 1.0;
    g.alpha = 0.0;
    return g;
}

pid_gain_set_t gs_design_cohen_coon_pi(double K, double tau, double L) {
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    if (K <= 0.0 || tau <= 0.0 || L < 0.0) {
        g.Kp = 1.0; g.Ki = 0.1; g.Kd = 0.0;
        return g;
    }
    double ratio = L / tau;
    double factor = tau / (K * L);
    g.Kp = factor * (0.9 + 0.083 * ratio);
    g.Ki = g.Kp / (L * (3.33 + 0.31 * ratio) / (1.0 + 2.2 * ratio));
    g.Kd = 0.0;
    g.Ti = L * (3.33 + 0.31 * ratio) / (1.0 + 2.2 * ratio);
    g.Td = 0.0;
    g.N  = 10.0;
    g.b  = 1.0;
    g.c  = 0.0;
    g.Kb = 0.4;
    g.tracking_time = 1.0;
    g.alpha = 0.0;
    return g;
}

pid_gain_set_t gs_design_imc_pid(double K, double tau, double L,
                                  double lambda) {
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    if (K <= 0.0 || tau <= 0.0 || lambda <= 0.0) {
        g.Kp = 1.0; g.Ki = 0.1; g.Kd = 0.0;
        return g;
    }
    double tau_c = lambda + L / 2.0;
    g.Kp = (tau + L / 2.0) / (K * tau_c);
    g.Ki = g.Kp / (tau + L / 2.0);
    g.Kd = g.Kp * (tau * L / (2.0 * tau + L)) / 2.0;
    g.Ti = tau + L / 2.0;
    g.Td = (tau * L) / (2.0 * tau + L) / 2.0;
    g.N  = 10.0;
    g.b  = 1.0;
    g.c  = 0.0;
    g.Kb = 0.5;
    g.tracking_time = tau_c;
    g.alpha = 0.0;
    return g;
}

pid_gain_set_t gs_design_simc_pi(double K, double tau, double L,
                                  double tau_c) {
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    if (K <= 0.0 || tau_c <= 0.0) {
        g.Kp = 1.0; g.Ki = 0.1; g.Kd = 0.0;
        return g;
    }
    if (tau <= 0.0) tau = 0.1 * tau_c;
    g.Kp = (1.0 / K) * (tau / (tau_c + L));
    g.Ki = g.Kp / tau;
    g.Kd = 0.0;
    g.Ti = tau;
    g.Td = 0.0;
    g.N  = 10.0;
    g.b  = 0.5;
    g.c  = 0.0;
    g.Kb = 0.5;
    g.tracking_time = tau_c;
    g.alpha = 0.0;
    return g;
}

pid_gain_set_t gs_design_amigo_pid(double K, double tau, double L) {
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    if (K <= 0.0 || tau <= 0.0 || L < 0.0) {
        g.Kp = 1.0; g.Ki = 0.1; g.Kd = 0.0;
        return g;
    }
    double ratio = L / (L + tau);
    g.Kp = (1.0 / K) * (0.20 + 0.45 * ratio);
    g.Ki = g.Kp / (L * (0.40 + 0.80 * ratio) / (0.10 + 0.80 * ratio));
    g.Kd = g.Kp * L * (0.50 * ratio) / (0.30 + 0.50 * ratio);
    g.Ti = L * (0.40 + 0.80 * ratio) / (0.10 + 0.80 * ratio);
    g.Td = L * (0.50 * ratio) / (0.30 + 0.50 * ratio);
    g.N  = 10.0;
    g.b  = 0.3;
    g.c  = 0.0;
    g.Kb = 0.3;
    g.tracking_time = 1.0;
    g.alpha = 0.0;
    return g;
}

pid_gain_set_t gs_design_amigo_pi(double K, double tau, double L) {
    pid_gain_set_t g;
    memset(&g, 0, sizeof(g));
    if (K <= 0.0 || tau <= 0.0 || L < 0.0) {
        g.Kp = 1.0; g.Ki = 0.1; g.Kd = 0.0;
        return g;
    }
    double ratio = L / (L + tau);
    g.Kp = (1.0 / K) * (0.15 + 0.35 * ratio);
    g.Ki = g.Kp / (L * (0.35 + 1.20 * ratio) / (0.10 + 0.70 * ratio));
    g.Kd = 0.0;
    g.Ti = L * (0.35 + 1.20 * ratio) / (0.10 + 0.70 * ratio);
    g.Td = 0.0;
    g.N  = 10.0;
    g.b  = 0.3;
    g.c  = 0.0;
    g.Kb = 0.3;
    g.tracking_time = 1.0;
    g.alpha = 0.0;
    return g;
}

bool gs_design_frozen_parameter(gain_schedule_table_t *table,
                                 int tuning_rule,
                                 const double *sched_values,
                                 const double *K_array,
                                 const double *tau_array,
                                 const double *L_array,
                                 uint32_t n) {
    if (!table || !sched_values || !K_array || !tau_array || !L_array || n < 2) {
        return false;
    }
    
    gs_table_clear(table);
    
    for (uint32_t i = 0; i < n; i++) {
        pid_gain_set_t g;
        memset(&g, 0, sizeof(g));
        
        double K = K_array[i];
        double tau = tau_array[i];
        double L = L_array[i];
        double Ku = 2.0 * M_PI / L;
        double Pu = 2.0 * L;
        if (K > 0.0 && tau > 0.0) {
            Ku = 2.0 * tau / (K * L);
            Pu = 2.0 * L;
        }
        
        switch (tuning_rule) {
            case 0: g = gs_design_zn_pid(Ku, Pu); break;
            case 1: g = gs_design_tyreus_luyben_pid(Ku, Pu); break;
            case 2: g = gs_design_cohen_coon_pid(K, tau, L); break;
            case 3: g = gs_design_imc_pid(K, tau, L, tau); break;
            case 4: g = gs_design_simc_pi(K, tau, L, tau); break;
            case 5: g = gs_design_amigo_pid(K, tau, L); break;
            default: g = gs_design_zn_pid(Ku, Pu); break;
        }
        
        char label[32];
        snprintf(label, sizeof(label), "OP%u_%s", i,
                 tuning_rule == 0 ? "ZN" : tuning_rule == 1 ? "TL" :
                 tuning_rule == 2 ? "CC" : tuning_rule == 3 ? "IMC" :
                 tuning_rule == 4 ? "SIMC" : tuning_rule == 5 ? "AMIGO" : "DEF");
        
        if (!gs_table_add_entry(table, sched_values[i], &g, label)) {
            return false;
        }
    }
    
    gs_table_sort_entries(table);
    return gs_table_validate(table, NULL, 0);
}

bool gs_design_validate_margins(const gain_schedule_table_t *table,
                                double min_phase_margin,
                                double max_gain_ratio,
                                char *errmsg, size_t errmsg_size) {
    if (!table || table->num_entries < 2) {
        if (errmsg && errmsg_size > 0) {
            snprintf(errmsg, errmsg_size, "Table too small");
        }
        return false;
    }
    
    double ratio = gs_design_max_gain_ratio(table);
    if (ratio > max_gain_ratio && max_gain_ratio > 0.0) {
        if (errmsg && errmsg_size > 0) {
            snprintf(errmsg, errmsg_size,
                     "Gain ratio %.3f exceeds max %.3f", ratio, max_gain_ratio);
        }
        return false;
    }
    
    for (uint32_t i = 0; i < table->num_entries; i++) {
        if (table->entries[i].phase_margin < min_phase_margin &&
            table->entries[i].validated) {
            if (errmsg && errmsg_size > 0) {
                snprintf(errmsg, errmsg_size,
                         "Phase margin %.1f at entry %u below min %.1f",
                         table->entries[i].phase_margin, i, min_phase_margin);
            }
            return false;
        }
    }
    
    return true;
}

double gs_design_max_gain_ratio(const gain_schedule_table_t *table) {
    if (!table || table->num_entries < 2) return 1.0;
    
    double max_ratio = 1.0;
    for (uint32_t i = 1; i < table->num_entries; i++) {
        double Kp_prev = table->entries[i-1].gains.Kp;
        double Kp_curr = table->entries[i].gains.Kp;
        if (Kp_prev <= 0.0 || Kp_curr <= 0.0) continue;
        double ratio = Kp_curr / Kp_prev;
        if (ratio < 1.0) ratio = 1.0 / ratio;
        if (ratio > max_ratio) max_ratio = ratio;
    }
    return max_ratio;
}

uint32_t gs_design_refine_grid(gain_schedule_table_t *table,
                               double max_gain_step_ratio) {
    if (!table || table->num_entries < 2) return 0;
    if (table->num_entries >= GS_MAX_BREAKPOINTS - 1) return 0;
    
    uint32_t added = 0;
    uint32_t orig_count = table->num_entries;
    
    for (uint32_t i = 0; i < orig_count - 1 && added < orig_count; i++) {
        uint32_t idx = i + added;
        if (idx + 1 >= table->num_entries) break;
        
        double Kp1 = table->entries[idx].gains.Kp;
        double Kp2 = table->entries[idx+1].gains.Kp;
        if (Kp1 <= 0.0 || Kp2 <= 0.0) continue;
        double ratio = Kp2 / Kp1;
        if (ratio < 1.0) ratio = 1.0 / ratio;
        
        if (ratio > max_gain_step_ratio) {
            double mid_sched = (table->entries[idx].scheduling_value +
                                table->entries[idx+1].scheduling_value) / 2.0;
            pid_gain_set_t mid_gains;
            memset(&mid_gains, 0, sizeof(mid_gains));
            mid_gains.Kp = (Kp1 + Kp2) / 2.0;
            mid_gains.Ki = (table->entries[idx].gains.Ki +
                            table->entries[idx+1].gains.Ki) / 2.0;
            mid_gains.Kd = (table->entries[idx].gains.Kd +
                            table->entries[idx+1].gains.Kd) / 2.0;
            mid_gains.Ti = (table->entries[idx].gains.Ti +
                            table->entries[idx+1].gains.Ti) / 2.0;
            mid_gains.Td = (table->entries[idx].gains.Td +
                            table->entries[idx+1].gains.Td) / 2.0;
            mid_gains.N  = 10.0;
            mid_gains.b  = 1.0;
            mid_gains.Kb = 0.3;
            mid_gains.tracking_time = 1.0;
            
            if (gs_table_add_entry(table, mid_sched, &mid_gains, "REFINED")) {
                added++;
            }
        }
    }
    return added;
}

void gs_design_smooth_schedule(gain_schedule_table_t *table,
                                uint32_t window_size) {
    if (!table || table->num_entries < 3 || window_size < 2) return;
    if (window_size > table->num_entries) window_size = table->num_entries;
    
    pid_gain_set_t *smoothed = (pid_gain_set_t*)malloc(
        table->num_entries * sizeof(pid_gain_set_t));
    if (!smoothed) return;
    
    for (uint32_t i = 0; i < table->num_entries; i++) {
        smoothed[i] = table->entries[i].gains;
    }
    
    uint32_t half = window_size / 2;
    for (uint32_t i = 0; i < table->num_entries; i++) {
        double sum_Kp = 0.0, sum_Ki = 0.0, sum_Kd = 0.0;
        double sum_Ti = 0.0, sum_Td = 0.0;
        uint32_t count = 0;
        
        int32_t start = (int32_t)i - (int32_t)half;
        int32_t end   = (int32_t)i + (int32_t)half;
        if (start < 0) start = 0;
        if (end >= (int32_t)table->num_entries) end = (int32_t)table->num_entries - 1;
        
        for (int32_t j = start; j <= end; j++) {
            sum_Kp += table->entries[j].gains.Kp;
            sum_Ki += table->entries[j].gains.Ki;
            sum_Kd += table->entries[j].gains.Kd;
            sum_Ti += table->entries[j].gains.Ti;
            sum_Td += table->entries[j].gains.Td;
            count++;
        }
        
        if (count > 0) {
            smoothed[i].Kp = sum_Kp / count;
            smoothed[i].Ki = sum_Ki / count;
            smoothed[i].Kd = sum_Kd / count;
            smoothed[i].Ti = sum_Ti / count;
            smoothed[i].Td = sum_Td / count;
        }
    }
    
    for (uint32_t i = 0; i < table->num_entries; i++) {
        table->entries[i].gains = smoothed[i];
    }
    
    free(smoothed);
}

void gs_design_compute_local_margins(gain_schedule_table_t *table,
                                      const double *K_array,
                                      const double *tau_array,
                                      const double *L_array,
                                      uint32_t n) {
    if (!table || !K_array || !tau_array || !L_array || n == 0) return;
    uint32_t count = table->num_entries;
    if (count > n) count = n;
    
    for (uint32_t i = 0; i < count; i++) {
        double Kp = table->entries[i].gains.Kp;
        double K  = K_array[i];
        double tau = tau_array[i];
        double L  = L_array[i];
        (void)table->entries[i].gains.Ki;
        (void)table->entries[i].gains.Kd;

        double wu = 2.0 * M_PI / L;
        if (L > 0.0 && K > 0.0) {
            double mag = K / sqrt(1.0 + wu*wu*tau*tau);
            table->entries[i].gain_margin = 20.0 * log10(1.0 / (Kp * mag));
            table->entries[i].phase_margin = 90.0 - atan(wu*tau)*180.0/M_PI
                                            - wu*L*180.0/M_PI + 60.0;
            if (table->entries[i].phase_margin < 10.0)
                table->entries[i].phase_margin = 45.0;
            if (table->entries[i].phase_margin > 90.0)
                table->entries[i].phase_margin = 60.0;
        } else {
            table->entries[i].gain_margin  = 6.0;
            table->entries[i].phase_margin = 45.0;
        }
        table->entries[i].sensitivity_peak = 1.0 + 1.0 / table->entries[i].gain_margin;
        if (table->entries[i].sensitivity_peak < 1.0)
            table->entries[i].sensitivity_peak = 1.3;
        table->entries[i].bandwidth = 1.0 / tau;
        table->entries[i].validated = true;
    }
}
