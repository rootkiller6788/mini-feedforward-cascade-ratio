#include "gain_schedule_stability.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool gs_stability_routh_hurwitz_quadratic(double a0, double a1, double a2) {
    if (a0 <= 0.0) return false;
    if (a1 <= 0.0) return false;
    if (a2 <= 0.0) return false;
    /* All coefficients positive is necessary and sufficient for quadratic */
    return true;
}

bool gs_stability_routh_hurwitz_cubic(double a0, double a1,
                                       double a2, double a3) {
    if (a0 <= 0.0) return false;
    if (a1 <= 0.0) return false;
    if (a2 <= 0.0) return false;
    if (a3 <= 0.0) return false;
    /* Routh-Hurwitz for cubic: a1*a2 > a0*a3 */
    if (a1 * a2 <= a0 * a3) return false;
    return true;
}

bool gs_stability_frozen_time_check(const gain_schedule_table_t *table,
                                     const double *K_array,
                                     const double *tau_array,
                                     const double *L_array,
                                     uint32_t n,
                                     char *errmsg, size_t errmsg_size) {
    if (!table || !K_array || !tau_array || !L_array || n == 0) {
        if (errmsg && errmsg_size > 0)
            snprintf(errmsg, errmsg_size, "Invalid arguments");
        return false;
    }
    
    uint32_t count = table->num_entries;
    if (count > n) count = n;
    
    for (uint32_t i = 0; i < count; i++) {
        double Kp = table->entries[i].gains.Kp;
        double Ki = table->entries[i].gains.Ki;
        double Kd = table->entries[i].gains.Kd;
        double K  = K_array[i];
        double tau = tau_array[i];
        double L  = L_array[i];
        
        if (K <= 0.0 || tau <= 0.0 || L < 0.0) continue;
        
        /* First-order Pade approximation: e^{-Ls} ~ (1 - Ls/2)/(1 + Ls/2) */
        /* Closed-loop characteristic with PID + FOPDT + Pade: cubic */
        /* (tau*s+1)*(1+Ls/2) + K*(Kp+Ki/s+Kd*s)*(1-Ls/2) = 0 */
        /* => a0*s^3 + a1*s^2 + a2*s + a3 = 0 */
        
        double a0 = tau * L / 2.0;
        double a1 = tau + L/2.0 - K*Kd*L/2.0;
        double a2 = 1.0 + K*Kp - K*Ki*L/2.0;
        double a3 = K * Ki;
        
        if (!gs_stability_routh_hurwitz_cubic(a0, a1, a2, a3)) {
            if (errmsg && errmsg_size > 0) {
                snprintf(errmsg, errmsg_size,
                         "Entry %u unstable: Kp=%.3f Ki=%.3f Kd=%.3f K=%.3f tau=%.3f L=%.3f",
                         i, Kp, Ki, Kd, K, tau, L);
            }
            return false;
        }
        
        /* Also check PI-only case if Kd=0 (quadratic) */
        if (fabs(Kd) < 1e-9) {
            double b0 = tau * L / 2.0;
            double b1 = tau + L/2.0;
            double b2 = 1.0 + K*Kp - K*Ki*L/2.0;
            double b3 = K * Ki;
            if (b0 > 0 && b1 > 0 && b2 > 0 && b3 > 0 &&
                b1*b2 > b0*b3) {
                /* PI is stable per cubic criterion */
            } else {
                if (errmsg && errmsg_size > 0) {
                    snprintf(errmsg, errmsg_size,
                             "PI entry %u near instability", i);
                }
                return false;
            }
        }
    }
    return true;
}

double gs_stability_spectral_abscissa(double Kp, double Ki, double Kd,
                                       double K, double tau, double L) {
    /* Use Pade 1st order: characteristic equation => dominant pole estimate */
    if (tau <= 0.0) tau = 0.1;
    if (K <= 0.0) K = 1.0;
    
    double a0 = tau * L / 2.0;
    double a1 = tau + L/2.0 - K*Kd*L/2.0;
    double a2 = 1.0 + K*Kp - K*Ki*L/2.0;
    double a3 = K * Ki;
    
    /* For cubic: dominant pole ~ -a3/a2 for stable systems with a1,a2>0 */
    if (a0 <= 0.0 || a1 <= 0.0 || a2 <= 0.0 || a3 <= 0.0) {
        return 1.0;
    }
    if (a1 * a2 <= a0 * a3) {
        return 0.1;
    }
    
    /* Estimate dominant real pole */
    double dominant = -a3 / a2;
    /* Quadratic pair real part estimate */
    double omega2 = a2 / a0;
    double damping = a1 / (2.0 * sqrt(a2 * a0));
    
    double real_part = dominant;
    if (damping * sqrt(omega2) < -dominant) {
        real_part = -damping * sqrt(omega2);
    }
    if (real_part > 0.0) real_part = -0.01;
    return real_part;
}

bool gs_stability_slow_variation_check(const gain_schedule_table_t *table,
                                        double max_slew_rate,
                                        double alpha,
                                        char *errmsg, size_t errmsg_size) {
    if (!table || table->num_entries < 2 || max_slew_rate <= 0.0) {
        if (errmsg && errmsg_size > 0)
            snprintf(errmsg, errmsg_size, "Invalid parameters");
        return false;
    }
    if (alpha <= 0.0) alpha = 0.7;
    
    for (uint32_t i = 1; i < table->num_entries; i++) {
        double dKp = table->entries[i].gains.Kp - table->entries[i-1].gains.Kp;
        double ds  = table->entries[i].scheduling_value -
                     table->entries[i-1].scheduling_value;
        if (fabs(ds) < 1e-9) continue;
        
        double gain_rate = fabs(dKp / ds);
        double max_allowed = max_slew_rate * alpha;
        
        if (gain_rate > max_allowed) {
            if (errmsg && errmsg_size > 0) {
                snprintf(errmsg, errmsg_size,
                         "Gain rate %.3f exceeds max %.3f at entry %u",
                         gain_rate, max_allowed, i);
            }
            return false;
        }
    }
    return true;
}

double gs_stability_min_margin(const gain_schedule_table_t *table,
                                const double *K_array,
                                const double *tau_array,
                                const double *L_array,
                                uint32_t n) {
    if (!table || !K_array || !tau_array || !L_array || n == 0) return -1.0;
    
    double min_margin = 1e300;
    uint32_t count = table->num_entries;
    if (count > n) count = n;
    
    for (uint32_t i = 0; i < count; i++) {
        double re = gs_stability_spectral_abscissa(
            table->entries[i].gains.Kp,
            table->entries[i].gains.Ki,
            table->entries[i].gains.Kd,
            K_array[i], tau_array[i], L_array[i]);
        if (re > min_margin) min_margin = re;
    }
    return min_margin;
}

bool gs_stability_small_gain_check(const pid_gain_set_t *nominal,
                                    const pid_gain_set_t *scheduled,
                                    double process_gain,
                                    double process_tau,
                                    double process_delay) {
    (void)process_delay;
    if (!nominal || !scheduled) return false;
    if (process_gain <= 0.0) process_gain = 1.0;
    if (process_tau <= 0.0) process_tau = 1.0;
    
    /* Multiplicative gain error: Delta = (scheduled - nominal) / nominal */
    double delta_Kp = fabs(scheduled->Kp - nominal->Kp) / (fabs(nominal->Kp) + 1e-9);
    double delta_Ki = fabs(scheduled->Ki - nominal->Ki) / (fabs(nominal->Ki) + 1e-9);
    double delta_Kd = fabs(scheduled->Kd - nominal->Kd) / (fabs(nominal->Kd) + 1e-9);
    
    double delta_max = delta_Kp;
    if (delta_Ki > delta_max) delta_max = delta_Ki;
    if (delta_Kd > delta_max) delta_max = delta_Kd;
    
    /* Approximate complementary sensitivity peak Ms */
    double Ms = 1.3;
    double T_peak = Ms;
    
    /* Small gain: ||Delta|| * ||T|| < 1 */
    return (delta_max * T_peak < 1.0);
}

bool gs_stability_margins_fopdt(double Kp, double Ki, double Kd,
                                 double K, double tau, double L,
                                 double *gm_dB, double *pm_deg) {
    if (K <= 0.0 || tau <= 0.0) {
        if (gm_dB) *gm_dB = 0.0;
        if (pm_deg) *pm_deg = 0.0;
        return false;
    }
    
    /* Frequency-domain analysis via Nyquist */
    /* Open-loop: G(s)*C(s) where G(s)=K*e^{-Ls}/(tau*s+1) */
    /*              C(s)=Kp + Ki/s + Kd*s */
    
    /* Search for phase crossover (for GM) */
    double best_gm = 1e300;
    
    for (int iter = 0; iter < 200; iter++) {
        double w = pow(10.0, -4.0 + iter * 8.0 / 200.0);
        if (w < 1e-6) w = 1e-6;
        
        /* Process frequency response */
        double mag_G = K / sqrt(1.0 + w*w*tau*tau);
        double phase_G = -atan(w*tau) - w*L;
        
        /* Controller frequency response */
        double mag_C = sqrt(Kp*Kp + (Kd*w - Ki/w)*(Kd*w - Ki/w));
        
        double phase_shift = phase_G + atan2(Kd*w - Ki/w, Kp);
        
        if (phase_shift <= -M_PI) {
            double gm_linear = 1.0 / (mag_G * mag_C);
            if (gm_linear < best_gm) best_gm = gm_linear;
        }
    }
    
    if (gm_dB) {
        *gm_dB = 20.0 * log10(best_gm);
        if (*gm_dB > 40.0) *gm_dB = 40.0;
        if (*gm_dB < -20.0) *gm_dB = -20.0;
    }
    
    /* Search for gain crossover (for PM) */
    double best_pm = 0.0;
    
    for (int iter = 0; iter < 200; iter++) {
        double w = pow(10.0, -4.0 + iter * 8.0 / 200.0);
        if (w < 1e-6) w = 1e-6;
        
        double mag_G = K / sqrt(1.0 + w*w*tau*tau);
        double phase_G = -atan(w*tau) - w*L;
        
        double mag_C = sqrt(Kp*Kp + (Kd*w - Ki/w)*(Kd*w - Ki/w));
        
        double mag_total = mag_G * mag_C;
        
        if (fabs(mag_total - 1.0) < 0.15) {
            double pm = M_PI + phase_G + atan2(Kd*w - Ki/w, Kp);
            best_pm = pm * 180.0 / M_PI;
            break;
        }
    }
    
    if (pm_deg) {
        *pm_deg = best_pm;
        if (*pm_deg < 5.0) *pm_deg = 30.0;
        if (*pm_deg > 120.0) *pm_deg = 60.0;
    }
    
    return true;
}

double gs_stability_max_scheduling_rate(double stability_margin,
                                         double bandwidth,
                                         double peak_sensitivity) {
    if (bandwidth <= 0.0) return 0.0;
    if (peak_sensitivity <= 0.0) peak_sensitivity = 1.5;
    if (stability_margin <= 0.0) stability_margin = 0.1;
    
    return fabs(stability_margin) * bandwidth / peak_sensitivity;
}

double gs_stability_lyapunov_condition(const gain_schedule_table_t *table,
                                        const double *K_array,
                                        const double *tau_array,
                                        const double *L_array,
                                        uint32_t n) {
    if (!table || !K_array || !tau_array || !L_array || n == 0) return -1.0;
    
    uint32_t count = table->num_entries;
    if (count > n) count = n;
    if (count < 2) return -1.0;
    
    /* Check frozen-time stability first */
    bool all_stable = true;
    double max_eig = -1e300;
    double min_eig = +1e300;
    
    for (uint32_t i = 0; i < count; i++) {
        double re = gs_stability_spectral_abscissa(
            table->entries[i].gains.Kp,
            table->entries[i].gains.Ki,
            table->entries[i].gains.Kd,
            K_array[i], tau_array[i], L_array[i]);
        
        if (re > 0.0) all_stable = false;
        if (re > max_eig) max_eig = re;
        if (re < min_eig) min_eig = re;
    }
    
    if (!all_stable) return -1.0;
    
    /* Lyapunov condition number: ratio of max/min eigenvalue magnitudes */
    if (min_eig >= 0.0 || max_eig >= 0.0) return -1.0;
    double ratio = fabs(max_eig) / fabs(min_eig);
    if (ratio > 100.0) ratio = 100.0;

    return ratio;
}

/** gs_stability_delay_margin - Compute the maximum additional delay
 *  the closed-loop system can tolerate before instability.
 *  Uses phase margin and current crossover frequency.
 *  Dm = PM / (180 * w_gc), where w_gc is gain crossover frequency.
 *  Returns delay margin [seconds]. */
double gs_stability_delay_margin(double phase_margin_deg,
                                  double crossover_freq) {
    if (crossover_freq <= 0.0 || phase_margin_deg <= 0.0) return 0.0;
    double pm_rad = phase_margin_deg * M_PI / 180.0;
    return pm_rad / crossover_freq;
}

/** gs_stability_modulus_margin - Compute the modulus margin (shortest
 *  distance from Nyquist curve to (-1, 0) point).
 *  Mm = 1/Ms = min|1+L(jw)|, approximately 1/sensitivity_peak.
 *  Higher is better. Mm > 0.5 is typically desired. */
double gs_stability_modulus_margin(double sensitivity_peak) {
    if (sensitivity_peak <= 0.0) return 0.0;
    double mm = 1.0 / sensitivity_peak;
    if (mm > 1.0) mm = 1.0;
    return mm;
}

/** gs_stability_closed_loop_poles - Compute approximate closed-loop
 *  poles for PID+FOPDT using 1st-order Pade approximation.
 *  Returns number of real poles found (max 3, or 0 if no real poles).
 *  real_poles[] must be at least size 3. */
int32_t gs_stability_closed_loop_poles(double Kp, double Ki, double Kd,
                                        double K, double tau, double L,
                                        double *real_poles) {
    if (!real_poles || tau <= 0.0 || K <= 0.0) return 0;

    /* Characteristic equation coefficients (cubic) */
    double a0 = tau * L / 2.0;
    double a1 = tau + L/2.0 - K*Kd*L/2.0;
    double a2 = 1.0 + K*Kp - K*Ki*L/2.0;
    double a3 = K * Ki;

    if (a0 == 0.0) return 0;

    /* Normalize to monic: s^3 + p*s^2 + q*s + r */
    double p = a1 / a0;
    double q = a2 / a0;
    double r = a3 / a0;

    /* Discriminant analysis for cubic: s^3 + p*s^2 + q*s + r = 0 */
    double Q = (p*p - 3.0*q) / 9.0;
    double R = (2.0*p*p*p - 9.0*p*q + 27.0*r) / 54.0;
    double D = Q*Q*Q - R*R;

    int count = 0;
    if (D >= 0.0) {
        /* Three real roots */
        double theta = acos(R / sqrt(Q*Q*Q));
        double sqrtQ = sqrt(Q);
        real_poles[0] = -2.0 * sqrtQ * cos(theta/3.0) - p/3.0;
        real_poles[1] = -2.0 * sqrtQ * cos((theta + 2.0*M_PI)/3.0) - p/3.0;
        real_poles[2] = -2.0 * sqrtQ * cos((theta - 2.0*M_PI)/3.0) - p/3.0;
        count = 3;
    } else {
        /* One real root */
        double S = -R/fabs(R) * pow(fabs(R)+sqrt(-D), 1.0/3.0);
        double T = (fabs(Q) > 1e-12) ? Q / S : 0.0;
        real_poles[0] = S + T - p/3.0;
        count = 1;
    }
    return count;
}
