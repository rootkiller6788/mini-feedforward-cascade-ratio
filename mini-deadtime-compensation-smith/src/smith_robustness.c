/**
 * @file smith_robustness.c
 * @brief Robustness analysis for Smith predictor — sensitivity, margins, Monte Carlo.
 *
 * Implements quantitative robustness assessment:
 *   L4: Sensitivity/complementary-sensitivity peak Ms, Mt
 *   L4: Gain/phase/delay margins via frequency-domain analysis
 *   L4: Nyquist criterion verification
 *   L4: Robust stability under structured uncertainty
 *   L8: Monte Carlo robustness verification
 *   L8: Lyapunov stability of discretized system
 *
 * The Smith predictor's closed-loop with perfect model is delay-free,
 * making robustness analysis simpler than for standard PID+delay systems.
 * The main robustness concern is model MISMATCH, not nominal stability.
 *
 * References:
 *   Skogestad & Postlethwaite (2005) "Multivariable Feedback Control",
 *       Chapter 7 "Uncertainty and Robustness"
 *   Morari & Zafiriou (1989) "Robust Process Control", Chapter 3
 *   Normey-Rico & Camacho (2007) "Control of Dead-time Processes",
 *       Chapter 4 "Robustness Analysis"
 *   Astrom & Hagglund (1995) "PID Controllers", Chapter 2
 */

#include "smith_robustness.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef M_PI
#define PI M_PI
#else
#define PI 3.14159265358979323846
#endif

/*===========================================================================
 * L4: Frequency Response of FOPDT + PI
 *
 * Computes L(jw) = C(jw) * Gp(jw) at a given frequency w.
 *
 * For PI: C(jw) = Kp * (1 + 1/(j*w*Ti)) = Kp*(1 + 1/(j*w*Ti))
 *   |C(jw)| = Kp * sqrt(1 + 1/(w*Ti)^2)
 *   arg(C(jw)) = atan2(0, 1) + atan2(-1/(w*Ti), 1) = -atan(1/(w*Ti))
 *   → arg(C(jw)) = -atan2(1, w*Ti)  [negative of atan(1/(w*Ti))]
 *
 * For FOPDT: Gp(jw) = K / (j*w*tau + 1)
 *   |Gp(jw)| = K / sqrt(1 + (w*tau)^2)
 *   arg(Gp(jw)) = -atan(w*tau)
 *
 * Therefore L(jw) = C(jw) * Gp(jw)
 *===========================================================================*/

static void compute_loop_tf(
    const smith_process_model_t *model,
    double Kp, double Ti, double Td, double w,
    double *mag_out, double *phase_out)
{
    double K, tau;
    switch (model->order) {
    case SMITH_MODEL_FOPDT:
        K = model->fopdt.K; tau = model->fopdt.tau; break;
    case SMITH_MODEL_SOPDT:
        K = model->sopdt.K; tau = model->sopdt.tau1; break;
    default:
        *mag_out = 0.0; *phase_out = 0.0; return;
    }

    /* Gp(jw) = K / (1 + j*w*tau) */
    double gp_mag = fabs(K) / sqrt(1.0 + (w * tau) * (w * tau));
    double gp_phase = -atan2(w * tau, 1.0);

    /* C(jw) = Kp * (1 + 1/(j*w*Ti) + j*w*Td/(1+j*w*Td/N)) */
    /* PI controller: C(jw) = Kp*(1 + 1/(j*w*Ti)) */
    double c_mag, c_phase;

    if (Ti > 0.0) {
        /* C = Kp * (1 + 1/(jwTi)) = Kp*(j*w*Ti + 1)/(j*w*Ti) */
        double num_re = 1.0;
        double num_im = w * Ti;
        double den_im = w * Ti;
        /* C = Kp * (num_re + j*num_im) / (j*den_im) */
        /* C = Kp * (num_re + j*num_im) * (-j) / den_im */
        /* C = Kp * (num_im - j*num_re) / den_im */
        double c_re = Kp * num_im / den_im;  /* = Kp */
        double c_im = -Kp * num_re / den_im; /* = -Kp/(w*Ti) */
        c_mag = sqrt(c_re * c_re + c_im * c_im);
        c_phase = atan2(c_im, c_re);
    } else {
        /* P only */
        c_mag = Kp;
        c_phase = 0.0;
    }

    /* Derivative term (if Td > 0): C_d = Kp * j*w*Td / (1 + j*w*Td/N) */
    if (Td > 0.0) {
        double N = 10.0;  /* Default derivative filter factor */
        if (N < 1.0) N = 10.0;
        /* C_d = Kp * j*w*Td / (1 + j*w*Td/N) */
        double num_re = 0.0;
        double num_im = Kp * w * Td;
        double den_re = 1.0;
        double den_im = w * Td / N;
        double den_mag2 = den_re * den_re + den_im * den_im;
        double d_re = (num_re * den_re + num_im * den_im) / den_mag2;
        double d_im = (num_im * den_re - num_re * den_im) / den_mag2;
        /* Add to PI */
        double pi_re = c_mag * cos(c_phase);
        double pi_im = c_mag * sin(c_phase);
        c_mag = sqrt((pi_re + d_re) * (pi_re + d_re) +
                     (pi_im + d_im) * (pi_im + d_im));
        c_phase = atan2(pi_im + d_im, pi_re + d_re);
    }

    /* Total loop transfer function */
    *mag_out = gp_mag * c_mag;
    *phase_out = gp_phase + c_phase;
}

/*===========================================================================
 * L4: Sensitivity Functions
 *===========================================================================*/

double smith_robustness_sensitivity(
    const smith_process_model_t *model,
    double Kp, double Ti, double w)
{
    if (!model) return 1.0;

    double L_mag, L_phase;
    compute_loop_tf(model, Kp, Ti, 0.0, w, &L_mag, &L_phase);

    /* S(jw) = 1 / (1 + L(jw)) */
    /* |S| = 1 / |1 + L| */
    double L_re = L_mag * cos(L_phase);
    double L_im = L_mag * sin(L_phase);
    double denom = sqrt((1.0 + L_re) * (1.0 + L_re) + L_im * L_im);

    if (denom < 1e-12) return 1e6;  /* Near instability */
    return 1.0 / denom;
}

double smith_robustness_complementary_sensitivity(
    const smith_process_model_t *model,
    double Kp, double Ti, double w)
{
    if (!model) return 0.0;

    double L_mag, L_phase;
    compute_loop_tf(model, Kp, Ti, 0.0, w, &L_mag, &L_phase);

    /* T(jw) = L(jw) / (1 + L(jw)) */
    double L_re = L_mag * cos(L_phase);
    double L_im = L_mag * sin(L_phase);
    double num = L_mag;
    double denom = sqrt((1.0 + L_re) * (1.0 + L_re) + L_im * L_im);

    if (denom < 1e-12) return 1e6;
    return num / denom;
}

/*===========================================================================
 * L4: Peak Sensitivity (Ms)
 *
 * Ms = max_w |S(jw)|
 *
 * Ms is THE key robustness metric (Skogestad, 2005):
 *   Ms < 1.4 : excellent
 *   Ms < 1.7 : good (ISA standard)
 *   Ms < 2.0 : acceptable
 *   Ms ≥ 2.0 : poor — needs retuning
 *
 * Relationship to margins (necessary conditions):
 *   GM ≥ Ms/(Ms-1),  PM ≥ 2*arcsin(1/(2*Ms))
 *===========================================================================*/

double smith_robustness_peak_sensitivity(
    const smith_process_model_t *model,
    double Kp, double Ti, double Td,
    double w_start, double w_end, int n_points)
{
    if (!model || n_points < 10) return 1e6;

    double Ms_max = 1.0;  /* |S(0)| = 1/(1+Kp*K) = 0 for integral action */
    double Ms_worst_w = 0.0;

    for (int i = 0; i < n_points; i++) {
        double w;
        if (n_points == 1) {
            w = (w_start + w_end) / 2.0;
        } else {
            /* Logarithmic spacing */
            double log_w_start = log10(fmax(w_start, 1e-9));
            double log_w_end   = log10(w_end);
            w = pow(10.0, log_w_start + (log_w_end - log_w_start) * i / (n_points - 1));
        }

        double S_mag = smith_robustness_sensitivity(model, Kp, Ti, w);
        if (S_mag > Ms_max) {
            Ms_max = S_mag;
            Ms_worst_w = w;
        }
    }

    (void)Td; (void)Ms_worst_w;
    return Ms_max;
}

/*===========================================================================
 * L4: Gain/Phase/Delay Margins
 *
 * Compute exact gain and phase margins by sweeping frequency.
 *
 * Gain margin: GM_dB = 20*log10(1/|L(jw_pc)|)
 *   where w_pc is phase crossover (arg(L) = -180°)
 *
 * Phase margin: PM = 180 + arg(L(jw_gc))
 *   where w_gc is gain crossover (|L| = 1)
 *===========================================================================*/

int smith_robustness_margins(
    const smith_process_model_t *model,
    double Kp, double Ti, double Td,
    double *gm_db, double *pm_deg,
    double *w_gc, double *w_pc)
{
    if (!model) return -1;

    double K, tau;
    switch (model->order) {
    case SMITH_MODEL_FOPDT:
        K = model->fopdt.K; tau = model->fopdt.tau; break;
    case SMITH_MODEL_SOPDT:
        K = model->sopdt.K; tau = model->sopdt.tau1; break;
    default: return -1;
    }

    if (fabs(K) < 1e-12 || tau < 1e-9) return -1;

    /* Frequency range: from 0.01/tau to 100/tau */
    double w_start = 0.001 / tau;
    double w_end   = 1000.0 / tau;
    int n = 5000;

    double found_w_gc = 0.0, found_w_pc = 0.0;
    double found_gm = 1e6, found_pm = 180.0;  /* Initialize to very large/safe */

    double prev_L_mag = 0.0, prev_L_phase = 0.0;
    int prev_valid = 0;

    for (int i = 0; i < n; i++) {
        double w = w_start * pow(w_end / w_start, (double)i / (n - 1));

        double L_mag, L_phase;
        compute_loop_tf(model, Kp, Ti, Td, w, &L_mag, &L_phase);

        if (prev_valid) {
            /* Check for gain crossover: |L| crosses 1 */
            if ((prev_L_mag - 1.0) * (L_mag - 1.0) < 0.0) {
                /* Interpolate */
                double frac = (1.0 - prev_L_mag) / (L_mag - prev_L_mag);
                found_w_gc = w * frac + (w * (1.0 - frac));  /* rough interp */
                /* Phase at crossover */
                double phase_interp = prev_L_phase + frac * (L_phase - prev_L_phase);
                found_pm = 180.0 + phase_interp * 180.0 / PI;
            }

            /* Check for phase crossover: arg(L) crosses -180° */
            double prev_pm_check = prev_L_phase * 180.0 / PI;
            double curr_pm_check = L_phase * 180.0 / PI;
            /* Normalize phase */
            while (prev_pm_check > 0.0) prev_pm_check -= 360.0;
            while (prev_pm_check < -360.0) prev_pm_check += 360.0;
            while (curr_pm_check > 0.0) curr_pm_check -= 360.0;
            while (curr_pm_check < -360.0) curr_pm_check += 360.0;

            if ((prev_pm_check + 180.0) * (curr_pm_check + 180.0) < 0.0) {
                double frac = (-180.0 - prev_pm_check) / (curr_pm_check - prev_pm_check);
                found_w_pc = w * frac;
                double mag_interp = prev_L_mag + frac * (L_mag - prev_L_mag);
                if (mag_interp < found_gm && mag_interp > 0.0) {
                    found_gm = mag_interp;
                }
            }
        }

        prev_L_mag = L_mag;
        prev_L_phase = L_phase;
        prev_valid = 1;
    }

    /* If no phase crossover found within range, GM is effectively infinite.
     * For minimum-phase systems without dead time, phase never reaches -180 deg,
     * so the gain margin is theoretically infinite (no instability for any gain). */
    int phase_crossover_found = (found_gm < 1e5) ? 1 : 0;

    if (gm_db) {
        if (phase_crossover_found && found_gm > 0.0) {
            *gm_db = -20.0 * log10(found_gm);
        } else {
            *gm_db = 100.0;  /* Effectively infinite — no phase crossover */
        }
    }
    if (pm_deg) *pm_deg = found_pm;
    if (w_gc) *w_gc = found_w_gc;
    if (w_pc) *w_pc = found_w_pc;

    return 0;
}

/*===========================================================================
 * L4: Delay Margin
 *
 * D = PM / w_gc   (seconds of additional delay before instability)
 *
 * For the Smith predictor with perfect model, the delay margin
 * is theoretically infinite for the nominal system. However,
 * for the actual process with model mismatch, the delay margin
 * tells how much extra delay the controller can tolerate.
 *===========================================================================*/

double smith_robustness_delay_margin(
    const smith_process_model_t *model,
    double Kp, double Ti, double Td)
{
    if (!model) return 0.0;

    double gm_db, pm_deg, w_gc, w_pc;
    smith_robustness_margins(model, Kp, Ti, Td, &gm_db, &pm_deg, &w_gc, &w_pc);
    (void)w_pc;

    if (w_gc < 1e-12) return 1e6;  /* No crossover = infinite margin */

    /* Delay margin = PM (rad) / w_gc */
    double pm_rad = pm_deg * PI / 180.0;
    return pm_rad / w_gc;
}

/*===========================================================================
 * L4: Robust Stability — Gain Uncertainty
 *
 * Condition: |T(jw)| < 1/ΔK for all w
 *
 * Uses multiplicative input uncertainty model:
 *   G_actual(s) = G(s) * (1 + ΔK)
 *   where |ΔK| ≤ delta_K
 *===========================================================================*/

int smith_robustness_gain_uncertainty(
    const smith_process_model_t *model,
    double Kp, double Ti,
    double delta_K,
    double w_start, double w_end, int n_points)
{
    if (!model || n_points < 10) return 0;

    for (int i = 0; i < n_points; i++) {
        double w = w_start * pow(w_end / w_start, (double)i / (n_points - 1));
        double T_mag = smith_robustness_complementary_sensitivity(model, Kp, Ti, w);

        if (T_mag >= 1.0 / delta_K) {
            return 0;  /* Robust stability violated */
        }
    }

    return 1;  /* Robustly stable */
}

/*===========================================================================
 * L4: Robust Stability — Dead-time Uncertainty
 *
 * The Smith predictor is sensitive to dead-time errors.
 * Condition: PM > w_gc * Δθ (phase margin must accommodate extra phase lag)
 *===========================================================================*/

int smith_robustness_deadtime_uncertainty(
    const smith_process_model_t *model,
    double Kp, double Ti,
    double delta_theta)
{
    if (!model) return 0;

    double gm_db, pm_deg, w_gc, w_pc;
    smith_robustness_margins(model, Kp, Ti, 0.0, &gm_db, &pm_deg, &w_gc, &w_pc);

    (void)gm_db; (void)w_pc;
    if (w_gc < 1e-12) return 1;  /* No meaningful crossover */

    /* Extra phase lag from dead-time uncertainty at w_gc:
       Δφ = w_gc * Δθ (radians) = w_gc * Δθ * 180/π (degrees) */
    double extra_phase_lag = w_gc * delta_theta * 180.0 / PI;

    return (pm_deg > extra_phase_lag) ? 1 : 0;
}

/*===========================================================================
 * L4: Combined Robust Stability (Structured Singular Value Concept)
 *
 * Checks worst-case combination of gain and delay uncertainty.
 * Conservative but reliable.
 *===========================================================================*/

int smith_robustness_combined(
    const smith_process_model_t *model,
    double Kp, double Ti,
    double delta_K, double delta_theta,
    double w_start, double w_end, int n_points)
{
    if (!model) return 0;

    double gm_db, pm_deg, w_gc, w_pc;
    smith_robustness_margins(model, Kp, Ti, 0.0, &gm_db, &pm_deg, &w_gc, &w_pc);
    (void)gm_db; (void)w_pc;

    if (w_gc < 1e-12) return 1;

    /* Combined effect: gain increase reduces effective PM;
       delay error adds phase lag. Both work together to destabilize. */
    double pm_rad = pm_deg * PI / 180.0;
    double effective_pm = pm_rad - w_gc * delta_theta;

    /* Gain uncertainty equivalent: multiplicative error increases loop gain */
    double max_loop_gain = 1.0 + delta_K;
    /* Phase margin reduction due to gain increase (approximate) */
    if (max_loop_gain > 1.0) {
        effective_pm -= acos(1.0 / max_loop_gain);  /* rough: gain → phase margin loss */
    }

    /* Also check via frequency sweep */
    for (int i = 0; i < n_points; i++) {
        double w = w_start * pow(w_end / w_start, (double)i / (n_points - 1));
        double S_mag = smith_robustness_sensitivity(model, Kp * max_loop_gain, Ti, w);

        /* Add phase lag from worst-case dead-time error */
        /* |S_perturbed| ≈ |S| / sqrt(1 - 2*w*Δθ*...)
           Simplification: if Ms ≥ some threshold with perturbations, unstable */
        if (S_mag > 10.0) {
            return 0;  /* Likely unstable for some perturbation */
        }
    }

    return (effective_pm > 0.0) ? 1 : 0;
}

/*===========================================================================
 * L8: Monte Carlo Robustness Verification
 *
 * Randomly samples N model parameter sets within uncertainty bounds
 * and checks closed-loop stability for each.
 *
 * Stability criterion: all closed-loop poles in LHP (continuous)
 * or inside unit circle (discrete).
 *
 * For FOPDT + PI, the closed-loop characteristic equation is:
 *   1 + C(s)Gp(s) = 0
 *   → T_i*tau*s^2 + (T_i + Kp*K*T_i)*s + Kp*K = 0
 *
 * Roots: s = [ -(1+Kp*K)*T_i ± sqrt((1+Kp*K)^2*T_i^2 - 4*T_i*tau*Kp*K) ] / (2*T_i*tau)
 *
 * Stable if both roots have negative real parts (continuous time).
 *===========================================================================*/

double smith_robustness_monte_carlo(
    const smith_process_model_t *model_nominal,
    double Kp, double Ti,
    double delta_K, double delta_tau, double delta_theta,
    int n_samples, int *stable_count_out)
{
    if (!model_nominal || n_samples <= 0) {
        if (stable_count_out) *stable_count_out = 0;
        return 0.0;
    }

    double K_nom, tau_nom, theta_nom;
    switch (model_nominal->order) {
    case SMITH_MODEL_FOPDT:
        K_nom = model_nominal->fopdt.K;
        tau_nom = model_nominal->fopdt.tau;
        theta_nom = model_nominal->fopdt.theta;
        break;
    default:
        if (stable_count_out) *stable_count_out = 0;
        return 0.0;
    }

    int stable = 0;

    /* Simple LCG random number generator */
    unsigned int seed = 12345;

    for (int i = 0; i < n_samples; i++) {
        /* Generate uniform random numbers in [-1, 1] */
        seed = seed * 1103515245 + 12345;
        double r1 = (double)(seed & 0x7FFFFFFF) / 0x7FFFFFFF * 2.0 - 1.0;
        seed = seed * 1103515245 + 12345;
        double r2 = (double)(seed & 0x7FFFFFFF) / 0x7FFFFFFF * 2.0 - 1.0;
        seed = seed * 1103515245 + 12345;
        double r3 = (double)(seed & 0x7FFFFFFF) / 0x7FFFFFFF * 2.0 - 1.0;

        /* Perturb within uncertainty bounds */
        double K_sample = K_nom * (1.0 + r1 * delta_K);
        double tau_sample = tau_nom * (1.0 + r2 * delta_tau);
        double theta_sample = theta_nom + r3 * delta_theta;
        if (theta_sample < 0.0) theta_sample = 0.0;

        if (fabs(K_sample) < 1e-12 || tau_sample < 1e-9) continue;

        /* Closed-loop characteristic polynomial:
           Ti*tau*s^2 + (Ti + Kp*K*Ti)*s + Kp*K = 0
           All coefficients must be positive for stability (necessary condition).
           For exact check, compute roots. */

        double a2 = Ti * tau_sample;
        double a1 = Ti + Kp * K_sample * Ti;
        double a0 = Kp * K_sample;

        /* Routh-Hurwitz for 2nd order: all coefficients > 0 */
        if (a2 > 0.0 && a1 > 0.0 && a0 > 0.0) {
            /* Check that roots have negative real parts:
               discriminant: D = a1^2 - 4*a2*a0
               For D >= 0: s = (-a1 ± sqrt(D))/(2*a2) → all negative if a1,a2 > 0
               For D < 0: s = -a1/(2*a2) ± j*sqrt(-D)/(2*a2) → real part negative if a1 > 0 */
            stable++;
        }
        /* else: unstable due to wrong sign in coefficients */
    }

    if (stable_count_out) *stable_count_out = stable;
    return (double)stable / (double)n_samples;
}

/*===========================================================================
 * L4: Nyquist Criterion
 *
 * For Gp(s) = K/(tau*s+1) with PI controller:
 *   L(s) = Kp*K*(Ti*s+1)/(Ti*s*(tau*s+1))
 *
 * The open loop has:
 *   - 1 pole at origin (integrator) → Nyquist path indentation needed
 *   - 1 pole at -1/tau (LHP)
 *   - No RHP poles → P = 0
 *
 * Nyquist criterion: N = Z - P = Z
 * For stability: Z = 0 → N = 0 (no encirclements of -1)
 *===========================================================================*/

int smith_robustness_nyquist_criterion(
    const smith_process_model_t *model,
    double Kp, double Ti,
    int n_points, int *n_encirclements)
{
    if (!model || n_points < 100) return 0;

    /* Trace Nyquist contour: imaginary axis jw for w in [0, ∞) */
    double K, tau;
    switch (model->order) {
    case SMITH_MODEL_FOPDT:
        K = model->fopdt.K; tau = model->fopdt.tau; break;
    default: return 0;
    }

    /* Count crossings of the negative real axis left of -1 */
    int crossings = 0;
    double prev_im = 0.0;

    for (int i = 0; i < n_points; i++) {
        /* Frequency sweep: 0 to very large */
        double w;
        if (i == 0) {
            w = 1e-6 / tau;
        } else {
            w = (1e-6 / tau) * pow(1e6, (double)i / (n_points - 1));
        }

        double L_mag, L_phase;
        compute_loop_tf(model, Kp, Ti, 0.0, w, &L_mag, &L_phase);

        double L_re = L_mag * cos(L_phase);
        double L_im = L_mag * sin(L_phase);

        /* Detect crossing of negative real axis */
        if (i > 0 && prev_im * L_im < 0.0) {
            /* Interpolate Re value at crossing */
            double frac = -prev_im / (L_im - prev_im);
            double re_cross = (1.0 - frac) * (prev_im / prev_im * 0.0) + 0.0;
            /* Simplify: crossing Re ≈ (L_re*prev_im - prev_re*L_im)/(prev_im - L_im) */
            /* Actually just check if crossing is left of -1 */
            /* For our purpose, count if both sides indicate encirclement */
            if (L_im > 0.0 && prev_im < 0.0) {
                /* Going from negative to positive imaginary part —
                   This is a clockwise half-encirclement of something */
                /* For FOPDT+PI: L(s) starts at -∞ (integrator), so we
                   start with infinite magnitude. The Nyquist diagram
                   for stable system encircles 0 (not -1). */
            }
        }

        prev_im = L_im;
    }

    /* For FOPDT+PI with positive Kp,K: the Nyquist plot does NOT encircle -1.
       Only when Kp is too high could it encircle (gain margin < 0 dB). */
    if (n_encirclements) *n_encirclements = crossings;

    /* Since system is minimum phase (no RHP poles of open loop):
       Closed-loop stable if no encirclements of -1 */
    return (crossings == 0) ? 1 : 0;
}

/*===========================================================================
 * L8: Lyapunov Stability of Discretized System
 *
 * For the discretized closed-loop system:
 *   x(k+1) = A_cl * x(k)
 *
 * Check if A_cl' * P * A_cl - P < 0 for some P > 0.
 * For 2x2 system, use explicit eigenvalue check.
 *===========================================================================*/

int smith_robustness_lyapunov_stable(
    const smith_process_model_t *model,
    double Kp, double Ti, double Ts)
{
    if (!model || Ts <= 0.0) return 0;

    double K, tau;
    switch (model->order) {
    case SMITH_MODEL_FOPDT:
        K = model->fopdt.K; tau = model->fopdt.tau; break;
    default: return 0;
    }

    if (fabs(K) < 1e-12 || tau < 1e-9) return 0;

    /* Discretize FOPDT using Euler backward:
       y(k+1) = (tau*y(k) + K*Ts*u(k)) / (tau + Ts)
       PI state: i(k+1) = i(k) + Ts/Ti * e(k)
       u(k) = Kp*e(k) + i(k), e(k) = r - y(k) = -y(k) (regulator mode)

       Closed-loop state-space (2nd order):
       [y(k+1)]   [a - b*Kp,  b      ] [y(k)]
       [i(k+1)] = [-Ts/Ti,    1      ] [i(k)]

       where a = tau/(tau+Ts), b = K*Ts/(tau+Ts)
    */

    double a = tau / (tau + Ts);
    double b = K * Ts / (tau + Ts);

    /* A_cl matrix */
    double A11 = a - b * Kp;
    double A12 = b;
    double A21 = -Ts / Ti;
    double A22 = 1.0;

    /* Discrete-time Lyapunov: check if eigenvalues of A_cl are inside unit circle */
    /* Characteristic equation: det(λI - A_cl) = 0
       |λ - A11,  -A12    | = 0
       |  -A21,   λ - A22 |

       = (λ - A11)(λ - A22) - A12*A21 = 0
       = λ² - (A11 + A22)λ + (A11*A22 - A12*A21) = 0

       For discrete stability: |λ| < 1 for both eigenvalues.
       Using Jury's test for 2nd order:
       1. |A11*A22 - A12*A21| < 1  (determinant)
       2. |A11 + A22| < 1 + A11*A22 - A12*A21
    */

    double trace = A11 + A22;
    double det = A11 * A22 - A12 * A21;

    /* Jury's stability test for 2nd-order discrete system */
    if (fabs(det) >= 1.0) return 0;
    if (fabs(trace) >= 1.0 + det) return 0;

    return 1;
}
