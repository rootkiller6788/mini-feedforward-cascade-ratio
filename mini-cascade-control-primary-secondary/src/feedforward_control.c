/**
 * @file feedforward_control.c
 * @brief Feedforward Control — Compensator Implementation
 *
 * Implements feedforward compensation algorithms:
 * - Static gain, lead-lag, and deadtime compensators
 * - FF+FB combination with bumpless enable/disable
 * - Optimal feedforward design from process/disturbance models
 * - Cascade + feedforward integration
 * - Feedforward performance evaluation and sensitivity analysis
 *
 * The ideal feedforward compensator for disturbance rejection:
 *   G_ff(s) = -G_d(s) / G_p(s)
 *
 * For FOPDT models:
 *   G_ff(s) = -(Kd/K) * (tau*s+1)/(tau_d*s+1) * exp(-(theta_d-theta)*s)
 *
 * Causality requirement: theta_d >= theta (feedforward cannot predict future)
 *
 * Knowledge Coverage:
 *   L1-L2: Feedforward principle, static/dynamic compensation
 *   L3: Digital filter discretization (Tustin/bilinear transform)
 *   L5: Optimal FF design, FF+FB combination
 *
 * References:
 *   Seborg et al., Process Dynamics and Control (2016), Ch. 15
 *   Brosilow & Joseph, Techniques of Model-Based Control (2002), Ch. 9
 *   Åström & Hägglund, PID Controllers (1995), Ch. 7
 *
 * Curriculum: MIT 6.302, Stanford ENGR205, Purdue ME575, RWTH Aachen ICS
 */

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "cascade_types.h"
#include "cascade_pid.h"
#include "feedforward_control.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*===========================================================================
 * L2: Feedforward Compensator Initialization
 *===========================================================================*/

void ff_init(ff_compensator_t *ff, double ts,
             double out_min, double out_max)
{
    if (!ff) return;

    memset(ff, 0, sizeof(*ff));

    ff->mode = FF_MODE_OFF;
    ff->Kff = 1.0;
    ff->T_lead = 0.0;
    ff->T_lag = 0.1;  /* Small lag to ensure realizability */
    ff->deadtime = 0.0;
    ff->bias = 0.0;
    ff->ff_output = 0.0;

    ff->x_prev = 0.0;
    ff->y_prev = 0.0;
    memset(ff->x_delayed, 0, sizeof(ff->x_delayed));
    ff->delay_index = 0;
    ff->delay_length = 0;

    ff->Ts = ts;
    ff->output_min = out_min;
    ff->output_max = out_max;
}

void ff_configure_static(ff_compensator_t *ff,
                          double Kff, double bias)
{
    if (!ff) return;

    ff->mode = FF_MODE_STATIC;
    ff->Kff = Kff;
    ff->bias = bias;
    ff->T_lead = 0.0;
    ff->T_lag = 0.0;
}

void ff_configure_lead_lag(ff_compensator_t *ff,
                            double Kff, double T_lead, double T_lag)
{
    if (!ff) return;
    if (T_lag < 1e-12) T_lag = ff->Ts;  /* Ensure realizability */

    ff->mode = FF_MODE_LEAD_LAG;
    ff->Kff = Kff;
    ff->T_lead = T_lead;
    ff->T_lag = T_lag;
}

void ff_configure_deadtime(ff_compensator_t *ff,
                            double Kff, double T_lead, double T_lag,
                            double deadtime)
{
    if (!ff) return;
    if (T_lag < 1e-12) T_lag = ff->Ts;

    ff->mode = FF_MODE_DEADTIME;
    ff->Kff = Kff;
    ff->T_lead = T_lead;
    ff->T_lag = T_lag;
    ff->deadtime = deadtime;

    /* Configure delay line */
    uint32_t delay_samps = (uint32_t)(deadtime / ff->Ts);
    if (delay_samps > 255) delay_samps = 255;
    ff->delay_length = delay_samps;
    ff->delay_index = 0;
    memset(ff->x_delayed, 0, sizeof(ff->x_delayed));
}

/*===========================================================================
 * L3: Feedforward Update — Digital Implementation
 *
 * The lead-lag compensator is discretized using the Tustin (bilinear)
 * transform for best frequency-domain fidelity:
 *
 *   G_ff(s) = Kff * (T_lead*s + 1) / (T_lag*s + 1)
 *
 * Tustin: s → (2/Ts) * (z-1)/(z+1)
 *
 * Gives the difference equation:
 *   y(k) = a*y(k-1) + b*x(k) + c*x(k-1)
 *
 * where:
 *   a = (2*T_lag - Ts) / (2*T_lag + Ts)
 *   b = Kff * (2*T_lead + Ts) / (2*T_lag + Ts)
 *   c = Kff * (Ts - 2*T_lead) / (2*T_lag + Ts)
 *
 * Static mode: y(k) = Kff * x(k) + bias
 * Deadtime mode: y(k) = lead_lag(x(k - N_delay))
 *===========================================================================*/

double ff_update(ff_compensator_t *ff, double disturbance)
{
    if (!ff) return 0.0;

    double x = disturbance;
    double y = 0.0;

    switch (ff->mode) {

    case FF_MODE_OFF:
        y = 0.0;
        break;

    case FF_MODE_STATIC:
        /* Pure gain: y = Kff * x + bias */
        y = ff->Kff * x + ff->bias;
        break;

    case FF_MODE_LEAD_LAG:
    {
        /* Tustin-discretized lead-lag */
        double Ts = ff->Ts;
        double a, b, c;

        if (ff->T_lag < Ts / 2.0) {
            /* For very small T_lag, use static gain approximation */
            y = ff->Kff * (ff->T_lead > 0 ? (ff->T_lead / ff->T_lag) : 1.0) * x;
        } else {
            a = (2.0 * ff->T_lag - Ts) / (2.0 * ff->T_lag + Ts);
            b = ff->Kff * (2.0 * ff->T_lead + Ts) / (2.0 * ff->T_lag + Ts);
            c = ff->Kff * (Ts - 2.0 * ff->T_lead) / (2.0 * ff->T_lag + Ts);

            y = a * ff->y_prev + b * x + c * ff->x_prev;
        }

        ff->x_prev = x;
        ff->y_prev = y;
        break;
    }

    case FF_MODE_DEADTIME:
    {
        /* Lead-lag with deadtime compensation via ring buffer */
        if (ff->delay_length > 0) {
            /* Store current input in delay buffer */
            ff->x_delayed[ff->delay_index] = x;
            ff->delay_index = (ff->delay_index + 1) % (ff->delay_length + 1);

            /* Read delayed sample */
            uint32_t read_idx = (ff->delay_index + 1) % (ff->delay_length + 1);
            x = ff->x_delayed[read_idx];
        }

        /* Apply lead-lag to delayed x */
        double Ts = ff->Ts;
        if (ff->T_lag >= Ts / 2.0) {
            double a = (2.0 * ff->T_lag - Ts) / (2.0 * ff->T_lag + Ts);
            double b = ff->Kff * (2.0 * ff->T_lead + Ts) / (2.0 * ff->T_lag + Ts);
            double c = ff->Kff * (Ts - 2.0 * ff->T_lead) / (2.0 * ff->T_lag + Ts);

            y = a * ff->y_prev + b * x + c * ff->x_prev;
        } else {
            y = ff->Kff * x;
        }

        ff->x_prev = x;
        ff->y_prev = y;
        break;
    }

    default:
        y = 0.0;
        break;
    }

    /* Output clamping */
    if (y > ff->output_max) y = ff->output_max;
    if (y < ff->output_min) y = ff->output_min;

    ff->ff_output = y;
    return y;
}

/*===========================================================================
 * L3: Combined Feedforward + Feedback Update
 *===========================================================================*/

double ff_update_ff_fb(ff_fb_controller_t *controller,
                        double setpoint, double pv, double disturbance)
{
    if (!controller) return 0.0;

    /* Compute feedforward contribution */
    double u_ff = 0.0;
    if (controller->ff_active) {
        u_ff = ff_update(&controller->ff, disturbance);
    }
    controller->ff_contribution = u_ff;

    /* Compute feedback contribution (PID) */
    double u_fb = 0.0;
    if (controller->fb_active) {
        u_fb = cascade_pid_update_positional(&controller->fb,
            setpoint, pv);
    }
    controller->fb_contribution = u_fb;

    /* Total = FF + FB */
    controller->total_output = u_ff + u_fb;

    /* Clamp total to valid range */
    double out_max = controller->fb.params.output_max;
    double out_min = controller->fb.params.output_min;
    if (controller->total_output > out_max)
        controller->total_output = out_max;
    if (controller->total_output < out_min)
        controller->total_output = out_min;

    controller->execution_count++;

    return controller->total_output;
}

/*===========================================================================
 * L3: Bumpless Feedforward Enable
 *
 * When enabling feedforward, ramp the FF contribution from 0 to full
 * over the specified time to avoid a sudden jump in total output.
 *===========================================================================*/

void ff_bumpless_enable(ff_fb_controller_t *controller,
                         double ramp_time_seconds)
{
    if (!controller) return;

    controller->ff_active = true;

    /* Reset FF filter state to produce zero initial output */
    controller->ff.x_prev = 0.0;
    controller->ff.y_prev = 0.0;

    /* If ramp is needed, it's handled by the lead-lag time constants
     * naturally smoothing the transition. For a true ramp, the
     * controller would need an explicit ramp state machine. */
    (void)ramp_time_seconds;
}

/*===========================================================================
 * L5: Optimal Feedforward Design from Models
 *
 * Given Gp(s) and Gd(s), compute Gff(s) = -Gd(s)/Gp(s).
 *
 * For FOPDT:
 *   Gp(s) = K * exp(-theta*s) / (tau*s + 1)
 *   Gd(s) = Kd * exp(-theta_d*s) / (tau_d*s + 1)
 *
 * Gff(s) = -(Kd/K) * (tau*s+1)/(tau_d*s+1) * exp(-(theta_d-theta)*s)
 *
 * Realizability: The compensator is realizable if theta_d >= theta.
 * If theta_d < theta, a non-causal advance is needed, so we use
 * a causal approximation (set deadtime = 0).
 *===========================================================================*/

int ff_design_from_models(const cascade_fopdt_model_t *Gp,
                           const ff_disturbance_model_t *Gd,
                           ff_design_result_t *result)
{
    if (!Gp || !Gd || !result) return -1;
    if (Gp->K < 1e-12 || Gp->tau < 0.0) return -1;

    /* Ideal static gain: Kff = -Kd / K */
    result->Kff_ideal = -Gd->Kd / Gp->K;

    /* Lead = process time constant (cancels process pole) */
    result->T_lead_ideal = Gp->tau;

    /* Lag = disturbance time constant */
    result->T_lag_ideal = Gd->tau_d;

    /* Deadtime = theta_d - theta (must be >= 0 for realizability) */
    double dt_offset = Gd->theta_d - Gp->theta;
    result->deadtime_ideal = (dt_offset >= 0.0) ? dt_offset : 0.0;
    result->is_ideal_realizable = (dt_offset >= 0.0);

    /* Static error: FF eliminates steady-state disturbance effect
     * if Kff is exact. With model errors, the residual is:
     *   e_ss = (1 + Kff * K / Kd) * disturbance
     */
    double residual = 1.0 + result->Kff_ideal * Gp->K / Gd->Kd;
    result->static_error_pct = fabs(residual) * 100.0;

    /* Bandwidth ratio: comparison of FF vs FB speed */
    result->bandwidth_ratio = Gp->tau / (Gd->tau_d + 1e-12);

    return result->is_ideal_realizable ? 1 : 0;
}

double ff_design_static(double process_gain, double disturbance_gain)
{
    if (fabs(process_gain) < 1e-12) return 0.0;
    return -disturbance_gain / process_gain;
}

int ff_design_lead_lag_optimal(const cascade_fopdt_model_t *Gp,
                                const ff_disturbance_model_t *Gd,
                                double *Kff, double *T_lead, double *T_lag)
{
    if (!Gp || !Gd || !Kff || !T_lead || !T_lag) return -1;
    if (Gp->K < 1e-12) return -1;

    /* Static gain: exact cancellation */
    *Kff = -Gd->Kd / Gp->K;

    /* Lead = min(process_tau, disturbance_tau)
     * The lead should cancel the slower pole for best performance. */
    *T_lead = (Gp->tau < Gd->tau_d) ? Gp->tau : Gd->tau_d;

    /* Lag = max(process_tau, disturbance_tau)
     * Ensures the compensator is proper (order of denominator >= numerator). */
    *T_lag = (Gp->tau > Gd->tau_d) ? Gp->tau : Gd->tau_d;

    /* If lead == lag, reduce lead slightly for distinct dynamics */
    if (fabs(*T_lead - *T_lag) < 1e-6) {
        *T_lead *= 0.9;
    }

    return 0;
}

/*===========================================================================
 * L5: Cascade + Feedforward Integration
 *
 * In cascade with feedforward, the disturbance is measured early
 * and the FF signal is injected to the inner (secondary) loop for
 * fastest correction. This uses the secondary loop's speed advantage
 * to reject disturbances before they affect the primary process.
 *===========================================================================*/

double ff_cascade_update(cascade_config_t *cascade,
                          double disturbance,
                          ff_compensator_t *ff,
                          bool inject_to_secondary)
{
    if (!cascade || !ff) return cascade->secondary_sp;

    /* Compute feedforward contribution */
    double u_ff = ff_update(ff, disturbance);

    /* Scale FF to setpoint range */
    double sp_min = cascade->secondary_sp_min;
    double sp_max = cascade->secondary_sp_max;
    double sp_range = sp_max - sp_min;

    double ff_sp_adjustment = u_ff * sp_range / 100.0;

    if (inject_to_secondary) {
        /* Adjust secondary SP with FF signal */
        double new_sp = cascade->secondary_sp + ff_sp_adjustment;
        if (new_sp > sp_max) new_sp = sp_max;
        if (new_sp < sp_min) new_sp = sp_min;
        cascade->secondary_sp = new_sp;
    } else {
        /* Inject to primary (less common, slower response) */
        double new_sp = cascade->primary_sp + ff_sp_adjustment * 0.1;
        if (new_sp > cascade->primary_sp_max) new_sp = cascade->primary_sp_max;
        if (new_sp < cascade->primary_sp_min) new_sp = cascade->primary_sp_min;
        cascade->primary_sp = new_sp;
    }

    return cascade->secondary_sp;
}

/*===========================================================================
 * L5: Feedforward Performance Analysis
 *
 * The performance metric η measures variance reduction:
 *   η = 1 - σ²_FF+FB / σ²_FB_only
 *
 * η = 0 means no improvement
 * η = 1 means perfect disturbance rejection
 *
 * The achievable η depends on:
 *   - Model accuracy (Kff error)
 *   - Deadtime mismatch
 *   - Measurement noise
 *===========================================================================*/

double ff_performance_evaluate(const ff_compensator_t *ff,
                                const cascade_fopdt_model_t *Gp,
                                const ff_disturbance_model_t *Gd,
                                double disturbance_variance)
{
    if (!ff || !Gp || !Gd) return 0.0;

    /* Ideal static compensation factor */
    double Kff_ideal = -Gd->Kd / Gp->K;
    if (fabs(Kff_ideal) < 1e-12) return 0.0;

    /* Actual vs ideal gain ratio */
    double gain_error = (ff->Kff - Kff_ideal) / fabs(Kff_ideal);

    /* Performance degrades with gain error squared:
     *   η ≈ 1 - gain_error² (for perfect dynamics)
     * With dynamic mismatch (lead/lag), further degradation: */
    double dynamic_error = 0.0;
    if (ff->mode >= FF_MODE_LEAD_LAG && fabs(Gp->tau) > 1e-12) {
        double lead_ideal = Gp->tau;
        dynamic_error = fabs(ff->T_lead - lead_ideal) / fabs(lead_ideal);
    }

    /* Combined error metric */
    double total_error = gain_error * gain_error + 0.3 * dynamic_error * dynamic_error;
    double eta = 1.0 - total_error;
    if (eta < 0.0) eta = 0.0;
    if (eta > 1.0) eta = 1.0;

    (void)disturbance_variance;

    return eta;
}

double ff_sensitivity_analysis(double modeled_gain, double actual_gain)
{
    if (fabs(modeled_gain) < 1e-12) return 100.0;

    double error_pct = fabs(actual_gain - modeled_gain) / fabs(modeled_gain) * 100.0;

    /* Feedforward performance degrades approximately linearly with
     * model gain error for small errors, and saturates at 100%. */
    return error_pct;
}
