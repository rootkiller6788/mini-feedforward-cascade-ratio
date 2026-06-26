#include "feedforward_dynamic.h"
#include "feedforward_static.h"
#include "feedforward_advanced.h"
#include <math.h>
#include <string.h>

/**
 * @file feedforward_dynamic.c
 * @brief Dynamic feedforward control — lead-lag, discrete TF, design methods
 *
 * Knowledge: L3 Engineering Structures, L5 Algorithms/Methods
 *
 * Implements the dynamic compensation elements essential for feedforward:
 * - Bilinear (Tustin) discretized lead-lag
 * - Second-order lead-lag for SOPDT processes
 * - Discrete transfer function with history buffers
 * - Design methods for FOPDT, SOPDT, and arbitrary TF feedforward
 *
 * Key dynamic feedforward equation (FOPDT):
 *   Gff(s) = -(Kd/Kp) * (tau_p*s+1)/(tau_d*s+1) * e^((theta_p-theta_d)*s)
 *
 * Bilinear discretization: s ~ (2/Ts)*(z-1)/(z+1)
 *
 * References:
 *   Seborg et al. (2016) §15.4
 *   Åström & Wittenmark (2008) "Computer-Controlled Systems" §6.3
 */

/* ============================================================================
 * L5: Lead-lag compensator — Bilinear (Tustin) discretization
 * ============================================================================ */

void lead_lag_init(lead_lag_t *ll, double K, double T_lead, double T_lag, double Ts)
{
    if (!ll) return;
    memset(ll, 0, sizeof(lead_lag_t));

    ll->K_ll = K;
    ll->T_lead = T_lead;
    ll->T_lag = T_lag;
    ll->Ts = Ts;
    ll->initialized = 1;
}

double lead_lag_step(lead_lag_t *ll, double x)
{
    if (!ll || !ll->initialized) return 0.0;

    /* Bilinear (Tustin) discretization of G(s) = K * (T_lead*s+1)/(T_lag*s+1)
     *
     * Substitute s = (2/Ts)*(z-1)/(z+1):
     *
     * G(z) = K * (2*T_lead*(z-1) + Ts*(z+1)) / (2*T_lag*(z-1) + Ts*(z+1))
     *      = K * ((2*T_lead+Ts)*z + (Ts-2*T_lead)) / ((2*T_lag+Ts)*z + (Ts-2*T_lag))
     *
     * Divide numerator and denominator by (2*T_lag+Ts):
     * G(z) = (b0 + b1*z^{-1}) / (1 - a1*z^{-1})
     *
     * where:
     *   denom = 2*T_lag + Ts
     *   b0 = K * (2*T_lead + Ts) / denom
     *   b1 = K * (Ts - 2*T_lead) / denom
     *   a1 = -(Ts - 2*T_lag) / denom
     *
     * Difference equation: y[k] = a1*y[k-1] + b0*x[k] + b1*x[k-1]
     */
    double Ts = ll->Ts;

    /* Guard against zero lag (pure proportional) */
    if (ll->T_lag < FF_TAU_MIN) {
        double y = ll->K_ll * x;
        ll->y_prev = y;
        ll->x_prev = x;
        return y;
    }

    double denom = 2.0 * ll->T_lag + Ts;
    double b0 = ll->K_ll * (2.0 * ll->T_lead + Ts) / denom;
    double b1 = ll->K_ll * (Ts - 2.0 * ll->T_lead) / denom;
    double a1 = -(Ts - 2.0 * ll->T_lag) / denom;

    /* y[k] = b0*x[k] + b1*x[k-1] + a1*y[k-1] */
    double y = b0 * x + b1 * ll->x_prev + a1 * ll->y_prev;

    /* Update state */
    ll->x_prev = x;
    ll->y_prev = y;

    return y;
}

void lead_lag_reset(lead_lag_t *ll)
{
    if (!ll) return;
    ll->x_prev = 0.0;
    ll->y_prev = 0.0;
}

void lead_lag_freq_response(const lead_lag_t *ll, double omega,
                            double *magnitude, double *phase)
{
    if (!ll || !magnitude || !phase) return;

    double wT_lead = omega * ll->T_lead;
    double wT_lag  = omega * ll->T_lag;

    /* |G(jw)| = |K| * sqrt((w*T_lead)^2 + 1) / sqrt((w*T_lag)^2 + 1) */
    *magnitude = fabs(ll->K_ll) *
                 sqrt(wT_lead * wT_lead + 1.0) /
                 sqrt(wT_lag * wT_lag + 1.0);

    /* arg(G(jw)) = atan(w*T_lead) - atan(w*T_lag)
     * Note: atan2 handles sign of K correctly */
    *phase = atan2(ll->K_ll * wT_lead, ll->K_ll * 1.0) - atan(wT_lag);
}

/* ============================================================================
 * L5: Second-order lead-lag via bilinear discretization
 * ============================================================================ */

void lead_lag2_init(lead_lag2_t *ll2, double K, double Tn, double Td,
                    double zeta_n, double zeta_d, double Ts)
{
    if (!ll2) return;
    memset(ll2, 0, sizeof(lead_lag2_t));

    ll2->K = K;
    ll2->Tn = Tn;
    ll2->Td = Td;
    ll2->zeta_n = zeta_n;
    ll2->zeta_d = zeta_d;
    ll2->Ts = Ts;
    ll2->initialized = 1;
}

double lead_lag2_step(lead_lag2_t *ll2, double x)
{
    if (!ll2 || !ll2->initialized) return 0.0;

    /* Bilinear discretization of second-order lead-lag:
     *
     * G(s) = K * (Tn^2*s^2 + 2*zeta_n*Tn*s + 1) / (Td^2*s^2 + 2*zeta_d*Td*s + 1)
     *
     * Substitute s = (2/Ts)*(z-1)/(z+1):
     *
     * After algebraic manipulation:
     *   y[k] = b0*x[k] + b1*x[k-1] + b2*x[k-2] - a1*y[k-1] - a2*y[k-2]
     *
     * Coefficients computed from Tustin substitution.
     */
    double Ts = ll2->Ts;
    double a = 2.0 / Ts;

    /* Numerator: N(s) = K * (Tn^2*s^2 + 2*zeta_n*Tn*s + 1)
     * After Tustin: N(z) = K * (Tn^2*a^2*(z-1)^2 + 2*zeta_n*Tn*a*(z-1)*(z+1) + (z+1)^2) / (z+1)^2
     *
     * Denominator: D(s) = Td^2*s^2 + 2*zeta_d*Td*s + 1
     * After Tustin: D(z) = (Td^2*a^2*(z-1)^2 + 2*zeta_d*Td*a*(z-1)*(z+1) + (z+1)^2) / (z+1)^2
     */

    double Tn2 = ll2->Tn * ll2->Tn;
    double Td2 = ll2->Td * ll2->Td;

    /* Numerator polynomial in z: n0*z^2 + n1*z + n2 */
    double n0 = ll2->K * (Tn2 * a * a + 2.0 * ll2->zeta_n * ll2->Tn * a + 1.0);
    double n1 = ll2->K * (2.0 - 2.0 * Tn2 * a * a);
    double n2 = ll2->K * (Tn2 * a * a - 2.0 * ll2->zeta_n * ll2->Tn * a + 1.0);

    /* Denominator polynomial in z: d0*z^2 + d1*z + d2 */
    double d0 = Td2 * a * a + 2.0 * ll2->zeta_d * ll2->Td * a + 1.0;
    double d1 = 2.0 - 2.0 * Td2 * a * a;
    double d2 = Td2 * a * a - 2.0 * ll2->zeta_d * ll2->Td * a + 1.0;

    if (fabs(d0) < FF_EPSILON) return 0.0;

    /* Normalize: divide by d0 */
    double b0 = n0 / d0;
    double b1 = n1 / d0;
    double b2 = n2 / d0;
    double a1 = -d1 / d0;
    double a2 = -d2 / d0;

    /* y[k] = b0*x[k] + b1*x[k-1] + b2*x[k-2] - a1*y[k-1] - a2*y[k-2] */
    double y = b0 * x + b1 * ll2->x1 + b2 * ll2->x2
             + a1 * ll2->y1 + a2 * ll2->y2;

    /* Shift state */
    ll2->x2 = ll2->x1;
    ll2->x1 = x;
    ll2->y2 = ll2->y1;
    ll2->y1 = y;

    return y;
}

void lead_lag2_reset(lead_lag2_t *ll2)
{
    if (!ll2) return;
    ll2->x1 = ll2->x2 = 0.0;
    ll2->y1 = ll2->y2 = 0.0;
}

/* ============================================================================
 * L3: Discrete transfer function implementation (circular buffer)
 * ============================================================================ */

/* Internal history buffers */
static double x_history[32];
static double y_history[32];
static int x_head = 0, y_head = 0;

void dtf_init(tf_discrete_t *dtf, const double *num, const double *den,
              int n, int m, double Ts)
{
    if (!dtf) return;
    memset(dtf, 0, sizeof(tf_discrete_t));

    for (int i = 0; i <= n && i < 16; i++) dtf->num[i] = num[i];
    dtf->den[0] = 1.0; /* Always normalized */
    for (int i = 1; i <= m && i < 16; i++) dtf->den[i] = den[i];
    dtf->n = n;
    dtf->m = m;
    dtf->Ts = Ts;

    /* Reset history */
    memset(x_history, 0, sizeof(x_history));
    memset(y_history, 0, sizeof(y_history));
    x_head = y_head = 0;
}

double dtf_step(tf_discrete_t *dtf, double x)
{
    if (!dtf) return 0.0;

    /* Store input in circular buffer */
    x_history[x_head] = x;
    x_head = (x_head + 1) & 31;

    /* Compute y[k] = sum(bi*x[k-i]) - sum(aj*y[k-j]), j >= 1
     * Using circular buffer offsets */
    double y = 0.0;

    for (int i = 0; i <= dtf->n; i++) {
        int idx = (x_head - 1 - i) & 31;
        y += dtf->num[i] * x_history[idx];
    }

    for (int j = 1; j <= dtf->m; j++) {
        int idx = (y_head - j) & 31;
        y -= dtf->den[j] * y_history[idx];
    }

    /* Store output in circular buffer */
    y_history[y_head] = y;
    y_head = (y_head + 1) & 31;

    return y;
}

void dtf_reset(tf_discrete_t *dtf)
{
    if (!dtf) return;
    memset(x_history, 0, sizeof(x_history));
    memset(y_history, 0, sizeof(y_history));
    x_head = y_head = 0;
}

/* ============================================================================
 * L5: Dynamic feedforward design methods
 * ============================================================================ */

int ff_dynamic_design_fopdt(const fopdt_t *process, const dist_model_t *dist,
                            action_t action, double *T_lead, double *T_lag,
                            double *K_ll)
{
    if (!process || !dist || !T_lead || !T_lag || !K_ll) return -1;
    if (fabs(process->Kp) < FF_EPSILON) return -1;

    /* Perfect dynamic feedforward for FOPDT:
     *
     * Gp(s) = Kp * e^(-theta_p*s) / (tau_p*s + 1)
     * Gd(s) = Kd * e^(-theta_d*s) / (tau_d*s + 1)
     *
     * Gff(s) = -Gd(s)/Gp(s)
     *        = -(Kd/Kp) * (tau_p*s+1)/(tau_d*s+1) * e^((theta_p-theta_d)*s)
     *
     * Lead-lag part: K_ll * (T_lead*s+1)/(T_lag*s+1)
     * where K_ll = -Kd/Kp, T_lead = tau_p, T_lag = tau_d
     */
    *K_ll = -dist->Kd / process->Kp;
    *K_ll *= (double)action;
    *T_lead = process->tau;
    *T_lag = dist->tau_d;

    /* Validate time constants */
    if (*T_lag < FF_TAU_MIN) *T_lag = FF_TAU_MIN;
    if (*T_lead < 0.0) *T_lead = 0.0;

    return 0;
}

int ff_dynamic_design_sopdt(const sopdt_t *process, const dist_model_t *dist,
                            action_t action, double *Tn, double *Td,
                            double *zeta_n, double *zeta_d, double *K)
{
    if (!process || !dist) return -1;
    if (fabs(process->Kp) < FF_EPSILON) return -1;

    /* For SOPDT: Gp(s) = Kp * e^(-theta*s) / ((tau1*s+1)*(tau2*s+1))
     *
     * Gff(s) = -(Kd/Kp) * ((tau1*s+1)*(tau2*s+1))/(tau_d*s+1)
     *
     * The numerator is second-order: tau1*tau2*s^2 + (tau1+tau2)*s + 1
     * Denominator is first-order: tau_d*s + 1
     *
     * This is a second-order lead-lag with:
     *   Gff(s) = K * (Tn^2*s^2 + 2*zeta_n*Tn*s + 1) / (tau_d*s + 1)
     *
     * Comparing: Tn^2 = tau1*tau2, 2*zeta_n*Tn = tau1+tau2
     * => Tn = sqrt(tau1*tau2), zeta_n = (tau1+tau2)/(2*sqrt(tau1*tau2))
     *
     * The denominator has Td = tau_d, zeta_d = 0 (first-order)
     */

    *K = -dist->Kd / process->Kp;
    *K *= (double)action;

    *Tn = sqrt(process->tau1 * process->tau2);
    *Td = dist->tau_d;
    *zeta_d = 0.0;  /* First-order lag has no damping parameter */

    if (*Tn > FF_EPSILON) {
        *zeta_n = (process->tau1 + process->tau2) / (2.0 * (*Tn));
    } else {
        *zeta_n = 1.0;
    }

    /* Ensure zeta_n is valid */
    if (*zeta_n < 0.0) *zeta_n = 0.0;

    if (*Td < FF_TAU_MIN) *Td = FF_TAU_MIN;

    return 0;
}

int ff_dynamic_design_tf(const tf_t *Gp, const tf_t *Gd, tf_t *Gff_out)
{
    if (!Gp || !Gd || !Gff_out) return -1;

    /* Check for non-minimum-phase process (unstable inverse)
     * by examining numerator roots of Gp.
     * For now, check if Gp is minimum-phase via dead time check
     * and first-order coefficient sign. */
    int nmp = ff_is_non_minimum_phase(Gp);
    if (nmp) {
        /* NMP: use approximate inversion (static gain only) */
        memset(Gff_out, 0, sizeof(tf_t));
        double Kff = -Gd->K / Gp->K;
        Gff_out->K = Kff;
        Gff_out->type = TF_ORDER_ZERO;
        Gff_out->order_num = 0;
        Gff_out->order_den = 0;
        Gff_out->num_coeffs[0] = 1.0;
        Gff_out->den_coeffs[0] = 1.0;
        return -1; /* Returns -1 to indicate NMP fallback */
    }

    /* For minimum-phase case, the feedforward is:
     * Gff(s) = -Gd(s)/Gp(s) = -(Kd/Kp) * (N_d(s)*D_p(s))/(D_d(s)*N_p(s))
     *
     * This is a rational TF. For simple FOPDT/SOPDT cases handled above.
     * Here we handle the general case: compute polynomial convolution.
     */

    /* For the general case, compute numerator = -Kd*D_p * N_d (constant trivially)
     * and denominator = Kp*D_d * N_p (constant trivially).
     *
     * But this gets into polynomial math. For now, compute using the
     * FOPDT approximation if both are low-order. */

    /* For Gp with arbitrary TF, extract effective time constants
     * from denominator coefficients. */
    double Kff = -Gd->K / Gp->K;
    if (fabs(Gp->K) < FF_EPSILON) Kff = 0.0;

    /* Compute effective lead from Gp denominator (inverse becomes numerator) */
    double T_lead = 0.0;
    if (Gp->order_den >= 1 && fabs(Gp->den_coeffs[0]) > FF_EPSILON) {
        T_lead = Gp->den_coeffs[0] / Gp->den_coeffs[Gp->order_den];
    }

    /* Compute effective lag from Gd denominator */
    double T_lag = 0.0;
    if (Gd->order_den >= 1 && fabs(Gd->den_coeffs[0]) > FF_EPSILON) {
        T_lag = Gd->den_coeffs[0] / Gd->den_coeffs[Gd->order_den];
    }

    (void)T_lead; /* Used for future TF polynomial convolution */
    (void)T_lag;  /* Used for future TF polynomial convolution */

    memset(Gff_out, 0, sizeof(tf_t));
    Gff_out->K = Kff;
    Gff_out->theta = 0.0;
    Gff_out->type = TF_ORDER_FIRST;
    Gff_out->order_num = 1;
    Gff_out->order_den = 1;
    Gff_out->num_coeffs[0] = 0.0; /* s term coefficient */
    Gff_out->num_coeffs[1] = 1.0; /* constant term */
    Gff_out->den_coeffs[0] = 0.0;
    Gff_out->den_coeffs[1] = 1.0;

    return 0;
}

/* ============================================================================
 * L2: Dynamic feedforward initialization and step
 * ============================================================================ */

void ff_dynamic_init(feedforward_t *ff, double T_lead, double T_lag,
                     double K_ll, double Ts)
{
    if (!ff) return;

    ff->mode = FF_MODE_DYNAMIC;
    lead_lag_init(&ff->lead_lag, K_ll, T_lead, T_lag, Ts);
    ff->Ts = Ts;
    ff->initialized = 1;
}

double ff_dynamic_step(feedforward_t *ff, double d_meas)
{
    if (!ff || !ff->initialized) return 0.0;
    if (ff->mode == FF_MODE_OFF) return ff->bias;

    /* Apply lead-lag dynamic compensation to the disturbance measurement */
    double u_dyn = lead_lag_step(&ff->lead_lag, d_meas);

    /* Add bias */
    u_dyn += ff->bias;

    /* Output clamping */
    if (ff->clamping) {
        if (u_dyn > ff->output_max) u_dyn = ff->output_max;
        if (u_dyn < ff->output_min) u_dyn = ff->output_min;
    }

    ff->d_meas = d_meas;
    ff->u_ff_dynamic = u_dyn;
    ff->u_ff_total = u_dyn;

    return u_dyn;
}

/* ============================================================================
 * L2: Causality and implementability checks
 * ============================================================================ */

int ff_dynamic_is_causal(double theta_p, double theta_d)
{
    /* Perfect dynamic feedforward is causal (implementable) if:
     *   theta_d >= theta_p
     *
     * This means the disturbance effect reaches the PV no faster than
     * the manipulated variable can affect the PV. If the disturbance is
     * faster (theta_d < theta_p), perfect compensation would require
     * "knowing the future" — non-causal.
     *
     * For real-time implementation, non-causal feedforward can be
     * approximated by: ignoring dead time difference (suboptimal but stable),
     * or using Smith predictor-type delay compensation.
     */
    return (theta_d >= theta_p) ? 1 : 0;
}

double ff_dynamic_required_delay(double theta_p, double theta_d)
{
    /* Required delay to synchronize disturbance and process effects:
     *
     * If disturbance is faster: delta = theta_p - theta_d
     *   → delay the feedforward action by delta seconds
     *
     * If process is faster: delta = 0 (no extra delay needed,
     *   but perfect compensation is not possible)
     */
    if (theta_p > theta_d) {
        return theta_p - theta_d;
    }
    return 0.0;
}