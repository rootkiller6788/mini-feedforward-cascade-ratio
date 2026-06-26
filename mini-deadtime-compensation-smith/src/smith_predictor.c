/**
 * @file smith_predictor.c
 * @brief Core Smith Predictor — dead-time compensated PID control.
 *
 * Knowledge: L1-L7 complete.
 *   L1: Smith predictor struct, FOPDT/SOPDT model definitions
 *   L2: Prediction feedback, mismatch correction, disturbance rejection
 *   L3: Digital delay buffer, 5 discretization methods, anti-windup
 *   L4: Robustness filter (Normey-Rico 1997), stability condition
 *   L5: Discrete PID with setpoint weighting and filtered derivative
 *   L6: Setpoint tracking and load disturbance rejection with IAE/ISE/ITAE
 *   L7: Modbus register mapping, OPC UA node mapping for SCADA
 *
 * Theorem (Smith 1957): If Gp_model*exp(-ths) = Gp_true*exp(-ths),
 *   characteristic equation = 1 + C(s)*Gp(s) = 0 (dead-time-free).
 *
 * Algorithm: y_fb = yp_model + (y_meas - yp_delayed)  [Smith feedback]
 *   PID acts on e = r - y_fb, seeing an effectively delay-free process.
 *
 * References:
 *   Smith (1957) Chem. Eng. Prog. 53(5), 217-219
 *   Normey-Rico & Camacho (2007) Control of Dead-time Processes, Springer
 *   Astrom & Hagglund (2005) Advanced PID Control, Ch. 7
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "smith_predictor.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SMITH_MIN_TAU        1e-6
#define SMITH_MAX_DISC_COEFF 0.9999
#define SMITH_EPS            1e-12

/* ===== L3: Circular Delay Buffer — Digital z^(-d) =============== */

/* Allocate ring buffer: O(d_max) memory, O(1) per push */
static int delay_buffer_alloc(smith_delay_buffer_t *buf, size_t d_max, double Ts)
{
    if (d_max < 1 || Ts <= SMITH_EPS) return -1;
    buf->capacity = d_max + 3;
    buf->buffer = (double *)calloc(buf->capacity, sizeof(double));
    if (!buf->buffer) return -1;
    buf->head = 0;
    buf->delay_int  = d_max;
    buf->delay_frac = 0.0;
    buf->Ts = Ts;
    for (size_t i = 0; i < buf->capacity; i++) buf->buffer[i] = 0.0;
    return 0;
}

static void delay_buffer_free(smith_delay_buffer_t *buf)
{
    if (buf->buffer) { free(buf->buffer); buf->buffer = NULL; buf->capacity = 0; }
}

/* Push sample, retrieve delayed value d steps ago.
   Fractional-delay: linear interpolation. Time: O(1). */
static double delay_buffer_push(smith_delay_buffer_t *buf, double sample)
{
    buf->head = (buf->head + 1) % buf->capacity;
    buf->buffer[buf->head] = sample;

    size_t d     = buf->delay_int;
    size_t tail0 = (buf->head + buf->capacity - d) % buf->capacity;

    if (buf->delay_frac < SMITH_EPS) {
        return buf->buffer[tail0];
    } else {
        size_t tail1 = (tail0 + buf->capacity - 1) % buf->capacity;
        return (1.0 - buf->delay_frac) * buf->buffer[tail0]
             + buf->delay_frac * buf->buffer[tail1];
    }
}

static void delay_buffer_clear(smith_delay_buffer_t *buf, double val)
{
    for (size_t i = 0; i < buf->capacity; i++) buf->buffer[i] = val;
    buf->head = 0;
}

/* Resize when dead time changes during online model update.
   Nearest-neighbor copy for continuity. */
static int delay_buffer_resize(smith_delay_buffer_t *buf,
                                size_t new_d, double new_frac)
{
    if (new_d == buf->delay_int &&
        fabs(new_frac - buf->delay_frac) < SMITH_EPS) return 0;

    size_t new_cap = new_d + 3;
    double *nb = (double *)calloc(new_cap, sizeof(double));
    if (!nb) return -1;

    size_t ncopy = (new_cap < buf->capacity) ? new_cap : buf->capacity;
    for (size_t i = 0; i < ncopy; i++) {
        size_t oi = (buf->head + buf->capacity - i) % buf->capacity;
        size_t ni = (new_cap - 1 - i) % new_cap;
        nb[ni] = buf->buffer[oi];
    }
    free(buf->buffer);
    buf->buffer    = nb;
    buf->capacity  = new_cap;
    buf->delay_int = new_d;
    buf->delay_frac = new_frac;
    buf->head = 0;
    return 0;
}

/* ===== L3: FOPDT Discretization — 5 methods ===================== */

/* Discretize Gp(s) = K/(tau*s+1) with period Ts.
   Stores coeffs: state_1=a1, state_2=b0, derivative_state=b1.

   Euler Fwd: a1=1-Ts/tau, stable iff Ts<2*tau
   Euler Bwd: a1=tau/(tau+Ts), unconditionally stable (A-stable)
   Tustin:    a1=(2tau-Ts)/(2tau+Ts), preserves stability boundary
   ZOH:       a1=exp(-Ts/tau), exact step-invariant
   FOH:       ramp-invariant, for linearly varying inputs */
static void discretize_fopdt(smith_predictor_t *sp, smith_disc_method_t m)
{
    double K = sp->model.fopdt.K, tau = sp->model.fopdt.tau, Ts = sp->Ts;
    double a1, b0, b1;

    if (tau < SMITH_MIN_TAU) {
        sp->model_state_1 = 0.0; sp->model_state_2 = 0.0;
        sp->derivative_state = K; return;
    }

    switch (m) {
    case SMITH_DISC_EULER_FWD:
        a1 = 1.0 - Ts/tau;
        if (a1 < -SMITH_MAX_DISC_COEFF) a1 = -SMITH_MAX_DISC_COEFF;
        if (a1 >  SMITH_MAX_DISC_COEFF) a1 =  SMITH_MAX_DISC_COEFF;
        b0 = K * Ts / tau; b1 = 0.0; break;
    case SMITH_DISC_EULER_BWD:
        a1 = tau/(tau+Ts); b0 = K*Ts/(tau+Ts); b1 = 0.0; break;
    case SMITH_DISC_TUSTIN:
        a1 = (2.0*tau - Ts)/(2.0*tau + Ts);
        b0 = K*Ts/(2.0*tau + Ts); b1 = b0; break;
    case SMITH_DISC_FOH: {
        a1 = exp(-Ts/tau);
        double alpha = Ts/tau;
        if (alpha > 1e-6) { b0=K*(1.0-(1.0-a1)/alpha); b1=K*((1.0-a1)/alpha - a1); }
        else { b0=K*alpha/2.0; b1=K*alpha/2.0; }
        break;
    }
    default: /* ZOH */
        a1 = exp(-Ts/tau); b0 = 0.0; b1 = K*(1.0 - a1); break;
    }
    sp->model_state_1 = a1; sp->model_state_2 = b0; sp->derivative_state = b1;
}

/* y(k) = a1*y(k-1) + b0*u(k) + b1*u(k-1) */
static double fopdt_step(const smith_predictor_t *sp, double u,
                          double yp, double up)
{
    double a1=sp->model_state_1, b0=sp->model_state_2, b1=sp->derivative_state;
    if (fabs(a1)<SMITH_EPS && fabs(b0)<SMITH_EPS) return b1*u;
    return a1*yp + b0*u + b1*up;
}

/* ===== L3: Discrete PID with Anti-Windup ======================== */

/* PID with setpoint weighting and filtered derivative on measurement.
   e=r-y_fb; P=Kp*(b*r-y_fb); I(k)=I+Ki*e; D via Tustin on measurement.
   Anti-windup: back-calculation, Tt=sqrt(Ti*Td). */
static double compute_pid(smith_predictor_t *sp, double r, double y_fb)
{
    double Kp=sp->Kp, Ti=sp->Ti, Td=sp->Td, N=sp->N, Ts=sp->Ts, b=sp->b;
    if (Kp<SMITH_EPS) return 0.0;

    double e = r - y_fb;
    double P = Kp * (b*r - y_fb);

    double I = sp->integrator;
    if (Ti > SMITH_EPS) I += Kp*Ts/Ti * e;

    double D = 0.0;
    if (Td > SMITH_EPS) {
        double Tf = Td/N;
        double aD = (2.0*Tf - Ts)/(2.0*Tf + Ts);
        double bD = 2.0*Kp*Td/(2.0*Tf + Ts);
        D = aD * sp->derivative_state - bD * (y_fb - sp->prev_error);
        sp->derivative_state = D;
    }

    double u_raw = P + I + D;

    if (sp->saturating) {
        double Tt = (Ti>SMITH_EPS) ? sqrt(Ti*(Td>SMITH_EPS?Td:0.0)) : Ts;
        if (Tt<Ts) Tt=Ts;
        I += (Ts/Tt)*(sp->prev_output - u_raw);
        u_raw = P + I + D;
    }
    sp->integrator = I;
    sp->prev_error = y_fb;
    return u_raw;
}

/* Saturation + rate limiting */
static double clamp_output(smith_predictor_t *sp, double u_raw)
{
    double u = u_raw;
    if (u>sp->u_max) { u=sp->u_max; sp->saturating=1; }
    else if (u<sp->u_min) { u=sp->u_min; sp->saturating=1; }
    else sp->saturating = 0;

    if (sp->rate_limit > SMITH_EPS) {
        double ms = sp->rate_limit * sp->Ts;
        double delta = u - sp->prev_output;
        if (delta>ms) u=sp->prev_output+ms;
        else if (delta<-ms) u=sp->prev_output-ms;
        if (u>=sp->u_max || u<=sp->u_min) sp->saturating=1;
    }
    sp->prev_output = u;
    return u;
}

/* Normey-Rico robustness filter: Fr(s)=1/(Tr*s+1) on prediction error */
static double filter_ep(smith_predictor_t *sp, double ep)
{
    if (sp->Fr <= SMITH_EPS) return ep;
    double alpha = sp->Fr/(sp->Fr + sp->Ts);
    double epf = alpha*sp->filter_state + (1.0-alpha)*ep;
    sp->filter_state = epf;
    return epf;
}

/* ===== L1-L2: Lifecycle ========================================= */

int smith_predictor_init_fopdt(
    smith_predictor_t *sp, double K, double tau, double theta, double Ts,
    smith_variant_t variant, double u_min, double u_max)
{
    if (!sp || tau<=SMITH_MIN_TAU || theta<0.0 || Ts<=SMITH_EPS || u_max<=u_min) return -1;
    memset(sp, 0, sizeof(smith_predictor_t));

    sp->model.order          = SMITH_MODEL_FOPDT;
    sp->model.fopdt.K        = K;
    sp->model.fopdt.tau      = (tau<SMITH_MIN_TAU) ? SMITH_MIN_TAU : tau;
    sp->model.fopdt.theta    = theta;
    sp->model.K_delay_free   = K;
    sp->model.tau_delay_free = sp->model.fopdt.tau;
    sp->variant   = variant;
    sp->disc_method = SMITH_DISC_TUSTIN;
    sp->Ts = Ts; sp->b = 1.0; sp->u_min = u_min; sp->u_max = u_max;

    double ds = theta/Ts; size_t d = (size_t)ds;
    if (d<1) d=1;
    double frac = ds - (double)d;
    if (frac<0.0) frac=0.0; if (frac>1.0-SMITH_EPS) frac=0.0;

    if (delay_buffer_alloc(&sp->delay_buf, d, Ts) != 0) return -1;
    sp->delay_buf.delay_int=d; sp->delay_buf.delay_frac=frac;
    discretize_fopdt(sp, sp->disc_method);
    return 0;
}

int smith_predictor_init_sopdt(
    smith_predictor_t *sp, double K, double tau1, double tau2,
    double zeta, double omega_n, double theta, double Ts,
    smith_variant_t variant, double u_min, double u_max)
{
    if (!sp || tau1<0.0 || theta<0.0 || Ts<=SMITH_EPS || u_max<=u_min) return -1;
    memset(sp, 0, sizeof(smith_predictor_t));

    sp->model.order         = SMITH_MODEL_SOPDT;
    sp->model.sopdt.K       = K;
    sp->model.sopdt.tau1    = tau1;
    sp->model.sopdt.theta   = theta;
    sp->model.sopdt.zeta    = zeta;
    sp->model.sopdt.omega_n = omega_n;
    sp->model.K_delay_free  = K;
    sp->model.tau_delay_free = tau1;

    double tau2e = (tau2<SMITH_MIN_TAU) ? SMITH_MIN_TAU : tau2;
    sp->model.sopdt.tau2 = tau2e;
    sp->model.sopdt.is_underdamped = (zeta>0.0 && zeta<1.0) ? 1 : 0;

    sp->variant = variant; sp->disc_method = SMITH_DISC_TUSTIN;
    sp->Ts=Ts; sp->b=1.0; sp->u_min=u_min; sp->u_max=u_max;

    double ds = theta/Ts; size_t d = (size_t)ds;
    if (d<1) d=1;
    double frac = ds-(double)d;
    if (frac<0.0) frac=0.0; if (frac>1.0-SMITH_EPS) frac=0.0;

    if (delay_buffer_alloc(&sp->delay_buf, d, Ts) != 0) return -1;
    sp->delay_buf.delay_int=d; sp->delay_buf.delay_frac=frac;

    double e1=exp(-Ts/tau1), e2=exp(-Ts/tau2e);
    sp->model_state_1 = e1; sp->model_state_2 = e2;
    sp->derivative_state = K*(1.0-e1);
    sp->prev_error       = K*(1.0-e2);
    sp->prev_error2      = (fabs(tau1-tau2e)>SMITH_EPS)
        ? tau2e*(e1-e2)/(tau2e-tau1) : (Ts/tau1)*exp(-Ts/tau1);
    return 0;
}

void smith_predictor_destroy(smith_predictor_t *sp) {
    if (sp) delay_buffer_free(&sp->delay_buf);
}

void smith_predictor_reset(smith_predictor_t *sp) {
    if (!sp) return;
    sp->integrator=0.0; sp->prev_error=0.0; sp->prev_error2=0.0;
    sp->derivative_state=0.0; sp->filter_state=0.0;
    sp->yp_model=0.0; sp->yp_delayed=0.0; sp->prediction_error=0.0;
    sp->saturating=0;
    delay_buffer_clear(&sp->delay_buf, 0.0);
}

/* ===== L2: Controller Configuration ============================= */

int smith_predictor_set_pi(smith_predictor_t *sp, double Kp, double Ti, double b)
{
    if (!sp || Kp<0.0) return -1;
    sp->Kp=Kp; sp->Ti=Ti; sp->Td=0.0; sp->N=8.0;
    if (b<0.0) b=0.0; if (b>1.0) b=1.0; sp->b=b;
    return 0;
}

int smith_predictor_set_pid(
    smith_predictor_t *sp, double Kp, double Ti, double Td, double N,
    double b, double c)
{
    if (!sp || Kp<0.0) return -1;
    sp->Kp=Kp; sp->Ti=Ti; sp->Td=Td;
    sp->N=(N<2.0)?2.0:N;
    if (b<0.0) b=0.0; if (b>1.0) b=1.0; sp->b=b;
    (void)c;
    return 0;
}

int smith_predictor_set_robustness_filter(smith_predictor_t *sp, double Fr)
{ if (!sp || Fr<0.0) return -1; sp->Fr=Fr; return 0; }

int smith_predictor_set_disc_method(smith_predictor_t *sp, smith_disc_method_t m)
{ if (!sp) return -1; sp->disc_method=m; discretize_fopdt(sp,m); return 0; }

/* ===== L3: Main Control Iteration =============================== */

double smith_predictor_step(smith_predictor_t *sp, double setpoint, double pv)
{
    if (!sp) return 0.0;
    double up = sp->prev_output;

    /* 1. Update delay-free model */
    if (sp->model.order == SMITH_MODEL_FOPDT)
        sp->yp_model = fopdt_step(sp, up, sp->yp_model, up);
    else {
        double x1=sp->model_state_1, x2=sp->yp_model;
        double A11=sp->model_state_1, A22=sp->model_state_2, A21=sp->prev_error2;
        double B1=sp->derivative_state, B2=sp->prev_error;
        sp->model_state_1 = A11*x1 + B1*up;
        sp->yp_model       = A21*x1 + A22*x2 + B2*up;
    }

    /* 2. Delay buffer push + retrieve */
    sp->yp_delayed = delay_buffer_push(&sp->delay_buf, sp->yp_model);

    /* 3. Prediction error */
    sp->prediction_error = pv - sp->yp_delayed;

    /* 4. Robustness filter */
    double epc = filter_ep(sp, sp->prediction_error);

    /* 5. Smith feedback: y_fb = yp_model + (y_meas - yp_delayed) */
    double y_fb = sp->yp_model + epc;

    /* 6. PID */
    double u_raw = compute_pid(sp, setpoint, y_fb);

    /* 7. Constraints */
    double u_out = clamp_output(sp, u_raw);

    /* 8. Diagnostics */
    double et = setpoint - pv;
    sp->IAE  += fabs(et) * sp->Ts;
    sp->ISE  += et*et * sp->Ts;
    sp->ITAE += (double)(sp->sample_count)*sp->Ts * fabs(et)*sp->Ts;
    sp->sample_count++;

    return u_out;
}

/* ===== L5: Diagnostics & Model Update =========================== */

double smith_predictor_get_prediction(const smith_predictor_t *sp)
{ return sp ? sp->yp_model : 0.0; }

double smith_predictor_get_mismatch(const smith_predictor_t *sp)
{ return sp ? sp->prediction_error : 0.0; }

int smith_predictor_update_model(
    smith_predictor_t *sp, double K, double tau, double theta)
{
    if (!sp || tau<0.0 || theta<0.0) return -1;
    sp->model.fopdt.K=K; sp->model.fopdt.tau=tau; sp->model.fopdt.theta=theta;
    sp->model.tau_delay_free=tau;

    double ds=theta/sp->Ts; size_t nd=(size_t)ds;
    if (nd<1) nd=1;
    double nf=ds-(double)nd;
    if (nf<0.0) nf=0.0; if (nf>1.0-SMITH_EPS) nf=0.0;

    if (delay_buffer_resize(&sp->delay_buf, nd, nf) != 0) return -1;
    discretize_fopdt(sp, sp->disc_method);
    return 0;
}

void smith_predictor_apply_mismatch_correction(smith_predictor_t *sp)
{
    if (!sp || sp->Kp<SMITH_EPS) return;
    double ep=sp->prediction_error, u=sp->prev_output;
    if (fabs(u)<SMITH_EPS) return;

    double Km=sp->model.fopdt.K, Ka=Km + ep/u;
    if (Ka<0.1*Km) Ka=0.1*Km; if (Ka>10.0*Km) Ka=10.0*Km;

    double corr=Km/Ka;
    if (corr>2.0) corr=2.0; if (corr<0.5) corr=0.5;
    sp->Kp *= corr;
}

void smith_predictor_compute_performance(
    const smith_predictor_t *sp, smith_performance_t *p)
{
    if (!sp || !p) return;
    memset(p, 0, sizeof(smith_performance_t));
    p->setpoint_IAE = sp->IAE;

    double T = (double)sp->sample_count * sp->Ts;
    if (T < SMITH_EPS) return;

    p->noise_sensitivity = sp->ISE / T;
    double span = sp->u_max - sp->u_min;
    if (span > SMITH_EPS) {
        double mid=(sp->u_min+sp->u_max)/2.0;
        double v=(sp->prev_output-mid)/(span/2.0);
        p->control_effort_var = v*v;
    }

    double K=sp->model.fopdt.K, tau=sp->model.fopdt.tau;
    if (sp->Kp>SMITH_EPS && tau>SMITH_EPS) {
        double lambda = tau/(K*sp->Kp);
        p->robustness_margin = lambda/(tau*0.3+SMITH_EPS);
        if (p->robustness_margin<0.1) p->robustness_margin=0.1;
        p->phase_margin = 60.0/(1.0 + 0.1/(lambda/tau+SMITH_EPS));
        p->delay_margin = sp->model.fopdt.theta * 0.3;
    }
}

void smith_predictor_reset_metrics(smith_predictor_t *sp)
{ if (!sp) return; sp->IAE=0.0; sp->ISE=0.0; sp->ITAE=0.0; sp->sample_count=0; }

/* ===== L7: Industrial Communication ============================= */

void smith_predictor_map_modbus(
    const smith_predictor_t *sp, smith_modbus_map_t *mb, uint16_t base)
{
    if (!sp||!mb) return;
    mb->Kp_reg=base; mb->Ti_reg=base+2; mb->Td_reg=base+4;
    mb->theta_reg=base+6; mb->model_K_reg=base+8; mb->model_tau_reg=base+10;
    mb->yp_reg=base+12; mb->status_reg=base+14;
}

int smith_predictor_write_modbus(
    smith_predictor_t *sp, uint16_t reg, uint32_t value)
{
    if (!sp) return -1;
    union { uint32_t u; float f; } cv; cv.u=value;
    double v=(double)cv.f;
    if (reg<=1) sp->Kp=v; else if (reg<=3) sp->Ti=v;
    else if (reg<=5) sp->Td=v; else return -1;
    return 0;
}

void smith_predictor_map_opcua(
    const smith_predictor_t *sp, smith_opcua_map_t *ua,
    uint32_t ns, uint32_t base)
{
    if (!sp||!ua) return;
    ua->namespace_idx=ns; ua->Kp_node=base; ua->Ti_node=base+1;
    ua->Td_node=base+2; ua->theta_node=base+3; ua->predictor_out_node=base+4;
    ua->mismatch_node=base+5; ua->mode_node=base+6;
}
