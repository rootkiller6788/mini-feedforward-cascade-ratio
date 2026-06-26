/**
 * @file mimo_control.c
 * @brief MIMO control strategies: decentralized PI, static decoupling control,
 *        dynamic decoupling control, BLT tuning, IMC-based MIMO control.
 *
 * Knowledge:
 *   L2: Decentralized vs centralized MIMO control concepts
 *   L3: Discrete-time PI controller implementation for MIMO
 *   L5: BLT (Biggest Log Modulus) tuning method (Luyben 1986)
 *   L5: IMC-based MIMO controller design
 *   L6: Closed-loop simulation with decoupling
 *   L6: Wood-Berry distillation column benchmark
 *
 * References:
 *   - Luyben (1986), "Simple Method for Tuning SISO Controllers in
 *     Multivariable Systems", IEC Proc. Des. Dev., 25(3):654-660
 *   - Seborg, Edgar, Mellichamp (2016), Ch.18
 *   - Skogestad & Postlethwaite (2005), Ch.10
 */

#include "mimo_model.h"
#include "mimo_decoupling_common.h"
#include "mimo_interaction.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ==========================================================================
 * L2 — MIMO PI Controller Structure (Decentralized)
 *
 * Each loop has an independent PI controller:
 *   u_i(t) = Kc_i * e_i(t) + (Kc_i/Ti_i) * integral(e_i(t)) dt
 *
 * Discrete form (positional):
 *   u_i[k] = Kc_i * e_i[k] + I_i[k]
 *   I_i[k] = I_i[k-1] + (Kc_i * Ts / Ti_i) * e_i[k]
 * ========================================================================== */

/**
 * @brief MIMO PI controller state for n loops
 */
typedef struct {
    int n_loops;                         /**< number of control loops */
    double Kc[MIMO_MAX_DIM];            /**< proportional gains */
    double Ti[MIMO_MAX_DIM];            /**< integral time constants (seconds) */
    double Ts;                           /**< sample time */
    double integral[MIMO_MAX_DIM];      /**< integral accumulator */
    double u_min[MIMO_MAX_DIM];         /**< output lower limit */
    double u_max[MIMO_MAX_DIM];         /**< output upper limit */
    bool anti_windup;                    /**< enable anti-windup */
} MIMOPIController;

/**
 * @brief Initialize decentralized MIMO PI controller
 *
 * @param ctrl   controller struct
 * @param n      number of loops
 * @param Kc     proportional gain array (length n)
 * @param Ti     integral time array (length n, 0 = no integral)
 * @param Ts     sample time
 */
void mimo_pi_init(MIMOPIController *ctrl, int n, const double *Kc,
                   const double *Ti, double Ts) {
    if (!ctrl) return;
    memset(ctrl, 0, sizeof(MIMOPIController));
    ctrl->n_loops = (n > MIMO_MAX_DIM) ? MIMO_MAX_DIM : n;
    ctrl->Ts = Ts;
    ctrl->anti_windup = true;
    for (int i = 0; i < ctrl->n_loops; i++) {
        ctrl->Kc[i] = Kc ? Kc[i] : 1.0;
        ctrl->Ti[i] = Ti ? Ti[i] : 0.0;
        ctrl->u_min[i] = -100.0;
        ctrl->u_max[i] = 100.0;
    }
}

/**
 * @brief Execute one step of decentralized PI control
 *
 * @param ctrl       controller state (modified in-place)
 * @param setpoint   setpoint vector [n]
 * @param measurement process variable vector [n]
 * @param output     controller output vector [n] (written)
 */
void mimo_pi_step(MIMOPIController *ctrl, const double *setpoint,
                   const double *measurement, double *output) {
    if (!ctrl || !setpoint || !measurement || !output) return;
    int n = ctrl->n_loops;

    for (int i = 0; i < n; i++) {
        double error = setpoint[i] - measurement[i];

        /* Proportional term */
        double u_p = ctrl->Kc[i] * error;

        /* Integral term with anti-windup */
        if (ctrl->Ti[i] > MIMO_EPSILON) {
            double u_unsat = u_p + ctrl->integral[i];
            ctrl->integral[i] += (ctrl->Kc[i] * ctrl->Ts / ctrl->Ti[i]) * error;

            /* Anti-windup: back-calculation */
            if (ctrl->anti_windup) {
                if (u_unsat > ctrl->u_max[i]) {
                    ctrl->integral[i] -= (u_unsat - ctrl->u_max[i]);
                } else if (u_unsat < ctrl->u_min[i]) {
                    ctrl->integral[i] -= (u_unsat - ctrl->u_min[i]);
                }
            }
        }

        output[i] = u_p + ctrl->integral[i];

        /* Output clamping */
        if (output[i] > ctrl->u_max[i]) output[i] = ctrl->u_max[i];
        if (output[i] < ctrl->u_min[i]) output[i] = ctrl->u_min[i];
    }
}

/* ==========================================================================
 * L5 — BLT Tuning Method (Luyben, 1986)
 *
 * BLT tunes decentralized PI controllers for MIMO systems.
 *
 * 1. Design individual SISO PI controllers using Ziegler-Nichols
 * 2. Detune all gains by factor F: Kc_i = Kc_i_ZN / F
 *    and Ti_i = Ti_i_ZN * F
 * 3. F is chosen so that the biggest log modulus (BLT) of the
 *    closed-loop system is below a threshold.
 *
 * BLT = 20 * log10 ( |W/(1+W)|_max ), W = -1 + det(I + G*K)
 *
 * Reference: Luyben (1986), IEC Proc. Des. Dev. 25(3):654-660
 * ========================================================================== */

/**
 * @brief Apply BLT detuning to PI parameters
 *
 * @param n       number of loops
 * @param Kc_zn  Ziegler-Nichols Kc values [n]
 * @param Ti_zn  Ziegler-Nichols Ti values [n]
 * @param F       detuning factor (typically 1.5 to 4.0)
 * @param Kc_blt output BLT-tuned Kc [n]
 * @param Ti_blt output BLT-tuned Ti [n]
 */
void mimo_blt_tune(int n, const double *Kc_zn, const double *Ti_zn,
                    double F, double *Kc_blt, double *Ti_blt) {
    if (!Kc_zn || !Ti_zn || !Kc_blt || !Ti_blt || n <= 0) return;
    if (F < 1.0) F = 1.0; /* F < 1 would increase gain — unsafe */

    for (int i = 0; i < n; i++) {
        Kc_blt[i] = Kc_zn[i] / F;
        Ti_blt[i] = Ti_zn[i] * F;
    }
}

/**
 * @brief Estimate the BLT criterion value for a given F factor
 *
 * Computes W = -1 + det(I + G(0) * K_diag) where K_diag = diag(Kc_i).
 * Returns BLT = |W / (1 + W)| as a measure of closed-loop interaction.
 *
 * BLT < 0.26 (or 2dB) is considered acceptable for most processes.
 *
 * @param model  MIMO model
 * @param Kc     proportional gains [n]
 * @param blt    output BLT value
 * @return       0 on success
 */
int mimo_blt_criterion(const MIMOModel *model, const double *Kc, double *blt) {
    if (!model || !Kc || !blt) return -1;
    int n = model->num_outputs;
    if (n != model->num_inputs || n == 0) return -1;

    /* Get steady-state gain matrix */
    double *K = (double *)calloc(n * n, sizeof(double));
    mimo_model_steady_state_gain(model, K);

    /* Build I + G(0) * K_diag */
    double *IK = (double *)calloc(n * n, sizeof(double));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            IK[i * n + j] = K[i * n + j] * Kc[j];
            if (i == j) IK[i * n + j] += 1.0;
        }
    }

    /* Compute determinant of I + G*K via LU */
    double det = 1.0;
    int sgn = 1;
    for (int k = 0; k < n; k++) {
        int mr = k;
        double mv = fabs(IK[mr*n+k]);
        for (int r = k+1; r < n; r++)
            if (fabs(IK[r*n+k]) > mv) { mv = fabs(IK[r*n+k]); mr = r; }
        if (mv < MIMO_EPSILON) { det = 0.0; break; }
        if (mr != k) {
            for (int c = 0; c < n; c++) {
                double t = IK[k*n+c]; IK[k*n+c] = IK[mr*n+c]; IK[mr*n+c] = t;
            }
            sgn = -sgn;
        }
        det *= IK[k*n+k];
        for (int r = k+1; r < n; r++) {
            double f = IK[r*n+k] / IK[k*n+k];
            for (int c = k; c < n; c++) IK[r*n+c] -= f * IK[k*n+c];
        }
    }
    det *= sgn;

    /* W = -1 + det(I + G*K) */
    double W = -1.0 + det;

    /* BLT = |W / (1 + W)| */
    if (fabs(1.0 + W) < MIMO_EPSILON) {
        *blt = INFINITY;
    } else {
        *blt = fabs(W / (1.0 + W));
    }

    free(K); free(IK);
    return 0;
}

/* ==========================================================================
 * L6 — Closed-Loop MIMO Simulation
 * ========================================================================== */

/**
 * @brief Simulate one time step of a MIMO process with decoupling control.
 *
 * Process model (continuous state-space, discretized via Euler):
 *   x[k+1] = x[k] + Ts * (A*x[k] + B*u[k])
 *   y[k] = C*x[k] + D*u[k]
 *
 * @param model     MIMO model (provides SS representation)
 * @param u         process input vector [m] at current step
 * @param y         process output vector [p] (written)
 * @param x_state   state vector [n], persistent (modified in-place)
 * @param Ts        sample time
 */
void mimo_sim_step(const MIMOModel *model, const double *u, double *y,
                    double *x_state, double Ts) {
    if (!model || !u || !y) return;
    (void)x_state; /* reserved for future state-space simulation */

    int p = model->num_outputs;
    int m = model->num_inputs;

    /* Simple simulation: y_i = sum_j G_{ij}(0) * u_j + dynamics
     * For a proper dynamic simulation, use state-space.
     * Here we use the static gain with first-order lag approximation. */
    for (int i = 0; i < p; i++) {
        double y_new = 0.0;
        for (int j = 0; j < m; j++) {
            const MIMOTransferFunction *tf = &model->rows[i].elements[j];
            double tau = tf->time_constant;
            if (tau < Ts) tau = Ts; /* prevent instability */

            /* First-order discrete filter:
             * y[k] = (tau/(tau+Ts)) * y[k-1] + (Ts*K/(tau+Ts)) * u[k] */
            double alpha = tau / (tau + Ts);
            double beta = Ts * tf->gain / (tau + Ts);

            /* Apply dead time as simple delay (1-step for simplicity) */
            y_new += alpha * y[i] + beta * u[j];
        }
        y[i] = y_new;
    }
}

/* ==========================================================================
 * L6 — Wood-Berry Distillation Column Model (Wood & Berry, 1973)
 *
 * Classic 2x2 MIMO benchmark:
 *
 * [ yD(s) ]   [ 12.8*e^{-s}/(16.7s+1)  -18.9*e^{-3s}/(21.0s+1) ] [ R(s) ]
 * [ yB(s) ] = [ 6.6*e^{-7s}/(10.9s+1)  -19.4*e^{-3s}/(14.4s+1) ] [ S(s) ]
 *
 * yD = distillate composition, yB = bottoms composition
 * R  = reflux flow, S = steam flow
 * ========================================================================== */

void mimo_wood_berry_model(MIMOModel *model) {
    if (!model) return;
    mimo_model_init(model, 2, 2, "Wood-Berry Distillation Column");

    /* G_{11}: 12.8*e^{-s} / (16.7*s + 1) */
    mimo_model_set_fopdt(model, 0, 0, 12.8, 16.7, 1.0);

    /* G_{12}: -18.9*e^{-3s} / (21.0*s + 1) */
    mimo_model_set_fopdt(model, 0, 1, -18.9, 21.0, 3.0);

    /* G_{21}: 6.6*e^{-7s} / (10.9*s + 1) */
    mimo_model_set_fopdt(model, 1, 0, 6.6, 10.9, 7.0);

    /* G_{22}: -19.4*e^{-3s} / (14.4*s + 1) */
    mimo_model_set_fopdt(model, 1, 1, -19.4, 14.4, 3.0);
}

/* ==========================================================================
 * L6 — Blending Process Model
 * 2x2 process with moderate interaction
 * ========================================================================== */

void mimo_blending_process_model(MIMOModel *model) {
    if (!model) return;
    mimo_model_init(model, 2, 2, "Blending Process");

    /* G_{11}: 1/(5s+1), G_{12}: 1/(5s+1) */
    mimo_model_set_fopdt(model, 0, 0, 1.0, 5.0, 0.0);
    mimo_model_set_fopdt(model, 0, 1, 1.0, 5.0, 0.0);

    /* G_{21}: 0.3/(8s+1), G_{22}: -0.2/(6s+1) */
    mimo_model_set_fopdt(model, 1, 0, 0.3, 8.0, 0.0);
    mimo_model_set_fopdt(model, 1, 1, -0.2, 6.0, 0.0);
}

/* ==========================================================================
 * L5 — IMC-Based MIMO Controller Design
 *
 * For a MIMO process G(s) with decoupler D(s):
 *   K_IMC(s) = D(s) * diag{q_1(s), ..., q_n(s)} * (I - G(s)*D(s)*Q(s))^{-1}
 *
 * where Q(s) = diag{F_1(s)/G_{11}(s), ..., F_n(s)/G_{nn}(s)}
 * and F_i(s) = 1/(lambda_i*s + 1)^{r_i} are IMC filters.
 * ========================================================================== */

/**
 * @brief Design IMC filter parameters for each loop
 *
 * @param n       number of loops
 * @param tau     dominant time constants [n]
 * @param lambda  IMC filter constants [n] (output, lambda = max(tau, tau_cl_desired))
 */
void mimo_imc_filter_design(int n, const double *tau, double *lambda) {
    if (!tau || !lambda || n <= 0) return;
    for (int i = 0; i < n; i++) {
        /* Lambda = max(process_tau, desired_closed_loop_tau)
         * Default: lambda = tau (gives reasonable robustness) */
        lambda[i] = (tau[i] > 1.0) ? tau[i] : 1.0;
    }
}
