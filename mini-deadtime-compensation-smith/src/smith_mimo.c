/**
 * @file smith_mimo.c
 * @brief MIMO (Multi-Input Multi-Output) Smith predictor extension.
 *
 * Extends the Smith predictor to multi-variable processes with
 * multiple interacting dead-time channels.
 *
 * Levels: L5 (Algorithms), L8 (Advanced Topics — MIMO)
 *
 * Key concepts:
 *   - Decentralized Smith predictor (one per loop)
 *   - Decoupled Smith predictor (with static decoupler)
 *   - Centralized MIMO Smith predictor
 *   - Multi-delay Smith predictor (different delays per channel)
 *
 * The MIMO Smith predictor structure:
 *   Y(s) = G(s) * U(s)  where G(s) = [G_ij(s) * e^(-theta_ij*s)]
 *   Gp(s) = [Gp_ij(s)]  (delay-free transfer matrix)
 *
 * Control law:
 *   U(s) = C(s) * [R(s) - Yp(s) - (Y(s) - Ym(s))]
 *   Yp(s) = Gp(s) * U(s)
 *   Ym(s) = diag(e^(-theta_i*s)) * Gp(s) * U(s)
 *
 * References:
 *   Ogunnaike, B.A. & Ray, W.H. (1979) "Multivariable controller design
 *       for linear systems having multiple time delays", AIChE J., 25, 1043-1057
 *   Jerome, N.F. & Ray, W.H. (1986) "High-performance multivariable control
 *       strategies for systems having time delays", AIChE J., 32, 914-931
 *   Wang, Q.G. et al. (1999) "PID tuning for MIMO processes",
 *       Automatica, 35(8), 1471-1478
 *   Skogestad & Postlethwaite (2005) "Multivariable Feedback Control"
 *       Chapter 10: "Control Structure Design"
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

/*===========================================================================
 * L5: MIMO Process Model Data Structures
 *===========================================================================*/

/**
 * @brief 2x2 process transfer matrix with individual delays.
 *
 * G(s) = [ G11(s)*e^(-θ11*s)   G12(s)*e^(-θ12*s) ]
 *        [ G21(s)*e^(-θ21*s)   G22(s)*e^(-θ22*s) ]
 *
 * Each Gij is a FOPDT model: Gij(s) = K_ij / (tau_ij * s + 1)
 */
typedef struct {
    double K[4];      /**< Gain matrix [K11, K12, K21, K22] */
    double tau[4];    /**< Time constants [τ11, τ12, τ21, τ22] */
    double theta[4];  /**< Dead times [θ11, θ12, θ21, θ22] */
} smith_mimo_2x2_model_t;

/**
 * @brief 2x2 MIMO Smith predictor state.
 */
typedef struct {
    smith_mimo_2x2_model_t model;   /**< Process model */
    double Kp[2];                   /**< PI gains for loops 1 and 2 */
    double Ti[2];                   /**< Integral times */
    double integrator[2];           /**< PI integrator states */
    double prev_error[2];           /**< Previous errors */
    double prev_output[2];          /**< Previous outputs */
    double model_state[4];          /**< Delay-free model states [11,12,21,22] */
    /* Delay buffers — one per channel */
    double *delay_buf[4];
    size_t delay_len[4];
    size_t delay_head[4];
    size_t delay_cap[4];
    double Ts;                      /**< Sampling period */
    double u_min[2], u_max[2];      /**< Actuator constraints per loop */
} smith_mimo_2x2_t;

/*===========================================================================
 * L5: Static Decoupler Design
 *
 * The static decoupler D is the inverse of the DC gain matrix:
 *   D = Gp(0)^{-1} = [K11 K12; K21 K22]^{-1}
 *
 * If K11*K22 - K12*K21 ≠ 0 (non-singular):
 *   D = 1/det * [ K22, -K12; -K21, K11 ]
 *
 * The decoupler makes each loop appear independent at steady state.
 *===========================================================================*/

/**
 * @brief Compute static decoupler for 2x2 MIMO process.
 *
 * @param K     Gain matrix [K11, K12, K21, K22]
 * @param D_out Decoupler matrix [D11, D12, D21, D22]
 * @return      0 on success, -1 if singular
 */
int smith_mimo_static_decoupler(const double K[4], double D_out[4])
{
    double det = K[0] * K[3] - K[1] * K[2];  /* K11*K22 - K12*K21 */

    if (fabs(det) < 1e-12) return -1;  /* Singular gain matrix */

    D_out[0] =  K[3] / det;  /* D11 =  K22/det */
    D_out[1] = -K[1] / det;  /* D12 = -K12/det */
    D_out[2] = -K[2] / det;  /* D21 = -K21/det */
    D_out[3] =  K[0] / det;  /* D22 =  K11/det */

    return 0;
}

/**
 * @brief Apply static decoupler to controller outputs.
 *
 * u_physical = D * u_controller
 *
 * @param D           Decoupler matrix
 * @param u_control   Controller outputs [u1, u2]
 * @param u_physical  Physical actuator signals [u1_phys, u2_phys] (output)
 */
void smith_mimo_apply_decoupler(const double D[4],
                                 const double u_control[2],
                                 double u_physical[2])
{
    u_physical[0] = D[0] * u_control[0] + D[1] * u_control[1];
    u_physical[1] = D[2] * u_control[0] + D[3] * u_control[1];
}

/*===========================================================================
 * L5: Relative Gain Array (RGA)
 *
 * RGA = K .* K^{-T}  (element-wise multiplication with inverse transpose)
 *
 * For 2x2:
 *   λ = K11*K22 / (K11*K22 - K12*K21)
 *   RGA = [ λ,    1-λ ]
 *         [ 1-λ,  λ    ]
 *
 * RGA interpretation (Bristol, 1966):
 *   λ ≈ 1 : diagonal pairing is correct
 *   λ ≈ 0 : off-diagonal pairing is correct
 *   0 < λ < 0.5 : strong interaction
 *   λ < 0  : unstable if paired diagonally!
 *
 * Reference: Bristol, E.H. (1966) "On a new measure of interaction
 *   for multivariable process control", IEEE TAC, 11(1), 133-134
 *===========================================================================*/

/**
 * @brief Compute RGA for 2x2 process.
 *
 * @param K     Gain matrix [K11, K12, K21, K22]
 * @param rga   Output RGA [λ11, λ12, λ21, λ22]
 * @return      0 on success, -1 if singular
 */
int smith_mimo_rga(const double K[4], double rga[4])
{
    double det = K[0] * K[3] - K[1] * K[2];

    if (fabs(det) < 1e-12) return -1;

    double lambda = K[0] * K[3] / det;

    rga[0] = lambda;       /* λ11 */
    rga[1] = 1.0 - lambda; /* λ12 */
    rga[2] = 1.0 - lambda; /* λ21 */
    rga[3] = lambda;       /* λ22 */

    return 0;
}

/**
 * @brief Recommend loop pairing based on RGA.
 *
 * Positive RGA rule: pair inputs to outputs such that RGA elements
 * are closest to 1 (for diagonal pairing) or 0 (for off-diagonal).
 *
 * @param rga           RGA matrix
 * @param pairing       Output: [input_for_y1, input_for_y2], 0-based indices
 * @param interaction   Output: RGA-based interaction measure [0,1]
 */
void smith_mimo_pairing_recommend(const double rga[4],
                                   int pairing[2],
                                   double *interaction)
{
    /* Diagonal pairing if λ is near 1 */
    if (rga[0] > 0.5 && rga[0] < 1.5) {
        pairing[0] = 0;
        pairing[1] = 1;
        *interaction = fabs(1.0 - rga[0]);  /* 0 = no interaction */
    } else if (rga[0] < -0.5) {
        /* Negative RGA: diagonal is unstable, use off-diagonal */
        pairing[0] = 1;
        pairing[1] = 0;
        *interaction = fabs(rga[1]);  /* Check if off-diagonal pairing is good */
    } else {
        /* Ambiguous: check which pairing gives RGA closer to 1 */
        if (fabs(rga[0] - 1.0) < fabs(rga[1] - 1.0)) {
            pairing[0] = 0; pairing[1] = 1;
            *interaction = fabs(1.0 - rga[0]);
        } else {
            pairing[0] = 1; pairing[1] = 0;
            *interaction = fabs(1.0 - rga[1]);
        }
    }
}

/*===========================================================================
 * L5: MIMO Smith Predictor Initialization
 *===========================================================================*/

/**
 * @brief Allocate and initialize 2x2 MIMO Smith predictor.
 *
 * @param mimo    Pointer to MIMO Smith predictor state
 * @param K       Gain matrix [K11, K12, K21, K22]
 * @param tau     Time constants [τ11, τ12, τ21, τ22]
 * @param theta   Dead times [θ11, θ12, θ21, θ22]
 * @param Ts      Sampling period
 * @param u_min   Lower limits [u1_min, u2_min]
 * @param u_max   Upper limits [u1_max, u2_max]
 * @return        0 on success
 */
int smith_mimo_2x2_init(smith_mimo_2x2_t *mimo,
                         const double K[4], const double tau[4],
                         const double theta[4], double Ts,
                         const double u_min[2], const double u_max[2])
{
    if (!mimo) return -1;
    memset(mimo, 0, sizeof(*mimo));

    /* Store model */
    memcpy(mimo->model.K, K, 4 * sizeof(double));
    memcpy(mimo->model.tau, tau, 4 * sizeof(double));
    memcpy(mimo->model.theta, theta, 4 * sizeof(double));

    mimo->Ts = Ts;
    memcpy(mimo->u_min, u_min, 2 * sizeof(double));
    memcpy(mimo->u_max, u_max, 2 * sizeof(double));

    /* Allocate delay buffers for each channel */
    for (int ch = 0; ch < 4; ch++) {
        double th = theta[ch];
        if (th < 0.0) th = 0.0;
        size_t len = (size_t)ceil(th / Ts);
        size_t cap = len + 4;
        if (cap < 2) cap = 2;

        mimo->delay_buf[ch] = (double *)calloc(cap, sizeof(double));
        if (!mimo->delay_buf[ch]) {
            /* Cleanup partial allocation */
            for (int j = 0; j < ch; j++) free(mimo->delay_buf[j]);
            return -1;
        }
        mimo->delay_len[ch] = len;
        mimo->delay_cap[ch] = cap;
        mimo->delay_head[ch] = 0;
    }

    /* Initialize PI gains */
    for (int i = 0; i < 2; i++) {
        mimo->Kp[i] = 0.0;
        mimo->Ti[i] = 0.0;
    }

    return 0;
}

/**
 * @brief Free MIMO Smith predictor resources.
 */
void smith_mimo_2x2_destroy(smith_mimo_2x2_t *mimo)
{
    if (!mimo) return;
    for (int ch = 0; ch < 4; ch++) {
        free(mimo->delay_buf[ch]);
        mimo->delay_buf[ch] = NULL;
    }
}

/**
 * @brief Set PI tuning for one loop of the MIMO Smith predictor.
 *
 * @param mimo    MIMO Smith predictor
 * @param loop    Loop index (0 or 1)
 * @param Kp      Proportional gain
 * @param Ti      Integral time
 * @return        0 on success
 */
int smith_mimo_2x2_set_pi(smith_mimo_2x2_t *mimo, int loop,
                           double Kp, double Ti)
{
    if (!mimo || loop < 0 || loop > 1) return -1;
    mimo->Kp[loop] = Kp;
    mimo->Ti[loop] = Ti;
    return 0;
}

/*===========================================================================
 * L5: MIMO Smith Predictor — Control Step
 *
 * For each loop i:
 *   1. Update all 4 delay-free model states
 *   2. Compute delayed model outputs
 *   3. Form Smith predictor feedback per loop
 *   4. Compute PI outputs
 *   5. Optionally apply static decoupler
 *===========================================================================*/

/**
 * @brief Execute one control step of the 2x2 MIMO Smith predictor.
 *
 * @param mimo      MIMO Smith predictor state
 * @param setpoint  Setpoints [r1, r2]
 * @param pv        Process measurements [y1, y2]
 * @param u_out     Controller outputs (actuator signals) [u1, u2]
 * @param decouple  1 = apply static decoupler, 0 = decentralized
 */
void smith_mimo_2x2_step(smith_mimo_2x2_t *mimo,
                          const double setpoint[2],
                          const double pv[2],
                          double u_out[2],
                          int decouple)
{
    if (!mimo) return;

    double *K = mimo->model.K;
    double *tau = mimo->model.tau;
    double *theta = mimo->model.theta;
    double Ts = mimo->Ts;
    double *ms = mimo->model_state;

    /* Previous outputs (delayed by one sample for model update) */
    double u_prev[2] = { mimo->prev_output[0], mimo->prev_output[1] };

    /* --- Step 1: Update all 4 delay-free model states --- */
    /* FOPDT discrete model (forward Euler):
       ms[ij](k) = ms[ij](k-1) + Ts/tau[ij] * (K[ij]*u_j(k-1) - ms[ij](k-1)) */
    for (int ch = 0; ch < 4; ch++) {
        int input_idx = ch % 2;  /* ch 0,2 → input 0; ch 1,3 → input 1 */
        double uj = u_prev[input_idx];

        if (tau[ch] > 1e-9) {
            ms[ch] += Ts / tau[ch] * (K[ch] * uj - ms[ch]);
        } else {
            ms[ch] = K[ch] * uj;  /* Instantaneous response */
        }
    }

    /* --- Step 2: Push to delay buffers and retrieve delayed outputs --- */
    double ym[4];  /* Delayed model outputs */
    for (int ch = 0; ch < 4; ch++) {
        size_t cap = mimo->delay_cap[ch];
        /* Push */
        mimo->delay_head[ch] = (mimo->delay_head[ch] + 1) % cap;
        mimo->delay_buf[ch][mimo->delay_head[ch]] = ms[ch];
        /* Pop */
        size_t d = mimo->delay_len[ch];
        size_t idx;
        if (mimo->delay_head[ch] >= d) {
            idx = mimo->delay_head[ch] - d;
        } else {
            idx = cap - (d - mimo->delay_head[ch]);
        }
        ym[ch] = mimo->delay_buf[ch][idx];
    }

    /* --- Step 3 & 4: Per-loop Smith predictor feedback + PI control --- */
    double u_ctrl[2] = {0.0, 0.0};

    for (int loop_i = 0; loop_i < 2; loop_i++) {
        int ch_main = loop_i * 3;  /* 0 or 3 (diagonal channels) */
        /* Actually: ch for loop 0 = [0 (G11), 1 (G12)]
                    ch for loop 1 = [2 (G21), 3 (G22)] */
        int ch_i = loop_i * 2 + loop_i;  /* Loop 0 → ch 0 (G11), Loop 1 → ch 3 (G22) */

        /* Smith feedback for this loop:
           y_fb_i = yp_i + (y_i - Σ ym_ij)
           where yp_i = Σ ms_ij  (delay-free prediction for loop i)
                 Σ ym_ij = ym_i0 + ym_i1 (sum of delayed model contributions) */
        double yp_i = 0.0;
        double ym_sum_i = 0.0;

        for (int j = 0; j < 2; j++) {
            int ch = loop_i * 2 + j;  /* ch = 0,1 for loop 0; 2,3 for loop 1 */
            yp_i += ms[ch];
            ym_sum_i += ym[ch];
        }

        /* Smith feedback: y_fb = yp + (y_i - ym_sum) */
        double y_fb = yp_i + (pv[loop_i] - ym_sum_i);

        /* PI control for this loop */
        double error = setpoint[loop_i] - y_fb;

        /* P term */
        double p_term = mimo->Kp[loop_i] * error;

        /* I term */
        double i_term = 0.0;
        if (mimo->Ti[loop_i] > 0.0) {
            mimo->integrator[loop_i] +=
                mimo->Kp[loop_i] * Ts / mimo->Ti[loop_i] * error;
        }
        i_term = mimo->integrator[loop_i];

        u_ctrl[loop_i] = p_term + i_term;

        /* Saturation */
        if (u_ctrl[loop_i] > mimo->u_max[loop_i]) {
            u_ctrl[loop_i] = mimo->u_max[loop_i];
            /* Anti-windup: freeze integrator */
            mimo->integrator[loop_i] = u_ctrl[loop_i] - p_term;
        }
        if (u_ctrl[loop_i] < mimo->u_min[loop_i]) {
            u_ctrl[loop_i] = mimo->u_min[loop_i];
            mimo->integrator[loop_i] = u_ctrl[loop_i] - p_term;
        }

        mimo->prev_error[loop_i] = error;
    }

    /* --- Step 5: Optionally apply static decoupler --- */
    if (decouple) {
        double D[4];
        if (smith_mimo_static_decoupler(K, D) == 0) {
            smith_mimo_apply_decoupler(D, u_ctrl, u_out);
        } else {
            /* Decoupler singular — use decentralized (no transform) */
            u_out[0] = u_ctrl[0];
            u_out[1] = u_ctrl[1];
        }
    } else {
        u_out[0] = u_ctrl[0];
        u_out[1] = u_ctrl[1];
    }

    /* Store for next iteration */
    mimo->prev_output[0] = u_out[0];
    mimo->prev_output[1] = u_out[1];
}

/*===========================================================================
 * L8: Niederlinski Index (Stability Condition for MIMO)
 *
 * NI = det(K) / Π K_ii
 *
 * If NI < 0, the system will be unstable with any controller that
 * has integral action and is tuned one loop at a time.
 *
 * This is a necessary condition for decentralized integral controllability.
 *
 * Reference: Niederlinski, A. (1971) "A heuristic approach to the design
 *   of linear multivariable interacting control systems", Automatica, 7, 691-701
 *===========================================================================*/

/**
 * @brief Compute Niederlinski Index for 2x2 process.
 *
 * @param K     Gain matrix
 * @return      NI value
 */
double smith_mimo_niederlinski_index(const double K[4])
{
    double det = K[0] * K[3] - K[1] * K[2];
    double prod_diag = K[0] * K[3];

    if (fabs(prod_diag) < 1e-12) return 0.0;

    return det / prod_diag;
}

/*===========================================================================
 * L8: Morari Resiliency Index
 *
 * MRI = minimum singular value of G(0)
 *
 * Higher MRI means the system is easier to control.
 * MRI << 1: ill-conditioned, hard to control
 * MRI ≥ 1: well-conditioned, easy to control
 *
 * Reference: Morari, M. (1983) "Design of resilient processing plants",
 *   Chem. Eng. Science, 38(11), 1881-1891
 *===========================================================================*/

/**
 * @brief Compute Morari Resiliency Index (minimum singular value).
 *
 * For 2x2 matrix K:
 *   σ_min = sqrt(C - sqrt(C² - D))  where C = (a²+b²+c²+d²)/2, D = det(K)²
 *   for K = [a b; c d]
 *
 * @param K     Gain matrix
 * @return      Minimum singular value (MRI)
 */
double smith_mimo_resiliency_index(const double K[4])
{
    double a = K[0], b = K[1], c = K[2], d = K[3];
    double C = (a*a + b*b + c*c + d*d) / 2.0;
    double det = a * d - b * c;
    double D = det * det;

    double discriminant = C * C - D;
    if (discriminant < 0.0) discriminant = 0.0;

    double sigma_min_sq = C - sqrt(discriminant);
    if (sigma_min_sq < 0.0) sigma_min_sq = 0.0;

    return sqrt(sigma_min_sq);
}

/*===========================================================================
 * L5: Condition Number
 *
 * κ = σ_max / σ_min
 *
 * κ < 10 : well-conditioned
 * 10 ≤ κ < 100 : moderately ill-conditioned
 * κ ≥ 100 : severely ill-conditioned — decoupling or MPC recommended
 *===========================================================================*/

/**
 * @brief Compute condition number of the 2x2 gain matrix.
 *
 * @param K     Gain matrix
 * @return      Condition number κ
 */
double smith_mimo_condition_number(const double K[4])
{
    double a = K[0], b = K[1], c = K[2], d = K[3];
    double C = (a*a + b*b + c*c + d*d) / 2.0;
    double det = a * d - b * c;
    double D = det * det;

    double discriminant = C * C - D;
    if (discriminant < 0.0) discriminant = 0.0;

    double sigma_max_sq = C + sqrt(discriminant);
    double sigma_min_sq = C - sqrt(discriminant);

    if (sigma_min_sq < 1e-12) return 1e6;

    return sqrt(sigma_max_sq / sigma_min_sq);
}
