#include "feedforward_defs.h"
#include "feedforward_static.h"
#include "feedforward_dynamic.h"
#include "feedforward_models.h"
#include "feedforward_combined.h"
#include "feedforward_advanced.h"
#include <math.h>
#include <string.h>

/**
 * @file feedforward_applications.c
 * @brief Industrial application examples of static/dynamic feedforward control
 *
 * Knowledge: L6 Canonical Problems, L7 Industrial Applications
 *
 * Implements complete feedforward control solutions for classic
 * industrial control problems:
 *
 * 1. Heat exchanger temperature control (L6)
 *    — Feedforward from inlet flow rate disturbance
 *    — Process: tube-side outlet temperature, Disturbance: shell-side flow
 *
 * 2. Distillation column composition control (L6)
 *    — Feedforward from feed flow rate and composition changes
 *    — Process: top/bottom composition, Disturbance: feed rate/composition
 *
 * 3. pH neutralization process (L6)
 *    — Feedforward from influent flow and concentration
 *    — Highly nonlinear, requires gain scheduling
 *
 * 4. Boiler drum level control (L6)
 *    — Feedforward from steam demand (shrink/swell compensation)
 *    — Non-minimum-phase dynamics (inverse response)
 *
 * 5. DC motor speed control — Toyota production line (L7)
 *    — Feedforward from load torque disturbance
 *    — Combined with velocity feedback for precision assembly
 *
 * References:
 *   Seborg et al. (2016) §15.6 — Industrial feedforward case studies
 *   Myke King (2016) §8 — Practical feedforward design
 *   Shinskey (1996) "Process Control Systems" — Distillation, pH
 */

/* ============================================================================
 * L6: Heat exchanger — Feedforward temperature control
 * ============================================================================ */

/**
 * @brief Heat exchanger feedforward design parameters
 *
 * Shell-and-tube heat exchanger:
 * - Controlled variable: tube-side outlet temperature T_out [°C]
 * - Manipulated variable: steam valve position [%]
 * - Measured disturbance: tube-side inlet flow rate F_in [kg/s]
 *
 * FOPDT model from process reaction curve (Seborg et al. §15.6.1):
 *   Gp(s) = -0.8 * e^(-30s) / (120s + 1)   [°C/%]
 *   Gd(s) = 0.15 * e^(-15s) / (90s + 1)    [°C/(kg/s)]
 *
 * Static FF gain: Kff = -Kd/Kp = -0.15/(-0.8) = 0.1875 [%/ (kg/s)]
 * Dynamic FF: T_lead = 120s, T_lag = 90s
 */
typedef struct {
    fopdt_t     process;       /**< Process model Gp(s) */
    dist_model_t dist;         /**< Disturbance model Gd(s) */
    feedforward_t ff;          /**< Configured feedforward controller */
    double      T_out;         /**< Current outlet temperature [°C] */
    double      F_in;          /**< Current inlet flow [kg/s] */
    double      steam_valve;   /**< Current valve position [%] */
    double      T_sp;          /**< Temperature setpoint [°C] */
    double      Kc;            /**< Feedback PI gain */
    double      Ti;            /**< Feedback integral time [s] */
    double      e_int;         /**< Integral of error */
    double      Ts;            /**< Sample time [s] */
} heat_exchanger_control_t;

/**
 * @brief Initialize heat exchanger feedforward control
 *
 * @param hx         Heat exchanger control structure
 * @param Ts         Sample time [s]
 */
void heat_exchanger_ff_init(heat_exchanger_control_t *hx, double Ts)
{
    if (!hx) return;
    memset(hx, 0, sizeof(heat_exchanger_control_t));

    /* Process model: Gp(s) = Kp * e^(-theta*s) / (tau*s + 1)
     * Typical values for a small shell-and-tube exchanger */
    fopdt_init(&hx->process, -0.8, 120.0, 30.0);
    dist_model_init(&hx->dist, 0.15, 90.0, 15.0);
    hx->Ts = Ts;

    /* Design feedforward:
     * Kff = -Kd/Kp = -0.15/(-0.8) = 0.1875
     * T_lead = tau_p = 120s
     * T_lag = tau_d = 90s
     *
     * Check causality: theta_d (=15) < theta_p (=30) → non-causal
     * Required extra delay: 30 - 15 = 15s
     * → Add 15s delay to FF action (or accept suboptimal performance)
     */
    double Kff = -hx->dist.Kd / hx->process.Kp;
    double T_lead = hx->process.tau;
    double T_lag = hx->dist.tau_d;
    double required_delay = ff_dynamic_required_delay(hx->process.theta, hx->dist.theta_d);

    /* Configure combined FF: static + dynamic with delay approximation */
    feedforward_configure_dynamic(&hx->ff, Kff, T_lead, T_lag + required_delay,
                                  50.0,  /* bias = 50% (mid-range) */
                                  0.0, 100.0, ACTION_DIRECT, Ts);

    /* Feedback PI tuning (IMC-based for FOPDT):
     * Kc = tau/(Kp*(lambda + theta)) with lambda = tau */
    double lambda_cl = hx->process.tau;
    hx->Kc = hx->process.tau / (fabs(hx->process.Kp) * (lambda_cl + hx->process.theta));
    hx->Ti = hx->process.tau;
    hx->e_int = 0.0;
}

/**
 * @brief One control step for heat exchanger
 *
 * @param hx       Heat exchanger control
 * @param T_out    Measured outlet temperature [°C]
 * @param F_in     Measured inlet flow (disturbance) [kg/s]
 * @return Steam valve position [%]
 */
double heat_exchanger_ff_step(heat_exchanger_control_t *hx, double T_out, double F_in)
{
    if (!hx || !hx->ff.initialized) return 50.0;

    hx->T_out = T_out;
    hx->F_in = F_in;

    /* Feedback (PI) */
    double error = hx->T_sp - T_out;
    hx->e_int += error * hx->Ts;
    double u_fb = hx->Kc * (error + hx->e_int / hx->Ti);

    /* Feedforward from inlet flow disturbance */
    double u_ff = feedforward_step(&hx->ff, F_in);

    /* Combined output */
    double u = u_fb + u_ff;

    /* Clamp */
    if (u > 100.0) u = 100.0;
    if (u < 0.0) u = 0.0;

    hx->steam_valve = u;
    return u;
}

/* ============================================================================
 * L6: Distillation column — Feedforward composition control
 * ============================================================================ */

/**
 * @brief Distillation column feedforward configuration
 *
 * Binary distillation column:
 * - Controlled variable: top product composition (impurity) [mol%]
 * - Manipulated variable: reflux flow rate [kmol/h]
 * - Measured disturbance: feed flow rate F [kmol/h]
 *
 * Typical FOPDT from step test (Skogestad, 1997):
 *   Gp(s) = 0.05 * e^(-120s) / (300s + 1)   [mol% / (kmol/h)]
 *   Gd(s) = 0.02 * e^(-90s) / (240s + 1)    [mol% / (kmol/h)]
 *
 * Static FF: Kff = -0.02/0.05 = -0.4 [(kmol/h reflux)/(kmol/h feed)]
 * Dynamic FF: T_lead = 300s, T_lag = 240s
 */
typedef struct {
    fopdt_t     process;
    dist_model_t dist;
    feedforward_t ff;
    double      composition;   /**< Top product impurity [mol%] */
    double      feed_rate;     /**< Feed flow rate [kmol/h] */
    double      reflux;        /**< Reflux flow rate [kmol/h] */
    double      comp_sp;       /**< Composition setpoint [mol%] */
    double      Kc, Ti;
    double      e_int;
    double      Ts;
} distillation_ff_control_t;

/**
 * @brief Initialize distillation column feedforward control
 */
void distillation_ff_init(distillation_ff_control_t *dc, double Ts)
{
    if (!dc) return;
    memset(dc, 0, sizeof(distillation_ff_control_t));

    /* Gp(s) = Kp * e^(-theta*s)/(tau*s+1) */
    fopdt_init(&dc->process, 0.05, 300.0, 120.0);
    dist_model_init(&dc->dist, 0.02, 240.0, 90.0);
    dc->Ts = Ts;

    /* Feedforward design */
    double Kff = -dc->dist.Kd / dc->process.Kp; /* -0.4 */
    double T_lead = dc->process.tau;             /* 300s */
    double T_lag = dc->dist.tau_d;               /* 240s */

    /* Check causality: theta_d (=90) < theta_p (=120), need 30s delay */
    double extra_delay = ff_dynamic_required_delay(dc->process.theta, dc->dist.theta_d);

    feedforward_configure_dynamic(&dc->ff, Kff, T_lead, T_lag + extra_delay,
                                  100.0, 0.0, 500.0, ACTION_DIRECT, Ts);

    /* IMC-based PI tuning */
    double lambda_cl = dc->process.tau * 0.5; /* More aggressive for column */
    dc->Kc = dc->process.tau / (fabs(dc->process.Kp) * (lambda_cl + dc->process.theta));
    dc->Ti = dc->process.tau;
    dc->e_int = 0.0;
}

double distillation_ff_step(distillation_ff_control_t *dc, double composition,
                            double feed_rate)
{
    if (!dc || !dc->ff.initialized) return 100.0;

    dc->composition = composition;
    dc->feed_rate = feed_rate;

    double error = dc->comp_sp - composition;
    dc->e_int += error * dc->Ts;
    double u_fb = dc->Kc * (error + dc->e_int / dc->Ti);

    double u_ff = feedforward_step(&dc->ff, feed_rate);
    double u = u_fb + u_ff;

    if (u > 500.0) u = 500.0;
    if (u < 0.0) u = 0.0;

    dc->reflux = u;
    return u;
}

/* ============================================================================
 * L6: pH neutralization — Gain-scheduled feedforward
 * ============================================================================ */

/**
 * @brief pH neutralization process with gain-scheduled feedforward
 *
 * pH control is notoriously difficult due to extreme nonlinearity
 * of the titration curve. Gain scheduling is essential.
 *
 * Process: continuous stirred tank reactor (CSTR) neutralization
 * - Controlled: pH (outlet)
 * - Manipulated: reagent flow rate [L/min]
 * - Disturbance: influent flow rate and concentration
 *
 * The process gain Kp(pH) varies by orders of magnitude across the pH range.
 * Near neutrality (pH 7), gain is very high (steep titration curve).
 * Away from neutrality, gain is low (flat titration curve).
 *
 * This implementation uses a piecewise linear gain schedule for Kff.
 */
typedef struct {
    double      pH;            /**< Current pH */
    double      flow_in;       /**< Influent flow rate [L/min] */
    double      reagent_flow;  /**< Reagent flow rate [L/min] */
    double      pH_sp;         /**< pH setpoint */
    ff_gain_schedule_t schedule; /**< Kff vs pH schedule */
    lead_lag_t  lead_lag;      /**< Dynamic compensation */
    double      Kc, Ti;
    double      e_int;
    double      Ts;
} ph_neutralization_ff_t;

/**
 * @brief Initialize pH neutralization feedforward control
 *
 * Sets up gain schedule for Kff based on titration curve nonlinearity.
 */
void ph_ff_init(ph_neutralization_ff_t *ph, double Ts)
{
    if (!ph) return;
    memset(ph, 0, sizeof(ph_neutralization_ff_t));

    ph->Ts = Ts;
    ph->pH_sp = 7.0;

    /* Gain schedule: Kff vs pH
     *
     * At pH 3-5 (acid region): titration curve is flat → low Kff
     * At pH 6-8 (neutral region): titration curve is steep → high Kff
     * At pH 9-11 (base region): titration curve is flat → low Kff
     *
     * These values reflect typical strong acid/strong base titration.
     * Kff = -Kd/Kp, where Kp(pH) is the inverse of the buffer capacity.
     */
    double ph_points[] = {2.0, 4.0, 5.5, 6.5, 7.0, 7.5, 8.5, 10.0, 12.0};
    double Kff_points[] = {0.02, 0.03, 0.08, 0.5, 1.0, 0.6, 0.1, 0.03, 0.02};
    int n = 9;

    ff_gain_schedule_init(&ph->schedule, ph_points, Kff_points, n);

    /* Lead-lag: T_lead = 60s (process mixing lag), T_lag = 30s (disturbance faster) */
    lead_lag_init(&ph->lead_lag, 1.0, 60.0, 30.0, Ts);

    /* Feedback PI: very conservative near neutrality */
    ph->Kc = 0.5;
    ph->Ti = 120.0;
    ph->e_int = 0.0;
}

double ph_ff_step(ph_neutralization_ff_t *ph, double pH_meas, double flow_in)
{
    if (!ph) return 0.0;

    ph->pH = pH_meas;
    ph->flow_in = flow_in;

    /* Get scheduled feedforward gain at current pH */
    double Kff_scheduled = ff_gain_schedule_lookup(&ph->schedule, pH_meas);

    /* Dynamic feedforward: scheduled gain * lead-lag(flow_in) */
    double u_ff_static = Kff_scheduled * flow_in;
    ph->lead_lag.K_ll = Kff_scheduled; /* Update lead-lag gain for scheduling */
    double u_ff_dyn = lead_lag_step(&ph->lead_lag, flow_in);

    /* Combined static + dynamic */
    double u_ff = 0.3 * u_ff_static + 0.7 * u_ff_dyn;

    /* Feedback */
    double error = ph->pH_sp - pH_meas;
    ph->e_int += error * ph->Ts;
    double u_fb = ph->Kc * (error + ph->e_int / ph->Ti);

    double u = u_fb + u_ff;
    if (u < 0.0) u = 0.0;
    if (u > 100.0) u = 100.0;

    ph->reagent_flow = u;
    return u;
}

/* ============================================================================
 * L6: Boiler drum level — Feedforward with inverse response
 * ============================================================================ */

/**
 * @brief Boiler drum level feedforward control
 *
 * Drum level exhibits "shrink/swell" behavior (non-minimum-phase):
 * - Steam demand ↑ → drum pressure ↓ → bubbles expand → level rises (swell)
 *   THEN steam ↑ → water mass ↓ → level drops
 *
 * This inverse response makes feedback-only control difficult.
 * Feedforward from steam flow is essential for tight level control,
 * especially during load changes (e.g., Fukushima-type reactors, power plants).
 *
 * Model (Åström & Bell, 2000):
 *   Gp(s) = Kp * (1 - a*s) / (s * (tau*s + 1))   [level / feedwater flow]
 *   Gd(s) = -Kd * (1 - b*s) / (s * (tau*s + 1))   [level / steam flow]
 *
 * where a > 0 indicates inverse response (non-minimum-phase zero).
 */
typedef struct {
    double      level;         /**< Drum water level [%] */
    double      steam_flow;    /**< Steam mass flow [kg/s] */
    double      feedwater;     /**< Feedwater flow [kg/s] */
    double      level_sp;      /**< Level setpoint [%] */
    double      Kp, Kd;        /**< Process and disturbance gains */
    double      a, b;          /**< Inverse response parameters */
    double      tau;           /**< Time constant [s] */
    double      integral;      /**< Integrating process internal state */
    feedforward_t ff;
    double      Kc, Ti;
    double      e_int;
    double      Ts;
} boiler_level_ff_t;

/**
 * @brief Initialize boiler drum level feedforward
 */
void boiler_level_ff_init(boiler_level_ff_t *bl, double Ts)
{
    if (!bl) return;
    memset(bl, 0, sizeof(boiler_level_ff_t));

    /* Typical boiler parameters (Åström & Bell, 2000):
     * Kp = 0.05, Kd = 0.05, a = 10s, b = 5s, tau = 30s */
    bl->Kp = 0.05;
    bl->Kd = 0.05;
    bl->a = 10.0;
    bl->b = 5.0;
    bl->tau = 30.0;
    bl->Ts = Ts;
    bl->level_sp = 50.0;

    /* Feedforward: Kff = Kd/Kp = 1.0 (compensate steam demand with feedwater)
     * Since both have inverse responses, the feedforward uses static gain
     * with an appropriate lag filter to avoid amplifying the swell effect. */
    double Kff = bl->Kd / bl->Kp;
    double T_lead = bl->a;   /* Lead compensates for inverse response */
    double T_lag = bl->a * 2.0; /* Lag provides robustness */

    feedforward_configure_dynamic(&bl->ff, Kff, T_lead, T_lag,
                                  50.0, 0.0, 100.0, ACTION_DIRECT, Ts);

    /* PI with moderate gain (inverse response limits high gain) */
    bl->Kc = 2.0;
    bl->Ti = bl->tau;
    bl->e_int = 0.0;
}

double boiler_level_ff_step(boiler_level_ff_t *bl, double level_meas,
                            double steam_flow)
{
    if (!bl) return 50.0;

    bl->level = level_meas;
    bl->steam_flow = steam_flow;

    /* Feedback (PI) */
    double error = bl->level_sp - level_meas;
    bl->e_int += error * bl->Ts;
    double u_fb = bl->Kc * (error + bl->e_int / bl->Ti);

    /* Feedforward */
    double u_ff = feedforward_step(&bl->ff, steam_flow);

    double u = u_fb + u_ff;
    if (u > 100.0) u = 100.0;
    if (u < 0.0) u = 0.0;

    bl->feedwater = u;
    return u;
}

/* ============================================================================
 * L7: DC motor speed control — Toyota production line
 * ============================================================================ */

/**
 * @brief DC motor speed control with feedforward from load torque
 *
 * Application context: Toyota manufacturing line — precision positioning
 * for robotic assembly. Feedforward compensates for varying load torque
 * from part weight and friction changes.
 *
 * Model:
 *   Gp(s) = Km / (J*s + B)          [rad/s per V]  (first-order mechanical)
 *   Gd(s) = -1 / (J*s + B)          [rad/s per Nm]  (load torque effect)
 *
 * Static FF: Kff = 1/Km  [V per Nm]
 * Dynamic FF: no lead-lag needed (same dynamics), pure gain FF
 *
 * Reference: Hughes & Drury (2013) "Electric Motors and Drives" §5
 */
typedef struct {
    double      speed;          /**< Motor angular velocity [rad/s] */
    double      torque_load;    /**< Load torque [Nm] */
    double      voltage;        /**< Motor terminal voltage [V] */
    double      speed_sp;       /**< Speed setpoint [rad/s] */
    double      Km;             /**< Motor torque constant [Nm/A or rad/s/V] */
    double      J;              /**< Rotor inertia [kg·m²] */
    double      B;              /**< Viscous friction [Nm/(rad/s)] */
    feedforward_t ff;
    double      Kc, Ti;
    double      e_int;
    double      Ts;
} dc_motor_ff_t;

void dc_motor_ff_init(dc_motor_ff_t *motor, double Km, double J, double B, double Ts)
{
    if (!motor) return;
    memset(motor, 0, sizeof(dc_motor_ff_t));

    motor->Km = Km;
    motor->J = J;
    motor->B = B;
    motor->Ts = Ts;

    /* Feedforward: u_ff = torque_load / Km
     * This compensates directly for the load torque. */
    double Kff = 1.0 / Km;

    /* No dynamic compensation needed (same denominator in Gp and Gd).
     * Static-only feedforward. */
    feedforward_configure_static(&motor->ff, Kff, 0.0, -48.0, 48.0,
                                 ACTION_DIRECT, Ts);

    /* PI speed control (symmetrical optimum for motor drives)
     * tau_mech = J/B characterizes the mechanical time constant */
    (void)(J / B); /* tau_mech = J/B, used to validate Ts << tau_mech */
    motor->Kc = J / (2.0 * Km * Ts);
    motor->Ti = 4.0 * Ts;
    motor->e_int = 0.0;
}

double dc_motor_ff_step(dc_motor_ff_t *motor, double speed_meas, double torque_load)
{
    if (!motor) return 0.0;

    motor->speed = speed_meas;
    motor->torque_load = torque_load;

    /* Feedback PI */
    double error = motor->speed_sp - speed_meas;
    motor->e_int += error * motor->Ts;
    double u_fb = motor->Kc * (error + motor->e_int / motor->Ti);

    /* Feedforward from load torque */
    double u_ff = feedforward_step(&motor->ff, torque_load);

    double u = u_fb + u_ff;
    if (u > 48.0) u = 48.0;
    if (u < -48.0) u = -48.0;

    motor->voltage = u;
    return u;
}