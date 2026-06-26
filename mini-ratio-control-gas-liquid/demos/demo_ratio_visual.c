/**
 * @file demo_ratio_visual.c
 * @brief Demo: Visual Ratio Control Loop Response
 *
 * Generates CSV data for plotting ratio control loop response
 * to a master flow step change. Demonstrates:
 *   - Wild stream feedforward
 *   - Ratio trim feedback
 *   - Lead-lag dynamic compensation
 *
 * Output: CSV to stdout with columns for time, master, slave, ratio, trim
 */

#include <stdio.h>
#include <math.h>

/* Lead-lag compensator */
typedef struct { double gain, T_lead, T_lag, Ts, a1, b0, b1, prev_input, prev_output; int initialized; } lead_lag_compensator_t;

extern void   lead_lag_init(lead_lag_compensator_t *comp, double K, double T_lead, double T_lag, double Ts);
extern double lead_lag_step(lead_lag_compensator_t *comp, double input);
extern double flow_ewma_filter(double x_raw, double y_prev, double Ts, double tau);

/* Simple PI flow controller */
typedef struct { double Kp, Ti, Ts, integrator, prev_error; } pi_controller_t;

static void pi_init(pi_controller_t *pi, double Kp, double Ti, double Ts) {
    pi->Kp = Kp; pi->Ti = Ti; pi->Ts = Ts;
    pi->integrator = 0.0; pi->prev_error = 0.0;
}

static double pi_step(pi_controller_t *pi, double setpoint, double pv) {
    double error = setpoint - pv;
    double P = pi->Kp * error;
    if (pi->Ti > 0.0)
        pi->integrator += (pi->Kp * pi->Ts / pi->Ti) * error;
    double output = P + pi->integrator;
    if (output < 0.0) output = 0.0;
    if (output > 100.0) output = 100.0;
    pi->prev_error = error;
    return output;
}

int main(void)
{
    printf("time,master_flow,slave_flow,actual_ratio,target_ratio,trim_output,slave_sp\n");

    /* Simulation parameters */
    double Ts = 0.1;
    double R_sp = 2.0;          /* Target ratio */
    double F_master = 50.0;     /* Initial master flow */
    double F_slave = 100.0;     /* Initial slave flow */
    double slave_valve_pos = 50.0; /* Valve position */

    /* Slave flow PI controller */
    pi_controller_t slave_fc;
    pi_init(&slave_fc, 2.0, 5.0, Ts);

    /* Lead-lag compensator for dynamic alignment */
    lead_lag_compensator_t comp;
    lead_lag_init(&comp, 1.0, 1.0, 3.0, Ts);

    /* Ratio trim PI */
    typedef struct { double Kp, Ti, Ts, integrator, output; } trim_t;
    trim_t trim = {0.1, 60.0, 1.0, 0.0, 0.0};
    double trim_Ts = 1.0;
    int trim_counter = 0;

    /* Master flow filtered */
    double F_master_filt = F_master;

    /* Simulation: 100 seconds */
    for (int step = 0; step <= 1000; step++) {
        double t = step * Ts;

        /* Master flow step at t=10s: 50 → 80 */
        if (t >= 10.0) F_master = 80.0;

        /* Filter master flow */
        F_master_filt = flow_ewma_filter(F_master, F_master_filt, Ts, 1.0);

        /* Dynamic compensation */
        double F_master_comp = lead_lag_step(&comp, F_master_filt);

        /* Slave setpoint from ratio */
        double R_eff = R_sp + trim.output;
        double SP_slave = R_eff * F_master_comp;

        /* Slave flow control */
        double u_slave = pi_step(&slave_fc, SP_slave, F_slave);

        /* Valve dynamics (simplified 1st order) */
        double valve_tau = 0.5;
        double alpha_v = Ts / (valve_tau + Ts);
        slave_valve_pos += alpha_v * (u_slave - slave_valve_pos);

        /* Flow = valve_pos * sqrt(ΔP) simplified */
        F_slave = slave_valve_pos * 2.5; /* gain */

        /* Ratio trim (every 10 samples = 1 second) */
        trim_counter++;
        if (trim_counter >= 10) {
            trim_counter = 0;
            double R_actual = (F_slave > 0.01) ? F_slave / fmax(F_master_filt, 0.01) : 0.0;
            double trim_error = R_sp - R_actual;
            if (trim.Ti > 0.0)
                trim.integrator += (trim.Kp * trim_Ts / trim.Ti) * trim_error;
            trim.output = trim.Kp * trim_error + trim.integrator;
            if (trim.output < -0.5) trim.output = -0.5;
            if (trim.output >  0.5) trim.output =  0.5;
        }

        /* Output every 5 steps */
        if (step % 5 == 0) {
            double R_actual = (F_slave > 0.01) ? F_slave / fmax(F_master, 0.01) : 0.0;
            printf("%.1f,%.2f,%.2f,%.3f,%.3f,%.4f,%.2f\n",
                   t, F_master, F_slave, R_actual, R_sp, trim.output, SP_slave);
        }
    }

    fprintf(stderr, "Demo complete: CSV data written to stdout\n");
    return 0;
}
