/** @file example_servo_position.c
 * @brief Servo Position Control with Velocity-Dependent Gains (L6/L7)
 *
 * Demonstrates gain-scheduled PID for a servo motor position control
 * where the effective inertia and damping vary with velocity.
 * Uses AMIGO tuning at each velocity breakpoint.
 *
 * Application: CNC machine tool positioning, robotic arm joint control.
 * Scheduling variable: Motor velocity (SCHED_VAR_VELOCITY)
 */
#include "../include/gain_schedule_core.h"
#include "../include/gain_schedule_design.h"
#include "../include/gain_schedule_pid.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double servo_model(double theta, double omega,
                          double u, double dt,
                          double *omega_out) {
    double J = 0.01 + 0.001 * fabs(omega) * 10.0;
    double B = 0.1 + 0.02 * fabs(omega);
    double Kt = 0.5;
    double alpha = (Kt * u - B * omega) / J;
    double omega_new = omega + alpha * dt;
    double theta_new = theta + omega_new * dt;
    *omega_out = omega_new;
    return theta_new;
}

int main(void) {
    printf("\n========================================\n");
    printf("  Servo Position Gain-Scheduled PID Demo\n");
    printf("========================================\n\n");
    
    gain_schedule_table_t table;
    gs_table_init(&table, SCHED_VAR_VELOCITY);
    
    double sv[] = {0.0, 5.0, 10.0, 20.0, 30.0};
    double K_arr[] = {2.0, 1.5, 1.2, 0.9, 0.7};
    double tau_arr[] = {0.1, 0.12, 0.15, 0.18, 0.2};
    double L_arr[] = {0.005, 0.005, 0.005, 0.005, 0.005};
    
    gs_design_frozen_parameter(&table, 5, sv, K_arr, tau_arr, L_arr, 5);
    
    printf("Schedule Table (%u entries):\n", gs_table_count(&table));
    printf("  Vel[rad/s]  Kp      Ki      Kd\n");
    printf("  ----------  ------  ------  ------\n");
    for (uint32_t i = 0; i < table.num_entries; i++) {
        printf("  %7.1f    %6.3f  %6.3f  %6.4f\n",
               table.entries[i].scheduling_value,
               table.entries[i].gains.Kp,
               table.entries[i].gains.Ki,
               table.entries[i].gains.Kd);
    }
    
    gs_pid_state_t pid;
    gs_pid_init(&pid, "SERVO-X-201", GS_PID_PARALLEL_IDEAL);
    gs_pid_set_dt(&pid, 0.001);
    gs_pid_set_saturation(&pid, 24.0, -24.0);
    
    double theta = 0.0, omega = 0.0;
    double setpoint = M_PI / 2.0;
    
    printf("\nStep response to %.2f rad:\n\n", setpoint);
    printf("  Time[s]  Pos[rad]  SP[rad]  Vel[rad/s]  u[V]    Kp\n");
    printf("  -------  --------  -------  ----------  ------  -----\n");
    
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        double u;
        gs_pid_update(&pid, &table, setpoint, theta, fabs(omega), &u);
        theta = servo_model(theta, omega, u, 0.001, &omega);
        
        if (fmod(t * 1000.0, 50.0) < 0.5) {
            printf("  %7.3f  %8.4f  %7.3f  %10.3f  %6.2f  %6.3f\n",
                   t, theta, setpoint, omega, u, pid.Kp_current);
        }
    }
    
    printf("\nFinal position: %.4f rad (target: %.4f)\n", theta, setpoint);
    printf("Steady-state error: %.4f rad\n", setpoint - theta);
    printf("Schedule switches: %llu\n",
           (unsigned long long)gs_pid_get_switch_count(&pid));
    printf("Example complete.\n");
    return 0;
}
