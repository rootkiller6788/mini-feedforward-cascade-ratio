#include <stdio.h>
#include <math.h>
#include "../include/feedforward_defs.h"
#include "../include/feedforward_static.h"
#include "../include/feedforward_dynamic.h"
#include "../include/feedforward_combined.h"

int main(void) {
    printf("=== DC Motor Speed Control with Feedforward ===\n");
    printf("Application: Toyota production line — precision robotic assembly\n\n");

    double Km = 0.05, J = 0.001, B = 0.0005;
    printf("Motor parameters: Km=%.3f Nm/A, J=%.4f kg.m2, B=%.5f Nm/(rad/s)\n", Km, J, B);
    printf("Time constant tau_mech = J/B = %.2f s\n\n", J/B);

    /* Static feedforward: u_ff = torque_load / Km */
    double Kff = 1.0 / Km;
    printf("Feedforward gain: Kff = 1/Km = %.1f V/Nm\n", Kff);
    printf("(Each 1 Nm of load torque requires %.1f V additional voltage)\n\n", Kff);

    feedforward_t ff;
    feedforward_configure_static(&ff, Kff, 0.0, -48.0, 48.0, ACTION_DIRECT, 0.001);

    /* PI speed control */
    double Ts = 0.001, Kc = J / (2.0 * Km * Ts), Ti = 4.0 * Ts;
    printf("PI: Kc=%.2f, Ti=%.4f s (symmetrical optimum for motor drives)\n\n", Kc, Ti);

    /* Simulation: load torque step at t=0.1s */
    double speed = 0.0, speed_fb_only = 0.0, speed_sp = 100.0;
    double e_int_ff = 0.0, e_int_fb = 0.0;
    double ISE_ff = 0.0, ISE_fb = 0.0;

    printf("Time[s]   Speed(FF)  Speed(FB)  Torque   Voltage(FF)  Error(FF)\n");
    printf("--------  ---------  ---------  -------  -----------  ---------\n");

    for (double t = 0; t <= 0.5; t += Ts) {
        double torque = (t >= 0.1) ? 2.0 : 0.0;

        /* FF+FB */
        double e_ff = speed_sp - speed;
        e_int_ff += e_ff * Ts;
        double u_fb_ff = Kc * (e_ff + e_int_ff / Ti);
        double u_ff = feedforward_step(&ff, torque);
        double u_total = u_fb_ff + u_ff;
        if (u_total > 48.0) u_total = 48.0;
        if (u_total < -48.0) u_total = -48.0;
        double dw = (Km * u_total - torque - B * speed) / J;
        speed += dw * Ts;
        ISE_ff += e_ff * e_ff * Ts;

        /* FB Only */
        double e_fb_only = speed_sp - speed_fb_only;
        e_int_fb += e_fb_only * Ts;
        double u_fb_only = Kc * (e_fb_only + e_int_fb / Ti);
        if (u_fb_only > 48.0) u_fb_only = 48.0;
        if (u_fb_only < -48.0) u_fb_only = -48.0;
        double dw_fb = (Km * u_fb_only - torque - B * speed_fb_only) / J;
        speed_fb_only += dw_fb * Ts;
        ISE_fb += e_fb_only * e_fb_only * Ts;

        if ((int)(t * 1000) % 50 == 0 || (t >= 0.095 && t <= 0.11)) {
            printf("%7.3f   %9.2f  %9.2f  %7.2f  %11.3f  %9.3f\n",
                   t, speed, speed_fb_only, torque, u_total, e_ff);
        }
    }

    printf("\n=== Results ===\n");
    printf("With FF:    ISE = %.6f, Final speed = %.2f rad/s\n", ISE_ff, speed);
    printf("Without FF: ISE = %.6f, Final speed = %.2f rad/s\n", ISE_fb, speed_fb_only);
    printf("ISE reduction: %.1f%%\n", (1.0 - ISE_ff/ISE_fb)*100.0);

    printf("\n=== Key Takeaway ===\n");
    printf("In motor drives, feedforward from load torque provides immediate\n");
    printf("compensation. The feedback PI only corrects residual error from\n");
    printf("model mismatch and unmeasured disturbances (friction variation).\n");
    printf("This is critical for precision assembly (e.g., Toyota production).\n");

    printf("\nExample complete.\n");
    return 0;
}