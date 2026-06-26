/**
 * @file example_heat_exchanger.c
 * @brief L6 Canonical Problem: Heat exchanger temperature control with Smith predictor.
 *
 * Scenario: Shell-and-tube heat exchanger where the temperature sensor is
 * located downstream, introducing a transport delay of 30 seconds.
 * Process: FOPDT with K=0.8 C/percent, tau=60s, theta=30s.
 *
 * The Smith predictor enables aggressive tuning despite the 30s dead time.
 */
#include <stdio.h>
#include <math.h>
#include "smith_predictor.h"
#include "smith_tuning.h"

static double simulate_process(double u, double *y_state, double *buf, int *bi, int d,
                                double K, double tau, double Ts) {
    buf[*bi % (d+10)] = u;
    double ud = buf[(*bi + d + 10 - d) % (d+10)];
    (*bi)++;
    double dy = (K * ud - *y_state) / tau * Ts;
    *y_state += dy;
    return *y_state;
}

int main(void) {
    double K=0.8, tau=60.0, theta=30.0, Ts=2.0;
    double y=25.0, u=50.0;
    double dbuf[50]={0};
    int bi=0, ds=(int)(theta/Ts);

    smith_predictor_t sp;
    smith_predictor_init_fopdt(&sp,K,tau,theta,Ts,SMITH_VARIANT_CLASSIC,0.0,100.0);

    double Kp,Ti;
    smith_tune_simc_pi(&sp.model,tau,&Kp,&Ti);
    smith_predictor_set_pi(&sp,Kp,Ti,1.0);
    smith_predictor_set_robustness_filter(&sp,theta/2.0);

    printf("Heat Exchanger Temperature Control (Smith Predictor)\n");
    printf("Model: K=%.1f, tau=%.0fs, theta=%.0fs\n",K,tau,theta);
    printf("Tuning: Kp=%.3f, Ti=%.1fs\n\n",Kp,Ti);
    printf("Time(s)  Setpoint  Process  Output\n");

    double sp_val=60.0;
    for(int k=0;k<300;k++){
        double t=k*Ts;
        if(t>100.0)sp_val=70.0;
        if(t>500.0)sp_val=55.0;
        u=smith_predictor_step(&sp,sp_val,y);
        y=simulate_process(u,&y,dbuf,&bi,ds,K,tau,Ts);
        if(k%20==0)printf("%6.0f   %6.1f    %6.1f   %6.1f\n",t,sp_val,y,u);
    }
    printf("\nFinal temperature: %.1f C (setpoint: %.1f C)\n",y,sp_val);
    smith_predictor_destroy(&sp);
    return 0;
}
