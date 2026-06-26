/**
 * @file example_flow_control.c
 * @brief L6: Pipeline flow control with transport delay.
 *
 * Flow loops with long pipelines exhibit pure transport delay.
 * The Smith predictor handles this by modeling the pipeline as a
 * delay + fast first-order dynamics (valve + flow meter).
 *
 * Scenario: Water flow control through 500m pipeline.
 *   Pipeline delay: theta = L/v = 500m / 2m/s = 250s (dominant!)
 *   Valve + meter: FOPDT K=0.02 (m3/s)/percent, tau=5s
 *
 * The dead-time ratio theta/tau = 250/5 = 50 — extremely difficult.
 * Standard PID would need Kp < 0.1, extremely slow response.
 * Smith predictor enables Kp ≈ 5.0 with Ti ≈ 5s.
 */
#include <stdio.h>
#include <math.h>
#include "smith_predictor.h"
#include "smith_tuning.h"

int main(void) {
    double K=0.02, tau=5.0, theta=250.0, Ts=10.0;
    double y=0.0, u=30.0;
    double dbuf[30]={0};
    int bi=0, ds=(int)(theta/Ts);

    smith_predictor_t sp;
    smith_predictor_init_fopdt(&sp,K,tau,theta,Ts,
        SMITH_VARIANT_TWO_DOF,0.0,100.0);

    double Kp,Ti;
    smith_tune_simc_pi(&sp.model,10.0,&Kp,&Ti);
    smith_predictor_set_pi(&sp,Kp,Ti,0.5);
    smith_predictor_set_robustness_filter(&sp,theta/3.0);

    printf("Pipeline Flow Control (500m, v=2m/s)\n");
    printf("Delay: %.0fs, Time constant: %.0fs, Ratio: %.0f\n",
           theta,tau,theta/tau);
    printf("SIMC Tuning: Kp=%.3f, Ti=%.1fs\n\n",Kp,Ti);
    printf("Time(s)  SP(m3/s)  Flow(m3/s)  Valve(percent)\n");

    double sp_val=0.5;
    for(int k=0;k<400;k++){
        double t=k*Ts;
        if(t>500.0)sp_val=0.8;
        u=smith_predictor_step(&sp,sp_val,y);
        dbuf[bi%(ds+5)]=u;
        double ud=dbuf[(bi+ds+5-ds)%(ds+5)]; bi++;
        y+=(K*ud-y)/tau*Ts;
        if(k%25==0)printf("%7.0f  %7.2f   %7.3f     %6.1f\n",t,sp_val,y,u);
    }
    smith_predictor_destroy(&sp);
    return 0;
}
