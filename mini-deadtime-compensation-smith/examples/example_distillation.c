/**
 * @file example_distillation.c
 * @brief L6: Distillation column composition control with Smith predictor.
 *
 * Distillation columns exhibit large dead times due to tray hydraulics
 * and analyzer sampling delays. The Smith predictor compensates for the
 * combined transport + analysis delay.
 *
 * Scenario: Bottom composition control, FOPDT model from step test.
 *   K = -0.5 mol percent / percent reflux  (inverse response)
 *   tau = 120 s, theta = 60 s (analyzer delay)
 */
#include <stdio.h>
#include <math.h>
#include "smith_predictor.h"
#include "smith_tuning.h"
#include "smith_identification.h"

int main(void) {
    double K=-0.5, tau=120.0, theta=60.0, Ts=5.0;
    double y=5.0, u=40.0;
    double dbuf[20]={0};
    int bi=0, ds=(int)(theta/Ts);

    smith_predictor_t sp;
    smith_predictor_init_fopdt(&sp,fabs(K),tau,theta,Ts,
        SMITH_VARIANT_FILTERED,0.0,100.0);

    double Kp,Ti;
    smith_tune_imc_pi(&sp.model,60.0,&Kp,&Ti);
    /* Negative gain: invert Kp sign for correct direction */
    if(K<0.0)Kp=-Kp;
    smith_predictor_set_pi(&sp,Kp,Ti,1.0);
    smith_predictor_set_robustness_filter(&sp,30.0);

    printf("Distillation Bottom Composition Control\n");
    printf("Model: K=%.1f, tau=%.0fs, theta=%.0fs\n",K,tau,theta);
    printf("Tuning: Kp=%.4f, Ti=%.1fs\n\n",Kp,Ti);
    printf("Time(min)  Setpoint  Composition  Reflux\n");

    double sp_val=3.0;
    for(int k=0;k<600;k++){
        double t=k*Ts/60.0;
        if(t>20.0)sp_val=4.5;
        if(t>60.0)sp_val=2.5;
        u=smith_predictor_step(&sp,sp_val,y);
        /* Simulate process */
        dbuf[bi%(ds+5)]=u;
        double ud=dbuf[(bi+ds+5-ds)%(ds+5)]; bi++;
        y+=(fabs(K)*ud-y)/tau*Ts;
        if(k%30==0)printf("%6.1f    %5.1f      %5.2f       %5.1f\n",t,sp_val,y,u);
    }
    printf("\nDeadtime ratio: %.2f\n",smith_deadtime_ratio(&sp.model));
    smith_predictor_destroy(&sp);
    return 0;
}
