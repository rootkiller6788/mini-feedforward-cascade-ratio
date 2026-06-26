/** @file test_smith.c - Master test runner for Smith predictor module. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "smith_predictor.h"
#include "smith_tuning.h"
#include "smith_identification.h"
#include "smith_robustness.h"
#include "smith_adaptation.h"

#define EPS 1e-6
static int passed=0,total=0;
static void chk(int c,const char*m){total++;if(c){passed++;printf("  PASS: %s\n",m);}else{printf("  FAIL: %s\n",m);}}

int main(void){
printf("=== Smith Predictor Module Tests ===\n\n");

{smith_predictor_t sp;
 chk(smith_predictor_init_fopdt(&sp,1.0,10.0,5.0,1.0,SMITH_VARIANT_CLASSIC,0.0,100.0)==0,"L1 init_fopdt");
 chk(sp.model.order==SMITH_MODEL_FOPDT,"L1 model order");
 chk(smith_predictor_init_fopdt(&sp,1.0,-1.0,5.0,1.0,SMITH_VARIANT_CLASSIC,0.0,100.0)==-1,"L1 reject neg tau");
 smith_predictor_destroy(&sp);}
{smith_predictor_t sp;
 chk(smith_predictor_init_sopdt(&sp,1.0,5.0,2.0,0.0,0.0,3.0,1.0,SMITH_VARIANT_CLASSIC,0.0,100.0)==0,"L1 init_sopdt");
 smith_predictor_destroy(&sp);}

{smith_predictor_t sp;
 smith_predictor_init_fopdt(&sp,1.0,10.0,5.0,1.0,SMITH_VARIANT_CLASSIC,0.0,100.0);
 chk(smith_predictor_set_pi(&sp,2.0,10.0,0.5)==0,"L2 set_pi");
 chk(fabs(sp.Kp-2.0)<EPS,"L2 Kp set");
 chk(smith_predictor_set_pid(&sp,1.5,8.0,1.0,10.0,0.7,0.0)==0,"L2 set_pid");
 chk(smith_predictor_set_robustness_filter(&sp,2.5)==0,"L2 robustness filter");
 smith_predictor_destroy(&sp);}

{smith_predictor_t sp;
 smith_predictor_init_fopdt(&sp,1.0,10.0,5.0,1.0,SMITH_VARIANT_CLASSIC,0.0,100.0);
 smith_predictor_set_pi(&sp,0.5,10.0,1.0);
 double u=smith_predictor_step(&sp,1.0,0.0);
 chk(u>0.0,"L3 step output >0"); chk(u<=100.0,"L3 output <=100");
 smith_predictor_t sp2;
 smith_predictor_init_fopdt(&sp2,1.0,10.0,5.0,1.0,SMITH_VARIANT_CLASSIC,20.0,80.0);
 smith_predictor_set_pi(&sp2,100.0,1.0,1.0);
 double u2=smith_predictor_step(&sp2,100.0,0.0);
 chk(u2<=80.0,"L3 saturation clamp");
 smith_predictor_reset(&sp);
 chk(fabs(sp.integrator)<EPS,"L3 reset integrator");
 smith_predictor_destroy(&sp);smith_predictor_destroy(&sp2);}

{smith_predictor_t sp;
 smith_predictor_init_fopdt(&sp,1.0,10.0,5.0,1.0,SMITH_VARIANT_CLASSIC,0.0,100.0);
 double Kp,Ti;
 chk(smith_tune_simc_pi(&sp.model,5.0,&Kp,&Ti)==0,"L4 SIMC PI");
 chk(Kp>0.0&&Ti>0.0,"L4 SIMC gains");
 chk(smith_tune_imc_pi(&sp.model,5.0,&Kp,&Ti)==0,"L4 IMC PI");
 chk(fabs(Kp-2.0)<0.2,"L4 IMC Kp=tau/(K*lambda)");
 double gm,pm;
 chk(smith_tune_stability_margins(&sp.model,0.5,10.0,0.0,&gm,&pm)==0,"L4 margins");
 chk(gm>0.0,"L4 GM>0");
 chk(smith_tune_is_stable(0.5,10.0,&sp.model)==1,"L4 is_stable");
 smith_predictor_destroy(&sp);}

{int n=200;double*t=malloc(n*8);double*y=malloc(n*8);double*u=malloc(n*8);
 for(int i=0;i<n;i++){t[i]=i*0.1;u[i]=1.0;y[i]=(t[i]<1.0)?0.0:(1.0*(1.0-exp(-(t[i]-1.0)/2.0)));}
 smith_step_test_t d={t,y,u,n,1.0,0.0,1.0};smith_process_model_t m;
 chk(smith_identify_step_fopdt(&d,&m)==0,"L5 identify FOPDT");
 chk(fabs(m.fopdt.K-1.0)<0.3,"L5 K est"); double fit;
 m.fopdt.K=1.0;m.fopdt.tau=2.0;m.fopdt.theta=1.0;
 chk(smith_validate_model_fit(&d,&m,&fit)==0,"L5 validate");chk(fit>70.0,"L5 fit>70");
 free(t);free(y);free(u);
 smith_rls_identifier_t r;smith_rls_init(&r,0.98,1.0,2.0,1.0);
 double a=exp(-0.1/2.0),b=1.0*(1.0-a),yv=0.0,uv=1.0;
 for(int i=0;i<200;i++){double yn=a*yv+b*uv;smith_rls_update(&r,uv,yn,0.1);yv=yn;}
 chk(smith_rls_to_fopdt(&r,&m)==0,"L5 RLS");
 n=100;t=malloc(n*8);y=malloc(n*8);
 for(int i=0;i<n;i++){t[i]=i*0.1;y[i]=(t[i]<2.5)?0.0:(1.0*(1.0-exp(-(t[i]-2.5)/3.0)));}
 double th;chk(smith_estimate_deadtime(t,y,n,0.02,&th)==0,"L5 deadtime");
 chk(th>=2.0&&th<=3.0,"L5 theta");free(t);free(y);
 smith_process_model_t mr;mr.order=SMITH_MODEL_FOPDT;
 mr.fopdt.K=1.0;mr.fopdt.tau=10.0;mr.fopdt.theta=5.0;
 chk(fabs(smith_deadtime_ratio(&mr)-0.5)<EPS,"L5 ratio");}

{smith_predictor_t sp;
 smith_predictor_init_fopdt(&sp,1.0,10.0,5.0,1.0,SMITH_VARIANT_CLASSIC,0.0,100.0);
 double S=smith_robustness_sensitivity(&sp.model,2.0,10.0,0.1);
 chk(S>0.0&&S<10.0,"L4 sensitivity");
 double Ms=smith_robustness_peak_sensitivity(&sp.model,2.0,10.0,0.0,0.001,100.0,200);
 chk(Ms>=1.0&&Ms<10.0,"L4 Ms");
 double dm=smith_robustness_delay_margin(&sp.model,2.0,10.0,0.0);
 chk(dm>0.0,"L4 delay margin");
 int sc;smith_robustness_monte_carlo(&sp.model,2.0,10.0,0.1,0.1,0.5,100,&sc);
 chk(sc>80,"L8 Monte Carlo");
 smith_predictor_destroy(&sp);}

{smith_predictor_t sp;
 smith_predictor_init_fopdt(&sp,1.0,2.0,3.0,0.5,SMITH_VARIANT_CLASSIC,0.0,100.0);
 smith_predictor_set_pi(&sp,0.5,2.0,1.0);
 double y=0.0,db[10]={0};int di=0,ds=6;
 for(int k=0;k<200;k++){double u=smith_predictor_step(&sp,1.0,y);
  db[di%10]=u;double ud=db[(di+10-ds)%10];di++;y+=(1.0*ud-y)/2.0*0.5;}
 chk(fabs(y-1.0)<0.2,"L6 closed-loop convergence");
 smith_predictor_destroy(&sp);}

{smith_predictor_t sp;
 smith_predictor_init_fopdt(&sp,1.0,10.0,5.0,1.0,SMITH_VARIANT_CLASSIC,0.0,100.0);
 smith_modbus_map_t mb;smith_predictor_map_modbus(&sp,&mb,100);
 chk(mb.Kp_reg==100,"L7 Modbus reg");
 smith_opcua_map_t ua;smith_predictor_map_opcua(&sp,&ua,2,1000);
 chk(ua.namespace_idx==2,"L7 OPC UA ns");
 smith_predictor_destroy(&sp);}

{smith_adaptive_t adp;
 chk(smith_adaptive_init(&adp,1.0,10.0,5.0,1.0,SMITH_VARIANT_CLASSIC,0.0,100.0)==0,"L8 adaptive init");
 double u=smith_adaptive_step(&adp,1.0,0.0);
 chk(u>0.0,"L8 adaptive step");
 smith_adaptive_destroy(&adp);}

{smith_predictor_t sp;
 smith_predictor_init_fopdt(&sp,1.0,10.0,5.0,1.0,SMITH_VARIANT_CLASSIC,0.0,100.0);
 smith_predictor_set_pi(&sp,0.5,10.0,1.0);smith_predictor_step(&sp,1.0,0.0);
 chk(isfinite(smith_predictor_get_prediction(&sp)),"L5 prediction");
 chk(isfinite(smith_predictor_get_mismatch(&sp)),"L5 mismatch");
 smith_performance_t perf;smith_predictor_compute_performance(&sp,&perf);
 chk(perf.setpoint_IAE>=0.0,"L6 IAE");
 smith_predictor_destroy(&sp);}

printf("\n=== %d/%d passed ===\n",passed,total);
return (passed==total)?0:1;}
