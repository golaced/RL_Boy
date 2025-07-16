#include "imu.h"
#include "gait_math.h"
#include "arm_math.h"
#include "nav.h"
#include "ms5611.h"
#include "baro_kf.h"
#include "baro_ekf_oldx.h"

#define BARO_EKF

double X_f32[4];
double P_f32[16] =
{
  100.0,     0,        0,       0,
  0,    	   100,      0,       0,
  0,         0,        100,     0,
  0,         0,        0,       100,
};

double Qs_4_4[4] ={0.002,    1e-8,   0.01,   0.00035};
double Rs_2_2[2] ={15, 0.02};

float X_apo_height[2] = {0.0f, 0.0f};
float P_apo_k_height[4] = {100.0f,0.0f,0.0f,100.0f};
float r_baro = 10;
float r_acc =  0.1; 

float FLT_BARO=2;
void baro_fushion(float dt)
{
int i;	
static char init;	
if(!init){
	init=1;
}

	if(ms5611.init_off){

	DigitalLPF((float)ms5611.baroAlt_flt/100., &ms5611.baroAlt_flt1, FLT_BARO, dt);
	double Z_2_1[2] ={(float)ms5611.baroAlt_flt1,vmc_all.acc[Zr] };
	double Z_2_2[2] ={(float)(ms5611.baroAlt-ms5611.baroAlt_off)/100.,vmc_all.acc[Zr] };  
	float Z_baro_ekf[2]={Z_2_1[0],Z_2_1[1]};		
	#if defined(BARO_EKF)
	BARO_EKF_OLDX(X_apo_height,P_apo_k_height, X_apo_height, P_apo_k_height ,Z_baro_ekf,  r_baro,  r_acc, dt);
	nav.pos_n[Zr]=X_apo_height[0];
	nav.spd_b[Zr]=X_apo_height[1];
	#else
	baro_kf(X_f32, P_f32, Z_2_2, ms5611.update, Qs_4_4,Rs_2_2,  dt);
	nav.pos_n[Zr]=X_f32[0];
	nav.spd_n[Zr]=X_f32[1];
	nav.acc_n[Zr]=X_f32[2];
	#endif
	}
}