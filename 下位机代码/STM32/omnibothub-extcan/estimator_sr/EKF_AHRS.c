
#include "EKF_UPDATE.h"
#include "EKF_AHRS.h"

 _AHRS_EKF ahrs_ekf;

void EKF_AHRS_INIT(void)
{
	int i=0;
	ahrs_ekf.Xss[0]=1;ahrs_ekf.Xss[1]=ahrs_ekf.Xss[2]=ahrs_ekf.Xss[3]=0;
	ahrs_ekf.Xss[4]=ahrs_ekf.Xss[5]=ahrs_ekf.Xss[6]=0;
	ahrs_ekf.Xssy_m[0]=ahrs_ekf.Xssy_m[1]=0;
	
	for(i=0;i<49;i++)
		ahrs_ekf.Pk[i]=0;
	
	ahrs_ekf.Pk[0]=ahrs_ekf.Pk[6]=ahrs_ekf.Pk[6*2]=ahrs_ekf.Pk[6*3]=ahrs_ekf.Pk[6*4]=ahrs_ekf.Pk[6*5]=ahrs_ekf.Pk[6*6]=ahrs_ekf.Pk[6*7];
	
	for(i=0;i<4;i++)
		ahrs_ekf.P_mag[i]=0;
	ahrs_ekf.P_mag[0]=ahrs_ekf.P_mag[3]=1;
	
	ahrs_ekf.m_flag[0]=1;
	ahrs_ekf.m_flag[1]=1;
	ahrs_ekf.m_flag[2]=0;
	ahrs_ekf.m_flag[3]=0;
	
	ahrs_ekf.wn=1e-6;//ÍÓÂÝÒÇÔëÉù
	ahrs_ekf.wbn=1e-8;//ÍÓÂÝÆ«²îÔëÉù
	ahrs_ekf.an=0.1;//1e-1;//¼ÓËÙ¶È²âÁ¿ÔëÉù
	ahrs_ekf.wyn=ahrs_ekf.wn; 
	ahrs_ekf.wybn=ahrs_ekf.wbn;
  ahrs_ekf.mn=1e-3;//´Å³¡²âÁ¿ÔëÉù
}

void EKF_AHRS_UPDATE(float dT)
{

	float measure[10]={0};
	if(!ahrs_ekf.init){ahrs_ekf.init=1;
		EKF_AHRS_INIT();
	}	
	
	float z_flag[4]={ahrs_ekf.m_flag[0],//gyro
									 ahrs_ekf.m_flag[1],//acc
									 ahrs_ekf.m_flag[2],//mag
									 ahrs_ekf.m_flag[3]};//yaw out
									 
	measure[0]=ahrs_ekf.acc_m[0];//acc
	measure[1]=ahrs_ekf.acc_m[1];
  measure[2]=ahrs_ekf.acc_m[2];									 
							//gyro
	measure[3]=ahrs_ekf.gyro_m[0];
	measure[4]=ahrs_ekf.gyro_m[1];
	measure[5]=ahrs_ekf.gyro_m[2];									 									 
							//mag								 
	measure[6]=ahrs_ekf.mag_m[0];
	measure[7]=ahrs_ekf.mag_m[1];
	measure[8]=ahrs_ekf.mag_m[2];									 
							//yawout
	measure[9]=ahrs_ekf.yaw_out;	
									 
  EKF_UPDATE(ahrs_ekf.Xss, ahrs_ekf.Pk, ahrs_ekf.Xssy_m, ahrs_ekf.P_mag, 
						 ahrs_ekf.wn, ahrs_ekf.wbn, ahrs_ekf.an, ahrs_ekf.wyn, ahrs_ekf.wybn, ahrs_ekf.mn, 
						 measure, z_flag, ahrs_ekf.att_f, ahrs_ekf.gyro_f, dT);
									 
	ahrs_ekf.att_f_use[0]= ahrs_ekf.att_f[0];
	ahrs_ekf.att_f_use[1]=-ahrs_ekf.att_f[1];
	ahrs_ekf.att_f_use[2]=-ahrs_ekf.att_f[2];
	ahrs_ekf.gyro_f_use[0]= ahrs_ekf.gyro_f[0];
	ahrs_ekf.gyro_f_use[1]= ahrs_ekf.gyro_f[1];
	ahrs_ekf.gyro_f_use[2]= ahrs_ekf.gyro_f[2];
}
