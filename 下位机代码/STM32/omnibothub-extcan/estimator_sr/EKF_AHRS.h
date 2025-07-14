/****************Doghome ×ËÌ¬½âËã EKF************************/
#ifndef EKF_AHRS_H
#define EKF_AHRS_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "EKF_UPDATE_types.h"

typedef struct 
{
	char init;
	float Xss[7];
	float Pk[49];
	float Xssy_m[2];
	float P_mag[4];
	float wn;
	float wbn;
	float an;
	float wyn;
	float wybn;
	float mn;
	float att_f[3];
	float att_f_use[3];
	float gyro_f[3];
	float gyro_f_use[3];
	float acc_m[3];
	float gyro_m[3];
	float mag_m[3];
	float yaw_out;
	float m_flag[4];
}_AHRS_EKF;

extern _AHRS_EKF ahrs_ekf;

void EKF_AHRS_INIT(void);
void EKF_AHRS_UPDATE(float dT);

#endif

/*
 * File trailer for main.h
 *
 * [EOF]
 */
