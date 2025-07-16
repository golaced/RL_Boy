/****************Doghome ×ËÌ¬½âËã EKF************************/

#ifndef EKF_UPDATE_H
#define EKF_UPDATE_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "EKF_UPDATE_types.h"

/* Function Declarations */
extern void EKF_UPDATE(float Xs[7], float Pk[49], float Xsy_m[2], float P_mag[4],
  float wn, float wbn, float an, float wyn, float wybn, float mn, const float
  measure[10], const float z_flag[4], float att_f[3], float gyro_f[3], float T);
extern void EKF_UPDATE_initialize(void);
extern void EKF_UPDATE_terminate(void);

#endif


