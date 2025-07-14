/****************Doghome 姿态解算 EKF************************/

#include <math.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <string.h>
#include "EKF_UPDATE.h"

/* Function Declarations */
static float b_norm(const float x[4]);
static float norm(const float x[3]);
static float rt_atan2f_snf(float u0, float u1);

/* Function Definitions */

/*
 * Arguments    : const float x[4]
 * Return Type  : float
 */
static float b_norm(const float x[4])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.29246971E-26F;
  for (k = 0; k < 4; k++) {
    absxk = (float)fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (float)sqrt(y);
}

/*
 * Arguments    : const float x[3]
 * Return Type  : float
 */
static float norm(const float x[3])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.29246971E-26F;
  for (k = 0; k < 3; k++) {
    absxk = (float)fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (float)sqrt(y);
}

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_atan2f_snf(float u0, float u1)
{
  float y;
  int b_u0;
  int b_u1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0F) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = (float)atan2((float)b_u0, (float)b_u1);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (float)atan2(u0, u1);
  }

  return y;
}

/*
 * function [Xs,Pk,Xsy_m,P_mag,att_f,gyro_f]=EKF_UPDATE(Xs,Pk,Xsy_m,P_mag,wn,wbn,an,wyn,wybn,mn,measure,z_flag,att_f,gyro_f,T)
 * wn_var  = 1e-6 * ones(1,4);
 *      wbn_var = 1e-8 * ones(1,3);
 *      an_var  = 1e-1 * ones(1,3);
 *      Q = diag([wn_var, wbn_var]);
 *      R = diag(an_var);
 *      Q_mag = diag([1e-6,1e-8]);
 *      R_mag= 1e-3;
 * 1e-6,1e-8,1e-1,1e-6,1e-8,1e-3
 * Arguments    : float Xs[7]
 *                float Pk[49]
 *                float Xsy_m[2]
 *                float P_mag[4]
 *                float wn
 *                float wbn
 *                float an
 *                float wyn
 *                float wybn
 *                float mn
 *                const float measure[10]
 *                const float z_flag[4]
 *                float att_f[3]
 *                float gyro_f[3]
 *                float T
 * Return Type  : void
 */
void EKF_UPDATE(float Xs[7], float Pk[49], float Xsy_m[2], float P_mag[4], float
                wn, float wbn, float an, float wyn, float wybn, float mn, const
                float measure[10], const float z_flag[4], float att_f[3], float
                gyro_f[3], float T)
{
  int p1;
  float an_var[3];
  float v[7];
  float Q[49];
  float R[9];
  float x_mag_[2];
  float Q_mag[4];
  float value;
  float gyro_x_bias;
  float gyro_y_bias;
  float c;
  float b_c;
  float c_c;
  float d_c;
  float absx21;
  float absx31;
  float e_c;
  float f_c[49];
  int p2;
  float Ak[49];
  static const signed char iv0[49] = { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 1 };

  float fv2[7];
  static const signed char iv1[21] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 1 };

  int p3;
  float Pk_[49];
  float Hk[21];
  float g_c[9];
  float b_Hk[21];
  int itmp;
  float Kk[21];
  float fv3[3];
  float q[4];
  static const signed char iv2[3] = { 6, 7, 8 };

  float b_q[4];
  static const signed char iv3[3] = { 0, 0, 1 };

  float fv4[3];
  float P_22[4];
  float b_P_22[2];
  float K_2[2];
  static const signed char iv4[4] = { 1, 0, 0, 1 };

  /* 'EKF_UPDATE:11' pitchEKF=0; */
  /* 'EKF_UPDATE:12' rollEKF=0; */
  /* 'EKF_UPDATE:13' yawEKF=0; */
  /* 'EKF_UPDATE:14' z_2=0; */
  /* 'EKF_UPDATE:15' acc_m=measure(1:3); */
  /* 'EKF_UPDATE:16' gyro_m=measure(4:6); */
  /* 'EKF_UPDATE:17' mag_m=measure(7:9); */
  /* 'EKF_UPDATE:18' yaw_out=measure(10); */
  /* 'EKF_UPDATE:20' wn_var  = wn * ones(1,4); */
  /* 'EKF_UPDATE:21' wbn_var = wbn * ones(1,3); */
  /* 'EKF_UPDATE:22' an_var  = an * ones(1,3); */
  for (p1 = 0; p1 < 3; p1++) {
    an_var[p1] = an;
  }

  /* 'EKF_UPDATE:24' Q = diag([wn_var, wbn_var]); */
  for (p1 = 0; p1 < 4; p1++) {
    v[p1] = wn;
  }

  for (p1 = 0; p1 < 3; p1++) {
    v[p1 + 4] = wbn;
  }

  memset(&Q[0], 0, 49U * sizeof(float));
  for (p1 = 0; p1 < 7; p1++) {
    Q[p1 + 7 * p1] = v[p1];
  }

  /* 'EKF_UPDATE:25' R = diag(an_var); */
  for (p1 = 0; p1 < 9; p1++) {
    R[p1] = 0.0F;
  }

  for (p1 = 0; p1 < 3; p1++) {
    R[p1 + 3 * p1] = an_var[p1];
  }

  /* 'EKF_UPDATE:27' Q_mag = diag([wyn,wybn]); */
  x_mag_[0] = wyn;
  x_mag_[1] = wybn;
  for (p1 = 0; p1 < 4; p1++) {
    Q_mag[p1] = 0.0F;
  }

  for (p1 = 0; p1 < 2; p1++) {
    Q_mag[p1 + (p1 << 1)] = x_mag_[p1];
  }

  /* 'EKF_UPDATE:28' R_mag = mn; */
  /* 'EKF_UPDATE:30' acc = acc_m/norm(acc_m,2); */
  value = norm(*(float (*)[3])&measure[0]);

  /* 'EKF_UPDATE:31' z = acc; */
  /* 'EKF_UPDATE:33' x=Xs; */
  /* 获取外部状态 */
  /* 'EKF_UPDATE:34' yaw_mag=Xsy_m(1); */
  /* 'EKF_UPDATE:35' z_bias=Xsy_m(2); */
  /* 'EKF_UPDATE:37' gyro_x_bias = gyro_m(1)-x(5); */
  gyro_x_bias = measure[3] - Xs[4];

  /* 'EKF_UPDATE:38' gyro_y_bias = gyro_m(2)-x(6); */
  gyro_y_bias = measure[4] - Xs[5];

  /* 'EKF_UPDATE:39' gyro_z_bias = gyro_m(3); */
  /* 'EKF_UPDATE:41' A_11=[1,-(T/2)*gyro_x_bias,-(T/2)*gyro_y_bias,-(T/2)*gyro_z_bias; */
  /* 'EKF_UPDATE:42'             (T/2)*gyro_x_bias,1,(T/2)*gyro_z_bias,-(T/2)*gyro_y_bias; */
  /* 'EKF_UPDATE:43'             (T/2)*gyro_y_bias,-(T/2)*gyro_z_bias,1,(T/2)*gyro_x_bias; */
  /* 'EKF_UPDATE:44'            (T/2)*gyro_z_bias,(T/2)*gyro_y_bias,-(T/2)*gyro_x_bias,1]; */
  c = T / 2.0F;
  b_c = T / 2.0F;
  c_c = T / 2.0F;
  d_c = T / 2.0F;
  absx21 = T / 2.0F;
  absx31 = T / 2.0F;

  /* 'EKF_UPDATE:46' A_12=[0,0,0; */
  /* 'EKF_UPDATE:47'           0,0,0; */
  /* 'EKF_UPDATE:48'           0,0,0; */
  /* 'EKF_UPDATE:49'           0,0,0]; */
  /* 'EKF_UPDATE:50' A_21=[0,0,0,0; */
  /* 'EKF_UPDATE:51'           0,0,0,0; */
  /* 'EKF_UPDATE:52'           0,0,0,0]; */
  /* 'EKF_UPDATE:53' A_22=[1,0,0; */
  /* 'EKF_UPDATE:54'           0,1,0; */
  /* 'EKF_UPDATE:55'           0,0,1]; */
  /* 'EKF_UPDATE:56' A=[A_11,A_12;A_21,A_22]; */
  /* 'EKF_UPDATE:58' Ak = eye(7)+T/2*... */
  /* 'EKF_UPDATE:59'          [0    -(gyro_x_bias)  -(gyro_y_bias) -(gyro_z_bias)   x(2) x(3)  x(4); */
  /* 'EKF_UPDATE:60'           (gyro_x_bias) 0    (gyro_z_bias)  -(gyro_y_bias)   -x(1) x(4)  -x(3); */
  /* 'EKF_UPDATE:61'           (gyro_y_bias) -(gyro_z_bias)  0  (gyro_x_bias)      -x(4) -x(1) x(2); */
  /* 'EKF_UPDATE:62'           (gyro_z_bias) (gyro_y_bias)   -(gyro_x_bias)  0     x(3) -x(2) -x(1); */
  /* 'EKF_UPDATE:63'           0 0 0 0 0 0 0; */
  /* 'EKF_UPDATE:64'           0 0 0 0 0 0 0; */
  /* 'EKF_UPDATE:65'           0 0 0 0 0 0 0]; */
  e_c = T / 2.0F;
  f_c[0] = e_c * 0.0F;
  f_c[7] = e_c * -gyro_x_bias;
  f_c[14] = e_c * -gyro_y_bias;
  f_c[21] = e_c * -measure[5];
  f_c[28] = e_c * Xs[1];
  f_c[35] = e_c * Xs[2];
  f_c[42] = e_c * Xs[3];
  f_c[1] = e_c * gyro_x_bias;
  f_c[8] = e_c * 0.0F;
  f_c[15] = e_c * measure[5];
  f_c[22] = e_c * -gyro_y_bias;
  f_c[29] = e_c * -Xs[0];
  f_c[36] = e_c * Xs[3];
  f_c[43] = e_c * -Xs[2];
  f_c[2] = e_c * gyro_y_bias;
  f_c[9] = e_c * -measure[5];
  f_c[16] = e_c * 0.0F;
  f_c[23] = e_c * gyro_x_bias;
  f_c[30] = e_c * -Xs[3];
  f_c[37] = e_c * -Xs[0];
  f_c[44] = e_c * Xs[1];
  f_c[3] = e_c * measure[5];
  f_c[10] = e_c * gyro_y_bias;
  f_c[17] = e_c * -gyro_x_bias;
  f_c[24] = e_c * 0.0F;
  f_c[31] = e_c * Xs[2];
  f_c[38] = e_c * -Xs[1];
  f_c[45] = e_c * -Xs[0];
  for (p1 = 0; p1 < 7; p1++) {
    f_c[4 + 7 * p1] = e_c * 0.0F;
    f_c[5 + 7 * p1] = e_c * 0.0F;
    f_c[6 + 7 * p1] = e_c * 0.0F;
    for (p2 = 0; p2 < 7; p2++) {
      Ak[p2 + 7 * p1] = (float)iv0[p2 + 7 * p1] + f_c[p2 + 7 * p1];
    }
  }

  /* 状态预测 */
  /* 'EKF_UPDATE:67' x_ = (A*x')'; */
  f_c[0] = 1.0F;
  f_c[7] = -(T / 2.0F) * gyro_x_bias;
  f_c[14] = -(T / 2.0F) * gyro_y_bias;
  f_c[21] = -(T / 2.0F) * measure[5];
  f_c[1] = c * gyro_x_bias;
  f_c[8] = 1.0F;
  f_c[15] = b_c * measure[5];
  f_c[22] = -(T / 2.0F) * gyro_y_bias;
  f_c[2] = c_c * gyro_y_bias;
  f_c[9] = -(T / 2.0F) * measure[5];
  f_c[16] = 1.0F;
  f_c[23] = d_c * gyro_x_bias;
  f_c[3] = absx21 * measure[5];
  f_c[10] = absx31 * gyro_y_bias;
  f_c[17] = -(T / 2.0F) * gyro_x_bias;
  f_c[24] = 1.0F;
  for (p1 = 0; p1 < 3; p1++) {
    for (p2 = 0; p2 < 4; p2++) {
      f_c[p2 + 7 * (p1 + 4)] = 0.0F;
    }
  }

  for (p1 = 0; p1 < 7; p1++) {
    for (p2 = 0; p2 < 3; p2++) {
      f_c[(p2 + 7 * p1) + 4] = iv1[p2 + 3 * p1];
    }
  }

  for (p1 = 0; p1 < 7; p1++) {
    fv2[p1] = 0.0F;
    for (p2 = 0; p2 < 7; p2++) {
      fv2[p1] += f_c[p1 + 7 * p2] * Xs[p2];
    }

    v[p1] = fv2[p1];
  }

  f_c[0] = 1.0F;
  f_c[7] = -(T / 2.0F) * gyro_x_bias;
  f_c[14] = -(T / 2.0F) * gyro_y_bias;
  f_c[21] = -(T / 2.0F) * measure[5];
  f_c[1] = c * gyro_x_bias;
  f_c[8] = 1.0F;
  f_c[15] = b_c * measure[5];
  f_c[22] = -(T / 2.0F) * gyro_y_bias;
  f_c[2] = c_c * gyro_y_bias;
  f_c[9] = -(T / 2.0F) * measure[5];
  f_c[16] = 1.0F;
  f_c[23] = d_c * gyro_x_bias;
  f_c[3] = absx21 * measure[5];
  f_c[10] = absx31 * gyro_y_bias;
  f_c[17] = -(T / 2.0F) * gyro_x_bias;
  f_c[24] = 1.0F;
  for (p1 = 0; p1 < 3; p1++) {
    for (p2 = 0; p2 < 4; p2++) {
      f_c[p2 + 7 * (p1 + 4)] = 0.0F;
    }
  }

  for (p1 = 0; p1 < 7; p1++) {
    for (p2 = 0; p2 < 3; p2++) {
      f_c[(p2 + 7 * p1) + 4] = iv1[p2 + 3 * p1];
    }
  }

  for (p1 = 0; p1 < 7; p1++) {
    fv2[p1] = 0.0F;
    for (p2 = 0; p2 < 7; p2++) {
      fv2[p1] += f_c[p1 + 7 * p2] * Xs[p2];
    }
  }

  for (p1 = 0; p1 < 7; p1++) {
    Xs[p1] = fv2[p1];
  }

  /* 四元数限幅度 */
  /* 'EKF_UPDATE:68' x_(1:4) = x_(1:4)/norm(x_(1:4),2); */
  gyro_y_bias = b_norm(*(float (*)[4])&v[0]);
  for (p1 = 0; p1 < 4; p1++) {
    Xs[p1] = v[p1] / gyro_y_bias;
  }

  /* 'EKF_UPDATE:70' Pk_ = Ak * Pk(:,:) * Ak' + Q; */
  for (p1 = 0; p1 < 7; p1++) {
    for (p2 = 0; p2 < 7; p2++) {
      f_c[p1 + 7 * p2] = 0.0F;
      for (p3 = 0; p3 < 7; p3++) {
        f_c[p1 + 7 * p2] += Ak[p1 + 7 * p3] * Pk[p3 + 7 * p2];
      }
    }

    for (p2 = 0; p2 < 7; p2++) {
      gyro_x_bias = 0.0F;
      for (p3 = 0; p3 < 7; p3++) {
        gyro_x_bias += f_c[p1 + 7 * p3] * Ak[p2 + 7 * p3];
      }

      Pk_[p1 + 7 * p2] = gyro_x_bias + Q[p1 + 7 * p2];
    }
  }

  /* 计算协方差 */
  /* 'EKF_UPDATE:71' hk = [2*(x_(2)*x_(4)-x_(1)*x_(3)) 2*(x_(1)*x_(2)+x_(3)*x_(4)) x_(1)^2+x_(4)^2-x_(2)^2-x_(3)^2]; */
  c = Xs[1] * Xs[3] - Xs[0] * Xs[2];
  b_c = Xs[0] * Xs[1] + Xs[2] * Xs[3];
  c_c = (Xs[0] * Xs[0] + Xs[3] * Xs[3]) - Xs[1] * Xs[1];
  d_c = Xs[2] * Xs[2];

  /* 'EKF_UPDATE:72' Hk = 2*[  -x_(3)  x_(4) -x_(1)  x_(2) 0 0 0; */
  /* 'EKF_UPDATE:73'                x_(2)  x_(1)  x_(4)  x_(3) 0 0 0; */
  /* 'EKF_UPDATE:74'                x_(1) -x_(2) -x_(3)  x_(4) 0 0 0]; */
  Hk[0] = 2.0F * -Xs[2];
  Hk[3] = 2.0F * Xs[3];
  Hk[6] = 2.0F * -Xs[0];
  Hk[9] = 2.0F * Xs[1];
  Hk[12] = 0.0F;
  Hk[15] = 0.0F;
  Hk[18] = 0.0F;
  Hk[1] = 2.0F * Xs[1];
  Hk[4] = 2.0F * Xs[0];
  Hk[7] = 2.0F * Xs[3];
  Hk[10] = 2.0F * Xs[2];
  Hk[13] = 0.0F;
  Hk[16] = 0.0F;
  Hk[19] = 0.0F;
  Hk[2] = 2.0F * Xs[0];
  Hk[5] = 2.0F * -Xs[1];
  Hk[8] = 2.0F * -Xs[2];
  Hk[11] = 2.0F * Xs[3];
  Hk[14] = 0.0F;
  Hk[17] = 0.0F;
  Hk[20] = 0.0F;

  /* 观测方程 */
  /* 'EKF_UPDATE:76' Kk = Pk_ * Hk' * ((Hk * Pk_ * Hk' + R)^(-1)); */
  for (p1 = 0; p1 < 3; p1++) {
    for (p2 = 0; p2 < 7; p2++) {
      b_Hk[p1 + 3 * p2] = 0.0F;
      for (p3 = 0; p3 < 7; p3++) {
        b_Hk[p1 + 3 * p2] += Hk[p1 + 3 * p3] * Pk_[p3 + 7 * p2];
      }
    }

    for (p2 = 0; p2 < 3; p2++) {
      gyro_x_bias = 0.0F;
      for (p3 = 0; p3 < 7; p3++) {
        gyro_x_bias += b_Hk[p1 + 3 * p3] * Hk[p2 + 3 * p3];
      }

      g_c[p1 + 3 * p2] = gyro_x_bias + R[p1 + 3 * p2];
    }
  }

  for (p1 = 0; p1 < 9; p1++) {
    R[p1] = g_c[p1];
  }

  p1 = 0;
  p2 = 3;
  p3 = 6;
  gyro_y_bias = (float)fabs(g_c[0]);
  absx21 = (float)fabs(g_c[1]);
  absx31 = (float)fabs(g_c[2]);
  if ((absx21 > gyro_y_bias) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    R[0] = g_c[1];
    R[1] = g_c[0];
    R[3] = g_c[4];
    R[4] = g_c[3];
    R[6] = g_c[7];
    R[7] = g_c[6];
  } else {
    if (absx31 > gyro_y_bias) {
      p1 = 6;
      p3 = 0;
      R[0] = g_c[2];
      R[2] = g_c[0];
      R[3] = g_c[5];
      R[5] = g_c[3];
      R[6] = g_c[8];
      R[8] = g_c[6];
    }
  }

  gyro_x_bias = R[1] / R[0];
  R[1] /= R[0];
  absx31 = R[2] / R[0];
  R[2] /= R[0];
  R[4] -= gyro_x_bias * R[3];
  R[5] -= absx31 * R[3];
  R[7] -= gyro_x_bias * R[6];
  R[8] -= absx31 * R[6];
  if ((float)fabs(R[5]) > (float)fabs(R[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    R[1] = absx31;
    R[2] = gyro_x_bias;
    gyro_y_bias = R[4];
    R[4] = R[5];
    R[5] = gyro_y_bias;
    gyro_y_bias = R[7];
    R[7] = R[8];
    R[8] = gyro_y_bias;
  }

  gyro_x_bias = R[5] / R[4];
  R[5] /= R[4];
  R[8] -= gyro_x_bias * R[7];
  gyro_x_bias = (R[5] * R[1] - R[2]) / R[8];
  gyro_y_bias = -(R[1] + R[7] * gyro_x_bias) / R[4];
  g_c[p1] = ((1.0F - R[3] * gyro_y_bias) - R[6] * gyro_x_bias) / R[0];
  g_c[p1 + 1] = gyro_y_bias;
  g_c[p1 + 2] = gyro_x_bias;
  gyro_x_bias = -R[5] / R[8];
  gyro_y_bias = (1.0F - R[7] * gyro_x_bias) / R[4];
  g_c[p2] = -(R[3] * gyro_y_bias + R[6] * gyro_x_bias) / R[0];
  g_c[p2 + 1] = gyro_y_bias;
  g_c[p2 + 2] = gyro_x_bias;
  gyro_x_bias = 1.0F / R[8];
  gyro_y_bias = -R[7] * gyro_x_bias / R[4];
  g_c[p3] = -(R[3] * gyro_y_bias + R[6] * gyro_x_bias) / R[0];
  g_c[p3 + 1] = gyro_y_bias;
  g_c[p3 + 2] = gyro_x_bias;
  for (p1 = 0; p1 < 7; p1++) {
    for (p2 = 0; p2 < 3; p2++) {
      b_Hk[p1 + 7 * p2] = 0.0F;
      for (p3 = 0; p3 < 7; p3++) {
        b_Hk[p1 + 7 * p2] += Pk_[p1 + 7 * p3] * Hk[p2 + 3 * p3];
      }
    }

    for (p2 = 0; p2 < 3; p2++) {
      Kk[p1 + 7 * p2] = 0.0F;
      for (p3 = 0; p3 < 3; p3++) {
        Kk[p1 + 7 * p2] += b_Hk[p1 + 7 * p3] * g_c[p3 + 3 * p2];
      }
    }
  }

  /* KF增益 */
  /* 'EKF_UPDATE:77' if(z_flag(2)) */
  if (z_flag[1] != 0.0F) {
    /* 加速度有测量值 */
    /* 'EKF_UPDATE:78' x = (x_' + Kk * (z - hk)')'; */
    fv3[0] = 2.0F * c;
    fv3[1] = 2.0F * b_c;
    fv3[2] = c_c - d_c;
    for (p1 = 0; p1 < 3; p1++) {
      an_var[p1] = measure[p1] / value - fv3[p1];
    }

    for (p1 = 0; p1 < 7; p1++) {
      gyro_x_bias = 0.0F;
      for (p2 = 0; p2 < 3; p2++) {
        gyro_x_bias += Kk[p1 + 7 * p2] * an_var[p2];
      }

      v[p1] = Xs[p1] + gyro_x_bias;
    }

    fv3[0] = 2.0F * c;
    fv3[1] = 2.0F * b_c;
    fv3[2] = c_c - d_c;
    for (p1 = 0; p1 < 3; p1++) {
      an_var[p1] = measure[p1] / value - fv3[p1];
    }

    for (p1 = 0; p1 < 7; p1++) {
      gyro_x_bias = 0.0F;
      for (p2 = 0; p2 < 3; p2++) {
        gyro_x_bias += Kk[p1 + 7 * p2] * an_var[p2];
      }

      Xs[p1] += gyro_x_bias;
    }

    /* 修正状态 */
    /* 'EKF_UPDATE:79' x(1:4) = x(1:4)/norm(x(1:4),2); */
    value = b_norm(*(float (*)[4])&v[0]);
    for (p1 = 0; p1 < 4; p1++) {
      Xs[p1] = v[p1] / value;
    }

    /* 四元数限幅度 */
    /* 'EKF_UPDATE:80' Pk(:,:) = (eye(7) - Kk*Hk) * Pk_; */
    for (p1 = 0; p1 < 7; p1++) {
      for (p2 = 0; p2 < 7; p2++) {
        gyro_x_bias = 0.0F;
        for (p3 = 0; p3 < 3; p3++) {
          gyro_x_bias += Kk[p1 + 7 * p3] * Hk[p3 + 3 * p2];
        }

        f_c[p1 + 7 * p2] = (float)iv0[p1 + 7 * p2] - gyro_x_bias;
      }

      for (p2 = 0; p2 < 7; p2++) {
        Pk[p1 + 7 * p2] = 0.0F;
        for (p3 = 0; p3 < 7; p3++) {
          Pk[p1 + 7 * p2] += f_c[p1 + 7 * p3] * Pk_[p3 + 7 * p2];
        }
      }
    }

    /* 计算后验协方差  */
  } else {
    /* 'EKF_UPDATE:81' else */
    /* 'EKF_UPDATE:82' x = x_; */
    /* 'EKF_UPDATE:83' Pk(:,:)= Pk_; */
    memcpy(&Pk[0], &Pk_[0], 49U * sizeof(float));
  }

  /* 'EKF_UPDATE:85' q=[x(1),x(2),x(3),x(4)]; */
  q[0] = Xs[0];
  q[1] = Xs[1];
  q[2] = Xs[2];
  q[3] = Xs[3];

  /* 获取系统状态四元数 */
  /* 'EKF_UPDATE:86' Xs=x; */
  /* 'EKF_UPDATE:88' [pitchEKF,rollEKF,yawEKF] = quattoeuler(q); */
  /* 'quattoeuler:2' rad2deg=180/pi; */
  /* 'quattoeuler:3' R_n2b=[ 1 - 2 * (q(4) *q(4) + q(3) * q(3)) 2 * (q(2) * q(3)-q(1) * q(4))         2 * (q(2) * q(4) +q(1) * q(3)) ; */
  /* 'quattoeuler:4'         2 * (q(2) * q(3) +q(1) * q(4))     1 - 2 * (q(4) *q(4) + q(2) * q(2))    2 * (q(3) * q(4)-q(1) * q(2)); */
  /* 'quattoeuler:5'         2 * (q(2) * q(4)-q(1) * q(3))      2 * (q(3) * q(4)+q(1) * q(2))         1 - 2 * (q(2) *q(2) + q(3) * q(3))]; */
  /* cnb */
  /* 'quattoeuler:7' pitch = asin(R_n2b(3,2))*rad2deg; */
  absx31 = (float)asin(2.0F * (q[2] * q[3] + q[0] * q[1])) * 57.2957764F;

  /* 'quattoeuler:8' roll  = atan2(-R_n2b(3,1),R_n2b(3,3))*rad2deg; */
  absx21 = rt_atan2f_snf(-(2.0F * (q[1] * q[3] - q[0] * q[2])), 1.0F - 2.0F *
    (q[1] * q[1] + q[2] * q[2])) * 57.2957764F;

  /* 'quattoeuler:9' yaw   = atan2(-R_n2b(1,2),R_n2b(2,2))*rad2deg; */
  /* 获取纯加速度与陀螺仪的姿态角 */
  /* 'EKF_UPDATE:89' att_f(1)=pitchEKF; */
  att_f[0] = absx31;

  /* 'EKF_UPDATE:90' att_f(2)=rollEKF; */
  att_f[1] = absx21;

  /* 加入磁力计的融合算法  */
  /* 'EKF_UPDATE:93' if (norm(mag_m,2)>0 || yaw_out~=0 || z_flag(3)|| z_flag(4)) */
  for (p1 = 0; p1 < 3; p1++) {
    an_var[p1] = measure[iv2[p1]];
  }

  if ((norm(an_var) > 0.0F) || (measure[9] != 0.0F) || (z_flag[2] != 0.0F) ||
      (z_flag[3] != 0.0F)) {
    /* 'EKF_UPDATE:94' mag_data = mag_m; */
    /* yawEKF(1) = Get_Yaw(pitchEKF(1),rollEKF(1),mag_data);%直接使用P R重投影磁力计测量航向角度   */
    /* 'EKF_UPDATE:96' A_2 = [1,-T;0,1]; */
    b_q[0] = 1.0F;
    b_q[2] = -T;
    for (p1 = 0; p1 < 2; p1++) {
      b_q[1 + (p1 << 1)] = (float)p1;
    }

    for (p1 = 0; p1 < 4; p1++) {
      q[p1] = b_q[p1];
    }

    /* 'EKF_UPDATE:97' H_2 = [1,0]; */
    /* 'EKF_UPDATE:99' yaw_mag(1) = CalEndAngle_Zcoord(yaw_mag(1),(gyro_m(3)-z_bias(1))*T ); */
    /* 'CalEndAngle_Zcoord:2' begin_rad = begin_angle*pi/180; */
    gyro_y_bias = Xsy_m[0] * 3.14159274F / 180.0F;

    /* 'CalEndAngle_Zcoord:3' add_rad = add_angle*pi/180 ; */
    gyro_x_bias = (measure[5] - Xsy_m[1]) * T * 3.14159274F / 180.0F;

    /* 'CalEndAngle_Zcoord:4' add_matrix = [cos(add_rad),-sin(add_rad),0;sin(add_rad),cos(add_rad),0;0,0,1]; */
    /* 'CalEndAngle_Zcoord:5' begin_xy = [cos(begin_rad),sin(begin_rad),0]'; */
    /* 'CalEndAngle_Zcoord:6' end_xy = add_matrix*begin_xy; */
    fv3[0] = (float)cos(gyro_y_bias);
    fv3[1] = (float)sin(gyro_y_bias);
    fv3[2] = 0.0F;
    R[0] = (float)cos(gyro_x_bias);
    R[3] = -(float)sin(gyro_x_bias);
    R[6] = 0.0F;
    R[1] = (float)sin(gyro_x_bias);
    R[4] = (float)cos(gyro_x_bias);
    R[7] = 0.0F;
    for (p1 = 0; p1 < 3; p1++) {
      R[2 + 3 * p1] = iv3[p1];
    }

    for (p1 = 0; p1 < 3; p1++) {
      fv4[p1] = 0.0F;
      for (p2 = 0; p2 < 3; p2++) {
        fv4[p1] += R[p1 + 3 * p2] * fv3[p2];
      }

      an_var[p1] = fv4[p1];
    }

    /* 'CalEndAngle_Zcoord:7' end_rad = atan2(end_xy(2),end_xy(1)); */
    /* 'CalEndAngle_Zcoord:8' end_angle = end_rad/pi*180; */
    /* 预测 */
    /* 'EKF_UPDATE:100' z_bias(1) = z_bias(1); */
    /* 'EKF_UPDATE:101' x_mag_ = [yaw_mag(1),z_bias(1)]; */
    x_mag_[0] = rt_atan2f_snf(an_var[1], an_var[0]) / 3.14159274F * 180.0F;
    x_mag_[1] = Xsy_m[1];

    /* 'EKF_UPDATE:103' P_22 = A_2 * P_mag(:,:) * A_2' + Q_mag; */
    for (p1 = 0; p1 < 2; p1++) {
      for (p2 = 0; p2 < 2; p2++) {
        b_q[p1 + (p2 << 1)] = 0.0F;
        for (p3 = 0; p3 < 2; p3++) {
          b_q[p1 + (p2 << 1)] += q[p1 + (p3 << 1)] * P_mag[p3 + (p2 << 1)];
        }
      }

      for (p2 = 0; p2 < 2; p2++) {
        gyro_x_bias = 0.0F;
        for (p3 = 0; p3 < 2; p3++) {
          gyro_x_bias += b_q[p1 + (p3 << 1)] * q[p2 + (p3 << 1)];
        }

        P_22[p1 + (p2 << 1)] = gyro_x_bias + Q_mag[p1 + (p2 << 1)];
      }
    }

    /* 更新协方差 */
    /* 'EKF_UPDATE:105' if yaw_out==0 */
    if (measure[9] == 0.0F) {
      /* 'EKF_UPDATE:106' z_2 = Get_Yaw(pitchEKF,rollEKF,mag_data); */
      /* 'Get_Yaw:3' pitch = pitch_angle*pi/180; */
      gyro_x_bias = absx31 * 3.14159274F / 180.0F;

      /* 'Get_Yaw:4' roll = roll_angle*pi/180; */
      gyro_y_bias = absx21 * 3.14159274F / 180.0F;

      /* 'Get_Yaw:5' m_x = mag_data(1)*cos(roll)+mag_data(3)*sin(roll); */
      /* 'Get_Yaw:6' m_y = mag_data(1)*sin(pitch)*sin(roll)+ mag_data(2)*cos(pitch)-mag_data(3)*cos(roll)*sin(pitch); */
      /* 'Get_Yaw:7' yaw0 = atan2(m_y,m_x)*180/pi; */
      gyro_y_bias = rt_atan2f_snf((measure[6] * (float)sin(gyro_x_bias) * (float)
        sin(gyro_y_bias) + measure[7] * (float)cos(gyro_x_bias)) - measure[8] *
        (float)cos(gyro_y_bias) * (float)sin(gyro_x_bias), measure[6] * (float)
        cos(gyro_y_bias) + measure[8] * (float)sin(gyro_y_bias)) * 180.0F /
        3.14159274F;

      /* 获取测量值 角度 */
    } else {
      /* 'EKF_UPDATE:107' else */
      /* 'EKF_UPDATE:108' z_2 = yaw_out; */
      gyro_y_bias = measure[9];
    }

    /* 'EKF_UPDATE:111' K_2 = P_22 * H_2' * ((H_2 * P_22 * H_2' + R_mag)^(-1)); */
    gyro_x_bias = 0.0F;
    for (p1 = 0; p1 < 2; p1++) {
      b_P_22[p1] = 0.0F;
      for (p2 = 0; p2 < 2; p2++) {
        b_P_22[p1] += (1.0F - (float)p2) * P_22[p2 + (p1 << 1)];
      }

      gyro_x_bias += b_P_22[p1] * (1.0F - (float)p1);
    }

    c = 1.0F / (gyro_x_bias + mn);
    for (p1 = 0; p1 < 2; p1++) {
      b_P_22[p1] = 0.0F;
      for (p2 = 0; p2 < 2; p2++) {
        b_P_22[p1] += P_22[p1 + (p2 << 1)] * (1.0F - (float)p2);
      }

      K_2[p1] = b_P_22[p1] * c;
    }

    /* 计算卡尔曼增益  */
    /* 'EKF_UPDATE:112' if(z_flag(3) || z_flag(4)) */
    if ((z_flag[2] != 0.0F) || (z_flag[3] != 0.0F)) {
      /* 'EKF_UPDATE:113' x_mag = x_mag_+ (K_2 * (z_2 - x_mag_(1)))'; */
      c = gyro_y_bias - x_mag_[0];

      /* 修正状态    */
      /* 'EKF_UPDATE:114' P_mag(:,:) = (eye(2) - K_2*H_2) * P_22; */
      for (p1 = 0; p1 < 2; p1++) {
        for (p2 = 0; p2 < 2; p2++) {
          b_q[p1 + (p2 << 1)] = (float)iv4[p1 + (p2 << 1)] - K_2[p1] * (1.0F -
            (float)p2);
        }

        for (p2 = 0; p2 < 2; p2++) {
          P_mag[p1 + (p2 << 1)] = 0.0F;
          for (p3 = 0; p3 < 2; p3++) {
            P_mag[p1 + (p2 << 1)] += b_q[p1 + (p3 << 1)] * P_22[p3 + (p2 << 1)];
          }
        }

        x_mag_[p1] += K_2[p1] * c;
      }

      /* 计算后验方程  */
    } else {
      /* 'EKF_UPDATE:115' else */
      /* 'EKF_UPDATE:116' x_mag = x_mag_; */
      /* 'EKF_UPDATE:117' P_mag(:,:)=P_22; */
      for (p1 = 0; p1 < 4; p1++) {
        P_mag[p1] = P_22[p1];
      }
    }

    /* 'EKF_UPDATE:119' yaw_mag(1) = x_mag(1); */
    /* 更新外部状态 */
    /* 'EKF_UPDATE:120' z_bias(1) =  x_mag(2); */
    gyro_y_bias = x_mag_[1];

    /* 'EKF_UPDATE:122' if(abs(z_bias(1))>0.1) */
    if ((float)fabs(x_mag_[1]) > 0.1F) {
      /*  限制状态 */
      /* 'EKF_UPDATE:123' z_bias(1)=0.1 ; */
      gyro_y_bias = 0.1F;
    }

    /* 'EKF_UPDATE:126' Xsy_m(1)=yaw_mag; */
    Xsy_m[0] = x_mag_[0];

    /* 'EKF_UPDATE:127' Xsy_m(2)=z_bias; */
    Xsy_m[1] = gyro_y_bias;

    /* 'EKF_UPDATE:129' att_f(3)=yaw_mag(1); */
    att_f[2] = x_mag_[0];
  } else {
    /* 'EKF_UPDATE:130' else */
    /* 'EKF_UPDATE:131' att_f(3)=yawEKF; */
    att_f[2] = rt_atan2f_snf(-(2.0F * (q[1] * q[2] - q[0] * q[3])), 1.0F - 2.0F *
      (q[3] * q[3] + q[1] * q[1])) * 57.2957764F;
  }

  /* 'EKF_UPDATE:133' gyro_f(1)=gyro_m(1)-Xs(5); */
  gyro_f[0] = measure[3] - Xs[4];

  /* 'EKF_UPDATE:134' gyro_f(2)=gyro_m(2)-Xs(6); */
  gyro_f[1] = measure[4] - Xs[5];

  /* 'EKF_UPDATE:135' gyro_f(3)=gyro_m(3)-Xsy_m(2); */
  gyro_f[2] = measure[5] - Xsy_m[1];
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void EKF_UPDATE_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void EKF_UPDATE_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for EKF_UPDATE.c
 *
 * [EOF]
 */
