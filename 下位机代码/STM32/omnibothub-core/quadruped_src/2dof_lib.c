#include <math.h>
#include "2dof_lib.h"
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include <float.h>
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
/* Function Declarations */
static void b_cosd(float *x);
static void b_sind(float *x);
static float rt_remf_snf(float u0, float u1);

/* Function Definitions */

/*
 * Arguments    : float *x
 * Return Type  : void
 */
static void b_cosd(float *x)
{
  float absx;
  signed char n;
  if (!((!rtIsInfF(*x)) && (!rtIsNaNF(*x)))) {
    *x = ((real32_T)rtNaN);
  } else {
    *x = rt_remf_snf(*x, 360.0F);
    absx = (float)fabs(*x);
    if (absx > 180.0F) {
      if (*x > 0.0F) {
        *x -= 360.0F;
      } else {
        *x += 360.0F;
      }

      absx = (float)fabs(*x);
    }

    if (absx <= 45.0F) {
      *x *= 0.0174532924F;
      n = 0;
    } else if (absx <= 135.0F) {
      if (*x > 0.0F) {
        *x = 0.0174532924F * (*x - 90.0F);
        n = 1;
      } else {
        *x = 0.0174532924F * (*x + 90.0F);
        n = -1;
      }
    } else if (*x > 0.0F) {
      *x = 0.0174532924F * (*x - 180.0F);
      n = 2;
    } else {
      *x = 0.0174532924F * (*x + 180.0F);
      n = -2;
    }

    if (n == 0) {
      *x = (float)cos(*x);
    } else if (n == 1) {
      *x = -(float)sin(*x);
    } else if (n == -1) {
      *x = (float)sin(*x);
    } else {
      *x = -(float)cos(*x);
    }
  }
}

/*
 * Arguments    : float *x
 * Return Type  : void
 */
static void b_sind(float *x)
{
  float absx;
  signed char n;
  if (!((!rtIsInfF(*x)) && (!rtIsNaNF(*x)))) {
    *x = ((real32_T)rtNaN);
  } else {
    *x = rt_remf_snf(*x, 360.0F);
    absx = (float)fabs(*x);
    if (absx > 180.0F) {
      if (*x > 0.0F) {
        *x -= 360.0F;
      } else {
        *x += 360.0F;
      }

      absx = (float)fabs(*x);
    }

    if (absx <= 45.0F) {
      *x *= 0.0174532924F;
      n = 0;
    } else if (absx <= 135.0F) {
      if (*x > 0.0F) {
        *x = 0.0174532924F * (*x - 90.0F);
        n = 1;
      } else {
        *x = 0.0174532924F * (*x + 90.0F);
        n = -1;
      }
    } else if (*x > 0.0F) {
      *x = 0.0174532924F * (*x - 180.0F);
      n = 2;
    } else {
      *x = 0.0174532924F * (*x + 180.0F);
      n = -2;
    }

    if (n == 0) {
      *x = (float)sin(*x);
    } else if (n == 1) {
      *x = (float)cos(*x);
    } else if (n == -1) {
      *x = -(float)cos(*x);
    } else {
      *x = -(float)sin(*x);
    }
  }
}

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_remf_snf(float u0, float u1)
{
  float y;
  float b_u1;
  float q;
  if (!((!rtIsNaNF(u0)) && (!rtIsInfF(u0)) && ((!rtIsNaNF(u1)) && (!rtIsInfF(u1)))))
  {
    y = ((real32_T)rtNaN);
  } else {
    if (u1 < 0.0F) {
      b_u1 = (float)ceil(u1);
    } else {
      b_u1 = (float)floor(u1);
    }

    if ((u1 != 0.0F) && (u1 != b_u1)) {
      q = (float)fabs(u0 / u1);
      if ((float)fabs(q - (float)floor(q + 0.5F)) <= FLT_EPSILON * q) {
        y = 0.0F * u0;
      } else {
        y = (float)fmod(u0, u1);
      }
    } else {
      y = (float)fmod(u0, u1);
    }
  }

  return y;
}

/*
 * Arguments    : float Alpha1
 *                float Alpha2
 *                const float Param[6]
 *                float *Xe
 *                float *Ye
 * Return Type  : void
 */
void f_2Dof(float Alpha1, float Alpha2, const float Param[6], float *Xe, float
            *Ye)
{
  float lengthAC;
  float Xa;
  float Ya;
  float Xc;
  float Yc;
  float B;
  lengthAC = Alpha1;
  b_cosd(&lengthAC);
  Xa = Param[0] * lengthAC;
  lengthAC = Alpha1;
  b_sind(&lengthAC);
  Ya = Param[0] * lengthAC;
  lengthAC = Alpha2;
  b_cosd(&lengthAC);
  Xc = Param[4] + Param[3] * lengthAC;
  lengthAC = Alpha2;
  b_sind(&lengthAC);
  Yc = Param[3] * lengthAC;
  lengthAC = Xc - Xa;
  B = Yc - Ya;
  lengthAC = (float)sqrt(lengthAC * lengthAC + B * B);
  Xc = 2.0F * Param[1] * (Xc - Xa);
  B = 2.0F * Param[1] * (Yc - Ya);
  lengthAC = (Param[1] * Param[1] + lengthAC * lengthAC) - Param[2] * Param[2];
  B = 2.0F * (57.2957802F * (float)atan((B + (float)sqrt((Xc * Xc + B * B) -
    lengthAC * lengthAC)) / (Xc + lengthAC + 0.0000001)));
  lengthAC = B;
  b_cosd(&lengthAC);
  *Xe = Xa + (Param[1] + Param[5]) * lengthAC;

  /* O */
  /* D */
  /* A */
  /* C */
  /* B */
  /* E */
  b_sind(&B);
  *Ye = -(Ya + (Param[1] + Param[5]) * B);
}

//快速平方根算法
static float my_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}


//------------------------------------------------------------------------------------

void b_sqrt(float *x)
{
  //*x = (float)sqrt(*x);
	*x = (float)my_sqrt(*x);
}

void b_sin(float x[2])
{
  int k;
  for (k = 0; k < 2; k++) {
    x[k] = sin(x[k]);
  }
}

void b_cos(float x[2])
{
  int k;
  for (k = 0; k < 2; k++) {
    x[k] = cos(x[k]);
  }
}

void i_2Dof(float x, float z, const float Param[6], float *Alpha1, float
            *Alpha2)
{
  float a;
  float b;
  float c;
  float b_y;
  float Alpha10[2];
  int i0;
  float dv0[2];
  float Xb[2];
  float Yb[2];
  float d[2];
  float Alpha20_idx_1;
  float e[2];
	float y;
  y = -z;
  a = 2.0F * x * Param[0];
  b = 2.0F * y * Param[0];
  c = Param[1] + Param[5];
  c = ((x * x + y * y) + Param[0] * Param[0]) - c * c;
  b_y = (a * a + b * b) - c * c;
  b_sqrt(&b_y);
  Alpha10[0] = 2.0F * (float)atan((b + b_y) / (a + c+ 0.0000001));
  b_y = (a * a + b * b) - c * c;
  b_sqrt(&b_y);
  Alpha10[1] = 2.0F * (float)atan((b - b_y) / (a + c+ 0.0000001));
  c = Param[1] + Param[5];
  for (i0 = 0; i0 < 2; i0++) {
    dv0[i0] = Alpha10[i0];
  }

  b_cos(dv0);
  b = Param[1] + Param[5];
  for (i0 = 0; i0 < 2; i0++) {
    Xb[i0] = x - Param[5] * ((x - Param[0] * (float)dv0[i0]) / (c+ 0.0000001));
    dv0[i0] = Alpha10[i0];
  }

  b_sin(dv0);
  c = 2.0F * Param[3];
  b_y = 2.0F * Param[3];
  for (i0 = 0; i0 < 2; i0++) {
    a = y - Param[5] * ((y - Param[0] * (float)dv0[i0]) / (b+ 0.0000001));
    d[i0] = c * (Xb[i0] - Param[4]);
    e[i0] = b_y * a;
    Yb[i0] = a;
  }

  a = Xb[0] - Param[4];
  Xb[0] = ((a * a + Param[3] * Param[3]) + Yb[0] * Yb[0]) - Param[2] * Param[2];
  Alpha20_idx_1 = 2.0F * (float)atan((e[0] - (float)sqrt((d[0] * d[0] + e[0] *
    e[0]) - Xb[0] * Xb[0])) / (d[0] + Xb[0]+ 0.0000001));
  if (Alpha10[0] < 0.0) {
    Alpha10[0] += 6.2831853071795862;
  }

  if (Alpha20_idx_1 < 0.0) {
    Alpha20_idx_1 += 6.2831853071795862;
  }

  *Alpha1 = Alpha10[0] * 57.3;
  *Alpha2 = Alpha20_idx_1 * 57.3;
}