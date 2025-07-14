#ifndef __NAV_H
#define __NAV_H
#include "include.h"
#include "base_struct.h"
#define NAV_MIN_GPS_ACC		8.80f					    // minimum gps hAcc needed to enter auto nav modes, in meters
#define NAV_MAX_GPS_AGE		1e6					    // maximum age of position update needed to enter auto nav modes, in microseconds
#define NAV_MIN_FIX_ACC		4.0f					    // minimum gps hAcc still considered a valid "2D" fix, in meters
#define NAV_MAX_FIX_AGE		10e6					    // maximum age of position update still considered a valid "2D" fix, in microseconds

#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters
#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))
#define M_PI			3.14159265f
#define M_PI_2			(M_PI / 2.0f)
#define NAV_HF_HOME_DIST_D_MIN	2.0f						// do not compute dynamic bearing when closer than this to home position (zero to never compute)
#define NAV_HF_HOME_DIST_FREQ	4						// update distance to home at this Hz, should be > 0 and <= 400
#define NAV_HF_HOME_BRG_D_MAX	1.0f * DEG_TO_RAD				// re-compute headfree reference angles when bearing to home changes by this many degrees (zero to always re-compute)
#define NAV_HF_DYNAMIC_DELAY	((int)3e6f)					// delay micros before entering dynamic mode after switch it toggled high


#define MODE_GPS 1
#define MODE_FLOW 2
typedef struct
{
	u8 init[3];
	float att[3],att_rate[3];
	float pos_n[3];
	float spd_n[3];
	float spd_b[3];
	float acc_n[3];
	float acc_b[3];
	u16 init_cnt[3];
	float pos_n_o[3];
	float spd_b_o[3];
	float spd_n_o[3];
	float acc_n_o[3];
	float acc_b_o[3];
	float fake_yaw;
	double lon,lat,local_lon,local_lat,home_lat,home_lon;
	double r1,r2;
}_NAV;

extern _NAV nav;

void baro_fushion(float T);
void pose_fushion(float T);
void flow_data_process(float T);
void CalcEarthRadius(double lat);
void CalcGlobalDistance(double lat, double lon,float local_Lat,float local_Lon,float *posNorth,float *posEast );
void CalcGlobalLocation(float posNorth,float posEast);
float navCalcBearing(double lat1, double lon1, double lat2, double lon2);
#endif

