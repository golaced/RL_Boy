
#include "imu.h"
#include "include.h"
#include "parameter.h"
#include "gait_math.h"
#include "gps.h"
u8 fly_ready;
static xyz_f_t reference_v;
static ref_t 	ref;
float reference_vr[3];
float Roll,Pitch,Yaw;    				//姿态角
static float q0=1,q1,q2,q3;
static float ref_q[4] = {1,0,0,0};
static float norm_acc,norm_q;
static float norm_acc_lpf;
static xyz_f_t mag_sim_3d;
extern u8 fly_ready;

void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out) //小范围内正确。
{
	static s8 pn;
	static float h_tmp_x,h_tmp_y;
	
	h_tmp_x = my_sqrt(my_pow(ref->z) + my_pow(ref->y));
	h_tmp_y = my_sqrt(my_pow(ref->z) + my_pow(ref->x));
	
	pn = ref->z < 0? -1 : 1;
	
	  out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
		out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
	
	out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;
}


float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
float Kp_yaw=0.5*2;
volatile float beta_start=6;
volatile float beta_end=0.03;
volatile float beta_step=0.01;
volatile float beta;
float accConfidenceDecay = 0.52f;
float hmlConfidenceDecay = 4.0;
float accConfidence      = 1.0f; 
float hmlConfidence 			= 1.0f; 
float norm_hml_view,norm_acc_view;
u8 en_hml=0;
#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)
#define accelOneG 9.8
void calculateAccConfidence(float accMag_in)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	static float accMagP = 1.0f;
  float accMag=accMag_in;
	accMag /= accelOneG;  // HJI Added to convert MPS^2 to G's

	accMagP  = HardFilter(accMagP, accMag );

	accConfidence=1-((accConfidenceDecay * sqrtf(fabs(accMagP - 1.0f))));
  if(accConfidence>1)
		accConfidence=1;
	if(accConfidence<0)
		accConfidence=0;
	accConfidence=1;
}

float hmlOneMAG= 200.0;
void calculateHmlConfidence(float Mag_in)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	static float MagP = 1.0f;
  float Mag;
  Mag  = Mag_in/hmlOneMAG;  // HJI Added to convert MPS^2 to G's

	MagP  = HardFilter(MagP, Mag );

	hmlConfidence=2-((hmlConfidenceDecay*sqrtf(fabs(MagP - 1.0f))));
  if(hmlConfidence>1)
		hmlConfidence=1;
	if(hmlConfidence<0)
		hmlConfidence=0;
	hmlConfidence=1;
}
// *  smpl_frq    sampling frequency of AHRS data
// *  b_start     algorithm gain starting value
// *  b_end       algorithm gain end value
// *  b_step      algorithm gain decrement step size

void mag_fail_check(float dt)
{ static u8 state;
	static float timer,timer_err;
  switch(state)
	{
		case 0:
			 if(fabs(mems.hmlOneMAG-hmlOneMAG)>0.2*hmlOneMAG)
					timer+=dt;
			 else
				  timer=0;
		   if(timer>0.68)
			 {state=1;timer=timer_err=0;mems.Mag_ERR=1;}
			 break;
		case 1:
			 if(fabs(mems.hmlOneMAG-hmlOneMAG)<0.35*hmlOneMAG)
					timer+=dt;
			 else
				  timer=0;
			 
			 timer_err+=dt;
		   if(timer>2||timer_err>10)
			 {state=0;timer=timer_err=0;mems.Mag_ERR=0;}
		break;
	}
}

u8 init_hml_norm,mag_cal_use_compass=0;
u16 init_hml_norm_cnt;

void Q_state_init(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
		
		ref_q[0]=q0;
		ref_q[1]=q1;
		ref_q[2]=q2;
		ref_q[3]=q3;

		float rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
		float pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
		float yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;//  
		reference_vr[0] = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
		reference_vr[1] = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
		reference_vr[2] = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);
}

int madgwick_update_new(float T,float wx, float wy, float wz, float ax, float ay, float az,float mx,float my,float mz,float *rol,float *pit,float *yaw) 							
{
    static u8 init,imu_init;
	  static float init_mag_cnt=0,yaw_correct_flt;
    float recip_norm,norm;
    float s0, s1, s2, s3;
    float dq1, dq2, dq3, dq4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float yaw_correct=0,beta_use;
		if(mems.Mag_CALIBRATE)
			init=0;
		if(!init){init=1;
				beta = beta_start;
			if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
			  Q_state_init(ax,ay,az,mx,my,mz);
		}
		
    /* Check for valid magnetometer data. */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    ;
			else if(!init_hml_norm&&init_hml_norm_cnt++>200){init_hml_norm=1;			
				hmlOneMAG = sqrtf(mx*(mx) + my*(my) + mz*mz)*1.1;
		}
		mems.hmlOneACC= norm = sqrtf(ax*(ax) + ay*(ay) + az*az)/4096.*9.8;	
		mems.hmlOneMAG= norm_hml_view = sqrtf(mx*(mx) + my*(my) + mz*mz);	
		if(mems.Mag_ERR||mag_cal_use_compass)
			mx=my=mz=0;
		else
			DigitalLPF(mems.hmlOneMAG,&hmlOneMAG,0.01,T);
		
    /* Check if beta has reached its specified end value or if it has to be
     * decremented. */
    if (beta > beta_end)
    {
        /* Decrement beta only if it does not fall below the specified end
         * value. */
        if ((beta - beta_step) > beta_end)
        {
            beta -= beta_step;
        }
        else
        {
            beta = beta_end;
        }
    }else beta=beta_end;

					
    /* Calculate quaternion rate of change of from angular velocity. */
    dq1 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    dq2 = 0.5f * (q0 * wx + q2 * wz - q3 * wy);
    dq3 = 0.5f * (q0 * wy - q1 * wz + q3 * wx);
    dq4 = 0.5f * (q0 * wz + q1 * wy - q2 * wx);

    /* Calculate feedback only if accelerometer measurement is valid. This
     * prevents NaNs in acceleration normalization. */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {   		  
				calculateAccConfidence(norm);
			  calculateHmlConfidence(norm_hml_view);
			  mag_fail_check(T);//check mag fail
			  
			if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
				beta_use = beta*accConfidence ;
			else
				beta_use = beta*accConfidence * hmlConfidence;
        /* Normalize accelerometer measurement. */
        recip_norm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* Normalize magnetometer measurement. */
        recip_norm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recip_norm;
        my *= recip_norm;
        mz *= recip_norm;
				
        /* Auxiliary variables to avoid repeated arithmetic and therefore
         * improve performance. */
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        /* Reference direction of earth magnetic field. */
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        /* Gradient decent algorithm corrective step. */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        /* Normalize step magnitude. */
        recip_norm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        /* Apply feedback step. */
        dq1 -= beta_use * s0;
        dq2 -= beta_use * s1;
        dq3 -= beta_use * s2;
        dq4 -= beta_use * s3;
    }
		
    /* Integrate quaternion rate of change to get quaternion describing the
     * current orientation. */
    q0 += dq1 * T;
    q1 += dq2 * T;
    q2 += dq3 * T;
    q3 += dq4 * T;

		if(imu_init==0)
			init_mag_cnt+=T;
		if( mems.Mag_update && mems.Mag_ERR==0&&mag_cal_use_compass)
		{   
				int magTmp2[3]; 
				float euler[2];
			  mems.Mag_update=0;
				magTmp2[0]=mems.Mag_Val.y;
				magTmp2[1]=-mems.Mag_Val.x;
				magTmp2[2]=mems.Mag_Val.z;
				euler[0]= Roll /57.3  ;
				euler[1]=-Pitch/57.3  ;
				float calMagY = magTmp2[0] * cosf(euler[1]) + magTmp2[1] * sinf(euler[1])* sinf(euler[0])+magTmp2[2] * sinf(euler[1]) * cosf(euler[0]); 
				float calMagX = magTmp2[1] * cosf(euler[0]) + magTmp2[2] * sinf(euler[0]);
				float wxf,wyf,wzf;
				if( (fabs(Roll)<30 && fabs(Pitch)<30))
				mems.Yaw_Mag=To_180_degrees(fast_atan2(calMagX,calMagY)* 57.3 );
 				
					if(init_mag_cnt<5)
					{ 
						yaw_correct = Kp_yaw *100 *LIMIT( To_180_degrees(mems.Yaw_Mag - Yaw),-45,45 );
					}
					else
					{ imu_init=1;
						yaw_correct = Kp_yaw *1.5f *To_180_degrees(mems.Yaw_Mag - Yaw);
					}
					DigitalLPF(yaw_correct,&yaw_correct_flt,1,T);
					wxf = (wx - reference_vr[0] *yaw_correct_flt) *ANGLE_TO_RADIAN;    
					wyf = (wy - reference_vr[1] *yaw_correct_flt) *ANGLE_TO_RADIAN;		
					wzf = (wz - reference_vr[2] *yaw_correct_flt) *ANGLE_TO_RADIAN;
					dq1 = 0.5f * (-q1 * wxf - q2 * wyf - q3 * wzf);
					dq2 = 0.5f * (q0 * wxf + q2 * wzf - q3 * wyf);
					dq3 = 0.5f * (q0 * wyf - q1 * wzf + q3 * wxf);
					dq4 = 0.5f * (q0 * wzf + q1 * wyf - q2 * wxf);
					q0 += dq1 * T;
					q1 += dq2 * T;
					q2 += dq3 * T;
					q3 += dq4 * T;
		}

    /* Normalize quaternion. */
    recip_norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;

	ref_q[0]=q0;
	ref_q[1]=q1;
	ref_q[2]=q2;
	ref_q[3]=q3;
	reference_vr[0] = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_vr[1] = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_vr[2] = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);
	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;// 
   if ((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)&&mag_cal_use_compass==0)
		*yaw = To_180_degrees(*yaw-90);
    return 1;
}

float Kp =0.36;   								  // proportional gain governs rate of convergence to accelerometer/magnetometer
float Kp_yaw1=0.1345;
float Ki =0.005f    ;            	// 0.001  integral gain governs rate of convergence of gyroscope biases
#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)
static xyz_f_t mag_sim_3d,acc_3d_hg,acc_ng,acc_ng_offset;
static float mag_norm ,mag_norm_xyz, yaw_mag_view[4];

float mag_yaw_cal(float ax,float ay,float az,float mx,float my,float mz,float pit,float rol){
	float initialRoll, initialPitch;
	float cosRoll, sinRoll, cosPitch, sinPitch;
	float magX, magY;
	float initialHdg, cosHeading, sinHeading;
  if(rol==0)
	initialRoll = atan2(-ay, -az);
	else
	initialRoll=rol;
	if(pit==0)
	initialPitch = atan2(ax, -az);
	else
	initialPitch=pit;	

	cosRoll = cosf(initialRoll);
	sinRoll = sinf(initialRoll);
	cosPitch = cosf(initialPitch);
	sinPitch = sinf(initialPitch);

	magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

	magY = my * cosRoll - mz * sinRoll;

	return  atan2f(-magY, magX)*57.3;
}		

char  mag_sel[2]={1,2};
char  yaw_fix_sel=1;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
{		
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;
	static xyz_f_t mag_tmp;
	static float yaw_mag;
	static u16 cnt,init_mag_cnt;
	static u8 init,init_yaw;
	float calMagY,calMagX,magTmp2[3],euler[3];
	float mx,my,mz;
	if(init==0)
	{
	init=1;
	ref.err.x=ref.err.y=ref.err.z=0;
	ref.err_Int.x=ref.err_Int.y=ref.err_Int.z=0;
	ref.err_lpf.x=ref.err_lpf.y=ref.err_lpf.z=0;
	ref.err_tmp.x=ref.err_tmp.y=ref.err_tmp.z=0;
	ref.g.x=ref.g.y=ref.g.z=0;
	}
	
	if(mems.Mag_CALIBRATE)//   11  20
		init_yaw=0;
	switch(mag_sel[0]){
	case 0:
		mx=mems.Mag.y;
		my=mems.Mag.x;
		mz=mems.Mag.z;
	break;		
	case 1:
		mx=mems.Mag.y;
		my=-mems.Mag.x;
		mz=mems.Mag.z;
	break;	
	case 2:
		mx=-mems.Mag.y;
		my=-mems.Mag.x;
		mz=mems.Mag.z;
		break;	
	case 3:
		mx=-mems.Mag.y;
		my=mems.Mag.x;
		mz=mems.Mag.z;
		break;			
	}
	if(module.hml_imu_o)
  switch(mag_sel[1]){
	case 0:
		mx=mems.Mago.y;
		my=mems.Mago.x;
		mz=-mems.Mago.z;	
	break;		
	case 1:
		mx=mems.Mago.y;
		my=-mems.Mago.x;
		mz=-mems.Mago.z;	
	break;	
	case 2:
		mx=-mems.Mago.y;
		my=-mems.Mago.x;
		mz=-mems.Mago.z;	
		break;	
	case 3:
		mx=-mems.Mago.y;
		my=mems.Mago.x;
		mz=-mems.Mago.z;	
		break;			
	}
	mag_norm_tmp = 20 *(6.28f *half_T);	
	mems.hmlOneMAG= mag_norm_xyz = sqrtf(mx* mx + my*my + mz*mz);
	calculateHmlConfidence(mems.hmlOneMAG);	
	if(mems.Mag_ERR)
		DigitalLPF(mems.hmlOneMAG,&hmlOneMAG,0.01,half_T*2);
	
	mag_fail_check(half_T*2);//check mag fail
	
	if(mag_norm_xyz==0)mag_norm_xyz=0.0001;
	if( mag_norm_xyz != 0)
	{
		mag_tmp.x += mag_norm_tmp *( (float)mx/( mag_norm_xyz ) - mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *( (float)my/( mag_norm_xyz ) - mag_tmp.y);	
		mag_tmp.z += mag_norm_tmp *( (float)mz/( mag_norm_xyz ) - mag_tmp.z);	
	}

	simple_3d_trans(&reference_v,&mag_tmp,&mag_sim_3d);
	
	mag_norm = sqrtf(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y);
	if(mag_norm==0)mag_norm=0.0001;
//	if( mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
//		yaw_mag_view[0] = fast_atan2( ( mag_sim_3d.y/mag_norm ) , ( mag_sim_3d.x/mag_norm) ) *57.3f;
//	
//	yaw_mag_view[1] =mag_yaw_cal(ax,ay,az,mx,my,mz,0,0);	
	
	magTmp2[0]=mx;
	magTmp2[1]=my;
	magTmp2[2]=mz;
	switch(yaw_fix_sel){
		case 0:
		euler[0]=Roll*0.0173  ;
		euler[1]=Pitch*0.0173  ;
		break;
		case 1:
		euler[0]=Roll*0.0173  ;
		euler[1]=-Pitch*0.0173  ;
		break;
		case 2:
		euler[0]=-Roll*0.0173  ;
		euler[1]=-Pitch*0.0173  ;
		break;
		case 3:
		euler[0]=-Roll*0.0173  ;
		euler[1]=Pitch*0.0173  ;
		break;
		case 4:
		euler[1]=Roll*0.0173  ;
		euler[0]=Pitch*0.0173  ;
		break;
		case 5:
		euler[1]=Roll*0.0173  ;
		euler[0]=-Pitch*0.0173  ;
		break;
		case 6:
		euler[1]=-Roll*0.0173  ;
		euler[0]=-Pitch*0.0173  ;
		break;
		case 7:
		euler[1]=-Roll*0.0173  ;
		euler[0]=Pitch*0.0173  ;
		break;			
	}

	calMagY = magTmp2[0] * cosf(euler[1]) + magTmp2[1] * sinf(euler[1])* sinf(euler[0])+magTmp2[2] * sinf(euler[1]) * cosf(euler[0]); 
	calMagX = magTmp2[1] * cosf(euler[0]) + magTmp2[2] * sinf(euler[0]);
	float tempy;
	if( (fabs(Roll)<30 && fabs(Pitch)<30))
	tempy=To_180_degrees(fast_atan2(calMagX,calMagY)* 57.3 );

	yaw_mag_view[2]=Moving_Median(14,5,tempy);	

	if((module.hml_imu||module.hml_imu_o)&&mems.Mag_Have_Param&&en_hml
		&& mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
		yaw_mag=yaw_mag_view[2]+Gps_information.off_earth;
	else
		{yaw_mag=Yaw;hmlConfidence=1;}
	//=============================================================================
	// 计算等效重力向量//十分重要

	reference_vr[0]=reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_vr[1]=reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_vr[2]=reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);
		
	//这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	//根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	//所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。       
//=============================================================================
	acc_ng.x = 10 *TO_M_S2 *(ax - 4096*reference_v.x) - acc_ng_offset.x;
	acc_ng.y = 10 *TO_M_S2 *(ay - 4096*reference_v.y) - acc_ng_offset.y;
	acc_ng.z = 10 *TO_M_S2 *(az - 4096*reference_v.z) - acc_ng_offset.z;
	
	acc_3d_hg.z = acc_ng.x *reference_v.x + acc_ng.y *reference_v.y + acc_ng.z *reference_v.z;

	// 计算加速度向量的模
	norm_acc = sqrtf(ax*ax + ay*ay + az*az);   
	norm_acc_lpf +=  NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001
	mems.hmlOneACC= norm_acc_lpf/4096.*9.8;	
	calculateAccConfidence(mems.hmlOneACC);
  if(norm_acc==0)norm_acc=0.0001;
	if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
	{	
		//把加计的三维向量转成单位向量。
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )
		{
			/* 叉乘得到误差 */
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
	    //ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;		
			/* 误差低通 */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
	//			 ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  - ref.err_lpf.z );
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
//				ref.err.z = ref.err_lpf.z ;
		}
	}
	else
	{
		ref.err.x = 0; 
		ref.err.y = 0  ;
//		ref.err.z = 0 ;
	}
	/* 误差积分 */
	ref.err_Int.x += ref.err.x *Ki *2 *half_T ;
	ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
	
	/* 积分限幅 */
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );

	
	if(init_mag_cnt++>5/0.005)
	{	
	 init_mag_cnt=0;init_yaw=1;
	}else  hmlOneMAG=mems.hmlOneMAG;
	
	if( reference_v.z > 0.0f )
	{	
		if(!init_yaw)
		{ 
			yaw_correct = Kp_yaw1 *100 *LIMIT( To_180_degrees(yaw_mag - Yaw),-45,45 );
		}
		else if(( fly_ready||(fabs(Pitch)>20)||(fabs(Roll)>20)))
		{ 
			yaw_correct = Kp_yaw1 *0.35 *LIMIT( To_180_degrees(yaw_mag - Yaw),-45,45 );
		}
		else
		{
			yaw_correct = Kp_yaw1 *1.5f *LIMIT( To_180_degrees(yaw_mag - Yaw),-45,45 );
		}
	}

	ref.g.x = (gx - hmlConfidence*reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( accConfidence*Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	ref.g.y = (gy - hmlConfidence*reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( accConfidence*Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	ref.g.z = (gz - hmlConfidence*reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;
	
	/* 用叉积误差来做PI修正陀螺零偏 */
	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;  

	/* 四元数规一化 normalise quaternion */
	norm_q = sqrtf(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	if(norm_q==0)norm_q=1;
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;
	
	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asinf(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;// 
}