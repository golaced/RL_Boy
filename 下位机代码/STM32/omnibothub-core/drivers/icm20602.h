#ifndef _DRV_ICM20602_H_
#define _DRV_ICM20602_H_

#include "include.h"

typedef struct
{
	float center_pos_cm[3];
	float gyro_rad[3];
	float gyro_rad_old[3];
	float gyro_rad_acc[3];
	float linear_acc[3];
}_center_pos_st;
extern _center_pos_st center_pos;

extern u8 mpu_buffer[14];

typedef struct
{
  u8 surface_CALIBRATE;
	float surface_vec[3];
	float surface_unitvec[3];
	
}_sensor_rotate_st;
extern _sensor_rotate_st sensor_rot ;


u8 Icm20602Reg_Init(void);
void Icm20602_Read(void);
void Sensor_Data_Prepare(u8 dT_ms);
void Center_Pos_Set(void);
void  Acc_Prepare(float Pitch,float Roll,float Yaw);
float converRT(float xn,float yn,float zn,s16 *xb,s16 *yb,s16 *zb);
float converRT_float(float xn,float yn,float zn,float *xb,float *yb,float *zb);
typedef struct 
{ 
u8 Cali_3d;	
char Acc_CALIBRATE;
char Gyro_CALIBRATE;
xyz_s16_t Acc_I16;
xyz_s16_t Gyro_I16;

xyz_f_t Acc,Acc_t;
xyz_f_t Gyro,Gyro_t;

//	XYZ_STRUCT Acc_deg;
xyz_f_t Gyro_deg,Gyro_deg_t;

xyz_f_t Acc_Offset;
xyz_f_t Gyro_Offset;
u8 acc_cal_3d_step;	
xyz_f_t Gain_3d;
xyz_f_t Off_3d;
float att_off[2];
	
xyz_f_t Gyro_Auto_Offset;
float Temprea_Offset;
float Gyro_Temprea_Adjust;
float ACC_Temprea_Adjust;

s16 Tempreature;
float TEM_LPF;
float Ftempreature;

xyz_s16_t Mag_Adc;			//采样值
xyz_f_t   Mag_Offset,Mag_Offseto;		//偏移值
xyz_f_t   Mag_Offset_c,Mag_Offset_co;		//偏移值
xyz_f_t   Mag_Gain,Mag_Gaino;		//偏移值
xyz_f_t 	Mag_Gain_c,Mag_Gain_co;			//比例缩放	
xyz_f_t 	Mag_Val,Mag_Val_t,Mag_Valo,Mag_Val_to;			//纠正后的值
u8 Mag_CALIBRATED;
float yaw;
float Pressure;
float Tem_bmp;
float Alt;
float hmlOneMAG,hmlOneACC;
}LIS3MDL_S;

extern LIS3MDL_S lis3mdl,lis3mdl_cov;


#define   WHO_AM_I     0x0F
#define   CTRL_REG1    0x20
#define   CTRL_REG2    0x21
#define   CTRL_REG3    0x22
#define   CTRL_REG4    0x23
#define   CTRL_REG5    0x24
#define    XLDA            0x1 
#define   STATUS_REG   0x27
#define   OUT_X_L      0x28
#define   OUT_X_H      0x29
#define   OUT_Y_L      0x2A
#define   OUT_Y_H      0x2B
#define  OUT_Z_L      0x2C
#define  OUT_Z_H      0x2D
#define   TEMP_OUT_L   0x2E
#define  TEMP_OUT_H   0x2F
#define   INT_CFG      0x30
#define  INT_SRC      0x31
#define  INT_THS_L    0x32
#define   INT_THS_H    0x33

#define     LSM6D33_ADDR_READ               0xd5
#define     LSM6D33_ADDR_WRITE              0xd4
#define  CTRL1_XL           0x10
#define CTRL2_G            0x11
#define CTRL3_C            0x12
#define CTRL4_C            0x13
#define CTRL5_C            0x14
#define CTRL6_C            0x15
#define  CTRL7_G            0x16
#define CTRL8_XL          0x17
#define CTRL9_XL           0x18
#define CTRL10_C           0x19

#define   OUTX_L_G           0x22
#define   OUTX_H_G           0x23
#define   OUTY_L_G           0x24
#define   OUTY_H_G           0x25
#define  OUTZ_L_G           0x26
#define   OUTZ_H_G           0x27
#define  OUTX_L_XL          0x28
#define  OUTX_H_XL          0x29
#define  OUTY_L_XL          0x2A
#define  OUTY_H_XL          0x2B
#define OUTZ_L_XL          0x2C
#define OUTZ_H_XL          0x2D
void LIS3MDL_Init(void);
void DS33_Init(void);
void LIS3MDL_read(void);
void LSM6_readAcc(u8 fast);
void LSM6_readGyro(u8 fast);

//-

//========ICM20602寄存器地址========================
/********************************************
*复位后所有寄存器地址都为0，除了
*Register 26  CONFIG				= 0x80
*Register 107 Power Management 1 	= 0x41
*Register 117 WHO_AM_I 				= 0x12
*********************************************/
//陀螺仪温度补偿
#define	ICM20_XG_OFFS_TC_H				0x04
#define	ICM20_XG_OFFS_TC_L				0x05
#define	ICM20_YG_OFFS_TC_H				0x07
#define	ICM20_YG_OFFS_TC_L				0x08
#define	ICM20_ZG_OFFS_TC_H				0x0A
#define	ICM20_ZG_OFFS_TC_L				0x0B
//加速度自检输出(出产时设置，用于与用户的自检输出值比较）
#define	ICM20_SELF_TEST_X_ACCEL			0x0D
#define	ICM20_SELF_TEST_Y_ACCEL			0x0E
#define	ICM20_SELF_TEST_Z_ACCEL			0x0F
//陀螺仪静态偏移
#define	ICM20_XG_OFFS_USRH				0x13
#define	ICM20_XG_OFFS_USRL				0x14
#define	ICM20_YG_OFFS_USRH				0x15
#define	ICM20_YG_OFFS_USRL				0x16
#define	ICM20_ZG_OFFS_USRH				0x17
#define	ICM20_ZG_OFFS_USRL				0x18

#define	ICM20_SMPLRT_DIV				0x19
#define	ICM20_CONFIG					0x1A
#define	ICM20_GYRO_CONFIG				0x1B
#define	ICM20_ACCEL_CONFIG				0x1C
#define	ICM20_ACCEL_CONFIG2				0x1D
#define	ICM20_LP_MODE_CFG				0x1E

//运动唤醒加速度阈值
#define	ICM20_ACCEL_WOM_X_THR			0x20
#define	ICM20_ACCEL_WOM_Y_THR			0x21
#define	ICM20_ACCEL_WOM_Z_THR			0x22


#define	ICM20_FIFO_EN					0x23
#define	ICM20_FSYNC_INT					0x36
#define	ICM20_INT_PIN_CFG				0x37
//#define	ICM20_INT_ENABLE				0x38
#define	ICM20_FIFO_WM_INT_STATUS		0x39
#define	ICM20_INT_STATUS				0x3A

//加速度输出
#define	ICM20_ACCEL_XOUT_H				0x3B
#define	ICM20_ACCEL_XOUT_L				0x3C
#define	ICM20_ACCEL_YOUT_H				0x3D
#define	ICM20_ACCEL_YOUT_L				0x3E
#define	ICM20_ACCEL_ZOUT_H				0x3F
#define	ICM20_ACCEL_ZOUT_L				0x40
//温度输出
#define	ICM20_TEMP_OUT_H				0x41
#define	ICM20_TEMP_OUT_L				0x42
//角速度输出
#define	ICM20_GYRO_XOUT_H				0x43
#define	ICM20_GYRO_XOUT_L				0x44
#define	ICM20_GYRO_YOUT_H				0x45
#define	ICM20_GYRO_YOUT_L				0x46
#define	ICM20_GYRO_ZOUT_H				0x47
#define	ICM20_GYRO_ZOUT_L				0x48
//陀螺仪自检输出
#define	ICM20_SELF_TEST_X_GYRO			0x50
#define	ICM20_SELF_TEST_Y_GYRO			0x51
#define	ICM20_SELF_TEST_Z_GYRO			0x52

#define	ICM20_FIFO_WM_TH1				0x60
#define	ICM20_FIFO_WM_TH2				0x61
#define	ICM20_SIGNAL_PATH_RESET			0x68
#define	ICM20_ACCEL_INTEL_CTRL 			0x69
#define	ICM20_USER_CTRL					0x6A
//电源控制
#define	ICM20_PWR_MGMT_1				0x6B
#define	ICM20_PWR_MGMT_2				0x6C

#define	ICM20_I2C_IF					0x70
#define	ICM20_FIFO_COUNTH				0x72
#define	ICM20_FIFO_COUNTL				0x73
#define	ICM20_FIFO_R_W					0x74

#define	ICM20_WHO_AM_I 					0x75
//加速度静态偏移
#define	ICM20_XA_OFFSET_H				0x77
#define	ICM20_XA_OFFSET_L				0x78
#define	ICM20_YA_OFFSET_H				0x7A
#define	ICM20_YA_OFFSET_L				0x7B
#define	ICM20_ZA_OFFSET_H				0x7D
#define	ICM20_ZA_OFFSET_L 				0x7E
//===========================================================


//加速度量程
#define ICM20_ACCEL_FS_2G			(0<<3)
#define ICM20_ACCEL_FS_4G			(1<<3)
#define ICM20_ACCEL_FS_8G			(2<<3)
#define ICM20_ACCEL_FS_16G			(3<<3)
//角速度量程
#define ICM20_GYRO_FS_250			(0<<3)
#define ICM20_GYRO_FS_500			(1<<3)
#define ICM20_GYRO_FS_1000			(2<<3)
#define ICM20_GYRO_FS_2000			(3<<3)
//CONFIG DPF
#define DLPF_BW_250         	0x00	//Rate=8k
#define DLPF_BW_176         	0x01
#define DLPF_BW_92          	0x02
#define DLPF_BW_41          	0x03
#define DLPF_BW_20          	0x04
#define DLPF_BW_10          	0x05
#define DLPF_BW_5           	0x06
#define DLPF_BW_328           	0x06	//Rate=8k
//ACCEL_CONFIG2
#define ACCEL_AVER_4         	(0x00<<4)	//Rate=8k
#define ACCEL_AVER_8			(0x01<<4)
#define ACCEL_AVER_16			(0x02<<4)
#define ACCEL_AVER_32			(0x03<<4)
//ACCEL_DLPF
#define ACCEL_DLPF_BW_218         	0x00	
//#define ACCEL_DLPF_BW_218         	0x01
#define ACCEL_DLPF_BW_99          	0x02
#define ACCEL_DLPF_BW_44          	0x03
#define ACCEL_DLPF_BW_21          	0x04
#define ACCEL_DLPF_BW_10          	0x05
#define ACCEL_DLPF_BW_5           	0x06
#define ACCEL_DLPF_BW_420           0x06	


#define GRAVITY_MSS 9.7833f                                //重力加速度
#define _ACCEL_SCALE_1G    (GRAVITY_MSS / 2048.0f)			//加计量程16G
#define _DEG_TO_RAD    0.0174532f 							//度转弧度


uint8_t icm20602_init(void);
uint8_t icm20602_set_gyro_fullscale(uint8_t fs);
uint8_t icm20602_set_accel_fullscale(uint8_t fs);
uint8_t icm20602_get_accel_adc(void);
uint8_t icm20602_get_gyro_adc(void);
float icm20602_get_temp(void);
#endif
