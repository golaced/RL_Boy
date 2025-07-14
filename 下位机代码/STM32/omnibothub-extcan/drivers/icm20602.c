#include "math.h"
#include "gait_math.h"
#include "spi.h"
#include "icm20602.h"
#include "cycle_cal_oldx.h"
//PIT向下 y- ROL向右x- ACC
//PIT向下 x-600  ROL向右  y+300  YAW顺-z
LIS3MDL_S lis3mdl;
LIS3MDL_S lis3mdl_cov;
#define DS33_WHO_AM_I_ID     0x69  
#define DS33_SA0_HIGH_ADDRESS 0x6b
#define DS33_SA0_LOW_ADDRESS  0x6a
// Reads the 3 mag channels and stores them in vector m
#define DS33_ADDRESS1  (DS33_SA0_HIGH_ADDRESS << 1)
#define DS33_ADDRESS2  (DS33_SA0_LOW_ADDRESS << 1)
#define DS33_IIC_ID DS33_ADDRESS1
#define ST_SENSORS_SPI_READ			0x80
void Lis3mdl_SPI_WR(u8 add,u8 wrdata,u8 sel)
{
  SPI_CS(sel,0);
	//Spi_RW(add&0x7F );	
  Spi_RW(add);
  Spi_RW(wrdata);
  SPI_CS(sel,1);
}


void SPI_BufferRead(u8*buf, u8 add, u8 len,u8 sel)
{
	u8 i=0;
  SPI_CS(sel,0);
	if(sel!=CS_LIS)
	Spi_RW(add|ST_SENSORS_SPI_READ);	
	else
	Spi_RW(add|0xC0);
	for(i=0;i<len;i++)
	{
	 *buf++ = Spi_RW(0xff); 
	} 
 SPI_CS(sel,1);
}


float converRT(float xn,float yn,float zn,
							 s16 *xb,s16 *yb,s16 *zb)
{
	float RT[3][3];
	float x,y,z;
	float pit=mems.imu_att.x+mems.imu_att_bias.x;
	float rol=mems.imu_att.y+mems.imu_att_bias.y;
	float yaw=-mems.imu_att.z+mems.imu_att_bias.z;
	
	RT[0][0] = cosd(-pit); RT[0][1] =sind(-pit)*sind(-rol)  ;RT[0][2] = sind(-pit)*cosd(-rol);
	RT[1][0] = 0;					 RT[1][1] =cosd(-rol) ;						 RT[1][2] = -sind(-rol);
	RT[2][0] = -sind(-pit);RT[2][1] =cosd(-pit)*sind(-rol) ; RT[2][2] = cosd(-pit)*cosd(-rol);
	
	x = RT[0][0]*xn + RT[0][1]*yn + RT[0][2]*zn;
	y = RT[1][0]*xn + RT[1][1]*yn + RT[1][2]*zn;
	z = RT[2][0]*xn + RT[2][1]*yn + RT[2][2]*zn;
	
	*xb=x*cosd(yaw)-y*sind(yaw);
	*yb=x*sind(yaw)+y*cosd(yaw);
	*zb=z;
}

float converRT_float(float xn,float yn,float zn,
							 float *xb,float *yb,float *zb)
{
	float RT[3][3];
	float x,y,z;
	float pit=mems.imu_att.x+mems.imu_att_bias.x;
	float rol=mems.imu_att.y+mems.imu_att_bias.y;
	float yaw=-mems.imu_att.z+mems.imu_att_bias.z;
	
	RT[0][0] = cosd(-pit); RT[0][1] =sind(-pit)*sind(-rol)  ;RT[0][2] = sind(-pit)*cosd(-rol);
	RT[1][0] = 0;					 RT[1][1] =cosd(-rol) ;						 RT[1][2] = -sind(-rol);
	RT[2][0] = -sind(-pit);RT[2][1] =cosd(-pit)*sind(-rol) ; RT[2][2] = cosd(-pit)*cosd(-rol);
	
	x = RT[0][0]*xn + RT[0][1]*yn + RT[0][2]*zn;
	y = RT[1][0]*xn + RT[1][1]*yn + RT[1][2]*zn;
	z = RT[2][0]*xn + RT[2][1]*yn + RT[2][2]*zn;
	
	*xb=x*cosd(yaw)-y*sind(yaw);
	*yb=x*sind(yaw)+y*cosd(yaw);
	*zb=z;
}

uint8_t id[3] ;
// Reads the 3 accelerometer channels and stores them in vector a
xyz_s16_t data_acc;
void LSM6_readAcc(u8 fast)
{u8 buffer[6];
	SPI_BufferRead(buffer, OUTX_L_XL, 6, MPU9250);

  data_acc.x = (buffer[1] << 8) | buffer[0];
	data_acc.y = (buffer[3] << 8) | buffer[2];
	data_acc.z = (buffer[5] << 8) | buffer[4];
	
	if(abs(data_acc.x)<6000&&abs(data_acc.y)<6000&&abs(data_acc.z)<6000)
	{	
	lis3mdl.Acc_I16.x=data_acc.y;
	lis3mdl.Acc_I16.y=-data_acc.x;
	lis3mdl.Acc_I16.z=data_acc.z;		

   converRT(lis3mdl.Acc_I16.x,lis3mdl.Acc_I16.y,lis3mdl.Acc_I16.z,
		&lis3mdl_cov.Acc_I16.x,&lis3mdl_cov.Acc_I16.y,&lis3mdl_cov.Acc_I16.z);
  }
}


// Reads the 3 gyro channels and stores them in vector g
  xyz_s16_t data_g;
void LSM6_readGyro(u8 fast)
{
	u8 buffer[6];
	SPI_BufferRead(buffer, OUTX_L_G, 6, MPU9250);

	data_g.x = (buffer[1] << 8) | buffer[0];
	data_g.y = (buffer[3] << 8) | buffer[2];
	data_g.z = (buffer[5] << 8) | buffer[4];
	if(abs(data_g.x)<6000&&abs(data_g.y)<6000&&abs(data_g.z)<6000)
	{
	lis3mdl.Gyro_I16.x=data_g.y;
	lis3mdl.Gyro_I16.y=-data_g.x;
	lis3mdl.Gyro_I16.z=data_g.z;	
		
	converRT(lis3mdl.Gyro_I16.x,lis3mdl.Gyro_I16.y,lis3mdl.Gyro_I16.z,
		&lis3mdl_cov.Gyro_I16.x,&lis3mdl_cov.Gyro_I16.y,&lis3mdl_cov.Gyro_I16.z);
	}
}


void DS33_Init(void)
{
//---------------init acc & gro		
		Lis3mdl_SPI_WR(0x21,0x04,MPU9250);
		Delay_ms(10);
	  Lis3mdl_SPI_WR(CTRL1_XL, 0x4f,MPU9250);Delay_ms(10);
	  Lis3mdl_SPI_WR(CTRL2_G, 0x4c,MPU9250);Delay_ms(10);
	  Lis3mdl_SPI_WR(CTRL3_C, 0x04,MPU9250);Delay_ms(10);
		
		Delay_ms(10);
		SPI_CS(MPU9250,0);
		Spi_RW(0x80|0x0f);
		u8 l_u8_ID1= Spi_RW(0xFF);
	  SPI_CS(MPU9250,1);
		id[1] = l_u8_ID1;
		if(DS33_WHO_AM_I_ID==l_u8_ID1)
			 module.acc_imu =module.gyro_imu= 1; 

		LSM6_readGyro(0);
}
//--------------------------HML  LIS------------------------			
#define LIS3MDL_SA1_HIGH_ADDRESS  0x1E
#define LIS3MDL_SA1_LOW_ADDRESS   0x1C
#define LIS3MDL_ADDRESS1  (LIS3MDL_SA1_HIGH_ADDRESS << 1)
#define LIS3MDL_ADDRESS2  (LIS3MDL_SA1_LOW_ADDRESS << 1)
#define TEST_REG_ERROR -1
#define LIS3MDL_WHO_ID  0x3D
#define LIS3MDL_IIC_ID LIS3MDL_ADDRESS1

void LIS3MDL_Init(void)
{  
	  Lis3mdl_SPI_WR(CTRL_REG1, 0x74,CS_LIS);
	  Lis3mdl_SPI_WR(CTRL_REG2, 0x60,CS_LIS);
	  Lis3mdl_SPI_WR(CTRL_REG3, 0x00,CS_LIS);
	  Lis3mdl_SPI_WR(CTRL_REG4, 0x0C,CS_LIS);
	  SPI_CS(CS_LIS,0);
	  Spi_RW(0x80|0x0f);
	  u8 l_u8_ID= Spi_RW(0xFF);
	  
		if(l_u8_ID==LIS3MDL_WHO_ID)
			 module.hml_imu =1; 
		SPI_CS(CS_LIS,1);
}

void LIS3MDL_read(void)
{ u8 buffer[6];
	static u8 temp=0;
	static u8 cnt;
	xyz_s16_t data;
	SPI_BufferRead(buffer, OUT_X_L, 6, CS_LIS);
 
   data.x= (buffer[1] << 8) | buffer[0];
   data.y= (buffer[3] << 8) | buffer[2];
   data.z= (buffer[5] << 8) | buffer[4];
	
	 if(abs(data.x)<2500&&abs(data.y)<2500&&abs(data.z)<2500)
	 {
	   lis3mdl.Mag_Adc.x=data.x;
		 lis3mdl.Mag_Adc.y=data.y;
		 lis3mdl.Mag_Adc.z=data.z;
	 }
	 if(lis3mdl.Mag_Adc.z==0&&module.hml_imu)
		 cnt++;
	 else
		 cnt=0;
	 if(cnt>244)
		 module.hml_imu=0;
}

//-----------------------------------------
int8_t spi2_read_write_byte(uint8_t TxData)
{
	uint8_t Rxdata;
	Rxdata=Spi_RW(TxData);
  //HAL_SPI_TransmitReceive(&hspi2,&TxData,&Rxdata,1,0xffff);     
 	return Rxdata;   	
}

uint8_t spi2_write_reg(uint8_t reg_addr,uint8_t reg_val)
{
	spi2_read_write_byte(reg_addr&0x7f);
	spi2_read_write_byte(reg_val);
	return 0;
}


uint8_t spi2_read_reg(uint8_t reg_addr)
{
	spi2_read_write_byte(reg_addr|0x80);
	return spi2_read_write_byte(0xff);
}

uint8_t spi2_read_reg_buffer(uint8_t reg_addr,void *buffer,uint16_t len)
{
	uint8_t *p = buffer;
	uint16_t i;
	spi2_read_write_byte(reg_addr|0x80);
	for(i=0;i<len;i++)
	{
		*p++= spi2_read_write_byte(0xff);
	}
	return 0;
}

static 	float _accel_scale;
static	float _gyro_scale;
#define ICM20602_ADDRESS	0xD2		

#define ICM_CS_Enable    SPI_CS(MPU9250,0)//HAL_GPIO_WritePin(GPIOB, ICM_CS_Pin, GPIO_PIN_RESET)  //在SPI总线上选中ICM
#define ICM_CS_Disable   SPI_CS(MPU9250,1)//HAL_GPIO_WritePin(GPIOB, ICM_CS_Pin, GPIO_PIN_SET)	   //失能ICM的CS


/**
  * @brief  往寄存器写
  * @param  寄存器/值
  * @retval 无
  */
uint8_t icm20602_write_reg(uint8_t reg,uint8_t val)
{
	//return myiic_write_reg(ICM20602_ADDRESS,reg,val);
	ICM_CS_Enable;
	spi2_write_reg(reg,val);
	ICM_CS_Disable;
	return 0;
}

/**
  * @brief  读寄存器
  * @param  寄存器
  * @retval 无
  */
uint8_t icm20602_read_reg(uint8_t reg)
{
	uint8_t res;
	//return myiic_read_reg(ICM20602_ADDRESS,reg);
	ICM_CS_Enable;
	res = spi2_read_reg(reg);
	ICM_CS_Disable;
	return res;
}

/**
  * @brief  读寄存器区域指定长度值
  * @param  寄存器/
  * @retval 无
  */
uint8_t icm20602_read_buffer(uint8_t reg,void *buffer,uint8_t len)
{
	//return myiic_read_buffer(ICM20602_ADDRESS,reg,len,buffer);
	ICM_CS_Enable;
	spi2_read_reg_buffer(reg,buffer,len);
	ICM_CS_Disable;
	return 0;
}

/**
  * @brief  ICM20602初始化
  * @param  无
  * @retval 无
  */
uint8_t icm20602_init()
{
	uint8_t id;
	icm20602_write_reg(ICM20_PWR_MGMT_1,0x80);	//复位，复位后位0x41,睡眠模式，
	Delay_ms(10);
	icm20602_write_reg(ICM20_PWR_MGMT_1,0x01);		//关闭睡眠，自动选择时钟
	Delay_ms(10);
	id = icm20602_read_reg(ICM20_WHO_AM_I);//读取ID
	//printf("icm_20602 id=0x%x\r\n",id);
	if(id != 0x12)
	{
		//printf("icm_20602 id error !!!\r\n");
		return 1;	
	}
	module.acc_imu =module.gyro_imu= 1; 
	//printf("icm20602 init pass\r\n\r\n");
	icm20602_write_reg(ICM20_PWR_MGMT_2, 0x00);
	Delay_ms(10);	
	icm20602_write_reg(ICM20_SMPLRT_DIV,0);			
	Delay_ms(10);
	icm20602_write_reg(ICM20_CONFIG,DLPF_BW_92);//DLPF_BW_20);	
	Delay_ms(10);
	icm20602_write_reg(ICM20_ACCEL_CONFIG2,ACCEL_AVER_4|ACCEL_DLPF_BW_21);//ACCEL_AVER_4|ACCEL_DLPF_BW_21);	
	Delay_ms(10);
	
	//设置量程
	icm20602_set_accel_fullscale(ICM20_ACCEL_FS_8G);
	Delay_ms(10);
	icm20602_set_gyro_fullscale(ICM20_GYRO_FS_2000);
	Delay_ms(10);
	
	icm20602_write_reg(ICM20_LP_MODE_CFG, 0x00);	
	Delay_ms(10);
	icm20602_write_reg(ICM20_FIFO_EN, 0x00);		
	Delay_ms(10);
	return 0;
}




//ICM20_ACCEL_FS_2G
//ICM20_ACCEL_FS_4G
//ICM20_ACCEL_FS_8G
//ICM20_ACCEL_FS_16G
uint8_t icm20602_set_accel_fullscale(uint8_t fs)
{
	switch(fs)
	{
		case ICM20_ACCEL_FS_2G:
			_accel_scale = 1.0f/16348.0f;
		break;
		case ICM20_ACCEL_FS_4G:
			_accel_scale = 1.0f/8192.0f;
		break;
		case ICM20_ACCEL_FS_8G:
			_accel_scale = 1.0f/4096.0f;
		break;
		case ICM20_ACCEL_FS_16G:
			_accel_scale = 1.0f/2048.0f;
		break;
		default:
			fs = ICM20_ACCEL_FS_8G;
			_accel_scale = 1.0f/4096.0f;
		break;

	}
	_accel_scale *= GRAVITY_MSS;
	return icm20602_write_reg(ICM20_ACCEL_CONFIG,fs);
}



//ICM20_GYRO_FS_250
//ICM20_GYRO_FS_500
//ICM20_GYRO_FS_1000
//ICM20_GYRO_FS_2000
uint8_t icm20602_set_gyro_fullscale(uint8_t fs)
{
	switch(fs)
	{
		case ICM20_GYRO_FS_250:
			_gyro_scale = 1.0f/131.068f;	//32767/250
		break;
		case ICM20_GYRO_FS_500:
			_gyro_scale = 1.0f/65.534f;
		break;
		case ICM20_GYRO_FS_1000:
			_gyro_scale = 1.0f/32.767f;
		break;
		case ICM20_GYRO_FS_2000:
			_gyro_scale = 1.0f/16.3835f;
		break;
		default:
			fs = ICM20_GYRO_FS_2000;
			_gyro_scale = 1.0f/16.3835f;
		break;

	}
	_gyro_scale *= _DEG_TO_RAD;
	return icm20602_write_reg(ICM20_GYRO_CONFIG,fs);
	
}

//PIT向下 y- ROL向右x- ACC
//PIT向下 x-600  ROL向右  y+300  YAW顺-z
uint8_t icm20602_get_accel_adc(void)
{
	uint8_t buf[6];
	if(icm20602_read_buffer(ICM20_ACCEL_XOUT_H,buf,6))return 1;
	
	data_acc.x= ((int16_t)buf[0]<<8) + buf[1];
	data_acc.y= ((int16_t)buf[2]<<8) + buf[3];
	data_acc.z= ((int16_t)buf[4]<<8) + buf[5];
	if(abs(data_acc.x)<6000&&abs(data_acc.y)<6000&&abs(data_acc.z)<6000)
	{	
		
	#if MCU_BIND1
	lis3mdl.Acc_I16.x=data_acc.y;
	lis3mdl.Acc_I16.y=-data_acc.z;
	lis3mdl.Acc_I16.z=-data_acc.x;		
	#elif MCU_BIND2
	lis3mdl.Acc_I16.y=-data_acc.z;
	lis3mdl.Acc_I16.z=data_acc.x;
	lis3mdl.Acc_I16.x=-data_acc.y;			
	#elif MCU_BIND3
	lis3mdl.Acc_I16.y=data_acc.z;
	lis3mdl.Acc_I16.x=-data_acc.x;
	lis3mdl.Acc_I16.z=data_acc.y;		
	#elif MCU_TINYPALE
	lis3mdl.Acc_I16.y=-data_acc.z;
	lis3mdl.Acc_I16.x=-data_acc.y;
	lis3mdl.Acc_I16.z=data_acc.x;			
	#elif MCU_TINKER
	lis3mdl.Acc_I16.x=-data_acc.x;
	lis3mdl.Acc_I16.y=data_acc.z;
	lis3mdl.Acc_I16.z=data_acc.y;	
	#else
	lis3mdl.Acc_I16.y=data_acc.y;
	lis3mdl.Acc_I16.x=data_acc.x;
	lis3mdl.Acc_I16.z=data_acc.z;		
	#endif
   converRT(lis3mdl.Acc_I16.x,lis3mdl.Acc_I16.y,lis3mdl.Acc_I16.z,
		&lis3mdl_cov.Acc_I16.x,&lis3mdl_cov.Acc_I16.y,&lis3mdl_cov.Acc_I16.z);
  }
	return 0;
}


uint8_t icm20602_get_gyro_adc(void)
{
	uint8_t buf[6];
	if(icm20602_read_buffer(ICM20_GYRO_XOUT_H,buf,6))return 1;
	data_g.x =(buf[0]<<8) + buf[1];
	data_g.y =(buf[2]<<8) + buf[3];
	data_g.z =(buf[4]<<8) + buf[5];

	if(abs(data_g.x)<6000&&abs(data_g.y)<6000&&abs(data_g.z)<6000)
	{
	#if MCU_BIND1
	lis3mdl.Gyro_I16.x=data_g.y;
	lis3mdl.Gyro_I16.y=-data_g.z;
	lis3mdl.Gyro_I16.z=-data_g.x;	
	#elif MCU_BIND2
	lis3mdl.Gyro_I16.x=-data_g.y;
	lis3mdl.Gyro_I16.y=-data_g.z;
	lis3mdl.Gyro_I16.z=data_g.x;	
  #elif MCU_BIND3
	lis3mdl.Gyro_I16.x=-data_g.x;
	lis3mdl.Gyro_I16.y=data_g.z;
	lis3mdl.Gyro_I16.z=data_g.y;
	#elif MCU_TINYPALE		
	lis3mdl.Gyro_I16.x=-data_g.y;
	lis3mdl.Gyro_I16.y=-data_g.z;
	lis3mdl.Gyro_I16.z=data_g.x;	
	#elif MCU_TINKER
	lis3mdl.Gyro_I16.x=-data_g.x;
	lis3mdl.Gyro_I16.y=data_g.z;
	lis3mdl.Gyro_I16.z=data_g.y;	
	#else
	lis3mdl.Gyro_I16.x=-data_g.x;
	lis3mdl.Gyro_I16.y=data_g.y;
	lis3mdl.Gyro_I16.z=data_g.z;	
	#endif
	converRT(lis3mdl.Gyro_I16.x,lis3mdl.Gyro_I16.y,lis3mdl.Gyro_I16.z,
		&lis3mdl_cov.Gyro_I16.x,&lis3mdl_cov.Gyro_I16.y,&lis3mdl_cov.Gyro_I16.z);
	}
	return 0;
}

float icm20602_get_temp()
{
	int16_t temp_adc;
	uint8_t buf[2];
	if(icm20602_read_buffer(ICM20_TEMP_OUT_H,buf,2))return 0.0f;

	temp_adc = (buf[0]<<8)+buf[1];

	return (25.0f + (float)temp_adc/326.8f);
}

