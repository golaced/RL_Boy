
#include "include.h"
#include "flash.h"
#include "mems.h"	
#include "flash_w25.h"
#include "icm20602.h"	
#include "mavl.h"
#include "gait_math.h"
#include "can.h"
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}

void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 


void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}
int flash_cnt=0;
float isnan_checkf(float in)
{
 if(isnan(in))
   return 0;
 else 
	 return in;
}

char isnan_checkc(char in)
{
 if(isnan(in))
   return 0;
 else 
	 return in;
}

int isnan_checki(int in)
{
 if(isnan(in))
   return 0;
 else 
	 return in;
}
//编码
static void setDataIntf(char * FLASH_Buffer,int i)
{
	*(FLASH_Buffer+flash_cnt++) = ((i << 24) >> 24);
	*(FLASH_Buffer+flash_cnt++) = ((i << 16) >> 24);
	*(FLASH_Buffer+flash_cnt++) = ((i << 8) >> 24);
	*(FLASH_Buffer+flash_cnt++) = (i >> 24);
}

static void setDataFloatf(char * FLASH_Buffer,float f)
{
	int i = *(int *)&f;
	*(FLASH_Buffer+flash_cnt++) = ((i << 24) >> 24);
	*(FLASH_Buffer+flash_cnt++) = ((i << 16) >> 24);
	*(FLASH_Buffer+flash_cnt++) = ((i << 8) >> 24);
	*(FLASH_Buffer+flash_cnt++) = (i >> 24);
}
//解码
static float floatFromDataf(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	float out=0;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	
	 out=isnan_checkf(*(float *)&i);
	return out;
}

static char charFromDataf(unsigned char *data,int* anal_cnt)
{
	char out=0;
	int temp=*anal_cnt ;
	*anal_cnt +=1;
	 out=*(data+temp);
	return isnan_checkc(out);
}

static int intFromDataf(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	*anal_cnt +=4;
	return isnan_checki(i);
}


//-----------------------------------------存储参数
#define SIZE_PARAM 50*10
u32 FLASH_SIZE=16*1024*1024;	//FLASH 大小为16字节

u8 need_init_mems=0;//mems flash error
u16 SBUS_MIN =868;
u16 SBUS_MAX =2180;
u16 SBUS_MID =1524;
u16 SBUS_MIN_A =644;//954;
u16 SBUS_MAX_A =2484;//2108;
u16 SBUS_MID_A =1524;
char flash_rd_end=0;
void READ_PARM(void)
{
u8 FLASH_Buffer[SIZE_PARAM]={0};	
u8 need_init=0;	
int i=0,j=0,temp=0;
int anal_cnt=0;
module.flash_lock=1;
#if FLASH_USE_STM32
STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)FLASH_Buffer,SIZE);	
#else	
W25QXX_Read(FLASH_Buffer,FLASH_SIZE-(SIZE_PARAM+10),SIZE_PARAM);					//从倒数第100个地址处开始,读出SIZE个字节
#endif
module.flash_lock=0;
mems.Gyro_Offset.x=intFromDataf(FLASH_Buffer,&anal_cnt);
mems.Gyro_Offset.y=intFromDataf(FLASH_Buffer,&anal_cnt);
mems.Gyro_Offset.z=intFromDataf(FLASH_Buffer,&anal_cnt);

mems.Acc_Offset.x=intFromDataf(FLASH_Buffer,&anal_cnt);
mems.Acc_Offset.y=intFromDataf(FLASH_Buffer,&anal_cnt);
mems.Acc_Offset.z=intFromDataf(FLASH_Buffer,&anal_cnt);

mems.Mag_Offset.x=intFromDataf(FLASH_Buffer,&anal_cnt);
mems.Mag_Offset.y=intFromDataf(FLASH_Buffer,&anal_cnt);
mems.Mag_Offset.z=intFromDataf(FLASH_Buffer,&anal_cnt);

mems.Mag_Gain.x=floatFromDataf(FLASH_Buffer,&anal_cnt);
mems.Mag_Gain.y=floatFromDataf(FLASH_Buffer,&anal_cnt);
mems.Mag_Gain.z=floatFromDataf(FLASH_Buffer,&anal_cnt);
		
if(fabs(mems.Mag_Gain.x)<10&&mems.Mag_Gain.x!=-1&&fabs(mems.Mag_Offset.x)<666&&mems.Mag_Offset.x!=-1
		&&mems.Mag_Gain.x!=0&&mems.Mag_Gain.y!=0&&mems.Mag_Gain.z!=0)
	mems.Mag_Have_Param=1;

mems.Mag_Offseto.x=intFromDataf(FLASH_Buffer,&anal_cnt);
mems.Mag_Offseto.y=intFromDataf(FLASH_Buffer,&anal_cnt);
mems.Mag_Offseto.z=intFromDataf(FLASH_Buffer,&anal_cnt);

mems.Mag_Gaino.x=floatFromDataf(FLASH_Buffer,&anal_cnt);
mems.Mag_Gaino.y=floatFromDataf(FLASH_Buffer,&anal_cnt);
mems.Mag_Gaino.z=floatFromDataf(FLASH_Buffer,&anal_cnt);

vmc_all.tar_att_bias[PITr]=floatFromDataf(FLASH_Buffer,&anal_cnt);
vmc_all.tar_att_bias[ROLr]=floatFromDataf(FLASH_Buffer,&anal_cnt);

temp=intFromDataf(FLASH_Buffer,&anal_cnt);
vmc_all.your_key[0]=temp/10000;
vmc_all.your_key[1]=(temp-vmc_all.your_key[0]*10000)/100;
vmc_all.your_key[2]=(temp-vmc_all.your_key[0]*10000-vmc_all.your_key[1]*100);
//------------
for(i=0;i<14;i++){
	motor_chassis[i].param.q_reset_angle=floatFromDataf(FLASH_Buffer,&anal_cnt);
	motor_chassis[i].param.t_inv_flag_cmd=intFromDataf(FLASH_Buffer,&anal_cnt);
	motor_chassis[i].param.t_inv_flag_measure=intFromDataf(FLASH_Buffer,&anal_cnt);
	motor_chassis[i].param.q_flag=intFromDataf(FLASH_Buffer,&anal_cnt);	
	motor_chassis[i].motor.type=intFromDataf(FLASH_Buffer,&anal_cnt);	
	motor_chassis[i].param.control_mode=intFromDataf(FLASH_Buffer,&anal_cnt);	
}

flash_rd_end=charFromDataf(FLASH_Buffer,&anal_cnt);

//异常处理
for(i=0;i<14;i++){
	if(motor_chassis[i].motor.type>30){
		motor_chassis[i].param.t_inv_flag_measure=1;
		motor_chassis[i].param.t_inv_flag_cmd=1;
		motor_chassis[i].param.q_flag=1;	
		motor_chassis[i].motor.type=DM_J4310;
		WRITE_PARM();
	}
}

if(module.hml_imu_o&&mems.Mag_Have_Param)
	mems.Mag_Have_Param=0;
if(fabs(mems.Mag_Gaino.x)<10&&mems.Mag_Gaino.x!=-1&&fabs(mems.Mag_Offseto.x)<666&&mems.Mag_Offseto.x!=-1
	&&mems.Mag_Gaino.x!=0&&mems.Mag_Gaino.y!=0&&mems.Mag_Gaino.z!=0)
	mems.Mag_Have_Param=1;

	if(SBUS_MIN==65535)
	{
	SBUS_MIN=860;
	SBUS_MID=1524;
	SBUS_MAX=2180;

	SBUS_MIN_A=644;
	SBUS_MID_A=1524;
	SBUS_MAX_A=2484;	
	}	
}

void WRITE_PARM(void)
{ 

int16_t _temp;
u8 cnt=0,i;
char FLASH_Buffer[SIZE_PARAM]={0};
flash_cnt=0;
setDataIntf(FLASH_Buffer,mems.Gyro_Offset.x);
setDataIntf(FLASH_Buffer,mems.Gyro_Offset.y);
setDataIntf(FLASH_Buffer,mems.Gyro_Offset.z);
setDataIntf(FLASH_Buffer,mems.Acc_Offset.x);
setDataIntf(FLASH_Buffer,mems.Acc_Offset.y);
setDataIntf(FLASH_Buffer,mems.Acc_Offset.z);
setDataIntf(FLASH_Buffer,mems.Mag_Offset.x);
setDataIntf(FLASH_Buffer,mems.Mag_Offset.y);
setDataIntf(FLASH_Buffer,mems.Mag_Offset.z);
setDataFloatf(FLASH_Buffer,mems.Mag_Gain.x);
setDataFloatf(FLASH_Buffer,mems.Mag_Gain.y);
setDataFloatf(FLASH_Buffer,mems.Mag_Gain.z);
setDataIntf(FLASH_Buffer,mems.Mag_Offseto.x);
setDataIntf(FLASH_Buffer,mems.Mag_Offseto.y);
setDataIntf(FLASH_Buffer,mems.Mag_Offseto.z);
setDataFloatf(FLASH_Buffer,mems.Mag_Gaino.x);
setDataFloatf(FLASH_Buffer,mems.Mag_Gaino.y);
setDataFloatf(FLASH_Buffer,mems.Mag_Gaino.z);

setDataFloatf(FLASH_Buffer,vmc_all.tar_att_bias[PITr]);
setDataFloatf(FLASH_Buffer,vmc_all.tar_att_bias[ROLr]);
setDataIntf(FLASH_Buffer,vmc_all.your_key[0]*10000+vmc_all.your_key[1]*100+vmc_all.your_key[2]);

//--------
for(i=0;i<14;i++)
{
	setDataFloatf(FLASH_Buffer,motor_chassis[i].param.q_reset_angle);
	setDataIntf(FLASH_Buffer,motor_chassis[i].param.t_inv_flag_cmd);
	setDataIntf(FLASH_Buffer,motor_chassis[i].param.t_inv_flag_measure);
	setDataIntf(FLASH_Buffer,motor_chassis[i].param.q_flag);
	setDataIntf(FLASH_Buffer,motor_chassis[i].motor.type);
	setDataIntf(FLASH_Buffer,motor_chassis[i].param.control_mode);
}
FLASH_Buffer[flash_cnt++]=99;

//参数
module.flash_lock=1;
#if FLASH_USE_STM32
STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)FLASH_Buffer,SIZE);
#else
W25QXX_Write((u8*)FLASH_Buffer,FLASH_SIZE-(SIZE_PARAM+10),SIZE_PARAM);		//从倒数第100个地址处开始,写入SIZE长度的数据
#endif
module.flash_lock=0;
}


#define FRAM_SIZE 16
#define SIZE_WAY FRAM_SIZE*(NAV_MAX_MISSION_LEGS+1)
void WRITE_PARM_WAY_POINTS(void)
{ 
int16_t _temp;
int32_t	_temp32;
u16 cnt=0,i;
u8 FLASH_Bufferw[SIZE_WAY]={0};		
int max_num=LIMIT(SIZE_WAY/FRAM_SIZE-2,0,NAV_MAX_MISSION_LEGS);
FLASH_Bufferw[cnt++]=navData.Leg_num;
for(i=0;i<LIMIT(navData.Leg_num,0,max_num);i++){
_temp32=(int32_t)(navData.missionLegs[i].targetLat*10000000);
FLASH_Bufferw[cnt++]=BYTE0(_temp32);
FLASH_Bufferw[cnt++]=BYTE1(_temp32);
FLASH_Bufferw[cnt++]=BYTE2(_temp32);
FLASH_Bufferw[cnt++]=BYTE3(_temp32);
_temp32=(int32_t)(navData.missionLegs[i].targetLon*10000000);
FLASH_Bufferw[cnt++]=BYTE0(_temp32);
FLASH_Bufferw[cnt++]=BYTE1(_temp32);
FLASH_Bufferw[cnt++]=BYTE2(_temp32);
FLASH_Bufferw[cnt++]=BYTE3(_temp32);
_temp=(int16_t)(navData.missionLegs[i].targetAlt*10);
FLASH_Bufferw[cnt++]=BYTE0(_temp);
FLASH_Bufferw[cnt++]=BYTE1(_temp);
_temp=(int16_t)(navData.missionLegs[i].poiHeading*1000);
FLASH_Bufferw[cnt++]=BYTE0(_temp);
FLASH_Bufferw[cnt++]=BYTE1(_temp);
_temp=(int16_t)(navData.missionLegs[i].maxHorizSpeed*100);
FLASH_Bufferw[cnt++]=BYTE0(_temp);
FLASH_Bufferw[cnt++]=BYTE1(_temp);
_temp=(int16_t)(navData.missionLegs[i].loiterTime*100);
FLASH_Bufferw[cnt++]=BYTE0(_temp);
FLASH_Bufferw[cnt++]=BYTE1(_temp);
}
W25QXX_Write((u8*)FLASH_Bufferw,FLASH_SIZE-(SIZE_WAY+10+SIZE_PARAM+10),SIZE_WAY);		//从倒数第100个地址处开始,写入SIZE长度的数据
}

void READ_WAY_POINTS(void)
{
u16 i;
u8 FLASH_Bufferw[SIZE_WAY]={0};	
W25QXX_Read(FLASH_Bufferw,FLASH_SIZE-(SIZE_WAY+10+SIZE_PARAM+10),SIZE_WAY);					//从倒数第100个地址处开始,读出SIZE个字节
navData.Leg_num=LIMIT(FLASH_Bufferw[0],0,NAV_MAX_MISSION_LEGS);
for(i=0;i<navData.Leg_num;i++){
navData.missionLegs[i].targetLat=(float)((vs32)(FLASH_Bufferw[4+i*FRAM_SIZE]<<24|FLASH_Bufferw[3+i*FRAM_SIZE]<<16|
	FLASH_Bufferw[2+i*FRAM_SIZE]<<8|FLASH_Bufferw[1+i*FRAM_SIZE]))/10000000.;
navData.missionLegs[i].targetLon=(float)((vs32)(FLASH_Bufferw[8+i*FRAM_SIZE]<<24|FLASH_Bufferw[7+i*FRAM_SIZE]<<16|
	FLASH_Bufferw[6+i*FRAM_SIZE]<<8|FLASH_Bufferw[5+i*FRAM_SIZE]))/10000000.;
navData.missionLegs[i].targetAlt=(float)((vs16)(FLASH_Bufferw[10+i*FRAM_SIZE]<<8|FLASH_Bufferw[9+i*FRAM_SIZE]))/10.;
navData.missionLegs[i].poiHeading=(float)((vs16)(FLASH_Bufferw[12+i*FRAM_SIZE]<<8|FLASH_Bufferw[11+i*FRAM_SIZE]))/1000.;
navData.missionLegs[i].maxHorizSpeed=(float)((vs16)(FLASH_Bufferw[14+i*FRAM_SIZE]<<8|FLASH_Bufferw[13+i*FRAM_SIZE]))/100.;
navData.missionLegs[i].loiterTime=(float)((vs16)(FLASH_Bufferw[16+i*FRAM_SIZE]<<8|FLASH_Bufferw[15+i*FRAM_SIZE]))/100.;
	if(navData.missionLegs[i].targetLat>20&&navData.missionLegs[i].targetLon>20)
		navData.missionLegs[i].type=2;
	}
}