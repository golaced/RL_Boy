#include "usbd_cdc_vcp.h" 
#include "string.h"	
#include "stdarg.h"		 
#include "stdio.h"	 

#include "include.h" 
#include "bat.h"
#include "imu.h"
#include "flash.h"
#include "led_fc.h"
#include "sbus.h"
#include "rc_mine.h"
#include "dog.h"
#include "iic_vl53.h"
#include "usart_fc.h"
#include "pwm_out.h"
#include "beep.h"
#include "nav.h"
#include "ms5611.h"
#include "bat.h"
#include "mavl.h"
#include "spi.h"
#include "mems.h"
#include "can.h"
#include "locomotion_header.h"

char can_master_connect=0;
char can_master_connect_has=0;
int  can_master_loss=0;
int  can_node_id=0;
int  can_write_flash=0;
int  can_trans_switch=0;
int  can_cmd_usb_disable=0;
int  ocu_connect=0;
float  usb_loss =0;
float ocu_loss_cnt=0;

//USB虚拟串口相关配置参数
LINE_CODING linecoding =
{
	2000000,//5000000,//576000,		//波特率
	0x01,   	//停止位,默认1位
	0x00,   	//校验位,默认无
	0x08    	//数据位,默认8位
}; 

u8  USART_PRINTF_Buffer[USB_USART_REC_LEN];	//usb_printf发送缓冲区 

//用类似串口1接收数据的方法,来处理USB虚拟串口接收到的数据.
u8 USB_USART_RX_BUF[USB_USART_REC_LEN]; 	//接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USB_USART_RX_STA=0;       				//接收状态标记	 

 
extern uint8_t  APP_Rx_Buffer [];			//虚拟串口发送缓冲区(发给电脑) 
extern uint32_t APP_Rx_ptr_in;   			//虚拟串口接收缓冲区(接收来自电脑的数据)

//虚拟串口配置函数(供USB内核调用)
CDC_IF_Prop_TypeDef VCP_fops = 
{
	VCP_Init,
	VCP_DeInit,
	VCP_Ctrl,
	VCP_DataTx,
	VCP_DataRx
}; 

//初始化VCP
//返回值:USBD_OK
uint16_t VCP_Init(void)
{ 
	return USBD_OK;
} 
//复位VCP
//返回值:USBD_OK
uint16_t VCP_DeInit(void)
{ 
	return USBD_OK;
} 
//控制VCP的设置
//buf:命令数据缓冲区/参数保存缓冲区
//len:数据长度
//返回值:USBD_OK
uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{ 
	switch (Cmd)
	{
		case SEND_ENCAPSULATED_COMMAND:break;   
		case GET_ENCAPSULATED_RESPONSE:break;  
		case SET_COMM_FEATURE:break;  
		case GET_COMM_FEATURE:break;  
 		case CLEAR_COMM_FEATURE:break;  
		case SET_LINE_CODING:
			linecoding.bitrate = 	2000000;//(uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));
			linecoding.format = Buf[4];
			linecoding.paritytype = Buf[5];
			linecoding.datatype = Buf[6]; 
			//打印配置参数
			//printf("linecoding.format:%d\r\n",linecoding.format);
			//printf("linecoding.paritytype:%d\r\n",linecoding.paritytype);
			//printf("linecoding.datatype:%d\r\n",linecoding.datatype);
			//printf("linecoding.bitrate:%d\r\n",linecoding.bitrate);
			break; 
		case GET_LINE_CODING:
			Buf[0] = (uint8_t)(linecoding.bitrate);
			Buf[1] = (uint8_t)(linecoding.bitrate >> 8);
			Buf[2] = (uint8_t)(linecoding.bitrate >> 16);
			Buf[3] = (uint8_t)(linecoding.bitrate >> 24);
			Buf[4] = linecoding.format;
			Buf[5] = linecoding.paritytype;
			Buf[6] = linecoding.datatype; 
			break; 
		case SET_CONTROL_LINE_STATE:break;   
		case SEND_BREAK:break;   
		default:break;  
	} 
	return USBD_OK;
}
//发送一个字节给虚拟串口(发给电脑)
//data:要发送的数据
//返回值:USBD_OK
uint16_t VCP_DataTx (uint8_t data)
{  
	APP_Rx_Buffer[APP_Rx_ptr_in]=data;	//写入发送buf
	APP_Rx_ptr_in++;  					//写位置加1
	if(APP_Rx_ptr_in==APP_RX_DATA_SIZE)	//超过buf大小了,归零.
	{
		APP_Rx_ptr_in = 0;
	}   
	return USBD_OK;
} 

uint16_t VCP_DataTxBuf (uint8_t *data,u16 len)
{  
	u16 i=0;
	for(i=0;i<len;i++){
		APP_Rx_Buffer[APP_Rx_ptr_in]=data[i];	//写入发送buf
		APP_Rx_ptr_in++;  					//写位置加1
		if(APP_Rx_ptr_in==APP_RX_DATA_SIZE)	//超过buf大小了,归零.
		{
			APP_Rx_ptr_in = 0;
		}   
	}
	return USBD_OK;
} 
//处理从USB虚拟串口接收到的数据
//databuffer:数据缓存区
//Nb_bytes:接收到的字节数.
//返回值:USBD_OK
unsigned char RxBuffer_USB[255];
float temp_time=0;
int cnt_rx_mode=0;
int cnt_usb_rx[128]={0};
void Anal_USB(u8 *data_buf,u8 num)//jiema
{ static u8 cnt[4];
	float temp;
	u8 id;
	char temp_char=0,need_save=0;
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i,j;
	int anal_cnt=4;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1))){		
		i=0;
		return;		//判断sum
	}
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))	{
		i=0;
		return;		//判断帧头
	}
  if(*(data_buf+2)==0x01)//robot config moco配置
  { 
		cnt_rx_mode++;
		cnt_usb_rx[*(data_buf+2)]++;
		vmc_all.param.param_save=charFromData(data_buf,&anal_cnt);
		vmc_all.your_key_all=intFromData(data_buf,&anal_cnt);
		vmc_all.param.robot_type=charFromData(data_buf,&anal_cnt);
		vmc_all.param.cmd_use_ocu=charFromData(data_buf,&anal_cnt);
		vmc_all.param.en_gait_switch=charFromData(data_buf,&anal_cnt);
		vmc_all.param.en_fall_protect=charFromData(data_buf,&anal_cnt);
		vmc_all.param.rc_type=charFromData(data_buf,&anal_cnt);
		vmc_all.param.dj_type=charFromData(data_buf,&anal_cnt);

		vmc_all.W=floatFromData(data_buf,&anal_cnt);
		vmc_all.H=floatFromData(data_buf,&anal_cnt);
		vmc_all.mess=floatFromData(data_buf,&anal_cnt);
		vmc_all.l1=floatFromData(data_buf,&anal_cnt);
		vmc_all.l2=floatFromData(data_buf,&anal_cnt);
		vmc_all.l3=floatFromData(data_buf,&anal_cnt);
		
		vmc_all.param.MAX_Z=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.MIN_Z=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.MAX_X=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.MIN_X=floatFromData(data_buf,&anal_cnt);
	
		vmc_all.param.leg_type=charFromData(data_buf,&anal_cnt);
		vmc[0].param.invert_knee_epos[Xr]=((int)charFromData(data_buf,&anal_cnt)==1?-1:1);
		vmc[1].param.invert_knee_epos[Xr]=((int)charFromData(data_buf,&anal_cnt)==1?-1:1);
		vmc[2].param.invert_knee_epos[Xr]=((int)charFromData(data_buf,&anal_cnt)==1?-1:1);
		vmc[3].param.invert_knee_epos[Xr]=((int)charFromData(data_buf,&anal_cnt)==1?-1:1);
		
		vmc_all.param.param_vmc.leg_off[0]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.leg_off[1]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.side_off[0]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.side_off[1]=floatFromData(data_buf,&anal_cnt);
		
		vmc_all.param.param_vmc.move_com_off[0]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.move_com_off[1]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.move_att_off[0]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.move_att_off[1]=floatFromData(data_buf,&anal_cnt);

//		custom_gait.gait_duty=floatFromData(data_buf,&anal_cnt);
//		custom_gait.high_leg_end=floatFromData(data_buf,&anal_cnt);
//		custom_gait.slip_fp=floatFromData(data_buf,&anal_cnt);
//		custom_gait.slip_kp=floatFromData(data_buf,&anal_cnt);
//		custom_gait.slip_kp_pos=floatFromData(data_buf,&anal_cnt);
		temp_char=floatFromData(data_buf,&anal_cnt);
		temp_char=floatFromData(data_buf,&anal_cnt);
		temp_char=floatFromData(data_buf,&anal_cnt);
		temp_char=floatFromData(data_buf,&anal_cnt);
		temp_char=floatFromData(data_buf,&anal_cnt);
		vmc_all.your_key[0]=vmc_all.your_key_all/10000;
		vmc_all.your_key[1]=(vmc_all.your_key_all-vmc_all.your_key[0]*10000)/100;
		vmc_all.your_key[2]=(vmc_all.your_key_all-vmc_all.your_key[0]*10000-vmc_all.your_key[1]*100);
		//get_license();
		}
	else if(*(data_buf+2)==0x21)//sensor config IMU 传感器标定
  { 
		cnt_usb_rx[*(data_buf+2)]++;
		need_save=charFromData(data_buf,&anal_cnt);//need_save
		temp_char=charFromData(data_buf,&anal_cnt);//cal_mode
		mems.imu_pos.x=floatFromData(data_buf,&anal_cnt);
		mems.imu_pos.y=floatFromData(data_buf,&anal_cnt);
		mems.imu_pos.z=floatFromData(data_buf,&anal_cnt);
		mems.imu_att.x=floatFromData(data_buf,&anal_cnt);
		mems.imu_att.y=floatFromData(data_buf,&anal_cnt);
		mems.imu_att.z=floatFromData(data_buf,&anal_cnt);
		mems.gps_pos.x=floatFromData(data_buf,&anal_cnt);
		mems.gps_pos.y=floatFromData(data_buf,&anal_cnt);
		mems.gps_pos.z=floatFromData(data_buf,&anal_cnt);
		if(need_save){
		if(temp_char==1)
			mems.Acc_CALIBRATE=1;
		else if(temp_char==2)
			mems.Gyro_CALIBRATE=1;
		else if(temp_char==3)
				mems.Mag_CALIBRATE=1;
		}else
		{
		mems.Acc_Offset.x=floatFromData(data_buf,&anal_cnt)*4096;
		mems.Acc_Offset.y=floatFromData(data_buf,&anal_cnt)*4096;
		mems.Acc_Offset.y=floatFromData(data_buf,&anal_cnt)*4096;
		mems.Acc_Scale.x=floatFromData(data_buf,&anal_cnt);
		mems.Acc_Scale.y=floatFromData(data_buf,&anal_cnt);
		mems.Acc_Scale.z=floatFromData(data_buf,&anal_cnt);
	  mems.Gyro_Offset.x=floatFromData(data_buf,&anal_cnt);
		mems.Gyro_Offset.y=floatFromData(data_buf,&anal_cnt);
		mems.Gyro_Offset.z=floatFromData(data_buf,&anal_cnt);		
    mems.Gyro_Scale.x=floatFromData(data_buf,&anal_cnt);
		mems.Gyro_Scale.y=floatFromData(data_buf,&anal_cnt);
		mems.Gyro_Scale.z=floatFromData(data_buf,&anal_cnt);	
		mems.Mag_Offset.x=floatFromData(data_buf,&anal_cnt);
		mems.Mag_Offset.y=floatFromData(data_buf,&anal_cnt);
		mems.Mag_Offset.z=floatFromData(data_buf,&anal_cnt);				
		mems.Mag_Gain.x=floatFromData(data_buf,&anal_cnt);
		mems.Mag_Gain.y=floatFromData(data_buf,&anal_cnt);
		mems.Mag_Gain.z=floatFromData(data_buf,&anal_cnt);	
		
		}
	}	else if(*(data_buf+2)==0x51)//param config custom  配置文件写入
  { 
		cnt_usb_rx[*(data_buf+2)]++;
		temp_char=charFromData(data_buf,&anal_cnt);
		if(temp_char==2){//复位参数
			vmc_all.param.send_mask=1;
			//vmc_param_reinit();
		}else if(temp_char==1){
			vmc_all.param.param_save=1;
		}
		else if(vmc_all.param.send_mask==0){
			vmc_all.param.param_vmc.pid_pit[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_pit[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_pit[2]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_rol[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_rol[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_rol[2]=floatFromData(data_buf,&anal_cnt);		
			vmc_all.param.param_vmc.pid_yaw[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_yaw[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_yaw[2]=floatFromData(data_buf,&anal_cnt);		
			vmc_all.param.param_vmc.pid_vx[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_vx[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_vx[2]=floatFromData(data_buf,&anal_cnt);		
			vmc_all.param.param_vmc.pid_vy[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_vy[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_vy[2]=floatFromData(data_buf,&anal_cnt);				
			vmc_all.param.param_vmc.pid_posxy[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_posxy[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_posxy[2]=floatFromData(data_buf,&anal_cnt);	
			vmc_all.param.param_vmc.pid_posz[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_posz[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.pid_posz[2]=floatFromData(data_buf,&anal_cnt);		
			vmc_all.param.param_vmc.stance_time[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.stance_time[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.swing_hight=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.swing_spdkp[0]=floatFromData(data_buf,&anal_cnt);	
			vmc_all.param.param_vmc.stance_xy_kp[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.stance_xy_kp[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.stance_zoff_kp=floatFromData(data_buf,&anal_cnt);	
			vmc_all.param.param_vmc.slip_p[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.slip_p[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.slip_p[2]=floatFromData(data_buf,&anal_cnt);		
			vmc_all.param.safe_sita[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.safe_sita[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.safe_sita[2]=floatFromData(data_buf,&anal_cnt);	
			temp=floatFromData(data_buf,&anal_cnt);
			temp=floatFromData(data_buf,&anal_cnt);
			temp=floatFromData(data_buf,&anal_cnt);	
			vmc_all.param.param_vmc.move_com_off[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.move_com_off[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.move_att_off[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.move_att_off[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.leg_off[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.leg_off[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.side_off[0]=floatFromData(data_buf,&anal_cnt);
			vmc_all.param.param_vmc.side_off[1]=floatFromData(data_buf,&anal_cnt);
			vmc_all.tar_att_bias[PITr]=floatFromData(data_buf,&anal_cnt);
			vmc_all.tar_att_bias[ROLr]=floatFromData(data_buf,&anal_cnt);
			if(temp_char!=3){
			vmc_all.your_key_all=intFromData(data_buf,&anal_cnt);
			vmc_all.your_key[0]=vmc_all.your_key_all/10000;
			vmc_all.your_key[1]=(vmc_all.your_key_all-vmc_all.your_key[0]*10000)/100;
			vmc_all.your_key[2]=(vmc_all.your_key_all-vmc_all.your_key[0]*10000-vmc_all.your_key[1]*100);
			//get_license();
			}
			vmc_all.param.param_save=1;		
		 }
	 }
		else if(*(data_buf+2)==0x61)//test config custom测试模式
  { 
		cnt_usb_rx[*(data_buf+2)]++;
		temp_time = Get_Cycle_T(GET_T_TEST); 	
		temp=charFromData(data_buf,&anal_cnt);//用于选择 测试模式ucos
		temp=charFromData(data_buf,&anal_cnt);//用于选择  位姿平衡模式与其他子状态
		vmc_all.param.param_save=charFromData(data_buf,&anal_cnt);
		temp=floatFromData(data_buf,&anal_cnt);
		temp=floatFromData(data_buf,&anal_cnt);
		temp=floatFromData(data_buf,&anal_cnt);
		temp=floatFromData(data_buf,&anal_cnt);
		temp=floatFromData(data_buf,&anal_cnt);
		temp=floatFromData(data_buf,&anal_cnt);
		temp=floatFromData(data_buf,&anal_cnt);
		temp=floatFromData(data_buf,&anal_cnt);
		temp=floatFromData(data_buf,&anal_cnt);
		vmc[0].param.invert_knee_epos[Xr]=(charFromData(data_buf,&anal_cnt)==1?-1:1);
		vmc[1].param.invert_knee_epos[Xr]=(charFromData(data_buf,&anal_cnt)==1?-1:1);
		vmc[2].param.invert_knee_epos[Xr]=(charFromData(data_buf,&anal_cnt)==1?-1:1);
		vmc[3].param.invert_knee_epos[Xr]=(charFromData(data_buf,&anal_cnt)==1?-1:1);
		vmc_all.param.param_vmc.pid_pit[0]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.pid_pit[1]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.pid_pit[2]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.pid_rol[0]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.pid_rol[1]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.pid_rol[2]=floatFromData(data_buf,&anal_cnt);		
		vmc_all.param.param_vmc.pid_yaw[0]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.pid_yaw[1]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.pid_yaw[2]=floatFromData(data_buf,&anal_cnt);		
		vmc_all.param.param_vmc.pid_posz[0]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.pid_posz[1]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.pid_posz[2]=floatFromData(data_buf,&anal_cnt);		
		temp=floatFromData(data_buf,&anal_cnt);
		temp=floatFromData(data_buf,&anal_cnt);
		temp=floatFromData(data_buf,&anal_cnt);	
		vmc_all.tar_att_bias[PITr]=floatFromData(data_buf,&anal_cnt);
		vmc_all.tar_att_bias[ROLr]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.leg_off[0]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.leg_off[1]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.side_off[0]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.side_off[1]=floatFromData(data_buf,&anal_cnt);
		vmc_all.param.param_vmc.swing_hight=floatFromData(data_buf,&anal_cnt);	
	}else if(*(data_buf+2)==0x2)//PWM config custom
  { 
			cnt_usb_rx[*(data_buf+2)]++;
		  need_save = charFromData(data_buf,&anal_cnt);
			vmc_all.param.cmd_use_ocu=charFromData(data_buf,&anal_cnt);//10 enable
			need_save=vmc_all.param.param_save=charFromData(data_buf,&anal_cnt);

			vmc_all.param.dj_type=charFromData(data_buf,&anal_cnt);

//			vmc[0].param.PWM_OUT_OCU[0]=intFromData(data_buf,&anal_cnt);
//			vmc[0].param.PWM_OUT_OCU[1]=intFromData(data_buf,&anal_cnt);
//			vmc[0].param.PWM_OUT_OCU[2]=intFromData(data_buf,&anal_cnt);
//			vmc[1].param.PWM_OUT_OCU[0]=intFromData(data_buf,&anal_cnt);
//			vmc[1].param.PWM_OUT_OCU[1]=intFromData(data_buf,&anal_cnt);
//			vmc[1].param.PWM_OUT_OCU[2]=intFromData(data_buf,&anal_cnt);
//			vmc[2].param.PWM_OUT_OCU[0]=intFromData(data_buf,&anal_cnt);
//			vmc[2].param.PWM_OUT_OCU[1]=intFromData(data_buf,&anal_cnt);
//			vmc[2].param.PWM_OUT_OCU[2]=intFromData(data_buf,&anal_cnt);
//			vmc[3].param.PWM_OUT_OCU[0]=intFromData(data_buf,&anal_cnt);
//			vmc[3].param.PWM_OUT_OCU[1]=intFromData(data_buf,&anal_cnt);
//			vmc[3].param.PWM_OUT_OCU[2]=intFromData(data_buf,&anal_cnt);
			
//			vmc[0].param.PWM_OFF_OCU[0]=intFromData(data_buf,&anal_cnt)+1500;
//			vmc[0].param.PWM_OFF_OCU[1]=intFromData(data_buf,&anal_cnt)+1500;
//			vmc[0].param.PWM_OFF_OCU[2]=intFromData(data_buf,&anal_cnt)+1500;
//			vmc[1].param.PWM_OFF_OCU[0]=intFromData(data_buf,&anal_cnt)+1500;
//			vmc[1].param.PWM_OFF_OCU[1]=intFromData(data_buf,&anal_cnt)+1500;
//			vmc[1].param.PWM_OFF_OCU[2]=intFromData(data_buf,&anal_cnt)+1500;
//			vmc[2].param.PWM_OFF_OCU[0]=intFromData(data_buf,&anal_cnt)+1500;
//			vmc[2].param.PWM_OFF_OCU[1]=intFromData(data_buf,&anal_cnt)+1500;
//			vmc[2].param.PWM_OFF_OCU[2]=intFromData(data_buf,&anal_cnt)+1500;
//			vmc[3].param.PWM_OFF_OCU[0]=intFromData(data_buf,&anal_cnt)+1500;
//			vmc[3].param.PWM_OFF_OCU[1]=intFromData(data_buf,&anal_cnt)+1500;
//			vmc[3].param.PWM_OFF_OCU[2]=intFromData(data_buf,&anal_cnt)+1500;
		
			vmc[0].param.sita_flag_ocu[0]=charFromData(data_buf,&anal_cnt);
			vmc[0].param.sita_flag_ocu[1]=charFromData(data_buf,&anal_cnt);
			vmc[0].param.sita_flag_ocu[2]=charFromData(data_buf,&anal_cnt);
			vmc[1].param.sita_flag_ocu[0]=charFromData(data_buf,&anal_cnt);
			vmc[1].param.sita_flag_ocu[1]=charFromData(data_buf,&anal_cnt);
			vmc[1].param.sita_flag_ocu[2]=charFromData(data_buf,&anal_cnt);
			vmc[2].param.sita_flag_ocu[0]=charFromData(data_buf,&anal_cnt);
			vmc[2].param.sita_flag_ocu[1]=charFromData(data_buf,&anal_cnt);
			vmc[2].param.sita_flag_ocu[2]=charFromData(data_buf,&anal_cnt);
			vmc[3].param.sita_flag_ocu[0]=charFromData(data_buf,&anal_cnt);
			vmc[3].param.sita_flag_ocu[1]=charFromData(data_buf,&anal_cnt);
			vmc[3].param.sita_flag_ocu[2]=charFromData(data_buf,&anal_cnt);
				
			temp=floatFromData(data_buf,&anal_cnt);
			temp=floatFromData(data_buf,&anal_cnt);	
			temp=floatFromData(data_buf,&anal_cnt);	
			
		
			
			for(i=0;i<4;i++){
				for(j=0;j<3;j++){//转换符号
					if(vmc[i].param.sita_flag_ocu[j]==1)
						vmc[i].param.sita_flag_ocu[j]= -1;
					else
						vmc[i].param.sita_flag_ocu[j]=  1;
					//vmc[i].param.PWM_PER_DEGREE_OCU[j]=vmc_all.param.param_vmc.servo_time[j];
				}
			}
			
			if(need_save==1)//配置文件保存
			{
				for(i=0;i<4;i++){
				 for(j=0;j<3;j++){
//						vmc[i].param.PWM_OFF[j]=vmc[i].param.PWM_OFF_OCU[j];
//						vmc[i].param.sita_flag[j]=vmc[i].param.sita_flag_ocu[j];
//					 	vmc[i].param.PWM_PER_DEGREE[j]=vmc[i].param.PWM_PER_DEGREE_OCU[j];
					}
				}
			}
			if(vmc_all.param.cmd_use_ocu==11)
			{
				vmc_all.sita_test[4]=1;//使能
				vmc_all.sita_test[0]=floatFromData(data_buf,&anal_cnt);	
				vmc_all.sita_test[1]=floatFromData(data_buf,&anal_cnt);	
				vmc_all.sita_test[2]=floatFromData(data_buf,&anal_cnt);	
				for(i=0;i<4;i++){
				 for(j=0;j<3;j++){
//						vmc[i].param.PWM_OFF[j]=vmc[i].param.PWM_OFF_OCU[j];
//						vmc[i].param.sita_flag[j]=vmc[i].param.sita_flag_ocu[j];
//					 	vmc[i].param.PWM_PER_DEGREE[j]=vmc[i].param.PWM_PER_DEGREE_OCU[j];
					}
				}
			}else
				vmc_all.sita_test[4]=0;
	}
	else if(*(data_buf+2)==0x31)//ocu speed keyboard
  { 
		cnt_usb_rx[*(data_buf+2)]++;
		ocu.connect=1;
		ocu.loss_cnt=0;
		ocu.mode=1;
		
		ocu.up_mode=charFromData(data_buf,&anal_cnt);
		ocu.rc_spd_b[Xr]=floatFromData(data_buf,&anal_cnt);
		ocu.rc_spd_b[Yr]=floatFromData(data_buf,&anal_cnt);
		ocu.rc_spd_b[Zr]=floatFromData(data_buf,&anal_cnt);
		ocu.rc_rate_b[PITr]=floatFromData(data_buf,&anal_cnt);
		ocu.rc_rate_b[ROLr]=floatFromData(data_buf,&anal_cnt);
		ocu.rc_rate_b[YAWr]=floatFromData(data_buf,&anal_cnt);

	}else if(*(data_buf+2)==0x32)//ocu speed remote
  { 
		cnt_usb_rx[*(data_buf+2)]++;
		ocu.connect=1;
		ocu.loss_cnt=0;
		ocu.up_mode=charFromData(data_buf,&anal_cnt);
		ocu.record.en_record=charFromData(data_buf,&anal_cnt);
		if(ocu.mode!=3){
			ocu.mode=2;
			ocu.rc_spd_w[Xr]=floatFromData(data_buf,&anal_cnt);
			ocu.rc_spd_w[Yr]=floatFromData(data_buf,&anal_cnt);
			ocu.rc_att_w[PITr]=floatFromData(data_buf,&anal_cnt);
			ocu.rc_att_w[ROLr]=floatFromData(data_buf,&anal_cnt);
			ocu.rate_yaw_w=floatFromData(data_buf,&anal_cnt);
			ocu.key_st=charFromData(data_buf,&anal_cnt);
			ocu.key_back=charFromData(data_buf,&anal_cnt);
			ocu.key_lr=intFromData(data_buf,&anal_cnt);
			ocu.key_ud=intFromData(data_buf,&anal_cnt);
			ocu.key_x=charFromData(data_buf,&anal_cnt);
			ocu.key_y=charFromData(data_buf,&anal_cnt);
			ocu.key_b=charFromData(data_buf,&anal_cnt);
			ocu.key_a=charFromData(data_buf,&anal_cnt);
			ocu.key_ll=charFromData(data_buf,&anal_cnt);
			ocu.key_rr=charFromData(data_buf,&anal_cnt);
			
			sdk.sdk_mode=0;
		}
	}
	else if(*(data_buf+2)==0x52)//强制断电
  {
		cnt_usb_rx[*(data_buf+2)]++;
		vmc_all.power_state=vmc_all.leg_power=vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.x=0;
		ocu.cmd_robot_state=vmc_all.param.cmd_use_ocu=vmc_all.sita_test[4]=0;	
	}
	else if(*(data_buf+2)==0x53)//主控复位
  { 
		cnt_usb_rx[*(data_buf+2)]++;
		vmc_all.power_state=vmc_all.leg_power=vmc_all.param.tar_spd_use_rc.x=vmc_all.tar_spd.x=0;
		ocu.cmd_robot_state=vmc_all.param.cmd_use_ocu=vmc_all.sita_test[4]=0;
		IWDG_Init(4,1);
	}
	else if(*(data_buf+2)==0x54)//单次保存
  { 
		cnt_usb_rx[*(data_buf+2)]++;
		vmc_all.param.param_save=1;		
	}
  //----------------------电机配置
	else if(*(data_buf+2)==0x82)//----------------------------------无刷电机配置 14电机--------------------------
  { 
		can_cmd_usb_disable=1;
		usb_loss=0;
		ocu_connect=1;
		ocu_loss_cnt=0;
		cnt_usb_rx[*(data_buf+2)]++;
		temp_char=charFromData(data_buf,&anal_cnt);
		if(temp_char==11){//1保存FLASH
			can_write_flash=1;
		}
		
		temp_char=charFromData(data_buf,&anal_cnt);
		if(!temp_char){
			for(i=0;i<14;i++){
				motor_chassis[i].param.usb_cmd_mode=temp_char;
				motor_chassis[i].en_cmd_ocu=1;
			}
		}
		else{
			for(i=0;i<14;i++){
				motor_chassis[i].param.usb_cmd_mode=0;
				motor_chassis[i].en_cmd_ocu=0;
			}
		}
		
		temp_char=charFromData(data_buf,&anal_cnt);
		for(i=0;i<14;i++){
			motor_chassis[i].reset_q_rx=temp_char;
			if(motor_chassis[i].reset_q_rx==2&&motor_chassis[i].reset_q==0)
					motor_chassis[i].reset_q=1;
		}
	
		temp_char=charFromData(data_buf,&anal_cnt);
		for(i=0;i<14;i++)
			motor_chassis[i].cmd_mode=temp_char;
		
		for(i=0;i<7;i++){
			motor_chassis[i].motor.type=charFromData(data_buf,&anal_cnt);
			motor_chassis[i].param.control_mode=charFromData(data_buf,&anal_cnt);
			if(motor_chassis[i].en_pos_trig==0)
				if(motor_chassis[i].param.control_mode==1)
					motor_chassis[i].set_qd=floatFromData(data_buf,&anal_cnt);
				else
				motor_chassis[i].set_q=floatFromData(data_buf,&anal_cnt);
			else
				temp_char=floatFromData(data_buf,&anal_cnt);
			motor_chassis[i].param.given_current=floatFromData(data_buf,&anal_cnt);
			if(charFromData(data_buf,&anal_cnt)*2-1==3)
				motor_chassis[i].param.t_inv_flag_cmd=-1;
			else
				motor_chassis[i].param.t_inv_flag_cmd=1;
			
			if(charFromData(data_buf,&anal_cnt)*2-1==3)
				motor_chassis[i].param.t_inv_flag_measure=-1;
			else
				motor_chassis[i].param.t_inv_flag_measure=1;
			
			if(charFromData(data_buf,&anal_cnt)==1)
			motor_chassis[i].param.q_flag=0;
			else
			motor_chassis[i].param.q_flag=1;	
			
			//motor_chassis[i].param.q_reset_angle=floatFromData(data_buf,&anal_cnt);
			motor_chassis[i].stiff=floatFromData(data_buf,&anal_cnt);
		}
		
		motor_chassis[0].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[1].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[2].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[3].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[4].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[5].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[6].cal_div=charFromData(data_buf,&anal_cnt);
	
		
	  for(i=0;i<14;i++){
			motor_chassis[i].set_q=LIMIT(motor_chassis[i].set_q,-178,178);
			if(fabs(motor_chassis[i].param.given_current)>0.01)
				motor_chassis[i].param.usb_cmd_mode=2;//电流
			else
				motor_chassis[i].param.usb_cmd_mode=1;//位置
			}
 	} 
		else if(*(data_buf+2)==0x83)//----------------------------------无刷电机配置 14电机---------------
  { 
		can_cmd_usb_disable=1;
		usb_loss=0;
		ocu_connect=1;
		ocu_loss_cnt=0;
		cnt_usb_rx[*(data_buf+2)]++;
		temp_char=charFromData(data_buf,&anal_cnt);
		if(temp_char==11){//1保存FLASH
			can_write_flash=1;
		}
		
		temp_char=charFromData(data_buf,&anal_cnt);
		if(!temp_char){
			for(i=0;i<14;i++){
				motor_chassis[i].param.usb_cmd_mode=temp_char;
				motor_chassis[i].en_cmd_ocu=1;
			}
		}
		else{
			for(i=0;i<14;i++){
				motor_chassis[i].param.usb_cmd_mode=0;
				motor_chassis[i].en_cmd_ocu=0;
			}
		}
		
		temp_char=charFromData(data_buf,&anal_cnt);
		for(i=0;i<14;i++){
			motor_chassis[i].reset_q_rx=temp_char;
			if(motor_chassis[i].reset_q_rx==2&&motor_chassis[i].reset_q==0)
					motor_chassis[i].reset_q=1;
		}
		for(i=7;i<14;i++){
			motor_chassis[i].motor.type=charFromData(data_buf,&anal_cnt);
			motor_chassis[i].param.control_mode=charFromData(data_buf,&anal_cnt);
			if(motor_chassis[i].en_pos_trig==0)
				if(motor_chassis[i].param.control_mode==1)
					motor_chassis[i].set_qd=floatFromData(data_buf,&anal_cnt);
				else
				motor_chassis[i].set_q=floatFromData(data_buf,&anal_cnt);
			else
				temp_char=floatFromData(data_buf,&anal_cnt);
			motor_chassis[i].param.given_current=floatFromData(data_buf,&anal_cnt);
			
			if(charFromData(data_buf,&anal_cnt)*2-1==3)
				motor_chassis[i].param.t_inv_flag_cmd=-1;
			else
				motor_chassis[i].param.t_inv_flag_cmd=1;
			
			if(charFromData(data_buf,&anal_cnt)*2-1==3)
				motor_chassis[i].param.t_inv_flag_measure=-1;
			else
				motor_chassis[i].param.t_inv_flag_measure=1;
			
			if(charFromData(data_buf,&anal_cnt)==1)
			motor_chassis[i].param.q_flag=0;
			else
			motor_chassis[i].param.q_flag=1;	
			
			//motor_chassis[i].param.q_reset_angle=floatFromData(data_buf,&anal_cnt);
			motor_chassis[i].stiff=floatFromData(data_buf,&anal_cnt);
		}
		
	  motor_chassis[7].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[8].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[9].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[10].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[11].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[12].cal_div=charFromData(data_buf,&anal_cnt);
		motor_chassis[13].cal_div=charFromData(data_buf,&anal_cnt);
		
	  for(i=0;i<14;i++){
			motor_chassis[i].set_q=LIMIT(motor_chassis[i].set_q,-178,178);
			motor_chassis[i].set_t=motor_chassis[i].param.given_current;
			if(fabs(motor_chassis[i].param.given_current)>0.01)
				motor_chassis[i].param.usb_cmd_mode=2;//电流
			else
				motor_chassis[i].param.usb_cmd_mode=1;//位置
			}
 	}
}


int _data_lenUSB=0;
int _data_cntUSB=0;
int _data_lenUSB_RX=0,_data_cntUSB_RX=0;
uint8_t USB_RX_BUF[100];
uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len)//接收
{
	int i;
	u8 res;
	static char RxStateUSB=0;
	static char state=0,rx_cnt;
	static int usb_send_cnt_send=0;
	static u8 _data_len2 = 0,_data_cnt2 = 0;
	static char state_spi=0;
	char sum_r=0;
	unsigned char  data_temp[8];
	char err,_cnt;
	char id;
	static u8 _data_len2_spi = 0,_data_cnt2_spi = 0;
	unsigned char data_spi;
	unsigned char data;//中断读取SPI数据
	unsigned char com_data;
	_data_lenUSB_RX=Len;
	_data_cntUSB_RX++;
	for(i=0;i<Len;i++)
	{  
		USB_RX_BUF[i]=Buf[i]; 
	}
	
	for(i=0;i<Len;i++)
	{  
 
		res=Buf[i]; 
		com_data=Buf[i]; 
		
		if(RxStateUSB==0&&com_data==0xAA)
		{
			RxStateUSB=1;
			RxBuffer_USB[0]=com_data;
		}
		else if(RxStateUSB==1&&com_data==0xAF)
		{
			RxStateUSB=2;
			RxBuffer_USB[1]=com_data;
		}
		else if(RxStateUSB==2&&com_data>0&&com_data<0XF1)
		{
			RxStateUSB=3;
			RxBuffer_USB[2]=com_data;
		}
		else if(RxStateUSB==3&&com_data<255)
		{
			RxStateUSB= 4;
			RxBuffer_USB[3]=com_data;
			_data_lenUSB = com_data;
			_data_cntUSB = 0;
		}
		else if(RxStateUSB==4&&_data_lenUSB>0)
		{
			_data_lenUSB--;
			RxBuffer_USB[4+_data_cntUSB++]=com_data;
			if(_data_lenUSB==0)
				RxStateUSB= 5;
		}
		else if(RxStateUSB==5)
		{
			RxStateUSB = 0;
			RxBuffer_USB[4+_data_cntUSB]=com_data;
      Anal_USB(RxBuffer_USB,_data_cntUSB+5);
			for(i=0;i<255;i++)
				RxBuffer_USB[i]=0;
		}
		else
			RxStateUSB = 0;
	 
		
		if((USB_USART_RX_STA&0x8000)==0)		//接收未完成
		{
			if(USB_USART_RX_STA&0x4000)			//接收到了0x0d
			{
				if(res!=0x0a)USB_USART_RX_STA=0;//接收错误,重新开始
				else USB_USART_RX_STA|=0x8000;	//接收完成了 
			}else //还没收到0X0D
			{	
				if(res==0x0d)USB_USART_RX_STA|=0x4000;
				else
				{
					USB_USART_RX_BUF[USB_USART_RX_STA&0X3FFF]=res;
					USB_USART_RX_STA++;
					if(USB_USART_RX_STA>(USB_USART_REC_LEN-1))USB_USART_RX_STA=0;//接收数据错误,重新开始接收	
				}					
			}
		}   
	} //end for 
	return USBD_OK;
}
//usb虚拟串口,printf 函数
//确保一次发送数据不超USB_USART_REC_LEN字节
void usb_printf(char* fmt,...)  
{  
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART_PRINTF_Buffer,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART_PRINTF_Buffer);//此次发送数据的长度
	for(j=0;j<i;j++)//循环发送数据
	{
		VCP_DataTx(USART_PRINTF_Buffer[j]); 
	}
} 

extern vu8 bDeviceState;		//USB连接 情况
uint8_t SendBuff_USB[500];
int usb_send_cnt=0;

void setDataInt(int i)
{
	#if 0
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	#else
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	#endif
}

void setDataFloat(float f)
{
	int i = *(int *)&f;
	#if 0
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	#else
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	#endif
}

void setDataFloat_int(float f,float size)
{
	vs16 _temp;
	_temp=f*size;
	SendBuff_USB[usb_send_cnt++] = BYTE1(_temp);
	SendBuff_USB[usb_send_cnt++] = BYTE0(_temp);
}

float floatFromData_int(unsigned char *data, int *anal_cnt,float size)
{
	float temp=0;
	temp=(float)((int16_t)(*(data + *anal_cnt + 0)<<8)|*(data + *anal_cnt + 1))/size;
	*anal_cnt += 2;
	return temp;
}


float floatFromData(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	return *(float *)&i;
}

char charFromData(unsigned char *data,int* anal_cnt)
{
	int temp=*anal_cnt ;
	*anal_cnt +=1;
	return *(data+temp);
}

int intFromData(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	*anal_cnt +=4;
	return i;
}


void data_per_usb_robot_state1(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0;
	int cnt_reg=0;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x04;
	SendBuff_USB[usb_send_cnt++]=0;

//  setDataFloat(vmc_all.att[PITr]);
//	setDataFloat(vmc_all.att[ROLr]);
//	setDataFloat(vmc_all.att[YAWr]);
	
	setDataFloat(imuo.acc_n[2]);
	setDataFloat(imuo.acc_n[1]);
	setDataFloat(imuo.acc_n[2]);
	setDataFloat(mems.Gyro_deg_rt.x);
	setDataFloat(mems.Gyro_deg_rt.y);
	setDataFloat(mems.Gyro_deg_rt.z);
	
	setDataFloat(vmc_all.pos_n.x);
	setDataFloat(vmc_all.pos_n.y);
	setDataFloat(vmc_all.pos_n.z);
	setDataFloat(vmc_all.spd_n.x);
	setDataFloat(vmc_all.spd_n.y);
	setDataFloat(vmc_all.spd_n.z);
	setDataFloat(vmc_all.acc_b.x);
	setDataFloat(vmc_all.acc_b.y);
	setDataFloat(vmc_all.acc_b.z);
	
	setDataFloat(vmc[2].epos.x);
	setDataFloat(vmc[2].epos.y);
	setDataFloat(vmc[2].epos.z);
	setDataFloat(vmc[0].epos.x);
	setDataFloat(vmc[0].epos.y);
	setDataFloat(vmc[0].epos.z);	

	
	setDataFloat(bat.percent);
	setDataFloat(Rc_Get.signal_rate);
	
	setDataFloat(vmc_all.param.tar_spd_use_rc.x);	
	setDataFloat(vmc_all.param.tar_spd_use_rc.y);	
	setDataFloat(vmc_all.param.tar_spd_use_rc.z);	
	setDataFloat(fabs(vmc_all.tar_pos.z));	
		
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_usb_robot_state2(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x41;
	SendBuff_USB[usb_send_cnt++]=0;

	setDataFloat(vmc[3].epos.x);
	setDataFloat(vmc[3].epos.y);
	setDataFloat(vmc[3].epos.z);
	setDataFloat(vmc[1].epos.x);
	setDataFloat(vmc[1].epos.y);
	setDataFloat(vmc[1].epos.z);

//	char id=2;
//	if(leg_motor.connect&&leg_motor.connect_motor[0]&&leg_motor.ready[1])
//		setDataFloat(vmc[id].sita2);
//	else{
//		if(!leg_motor.connect)
//			setDataFloat(999);
//		else if(!leg_motor.connect_motor[1])
//			setDataFloat(666);
//		else
//			setDataFloat(333);
//	}
//	
//	if(leg_motor.connect&&leg_motor.connect_motor[1]&&leg_motor.ready[0])
//		setDataFloat(vmc[id].sita1);
//	else{
//		if(!leg_motor.connect)
//			setDataFloat(999);
//		else if(!leg_motor.connect_motor[0])
//			setDataFloat(666);
//		else
//			setDataFloat(333);
//	}
//	
////	if(leg_motor.connect&&leg_motor.connect_motor[2]&&leg_motor.ready[2])
////		setDataFloat(vmc[id].sita3);
////	else{
////		if(!leg_motor.connect)
////			setDataFloat(999);
////		else if(!leg_motor.connect_motor[2])
////			setDataFloat(666);
////		else
////			setDataFloat(333);
////	}
//	if(leg_motor.connect){
//		if(stand_force_enable_flag[id])
//			setDataFloat(99);
//			else
//			setDataFloat(robotwb.Leg[id].beta);
//	}
//	else
//	setDataFloat(0);	
//	
//	id=0;
//	if(leg_motor.connect&&leg_motor.connect_motor[0]&&leg_motor.ready[1])
//		setDataFloat(vmc[id].sita2);
//	else{
//		if(!leg_motor.connect)
//			setDataFloat(999);
//		else if(!leg_motor.connect_motor[1])
//			setDataFloat(666);
//		else
//			setDataFloat(333);
//	}
//	if(leg_motor.connect&&leg_motor.connect_motor[1]&&leg_motor.ready[0])
//		setDataFloat(vmc[id].sita1);
//	else{
//		if(!leg_motor.connect)
//			setDataFloat(999);
//		else if(!leg_motor.connect_motor[0])
//			setDataFloat(666);
//		else
//			setDataFloat(333);
//	}
////	if(leg_motor.connect&&leg_motor.connect_motor[2]&&leg_motor.ready[2])
////		setDataFloat(vmc[id].sita3);
////	else{
////		if(!leg_motor.connect)
////			setDataFloat(999);
////		else if(!leg_motor.connect_motor[2])
////			setDataFloat(666);
////		else
////			setDataFloat(333);
////	}
//	if(leg_motor.connect){
//		if(stand_force_enable_flag[id])
//			setDataFloat(99);
//			else
//			setDataFloat(robotwb.Leg[id].beta);
//	}
//	else
//	setDataFloat(0);	
//	
//	id=3;
//	if(leg_motor.connect&&leg_motor.connect_motor[0]&&leg_motor.ready[1])
//		setDataFloat(vmc[id].sita2);
//	else{
//		if(!leg_motor.connect)
//			setDataFloat(999);
//		else if(!leg_motor.connect_motor[1])
//			setDataFloat(666);
//		else
//			setDataFloat(333);
//	}
//	if(leg_motor.connect&&leg_motor.connect_motor[1]&&leg_motor.ready[0])
//		setDataFloat(vmc[id].sita1);
//	else{
//		if(!leg_motor.connect)
//			setDataFloat(999);
//		else if(!leg_motor.connect_motor[0])
//			setDataFloat(666);
//		else
//			setDataFloat(333);
//	}
////	if(leg_motor.connect&&leg_motor.connect_motor[2]&&leg_motor.ready[2])
////		setDataFloat(vmc[id].sita3);
////	else{
////		if(!leg_motor.connect)
////			setDataFloat(999);
////		else if(!leg_motor.connect_motor[2])
////			setDataFloat(666);
////		else
////			setDataFloat(333);
////	}
//	if(leg_motor.connect){
//		if(stand_force_enable_flag[id])
//			setDataFloat(99);
//			else
//			setDataFloat(robotwb.Leg[id].beta);
//	}
//	else
//	setDataFloat(0);	
//	
//	id=1;
//	if(leg_motor.connect&&leg_motor.connect_motor[0]&&leg_motor.ready[1])
//		setDataFloat(vmc[id].sita2);
//	else{
//		if(!leg_motor.connect)
//			setDataFloat(999);
//		else if(!leg_motor.connect_motor[1])
//			setDataFloat(666);
//		else
//			setDataFloat(333);
//	}
//	if(leg_motor.connect&&leg_motor.connect_motor[1]&&leg_motor.ready[0])
//		setDataFloat(vmc[id].sita1);
//	else{
//		if(!leg_motor.connect)
//			setDataFloat(999);
//		else if(!leg_motor.connect_motor[0])
//			setDataFloat(666);
//		else
//			setDataFloat(333);
//	}
////	if(leg_motor.connect&&leg_motor.connect_motor[2]&&leg_motor.ready[2])
////		setDataFloat(vmc[id].sita3);
////	else{
////		if(!leg_motor.connect)
////			setDataFloat(999);
////		else if(!leg_motor.connect_motor[2])
////			setDataFloat(666);
////		else
////			setDataFloat(333);
////	}
//	if(leg_motor.connect){
//		if(stand_force_enable_flag[id])
//			setDataFloat(99);
//			else
//			setDataFloat(robotwb.Leg[id].beta);
//	}
//	else
//	setDataFloat(0);	
//	
//	setDataFloat(vmc[2].param.sita1_off);
//	setDataFloat(vmc[2].param.sita2_off);
//	setDataFloat(vmc[2].param.sita3_off);
//	setDataFloat(vmc[0].param.sita1_off);
//	setDataFloat(vmc[0].param.sita2_off);
//	setDataFloat(vmc[0].param.sita3_off);
//	setDataFloat(vmc[3].param.sita1_off);
//	setDataFloat(vmc[3].param.sita2_off);
//	setDataFloat(vmc[3].param.sita3_off);
//	setDataFloat(vmc[1].param.sita1_off);
//	setDataFloat(vmc[1].param.sita2_off);
//	setDataFloat(vmc[1].param.sita3_off);
//if((vmc_all.gait_mode==STAND_IMU||vmc_all.gait_mode==STAND_RC||vmc_all.gait_mode==TROT)){
//	SendBuff_USB[usb_send_cnt++]=vmc[2].ground;
//	SendBuff_USB[usb_send_cnt++]=vmc[0].ground;
//	SendBuff_USB[usb_send_cnt++]=vmc[3].ground;
//	SendBuff_USB[usb_send_cnt++]=vmc[1].ground;
//}
//else{
//	SendBuff_USB[usb_send_cnt++]=vmc[2].is_touch;
//	SendBuff_USB[usb_send_cnt++]=vmc[0].is_touch;
//	SendBuff_USB[usb_send_cnt++]=vmc[3].is_touch;
//	SendBuff_USB[usb_send_cnt++]=vmc[1].is_touch;
//}
//	
//	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
//		for( i=cnt_reg;i<usb_send_cnt;i++)
//	sum += SendBuff_USB[i];
//	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_usb_controller_config(void)
{ 
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x01;
	SendBuff_USB[usb_send_cnt++]=0;

  setDataFloat(vmc_all.W);
	setDataFloat(vmc_all.H);
	setDataFloat(vmc_all.mess);
  setDataFloat(vmc_all.l1);
	setDataFloat(vmc_all.l2);
	setDataFloat(vmc_all.l3);
	setDataFloat(vmc_all.param.MAX_Z);
	setDataFloat(vmc_all.param.MIN_Z);
	setDataFloat(vmc_all.param.MAX_X);
	setDataFloat(vmc_all.param.MIN_X);
	setDataFloat(vmc_all.param.param_vmc.move_com_off[0]);
	setDataFloat(vmc_all.param.param_vmc.move_com_off[1]);
	setDataFloat(vmc_all.param.param_vmc.move_att_off[0]);
	setDataFloat(vmc_all.param.param_vmc.move_att_off[1]);
	setDataFloat(vmc_all.param.param_vmc.leg_off[0]);
	setDataFloat(vmc_all.param.param_vmc.leg_off[1]);
	setDataFloat(vmc_all.param.param_vmc.side_off[0]);
	setDataFloat(vmc_all.param.param_vmc.side_off[1]);
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.en_gait_switch;//gait switch
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.en_fall_protect;//fall switch
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.rc_type;
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.dj_type;//dj type
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.robot_mode;//机器人配置
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.robot_type;
	SendBuff_USB[usb_send_cnt++]=module.acc_imu;
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.leg_type;
	SendBuff_USB[usb_send_cnt++]=(vmc[0].param.invert_knee_epos[Xr]==-1);
	SendBuff_USB[usb_send_cnt++]=(vmc[1].param.invert_knee_epos[Xr]==-1);
	SendBuff_USB[usb_send_cnt++]=(vmc[2].param.invert_knee_epos[Xr]==-1);
	SendBuff_USB[usb_send_cnt++]=(vmc[3].param.invert_knee_epos[Xr]==-1);
	
	setDataFloat(vmc_all.version[0]);
	setDataFloat(vmc_all.version[1]);
	setDataInt(vmc_all.board_id[0]*10000+vmc_all.board_id[1]*100+vmc_all.board_id[2]);
	SendBuff_USB[usb_send_cnt++]=(vmc_all.key_right);
	setDataInt(vmc_all.your_key[0]*10000+vmc_all.your_key[1]*100+vmc_all.your_key[2]);

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_usb_sensor_config(void)
{ 
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x21;
	SendBuff_USB[usb_send_cnt++]=0;

	setDataFloat(mems.imu_pos.x);
	setDataFloat(mems.imu_pos.y);
	setDataFloat(mems.imu_pos.z);
	setDataFloat(mems.imu_att.x);
	setDataFloat(mems.imu_att.y);
	setDataFloat(mems.imu_att.z);
	setDataFloat(mems.gps_pos.x);
	setDataFloat(mems.gps_pos.y);
	setDataFloat(mems.gps_pos.z);
	
  setDataFloat(mems.Acc_Offset.x/4096.);
	setDataFloat(mems.Acc_Offset.y/4096.);
	setDataFloat(mems.Acc_Offset.z/4096.);
	setDataFloat(1);
	setDataFloat(1);
	setDataFloat(1);
  setDataFloat(mems.Gyro_Offset.x);
	setDataFloat(mems.Gyro_Offset.y);
	setDataFloat(mems.Gyro_Offset.z);
	setDataFloat(1);
	setDataFloat(1);
	setDataFloat(1);
	setDataFloat(mems.Mag_Offset.x);
	setDataFloat(mems.Mag_Offset.y);
	setDataFloat(mems.Mag_Offset.z);
	setDataFloat(mems.Mag_Gain.x);
	setDataFloat(mems.Mag_Gain.y);
	setDataFloat(mems.Mag_Gain.z);

	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_usb_pwm_config(void)
{ 
	u8 i,j;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x02;
	SendBuff_USB[usb_send_cnt++]=0;

	for(i=0;i<4;i++){
  setDataInt(0);//(vmc[i].param.PWM_OUT[0]);
	setDataInt(0);//(vmc[i].param.PWM_OUT[1]);
	setDataInt(0);//(vmc[i].param.PWM_OUT[2]);
	}
	for(i=0;i<4;i++){
  setDataInt(0);//(vmc[i].param.PWM_OFF[0]-1500);
	setDataInt(0);//(vmc[i].param.PWM_OFF[1]-1500);
	setDataInt(0);//t(vmc[i].param.PWM_OFF[2]-1500);
	}
	for(i=0;i<4;i++){
		for(j=0;j<3;j++){
			if(vmc[i].param.sita_flag[j]==-1)//转换符号
				SendBuff_USB[usb_send_cnt++]=1;
			else
				SendBuff_USB[usb_send_cnt++]=0;
		}
	}
	setDataFloat(0);//vmc[0].param.PWM_PER_DEGREE[0]);
	setDataFloat(0);//(vmc[0].param.PWM_PER_DEGREE[1]);
	setDataFloat(0);//(vmc[0].param.PWM_PER_DEGREE[2]);
	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}



void data_per_usb_curve_config(void)
{ 
	u8 i,j;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x09;
	SendBuff_USB[usb_send_cnt++]=0;

	for(i=0;i<9;i++)
		setDataFloat(ocu.curve[i]);

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_usb_param_config(void)
{ 
	u8 i,j;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x51;
	SendBuff_USB[usb_send_cnt++]=0;

  if(vmc_all.param.send_mask==1)vmc_all.param.send_mask=0;
	setDataFloat(vmc_all.param.param_vmc.pid_pit[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_pit[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_pit[2]);
	setDataFloat(vmc_all.param.param_vmc.pid_rol[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_rol[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_rol[2]);
	setDataFloat(vmc_all.param.param_vmc.pid_yaw[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_yaw[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_yaw[2]);
	setDataFloat(vmc_all.param.param_vmc.pid_vx[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_vx[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_vx[2]);
	setDataFloat(vmc_all.param.param_vmc.pid_vy[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_vy[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_vy[2]);	
	setDataFloat(vmc_all.param.param_vmc.pid_posxy[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_posxy[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_posxy[2]);
	setDataFloat(vmc_all.param.param_vmc.pid_posz[0]);
	setDataFloat(vmc_all.param.param_vmc.pid_posz[1]);
	setDataFloat(vmc_all.param.param_vmc.pid_posz[2]);	
	setDataFloat(vmc_all.param.param_vmc.stance_xy_kp[0]);
	setDataFloat(vmc_all.param.param_vmc.stance_xy_kp[1]);
	setDataFloat(vmc_all.param.param_vmc.stance_zoff_kp);
	setDataFloat(vmc_all.param.param_vmc.stance_time[0]);
	setDataFloat(vmc_all.param.param_vmc.stance_time[1]);
	setDataFloat(vmc_all.param.param_vmc.swing_hight);
	setDataFloat(vmc_all.param.param_vmc.swing_spdkp[0]);
	setDataFloat(vmc_all.param.safe_sita[0]);
	setDataFloat(vmc_all.param.safe_sita[1]);
	setDataFloat(vmc_all.param.safe_sita[2]);		
	setDataFloat(vmc_all.param.param_vmc.slip_p[0]);
	setDataFloat(vmc_all.param.param_vmc.slip_p[1]);
	setDataFloat(vmc_all.param.param_vmc.slip_p[2]);		
	setDataFloat(0);//vmc_all.param.param_vmc.servo_pd[0]);
	setDataFloat(0);//(vmc_all.param.param_vmc.servo_pd[1]);
	setDataFloat(0);//(vmc_all.param.param_vmc.servo_flt);
	setDataFloat(vmc_all.param.param_vmc.move_com_off[0]);
	setDataFloat(vmc_all.param.param_vmc.move_com_off[1]);
	setDataFloat(vmc_all.param.param_vmc.move_att_off[PITr]);
	setDataFloat(vmc_all.param.param_vmc.move_att_off[ROLr]);
	setDataFloat(vmc_all.param.param_vmc.leg_off[0]);
	setDataFloat(vmc_all.param.param_vmc.leg_off[1]);
	setDataFloat(vmc_all.param.param_vmc.side_off[0]);
	setDataFloat(vmc_all.param.param_vmc.side_off[1]);
	setDataFloat(vmc_all.tar_att_bias[PITr]);
	setDataFloat(vmc_all.tar_att_bias[ROLr]);		

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

//简单模式
void data_per_usb_robot_state_simple1(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0;
	int cnt_reg=0;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x81;
	SendBuff_USB[usb_send_cnt++]=0;

  setDataFloat(vmc_all.att[PITr]);
	setDataFloat(vmc_all.att[ROLr]);
	setDataFloat(vmc_all.att[YAWr]);
	setDataFloat(vmc_all.att_rate[PITr]);
	setDataFloat(vmc_all.att_rate[ROLr]);
	setDataFloat(vmc_all.att_rate[YAWr]);

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

void data_per_usb_robot_state_simple2(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0;
	int cnt_reg=0;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x82;
	SendBuff_USB[usb_send_cnt++]=0;
	
	setDataFloat(vmc_all.pos_n.x);
	setDataFloat(vmc_all.pos_n.y);
	setDataFloat(vmc_all.pos_n.z);
	setDataFloat(vmc_all.spd_n.x);
	setDataFloat(vmc_all.spd_n.y);
	setDataFloat(vmc_all.spd_n.z);

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

void data_per_usb_robot_state_simple3(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0;
	int cnt_reg=0;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x83;
	SendBuff_USB[usb_send_cnt++]=0;
	
	setDataFloat(vmc_all.acc_b.x);
	setDataFloat(vmc_all.acc_b.y);
	setDataFloat(vmc_all.acc_b.z);
	setDataFloat(bat.percent);
	setDataFloat(Rc_Get.signal_rate);
	setDataFloat(fabs(vmc_all.tar_pos.z));	

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

void data_per_usb_robot_state_simple4(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0;
	int cnt_reg=0;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x84;
	SendBuff_USB[usb_send_cnt++]=0;
	
	setDataFloat(vmc_all.param.tar_spd_use_rc.x);	
	setDataFloat(vmc_all.param.tar_spd_use_rc.y);	
	setDataFloat(vmc_all.param.tar_spd_use_rc.z);	
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.rc_type;
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.robot_mode;//机器人配置
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.robot_type;
	SendBuff_USB[usb_send_cnt++]=module.acc_imu;
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.leg_type;

		
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_usb_robot_state_simple5(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0;
	int cnt_reg=0;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x85;
	SendBuff_USB[usb_send_cnt++]=0;
	
	setDataFloat(vmc_all.version[0]);
	setDataFloat(vmc_all.version[1]);
	setDataInt(vmc_all.board_id[0]*10000+vmc_all.board_id[1]*100+vmc_all.board_id[2]);
	SendBuff_USB[usb_send_cnt++]=(vmc_all.key_right);
	setDataInt(vmc_all.your_key[0]*10000+vmc_all.your_key[1]*100+vmc_all.your_key[2]);

		
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_sdk1(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0;
	int cnt_reg=0;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x91;
	SendBuff_USB[usb_send_cnt++]=0;
	
	setDataFloat(vmc_all.att[PITr]);
	setDataFloat(vmc_all.att[ROLr]);
	setDataFloat(vmc_all.att[YAWr]);
	setDataFloat(vmc_all.att_rate[PITr]);
	setDataFloat(vmc_all.att_rate[ROLr]);
	setDataFloat(vmc_all.att_rate[YAWr]);

	setDataFloat(vmc_all.pos_n.x);
	setDataFloat(vmc_all.pos_n.y);
	setDataFloat(vmc_all.pos_n.z);
	setDataFloat(vmc_all.spd_n.x);
	setDataFloat(vmc_all.spd_n.y);
	setDataFloat(vmc_all.spd_n.z);

	setDataFloat(vmc_all.acc_b.x);
	setDataFloat(vmc_all.acc_b.y);
	setDataFloat(vmc_all.acc_b.z);


	setDataFloat(vmc[2].epos.x);
	setDataFloat(vmc[2].epos.y);
	setDataFloat(vmc[2].epos.z);
	setDataFloat(vmc[0].epos.x);
	setDataFloat(vmc[0].epos.y);
	setDataFloat(vmc[0].epos.z);	
	setDataFloat(vmc[3].epos.x);
	setDataFloat(vmc[3].epos.y);
	setDataFloat(vmc[3].epos.z);
	setDataFloat(vmc[1].epos.x);
	setDataFloat(vmc[1].epos.y);
	setDataFloat(vmc[1].epos.z);
	
	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

void data_per_sdk2(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0;
	int cnt_reg=0;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x92;
	SendBuff_USB[usb_send_cnt++]=0;

	setDataFloat(vmc_all.W);
	setDataFloat(vmc_all.H);
	setDataFloat(vmc_all.l1);
	setDataFloat(vmc_all.l2);
	setDataFloat(vmc_all.l3);
	
	
	setDataFloat(vmc[2].sita1);
	setDataFloat(vmc[2].sita2);
	setDataFloat(vmc[2].sita3);
	setDataFloat(vmc[0].sita1);
	setDataFloat(vmc[0].sita2);
	setDataFloat(vmc[0].sita3);
	setDataFloat(vmc[3].sita1);
	setDataFloat(vmc[3].sita2);
	setDataFloat(vmc[3].sita3);
	setDataFloat(vmc[1].sita1);
	setDataFloat(vmc[1].sita2);
	setDataFloat(vmc[1].sita3);
	

	setDataFloat(vmc_all.param.tar_spd_use_rc.x);	
	setDataFloat(vmc_all.param.tar_spd_use_rc.y);	
	setDataFloat(vmc_all.param.tar_spd_use_rc.z);	
	setDataFloat(fabs(vmc_all.tar_pos.z));
	setDataFloat(bat.percent);
	setDataFloat(Rc_Get.signal_rate);
		
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.rc_type;
	SendBuff_USB[usb_send_cnt++]=vmc_all.param.robot_mode;//机器人配置
	SendBuff_USB[usb_send_cnt++]=module.acc_imu;

	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_sdk3(void)//里程计
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0;
	int cnt_reg=0;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x93;
	SendBuff_USB[usb_send_cnt++]=0;

	setDataFloat(vmc_all.cog_pos_n.x);
	setDataFloat(vmc_all.cog_pos_n.y);
	setDataFloat(vmc_all.cog_pos_n.z);
	
	setDataFloat(vmc_all.zmp_pos_n.x);
	setDataFloat(vmc_all.zmp_pos_n.y);
	setDataFloat(vmc_all.zmp_pos_n.z);
	
	setDataFloat(0);//walk_gait.now_cog_nn.x);
	setDataFloat(0);//(walk_gait.now_cog_nn.y);
	setDataFloat(0);//(walk_gait.now_cog_nn.z);

	SendBuff_USB[usb_send_cnt++]=vmc[2].ground;
	SendBuff_USB[usb_send_cnt++]=vmc[0].ground;
	SendBuff_USB[usb_send_cnt++]=vmc[3].ground;
	SendBuff_USB[usb_send_cnt++]=vmc[1].ground;
	
	setDataFloat(vmc[2].epos_nn.x);
	setDataFloat(vmc[2].epos_nn.y);
	setDataFloat(vmc[2].epos_nn.z);
	setDataFloat(vmc[0].epos_nn.x);
	setDataFloat(vmc[0].epos_nn.y);
	setDataFloat(vmc[0].epos_nn.z);	
	setDataFloat(vmc[3].epos_nn.x);
	setDataFloat(vmc[3].epos_nn.y);
	setDataFloat(vmc[3].epos_nn.z);
	setDataFloat(vmc[1].epos_nn.x);
	setDataFloat(vmc[1].epos_nn.y);
	setDataFloat(vmc[1].epos_nn.z);
	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

//--------------------------------Record BLDC
//		ocu.record.is_ground[i]=vmc[i].ground;
//		ocu.record.is_touch[i]=vmc[i].is_touch;
//	  ocu.record.force_en_flag[i]=stand_force_enable_flag[i];
//		ocu.record.leg_state[i]=vmc[i].param.trig_state;
//		
//		ocu.record.epos_n_now[i]=vmc[i].epos_n;
//		ocu.record.depos_n_tar[i].x=robotwb.Leg[i].force_imp_spd_h.x;
//		ocu.record.depos_n_tar[i].y=robotwb.Leg[i].force_imp_spd_h.y;
//		ocu.record.depos_n_tar[i].z=robotwb.Leg[i].force_imp_spd_h.z;
//		ocu.record.depos_n_now[i]=vmc[i].epos_spdd_n;
//		
//		ocu.record.sita_now[i][0]=vmc[i].sita1;
//		ocu.record.sita_now[i][1]=vmc[i].sita2;
//		ocu.record.sita_now[i][2]=vmc[i].sita3;
//		ocu.record.sita_tar[i][0]=vmc[i].tar_sita1;
//		ocu.record.sita_tar[i][1]=vmc[i].tar_sita2;
//		ocu.record.sita_tar[i][2]=vmc[i].tar_sita3;	
//		
//		ocu.record.GRF_n_tar[i].x=robotwb.Leg[i].tar_force_dis_n.x;
//		ocu.record.GRF_n_tar[i].y=robotwb.Leg[i].tar_force_dis_n.y;
//		ocu.record.GRF_n_tar[i].z=robotwb.Leg[i].tar_force_dis_n.z;
//		ocu.record.GRF_n_now[i].x=robotwb.Leg[i].force_est_n_output.x;
//		ocu.record.GRF_n_now[i].y=robotwb.Leg[i].force_est_n_output.y;
//		ocu.record.GRF_n_now[i].z=robotwb.Leg[i].force_est_n_output.z;
//		
//		ocu.record.att_now[0]=robotwb.now_att.pitch;
//		ocu.record.att_now[1]=robotwb.now_att.roll;
//		ocu.record.att_now[2]=robotwb.now_att.yaw;
//		ocu.record.datt_now[0]=robotwb.now_rate.pitch;
//		ocu.record.datt_now[1]=robotwb.now_rate.roll;
//		ocu.record.datt_now[2]=robotwb.now_rate.yaw;
//		ocu.record.att_tar[0]=robotwb.exp_att.pitch;
//		ocu.record.att_tar[1]=robotwb.exp_att.roll;
//		ocu.record.att_tar[2]=robotwb.exp_att.yaw;		
//		
//		ocu.record.com_n_tar[0]=robotwb.exp_pos_n.x;
//		ocu.record.com_n_tar[1]=robotwb.exp_pos_n.y;
//		ocu.record.com_n_tar[2]=robotwb.exp_pos_n.z;
//		ocu.record.com_n_now[0]=vmc_all.pos_n.x;
//		ocu.record.com_n_now[1]=vmc_all.pos_n.y;
//		ocu.record.com_n_now[2]=vmc_all.pos_n.z;		
//		
//		
//		ocu.record.dcom_n_tar[0]=robotwb.exp_spd_n.x;
//		ocu.record.dcom_n_tar[1]=robotwb.exp_spd_n.y;
//		ocu.record.dcom_n_tar[2]=robotwb.exp_spd_n.z;		
//		ocu.record.dcom_n_now[0]=vmc_all.spd_n.x;
//		ocu.record.dcom_n_now[1]=vmc_all.spd_n.y;
//		ocu.record.dcom_n_now[2]=vmc_all.spd_n.z;		

//		ocu.record.ground_att_now[0]=vmc_all.ground_att_est[0];
//		ocu.record.ground_att_now[1]=vmc_all.ground_att_est[1];
//		ocu.record.ground_att_now[2]=vmc_all.ground_att_est[2];
void data_per_usb_record1_config(char sel)
{ 
	u8 i,j;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x71+sel;
	SendBuff_USB[usb_send_cnt++]=0;
	switch(sel){
		case 0:case 1:case 2:case 3://leg data
		SendBuff_USB[usb_send_cnt++]=ocu.record.is_ground[sel];
		SendBuff_USB[usb_send_cnt++]=ocu.record.is_touch[sel];
		SendBuff_USB[usb_send_cnt++]=ocu.record.force_en_flag[sel];
		SendBuff_USB[usb_send_cnt++]=ocu.record.leg_state[sel];

		setDataFloat(ocu.record.epos_n_tar[sel].x);
		setDataFloat(ocu.record.epos_n_tar[sel].y);
		setDataFloat(ocu.record.epos_n_tar[sel].z);
		
		setDataFloat(ocu.record.epos_n_now[sel].x);
		setDataFloat(ocu.record.epos_n_now[sel].y);
		setDataFloat(ocu.record.epos_n_now[sel].z);
		
		setDataFloat(ocu.record.depos_n_tar[sel].x);
		setDataFloat(ocu.record.depos_n_tar[sel].y);
		setDataFloat(ocu.record.depos_n_tar[sel].z);	
		
		setDataFloat(ocu.record.depos_n_now[sel].x);
		setDataFloat(ocu.record.depos_n_now[sel].y);
		setDataFloat(ocu.record.depos_n_now[sel].z);	
		
		setDataFloat(ocu.record.sita_tar[sel][0]);
		setDataFloat(ocu.record.sita_tar[sel][1]);
		setDataFloat(ocu.record.sita_tar[sel][2]);	

		setDataFloat(ocu.record.sita_now[sel][0]);
		setDataFloat(ocu.record.sita_now[sel][1]);
		setDataFloat(ocu.record.sita_now[sel][2]);	
		
		
		setDataFloat(ocu.record.GRF_n_tar[sel].x);
		setDataFloat(ocu.record.GRF_n_tar[sel].y);
		setDataFloat(ocu.record.GRF_n_tar[sel].z);	
		
		setDataFloat(ocu.record.GRF_n_now[sel].x);
		setDataFloat(ocu.record.GRF_n_now[sel].y);
		setDataFloat(ocu.record.GRF_n_now[sel].z);	
	break;
		case 4://com
		setDataFloat(ocu.record.att_tar[0]);
		setDataFloat(ocu.record.att_tar[1]);
		setDataFloat(ocu.record.att_tar[2]);
		
		setDataFloat(ocu.record.att_now[0]);
		setDataFloat(ocu.record.att_now[1]);
		setDataFloat(ocu.record.att_now[2]);	
		
		setDataFloat(ocu.record.datt_now[0]);
		setDataFloat(ocu.record.datt_now[1]);
		setDataFloat(ocu.record.datt_now[2]);		

		
		setDataFloat(ocu.record.com_n_tar[0]);
		setDataFloat(ocu.record.com_n_tar[1]);
		setDataFloat(ocu.record.com_n_tar[2]);		
		
		setDataFloat(ocu.record.com_n_now[0]);
		setDataFloat(ocu.record.com_n_now[1]);
		setDataFloat(ocu.record.com_n_now[2]);	

		setDataFloat(ocu.record.dcom_n_tar[0]);
		setDataFloat(ocu.record.dcom_n_tar[1]);
		setDataFloat(ocu.record.dcom_n_tar[2]);	
		
		setDataFloat(ocu.record.dcom_n_now[0]);
		setDataFloat(ocu.record.dcom_n_now[1]);
		setDataFloat(ocu.record.dcom_n_now[2]);		

		setDataFloat(ocu.record.ground_att_now[0]);
		setDataFloat(ocu.record.ground_att_now[1]);
		setDataFloat(ocu.record.ground_att_now[2]);	
	break;
	case 5://temp
		for(i=0;i<10;i++)
		setDataFloat(ocu.record.temp_record[i]);
	break;
	}

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

//--------------------------USB与主控发送
void data_per_usb_comm(char sel)
{
 int i;
	char id=0;
	char sum_t=0,_cnt=0;

	usb_send_cnt=0;
	
	SendBuff_USB[usb_send_cnt++]=0xFF;
	SendBuff_USB[usb_send_cnt++]=0xFB;
	SendBuff_USB[usb_send_cnt++]=sel;
	SendBuff_USB[usb_send_cnt++]=0;
	switch(sel)
	{
	case 1:
		setDataFloat_int(robotwb.now_att.pitch,CAN_POS_DIV);
		setDataFloat_int(robotwb.now_att.roll,CAN_POS_DIV);
		setDataFloat_int(robotwb.now_att.yaw,CAN_POS_DIV);	
			
		setDataFloat(robotwb.now_rate.pitch);
		setDataFloat(robotwb.now_rate.roll);
		setDataFloat(robotwb.now_rate.yaw);	
			
		setDataFloat_int(vmc_all.acc_b.x,100);//vmc_all.acc[Xr]);
		setDataFloat_int(vmc_all.acc_b.y,100);//vmc_all.acc[Yr]);
		setDataFloat_int(vmc_all.acc_b.z,100);//vmc_all.acc[Zr]);	
	break;
  case 2:
		for(id=0;id<4;id++){
		#if !TEST_SPI_PI
		setDataFloat_int(leg_motor.q_now[0],CAN_POS_DIV);
		setDataFloat_int(leg_motor.q_now[1],CAN_POS_DIV);
		setDataFloat_int(leg_motor.t_now[0],CAN_T_DIV);
		setDataFloat_int(leg_motor.t_now[1],CAN_T_DIV);	
		#else
		setDataFloat_spi(id*1.23);
		setDataFloat_spi(id*-1.23);
		setDataFloat_spi(id*10.23);
		setDataFloat_spi(id*-10.23);
		#endif
		}
  break;
	case 3:
		for(id=0;id<4;id++){
		#if !TEST_SPI_PI
		setDataFloat_int(24,100);
		SendBuff_USB[usb_send_cnt++]=(leg_motor.connect*100+leg_motor.connect_motor[0]*10+leg_motor.ready[0]);
		SendBuff_USB[usb_send_cnt++]=(leg_motor.connect*100+leg_motor.connect_motor[1]*10+leg_motor.ready[1]);	
		#else
		setDataFloat_spi(24.123);
		SendBuff_USB[usb_send_cnt++]=(1*100+2*10+3);
		SendBuff_USB[usb_send_cnt++]=(1*100+2*10+3);
		#endif
		}	
//		ocu.key_st=ocu_rx.key_st;//RxBuffer4[9]-1;
//		ocu.key_back=ocu_rx.key_back;//RxBuffer4[8]-1;
//		ocu.key_lr=ocu_rx.key_lr;//-(RxBuffer4[3]-1);
//		ocu.key_ud=ocu_rx.key_ud;//RxBuffer4[2]-1;
//		ocu.key_x=ocu_rx.key_x;//RxBuffer4[10]-1;
//		ocu.key_a=ocu_rx.key_a;//RxBuffer4[12]-1;
//		ocu.key_b=ocu_rx.key_b;//RxBuffer4[13]-1;
//		ocu.key_y=ocu_rx.key_y;//RxBuffer4[11]-1;
//		ocu.key_ll=ocu_rx.key_ll;//RxBuffer4[4]-1;
//		ocu.key_rr=ocu_rx.key_rr;//RxBuffer4[5]-1;
//		ocu.rc_spd_w[Xr]=ocu_rx.rc_spd_w[Xr];//my_deathzoom((float)(RxBuffer4[14]-125)/125.0,0.05);
//		ocu.rc_spd_w[Yr]=ocu_rx.rc_spd_w[Yr];//my_deathzoom(-(float)(RxBuffer4[15]-125)/125.0,0.05);
//		ocu.rc_att_w[PITr]=ocu_rx.rc_att_w[PITr];//my_deathzoom((float)(RxBuffer4[16]-125)/125.0,0.05);
//		ocu.rc_att_w[ROLr]=ocu_rx.rc_att_w[ROLr];//my_deathzoom(-(float)(RxBuffer4[17]-125)/125.0,0.05);
//		ocu.rate_yaw_w=ocu_rx.rate_yaw_w;//-(RxBuffer4[6]*-0.5+RxBuffer4[7]*0.5);
		SendBuff_USB[usb_send_cnt++]=ocu.connect*100+ocu.key_st*10+ocu.key_back;
		SendBuff_USB[usb_send_cnt++]=ocu.key_x*100+ocu.key_a*10+ocu.key_b;
		SendBuff_USB[usb_send_cnt++]=ocu.key_y*100+ocu.key_ll*10+ocu.key_rr;
		SendBuff_USB[usb_send_cnt++]=(ocu.key_lr+1)*10+(ocu.key_ud+1);
		setDataFloat_int(ocu.rc_spd_w[Xr],100);
		setDataFloat_int(ocu.rc_spd_w[Yr],100);
		setDataFloat_int(ocu.rc_att_w[PITr],100);
		setDataFloat_int(ocu.rc_att_w[ROLr],100);
		setDataFloat_int(ocu.rate_yaw_w,100);
  break;
	case 20:
		setDataFloat_int(robotwb.now_att.pitch,CAN_POS_DIV);
		setDataFloat_int(robotwb.now_att.roll,CAN_POS_DIV);
		setDataFloat_int(robotwb.now_att.yaw,CAN_POS_DIV);	
			
		setDataFloat_int(robotwb.now_rate.pitch,CAN_DPOS_DIV);
		setDataFloat_int(robotwb.now_rate.roll,CAN_DPOS_DIV);
		setDataFloat_int(robotwb.now_rate.yaw,CAN_DPOS_DIV);	
		for(id=0;id<4;id++){	
		setDataFloat_int(leg_motor.q_now[0],CAN_POS_DIV);
		setDataFloat_int(leg_motor.q_now[1],CAN_POS_DIV);
		setDataFloat_int(leg_motor.t_now[0],CAN_T_DIV);
		setDataFloat_int(leg_motor.t_now[1],CAN_T_DIV);	
		}
	break;
	case 21://整体发送
		setDataFloat_int(robotwb.now_att.pitch,CAN_POS_DIV);
		setDataFloat_int(robotwb.now_att.roll,CAN_POS_DIV);
		setDataFloat_int(robotwb.now_att.yaw,CAN_POS_DIV);	
			
//		setDataFloat_int(robotwb.now_rate.pitch,CAN_DPOS_DIV);
//		setDataFloat_int(robotwb.now_rate.roll,CAN_DPOS_DIV);
//		setDataFloat_int(robotwb.now_rate.yaw,CAN_DPOS_DIV);	
	
		setDataFloat(robotwb.now_rate.pitch);
		setDataFloat(robotwb.now_rate.roll);
		setDataFloat(robotwb.now_rate.yaw);	
	
		setDataFloat_int(vmc_all.acc_b.x,100);//vmc_all.acc[Xr]);
		setDataFloat_int(vmc_all.acc_b.y,100);//vmc_all.acc[Yr]);
		setDataFloat_int(vmc_all.acc_b.z,100);//vmc_all.acc[Zr]);	

		for(id=0;id<4;id++){
		setDataFloat_int(leg_motor.q_now[0],CAN_POS_DIV);
		setDataFloat_int(leg_motor.q_now[1],CAN_POS_DIV);
		setDataFloat_int(leg_motor.t_now[0],CAN_T_DIV);
		setDataFloat_int(leg_motor.t_now[1],CAN_T_DIV);	
		SendBuff_USB[usb_send_cnt++]=(leg_motor.connect*100+leg_motor.connect_motor[0]*10+leg_motor.ready[0]);
		SendBuff_USB[usb_send_cnt++]=(leg_motor.connect*100+leg_motor.connect_motor[1]*10+leg_motor.ready[1]);	
		}		
		setDataFloat_int(24,100);
		
		SendBuff_USB[usb_send_cnt++]=ocu.connect*100+ocu.key_st*10+ocu.key_back;
		SendBuff_USB[usb_send_cnt++]=ocu.key_x*100+ocu.key_a*10+ocu.key_b;
		SendBuff_USB[usb_send_cnt++]=ocu.key_y*100+ocu.key_ll*10+ocu.key_rr;
		SendBuff_USB[usb_send_cnt++]=(ocu.key_lr+1)*10+(ocu.key_ud+1);
		setDataFloat_int(ocu.rc_spd_w[Xr],100);
		setDataFloat_int(ocu.rc_spd_w[Yr],100);
		setDataFloat_int(ocu.rc_att_w[PITr],100);
		setDataFloat_int(ocu.rc_att_w[ROLr],100);
		setDataFloat_int(ocu.rate_yaw_w,100);
	break;
	case 99:
		setDataFloat(1.23);
		setDataFloat(-1.23);	
	break;
	}
	SendBuff_USB[3] =(usb_send_cnt)-4;
		for( i=0;i<usb_send_cnt;i++)
	sum_t += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum_t;
	
//	SendBuff_USB[usb_send_cnt++]=0;//for break
//	SendBuff_USB[usb_send_cnt++]=0;
}

void data_per_usb_bldc_config(void)//发送电机配置 Tinker14
{ 
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;

	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x73;
	SendBuff_USB[usb_send_cnt++]=0;
	
	SendBuff_USB[usb_send_cnt++] = motor_chassis[0].en_pos_trig;
	for(i=0;i<14;i++){//0 1
		SendBuff_USB[usb_send_cnt++] =	motor_chassis[i].en_cmd;
		SendBuff_USB[usb_send_cnt++] = (motor_chassis[i].param.connect+motor_chassis[i].motor.ready*10);
		SendBuff_USB[usb_send_cnt++] = (motor_chassis[i].motor.type);
		SendBuff_USB[usb_send_cnt++] =  motor_chassis[i].param.control_mode;		
		SendBuff_USB[usb_send_cnt++] = (motor_chassis[i].param.t_inv_flag_cmd+1);
		SendBuff_USB[usb_send_cnt++] = (motor_chassis[i].param.t_inv_flag_measure+1);
		SendBuff_USB[usb_send_cnt++] = (motor_chassis[i].param.q_flag);
		SendBuff_USB[usb_send_cnt++] =	motor_chassis[i].param.err_flag;
		setDataFloat(motor_chassis[i].param.q_reset_angle);	
		setDataFloat(motor_chassis[i].stiff);	
	}	

	setDataFloat(vmc_all.version[0]);
	setDataFloat(vmc_all.version[1]);
	setDataInt(vmc_all.board_id[0]*10000+vmc_all.board_id[1]*100+vmc_all.board_id[2]);
	SendBuff_USB[usb_send_cnt++]=(vmc_all.key_right);
	setDataInt(vmc_all.your_key[0]*10000+vmc_all.your_key[1]*100+vmc_all.your_key[2]);
	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

void data_per_usb_bldc_data(void)//发送电机状态 反馈值 tinker14
{ 
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x71;
	SendBuff_USB[usb_send_cnt++]=0;
	
	for(i=0;i<14;i++){
		setDataFloat(motor_chassis[i].set_q);	
		setDataFloat(motor_chassis[i].q_now_flt);
		setDataFloat(motor_chassis[i].qd_now_flt);
		setDataFloat(motor_chassis[i].t_now_flt);//扭矩反馈
	}
	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


void data_per_usb_vr_data(void)//VR arm control 
{ 
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x95;
	SendBuff_USB[usb_send_cnt++]=0;
	
	SendBuff_USB[usb_send_cnt++]=servo_ss[i].connect;

	#if 1//rl boy
		//arm
		
		setDataFloat(servo_ss[1].q);//l0	
		setDataFloat(servo_ss[2].q);//l1	
		setDataFloat(servo_ss[3].q);//l2	
		setDataFloat(servo_ss[4].q);//l3	
		setDataFloat(servo_ss[5].q);//l4	
		setDataFloat(0);//l5	
	  setDataFloat(0);//l6	
	
		setDataFloat(servo_ss[7].q);//l0	
		setDataFloat(servo_ss[8].q);//l1	
		setDataFloat(servo_ss[9].q);//l2	
		setDataFloat(servo_ss[10].q);//l3	
		setDataFloat(servo_ss[11].q);//l4	
		setDataFloat(0);//l5	
	  setDataFloat(0);//l6	
	
		setDataFloat(servo_ss[6].q);//capl	
		setDataFloat(servo_ss[12].q);//capr	
		//base
		setDataFloat(0);//dx
		setDataFloat(0);//dy
		setDataFloat(0);//dyaw
		//head
		setDataFloat(0);//pit
		setDataFloat(0);//yaw
		//body
		setDataFloat(0);//pit
		setDataFloat(0);//rol
		setDataFloat(servo_ss[0].q);//yaw
		setDataFloat(0);//height
	#else
		//arm
		for(i=0;i<14;i++){
			setDataFloat(servo_ss[i].q);	
		}
		//base
		setDataFloat(servo_ss[i].q);//dx
		setDataFloat(servo_ss[i].q);//dy
		setDataFloat(servo_ss[i].q);//dyaw
		//body
		setDataFloat(servo_ss[i].q);//pit
		setDataFloat(servo_ss[i].q);//rol
		setDataFloat(servo_ss[i].q);//yaw
		setDataFloat(servo_ss[i].q);//height
	#endif
	//key 
	SendBuff_USB[usb_send_cnt++]=0;
	SendBuff_USB[usb_send_cnt++]=0;
	SendBuff_USB[usb_send_cnt++]=0;
	SendBuff_USB[usb_send_cnt++]=0;
	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}

void use_vcp_comm(float dt)//发送
{
	static char state=0,i;
	static char state_flag=0,flag1=0;
	int len,t;
	static int usb_send_cnt_send=0,send_flag=0;
	static float timer_sys=0;
	u8 usbstatus=0;	
	if(usbstatus!=bDeviceState)//USB连接状态发生了改变.
			usbstatus=bDeviceState;//记录新的状态
 
	  usb_send_cnt=0;
		spi_dt[0] = Get_Cycle_T(16); 
		
		timer_sys+=spi_dt[0];

		data_per_usb_comm(21);
 
		VCP_DataTxBuf(SendBuff_USB,usb_send_cnt);
		usb_send_cnt=0;
}


void use_vcp_record(float dt)//发送
{
	static char state=0,i;
	static char state_flag=0,flag1=0;
	int len,t;
	u8 usbstatus=0;	
	if(usbstatus!=bDeviceState)//USB连接状态发生了改变.
			usbstatus=bDeviceState;//记录新的状态
	
	  usb_send_cnt=0;
		
			switch(state){
				case 0:
				  data_per_usb_record1_config(ocu.up_mode-66);//显示腿号
				state++;
				break;
				case 1:
					data_per_usb_record1_config(4);//机体COM信息
				state++;
				break;		
				case 2:
					data_per_usb_record1_config(5);//而外记录
				state=0;
				break;	
		}
			
		for(t=0;t<usb_send_cnt;t++)
		{
			if(usbstatus==1)
				VCP_DataTx(SendBuff_USB[t]);//以字节方式,发送给USB 
		}
		usb_send_cnt=0;
}


char cnt_usb_send[3]={0};
char cnt_uart_send[3]={0};
u8 uart_send_usb_done=0;
int uart_send_usb_cnt=0;
char force_record=0;
void use_bldc_test(float dt)//电机配置发送 Tinker14电机模式
{
	static char state=0;
	static char state_flag=0,flag1=0;
	
	int len,t;
	u8 usbstatus=0;	
	if(usbstatus!=bDeviceState)//USB连接状态发生了改变.
			usbstatus=bDeviceState;//记录新的状态
	
	  usb_send_cnt=0;
	if(bDeviceState){//仅连接USB发送
		usb_loss=0;
		can_cmd_usb_disable=1;
		switch(state){
			case 0:
			cnt_uart_send[0]++;
			if(cnt_uart_send[0]>2){
				cnt_uart_send[0]=0;
				data_per_usb_bldc_config();
			}else	
			#if USE_VR
				data_per_usb_vr_data();
			#else
				data_per_usb_bldc_data();
			#endif
			break;
		}
		
		for(t=0;t<usb_send_cnt;t++)
		{
			if(usbstatus==1)
				VCP_DataTx(SendBuff_USB[t]);//以字节方式,发送给USB 
		}
		usb_send_cnt=0;
	}else
		usb_loss+=dt;
	
	if(usb_loss>0.5){
		usb_loss=0;
		can_cmd_usb_disable=0;
	}
}

void use_vcp_test(float dt)//发送机器人状态 
{
	static char state=0;
	static char state_flag=0,flag1=0;
	int len,t;
	u8 usbstatus=0;	
	if(usbstatus!=bDeviceState)//USB连接状态发生了改变.
			usbstatus=bDeviceState;//记录新的状态
	
	  usb_send_cnt=0;
		if(ocu.up_mode==88){//sdk 88	
					switch(cnt_usb_send[1]){
						case 0:
						data_per_sdk1();
						cnt_usb_send[1]++;
						break;
						case 1:
						data_per_sdk2();
						cnt_usb_send[1]++;
						break;
						case 2:
						data_per_sdk3();
						cnt_usb_send[1]=0;
						break;
						default:
						cnt_usb_send[1]=0;
						break;
					}				
			
		}
		else if(ocu.record.en_record==1||force_record)//记录模式
		{
						flag1=!flag1;
						switch(cnt_usb_send[2]){
						case 0:
						if(flag1)
						data_per_usb_record1_config(0);
						else
						data_per_usb_record1_config(1);
						cnt_usb_send[2]++;
						break;
						case 1:
						if(flag1)
						data_per_usb_record1_config(2);
						else
						data_per_usb_record1_config(3);
						cnt_usb_send[2]++;
						break;
						case 2:
						if(flag1)
						data_per_usb_record1_config(4);//com
						else
						data_per_usb_record1_config(5);//temp
						cnt_usb_send[2]=0;
						break;
						default:
						cnt_usb_send[2]=0;	
						break;
						}
		}
		else{//-------------------正常模式---------------
			switch(state){
				case 0:
					cnt_usb_send[0]++;
					if(cnt_usb_send[0]>2){
						cnt_usb_send[0]=0;
						data_per_usb_sensor_config();
					}else{				
						switch(cnt_usb_send[1]){
							case 0:
							data_per_usb_controller_config();
							cnt_usb_send[1]++;
							break;
							case 1:
							data_per_usb_param_config();
							cnt_usb_send[1]++;
							break;
							case 2:
							data_per_usb_pwm_config();
							cnt_usb_send[1]=0;
							break;
						}				
					}
				state++;
				break;
				case 1:
					if(state_flag){state_flag=0;
						data_per_usb_robot_state1();}
					else{
							state_flag=1;
			
					flag1=!flag1;
						if(flag1)
							data_per_usb_robot_state2();
						else
							data_per_usb_curve_config();
					}
				state=0;
				break;
			}
		}
		
		for(t=0;t<usb_send_cnt;t++)
		{
			if(usbstatus==1)
				VCP_DataTx(SendBuff_USB[t]);//以字节方式,发送给USB 
		}
		usb_send_cnt=0;
}

void use_uart_test(float dt)//发送
{
	static char state=0;
	static char state_flag=0;
	static char flag1=0;
	int len,t;

	  if(FORCE_UP_FULL){//||ocu.up_mode==0){//复杂回传模式
		if(ocu.up_mode==88){//sdk 88	
			switch(cnt_uart_send[1]){
			case 0:
			data_per_sdk1();
			cnt_uart_send[1]++;
			break;
			case 1:
			data_per_sdk2();
			cnt_uart_send[1]=0;
			break;
			case 2:
			data_per_sdk3();
			cnt_uart_send[1]=0;
			break;
			default:
			cnt_uart_send[1]=0;
			break;
		}				
			
		}else{
			switch(state){
				case 0:
					cnt_uart_send[0]++;
					if(cnt_uart_send[0]>2){
						cnt_uart_send[0]=0;
						data_per_usb_sensor_config();
					}else{				
						switch(cnt_uart_send[1]){
							case 0:
							data_per_usb_controller_config();
							cnt_uart_send[1]++;
							break;
							case 1:
							data_per_usb_param_config();
							cnt_uart_send[1]++;
							break;
							case 2:
							data_per_usb_pwm_config();
							cnt_uart_send[1]=0;
							break;
						}				
					}
				state++;
				break;
				case 1:
					if(state_flag){state_flag=0;
						data_per_usb_robot_state1();}
					else{state_flag=1;
						flag1=!flag1;
						if(flag1)
							data_per_usb_curve_config();
						else
							data_per_usb_robot_state2();
					}
				state=0;
				break;
			}
		}
	}else{//-----------------UART 简单回传模式
		  switch(state){
			case 0:
			data_per_usb_robot_state_simple1();
			state++;
			break;
			case 1:
			data_per_usb_robot_state_simple2();
			state++;
			break;
			case 2:
			data_per_usb_robot_state_simple3();
			state++;
			break;
			case 3:
			data_per_usb_robot_state_simple4();
			state++;
			break;
			case 4:
			data_per_usb_robot_state_simple5();
			state=0;
			break;
		}
	}

	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)
		{ 
			DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		
			clear_leg_uart();
			for(t=0;t<usb_send_cnt;t++)
				SendBuff1[t]=SendBuff_USB[t];
			USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);      
			MYDMA_Enable(DMA2_Stream7,usb_send_cnt+2);  
		}	
		usb_send_cnt=0;
}