#include "gps.h" 
#include "delay.h" 								   
#include "usart_fc.h"						   
#include "stdio.h"	 
#include "stdarg.h"	 
#include "string.h"	 
#include "math.h"
#include "time.h"
#define GPS_UART	USART2		
GPS_INF Gps_information;
unsigned short len;		  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n����
//����ֵ:m^n�η�.
u32 NMEA_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}
//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת�������ֵ
int NMEA_Str2num(u8 *buf,u8*dx)
{
	u8 *p=buf;
	u32 ires=0,fres=0;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	int res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
	for(i=0;i<ilen;i++)	//�õ�������������
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//���ȡ5λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	

float NMEA_Str2float(u8 *buf,u8*dx)
{
	u8 *p=buf;
	u32 ires=0,fres=0;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	float res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
	for(i=0;i<ilen;i++)	//�õ�������������
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//���ȡ5λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires+(float)fres/NMEA_Pow(10,flen);
	if(mask&0X02)res=-res;		   
	return res;
}	  	
//����GPGSV��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p,*p1,dx;
	u8 len,i,j,slx=0;
	u8 posx;   	 
	p=buf;
	p1=(u8*)strstr((const char *)p,"$GPGSV");
	len=p1[7]-'0';								//�õ�GPGSV������
	posx=NMEA_Comma_Pos(p1,3); 					//�õ��ɼ���������
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(u8*)strstr((const char *)p,"$GPGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//�õ����Ǳ��
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//�õ��������� 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//�õ����Ƿ�λ��
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//�õ����������
			else break;
			slx++;	   
		}   
 		p=p1+1;//�л�����һ��GPGSV��Ϣ
	}   
}
//����GPGGA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
/*
��λ����ָʾ	��������
0	������Ч�򲻿���
1	����
2	α����
4	RTK�̶���  //---
5	RTK������
6	��λ����ģʽ
7	�ֶ�ģʽ
8	ģ��ģʽ
9	WAAS (SBAS)ģʽ
*/
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"GPGGA");
	posx=NMEA_Comma_Pos(p1,6);								//�õ�GPS״̬
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);								//�õ����ڶ�λ��������
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	posx=NMEA_Comma_Pos(p1,9);								//�õ����θ߶�
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}
//����GPGSA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx; 
	u8 i;   
	p1=(u8*)strstr((const char *)buf,"$GNGSA");
	posx=NMEA_Comma_Pos(p1,2);								//�õ���λ����
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//�õ���λ���Ǳ��
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//�õ�PDOPλ�þ�������
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								//�õ�HDOPλ�þ�������
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								//�õ�VDOPλ�þ�������
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}
//����GPRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��    longitude-->����   LAT--��ά��
//buf:���յ���GPS���ݻ������׵�ַ25.0772235
u8 GPS_SEL1=7;
float gps_dt;
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{u32 temp_gps;
	u8 *p1,dx;			 
	u8 posx;     
	u32 temp;	   
	float rs;  
	p1=(u8*)strstr((const char *)buf,"GPRMC");//"$GPRMC",������&��GPRMC�ֿ������,��ֻ�ж�GPRMC.
	posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//�õ�UTCʱ��,ȥ��ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}	
	posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
	
	if(posx!=0XFF)
	{ //gps_dt = Get_Cycle_T(GET_T_GPS);		
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		temp_gps=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		temp_gps=temp_gps*NMEA_Pow(10,GPS_SEL1)+(rs*NMEA_Pow(10,GPS_SEL1-dx))/60;//ת��Ϊ�� 
		gpsx->latitude=(double)temp_gps/NMEA_Pow(10,GPS_SEL1);
	}
	posx=NMEA_Comma_Pos(p1,4);								//��γ���Ǳ�γ 
	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//�õ�����
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		temp_gps=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		temp_gps=temp_gps*NMEA_Pow(10,GPS_SEL1)+(rs*NMEA_Pow(10,GPS_SEL1-dx))/60;//ת��Ϊ�� 
		gpsx->longitude=(double)temp_gps/NMEA_Pow(10,GPS_SEL1);
	}
	posx=NMEA_Comma_Pos(p1,6);								//������������
	if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//�õ�UTC����
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	 	 
	} 
	posx=NMEA_Comma_Pos(p1,7);		//�ٶ�  m/s
	if(posx!=0XFF)
		{ 
			gpsx->spd=(float)(NMEA_Str2float(p1+posx,&dx)*1.852*1000)/3600.;       //???????????????(0-359?)(??100?)
		}
		posx=NMEA_Comma_Pos(p1,8);		//�Ƕ�
	if(posx!=0XFF)
		{
			gpsx->angle=NMEA_Str2float(p1+posx,&dx);       //???????????????(0-359?)(??100?)
		}	
	/*
ģʽ��д	ģʽָʾ
A	������λ
D	���
E	����ģʽ
M	�ֶ�����
N	������Ч
*/		
	posx=NMEA_Comma_Pos(p1,2);								
	if(posx!=0XFF)gpsx->rmc_mode=*(p1+posx);		 	
		
	
	posx=NMEA_Comma_Pos(p1,10);		//��ƫ�Ƕ�
	if(posx!=0XFF)
		{
			gpsx->angle_off=NMEA_Str2float(p1+posx,&dx);       
		}		
		
   posx=NMEA_Comma_Pos(p1,11);								//������������
	 if(posx!=0XFF)gpsx->ewhemi_angle_off=*(p1+posx);		 
	
}


//����GPVTG��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
u8 buf[50];
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GNVTG");			
	posx=NMEA_Comma_Pos(p1,1);								
	/*if(posx!=0XFF)
	{
		gpsx->course_earth=NMEA_Str2num(p1+posx,&dx);       //???????????????(0-359?)(??100?)
	}	
	posx=NMEA_Comma_Pos(p1,7);								//�õ���������
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//ȷ������1000��
	}*/
		p1=(uint8_t*)strstr((const char *)buf,"$GNVTG");				//?????$GNVTG?p?????????		 
		posx=NMEA_Comma_Pos(p1,1);			
	u8 i;
for(i=0;i<50;i++)
buf[i]=	posx++;
		if(posx!=0XFF)
		{
			gpsx->course_earth=NMEA_Str2num(p1+posx,&dx);       //???????????????(0-359?)(??100?)
		}
			posx=NMEA_Comma_Pos(p1,3);								
		if(posx!=0XFF)
		{
			gpsx->course_mag=NMEA_Str2num(p1+posx,&dx);         //???????????????(0-359?)(??100?)
		}
			posx=NMEA_Comma_Pos(p1,7);								//??????
		if(posx!=0XFF)
		{
			gpsx->speed=NMEA_Str2num(p1+posx,&dx);
			if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);             //????1000?(1??/?? -> 0.001??/??)
			else if(dx>3)gpsx->speed/=NMEA_Pow(10,dx-3);        //????1000?(1??/?? -> 0.001??/??)(??1000?)
		}
}  
//��ȡNMEA-0183��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
{
	NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV����
	NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA���� 	
	NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA����
	NMEA_GPRMC_Analysis(gpsx,buf);	//GPRMC����
	NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG����
}


unsigned char GPS_ubx_check_sum(unsigned char *Buffer)
{
	unsigned char CK_A = 0, CK_B = 0;
	unsigned short  i;
	
	len = Buffer[4] + (Buffer[5]<<8);
	if (len > 100)
	{
		return 0;
	}
	for (i = 2; i < len+6; i++)
	{
		CK_A = CK_A + Buffer[i];
		CK_B = CK_B + CK_A;
	}
	if (CK_A == Buffer[len+6] && CK_B == Buffer[len+7])
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

static void UART_Write_D(const unsigned char *send_buff, unsigned char len,u8 sel)
{
	unsigned char i;
	
	for (i = 0; i < len; i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)== RESET);
		USART_SendData(USART2,send_buff[i]);
	}
}

const unsigned char gps_petrol_out_config[28]=
{
//	0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xA0,0xA9
	0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xB8,0x42
};

const unsigned char gps_pvt_out_config[90]=
{
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1,
//	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x01,0x01,0x01,0x01,0x01,0x00,0x27,0x3D
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9,
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0,
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x16,0xD5,
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0x29
};

const unsigned char gps_rate_out_config[16]=
{
	0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12  //len 14
};

const unsigned char Enter_Send[]={0xB5,0x62,0x06,0x00,0x01,0x00,0x01,0x08,0x22};

void gps_baudrate_config(u8 sel)
{
	Delay_ms(200);
//		UART_Write_D(gps_rate_out_config,14);
	UART_Write_D(gps_petrol_out_config,28,sel);

	UART_Write_D(Enter_Send,sizeof(Enter_Send),sel);
		Delay_ms(100);
}

void gps_config(u8 sel)
{
	Delay_ms(100);

	UART_Write_D(gps_pvt_out_config,90,sel);
		Delay_ms(100);

	UART_Write_D(gps_rate_out_config,14,sel);
		Delay_ms(100);

//	UART_Write_D(gps_petrol_out_config,28);
//		Delay_ms(20);
	
	UART_Write_D(Enter_Send,sizeof(Enter_Send),sel);
		Delay_ms(100);
}


extern u8 data_to_send[50];
extern void ANO_DT_Send_Data(u8 *dataToSend , u8 length);
short Gps_send_Temp[10];
float wcx_acc_use;		
float wcy_acc_use;


void Drv_GpsPin_Init(void)
{ 
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	u8 sel=2;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	static u8 cnt;
	 for(cnt=0;cnt<5;cnt++){
	//??PC12??UART5 Tx

	USART_DeInit(GPS_UART);
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate          =   9600;//38400;//9600;//38400;
	USART_InitStructure.USART_WordLength        =   USART_WordLength_8b;
	USART_InitStructure.USART_StopBits          =   USART_StopBits_1;
	USART_InitStructure.USART_Parity            =   USART_Parity_No;
	USART_InitStructure.USART_Mode              =   USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(GPS_UART,&USART_InitStructure);
	USART_Cmd(GPS_UART,ENABLE);
	
	gps_baudrate_config(sel);
	Delay_ms(200);
  gps_config(sel);
  USART_Cmd(GPS_UART,DISABLE);
	USART_InitStructure.USART_BaudRate          =   38400;//38400;//9600;//38400;
	USART_InitStructure.USART_WordLength        =   USART_WordLength_8b;
	USART_InitStructure.USART_StopBits          =   USART_StopBits_1;
	USART_InitStructure.USART_Parity            =   USART_Parity_No;
	USART_InitStructure.USART_Mode              =   USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(GPS_UART,&USART_InitStructure);
	USART_Cmd(GPS_UART,ENABLE);
	
	gps_baudrate_config(sel);
	Delay_ms(200);
  gps_config(sel);
	
	USART_Cmd(GPS_UART,DISABLE);
	USART_InitStructure.USART_BaudRate          =   19200;//38400;//9600;//38400;
	USART_InitStructure.USART_WordLength        =   USART_WordLength_8b;
	USART_InitStructure.USART_StopBits          =   USART_StopBits_1;
	USART_InitStructure.USART_Parity            =   USART_Parity_No;
	USART_InitStructure.USART_Mode              =   USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(GPS_UART,&USART_InitStructure);
	USART_Cmd(GPS_UART,ENABLE);
	
	gps_baudrate_config(sel);
	Delay_ms(200);
  gps_config(sel);
	
	USART_Cmd(GPS_UART,DISABLE);
	USART_InitStructure.USART_BaudRate          =   57600;//38400;//9600;//38400;
	USART_InitStructure.USART_WordLength        =   USART_WordLength_8b;
	USART_InitStructure.USART_StopBits          =   USART_StopBits_1;
	USART_InitStructure.USART_Parity            =   USART_Parity_No;
	USART_InitStructure.USART_Mode              =   USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(GPS_UART,&USART_InitStructure);
	USART_Cmd(GPS_UART,ENABLE);
	
	gps_baudrate_config(sel);
	Delay_ms(200);
  gps_config(sel);
	//??????
	USART_InitStructure.USART_BaudRate = 115200;       //????????????
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8???
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //??????1????
	USART_InitStructure.USART_Parity = USART_Parity_No;    //??????
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //???????
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //???????
	USART_Init ( GPS_UART, &USART_InitStructure );
	USART_Cmd ( GPS_UART, ENABLE );		
	gps_config(sel);
	
	//??UART5????
	USART_ITConfig ( GPS_UART, USART_IT_RXNE, ENABLE );
	}
}

unsigned char GPS_data_buff[100];
unsigned char GPS_get_cnt = 0;

void GPS_data_analysis(GPS_INF *in, unsigned char buff[100])
{ 
	in->connect=1;
	in->lose_cnt=0;
	
	in->update[0]=in->update[1]=in->update[2]=in->update[3]=1;
	in->last_N_vel = (float)in->N_vel;							//��¼�ϴ��ϱ����ٶ�
	in->last_E_vel = (float)in->E_vel;							//��¼�ϴζ������ٶ�
	in->fix_type=buff[20+6];
	in->satellite_num = buff[29];									//��������
	in->longitude = buff[30] + (buff[31]<<8) + (buff[32]<<16) + (buff[33]<<24);		//����
	in->longitude /=10000000.;
	in->latitude  = buff[34] + (buff[35]<<8) + (buff[36]<<16) + (buff[37]<<24);
	in->latitude /=10000000.;	//γ��
	in->N_vel	  = buff[54] + (buff[55]<<8) + (buff[56]<<16) + (buff[57]<<24);		//�ϱ����ٶ�
	in->E_vel	  = buff[58] + (buff[59]<<8) + (buff[60]<<16) + (buff[61]<<24);		//�������ٶ�
	
	in->N_vel /=10;								//��λ���� cm/s
	in->E_vel /=10;								//��λ���� cm/s	
  in->real_D_vel= -(float)(int)((((u32)buff[59+6])<<24)|((u32)buff[58+6])<<16|((u32)buff[57+6])<<8|((u32)buff[56+6]))/1000.;
	in->Speed_norm=(float)(int)((((u32)buff[63+6])<<24)|((u32)buff[62+6])<<16|((u32)buff[61+6])<<8|((u32)buff[60+6]))/1000.;	

	in->PVT_Hacc=((((u32)buff[43+6])<<24)|((u32)buff[42+6])<<16|((u32)buff[41+6])<<8|((u32)buff[40+6]));		
	in->PVT_Vacc=((((u32)buff[47+6])<<24)|((u32)buff[46+6])<<16|((u32)buff[45+6])<<8|((u32)buff[44+6]));		 
	in->PVT_Sacc=((((u32)buff[71+6])<<24)|((u32)buff[70+6])<<16|((u32)buff[69+6])<<8|((u32)buff[68+6]));		
	in->PVT_Headacc=((((u32)buff[75+6])<<24)|((u32)buff[74+6])<<16|((u32)buff[73+6])<<8|((u32)buff[72+6]))*1e-5;			
	
	if (in->satellite_num >= 6 && in->new_pos_get == 0)			//������������6��,�ҵ�һ�λ�ȡ��γ�ȵ�
	{
		in->new_pos_get = 1;
		in->start_longitude = in->longitude;
		in->start_latitude  = in->latitude;
		in->hope_latitude  = 0;
		in->hope_longitude = 0;

	}
	
	if (in->new_pos_get)
	{
		in->latitude_offset  = in->latitude  - in->start_latitude;
		in->longitude_offset = in->longitude - in->start_longitude;
	}

	in->run_heart++;	
}

void GPS_IRQ(u8 RX_dat )  
{  
		if (GPS_get_cnt == 0)
		{
			if (RX_dat == 0xB5)									//֡ͷ1
			{
				GPS_data_buff[GPS_get_cnt] = RX_dat;
				GPS_get_cnt = 1;
			}
		}
		else if (GPS_get_cnt == 1)
		{
			if (RX_dat == 0x62)									//֡ͷ2
			{
				GPS_data_buff[GPS_get_cnt] = RX_dat;
				GPS_get_cnt = 2;
			}
			else
			{
				GPS_get_cnt = 0;
			}
		}
		else
		{
			GPS_data_buff[GPS_get_cnt] = RX_dat;
			GPS_get_cnt++;
			if (GPS_get_cnt >= 100)
			{
				GPS_get_cnt = 0;
				
				if (GPS_ubx_check_sum(GPS_data_buff))			//GPS����У��
				{
					GPS_data_analysis(&Gps_information,GPS_data_buff);						//GPS���ݽ���
				}
			}
		}			
} 



void GPS_Data_Processing_Task(u8 dT_ms)
{
	static float err_N_step, err_E_step;
	static unsigned char last_gps_heart = 0; 
	
	if (Gps_information.run_heart != last_gps_heart)
	{
		last_gps_heart = Gps_information.run_heart;
		err_N_step = (float)(Gps_information.N_vel - Gps_information.last_N_vel)/10;		//��������GPS�ٶ������Ϊ��ֵ
		err_E_step = (float)(Gps_information.E_vel - Gps_information.last_E_vel)/10;
	}
	Gps_information.last_N_vel += err_N_step;									//���ٶȲ�ֵ
	Gps_information.last_E_vel += err_E_step;
	
	if(Gps_information.satellite_num>5)
	{
	  Gps_information.real_E_vel=Gps_information.last_E_vel/100.;
	  Gps_information.real_N_vel=Gps_information.last_N_vel/100.;
	}else
	Gps_information.real_E_vel=Gps_information.real_N_vel=0;
}


// 1 byte - 4 bits for value + 1 bit for sign + 3 bits for repeats => 8 bits
typedef struct
{
    // Offset has a max value of 15
    uint8_t abs_offset : 4;
    // Sign of the offset, 0 = negative, 1 = positive
    uint8_t offset_sign : 1;
    // The highest repeat is 7
    uint8_t repeats : 3;
}
row_value;


// 730 bytes
static const uint8_t exceptions[10][73] =
{
    {150,145,140,135,130,125,120,115,110,105,100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5,0,4,9,14,19,24,29,34,39,44,49,54,59,64,69,74,79,84,89,94,99,104,109,114,119,124,129,134,139,144,149,154,159,164,169,174,179,175,170,165,160,155,150},
    {143,137,131,126,120,115,110,105,100,95,90,85,80,75,71,66,62,57,53,48,44,39,35,31,27,22,18,14,9,5,1,3,7,11,16,20,25,29,34,38,43,47,52,57,61,66,71,76,81,86,91,96,101,107,112,117,123,128,134,140,146,151,157,163,169,175,178,172,166,160,154,148,143},
    {130,124,118,112,107,101,96,92,87,82,78,74,70,65,61,57,54,50,46,42,38,34,31,27,23,19,16,12,8,4,1,2,6,10,14,18,22,26,30,34,38,43,47,51,56,61,65,70,75,79,84,89,94,100,105,111,116,122,128,135,141,148,155,162,170,177,174,166,159,151,144,137,130},
    {111,104,99,94,89,85,81,77,73,70,66,63,60,56,53,50,46,43,40,36,33,30,26,23,20,16,13,10,6,3,0,3,6,9,13,16,20,24,28,32,36,40,44,48,52,57,61,65,70,74,79,84,88,93,98,103,109,115,121,128,135,143,152,162,172,176,165,154,144,134,125,118,111},
    {85,81,77,74,71,68,65,63,60,58,56,53,51,49,46,43,41,38,35,32,29,26,23,19,16,13,10,7,4,1,1,3,6,9,13,16,19,23,26,30,34,38,42,46,50,54,58,62,66,70,74,78,83,87,91,95,100,105,110,117,124,133,144,159,178,160,141,125,112,103,96,90,85},
    {62,60,58,57,55,54,52,51,50,48,47,46,44,42,41,39,36,34,31,28,25,22,19,16,13,10,7,4,2,0,3,5,8,10,13,16,19,22,26,29,33,37,41,45,49,53,56,60,64,67,70,74,77,80,83,86,89,91,94,97,101,105,111,130,109,84,77,74,71,68,66,64,62},
    {46,46,45,44,44,43,42,42,41,41,40,39,38,37,36,35,33,31,28,26,23,20,16,13,10,7,4,1,1,3,5,7,9,12,14,16,19,22,26,29,33,36,40,44,48,51,55,58,61,64,66,68,71,72,74,74,75,74,72,68,61,48,25,2,22,33,40,43,45,46,47,46,46},
    {6,9,12,15,18,21,23,25,27,28,27,24,17,4,14,34,49,56,60,60,60,58,56,53,50,47,43,40,36,32,28,25,21,17,13,9,5,1,2,6,10,14,17,21,24,28,31,34,37,39,41,42,43,43,41,38,33,25,17,8,0,4,8,10,10,10,8,7,4,2,0,3,6},
    {22,24,26,28,30,32,33,31,23,18,81,96,99,98,95,93,89,86,82,78,74,70,66,62,57,53,49,44,40,36,32,27,23,19,14,10,6,1,2,6,10,15,19,23,27,31,35,38,42,45,49,52,55,57,60,61,63,63,62,61,57,53,47,40,33,28,23,21,19,19,19,20,22},
    {168,173,178,176,171,166,161,156,151,146,141,136,131,126,121,116,111,106,101,96,91,86,81,76,71,66,61,56,51,46,41,36,31,26,21,16,11,6,1,3,8,13,18,23,28,33,38,43,48,53,58,63,68,73,78,83,88,93,98,103,108,113,118,123,128,133,138,143,148,153,158,163,168}
};

// 100 bytes
static const uint8_t exception_signs[10][10] =
{
    {0,0,0,1,255,255,224,0,0,0},
    {0,0,0,1,255,255,240,0,0,0},
    {0,0,0,1,255,255,248,0,0,0},
    {0,0,0,1,255,255,254,0,0,0},
    {0,0,0,3,255,255,255,0,0,0},
    {0,0,0,3,255,255,255,240,0,0},
    {0,0,0,15,255,255,255,254,0,0},
    {0,3,255,255,252,0,0,7,252,0},
    {0,127,255,255,252,0,0,0,0,0},
    {0,0,31,255,254,0,0,0,0,0}
};

// 76 bytes
static const uint8_t declination_keys[2][37] =
{ \
// Row start values
    {36,30,25,21,18,16,14,12,11,10,9,9,9,8,8,8,7,6,6,5,4,4,4,3,4,4,4},
// Row length values
    {39,38,33,35,37,35,37,36,39,34,41,42,42,28,39,40,43,51,50,39,37,34,44,51,49,48,55}
};

// 1056 total values @ 1 byte each = 1056 bytes
static const row_value declination_values[] =
{ \
    {0,0,4},{1,1,0},{0,0,2},{1,1,0},{0,0,2},{1,1,3},{2,1,1},{3,1,3},{4,1,1},{3,1,1},{2,1,1},{3,1,0},{2,1,0},{1,1,0},{2,1,1},{1,1,0},{2,1,0},{3,1,4},{4,1,1},{3,1,0},{4,1,0},{3,1,2},{2,1,2},{1,1,1},{0,0,0},{1,0,1},{3,0,0},{4,0,0},{6,0,0},{8,0,0},{11,0,0},{13,0,1},{10,0,0},{9,0,0},{7,0,0},{5,0,0},{4,0,0},{2,0,0},{1,0,2},
    {0,0,6},{1,1,0},{0,0,6},{1,1,2},{2,1,0},{3,1,2},{4,1,2},{3,1,3},{2,1,0},{1,1,0},{2,1,0},{1,1,2},{2,1,2},{3,1,3},{4,1,0},{3,1,3},{2,1,1},{1,1,1},{0,0,0},{1,0,1},{2,0,0},{4,0,0},{5,0,0},{6,0,0},{7,0,0},{8,0,0},{9,0,0},{8,0,0},{6,0,0},{7,0,0},{6,0,0},{4,0,1},{3,0,0},{2,0,0},{1,0,0},{2,0,0},{0,0,0},{1,0,0},
    {0,0,1},{1,0,0},{0,0,1},{1,1,0},{0,0,6},{1,0,0},{1,1,0},{0,0,0},{1,1,1},{2,1,1},{3,1,0},{4,1,3},{3,1,0},{4,1,0},{3,1,1},{2,1,0},{1,1,7},{2,1,0},{3,1,6},{2,1,0},{1,1,2},{0,0,0},{1,0,0},{2,0,0},{3,0,1},{5,0,1},{6,0,0},{7,0,0},{6,0,2},{4,0,2},{3,0,1},{2,0,2},{1,0,1},
    {0,0,0},{1,0,0},{0,0,7},{0,0,5},{1,1,1},{2,1,1},{3,1,0},{4,1,5},{3,1,1},{1,1,0},{2,1,0},{1,1,0},{0,0,0},{1,1,0},{0,0,1},{1,1,0},{0,0,0},{2,1,2},{3,1,1},{2,1,0},{3,1,0},{2,1,1},{1,1,0},{0,0,1},{1,0,0},{2,0,1},{4,0,1},{5,0,4},{4,0,0},{3,0,1},{4,0,0},{2,0,0},{3,0,0},{2,0,2},{1,0,2},
    {0,0,0},{1,0,0},{0,0,7},{0,0,5},{1,1,2},{2,1,0},{4,1,0},{3,1,0},{5,1,0},{3,1,0},{5,1,0},{4,1,1},{3,1,0},{2,1,1},{1,1,2},{0,0,2},{1,0,0},{0,0,1},{1,1,0},{2,1,2},{3,1,0},{2,1,1},{1,1,1},{0,0,0},{1,0,0},{2,0,1},{3,0,1},{4,0,0},{5,0,0},{4,0,0},{5,0,0},{4,0,0},{3,0,1},{1,0,0},{3,0,0},{2,0,4},{1,0,3},
    {0,0,1},{1,0,0},{0,0,7},{1,1,0},{0,0,4},{1,1,0},{2,1,1},{3,1,0},{4,1,2},{5,1,0},{4,1,0},{3,1,1},{2,1,1},{1,1,1},{0,0,2},{1,0,1},{2,0,0},{1,0,0},{0,0,0},{1,1,1},{2,1,3},{1,1,1},{1,0,2},{2,0,0},{3,0,1},{4,0,2},{3,0,1},{2,0,0},{1,0,0},{2,0,1},{1,0,0},{2,0,1},{1,0,0},{2,0,0},{1,0,3},
    {0,0,2},{1,0,0},{0,0,5},{1,1,0},{0,0,4},{1,1,2},{2,1,0},{4,1,0},{3,1,0},{4,1,1},{5,1,0},{4,1,0},{3,1,1},{2,1,0},{1,1,1},{0,0,2},{1,0,0},{2,0,0},{1,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,1},{2,1,2},{1,1,0},{2,1,0},{0,0,1},{1,0,1},{2,0,1},{3,0,2},{4,0,0},{2,0,1},{1,0,2},{2,0,0},{1,0,1},{2,0,0},{1,0,5},
    {0,0,0},{1,0,0},{0,0,7},{0,0,1},{1,1,0},{0,0,2},{1,1,2},{3,1,2},{4,1,3},{3,1,0},{2,1,1},{1,1,0},{0,0,2},{1,0,1},{2,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,0},{1,1,0},{2,1,0},{1,1,0},{2,1,1},{0,0,0},{1,1,0},{1,0,2},{2,0,1},{3,0,1},{2,0,1},{1,0,1},{0,0,0},{1,0,2},{2,0,0},{1,0,5},
    {0,0,4},{1,0,0},{0,0,3},{1,1,0},{0,0,3},{1,1,0},{0,0,0},{1,1,0},{2,1,1},{3,1,1},{4,1,3},{3,1,0},{2,1,0},{1,1,0},{0,0,2},{1,0,0},{2,0,3},{3,0,0},{2,0,0},{3,0,0},{1,0,1},{1,1,1},{2,1,0},{1,1,0},{2,1,0},{1,1,0},{0,0,2},{1,0,0},{2,0,0},{1,0,0},{2,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,0},{1,0,7},{1,0,1},
    {0,0,7},{0,0,5},{1,1,0},{0,0,1},{2,1,0},{1,1,0},{3,1,3},{4,1,1},{3,1,1},{1,1,1},{0,0,1},{1,0,0},{2,0,3},{3,0,0},{2,0,3},{0,0,2},{2,1,0},{1,1,0},{2,1,0},{1,1,0},{0,0,0},{1,1,0},{1,0,0},{0,0,0},{1,0,0},{2,0,0},{1,0,0},{2,0,1},{0,0,0},{1,0,0},{0,0,1},{1,0,0},{0,0,0},{1,0,7},
    {0,0,6},{1,0,0},{0,0,0},{1,1,0},{0,0,4},{1,1,0},{0,0,0},{2,1,0},{1,1,0},{3,1,0},{2,1,0},{4,1,0},{3,1,0},{4,1,1},{2,1,2},{0,0,1},{1,0,0},{2,0,7},{2,0,0},{1,0,1},{0,0,1},{1,1,1},{2,1,0},{1,1,0},{0,0,0},{1,1,0},{0,0,0},{1,0,0},{0,0,0},{1,0,1},{2,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,2},{1,0,1},{0,0,0},{2,0,0},{1,0,2},{0,0,0},{1,0,0},
    {0,0,7},{0,0,3},{1,1,0},{0,0,2},{1,1,0},{2,1,0},{1,1,0},{3,1,0},{2,1,0},{4,1,0},{3,1,0},{4,1,0},{3,1,0},{2,1,1},{1,1,0},{0,0,0},{1,0,1},{2,0,1},{3,0,0},{2,0,2},{1,0,0},{2,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,0},{1,1,0},{0,0,0},{2,1,0},{1,1,0},{0,0,0},{1,1,0},{0,0,1},{1,0,0},{0,0,0},{1,0,2},{0,0,3},{1,0,0},{0,0,0},{1,0,6},{0,0,0},{1,0,0},
    {0,0,2},{1,1,0},{0,0,1},{1,0,0},{0,0,3},{1,1,0},{0,0,2},{1,1,2},{2,1,0},{3,1,0},{2,1,0},{3,1,0},{4,1,0},{3,1,1},{2,1,0},{1,1,1},{0,0,0},{1,0,0},{2,0,2},{3,0,0},{2,0,1},{1,0,0},{2,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,2},{1,1,0},{0,0,0},{1,1,1},{0,0,0},{1,1,0},{0,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,5},{1,0,7},{0,0,0},{1,0,0},
    {0,0,5},{1,0,0},{0,0,4},{1,1,0},{0,0,1},{1,1,1},{2,1,2},{3,1,4},{2,1,0},{1,1,0},{0,0,0},{1,0,1},{2,0,6},{1,0,1},{0,0,0},{1,0,1},{0,0,2},{1,1,1},{0,0,0},{1,1,0},{0,0,1},{1,1,0},{0,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,7},{1,0,7},
    {0,0,3},{1,0,0},{0,0,7},{1,1,0},{0,0,0},{1,1,0},{2,1,3},{3,1,3},{2,1,0},{1,1,1},{0,0,0},{1,0,1},{2,0,2},{3,0,0},{1,0,0},{2,0,0},{1,0,0},{2,0,0},{0,0,1},{1,0,1},{0,0,2},{1,1,0},{0,0,0},{1,1,0},{0,0,1},{1,1,0},{0,0,3},{1,0,0},{0,0,2},{1,1,0},{0,0,3},{1,0,0},{0,0,0},{1,0,0},{2,0,0},{1,0,1},{2,0,0},{0,0,0},{1,0,0},
    {0,0,1},{1,0,0},{0,0,2},{1,0,0},{0,0,5},{1,1,2},{2,1,1},{3,1,0},{2,1,0},{3,1,2},{2,1,1},{1,1,0},{0,0,1},{1,0,0},{2,0,0},{1,0,0},{2,0,4},{1,0,1},{0,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,0},{1,1,0},{0,0,0},{1,1,1},{0,0,7},{0,0,0},{1,1,0},{0,0,0},{1,1,0},{0,0,3},{1,0,1},{0,0,0},{1,0,0},{2,0,0},{1,0,0},{2,0,0},{1,0,0},{1,0,0},
    {0,0,0},{1,0,1},{0,0,1},{1,0,0},{0,0,0},{1,0,0},{0,0,3},{1,1,0},{0,0,0},{1,1,0},{2,1,2},{3,1,0},{2,1,0},{4,1,0},{3,1,0},{2,1,2},{1,1,0},{0,0,0},{1,0,2},{2,0,4},{1,0,0},{2,0,0},{0,0,0},{1,0,0},{0,0,0},{1,0,1},{0,0,2},{1,1,0},{0,0,0},{1,1,0},{0,0,0},{1,1,0},{0,0,5},{1,1,0},{0,0,0},{1,1,1},{0,0,0},{1,1,0},{0,0,1},{1,0,4},{2,0,1},{1,0,0},{1,0,0},
    {0,0,0},{2,0,0},{1,0,0},{0,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,3},{1,1,0},{0,0,0},{2,1,2},{3,1,0},{2,1,0},{3,1,0},{4,1,0},{3,1,0},{2,1,1},{1,1,1},{1,0,0},{0,0,0},{2,0,0},{1,0,0},{2,0,1},{1,0,0},{2,0,2},{1,0,0},{0,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,0},{1,0,0},{1,1,0},{0,0,1},{1,1,0},{0,0,0},{1,1,0},{0,0,1},{1,1,0},{0,0,2},{1,1,3},{0,0,0},{1,1,0},{0,0,2},{1,0,0},{2,0,0},{1,0,1},{2,0,0},{1,0,0},{2,0,0},{1,0,0},
    {0,0,0},{1,0,1},{2,0,0},{1,0,1},{0,0,0},{1,0,0},{0,0,0},{1,0,0},{0,0,0},{1,1,0},{0,0,0},{2,1,0},{1,1,0},{2,1,0},{3,1,1},{2,1,0},{4,1,1},{3,1,0},{2,1,1},{1,1,0},{0,0,1},{1,0,0},{2,0,0},{1,0,0},{2,0,2},{1,0,0},{2,0,1},{1,0,0},{0,0,0},{1,0,2},{0,0,0},{1,0,0},{0,0,3},{1,1,0},{0,0,1},{1,1,0},{0,0,0},{1,1,0},{0,0,0},{1,1,0},{0,0,0},{1,1,2},{2,1,0},{1,1,1},{0,0,1},{1,0,3},{2,0,0},{1,0,0},{2,0,1},{2,0,0},
    {0,0,0},{2,0,0},{1,0,0},{2,0,0},{1,0,4},{0,0,1},{1,1,0},{0,0,0},{2,1,0},{1,1,0},{3,1,3},{4,1,1},{3,1,0},{2,1,1},{1,1,0},{0,0,0},{1,0,2},{2,0,0},{1,0,0},{2,0,4},{1,0,0},{0,0,0},{1,0,3},{0,0,0},{1,0,0},{0,0,4},{1,1,0},{0,0,0},{1,1,0},{0,0,0},{1,1,4},{2,1,1},{1,1,1},{0,0,2},{1,0,1},{2,0,2},{1,0,0},{2,0,0},{2,0,0},
    {0,0,0},{2,0,3},{1,0,3},{0,0,2},{1,1,0},{2,1,2},{4,1,0},{3,1,0},{4,1,2},{3,1,1},{1,1,1},{0,0,0},{1,0,2},{2,0,4},{1,0,0},{2,0,1},{0,0,0},{1,0,0},{2,0,0},{0,0,0},{1,0,2},{0,0,0},{1,0,0},{0,0,3},{1,1,4},{2,1,0},{1,1,0},{2,1,2},{1,1,2},{0,0,1},{1,0,0},{2,0,1},{1,0,0},{3,0,0},{1,0,0},{2,0,0},{2,0,0},
    {0,0,0},{2,0,4},{1,0,3},{0,0,0},{1,1,2},{3,1,1},{4,1,2},{5,1,0},{4,1,0},{3,1,1},{1,1,1},{0,0,0},{1,0,1},{2,0,0},{1,0,0},{2,0,1},{3,0,0},{2,0,2},{1,0,2},{2,0,0},{1,0,5},{0,0,4},{1,1,1},{2,1,4},{3,1,0},{2,1,1},{1,1,1},{0,0,0},{1,0,2},{2,0,1},{3,0,0},{2,0,0},{1,0,0},{3,0,0},
    {0,0,0},{2,0,1},{3,0,0},{2,0,1},{1,0,0},{2,0,0},{1,0,0},{0,0,2},{1,1,0},{2,1,0},{3,1,1},{5,1,4},{3,1,1},{1,1,1},{1,0,0},{0,0,0},{2,0,1},{1,0,0},{3,0,0},{2,0,2},{3,0,0},{2,0,1},{1,0,1},{2,0,1},{1,0,0},{2,0,0},{1,0,3},{0,0,0},{1,0,0},{1,1,0},{0,0,0},{1,1,0},{2,1,2},{3,1,0},{2,1,0},{3,1,2},{2,1,0},{1,1,1},{0,0,0},{1,0,2},{2,0,1},{3,0,0},{2,0,1},{3,0,0},
    {0,0,0},{3,0,1},{2,0,0},{3,0,0},{2,0,0},{1,0,0},{2,0,0},{1,0,1},{0,0,1},{2,1,1},{3,1,0},{4,1,0},{6,1,0},{5,1,0},{7,1,0},{6,1,0},{5,1,0},{3,1,1},{1,1,0},{0,0,1},{1,0,0},{2,0,3},{3,0,0},{2,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,1},{1,0,0},{2,0,5},{1,0,2},{0,0,2},{1,1,0},{2,1,0},{3,1,2},{4,1,0},{3,1,0},{4,1,0},{3,1,0},{2,1,1},{1,1,0},{1,0,0},{0,0,0},{2,0,0},{1,0,0},{2,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,0},{2,0,0},
    {0,0,0},{2,0,0},{3,0,1},{2,0,0},{3,0,0},{2,0,1},{1,0,1},{0,0,1},{2,1,1},{4,1,0},{6,1,0},{7,1,1},{8,1,0},{7,1,0},{5,1,0},{3,1,0},{2,1,0},{1,1,0},{0,0,0},{1,0,1},{2,0,1},{3,0,0},{2,0,0},{3,0,2},{2,0,0},{3,0,2},{1,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,4},{1,0,1},{0,0,1},{1,1,0},{2,1,0},{3,1,0},{4,1,0},{5,1,0},{4,1,1},{5,1,0},{4,1,0},{2,1,1},{1,1,0},{0,0,0},{1,0,0},{2,0,3},{3,0,1},{2,0,0},{3,0,0},
    {0,0,0},{3,0,2},{2,0,0},{3,0,0},{2,0,2},{1,0,0},{0,0,1},{2,1,0},{3,1,0},{5,1,0},{8,1,0},{9,1,0},{10,1,1},{7,1,0},{5,1,0},{3,1,0},{1,1,0},{0,0,0},{1,0,1},{2,0,0},{3,0,0},{2,0,0},{3,0,3},{4,0,0},{3,0,7},{2,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,0},{1,0,0},{2,0,0},{0,0,2},{2,1,0},{3,1,0},{4,1,0},{5,1,0},{7,1,0},{5,1,0},{6,1,0},{4,1,1},{2,1,0},{0,0,1},{1,0,1},{2,0,1},{3,0,2},{2,0,0},{3,0,0},
    {0,0,0},{3,0,5},{2,0,1},{1,0,0},{0,0,0},{1,1,0},{2,1,0},{5,1,0},{8,1,0},{12,1,0},{14,1,0},{13,1,0},{9,1,0},{6,1,0},{3,1,0},{1,1,0},{0,0,0},{2,0,0},{1,0,0},{3,0,0},{2,0,0},{3,0,0},{4,0,0},{3,0,1},{4,0,0},{3,0,0},{4,0,1},{3,0,0},{4,0,0},{3,0,2},{4,0,0},{3,0,1},{4,0,0},{3,0,0},{2,0,0},{3,0,0},{2,0,2},{0,0,1},{1,1,0},{2,1,0},{4,1,0},{5,1,0},{7,1,0},{8,1,0},{6,1,1},{5,1,0},{3,1,0},{1,1,1},{1,0,1},{2,0,0},{3,0,0},{2,0,0},{3,0,1},{2,0,0},{3,0,0},
};

#define PGM_UINT8(p) (*(uint8_t *)(p))


static int16_t get_lookup_value(uint8_t x, uint8_t y)
{
    // return value
    int16_t val = 0;

    // These are exception indicies
    if(x <= 6 || x >= 34)
    {
        // If the x index is in the upper range we need to translate it
        // to match the 10 indicies in the exceptions lookup table
        if(x >= 34) x -= 27;

        // Read the unsigned value from the array
        val = PGM_UINT8(&exceptions[x][y]);

        // Read the 8 bit compressed sign values
        uint8_t sign = PGM_UINT8(&exception_signs[x][y/8]);

        // Check the sign bit for this index
        if(sign & (0x80 >> y%8))
            val = -val;

        return val;
    }

    // Because the values were removed from the start of the
    // original array (0-6) to the exception array, all the indicies
    // in this main lookup need to be shifted left 7
    // EX: User enters 7 -> 7 is the first row in this array so it needs to be zero
    if(x >= 7) x -= 7;

    // If we are looking for the first value we can just use the
    // row start value from declination_keys
    if(y == 0) return PGM_UINT8(&declination_keys[0][x]);

    // Init vars
    row_value stval;
    int16_t offset = 0;

    // These will never exceed the second dimension length of 73
    uint8_t current_virtual_index = 0, r;

    // This could be the length of the array or less (1075 or less)
    uint16_t start_index = 0, i;

    // Init value to row start
    val = PGM_UINT8(&declination_keys[0][x]);

    // Find the first element in the 1D array
    // that corresponds with the target row
    for(i = 0; i < x; i++) {
        start_index += PGM_UINT8(&declination_keys[1][i]);
    }

    // Traverse the row until we find our value
    for(i = start_index; i < (start_index + PGM_UINT8(&declination_keys[1][x])) && current_virtual_index <= y; i++) {

        // Pull out the row_value struct
        memcpy(&stval, &declination_values[i], sizeof(row_value));

        // Pull the first offset and determine sign
        offset = stval.abs_offset;
        offset = (stval.offset_sign == 1) ? -offset : offset;

        // Add offset for each repeat
        // This will at least run once for zero repeat
        for(r = 0; r <= stval.repeats && current_virtual_index <= y; r++) {
            val += offset;
            current_virtual_index++;
        }
    }
    return val;
}


#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


float get_declination(float lat, float lon)
{
    int16_t decSW, decSE, decNW, decNE, lonmin, latmin;
    uint8_t latmin_index,lonmin_index;
    float decmin, decmax;

    // Constrain to valid inputs
    lat = constrain(lat, -90, 90);
    lon = constrain(lon, -180, 180);

    latmin = floor(lat/5)*5;
    lonmin = floor(lon/5)*5;

    latmin_index= (90+latmin)/5;
    lonmin_index= (180+lonmin)/5;

    decSW = get_lookup_value(latmin_index, lonmin_index);
    decSE = get_lookup_value(latmin_index, lonmin_index+1);
    decNE = get_lookup_value(latmin_index+1, lonmin_index+1);
    decNW = get_lookup_value(latmin_index+1, lonmin_index);

    /* approximate declination within the grid using bilinear interpolation */
    decmin = (lon - lonmin) / 5 * (decSE - decSW) + decSW;
    decmax = (lon - lonmin) / 5 * (decNE - decNW) + decNW;
    return (lat - latmin) / 5 * (decmax - decmin) + decmin;
}
