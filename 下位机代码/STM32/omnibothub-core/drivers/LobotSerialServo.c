/*******************************************************************************
* 文件名： LobotSerial.c
* 作者： 深圳乐幻索尔科技
* 日期：20170217
* LX串口舵机Demo
*******************************************************************************/
#include "LobotSerialServo.h"
#include <stdarg.h>
#include <string.h>
#include "usart_fc.h"

#define GET_LOW_BYTE(A) ((uint8_t)(A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 将高地八位合成为十六位
uint8_t LobotRxBuf[16];

void LobotSerialWrite(uint8_t *buf, uint8_t len)
{
  char i=0;
	for(i=0;i<len;i++)
		SendBuff1[leg_uart_cnt++]=*buf+i;
	
	SendBuff1[leg_uart_cnt++]=0x00;
}

uint8_t LobotCheckSum(uint8_t buf[])
{
  uint8_t i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (uint8_t)temp;
  return i;
}

void LobotSerialServoSetID(uint8_t oldID, uint8_t newID)
{
	uint8_t buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  LobotSerialWrite(buf, 7);
}

void LobotSerialServoMove(uint8_t id, int16_t position, uint16_t time)
{
  uint8_t buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
	position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  LobotSerialWrite(buf, 10);
}

void LobotSerialServoUnload(uint8_t id)
{
  uint8_t buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  LobotSerialWrite(buf, 7);
}

void LobotSerialServoLoad(uint8_t id)
{
  uint8_t buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  LobotSerialWrite(buf, 7);
}

int LobotSerialServoReadPosition(uint8_t id)
{
  int ret;
  uint8_t buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);
	
	LobotSerialWrite(buf, 6);
	
	//ret = LobotSerialMsgHandle();
  return ret;
}

int dj_pos_rx[12];
float dj_pos_rx_angle[12];
int LobotSerialMsgHandle(void)
{
	int count = 50000;
	int id;
	uint8_t cmd;
	int ret;
	
	if(!isRxCompleted())
	{
		return -2049;
	}
	
	if(LobotCheckSum(LobotRxBuf) != LobotRxBuf[LobotRxBuf[3]+2])
	{
		return -2049;
	}
	id  = LobotRxBuf[2];
	cmd = LobotRxBuf[4];
	switch(cmd)
	{
		case LOBOT_SERVO_POS_READ:
			ret = (int)BYTE_TO_HW(LobotRxBuf[6], LobotRxBuf[5]);
		  dj_pos_rx[id]=ret;
		  convert_pwm_t_angle(id);
			return ret;
		default:
			break;
	}
	return 0;
}

void convert_pwm_t_angle(char i)
{
 int tempi;
 float tempf;
	tempi=dj_pos_rx[i]+200;
	tempf=(float)tempi*360/1400.;
	dj_pos_rx_angle[i]=dj_pos_rx[i];
}
