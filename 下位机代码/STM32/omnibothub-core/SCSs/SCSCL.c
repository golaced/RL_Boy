/*
 * SCSCL.c
 * 飞特SCSCL系列串行舵机应用层程序
 * 日期: 2019.7.31
 * 作者: 
 */

#include <string.h>
#include "INST.h"
#include "SCS.h"
#include "SCSCL.h"

static uint8_t Mem[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L+1];
static int Err = 0;

int getErr(void)
{
	return Err;
}

int WritePos(uint8_t IDss, uint16_t Position, uint16_t Time, uint16_t Speed)
{
	uint8_t bBuf[6];
	Host2SCS(bBuf+0, bBuf+1, Position);
	Host2SCS(bBuf+2, bBuf+3, Time);
	Host2SCS(bBuf+4, bBuf+5, Speed);
	
	return genWrite(IDss, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

int RegWritePos(uint8_t IDss, uint16_t Position, uint16_t Time, uint16_t Speed)
{
	uint8_t bBuf[6];
	Host2SCS(bBuf+0, bBuf+1, Position);
	Host2SCS(bBuf+2, bBuf+3, Time);
	Host2SCS(bBuf+4, bBuf+5, Speed);
	
	return regWrite(IDss, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

void RegWriteAction()
{
	regAction(0xfe);
}

void SyncWritePos(uint8_t IDss[], uint8_t IDN, uint16_t Position[], uint16_t Time[], uint16_t Speed[])
{
  uint8_t offbuf[32*6];
	uint8_t i;
  for(i = 0; i<IDN; i++){
		uint16_t T, V;
		if(Time){
			T = Time[i];
		}else{
			T = 0;
		}
		if(Speed){
			V = Speed[i];
		}else{
			V = 0;
		}
    Host2SCS(offbuf+i*6+0, offbuf+i*6+1, Position[i]);
    Host2SCS(offbuf+i*6+2, offbuf+i*6+3, T);
    Host2SCS(offbuf+i*6+4, offbuf+i*6+5, V);
  }
  syncWrite(IDss, IDN, SCSCL_GOAL_POSITION_L, offbuf, 6);
}

int PWMMode(uint8_t IDss)
{
	uint8_t bBuf[4];
	bBuf[0] = 0;
	bBuf[1] = 0;
	bBuf[2] = 0;
	bBuf[3] = 0;
	return genWrite(IDss, SCSCL_MIN_ANGLE_LIMIT_L, bBuf, 4);	
}

int WritePWM(uint8_t IDss, int16_t pwmOut)
{
	uint8_t bBuf[2];
	if(pwmOut<0){
		pwmOut = -pwmOut;
		pwmOut |= (1<<10);
	}
	Host2SCS(bBuf+0, bBuf+1, pwmOut);

	return genWrite(IDss, SCSCL_GOAL_TIME_L, bBuf, 2);
}
int EnableTorque(uint8_t IDss, uint8_t Enable)
{
	return writeByte(IDss, SCSCL_TORQUE_ENABLE, Enable);
}

int unLockEprom(uint8_t IDss)
{
	return writeByte(IDss, SCSCL_LOCK, 0);
}

int LockEprom(uint8_t IDss)
{
	return writeByte(IDss, SCSCL_LOCK, 1);
}

int FeedBack(int IDss)
{
	int nLen = Read(IDss, SCSCL_PRESENT_POSITION_L, Mem, sizeof(Mem));
	if(nLen!=sizeof(Mem)){
		Err = 1;
		return -1;
	}
	Err = 0;
	return nLen;
}
	
int ReadPos(int IDss)
{
	int Pos = -1;
	if(IDss==-1){
		Pos = Mem[SCSCL_PRESENT_POSITION_L-SCSCL_PRESENT_POSITION_L];
		Pos <<= 8;
		Pos |= Mem[SCSCL_PRESENT_POSITION_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Err = 0;
		Pos = readWord(IDss, SCSCL_PRESENT_POSITION_L);
		if(Pos==-1){
			Err = 1;
		}
	}
	return Pos;
}

int ReadSpeed(int IDss)
{
	int Speed = -1;
	if(IDss==-1){
		Speed = Mem[SCSCL_PRESENT_SPEED_L-SCSCL_PRESENT_POSITION_L];
		Speed <<= 8;
		Speed |= Mem[SCSCL_PRESENT_SPEED_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Err = 0;
		Speed = readWord(IDss, SCSCL_PRESENT_SPEED_L);
		if(Speed==-1){
			Err = 1;
			return -1;
		}
	}
	if(!Err && Speed&(1<<15)){
		Speed = -(Speed&~(1<<15));
	}	
	return Speed;
}

int ReadLoad(int IDss)
{
	int Load = -1;
	if(IDss==-1){
		Load = Mem[SCSCL_PRESENT_LOAD_L-SCSCL_PRESENT_POSITION_L];
		Load <<= 8;
		Load |= Mem[SCSCL_PRESENT_LOAD_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Err = 0;
		Load = readWord(IDss, SCSCL_PRESENT_LOAD_L);
		if(Load==-1){
			Err = 1;
		}
	}
	if(!Err && Load&(1<<10)){
		Load = -(Load&~(1<<10));
	}	
	return Load;
}

int ReadVoltage(int IDss)
{
	int Voltage = -1;
	if(IDss==-1){
		Voltage = Mem[SCSCL_PRESENT_VOLTAGE-SCSCL_PRESENT_POSITION_L];	
	}else{
		Err = 0;
		Voltage = readByte(IDss, SCSCL_PRESENT_VOLTAGE);
		if(Voltage==-1){
			Err = 1;
		}
	}
	return Voltage;
}

int ReadTemper(int IDss)
{
	int Temper = -1;
	if(IDss==-1){
		Temper = Mem[SCSCL_PRESENT_TEMPERATURE-SCSCL_PRESENT_POSITION_L];	
	}else{
		Err = 0;
		Temper = readByte(IDss, SCSCL_PRESENT_TEMPERATURE);
		if(Temper==-1){
			Err = 1;
		}
	}
	return Temper;
}

int ReadMove(int IDss)
{
	int Move = -1;
	if(IDss==-1){
		Move = Mem[SCSCL_MOVING-SCSCL_PRESENT_POSITION_L];	
	}else{
		Err = 0;
		Move = readByte(IDss, SCSCL_MOVING);
		if(Move==-1){
			Err = 1;
		}
	}
	return Move;
}

int ReadCurrent(int IDss)
{
	int Current = -1;
	if(IDss==-1){
		Current = Mem[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L];
		Current <<= 8;
		Current |= Mem[SCSCL_PRESENT_CURRENT_L-SCSCL_PRESENT_POSITION_L];
	}else{
		Err = 0;
		Current = readWord(IDss, SCSCL_PRESENT_CURRENT_L);
		if(Current==-1){
			Err = 1;
			return -1;
		}
	}
	if(!Err && Current&(1<<15)){
		Current = -(Current&~(1<<15));
	}	
	return Current;
}
