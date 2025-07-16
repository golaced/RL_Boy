/*
 * SCS.h
 * 飞特串行舵机通信协议程序
 * 日期: 2022.3.29
 * 作者: 
 */

#ifndef _SCS_H
#define _SCS_H

#include <stdint.h>

extern int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);//普通写指令
extern int regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);//异步写指令
extern int regAction(uint8_t ID);//异步写执行行
extern void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);//同步写指令
extern int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat);//写1个字节
extern int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat);//写2个字节
extern int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen);//读指令
extern int readByte(uint8_t ID, uint8_t MemAddr);//读1个字节
extern int readWord(uint8_t ID, uint8_t MemAddr);//读2个字节
extern int Ping(uint8_t ID);//Ping指令
extern int syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen);//同步读指令包发送
extern int syncReadPacketRx(uint8_t ID, uint8_t *nDat);//同步读返回包解码，成功返回内存字节数，失败返回0
extern int syncReadRxPacketToByte(void);//解码一个字节
extern int syncReadRxPacketToWrod(uint8_t negBit);//解码两个字节，negBit为方向为，negBit=0表示无方向
extern void syncReadBegin(uint8_t IDN, uint8_t rxLen, uint32_t TimeOut);//同步读开始
extern void syncReadEnd(void);//同步读结束
	
extern void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun);
extern void Host2SCS(uint8_t *DataL, uint8_t* DataH, int Data);//1个16位数拆分为2个8位数
extern int SCS2Host(uint8_t DataL, uint8_t DataH);//2个8位数组合为1个16位数
extern int Ack(uint8_t ID);//应答
extern int checkHead(void);//帧头检测

//硬件接口函数
extern int writeSCS(uint8_t *nDat, int nLen);
extern int writeByteSCS(unsigned char bDat);
extern int readSCS(uint8_t *nDat, int nLen);
extern int readSCSTimeOut(uint8_t *nDat, int nLen, uint32_t TimeOut);
extern void rFlushSCS(void);
extern void wFlushSCS(void);

#endif
