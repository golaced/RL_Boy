/*
 * SMS_STS.h
 * 飞特SMS/STS系列串行舵机应用层程序
 * 日期: 2022.1.6
 * 作者: 
 */

#ifndef _SMS_STS_H
#define _SMS_STS_H

#include <stdint.h>

#define	SMS_STS_1M 0
#define	SMS_STS_0_5M 1
#define	SMS_STS_250K 2
#define	SMS_STS_128K 3
#define	SMS_STS_115200 4
#define	SMS_STS_76800	5
#define	SMS_STS_57600	6
#define	SMS_STS_38400	7

//内存表定义
//-------EPROM(只读)--------
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

//-------EPROM(读写)--------
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD 26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OFS_L 31
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

//-------SRAM(读写)--------
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_LOCK 55

//-------SRAM(只读)--------
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70


extern int WritePosEx_sms(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC);//普通写位置指令
extern int RegWritePosEx_sms(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC);//异步写位置指令
extern void RegWriteAction_sms(void);//异步写位置执行
extern void SyncWritePosEx_sms(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]);//同步写位置指令
extern int WheelMode_sms(uint8_t ID);//恒速模式
extern int WriteSpe_sms(uint8_t ID, int16_t Speed, uint8_t ACC);//恒速模式控制指令
extern int EnableTorque_sms(uint8_t ID, uint8_t Enable);//扭矩控制指令
extern int unLockEprom_sms(uint8_t ID);//eprom解锁
extern int LockEprom_sms(uint8_t ID);//eprom加锁
extern int CalibrationOfs_sms(uint8_t ID);//中位校准
extern int FeedBack_sms(int ID);//反馈舵机信息
extern int ReadPos_sms(int ID);//读位置
extern int ReadSpeed_sms(int ID);//读速度
extern int ReadLoad_sms(int ID);//读输出扭力
extern int ReadVoltage_sms(int ID);//读电压
extern int ReadTemper_sms(int ID);//读温度
extern int ReadMove_sms(int ID);//读移动状态
extern int ReadCurrent_sms(int ID);//读电流
extern int getErr_sms(void);//返回通信状态,0无出错,1通信出错


#endif
