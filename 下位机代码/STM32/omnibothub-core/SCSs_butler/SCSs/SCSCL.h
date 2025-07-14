/*
 * SCSCL.h
 * 飞特SCSCL系列串行舵机应用层程序
 * 日期: 2019.7.31
 * 作者: 
 */

#ifndef _SCSCL_H
#define _SCSCL_H

#include <stdint.h>

//波特率定义
#define		SCSCL_1M			0
#define		SCSCL_0_5M		1
#define		SCSCL_250K		2
#define		SCSCL_128K		3
#define		SCSCL_115200	4
#define		SCSCL_76800		5
#define		SCSCL_57600		6
#define		SCSCL_38400		7

//内存表定义
//-------EPROM(只读)--------
#define SCSCL_MODEL_L 3
#define SCSCL_MODEL_H 4

//-------EPROM(读写)--------
#define SCSCL_ID 5
#define SCSCL_BAUD_RATE 6
#define SCSCL_MIN_ANGLE_LIMIT_L 9
#define SCSCL_MIN_ANGLE_LIMIT_H 10
#define SCSCL_MAX_ANGLE_LIMIT_L 11
#define SCSCL_MAX_ANGLE_LIMIT_H 12
#define SCSCL_CW_DEAD 26
#define SCSCL_CCW_DEAD 27

//-------SRAM(读写)--------
#define SCSCL_TORQUE_ENABLE 40
#define SCSCL_GOAL_POSITION_L 42
#define SCSCL_GOAL_POSITION_H 43
#define SCSCL_GOAL_TIME_L 44
#define SCSCL_GOAL_TIME_H 45
#define SCSCL_GOAL_SPEED_L 46
#define SCSCL_GOAL_SPEED_H 47
#define SCSCL_LOCK 48

//-------SRAM(只读)--------
#define SCSCL_PRESENT_POSITION_L 56
#define SCSCL_PRESENT_POSITION_H 57
#define SCSCL_PRESENT_SPEED_L 58
#define SCSCL_PRESENT_SPEED_H 59
#define SCSCL_PRESENT_LOAD_L 60
#define SCSCL_PRESENT_LOAD_H 61
#define SCSCL_PRESENT_VOLTAGE 62
#define SCSCL_PRESENT_TEMPERATURE 63
#define SCSCL_MOVING 66
#define SCSCL_PRESENT_CURRENT_L 69
#define SCSCL_PRESENT_CURRENT_H 70


extern int WritePos(uint8_t IDss, uint16_t Position, uint16_t Time, uint16_t Speed);//普通写位置指令
extern int RegWritePos(uint8_t IDss, uint16_t Position, uint16_t Time, uint16_t Speed);//异步写位置指令
extern void RegWriteAction(void);//异步写位置执行
extern void SyncWritePos(uint8_t IDss[], uint8_t IDN, uint16_t Position[], uint16_t Time[], uint16_t Speed[]);//同步写位置指令	
extern int PWMMode(uint8_t IDss);//PWM输出模式
extern int WritePWM(uint8_t IDss, int16_t pwmOut);//PWM输出模式指令
extern int EnableTorque(uint8_t IDss, uint8_t Enable);//扭矩控制指令
extern int unLockEprom(uint8_t IDss);//eprom解锁
extern int LockEprom(uint8_t IDss);//eprom加锁
extern int FeedBack(int IDss);//反馈舵机信息
extern int ReadPos(int IDss);//读位置
extern int ReadSpeed(int IDss);//读速度
extern int ReadLoad(int IDss);//读输出扭力
extern int ReadVoltage(int IDss);//读电压
extern int ReadTemper(int IDss);//读温度
extern int ReadMove(int IDss);//读移动状态
extern int ReadCurrent(int IDss);//读电流
extern int getErr(void);//返回通信状态,0无出错,1通信出错


#endif
