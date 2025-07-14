
#include "stm32f4xx.h"
/*
Copyright(C) 2018 Golaced Innovations Corporation. All rights reserved.
OLDX_VMC四足机器人库 designed by Golaced from 云逸创新

qq群:567423074
淘宝店铺：https://shop124436104.taobao.com/?spm=2013.1.1000126.2.iNj4nZ
手机店铺地址：http://shop124436104.m.taobao.com
vmc.lib或stm32F4.lib下封装了虚拟力模型下足式机器人步态算法基本的库
*/
void f_2Dof(float Alpha1, float Alpha2, const float Param[6], float *Xe, float *Ye);
void i_2Dof(float x, float z, const float Param[6], float *Alpha1, float *Alpha2);