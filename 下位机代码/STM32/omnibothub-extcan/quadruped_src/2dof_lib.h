
#include "stm32f4xx.h"
/*
Copyright(C) 2018 Golaced Innovations Corporation. All rights reserved.
OLDX_VMC��������˿� designed by Golaced from ���ݴ���

qqȺ:567423074
�Ա����̣�https://shop124436104.taobao.com/?spm=2013.1.1000126.2.iNj4nZ
�ֻ����̵�ַ��http://shop124436104.m.taobao.com
vmc.lib��stm32F4.lib�·�װ��������ģ������ʽ�����˲�̬�㷨�����Ŀ�
*/
void f_2Dof(float Alpha1, float Alpha2, const float Param[6], float *Xe, float *Ye);
void i_2Dof(float x, float z, const float Param[6], float *Alpha1, float *Alpha2);