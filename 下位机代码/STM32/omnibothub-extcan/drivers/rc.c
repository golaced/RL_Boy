

#include "include.h"
#include "rc.h"
s8 CH_in_Mapping[CH_NUM] = {0,1,2,3,4,5,6,7};    //ͨ��ӳ��
u16 RX_CH[CH_NUM];
int RX_CH_FIX[4];

s16 CH[CH_NUM];

float CH_Old[CH_NUM];
float CH_filter[CH_NUM];
float CH_filter_Old[CH_NUM];
float CH_filter_D[CH_NUM];
u8 NS,CH_Error[CH_NUM];
u16 NS_cnt,CLR_CH_Error[CH_NUM];
 
s16 MAX_CH[CH_NUM]  = {1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 };	//ҡ�����
s16 MIN_CH[CH_NUM]  = {1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 };	//ҡ����С
char CH_DIR[CH_NUM] = {0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    };  //ҡ�˷���
