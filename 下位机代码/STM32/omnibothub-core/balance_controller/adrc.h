#include <math.h>
//ADRC和LESO控制库，由Doghome改进编制 V1.0 2020/5/6
#define ABS_ADRC(X)  (((X)>0)?(X):-(X))

typedef struct
{
/***********SYS**************/
int sys_n;//系统阶次
int adrc_mode;//0->手动调节  1->带宽调节
/************TD*************/
float x1;//跟踪微分状态量
float x2;//跟踪微分状态量微分项
float r;//跟踪因子
float h;//采样周期
int N0;//跟踪微分器解决速度超调h0=N*h
float h0;//N*h的滤波因子
float fh;//最速微分加速度跟踪量
/***********ESO************/
int eso_mode;//0->线性 1->非线性
int est_use_measure;//1->微分估计使用系统测量
float max_err;//误差限幅
float Wo;//观测器带宽
float z1;
float z2;
float z3;//根据控制对象输入与输出，提取的扰动信息
float e,ed;//系统状态误差
float y;//系统输出量
float yd;//输出微分测量
float fe;
float fe1;
float beta_01;
float beta_02;
float beta_03;
/**********CONTROL*********/
int control_mode;//0->线性 1->非线性
float max_out;//输出限幅
float Wc;//控制器带宽 10/ts调节时间  ts=3*T上升时间
float damp;//阻尼比
float e0;//误差积分项
float e1;//偏差
float e2;//量微分项
float u0;//系统输出
float u;//带扰动补偿后的输出
float KP;//u0=beta_1*e1+beta_2*e2+(beta_0*e0);
float KI;//
float KD;//
float alpha1;//u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
float alpha2;//0<alpha1<1<alpha2
float zeta;//线性段的区间长度

float b0;//扰动补偿 观测
float b1;//扰动补偿 控制
}_ADRC;

extern _ADRC adrc_inner_rate[3];
extern _ADRC adrc_outter_att[3];
extern _ADRC adrc_temp[10];
void ADRC_Init(void);
void ADRC_Reset(_ADRC *adrc,float r_in,float measure);
void ADRC_Reset_KI(_ADRC *adrc);
void ADRC_Set_Gain(_ADRC *adrc,float kp,float ki,float kd);
void ADRC_Init_N1_L_H(_ADRC *adrc, float Wo, float kp, float ki, float kd, float b0, float b1, float max_err, float max_out, float dt);//1阶系统 手动调节 线性ADRC(角速度)
void ADRC_Init_N2_L_H(_ADRC *adrc, float Wo, float kp, float ki, float kd, float b0, float b1, float max_err, float max_out, float dt);//2阶系统 手动调节 线性ADRC(角度)
void ADRC_Init_N2_L_W(_ADRC *adrc, float Wc, float Wo, float Damp, float b0, float b1, float max_err, float max_out, float dt);//2阶系统 带宽 线性ADRC(角度)   

//Wo=4*Wc  2:beta01=2*Wo beta02=Wo*Wo  3:beta01=3*Wo beta02=3*Wo*Wo beta03=Wo*Wo*Wo
void TD_ADRC(_ADRC *adrc,float expect_ADRC);
void NESO_3(_ADRC *adrc);
void NESO_2(_ADRC *adrc);
void LESO_3(_ADRC *adrc);
void LESO_2(_ADRC *adrc);
void ADRC_Control(_ADRC *adrc,float expect,float measure,float measure_d);
       
float Constrain_Float(float amt, float low, float high);

void N_CONTROL(_ADRC *adrc);
void L_CONTROL(_ADRC *adrc);
int Sign_ADRC(float Input);
float Fal_ADRC(float e,float alpha,float zeta);
int Fsg_ADRC(float x,float d);
