#include "adrc.h"
#include "stdio.h"
_ADRC adrc_inner_rate[3];
_ADRC adrc_outter_att[3];
_ADRC adrc_temp[10];
void ADRC_Init(void) {
  int i=0;
  for(i=0;i<10;i++)
    ADRC_Reset(&adrc_temp[i],0,0);
  ADRC_Reset(&adrc_inner_rate[0], 0, 0);
  ADRC_Reset(&adrc_inner_rate[1], 0, 0);
  ADRC_Reset(&adrc_inner_rate[2], 0, 0);
  ADRC_Reset(&adrc_outter_att[0], 0, 0);
  ADRC_Reset(&adrc_outter_att[1], 0, 0);
  ADRC_Reset(&adrc_outter_att[2], 0, 0);
}

void ADRC_Init_N1_L_H(_ADRC *adrc, float Wo, float kp, float ki, float kd, float b0, float b1, float max_err, float max_out, float dt)//1阶系统 手动调节 线性ADRC(角速度)
{
  ADRC_Reset(adrc,0,0);
  adrc->sys_n=1;
  adrc->adrc_mode=0;//0->手动调节  1->带宽调节
  adrc->control_mode=0;//0->线性 1->非线性
  adrc->eso_mode=0;//0->线性 1->非线性

  adrc->r=300000;//TD
  adrc->h=dt;//采样时间
  adrc->N0=2;

  adrc->Wc=200;
  adrc->Wo=2.5*adrc->Wc;
  if(Wo>0)
    adrc->Wo=Wo;
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
if(adrc->sys_n==1){
  adrc->beta_01=adrc->Wo*3;
  adrc->beta_02=adrc->Wo*adrc->Wo;
}
else{
  adrc->beta_01=adrc->Wo*3;
  adrc->beta_02=adrc->Wo*adrc->Wo*3;
  //adrc->beta_03=adrc->Wo*adrc->Wo*adrc->Wo;
  adrc->beta_03=adrc->beta_02*4;
}
//控制器参数
//手动调节
  adrc->KP=kp;
  adrc->KI=ki;
  adrc->KD=kd;

  adrc->b0=b0;//观测模型增益
  adrc->b1=b1;//控制模型增益
 //固定参数 
  adrc->alpha1=0.5;
  adrc->alpha2=1.8;
  adrc->zeta=50;
  adrc->max_err=max_err;
  adrc->max_out=max_out;
}

//控制器初始化
void ADRC_Init_N2_L_H(_ADRC *adrc, float Wo, float kp, float ki, float kd, float b0, float b1, float max_err, float max_out, float dt)//2阶系统 手动调节 线性ADRC(角度)
{
  ADRC_Reset(adrc,0,0);
  adrc->sys_n=2;
  adrc->adrc_mode=0;//0->手动调节  1->带宽调节
  adrc->control_mode=0;//0->线性 1->非线性
  adrc->eso_mode=0;//0->线性 1->非线性

  adrc->r=300000;//TD
  adrc->h=dt;//采样时间
  adrc->N0=2;

  adrc->Wc=200;
  adrc->Wo=2.5*adrc->Wc;
  if(Wo>0)
    adrc->Wo=Wo;
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
if(adrc->sys_n==1){
  adrc->beta_01=adrc->Wo*3;
  adrc->beta_02=adrc->Wo*adrc->Wo;
}
else{
  adrc->beta_01=adrc->Wo*3;
  adrc->beta_02=adrc->Wo*adrc->Wo*3;
  adrc->beta_03=adrc->Wo*adrc->Wo*adrc->Wo;
  //adrc->beta_03=adrc->beta_02*2;
}
//控制器参数
//手动调节
  adrc->KP=kp;
  adrc->KI=ki;
  adrc->KD=kd;

  adrc->b0=b0;//观测模型增益
  adrc->b1=b1;//控制模型增益
 //固定参数 
  adrc->alpha1=0.5;
  adrc->alpha2=1.8;
  adrc->zeta=50;
  adrc->max_err=max_err;
  adrc->max_out=max_out;
}

void ADRC_Init_N2_L_W(_ADRC *adrc,float Wc, float Wo, float Damp, float b0, float b1, float max_err, float max_out, float dt)//2阶系统 带宽 线性ADRC(角度)
{
  ADRC_Reset(adrc,0,0);
  adrc->sys_n=2;
  adrc->adrc_mode=1;//0->手动调节  1->带宽调节
  adrc->control_mode=0;//0->线性 1->非线性
  adrc->eso_mode=0;//0->线性 1->非线性

  adrc->r=300000;//TD
  adrc->h=dt;//采样时间
  adrc->N0=2;

  adrc->Wc=200;
  if (Wc > 0)
	  adrc->Wc = Wc;
  adrc->Wo=2.5*adrc->Wc;
  if(Wo>0)
    adrc->Wo=Wo;
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
if(adrc->sys_n==1){
  adrc->beta_01=adrc->Wo*3;
  adrc->beta_02=adrc->Wo*adrc->Wo;
}
else{
  adrc->beta_01=adrc->Wo*3;
  adrc->beta_02=adrc->Wo*adrc->Wo*3;
  //adrc->beta_03=adrc->Wo*adrc->Wo*adrc->Wo;
  adrc->beta_03=adrc->beta_02*4;
}
//控制器参数
//               =1带宽调节
  adrc->damp=1;
  if(Damp>0)
    adrc->damp=Damp;
  adrc->KP=adrc->Wc*adrc->Wc;
  adrc->KI=0;
  adrc->KD=adrc->Wc*2*adrc->damp;

  adrc->b0=b0;//观测模型增益
  adrc->b1=b1;//控制模型增益
 //固定参数 
  adrc->alpha1=0.5;
  adrc->alpha2=1.8;
  adrc->zeta=50;
  adrc->max_err=max_err;
  adrc->max_out=max_out;
}

//----------------------------ADRC控制器-----------------------------
int Sign_ADRC(float Input)
{
    int output=0;
    if(Input>1E-6) output=1;
    else if(Input<-1E-6) output=-1;
    else output=0;
    return output;
}

float Fal_ADRC(float e,float alpha,float zeta)
{
    int s=0;
    float fal_output=0;
    s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
    fal_output=e*s/(powf(zeta,1-alpha))+powf(ABS_ADRC(e),alpha)*Sign_ADRC(e)*(1-s);
    return fal_output;
}

int Fsg_ADRC(float x,float d)
{
    int output=0;
    output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
    return output;
}

float Constrain_Float(float amt, float low, float high){
    if (low==0 && low==high)
        return amt;
    else
        return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

void TD_ADRC(_ADRC *adrc,float expect_ADRC)
{
    float d=0,a0=0,y=0,a1=0,a2=0,a=0;
    float x1_delta=0;//ADRC状态跟踪误差项
    x1_delta=adrc->x1-expect_ADRC;//用x1-v(k)替代x1得到离散更新公式
    adrc->h0=adrc->N0*adrc->h;//用h0替代h，解决最速跟踪微分器速度超调问题
    d=adrc->r*adrc->h0*adrc->h0;//d=rh^2;
    a0=adrc->h0*adrc->x2;//a0=h*x2
    y=x1_delta+a0;//y=x1+a0
    a1=sqrt(d*(d+8*ABS_ADRC(y)));//a1=sqrt(d*(d+8*ABS_ADRC(y))])
    a2=a0+Sign_ADRC(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
    a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
    adrc->fh=-adrc->r*(a/d)*Fsg_ADRC(a,d)
                    -adrc->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪量
    adrc->x1+=adrc->h*adrc->x2;//跟新最速跟踪状态量x1
    adrc->x2+=adrc->h*adrc->fh;//跟新最速跟踪状态量微分x2
}

void NESO_3(_ADRC *adrc)
{
  adrc->e=Constrain_Float(adrc->z1-adrc->y,-adrc->max_err,adrc->max_err);//状态误差
  adrc->fe=Fal_ADRC(adrc->e,0.5,adrc->h);//非线性函数，提取跟踪状态与当前状态误差
  adrc->fe1=Fal_ADRC(adrc->e,0.25,adrc->h);
  adrc->z1+=adrc->h*(adrc->z2-adrc->beta_01*adrc->e);
  if (adrc->est_use_measure){
    adrc->ed=adrc->z2-adrc->yd;//状态误差
    adrc->z2+=adrc->h*(adrc->z3-adrc->beta_02*adrc->fe-adrc->beta_03*adrc->ed+adrc->b0*adrc->u);
  }else
  adrc->z2+=adrc->h*(adrc->z3-adrc->beta_02*adrc->fe+adrc->b0*adrc->u); 
  adrc->z3+=adrc->h*(-adrc->beta_03*adrc->fe1);
}

void NESO_2(_ADRC *adrc)
{
  adrc->e=Constrain_Float(adrc->z1-adrc->y,-adrc->max_err,adrc->max_err);//状态误差
  adrc->fe=Fal_ADRC(adrc->e,0.5,adrc->h);//非线性函数，提取跟踪状态与当前状态误差
  adrc->z1+=adrc->h*(adrc->z2-adrc->beta_01*adrc->e+adrc->b0*adrc->u);
  adrc->z2+=adrc->h*(-adrc->beta_02*adrc->fe);
}

void LESO_3(_ADRC *adrc)
{
  adrc->e=Constrain_Float(adrc->z1-adrc->y,-adrc->max_err,adrc->max_err);//状态误差
  //printf("s1 z1=%f %f\n", adrc->z1, adrc->y);
  //printf("s2 z2=%f %f %f %f\n", adrc->z2, adrc->yd, adrc->b0, adrc->h);
  adrc->z1+=adrc->h*(adrc->z2-adrc->beta_01*adrc->e);
  if (adrc->est_use_measure){
    adrc->ed=adrc->z2-adrc->yd;//状态误差
    adrc->z2+=adrc->h*(adrc->z3-adrc->beta_02*adrc->e-adrc->beta_03*adrc->ed+adrc->b0*adrc->u);
  }
  else
    adrc->z2+=adrc->h*(adrc->z3-adrc->beta_02*adrc->e+adrc->b0*adrc->u);
  adrc->z3+=adrc->h*(-adrc->beta_03*adrc->e);
}

void LESO_2(_ADRC *adrc)
{
  adrc->e=Constrain_Float(adrc->z1-adrc->y,-adrc->max_err,adrc->max_err);//状态误差
  adrc->z1+=adrc->h*(adrc->z2-adrc->beta_01*adrc->e+adrc->b0*adrc->u);
  adrc->z2+=adrc->h*(-adrc->beta_02*adrc->e);
}
//---------------------------控制器------------------------------
void N_CONTROL(_ADRC *adrc)
{
    adrc->u0=adrc->KP*Fal_ADRC(adrc->e1,adrc->alpha1,adrc->zeta)
            +adrc->KI*adrc->e0
            +adrc->KD*Fal_ADRC(adrc->e2,adrc->alpha2,adrc->zeta);
    if(adrc->adrc_mode==1)//带宽整定
    {
        if (adrc->b1>0)
            adrc->u0=Constrain_Float(adrc->u0/adrc->b1,-adrc->max_out,adrc->max_out);
        else
            adrc->u0=0;   
    }    
    else
        adrc->u0=Constrain_Float(adrc->u0,-adrc->max_out,adrc->max_out);
}

void L_CONTROL(_ADRC *adrc)
{
    adrc->u0=adrc->KP*adrc->e1
            +adrc->KI*adrc->e0
            +adrc->KD*adrc->e2;
    if(adrc->adrc_mode==1)//带宽整定
    {
        if (adrc->b1>0)
            adrc->u0=Constrain_Float(adrc->u0/adrc->b1,-adrc->max_out,adrc->max_out);
        else
            adrc->u0=0;   
    }    
    else
        adrc->u0=Constrain_Float(adrc->u0,-adrc->max_out,adrc->max_out);
}

void ADRC_Control(_ADRC *adrc,float expect,float measure,float measure_d)
{

    TD_ADRC(adrc,expect);//TD
    adrc->y=measure;
    adrc->yd=measure_d;
    if(adrc->yd!=0)
        adrc->est_use_measure=1;
    /*    1、状态信号z1；    2、状态速度信号z2；    3、状态加速度信号z3 */
    
    if(adrc->eso_mode==1){//非线性
        switch(adrc->sys_n){
            case 1:NESO_2(adrc);break;     
            case 2:NESO_3(adrc);break;   
            default: NESO_3(adrc);break;        
        }
    }
    else{                //线性
        switch(adrc->sys_n){
            case 1:LESO_2(adrc);break;     
            case 2:LESO_3(adrc);break;   
            default: LESO_3(adrc);break;        
        }
    }
    
    /*************状态误差反馈率*************/
    if(adrc->sys_n==1){//1阶系统  微分直接阻尼
        adrc->e0+=adrc->e1*adrc->h;//积分
        adrc->e0=Constrain_Float(adrc->e0,-adrc->max_err*0.4,adrc->max_err*0.4);   
        adrc->e1=adrc->x1-adrc->z1;//偏差
        adrc->e2=        -adrc->z2;//微分
    }
    else               //2阶系统  微分TD期望
    {
        adrc->e0+=adrc->e1*adrc->h;//积分
        adrc->e0=Constrain_Float(adrc->e0,-adrc->max_err*0.4,adrc->max_err*0.4);   
        adrc->e1=adrc->x1-adrc->z1;//偏差
        adrc->e2=adrc->x2-adrc->z2;//微分
    }

    if(adrc->control_mode==0)//线性控制组合
        L_CONTROL(adrc);
    else
        N_CONTROL(adrc);
    
    /**********扰动补偿*******/
    if(adrc->sys_n==1){//1阶系统
        if(adrc->b1>0) 
            adrc->u0-=adrc->z2/adrc->b1;
    }
    else               //2阶系统
    {   if(adrc->b1>0) 
            adrc->u0-=adrc->z3/adrc->b1;
	//printf("dis %f\n", adrc->z3 / adrc->b1);
    }
    
    adrc->u=Constrain_Float(adrc->u0,-adrc->max_out,adrc->max_out);   
}

void ADRC_Reset(_ADRC *adrc,float r_in,float measure)
{
    adrc->x1=r_in;
    adrc->x2=0;
    adrc->z1=measure;
    adrc->z2=adrc->z3=0;
    adrc->e0=adrc->e1=adrc->e2=0;
    adrc->u=adrc->u0=0;
}

void ADRC_Reset_KI(_ADRC *adrc)
{
    adrc->e0=0;
}

void ADRC_Set_Gain(_ADRC *adrc,float kp,float ki,float kd)
{
    adrc->KP=kp;
    adrc->KI=ki;
    adrc->KD=kd;
}
