#if 0//lite tvm
#include "gait_math.h"
#include "eso.h"
#include "locomotion_header.h"
#include "include.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "can.h"
#include "sys_time.h"
#include <iostream>
#include "joint_map.h"
#include "ankle.h"
#include "iopack.h"
#include "spi_node.h"
Jnt::ankleClass ankle_lite_l(0);//左+右
Jnt::ankleClass ankle_lite_r(1);//左+右

#define OBS_NUM 47
#define ACT_NUM 12
#if RL_USE_TVM
    #include "tvm2.h"
    #include "tvm.h"
    #include <iomanip>

    Tvm::tvm2Class tvm2;
    vecNf(OBS_NUM) obs1;
    vecNf(OBS_NUM * 10) obs10;
    vecNf(ACT_NUM) action_2, actionOld_2, actionOut, actionTmp_2,actionTmp_3, actionFilt_2;

    Tvm::tvmClass tvm;
    vecNf(OBS_NUM * 10) obs_all;
    vecNf(ACT_NUM) action, actionOld, actionTmp, actionFilt;
    float smooth_rc = 0.03;
    float dead_zone_rc = 0.01;

    float cmd_x = 0.;
    float cmd_y = 0.;
    float cmd_rate = 0.;

    float cmd_x_temp = 0.;
    float cmd_y_temp = 0.;
    float cmd_rate_temp = 0.;

    float eu_ang_scale = 1;
    float omega_scale = 0.25;
    float pos_scale = 1.0;
    float vel_scale = 0.05;
    float lin_vel = 2.0;
    float ang_vel = 0.25;

    int id_list[ACT_NUM] = {0,1,2,3,4,5, 7,8,9,10,11,12};
    int rl_model_loaded=0;
    int first_loaded_model=0;
#endif

// 欧拉角转四元数 (roll, pitch, yaw) -> (qx, qy, qz, qw)
void eulerToQuaternion(const float euler_radians[3], float quat[4])
{
  float roll = euler_radians[0];  // roll
  float pitch = euler_radians[1]; // pitch
  float yaw = euler_radians[2];   // yaw

  float cy = cosf(yaw * 0.5f);
  float sy = sinf(yaw * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);

  quat[0] = sr * cp * cy - cr * sp * sy; // qx
  quat[1] = cr * sp * cy + sr * cp * sy; // qy
  quat[2] = cr * cp * sy - sr * sp * cy; // qz
  quat[3] = cr * cp * cy + sr * sp * sy; // qw
}

// 根据四元数计算重力方向向量 (body frame)
void getGravityOrientation(const float quat[4], float gravity[3])
{
  float qx = quat[0];
  float qy = quat[1];
  float qz = quat[2];
  float qw = quat[3];

  gravity[0] = 2.0f * (-qz * qx + qw * qy);       // x
  gravity[1] = -2.0f * (qz * qy + qw * qx);       // y
  gravity[2] = 1.0f - 2.0f * (qw * qw + qz * qz); // z
}

// 欧拉角转重力向量（统一使用 float）
void eulerToGravity(const float euler_radians[3], float gravity[3])
{
  float quat[4];
  eulerToQuaternion(euler_radians, quat);
  getGravityOrientation(quat, gravity);
}

void  RL_Modle_load_test(void)
{
     printf("!!!!!!!!!!!!!!!RL_Modle_load_test tvm-lite\n");
     tvm2.init("Model_Lite/Trot_2/policy_arm64_cpu.so", 45, 45*10, 12);//tort
}

void  Gait_RL_Active(char rst)
{
    int i = 0;
    //state estimateor
    printf("RL::Lite Activate!\n");
    vmc_all.param.robot_mode=M_RL;vmc_all.gait_mode=G_RL;
#if RUN_PI
    robotwb.exp_att.yaw =robotwb.now_att.yaw;
    robot_param_read();//reset param

    vmc_all.rl_gain=1.0;//doghome all gain

    for(int i=0;i<14;i++)
        leg_motor_all.stiff[i]=vmc_all.rl_gain;

//    vmc_all.rl_commond_rl_rst[0]=0.3;//tinker
//    vmc_all.net_run_dt=0.01;
#endif
    vmc_all.att_measure_bias[PITr]=config_gait["vmc_param"]["att_bias_pit"].as<float>();
    vmc_all.att_measure_bias[ROLr]=config_gait["vmc_param"]["att_bias_rol"].as<float>();

#if RL_USE_TVM
    if(rst!=vmc_all.rl_mode_used&&rst!=0){
        printf("RL:loaded start=%d\n",rst);
        if(!first_loaded_model){
            first_loaded_model=1;
            action.setZero();
            action.setZero();
            actionOld.setZero();
            actionTmp.setZero();

            action_2.setZero();
            action_2.setZero();
            actionOld_2.setZero();
            actionTmp_2.setZero();
            actionTmp_3.setZero();
            actionOut.setZero();
            obs1.setZero();
            obs10.setZero();
            obs_all.setZero();
        }

        int loaded=0;
        switch(rst){
            case 1:loaded=tvm2.init("Model_Lite/Trot_2/policy_arm64_cpu.so", OBS_NUM, OBS_NUM*10, ACT_NUM);vmc_all.rl_model_type=2;break;//tort
            case 2:loaded=tvm2.init("Model_Lite/Stand_2/policy_arm64_cpu.so", OBS_NUM, OBS_NUM*10, ACT_NUM);vmc_all.rl_model_type=2;break;//stand
        }

        if(loaded){
            rl_model_loaded=1;
            vmc_all.rl_mode_used=rst+vmc_all.rl_model_type*100;
            printf("RL:: Model policy=%d loaded good type=%d!\n",rst,vmc_all.rl_model_type);
        }
        else{
            vmc_all.rl_mode_used=0;
            printf("RL:: Model loaded fail!\n");
            ocu.cmd_robot_state = 2;
            vmc_all.param.robot_mode = M_STAND_RC;	vmc_all.gait_mode = STAND_RC;
            Gait_Stand_Active();//
            for(i=0;i<14;i++)
                leg_motor_all.q_set_servo[i]=move_joint_to_pos_all(leg_motor_all.q_set_servo[i],leg_motor_all.q_set_servo_init[i],90,0.005);//doghome
        }
    }else{
        printf("RL:: used estited Model policy loaded good!\n");
        vmc_all.rl_commond_rl_rst[0]=0;
        vmc_all.net_run_dt=config_gait["rl_gait"]["net_run_dt"].as<float>();
        rl_model_loaded=1;
    }
    vmc_all.rl_gait_time=0;
#endif
}

void  Gait_RL_Update_2(float dt)
{
    static float timer[10];

    timer[0]+=dt;
#if RL_USE_TVM
    if (timer[0] > vmc_all.net_run_dt1 && rl_model_loaded){
        //convert ankle fb
        vmc_all.rl_gait_time += 0.02;
//        vmc_all.rl_gait_duty = 0.6;

        float q_convert[2][2];
        float qd_convert[2][2];
        float temp;
        ankle_lite_l.setActMot(-leg_motor_all.q_now[id_list[4]] / 57.3,-leg_motor_all.q_now[id_list[5]] / 57.3,
                               -leg_motor_all.qd_now[id_list[4]] / 57.3,-leg_motor_all.qd_now[id_list[5]] / 57.3,
                             0,0);
        ankle_lite_r.setActMot(-leg_motor_all.q_now[id_list[10]] / 57.3,-leg_motor_all.q_now[id_list[11]] / 57.3,
                               -leg_motor_all.qd_now[id_list[10]] / 57.3,-leg_motor_all.qd_now[id_list[11]] / 57.3,
                             0,0);
        ankle_lite_l.getActAnk(	q_convert[0][0], q_convert[0][1],//rad
                                qd_convert[0][0], qd_convert[0][1],
                                temp,temp);
        ankle_lite_r.getActAnk(	q_convert[1][0], q_convert[1][1],
                                qd_convert[1][0], qd_convert[1][1],
                                temp,temp);

        timer[0]=0;
        float cmd_x_temp = 0;//
        float cmd_y_temp = 0;//
        //printf("%f %f %f\n",vmc_all.rl_commond_off[4],vmc_all.rl_commond_off[0],vmc_all.rl_commond_off[1]);
        cmd_x_temp=   LIMIT(vmc_all.tar_spd.x,-MAX_SPD_X*0.5,MAX_SPD_X)+vmc_all.rl_commond_rl_rst[0]+
                vmc_all.rl_commond_off[4]*vmc_all.rl_commond_off[0]*(vmc_all.gait_mode==G_RL);
        cmd_y_temp=   LIMIT(-vmc_all.tar_spd.y,-MAX_SPD_Y,MAX_SPD_Y)+
                vmc_all.rl_commond_off[4]*vmc_all.rl_commond_off[1]*(vmc_all.gait_mode==G_RL);

        float cmd_rate_temp = 0;
        if (fabs(vmc_all.tar_spd.z) > 1)
        {
            cmd_rate_temp = vmc_all.tar_spd.z/57.3*1.5;
            robotwb.exp_att.yaw = robotwb.now_att.yaw;
        }
        else {
            cmd_rate_temp = -dead(limitw(To_180_degrees(robotwb.exp_att.yaw - robotwb.now_att.yaw), -25, 25), 0.25)*1.4/57.3;
        }
        cmd_rate_temp=   LIMIT(cmd_rate_temp,-MAX_SPD_RAD/57.3,MAX_SPD_RAD/57.3);

        cmd_x = cmd_x * (1 - smooth_rc) + (std::fabs(cmd_x_temp) < dead_zone_rc ? 0.0 : cmd_x_temp) * smooth_rc;
        cmd_y = cmd_y * (1 - smooth_rc) + (std::fabs(cmd_y_temp) < dead_zone_rc ? 0.0 : cmd_y_temp) * smooth_rc;
        cmd_rate = cmd_rate * (1 - smooth_rc) + (std::fabs(cmd_rate_temp) < dead_zone_rc ? 0.0 : cmd_rate_temp) * smooth_rc;
        float omega[3];
        float eu_ang[3];
        float gvec[3];
#if 0//tinker inner IMU
          omega[0] =    vmc_all.att_rate[ROLr] / 57.3;
          omega[1] =   -vmc_all.att_rate[PITr] / 57.3;
          omega[2] =    vmc_all.att_rate[YAWr] / 57.3;
          eu_ang[0] =   vmc_all.att[ROLr] / 57.3;
          eu_ang[1] =  -vmc_all.att[PITr] / 57.3;
          eu_ang[2] =  -vmc_all.att[YAWr] / 57.3;
#else
        int en_imu=1;
            omega[0] =   -vmc_all.att_rate[ROLr] / 57.3* en_imu;
            omega[1] =    vmc_all.att_rate[PITr] / 57.3* en_imu;
            omega[2] =    vmc_all.att_rate[YAWr] / 57.3* en_imu;
            eu_ang[0] =  -vmc_all.att[ROLr] / 57.3* en_imu;
            eu_ang[1] =   (vmc_all.att[PITr]-2) / 57.3* en_imu;
            eu_ang[2] =  -vmc_all.att[YAWr] / 57.3* en_imu;
#endif
        eulerToGravity(eu_ang, gvec);
        float q_now[ACT_NUM];
        float qd_now[ACT_NUM];

        for (int i = 0; i < ACT_NUM; i++)
        {
            q_now[i]=leg_motor_all.q_now[id_list[i]];
            qd_now[i]=leg_motor_all.qd_now[id_list[i]];
        }
        q_now[4]=q_convert[0][0]*57.3; q_now[5]=q_convert[0][1]*57.3;//degreed
        q_now[10]=q_convert[1][0]*57.3; q_now[11]=q_convert[1][1]*57.3;
        qd_now[4]=qd_convert[0][0]*57.3; qd_now[5]=qd_convert[0][1]*57.3;
        qd_now[10]=qd_convert[1][0]*57.3; qd_now[11]=qd_convert[1][1]*57.3;

        float gait_signal[2] = {0};

        gait_signal[0] = sin(3.1415926 * 2 * vmc_all.rl_gait_time / vmc_all.rl_gait_duty);
        gait_signal[1] = cos(3.1415926 * 2 * vmc_all.rl_gait_time / vmc_all.rl_gait_duty);

        obs1 <<
            omega[0] * omega_scale,
            omega[1] * omega_scale,
            omega[2] * omega_scale,
            gvec[0],
            gvec[1],
            gvec[2],
            cmd_x * lin_vel,
            cmd_y * lin_vel,
            cmd_rate * ang_vel;

        for (int i = 0; i < ACT_NUM; i++)
        {
            obs1[9 + i] = (q_now[i] / 57.3 - vmc_all.default_action[id_list[i]]) * pos_scale;
            obs1[9+ACT_NUM + i] = (qd_now[i] / 57.3) * vel_scale;
            obs1[9+ACT_NUM+ACT_NUM + i] = actionTmp_3[i];
        }

        obs1[OBS_NUM-2]= gait_signal[0];
        obs1[OBS_NUM-1]= gait_signal[1];

        obs10.head<OBS_NUM * 9>() = obs10.tail<OBS_NUM * 9>();
        obs10.tail<OBS_NUM>() = obs1;

        for(int i=0;i<OBS_NUM;i++)
            nav_rx.temp_recordi[i]=obs1[i];
        tvm2.in1 = obs1;
        tvm2.in2 = obs10;
        float t1,t2;
        t1= Get_T_Trig();
        tvm2.run();
        t2= Get_T_Trig();
        for(int i=0;i<ACT_NUM;i++)
            nav_rx.temp_recordo[i]=tvm2.out[i];
        actionTmp_2 = tvm2.out;

#if 0
        //printf("eu_ang: %.2f, %.2f, %.2f, gvec: %.2f,%.2f,%.2f\n", eu_ang[0], eu_ang[1], eu_ang[2], gvec[0], gvec[1], gvec[2]);
        printf("Rat:%0.2f %0.2f %0.2f\n",obs1[0]*57.3/0.25,obs1[1]*57.3/0.25,obs1[2]*57.3/0.25);
        printf("Grv:%0.3f %0.3f %0.3f\n",obs1[3],obs1[4],obs1[5]);
        printf("L:%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n",leg_motor_all.q_now[id_list[0]],leg_motor_all.q_now[id_list[1]],leg_motor_all.q_now[id_list[2]]
                ,leg_motor_all.q_now[id_list[3]],leg_motor_all.q_now[id_list[4]],leg_motor_all.q_now[id_list[5]]);
        //printf("L_ankle:%.2f %.2f\n",q_convert[0][0]*57.3,q_convert[0][1]*57.3);
        printf("R:%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n",leg_motor_all.q_now[id_list[6]],leg_motor_all.q_now[id_list[7]],leg_motor_all.q_now[id_list[8]]
                ,leg_motor_all.q_now[id_list[9]],leg_motor_all.q_now[id_list[10]],leg_motor_all.q_now[id_list[11]]);
        //printf("r_ankle:%.2f %.2f\n",q_convert[1][0]*57.3,q_convert[1][1]*57.3);
        //printf("t1=%f t2=%f dt=%f\n",t1,t2,t2-t1);
#endif

    }

    float filt_omega=0.2;
    actionFilt_2 = (filt_omega) * actionTmp_2 + (1 - filt_omega) * actionFilt_2;

    for (int i = 0; i < ACT_NUM; i++)
    {
        actionFilt_2[i]=LIMIT(actionFilt_2[i], -5, 5);
    }
    actionFilt_2[2]=LIMIT(actionFilt_2[2], -0.6, 0.6);//yaw
    actionFilt_2[8]=LIMIT(actionFilt_2[8], -0.6, 0.6);
    actionTmp_2 = actionFilt_2;

#if 0//disable doghome
    for (int i = 0; i < ACT_NUM; i++)
         actionTmp_2[i]=0;
#elif 0
    for (int i = 0; i < ACT_NUM; i++)
         actionTmp_2[i]=0;

    actionTmp_2[3]=sin(vmc_all.rl_gait_time*3)*20/57.3;//pitch
    actionTmp_2[4]=sin(vmc_all.rl_gait_time*3)*20/57.3;
    actionTmp_2[9]=sin(vmc_all.rl_gait_time*3)*20/57.3;
    actionTmp_2[10]=sin(vmc_all.rl_gait_time*3)*20/57.3;
#endif
    float set_convert[2][2];
    float temp=0;
    for (int i = 0; i < ACT_NUM; i++)
    {
        actionTmp_3[i] = actionTmp_2[i] * vmc_all.action_scale + vmc_all.default_action[id_list[i]];//doghome
    }
    //convert ankle command

    set_convert[0][0]=LIMIT(actionTmp_2[4]* vmc_all.action_scale + vmc_all.default_action[id_list[4]],-0.8,0.8);//pitch
    set_convert[0][1]=LIMIT(actionTmp_2[5]* vmc_all.action_scale + vmc_all.default_action[id_list[5]],-0.4,0.4);
    set_convert[1][0]=LIMIT(actionTmp_2[10]* vmc_all.action_scale + vmc_all.default_action[id_list[10]],-0.8,0.8);//pitch
    set_convert[1][1]=LIMIT(actionTmp_2[11]* vmc_all.action_scale + vmc_all.default_action[id_list[11]],-0.4,0.4);

    ankle_lite_l.setTgtAnk(	-set_convert[0][0], -set_convert[0][1],
                            temp,temp,
                            temp,temp);
    ankle_lite_r.setTgtAnk(	-set_convert[1][0], -set_convert[1][1],
                            temp,temp,
                            temp,temp);//rad
    ankle_lite_l.getTgtMot(actionTmp_3[4],actionTmp_3[5],
                         temp,temp,
                         temp,temp);
    ankle_lite_r.getTgtMot(actionTmp_3[10],actionTmp_3[11],
                         temp,temp,
                         temp,temp);

    for (int i = 0; i < ACT_NUM; i++)
        leg_motor_all.q_set[id_list[i]] = actionTmp_3[i] * 57.3;
#endif
}

void  Gait_RL_Update_1(float dt)//2ms
{

}


#include "gait_math.h"
#include "eso.h"
#include "locomotion_header.h"
#include "include.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "can.h"
#include "sys_time.h"
#include <iostream>
#include "joint_map.h"
#include "ankle.h"
#include "iopack.h"
#include "spi_node.h"
Jnt::ankleClass ankle_lite_l(0);//左+右
Jnt::ankleClass ankle_lite_r(1);//左+右

#define OBS_NUM 44
#define ACT_NUM 10
#if RL_USE_TVM
    #include "tvm2.h"
    #include "tvm.h"
    #include <iomanip>

    Tvm::tvm2Class tvm2;
    vecNf(OBS_NUM) obs1;
    vecNf(OBS_NUM * 10) obs10;
    vecNf(ACT_NUM) action_2, actionOld_2, actionOut, actionTmp_2,actionTmp_3, actionFilt_2;

    Tvm::tvmClass tvm;
    vecNf(OBS_NUM * 10) obs_all;
    vecNf(ACT_NUM) action, actionOld, actionTmp, actionFilt;
    float smooth_rc = 0.03;
    float dead_zone_rc = 0.01;

    float cmd_x = 0.;
    float cmd_y = 0.;
    float cmd_rate = 0.;

    float cmd_x_temp = 0.;
    float cmd_y_temp = 0.;
    float cmd_rate_temp = 0.;

    float eu_ang_scale = 1;
    float omega_scale = 0.25;
    float pos_scale = 1.0;
    float vel_scale = 0.05;
    float lin_vel = 2.0;
    float ang_vel = 0.25;

    int id_list[ACT_NUM] = {0,1,2,3,4,  7,8,9,10,11};
    int rl_model_loaded=0;
    int first_loaded_model=0;
#endif

// 欧拉角转四元数 (roll, pitch, yaw) -> (qx, qy, qz, qw)
void eulerToQuaternion(const float euler_radians[3], float quat[4])
{
  float roll = euler_radians[0];  // roll
  float pitch = euler_radians[1]; // pitch
  float yaw = euler_radians[2];   // yaw

  float cy = cosf(yaw * 0.5f);
  float sy = sinf(yaw * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);

  quat[0] = sr * cp * cy - cr * sp * sy; // qx
  quat[1] = cr * sp * cy + sr * cp * sy; // qy
  quat[2] = cr * cp * sy - sr * sp * cy; // qz
  quat[3] = cr * cp * cy + sr * sp * sy; // qw
}

// 根据四元数计算重力方向向量 (body frame)
void getGravityOrientation(const float quat[4], float gravity[3])
{
  float qx = quat[0];
  float qy = quat[1];
  float qz = quat[2];
  float qw = quat[3];

  gravity[0] = 2.0f * (-qz * qx + qw * qy);       // x
  gravity[1] = -2.0f * (qz * qy + qw * qx);       // y
  gravity[2] = 1.0f - 2.0f * (qw * qw + qz * qz); // z
}

// 欧拉角转重力向量（统一使用 float）
void eulerToGravity(const float euler_radians[3], float gravity[3])
{
  float quat[4];
  eulerToQuaternion(euler_radians, quat);
  getGravityOrientation(quat, gravity);
}

void  RL_Modle_load_test(void)
{
     printf("!!!!!!!!!!!!!!!RL_Modle_load_test tvm-tinker\n");
     tvm2.init("Model_Lite/Trot_2/policy_arm64_cpu.so",  OBS_NUM, OBS_NUM*10, ACT_NUM);//tort
}

void  Gait_RL_Active(char rst)
{
    int i = 0;
    //state estimateor
    printf("RL::Lite Activate!\n");
    vmc_all.param.robot_mode=M_RL;vmc_all.gait_mode=G_RL;
#if RUN_PI
    robotwb.exp_att.yaw =robotwb.now_att.yaw;
    robot_param_read();//reset param

    vmc_all.rl_gain=1.0;//doghome all gain

    for(int i=0;i<14;i++)
        leg_motor_all.stiff[i]=vmc_all.rl_gain;

//    vmc_all.rl_commond_rl_rst[0]=0.3;//tinker
//    vmc_all.net_run_dt=0.01;
#endif
    vmc_all.att_measure_bias[PITr]=config_gait["vmc_param"]["att_bias_pit"].as<float>();
    vmc_all.att_measure_bias[ROLr]=config_gait["vmc_param"]["att_bias_rol"].as<float>();

#if RL_USE_TVM
    if(rst!=vmc_all.rl_mode_used&&rst!=0){
        printf("RL:loaded start=%d\n",rst);
        if(!first_loaded_model){
            first_loaded_model=1;
            action.setZero();
            action.setZero();
            actionOld.setZero();
            actionTmp.setZero();

            action_2.setZero();
            action_2.setZero();
            actionOld_2.setZero();
            actionTmp_2.setZero();
            actionTmp_3.setZero();
            actionOut.setZero();
            obs1.setZero();
            obs10.setZero();
            obs_all.setZero();
        }

        int loaded=0;
        switch(rst){
            case 1:loaded=tvm2.init("Model_Lite/Trot_2/policy_arm64_cpu.so", OBS_NUM, OBS_NUM*10, ACT_NUM);vmc_all.rl_model_type=2;break;//tort
            case 2:loaded=tvm2.init("Model_Lite/Stand_2/policy_arm64_cpu.so", OBS_NUM, OBS_NUM*10, ACT_NUM);vmc_all.rl_model_type=2;break;//stand
        }

        if(loaded){
            rl_model_loaded=1;
            vmc_all.rl_mode_used=rst+vmc_all.rl_model_type*100;
            printf("RL:: Model policy=%d loaded good type=%d!\n",rst,vmc_all.rl_model_type);
        }
        else{
            vmc_all.rl_mode_used=0;
            printf("RL:: Model loaded fail!\n");
            ocu.cmd_robot_state = 2;
            vmc_all.param.robot_mode = M_STAND_RC;	vmc_all.gait_mode = STAND_RC;
            Gait_Stand_Active();//
            for(i=0;i<14;i++)
                leg_motor_all.q_set_servo[i]=move_joint_to_pos_all(leg_motor_all.q_set_servo[i],leg_motor_all.q_set_servo_init[i],90,0.005);//doghome
        }
    }else{
        printf("RL:: used estited Model policy loaded good!\n");
        vmc_all.rl_commond_rl_rst[0]=0;
        vmc_all.net_run_dt=config_gait["rl_gait"]["net_run_dt"].as<float>();
        rl_model_loaded=1;
    }
    vmc_all.rl_gait_time=0;
#endif
}
#endif
