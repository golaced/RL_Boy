#if 1//YH taitan onnx
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
Jnt::ankleClass ankle_lite_l(0);//左+右
Jnt::ankleClass ankle_lite_r(1);//左+右

#include "onnx.h"
#include <iomanip>

Onnx::onnxClass onnx[10];
Onnx::onnxClass *onnxPtr[10];
int frame_stack = 21;
int num_single_obs = 44;

vecNf(44) obs1_onnx, obs_now;
vecNf(21 * 44) obs_all_onnx;
vecNf(10) action_onnx, actionOld_onnx, actionTmp_onnx, actionFilt_onnx, actionFilt_out;

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

int id_list[10] = {0, 1, 2, 3, 4,    7, 8, 9, 10, 11};
int rl_model_loaded = 0;
int first_loaded_model = 0;


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

void RL_Modle_load_test(void)
{
    printf("RL_Modle_load_test YH Taitan\n");
    onnx[1].init("Model_YH/Trot_2/policy.onnx", "trot_onnx");
    onnx[2].init("Model_YH/Stand_2/policy.onnx", "stand_onnx");
    printf("RL_Modle_load_test pass| vmc_all.net_run_dt1=%f  vmc_all.rl_gait_duty=%f vmc_all.action_scale=%f\n",vmc_all.net_run_dt1, vmc_all.rl_gait_duty,vmc_all.action_scale);

}

void Gait_RL_Active(char rst)
{
    int i = 0;
    // state estimateor
    printf("RL::Activate!\n");
    vmc_all.param.robot_mode = M_RL;
    vmc_all.gait_mode = G_RL;
#if RUN_PI
    robotwb.exp_att.yaw = robotwb.now_att.yaw;
    robot_param_read(); // reset param

    vmc_all.rl_gain=1.1;//doghome

    for(int i=0;i<14;i++)
        leg_motor_all.stiff[i]=vmc_all.rl_gain;
    for(int i=0;i<14;i++){
        leg_motor_all.kp_servo[i]=5.0;
        leg_motor_all.kd_servo[i]=0.4;
        leg_motor_all.stiff_servo[i]=1.0;

        leg_motor_all.kp_servo[0]=6.0;
        leg_motor_all.kd_servo[0]=0.4;
        leg_motor_all.kp_servo[1]=6.0;
        leg_motor_all.kd_servo[1]=0.4;
        leg_motor_all.kp_servo[7]=6.0;
        leg_motor_all.kd_servo[7]=0.4;
        leg_motor_all.kp_servo[8]=6.0;
        leg_motor_all.kd_servo[8]=0.4;
    }
#endif
    leg_motor_all.arm_mode=0;

    vmc_all.att_measure_bias[PITr] = config_gait["vmc_param"]["att_bias_pit"].as<float>();
    vmc_all.att_measure_bias[ROLr] = config_gait["vmc_param"]["att_bias_rol"].as<float>();


    if (rst != vmc_all.rl_mode_used && rst != 0)
    {
        printf("RL:loaded start=%d\n", rst);
        if (!first_loaded_model)
        {
            first_loaded_model = 1;
            action_onnx.setZero();
            action_onnx.setZero();
            actionOld_onnx.setZero();
            actionTmp_onnx.setZero();
            actionFilt_out.setZero();
            obs1_onnx.setZero();
            obs_all_onnx.setZero();
        }

        int loaded = 0;
        switch (rst)
        {
            case 1:
                printf("1:Start Trot initialization!\n");
                loaded = 1;//onnx[1].init("Model_YH/Trot_2/policy.onnx", "trot_onnx");
                onnxPtr[1] = &onnx[1];
                printf("Initialization finished!");
                vmc_all.rl_model_type = 2;
                vmc_all.rl_gait_sel=1;
            break; // tort
            case 2:
                printf("2:Start Stand initialization!\n");
                loaded = 1;//onnx[2].init("Model_YH/Stand_2/policy.onnx", "stand_onnx");
                onnxPtr[2] = &onnx[2];
                printf("Initialization finished!");
                vmc_all.rl_model_type = 2;
                vmc_all.rl_gait_sel=2;
            break; // stand
        }

        if (loaded)
        {
            rl_model_loaded = 1;
            vmc_all.rl_mode_used = rst + vmc_all.rl_model_type * 100;
            printf("RL:: Model policy=%d loaded good type=%d!\n", rst, vmc_all.rl_model_type);
        }
        else
        {
            vmc_all.rl_mode_used = 0;
            printf("RL:: Model loaded fail!\n");
            ocu.cmd_robot_state = 2;
            vmc_all.param.robot_mode = M_STAND_RC;
            vmc_all.gait_mode = STAND_RC;
            Gait_Stand_Active(); //
        }
    }
    else
    {
        printf("RL:: used estited Model policy loaded good!\n");
        vmc_all.rl_commond_rl_rst[0] = 0;
        vmc_all.net_run_dt = config_gait["rl_gait"]["net_run_dt"].as<float>();
        rl_model_loaded = 1;
    }
    vmc_all.rl_gait_time = 0;

}

void Gait_RL_Update_2(float dt) // 2ms
{
    static float timer[10];
    static float q_set_servo[14];
    eu_ang_scale = 1;
    omega_scale = 1;
    pos_scale = 1;
    vel_scale = 0.05;
    lin_vel = 2;
    ang_vel = 0.25;
    float t1, t2;
    timer[0] += dt;
    static  int key_a_reg;
    static  int stop_flag=0;
#if 0
    if(key_a_reg==0&&ocu.key_a==1){
        if(stop_flag==1){
            stop_flag=0;
        }
        else
            stop_flag=1;
    }
    key_a_reg=ocu.key_a;
#endif
    vmc_all.rl_gait_time += dt;
    if (timer[0] > vmc_all.net_run_dt1 && rl_model_loaded)
    { //
        float cmd_x_temp = 0;
        float cmd_y_temp = 0;
        //vmc_all.rl_gait_time += timer[0];

        cmd_x_temp = LIMIT(vmc_all.tar_spd.x, -MAX_SPD_X * 0.5, MAX_SPD_X) + vmc_all.rl_commond_rl_rst[0] +
                     vmc_all.rl_commond_off[4] * vmc_all.rl_commond_off[0] * (vmc_all.gait_mode == G_RL);
        cmd_y_temp = LIMIT(-vmc_all.tar_spd.y, -MAX_SPD_Y, MAX_SPD_Y) +
                     vmc_all.rl_commond_off[4] * vmc_all.rl_commond_off[1] * (vmc_all.gait_mode == G_RL);
        //printf("cmd_x_temp=%f cmd_y_temp=%f\n",cmd_x_temp,cmd_y_temp);
        float cmd_rate_temp = 0;
        if (fabs(vmc_all.tar_spd.z) > 1)
        {
            cmd_rate_temp = vmc_all.tar_spd.z / 57.3 * 1.5;
            robotwb.exp_att.yaw = robotwb.now_att.yaw;
        }
        else
        {
            cmd_rate_temp = -dead(limitw(To_180_degrees(robotwb.exp_att.yaw - robotwb.now_att.yaw), -25, 25), 0.25) * 2 / 57.3;
        }
        //printf("cmd_rate_temp=%f\n",cmd_rate_temp);
        cmd_rate_temp = LIMIT(cmd_rate_temp, -MAX_SPD_RAD / 57.3, MAX_SPD_RAD / 57.3);

        cmd_x = cmd_x * (1 - smooth_rc) + (std::fabs(cmd_x_temp) < dead_zone_rc ? 0.0 : cmd_x_temp) * smooth_rc;
        cmd_y = cmd_y * (1 - smooth_rc) + (std::fabs(cmd_y_temp) < dead_zone_rc ? 0.0 : cmd_y_temp) * smooth_rc;
        cmd_rate = cmd_rate * (1 - smooth_rc) + (std::fabs(cmd_rate_temp) < dead_zone_rc ? 0.0 : cmd_rate_temp) * smooth_rc;
        if(vmc_all.rl_gait_sel==2)//stand
        {
            cmd_x=cmd_y=cmd_rate=0;
        }

        float gait_signal[2] = {0};

        float gait_rate=1;
//        gait_rate=1-LIMIT(dead(MAX(fabs(vmc_all.att[PITr]),fabs(vmc_all.att[ROLr]))*1.25,10)/45.0,0.0,1.0);
//        gait_rate=LIMIT(gait_rate,0.7,1);
//        printf("gait_rate=%f\n",gait_rate);

        gait_signal[0] = sin(3.1415926 * 2 * vmc_all.rl_gait_time*gait_rate / vmc_all.rl_gait_duty);
        gait_signal[1] = cos(3.1415926 * 2 * vmc_all.rl_gait_time*gait_rate / vmc_all.rl_gait_duty);

        t1 = Get_T_Trig();

        float q_now[10];
        float qd_now[10];
        for (int i = 0; i < 10; i++)
        {
            q_now[i]=leg_motor_all.q_now[id_list[i]];
            qd_now[i]=leg_motor_all.qd_now[id_list[i]];
        }

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
#elif 0//outter imu0
            omega[0] =   -vmc_all.att_rate[ROLr] / 57.3;
            omega[1] =    vmc_all.att_rate[PITr] / 57.3;
            omega[2] =    vmc_all.att_rate[YAWr] / 57.3;
            eu_ang[0] =  -vmc_all.att[ROLr] / 57.3;
            eu_ang[1] =   vmc_all.att[PITr] / 57.3;
            eu_ang[2] =  -vmc_all.att[YAWr] / 57.3*0;
#else//outter imu1
            omega[0] =    vmc_all.att_rate[PITr] / 57.3;
            omega[1] =    vmc_all.att_rate[ROLr] / 57.3;
            omega[2] =    vmc_all.att_rate[YAWr] / 57.3;
            eu_ang[0] =   (vmc_all.att[PITr]+vmc_all.att_measure_bias[PITr]) / 57.3;
            eu_ang[1] =   (vmc_all.att[ROLr]+vmc_all.att_measure_bias[ROLr]) / 57.3;
            eu_ang[2] =   -vmc_all.att[YAWr] / 57.3;
           // printf("eu_ang: %.2f, %.2f, %.2f, omega: %.2f,%.2f,%.2f, gvec: %.2f,%.2f,%.2f\n", eu_ang[0], eu_ang[1], eu_ang[2], omega[0], omega[1], omega[2],gvec[0], gvec[1], gvec[2]);
#endif
        eulerToGravity(eu_ang, gvec);
        //printf("eu_ang: %.2f, %.2f, %.2f, gvec: %.2f,%.2f,%.2f\n", eu_ang[0], eu_ang[1], eu_ang[2], gvec[0], gvec[1], gvec[2]);

        obs_now <<
            0,
            0,
            0,
            cmd_x * lin_vel ,
            cmd_y * lin_vel ,
            cmd_rate * ang_vel ,
            gait_signal[0]*!stop_flag,
            gait_signal[1]*!stop_flag,
            omega[0] * ang_vel,
            omega[1] * ang_vel,
            omega[2] * ang_vel,
            gvec[0],
            gvec[1],
            gvec[2],
            (q_now[0] / 57.3 - vmc_all.default_action[id_list[0]]) * pos_scale,
            (q_now[1] / 57.3 - vmc_all.default_action[id_list[1]]) * pos_scale,
            (q_now[2] / 57.3 - vmc_all.default_action[id_list[2]]) * pos_scale,
            (q_now[3] / 57.3 - vmc_all.default_action[id_list[3]]) * pos_scale,
            (q_now[4] / 57.3 - vmc_all.default_action[id_list[4]]) * pos_scale,
            (q_now[5] / 57.3 - vmc_all.default_action[id_list[5]]) * pos_scale,
            (q_now[6] / 57.3 - vmc_all.default_action[id_list[6]]) * pos_scale,
            (q_now[7] / 57.3 - vmc_all.default_action[id_list[7]]) * pos_scale,
            (q_now[8] / 57.3 - vmc_all.default_action[id_list[8]]) * pos_scale,
            (q_now[9] / 57.3 - vmc_all.default_action[id_list[9]]) * pos_scale,
            (qd_now[0] / 57.3) * vel_scale,
            (qd_now[1] / 57.3) * vel_scale,
            (qd_now[2] / 57.3) * vel_scale,
            (qd_now[3] / 57.3) * vel_scale,
            (qd_now[4] / 57.3) * vel_scale,
            (qd_now[5] / 57.3) * vel_scale,
            (qd_now[6] / 57.3) * vel_scale,
            (qd_now[7] / 57.3) * vel_scale,
            (qd_now[8] / 57.3) * vel_scale,
            (qd_now[9] / 57.3) * vel_scale,
            action_onnx[0],
            action_onnx[1],
            action_onnx[2],
            action_onnx[3],
            action_onnx[4],
            action_onnx[5],
            action_onnx[6],
            action_onnx[7],
            action_onnx[8],
            action_onnx[9];

        obs_all_onnx.head(num_single_obs * (frame_stack - 1)) = obs_all_onnx.tail(num_single_obs * (frame_stack - 1));
        obs_all_onnx.tail(num_single_obs) = obs_now;
        //printf("vmc_all.rl_gait_sel=%d\n",vmc_all.rl_gait_sel);
        onnxPtr[vmc_all.rl_gait_sel]->in = obs_all_onnx;
        onnxPtr[vmc_all.rl_gait_sel]->run();
        t2 = Get_T_Trig();
        //printf("o-t1=%f t2=%f dt=%f\n", t1, t2, t2 - t1);
        timer[0] = 0;
        action_onnx = onnxPtr[vmc_all.rl_gait_sel]->out;
        actionOld_onnx = action_onnx;

        for (int i = 0; i < 10; i++)
        {
            action_onnx[i] = LIMIT(action_onnx[i], -4, 4);
        }
    }
#if 0
        printf("vmc_all.rl_gait_time=%f timer[0]=%f\n",vmc_all.rl_gait_time,timer[0]);
        printf("L:%0.1f %0.1f %0.1f %0.1f %0.1f %0.1f\n",leg_motor_all.q_now[id_list[0]],leg_motor_all.q_now[id_list[1]],leg_motor_all.q_now[id_list[2]]
                ,leg_motor_all.q_now[id_list[3]],leg_motor_all.q_now[id_list[4]],leg_motor_all.q_now[id_list[5]]);
        //printf("L_ankle:%.2f %.2f\n",q_convert[0][0]*57.3,q_convert[0][1]*57.3);
        printf("R:%0.1f %0.1f %0.1f %0.1f %0.1f %0.1f\n",leg_motor_all.q_now[id_list[6]],leg_motor_all.q_now[id_list[7]],leg_motor_all.q_now[id_list[8]]
                ,leg_motor_all.q_now[id_list[9]],leg_motor_all.q_now[id_list[10]],leg_motor_all.q_now[id_list[11]]);
        //printf("r_ankle:%.2f %.2f\n",q_convert[1][0]*57.3,q_convert[1][1]*57.3);b
#endif
    float filt_omega=vmc_all.rl_flt;
    actionFilt_onnx = (filt_omega) * action_onnx + (1 - filt_omega) * actionFilt_onnx;

    actionTmp_onnx = actionFilt_onnx;

#if 0//disable!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    for (int i = 0; i < 10; i++)
         actionTmp_onnx[i]=0;
#endif
    for (int i = 0; i < 10; i++)
    {
        actionFilt_out[i] = 1 * actionTmp_onnx[i] * vmc_all.action_scale + vmc_all.default_action[id_list[i]];
    }

    for (int i = 0; i < 10; i++)
        leg_motor_all.q_set[id_list[i]] = actionFilt_out[i] * 57.3;



   //arm
// init_q00: 30
// init_q01: -80
// init_q02: 0
// init_q03: -110
// init_q04: 0
// init_q05: 0
// init_q06: 0
// init_q10: 30
// init_q11: 80
// init_q12: 0
// init_q13: 110
    float flt_rate=dead(LIMIT(fabs(vmc_all.tar_spd.x)/MAX_SPD_X*1.5,0,1),0.2);


   if(ocu.key_ll)
       leg_motor_all.arm_mode=10;
   if(ocu.key_rr&&ocu.vr_connect&&ocu.vr_enable)
       leg_motor_all.arm_mode=20;//vr
   static float flt_yaw_rate=0;
   switch(leg_motor_all.arm_mode){
    case 0://swing arm
       timer[5]=0;
       //Left
       q_set_servo[0]=20- sin(3.1415926 * 2 * vmc_all.rl_gait_time / vmc_all.rl_gait_duty)*10*flt_rate;//
       q_set_servo[1]=-85;
       q_set_servo[2]=0;
       q_set_servo[3]=-50;
       //right
       q_set_servo[7]=20+ sin(3.1415926 * 2 * vmc_all.rl_gait_time / vmc_all.rl_gait_duty)*10*flt_rate;//
       q_set_servo[8]=80;
       q_set_servo[9]=0;
       q_set_servo[10]=50;
       //waist
       DigitalLPF(vmc_all.att_rate[YAWr] / 57.3,&flt_yaw_rate,0.25,dt);
       leg_motor_all.q_set[5]=flt_yaw_rate*8;
       leg_motor_all.q_set[5]=LIMIT(leg_motor_all.q_set[5],-15,15);
       //head
       leg_motor_all.q_set[6]=flt_yaw_rate*30;
       leg_motor_all.q_set[6]=LIMIT(leg_motor_all.q_set[6],-35,35);
     break;
     case 10://bao1
       timer[5]+=dt;
       //Left
       q_set_servo[0]=-30;//
       q_set_servo[1]=-80;
       q_set_servo[2]=0;
       q_set_servo[3]=-70;
       //right
       q_set_servo[7]=-30;//
       q_set_servo[8]=80;
       q_set_servo[9]=0;
       q_set_servo[10]=70;
      //waist
      leg_motor_all.q_set[5]=0;
      //head
      leg_motor_all.q_set[6]=0;

      if(timer[5]>1.0){
          timer[5]=0;
          leg_motor_all.arm_mode++;
      }
     break;
      case 11://bao2
         timer[5]+=dt;
        //Left
        q_set_servo[0]=-30;//
        q_set_servo[1]=-90;
        q_set_servo[2]=32;
        q_set_servo[3]=-70;
        //right
        q_set_servo[7]=-30;//
        q_set_servo[8]=90;
        q_set_servo[9]=32;
        q_set_servo[10]=70;
        //waist
        leg_motor_all.q_set[5]=0;
        //head
        leg_motor_all.q_set[6]=0;

        if(timer[5]>1.5){
            timer[5]=0;
            leg_motor_all.arm_mode++;
        }
       break;

       case 20://vr
         timer[5]+=dt;
         //Left
         q_set_servo[0]=ocu.vr_l_arm[0];//20- sin(3.1415926 * 2 * vmc_all.rl_gait_time / vmc_all.rl_gait_duty)*10*flt_rate;//
         q_set_servo[1]=-ocu.vr_l_arm[1];//-85;
         q_set_servo[2]=LIMIT(To_180_degreesw(ocu.vr_l_arm[2]-90),-40,40);//0;
         q_set_servo[3]=ocu.vr_l_arm[3];//-50;
         //right
         q_set_servo[7]=-ocu.vr_r_arm[0];//20+ sin(3.1415926 * 2 * vmc_all.rl_gait_time / vmc_all.rl_gait_duty)*10*flt_rate;//
         q_set_servo[8]=-ocu.vr_r_arm[1];//80;
         q_set_servo[9]=LIMIT(-To_180_degreesw(ocu.vr_r_arm[2]+90),-40,40);//0;
         q_set_servo[10]=ocu.vr_r_arm[3];//50;
         //waist
         leg_motor_all.q_set[5]=ocu.vr_body[2];
         //head
         leg_motor_all.q_set[6]=0;

         leg_motor_all.q_set_servo[12]=ocu.vr_l_arm[7];//-25;
         leg_motor_all.q_set_servo[13]=-ocu.vr_r_arm[7];//25;
         if(ocu.vr_connect==0||ocu.vr_enable==0){
             printf("Loss VR:%d %d\n",ocu.vr_connect,ocu.vr_enable);
             leg_motor_all.arm_mode=0;
          }
        break;
   }

   for(int i=0;i<14;i++)
       leg_motor_all.q_set_servo[i]=q_set_servo[i];//doghome

}

void Gait_RL_Update_1(float dt)
{

}
#endif
