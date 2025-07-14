#if 0//tinker tvm
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
#if RL_USE_TVM
    #include "tvm2.h"
    #include "tvm.h"
    #include <iomanip>

    Tvm::tvm2Class tvm2;
    vecNf(39) obs1;
    vecNf(39 * 10) obs10;
    vecNf(10) action_2, actionOld_2, actionTmp_2, actionFilt_2;

    Tvm::tvmClass tvm;
    vecNf(41*15) obs_all;
    vecNf(10) action, actionOld, actionTmp, actionFilt;
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

    int id_list[10] = {0,1,2,3,4, 7,8,9,10,11};
    int rl_model_loaded=0;
    int first_loaded_model=0;
#endif
void  RL_Modle_load_test(void)
{
     printf("RL_Modle_load_test\n");
     tvm2.init("Model/Trot_2/policy_arm64_cpu.so", 39, 390, 10);//tort
}

void  Gait_RL_Active(char rst)
{
    int i = 0;
    //state estimateor
    printf("RL::Activate!\n");
    vmc_all.param.robot_mode=M_RL;vmc_all.gait_mode=G_RL;
#if RUN_PI
    robotwb.exp_att.yaw =robotwb.now_att.yaw;
    robot_param_read();//reset param

    vmc_all.rl_gain=1;//doghome
    leg_motor_all.stiff[0]=config_gait["imp_param"]["stiff_q00"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[1]=config_gait["imp_param"]["stiff_q01"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[2]=config_gait["imp_param"]["stiff_q02"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[3]=config_gait["imp_param"]["stiff_q03"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[4]=config_gait["imp_param"]["stiff_q04"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[5]=config_gait["imp_param"]["stiff_q05"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[6]=config_gait["imp_param"]["stiff_q06"].as<float>()*vmc_all.rl_gain;

    leg_motor_all.stiff[7]=config_gait["imp_param"]["stiff_q10"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[8]=config_gait["imp_param"]["stiff_q11"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[9]=config_gait["imp_param"]["stiff_q12"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[10]=config_gait["imp_param"]["stiff_q13"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[11]=config_gait["imp_param"]["stiff_q14"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[12]=config_gait["imp_param"]["stiff_q15"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[13]=config_gait["imp_param"]["stiff_q16"].as<float>()*vmc_all.rl_gain;

    vmc_all.rl_commond_rl_rst[0]=0.3;//tinker
    vmc_all.net_run_dt=0.02;
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
            obs1.setZero();
            obs10.setZero();
            obs_all.setZero();
        }
        //vmc_all.net_run_dt=0.016;
        int loaded=0;
        switch(rst){
            case 1:loaded=tvm2.init("Model/Trot_2/policy_arm64_cpu.so", 39, 390, 10);vmc_all.rl_model_type=2;break;//tort
            case 2:loaded=tvm2.init("Model/Stand_2/policy_arm64_cpu.so", 39, 390, 10);vmc_all.rl_model_type=2;break;//stand
            case 3:loaded=tvm2.init("Model/Dance1_2/policy_arm64_cpu.so", 39, 390, 10);vmc_all.rl_model_type=2;break;
            case 4:loaded=tvm2.init("Model/Dance2_2/policy_arm64_cpu.so", 39, 390, 10);vmc_all.rl_model_type=2;break;
            case 5:loaded=tvm2.init("Model/Dance3_2/policy_arm64_cpu.so", 39, 390, 10);vmc_all.rl_model_type=2;break;
            case 6:loaded=tvm2.init("Model/Jump_2/policy_arm64_cpu.so", 39, 390, 10);vmc_all.rl_model_type=2;break;
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
    if(timer[0]>vmc_all.net_run_dt&&rl_model_loaded){//50Hz to elav the net TVM
        timer[0]=0;
        float cmd_x_temp = 0;//LIMIT(vmc_all.tar_spd.x * 1.5, -0.15, 1.0) + vmc_all.rl_commond_off[4] * vmc_all.rl_commond_off[0];
        float cmd_y_temp = 0;//LIMIT(vmc_all.tar_spd.y * 1.5, -0.35, 0.35) + vmc_all.rl_commond_off[4] * vmc_all.rl_commond_off[1];
        //printf("%f %f %f\n",vmc_all.rl_commond_off[4],vmc_all.rl_commond_off[0],vmc_all.rl_commond_off[1]);
        cmd_x_temp=   LIMIT(vmc_all.tar_spd.x,-MAX_SPD_X*0.5,MAX_SPD_X)+vmc_all.rl_commond_rl_rst[0]+
                vmc_all.rl_commond_off[4]*vmc_all.rl_commond_off[0]*(vmc_all.gait_mode==G_RL);
        cmd_y_temp=   LIMIT(-vmc_all.tar_spd.y,-MAX_SPD_Y,MAX_SPD_Y)+
                vmc_all.rl_commond_off[4]*vmc_all.rl_commond_off[1]*(vmc_all.gait_mode==G_RL);
        //printf("MAX_SPD_X=%f\n",MAX_SPD_X);
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
#if 0//tinker inner imu
        obs1 <<  vmc_all.att_rate[ROLr] / 57.3 * omega_scale,
                -vmc_all.att_rate[PITr] / 57.3 * omega_scale,
                 vmc_all.att_rate[YAWr] / 57.3 * omega_scale,
                 vmc_all.att[ROLr] / 57.3 * eu_ang_scale,
                -vmc_all.att[PITr] / 57.3 * eu_ang_scale,
                -vmc_all.att[YAWr] / 57.3 * eu_ang_scale,
                 cmd_x * lin_vel,
                 cmd_y * lin_vel,
                 cmd_rate * ang_vel;
#else//tinker new outer imu
        obs1 <<  -vmc_all.att_rate[ROLr] / 57.3 * omega_scale,
                 vmc_all.att_rate[PITr] / 57.3 * omega_scale,
                 vmc_all.att_rate[YAWr] / 57.3 * omega_scale,
                 -vmc_all.att[ROLr] / 57.3 * eu_ang_scale,
                 vmc_all.att[PITr] / 57.3 * eu_ang_scale,
                -vmc_all.att[YAWr] / 57.3 * eu_ang_scale,
                 cmd_x * lin_vel,
                 cmd_y * lin_vel,
                 cmd_rate * ang_vel;

#endif
        //cout<<"att:"<<endl<<dt<<endl;
        //printf("net_input:%f %f %f\n",cmd_x,cmd_y,cmd_rate);
        for (int i = 0; i < 10; i++)
        {
            obs1[9 + i] = (leg_motor_all.q_now[id_list[i]] / 57.3 - vmc_all.default_action[id_list[i]]) * pos_scale;
            obs1[19 + i] = (leg_motor_all.qd_now[id_list[i]] / 57.3) * vel_scale;
            obs1[29 + i] = action_2[i];
        }

        obs10.head<39 * 9>() = obs10.tail<39 * 9>();
        obs10.tail<39>() = obs1;

        tvm2.in1 = obs1;
        tvm2.in2 = obs10;
        float t1,t2;
        t1= Get_T_Trig();
        tvm2.run();
        t2= Get_T_Trig();
        //printf("t1=%f t2=%f dt=%f\n",t1,t2,t2-t1);
    }
    action_2 = actionOld_2 * 0.2 + tvm2.out * 0.8;
    //action_2 = tvm2.out ;
    actionOld_2 = tvm2.out;

    for (int i = 0; i < 10; i++)
    {
        action_2[i]=LIMIT(action_2[i], -5, 5);
    }
    actionTmp_2 = action_2;
#if 0
    printf("********************RL ENTERED***********************");
    printf("rl-action:%.3f %.3f %.3f %.3f %.3f| %.3f %.3f %.3f %.3f %.3f\n",
           actionTmp[0],actionTmp[1],actionTmp[2],actionTmp[3],actionTmp[4],
           actionTmp[5],actionTmp[6],actionTmp[7],actionTmp[8],actionTmp[9]);
#endif
    for (int i = 0; i < 10; i++)
    {
        float temp = actionTmp_2[i] * vmc_all.action_scale + vmc_all.default_action[id_list[i]];
        leg_motor_all.q_set[id_list[i]] = temp * 57.3;
    }
#endif
}

void  Gait_RL_Update_1(float dt)//2ms
{
    static float timer[10];

    timer[0]+=dt;
    vmc_all.rl_gait_time+=dt;
#if RL_USE_TVM
    if(timer[0]>vmc_all.net_run_dt1&&rl_model_loaded){//50Hz to elav the net TVM
        float cmd_x_temp = 0;
        float cmd_y_temp = 0;

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

        eu_ang_scale = 1;
        omega_scale = 1;
        pos_scale = 1;
        vel_scale = 0.05;
        lin_vel = 2;
        ang_vel = 1;
        int frame_stack = 15;
        int num_single_obs = 41;
        float gait_signal[2]={0};

        gait_signal[0]= sin(3.1415926*2*vmc_all.rl_gait_time/vmc_all.rl_gait_duty);
        gait_signal[1]= cos(3.1415926*2*vmc_all.rl_gait_time/vmc_all.rl_gait_duty);
        vecNf(41) obs_now;
#if 0
        obs_now[0] = gait_signal[0];
        obs_now[1] = gait_signal[1];
        obs_now[2] = cmd_x * lin_vel;
        obs_now[3] = cmd_y * lin_vel;
        obs_now[4] = cmd_rate * ang_vel;

        for (int j = 0; j < 10; ++j) {
            obs_now[5 + j] =  (leg_motor_all.q_now[id_list[j]] / 57.3 - vmc_all.default_action[id_list[j]]) * pos_scale;
        }
        for (int j = 0; j < 10; ++j) {
            obs_now[15 + j] = (leg_motor_all.qd_now[id_list[j]] / 57.3) * vel_scale;
        }
        for (int j = 0; j < 10; ++j) {
            obs_now[25 + j] = action[j];
        }
        float omega[3];
        float eu_ang[3];
        omega[0]=-vmc_all.att_rate[ROLr] / 57.3;
        omega[1]=vmc_all.att_rate[PITr] / 57.3;
        omega[2]=vmc_all.att_rate[YAWr] / 57.3;
        eu_ang[0]=-vmc_all.att[ROLr] / 57.3;
        eu_ang[1]=vmc_all.att[PITr] / 57.3;
        eu_ang[2]=-vmc_all.att[YAWr] / 57.3;
        for (int j = 0; j < 3; ++j) {
            obs_now[35 + j] = omega[j] * omega_scale;
            obs_now[38 + j] = eu_ang[j] * eu_ang_scale;
        }
#else
        obs_now <<
                gait_signal[0],
                gait_signal[1],
                cmd_x * lin_vel,
                cmd_y * lin_vel,
                cmd_rate * ang_vel,
                (leg_motor_all.q_now[id_list[0]] / 57.3 - vmc_all.default_action[id_list[0]]) * pos_scale,
                (leg_motor_all.q_now[id_list[1]] / 57.3 - vmc_all.default_action[id_list[1]]) * pos_scale,
                (leg_motor_all.q_now[id_list[2]] / 57.3 - vmc_all.default_action[id_list[2]]) * pos_scale,
                (leg_motor_all.q_now[id_list[3]] / 57.3 - vmc_all.default_action[id_list[3]]) * pos_scale,
                (leg_motor_all.q_now[id_list[4]] / 57.3 - vmc_all.default_action[id_list[4]]) * pos_scale,
                (leg_motor_all.q_now[id_list[5]] / 57.3 - vmc_all.default_action[id_list[5]]) * pos_scale,
                (leg_motor_all.q_now[id_list[6]] / 57.3 - vmc_all.default_action[id_list[6]]) * pos_scale,
                (leg_motor_all.q_now[id_list[7]] / 57.3 - vmc_all.default_action[id_list[7]]) * pos_scale,
                (leg_motor_all.q_now[id_list[8]] / 57.3 - vmc_all.default_action[id_list[8]]) * pos_scale,
                (leg_motor_all.q_now[id_list[9]] / 57.3 - vmc_all.default_action[id_list[9]]) * pos_scale,
                (leg_motor_all.qd_now[id_list[0]] / 57.3) * vel_scale,
                (leg_motor_all.qd_now[id_list[1]] / 57.3) * vel_scale,
                (leg_motor_all.qd_now[id_list[2]] / 57.3) * vel_scale,
                (leg_motor_all.qd_now[id_list[3]] / 57.3) * vel_scale,
                (leg_motor_all.qd_now[id_list[4]] / 57.3) * vel_scale,
                (leg_motor_all.qd_now[id_list[5]] / 57.3) * vel_scale,
                (leg_motor_all.qd_now[id_list[6]] / 57.3) * vel_scale,
                (leg_motor_all.qd_now[id_list[7]] / 57.3) * vel_scale,
                (leg_motor_all.qd_now[id_list[8]] / 57.3) * vel_scale,
                (leg_motor_all.qd_now[id_list[9]] / 57.3) * vel_scale,
                action[0],
                action[1],
                action[2],
                action[3],
                action[4],
                action[5],
                action[6],
                action[7],
                action[8],
                action[9],
                -vmc_all.att_rate[ROLr] / 57.3 * omega_scale,
                vmc_all.att_rate[PITr] / 57.3 * omega_scale,
                vmc_all.att_rate[YAWr] / 57.3 * omega_scale,
                -vmc_all.att[ROLr] / 57.3 * eu_ang_scale,
                vmc_all.att[PITr] / 57.3 * eu_ang_scale,
               -vmc_all.att[YAWr] / 57.3 * eu_ang_scale;
#endif
        //printf("%f %f %f\n",dt,timer[0],vmc_all.net_run_dt1);
         //cout<<"obs:"<<endl<<obs_now<<endl;
         //cout<<"att:"<<endl<<dt<<obs_now[35]<<obs_now[36]<<obs_now[37]<<obs_now[38]<<obs_now[39]<<obs_now[40]<<endl;
         //printf("dt=%.3f| %.2f %.2f %.2f %.2f %.2f %.2f\n",dt,obs_now[35],obs_now[36],obs_now[37],obs_now[38],obs_now[39],obs_now[40]);
       // printf("vmc_all.rl_gait_time=%f s1=%f s2=%f\n",vmc_all.rl_gait_time,gait_signal[0],gait_signal[1]);
#endif
        obs_all.head(num_single_obs*(frame_stack-1)) = obs_all.tail(num_single_obs*(frame_stack-1));
        obs_all.tail(num_single_obs) = obs_now;

//        tvm2.in = obs_all;
//        float t1,t2;
//        t1= Get_T_Trig();
//        tvm2.run_s();
//        t2= Get_T_Trig();
//        printf("o-t1=%f t2=%f dt=%f\n",t1,t2,t2-t1);
        timer[0]=0;
    }
#if 1
    action = actionOld * 0.2 + tvm.out * 0.8;
    //action = tvm2.out;
#endif
    actionOld = tvm2.out;

    for (int i = 0; i < 10; i++)
    {
        action[i]=LIMIT(action[i], -3, 3);
    }
    actionTmp = action;
#if 0
    printf("********************RL ENTERED***********************");
    printf("rl-action:%.3f %.3f %.3f %.3f %.3f| %.3f %.3f %.3f %.3f %.3f\n",
           actionTmp[0],actionTmp[1],actionTmp[2],actionTmp[3],actionTmp[4],
           actionTmp[5],actionTmp[6],actionTmp[7],actionTmp[8],actionTmp[9]);
#endif
    for (int i = 0; i < 10; i++)
    {
        float temp = 1*actionTmp[i] * vmc_all.action_scale + vmc_all.default_action[id_list[i]];
        leg_motor_all.q_set[id_list[i]] = temp * 57.3;
    }

}
#endif
