#include "include.h"
#if 0 // YH lite onnx good 10 action
#include "gait_math.h"
#include "eso.h"
#include "locomotion_header.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "can.h"
#include "sys_time.h"
#include <iostream>
#include "joint_map.h"
#include "ankle.h"
#include "spi_node.h"
#include "iopack.h"
Jnt::ankleClass ankle_lite_l(0); // 左+右
Jnt::ankleClass ankle_lite_r(1); // 左+右

#include "onnx.h"
#include <iomanip>

Onnx::onnxClass onnx;
Onnx::onnxClass *onnxPtr;
#define TEST_SIN 0
#define obs_size 44
#define hist_len 21
#define act_size 10

int frame_stack = hist_len;
int num_single_obs = obs_size;

vecNf(obs_size) obs1_onnx, obs_now;
vecNf(hist_len *obs_size) obs_all_onnx;
vecNf(act_size) action_onnx, actionOld_onnx, actionTmp_onnx, actionFilt_onnx, actionFilt_out;

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

int id_list[12] = {0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12};
int id_list_o[10] = {0, 1, 2, 3, 4,  7, 8, 9, 10, 11};
int rl_model_loaded = 0;
int first_loaded_model = 0;

float ankle_ref_left[2] = {0};
float ankle_ref_right[2] = {0};

int file_open = 0;
std::string filename = "Data/output_lite.csv";
std::ofstream file(filename);

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
    printf("RL_Modle_load_test YH Lite\n");
    onnx.init("Model_YH/Trot_2/policy.onnx", "lite_onnx");
    printf("RL_Modle_load_test pass| vmc_all.net_run_dt1=%f  vmc_all.rl_gait_duty=%f vmc_all.action_scale=%f\n",vmc_all.net_run_dt1, vmc_all.rl_gait_duty,vmc_all.action_scale);

}

int first_mask=0;
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

    vmc_all.rl_gain=1.0;//doghome

    for(int i=0;i<14;i++)
        leg_motor_all.stiff[i]=vmc_all.rl_gain;
    for(int i=0;i<14;i++)
        leg_motor_all.stiff_servo[i]=1.6;
#endif
    vmc_all.att_measure_bias[PITr] = config_gait["vmc_param"]["att_bias_pit"].as<float>();
    vmc_all.att_measure_bias[ROLr] = config_gait["vmc_param"]["att_bias_rol"].as<float>();

    first_mask=0;
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
            printf("Start initialization!\n");
            loaded = onnx.init("Model_YH/Trot_2/policy.onnx", "trot_onnx");
            onnxPtr = &onnx;
            printf("Initialization finished! Model_YH");
            vmc_all.rl_model_type = 2;
            break; // tort
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
    static int cnt_loss_motor=0;
    static float q_now[12];
    static float qd_now[12];
    static float t1, t2;
    static float timer[10];
    int cnt_motor;

    eu_ang_scale = 1;
    omega_scale = 1;
    pos_scale = 1;
    vel_scale = 0.05;
    lin_vel = 2;
    ang_vel = 0.25;

    timer[0] += dt;
    // printf("RL cycle entered!\n");
    for(int i=0;i<14;i++)
        if(leg_motor_all.connect_motor[i]==1)
            cnt_motor++;

    if(cnt_loss_motor!=cnt_motor){
        printf("leg_motor change=%d\n",cnt_motor);
        for(int i=0;i<14;i++)
            printf("%d",leg_motor_all.connect_motor[i]);
        printf("\n");
        if(first_mask==0){
            first_mask=1;
        }else
            ocu.key_ud=-2;

    }

    cnt_loss_motor=cnt_motor;

    vmc_all.rl_gait_time += dt;
    if (timer[0] > vmc_all.net_run_dt1 && rl_model_loaded)
    { //
        float sys_dt = Get_Cycle_T(14);

        float cmd_x_temp = 0;
        float cmd_y_temp = 0;
        //vmc_all.rl_gait_time += vmc_all.net_run_dt1;
        cmd_x_temp = LIMIT(vmc_all.tar_spd.x, -MAX_SPD_X * 0.5, MAX_SPD_X) + vmc_all.rl_commond_rl_rst[0] +
                     vmc_all.rl_commond_off[4] * vmc_all.rl_commond_off[0] * (vmc_all.gait_mode == G_RL);
        cmd_y_temp = LIMIT(-vmc_all.tar_spd.y, -MAX_SPD_Y, MAX_SPD_Y) +
                     vmc_all.rl_commond_off[4] * vmc_all.rl_commond_off[1] * (vmc_all.gait_mode == G_RL);

        float cmd_rate_temp = 0;
        if (fabs(vmc_all.tar_spd.z) > 1)
        {
            cmd_rate_temp = vmc_all.tar_spd.z / 57.3 * 1.5;
            robotwb.exp_att.yaw = robotwb.now_att.yaw;
        }
        else
        {
            cmd_rate_temp = -dead(limitw(To_180_degrees(robotwb.exp_att.yaw - robotwb.now_att.yaw), -25, 25), 0.25) * 1.4 / 57.3;
        }
        cmd_rate_temp = LIMIT(cmd_rate_temp, -MAX_SPD_RAD / 57.3, MAX_SPD_RAD / 57.3);

        cmd_x = cmd_x * (1 - smooth_rc) + (std::fabs(cmd_x_temp) < dead_zone_rc ? 0.0 : cmd_x_temp) * smooth_rc;
        cmd_y = cmd_y * (1 - smooth_rc) + (std::fabs(cmd_y_temp) < dead_zone_rc ? 0.0 : cmd_y_temp) * smooth_rc;
        cmd_rate = cmd_rate * (1 - smooth_rc) + (std::fabs(cmd_rate_temp) < dead_zone_rc ? 0.0 : cmd_rate_temp) * smooth_rc;

        float gait_signal[2] = {0};

        gait_signal[0] = sin(3.1415926 * 2 * vmc_all.rl_gait_time / vmc_all.rl_gait_duty);
        gait_signal[1] = cos(3.1415926 * 2 * vmc_all.rl_gait_time / vmc_all.rl_gait_duty);

        float q_convert[2][2];
        float qd_convert[2][2];
        float temp;
        t1 = Get_T_Trig();

        ankle_lite_l.setActMot(-leg_motor_all.q_now[id_list[4]] / 57.3, -leg_motor_all.q_now[id_list[5]] / 57.3,
                               -leg_motor_all.qd_now[id_list[4]] / 57.3, -leg_motor_all.qd_now[id_list[5]] / 57.3,
                               0, 0);
        ankle_lite_r.setActMot(-leg_motor_all.q_now[id_list[10]] / 57.3, -leg_motor_all.q_now[id_list[11]] / 57.3,
                               -leg_motor_all.qd_now[id_list[10]] / 57.3, -leg_motor_all.qd_now[id_list[11]] / 57.3,
                               0, 0);
        ankle_lite_l.getActAnk(q_convert[0][0], q_convert[0][1],
                               qd_convert[0][0], qd_convert[0][1],
                               temp, temp);
        ankle_lite_r.getActAnk(q_convert[1][0], q_convert[1][1],
                               qd_convert[1][0], qd_convert[1][1],
                               temp, temp);

        for (int i = 0; i < 12; i++)
        {
            q_now[i] = leg_motor_all.q_now[id_list[i]];
            qd_now[i] = leg_motor_all.qd_now[id_list[i]];
        }

        q_now[4] = q_convert[0][0] * 57.3;// degreed
        q_now[5] = q_convert[0][1] * 57.3;
        q_now[10] = q_convert[1][0] * 57.3;
        q_now[11] = q_convert[1][1] * 57.3;

        qd_now[4] = qd_convert[0][0] * 57.3;// degreed
        qd_now[5] = qd_convert[0][1] * 57.3;
        qd_now[10] = qd_convert[1][0] * 57.3;
        qd_now[11] = qd_convert[1][1] * 57.3;

        float omega[3];
        float eu_ang[3];
        float gvec[3];
#if 0 // tinker inner IMU
          omega[0] =    vmc_all.att_rate[ROLr] / 57.3;
          omega[1] =   -vmc_all.att_rate[PITr] / 57.3;
          omega[2] =    vmc_all.att_rate[YAWr] / 57.3;
          eu_ang[0] =   vmc_all.att[ROLr] / 57.3;
          eu_ang[1] =  -vmc_all.att[PITr] / 57.3;
          eu_ang[2] =  -vmc_all.att[YAWr] / 57.3;
#elif 0 // outer IMU1
        omega[0] = -vmc_all.att_rate[ROLr] / 57.3;
        omega[1] = vmc_all.att_rate[PITr] / 57.3;
        omega[2] = vmc_all.att_rate[YAWr] / 57.3;
        eu_ang[0] = -vmc_all.att[ROLr] / 57.3;
        eu_ang[1] = vmc_all.att[PITr] / 57.3;
        eu_ang[2] = -vmc_all.att[YAWr] / 57.3*0;
#else  // outer IMU new
        omega[0] =    vmc_all.att_rate[PITr] / 57.3;
        omega[1] =    vmc_all.att_rate[ROLr] / 57.3;
        omega[2] =    vmc_all.att_rate[YAWr] / 57.3*0;
        eu_ang[0] =   vmc_all.att[PITr] / 57.3;
        eu_ang[1] =   vmc_all.att[ROLr] / 57.3;
        eu_ang[2] =   -vmc_all.att[YAWr] / 57.3;
#endif
        eulerToGravity(eu_ang, gvec);
        //printf("eu_ang: %.2f, %.2f, %.2f, gvec: %.2f,%.2f,%.2f\n", eu_ang[0], eu_ang[1], eu_ang[2], gvec[0], gvec[1], gvec[2]);

        obs_now <<
            0,
            0,
            0,
            cmd_x * lin_vel * 1,
            cmd_y * lin_vel * 1,
            cmd_rate * ang_vel * 1,
            gait_signal[0],
            gait_signal[1],
            omega[0] * ang_vel * 1,
            omega[1] * ang_vel * 1,
            omega[2] * ang_vel * 1,
            gvec[0] * 1,
            gvec[1] * 1,
            gvec[2] * 1,
            (q_now[0] / 57.3 - vmc_all.default_action[id_list[0]]) * pos_scale,
            (q_now[1] / 57.3 - vmc_all.default_action[id_list[1]]) * pos_scale,
            (q_now[2] / 57.3 - vmc_all.default_action[id_list[2]]) * pos_scale,
            (q_now[3] / 57.3 - vmc_all.default_action[id_list[3]]) * pos_scale,
            (q_now[4] / 57.3 - vmc_all.default_action[id_list[4]]) * pos_scale,
//            (q_now[5] / 57.3 - vmc_all.default_action[id_list[5]]) * pos_scale,
            (q_now[6] / 57.3 - vmc_all.default_action[id_list[6]]) * pos_scale,
            (q_now[7] / 57.3 - vmc_all.default_action[id_list[7]]) * pos_scale,
            (q_now[8] / 57.3 - vmc_all.default_action[id_list[8]]) * pos_scale,
            (q_now[9] / 57.3 - vmc_all.default_action[id_list[9]]) * pos_scale,
            (q_now[10] / 57.3 - vmc_all.default_action[id_list[10]]) * pos_scale,
//            (q_now[11] / 57.3 - vmc_all.default_action[id_list[11]]) * pos_scale,
            (qd_now[0] / 57.3) * vel_scale,
            (qd_now[1] / 57.3) * vel_scale,
            (qd_now[2] / 57.3) * vel_scale,
            (qd_now[3] / 57.3) * vel_scale,
            (qd_now[4] / 57.3) * vel_scale,
//            (qd_now[5] / 57.3) * vel_scale,
            (qd_now[6] / 57.3) * vel_scale,
            (qd_now[7] / 57.3) * vel_scale,
            (qd_now[8] / 57.3) * vel_scale,
            (qd_now[9] / 57.3) * vel_scale,
            (qd_now[10] / 57.3) * vel_scale,
//            (qd_now[11] / 57.3) * vel_scale,
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

        onnxPtr->in = obs_all_onnx;
        onnxPtr->run();
        t2 = Get_T_Trig();

        for (int i = 0; i < num_single_obs*frame_stack ; i++)
        {
            obs_all_onnx[i] = LIMIT(obs_all_onnx[i], -100, 100);
        }

        if(sys_dt>0.03)
            printf("!!!!!!!!!!!!!!sys_dt over timer=%f\n",sys_dt);

        // printf("o-t1=%f t2=%f dt=%f\n", t1, t2, t2 - t1);
        timer[0] = 0;
        action_onnx = onnxPtr->out;
//        action_onnx[1] = LIMIT(action_onnx[1], -0.6, 0.6); // roll
//        action_onnx[2] = LIMIT(action_onnx[2], -0.5, 0.5);
//        action_onnx[6] = LIMIT(action_onnx[6], -0.6, 0.6); // roll
//        action_onnx[7] = LIMIT(action_onnx[7], -0.5, 0.5);
        for (int i = 0; i < 10; i++)
        {
            action_onnx[i] = LIMIT(action_onnx[i], -100, 100);
        }
        actionOld_onnx = action_onnx;
    }
#if 1
    for(int i=0;i<14;i++)
        nav_rx.temp_recordi[i]=obs_all_onnx[i];
    for(int i=0;i<10;i++)
        nav_rx.temp_recordi[14+i]=leg_motor_all.q_now[id_list_o[i]]/ 57.3;
    for(int i=0;i<10;i++)
        nav_rx.temp_recordi[14+10+i]=leg_motor_all.q_set[id_list_o[i]]/ 57.3;
    for(int i=0;i<10;i++)
        nav_rx.temp_recordi[14+10+10+i]=leg_motor_all.qd_now[id_list_o[i]]/ 57.3;
    for(int i=0;i<10;i++)
        nav_rx.temp_recordi[14+10+10+10+i]=leg_motor_all.t_now[id_list_o[i]];

    for(int i=0;i<10;i++)
        nav_rx.temp_recordo[i]=action_onnx[i];
    nav_rx.temp_recordo[11]=t2 - t1;

    for (int i = 0; i < 10; i++)
    {
        if(fabs(action_onnx[i])>3)
            printf("action_onnx %d over=%f\n",i,action_onnx[i]);
    }
#endif
#if 0
        printf("vmc_all.rl_gait_time=%f timer[0]=%f\n",vmc_all.rl_gait_time,timer[0]);
        printf("L:%0.1f %0.1f %0.1f %0.1f %0.1f %0.1f\n",leg_motor_all.q_now[id_list[0]],leg_motor_all.q_now[id_list[1]],leg_motor_all.q_now[id_list[2]]
                ,leg_motor_all.q_now[id_list[3]],leg_motor_all.q_now[id_list[4]],leg_motor_all.q_now[id_list[5]]);
        //printf("L_ankle:%.2f %.2f\n",q_convert[0][0]*57.3,q_convert[0][1]*57.3);
        printf("R:%0.1f %0.1f %0.1f %0.1f %0.1f %0.1f\n",leg_motor_all.q_now[id_list[6]],leg_motor_all.q_now[id_list[7]],leg_motor_all.q_now[id_list[8]]
                ,leg_motor_all.q_now[id_list[9]],leg_motor_all.q_now[id_list[10]],leg_motor_all.q_now[id_list[11]]);
        //printf("r_ankle:%.2f %.2f\n",q_convert[1][0]*57.3,q_convert[1][1]*57.3);
#endif
    float filt_omega=vmc_all.rl_flt;
    for (int i = 0; i < 10; i++)
        actionFilt_onnx[i] = (filt_omega) * action_onnx[i] + (1 - filt_omega) * actionFilt_onnx[i];

    actionTmp_onnx = actionFilt_onnx;

#if TEST_SIN // disable!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    for (int i = 0; i < 10; i++)
         actionTmp_onnx[i]=0;
#endif
    for (int i = 0; i < 10; i++)
    {
        actionFilt_out[i] = 1 * actionTmp_onnx[i] * vmc_all.action_scale + vmc_all.default_action[id_list_o[i]];
    }
    float set_convert[2][2];
    float temp = 0;

#if TEST_SIN
    float gait_signal[2] = {0};

    gait_signal[0] = sin(3.1415926 * 2 * vmc_all.rl_gait_time / vmc_all.rl_gait_duty);
    gait_signal[1] = cos(3.1415926 * 2 * vmc_all.rl_gait_time / vmc_all.rl_gait_duty);

    ankle_ref_left[0] = 0; // gait_signal[0] * 0.1 + 0.1;
    ankle_ref_left[1] = gait_signal[0] * 0.4;
    ankle_ref_right[0] = 0; // gait_signal[0] * 0.1 + 0.1;
    ankle_ref_right[1] = gait_signal[0] * 0.4;

    set_convert[0][0] = ankle_ref_left[0]; // pitch
    set_convert[0][1] = ankle_ref_left[1];
    set_convert[1][0] = ankle_ref_right[0]; // pitch
    set_convert[1][1] = ankle_ref_right[1];
#else
     set_convert[0][0] = LIMIT(actionFilt_out[4], -0.8, 0.9); // pitch
     set_convert[0][1] = LIMIT(0, -0.4, 0.4);
     set_convert[1][0] = LIMIT(actionFilt_out[9], -0.8, 0.9); // pitch
     set_convert[1][1] = LIMIT(0, -0.4, 0.4);
#endif
    float set_convert_out[2][2];
    ankle_lite_l.setTgtAnk(-set_convert[0][0], -set_convert[0][1],
                           temp, temp,
                           temp, temp);
    ankle_lite_r.setTgtAnk(-set_convert[1][0], -set_convert[1][1],
                           temp, temp,
                           temp, temp);
    ankle_lite_l.getTgtMot(set_convert_out[0][0],set_convert_out[0][1],
                           temp, temp,
                           temp, temp);
    ankle_lite_r.getTgtMot(set_convert_out[1][0],set_convert_out[1][1],
                           temp, temp,
                           temp, temp);

   leg_motor_all.q_set[id_list[0]] = actionFilt_out[0] * 57.3;
   leg_motor_all.q_set[id_list[1]] = actionFilt_out[1] * 57.3;
   leg_motor_all.q_set[id_list[2]] = actionFilt_out[2] * 57.3;
   leg_motor_all.q_set[id_list[3]] = actionFilt_out[3] * 57.3;
   leg_motor_all.q_set[id_list[4]] = set_convert_out[0][0]  * 57.3;
   leg_motor_all.q_set[id_list[5]] = set_convert_out[0][1]  * 57.3;
   leg_motor_all.q_set[id_list[6]] = actionFilt_out[5] * 57.3;
   leg_motor_all.q_set[id_list[7]] = actionFilt_out[6] * 57.3;
   leg_motor_all.q_set[id_list[8]] = actionFilt_out[7] * 57.3;
   leg_motor_all.q_set[id_list[9]] = actionFilt_out[8] * 57.3;
   leg_motor_all.q_set[id_list[10]] = set_convert_out[1][0] * 57.3;
   leg_motor_all.q_set[id_list[11]] = set_convert_out[1][1] * 57.3;

   //arm
#if !TEST_SIN&&0
   leg_motor_all.q_set_servo[1]=8;
   leg_motor_all.q_set_servo[2]=-130;
   leg_motor_all.q_set_servo[3]=156;
   leg_motor_all.q_set_servo[4]=-145;

   leg_motor_all.q_set_servo[8]=-8;
   leg_motor_all.q_set_servo[9]=130;
   leg_motor_all.q_set_servo[10]=-156;
   leg_motor_all.q_set_servo[11]=145;
#endif
}

void Gait_RL_Update_1(float dt)
{
}
#endif
