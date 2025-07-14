#ifndef __LOCOMOITON_H__
#define __LOCOMOITON_H__
#include "base_struct.h"
//----------------------------------------------VMC controller---------------------
void body_traj_planner(float dt);
void body_servo_control(float dt);
void force_dis_n(void);
void reset_servo_interge(void);

//---------------------------------------------Force IMP controller----------------------
void force_control_and_dis_stand(float dt);
void force_control_and_dis_trot(float dt);
void force_control_and_dis_trot_local(float dt);
//---------------------------------------------state estimator-------------------------
typedef struct{
	END_POS acc_n_m;
	END_POS acc_n_f;
	END_POS acc_nn_f;
	
	END_POS odom_spd_n;
	END_POS odom_pos_n;

	END_POS odom_spd_nn;
	END_POS odom_pos_nn;
	
	END_POS odom_dspd_n;
	END_POS odom_dpos_n;
	
	END_POS odom_spd_n_r;
	END_POS odom_pos_n_r;
	float Kpv;
}_ODOM_1;
extern _ODOM_1 odom1;

typedef struct 
{
  int state_gait;

}Gait_Mode;

extern Gait_Mode gait_ww;
extern char sit_down_flag;

void reset_robot_statement(void);
void touchdown_check(robotTypeDef* rob, float dt);
void estimate_GRF(float dt);
void state_estimator(float dt);
void estimate_ground_att(float dt);

//---------------------------------------------hardware interface---------------------
void subscribe_imu_to_webot(robotTypeDef* rob,float dt);
void subscribe_webot_to_vmc(float dt);
void publish_vmc_to_webot(float dt);
void readAllMotorPos(robotTypeDef* rob,float dt);
void set_motor_q(int id);
void set_motor_t(int id);

//---------------------------------------------leg planner--------------------------------
void cal_vir_leg_pos(VMC* in, END_POS *td_vir_n, END_POS *lf_vir_n);
float end_pos_y_correction(VMC* in, float ep_slip_n, float dt);
void trig_curve_global(VMC *in, float *x, float *y, float *z,float *vx, float *vy, float *vz, float dt);
void espd_to_qspd(VMC *in, END_POS spd_end, float* spd_q, float dt);
void reset_sw_torque_as_now(VMC *in, float torque_now[3]);
void trig_plan(char leg_sel_trig, float dt);
char trig_lift(char leg_sel_trig, float dt);
char trig_swing(char leg_sel_trig, float dt);
char trig_td(char leg_sel_trig, float dt);
void swing_spd_control(char leg_sel_trig, float dt);
void reset_tar_pos(char leg_sel_trig);
void trig_plan_online(char leg_sel_trig, float dt);

//---------------------------------------------locomotion SFM--------------------------
typedef struct{
	char sdk_connect;
	int sdk_loss_cnt;
	char sdk_mode;
	char gait_mode;
	char cmd_power;
	char trig_mode;
	float cmd_vx;
	float cmd_vy;
	float cmd_vz;
	float cmd_vyaw;
	float cmd_z;
	float cmd_pit;
	float cmd_rol;
	float cmd_yaw;
	int cmd_pwm[12];
	float cmd_angle[4][3];
	END_POS trig_end_cmd[4];
}_SDK;
extern _SDK sdk;

extern char stand_force_enable_flag[5];
void vmc_param_init(void);
char safe_check(float dt);
void locomotion_sfm(float dt);
char gait_switch(float dt);
//-------------------------------------Gati  APP-----------------------------
void  Gait_Stand_Active(void);
void  Gait_Stand_Update(float dt);

void  Gait_Trot_Active(void);
void  Gait_Trot_Update_v1(float dt);

void  Gait_Recovery_Active(void);
void  Gait_Recovery_Falling(float dt);
void  Gait_Recovery_Update(float dt);
char move_joint_to_pos(VMC * in, int joint, float tar_angle, float max_spd, float dt);
char move_joint_to_pos1(VMC * in, int joint, float tar_angle, float max_spd, float err_check, float dt);
void move_joint_with_spd(VMC * in, int joint, float tar_spd, float dt);
char check_lisence(void);

void moco8_ml_lst(void);
void moco8_webots(void);
void moco8_ml(void);
#endif 