#include "include.h"
#include "locomotion_header.h"
#if !RUN_WEBOTS
#include "can.h"
#endif
#include "gait_math.h"

//测量陀螺仪数据    弧度制
float FLT_ATT_RATE=0;//WS
void subscribe_imu_to_webot(robotTypeDef* rob,float dt)
{
	char i,j;
	robotwb.IMU_now.pitch=vmc_all.att[PITr];
	robotwb.IMU_now.roll=vmc_all.att[ROLr];
	robotwb.IMU_now.yaw=vmc_all.att[YAWr];
	
  for(i=0;i<3;i++)
  {
    for(j=0;j<3;j++){
    robotwb.Rb_n[i][j]=vmc_all.Rb_n[i][j];
    robotwb.Rn_b[i][j]=vmc_all.Rn_b[i][j];
		robotwb.Rb_n_noroll[i][j]=vmc_all.Rb_n_noroll[i][j];
    robotwb.Rn_b_noroll[i][j]=vmc_all.Rn_b_noroll[i][j];
    }
  }

	DigitalLPF_Double(vmc_all.att_rate[PITr], &robotwb.IMU_dot.pitch, FLT_ATT_RATE, dt);
	DigitalLPF_Double(vmc_all.att_rate[ROLr], &robotwb.IMU_dot.roll, FLT_ATT_RATE, dt);
	DigitalLPF_Double(vmc_all.att_rate[YAWr], &robotwb.IMU_dot.yaw, FLT_ATT_RATE, dt);

	robotwb.now_att=robotwb.IMU_now;
	robotwb.now_rate=robotwb.IMU_dot;
}

void subscribe_webot_to_vmc(float dt)//将传感器数据赋值给VMC
{
  int i=0,j=0;
  for(i=0;i<4;i++)
  {
		leg_motor[i].t_to_i[0]=pos_force_p.t_to_i;//设置扭矩系数<<-----------------------------
		leg_motor[i].t_to_i[1]=leg_motor[i].t_to_i[0];
		
		#if Q_NOW_USE_SET
		vmc[i].sita1=vmc[i].tar_sita1;
    vmc[i].sita2=vmc[i].tar_sita2;
		#else
		if(vmc[i].param.q_now_use_tar){//使用期望作为当前反馈  舵狗模式
			vmc[i].sita1=vmc[i].tar_sita1;
			vmc[i].sita2=vmc[i].tar_sita2;		
		}else{
			vmc[i].sita1=robotwb.Leg[i].sita[0];
			vmc[i].sita2=robotwb.Leg[i].sita[1];
			vmc[i].dsita1=robotwb.Leg[i].sita_d[0];
			vmc[i].dsita2=robotwb.Leg[i].sita_d[1];	
		}
		#endif

		vmc[i].delta_ht=vmc_robot_p.sw_deltah;
    robotwb.Leg[i].is_ground= vmc[i].ground;
    robotwb.Leg[i].trig_state=vmc[i].param.trig_state;
		
    #if !GROUND_USE_EST//使用着地传感器
    vmc[i].is_touch=robotwb.Leg[i].is_touch;
    #else
    vmc[i].is_touch=robotwb.Leg[i].is_touch_est;
    #endif
		robotwb.Leg[i].limit_tao[0]=pos_force_p.max_t;
		robotwb.Leg[i].limit_tao[1]=pos_force_p.max_t;  

		robotwb.Leg[i].f_pos_pid[Xr]=pos_force_p.f_pos_pid_st[Xr]; 
		robotwb.Leg[i].f_pos_pid[Zr]=pos_force_p.f_pos_pid_st[Zr]; 
		robotwb.Leg[i].f_pid[Xr]=pos_force_p.f_pid_st[Xr]; 
		robotwb.Leg[i].f_pid[Zr]=pos_force_p.f_pid_st[Zr]; 
		
		//关节PD参数
		if(vmc_all.gait_mode==TROT||vmc_all.gait_mode==F_TROT)
		{
				if(stand_force_enable_flag[i]){
				robotwb.Leg[i].q_pid=pos_force_p.q_pid_st_trot; 	
				}	
				else
				{
				robotwb.Leg[i].q_pid=pos_force_p.q_pid_sw;//摆动参数
				}		
		}else{
				if(robotwb.Leg[i].is_ground){//支撑参数
				robotwb.Leg[i].q_pid=pos_force_p.q_pid_st_stance; 	
				}
				else
				{
				robotwb.Leg[i].q_pid=pos_force_p.q_pid_sw;//摆动参数
				}
		}
		leg_motor[i].max_t[0]=leg_motor[i].max_t[1]=leg_motor[i].max_t[2]=pos_force_p.max_t;
		leg_motor[i].max_i[0]=leg_motor[i].max_i[1]=leg_motor[i].max_i[2]=pos_force_p.max_i;
  }
	
	//地形角期望命令
	vmc_all.ground_att_cmd[PITr]=vmc_all.ground_att_est[PITr];

  robotwb.exp_att.pitch=vmc_all.tar_att[PITr]+LIMIT(vmc_all.ground_att_cmd[PITr]*(vmc_all.gait_mode>0)*EN_ATT_GROUND_CONTROL,-20,20);
  robotwb.exp_att.roll= vmc_all.tar_att[ROLr]+LIMIT(vmc_all.ground_att_cmd[ROLr]*(vmc_all.gait_mode>0)*EN_ATT_GROUND_CONTROL,-5,5);
}

void publish_vmc_to_webot(float dt)
{
    int i=0;
    for(i=0;i<4;i++){
    float alfa=(robotwb.Leg[i].sita[1]-robotwb.Leg[i].sita[0])/2;
	  float beta=(robotwb.Leg[i].sita[1]+robotwb.Leg[i].sita[0])/2;
		robotwb.Leg[i].beta=beta;
		robotwb.Leg[i].alfa=alfa;

    robotwb.Leg[i].r=vmc[i].r;
    robotwb.Leg[i].sita_r=vmc[i].sita;
    robotwb.Leg[i].epos_h.x=vmc[i].epos.x;
		robotwb.Leg[i].epos_h.y=vmc[i].epos.y;
    robotwb.Leg[i].epos_h.z=vmc[i].epos.z;

		robotwb.Leg[i].epos_b.x=vmc[i].epos_b.x;
		robotwb.Leg[i].epos_b.y=vmc[i].epos_b.y;
		robotwb.Leg[i].epos_b.z=vmc[i].epos_b.z;

		robotwb.Leg[i].epos_n.x=vmc[i].epos_n.x;
		robotwb.Leg[i].epos_n.y=vmc[i].epos_n.y;
		robotwb.Leg[i].epos_n.z=vmc[i].epos_n.z;
		
    robotwb.Leg[i].espd_h.x=vmc[i].spd.x;
		robotwb.Leg[i].espd_h.y=vmc[i].spd.y;
    robotwb.Leg[i].espd_h.z=vmc[i].spd.z;

    robotwb.Leg[i].espd_n.x=vmc[i].spd_n.x;
		robotwb.Leg[i].espd_n.y=vmc[i].spd_n.y;
    robotwb.Leg[i].espd_n.z=vmc[i].spd_n.z;
		
    robotwb.Leg[i].jacobi[0]=vmc[i].jacobi22[0];
    robotwb.Leg[i].jacobi[2]=vmc[i].jacobi22[2];

    robotwb.Leg[i].jacobi[1]=vmc[i].jacobi22[1];
    robotwb.Leg[i].jacobi[3]=vmc[i].jacobi22[3];

    robotwb.Leg[i].jacobi_inv[0]=vmc[i].ijacobi22[0];
    robotwb.Leg[i].jacobi_inv[2]=vmc[i].ijacobi22[2];

    robotwb.Leg[i].jacobi_inv[1]=vmc[i].ijacobi22[1];
    robotwb.Leg[i].jacobi_inv[3]=vmc[i].ijacobi22[3];
		}
}

void readAllMotorPos(robotTypeDef* rob,float dt)
{
    static float cnt;
		uint8_t i,j;
	  static char init=0;

		dt=LIMIT(dt,0.001,0.1);
    float taom[4][3]={0};
		float temp_sita[4][3]={0};
    cnt+=dt;
    for (i = 0; i < 4; i++){//在convert_vmc_webot_data中赋值给VMC
			if(leg_motor[i].connect&&
				leg_motor[i].connect_motor[0]&&
			  leg_motor[i].ready[0]){
				rob->Leg[i].sita[0]=(leg_motor[i].q_now[0]);//sita0  正负数
				temp_sita[i][0]=leg_motor[i].q_now[0];
				rob->Leg[i].taom[0]=leg_motor[i].t_now[0];
			}else{//非连接显示
				rob->Leg[i].sita[0]=0;
				rob->Leg[i].taom[0]=0;
				temp_sita[i][0]=0;			
			}
			
			if(leg_motor[i].connect&&
			  leg_motor[i].connect_motor[1]&&
			  leg_motor[i].ready[1]){
				rob->Leg[i].sita[1]=To_360_degreesw(leg_motor[i].q_now[1]);//sita1 0~360
				temp_sita[i][1]=leg_motor[i].q_now[1];	
				rob->Leg[i].taom[1]=leg_motor[i].t_now[1];
					}else{//非连接显示
				rob->Leg[i].sita[1]=180;
				rob->Leg[i].taom[1]=0;
				temp_sita[i][1]=180;					
			}

			rob->Leg[i].sita_d[0]=Moving_Median(2*i,	3,  To_180_degreesw(temp_sita[i][0]-rob->Leg[i].sita_reg[0])/dt);
			rob->Leg[i].sita_d[1]=Moving_Median(2*i+1,3,	To_180_degreesw(temp_sita[i][1]-rob->Leg[i].sita_reg[1])/dt);

			rob->Leg[i].sita_reg[0]=temp_sita[i][0];
			rob->Leg[i].sita_reg[1]=temp_sita[i][1];
    }
}


void set_motor_q(int id)
{  
    robotwb.Leg[id].tar_sita[0]=LIMIT(robotwb.Leg[id].tar_sita[0],-robotwb.Leg[id].limit_sita[0],180+robotwb.Leg[id].limit_sita[0]);
    robotwb.Leg[id].tar_sita[1]=LIMIT(robotwb.Leg[id].tar_sita[1],-robotwb.Leg[id].limit_sita[1],180+robotwb.Leg[id].limit_sita[1]);
}

void set_motor_t(int id)
{
    robotwb.Leg[id].taod[0]=LIMIT(robotwb.Leg[id].taod[0],-robotwb.Leg[id].limit_tao[0],robotwb.Leg[id].limit_tao[0]);
    robotwb.Leg[id].taod[1]=LIMIT(robotwb.Leg[id].taod[1],-robotwb.Leg[id].limit_tao[1],robotwb.Leg[id].limit_tao[1]);
    robotwb.Leg[id].taom_output[0]=robotwb.Leg[id].taod[0];//记录力矩输出
    robotwb.Leg[id].taom_output[1]=robotwb.Leg[id].taod[1];
	#if EN_TORQUE_CONTROL
		leg_motor[id].set_t[0]=robotwb.Leg[id].taom_output[0];
		leg_motor[id].set_t[1]=robotwb.Leg[id].taom_output[1];
		leg_motor[id].set_i[0]=leg_motor[id].set_t[0]/(leg_motor[id].t_to_i[0]+0.0000001);
		leg_motor[id].set_i[1]=leg_motor[id].set_t[1]/(leg_motor[id].t_to_i[1]+0.0000001);
	#else
		leg_motor[id].set_t[0]=0;
		leg_motor[id].set_t[1]=0;
	#endif
}
