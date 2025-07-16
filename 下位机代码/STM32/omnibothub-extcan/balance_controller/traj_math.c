#include "include.h"
#include "eso.h"
#include "locomotion_header.h"
#include "gait_math.h"
#if !RUN_WEBOTS
#include "arm_math.h"
#endif

//-------------------------------------------------jerk trajectory planner------------------------------------------
_TRA traj[10];
void GenerateTrajectory(float  p0, float v0, float a0, float pf, float vf, float af, float Tf, char defined[3], float*a, float*b, float*g, float *cost) {
	char accGoalDefined = defined[0];
	char posGoalDefined = defined[1];
	char velGoalDefined = defined[2];
	//define starting position:
	float  delta_a = af - a0;
	float  delta_v = vf - v0 - a0 * Tf;
	float  delta_p = pf - p0 - v0 * Tf - 0.5*a0*Tf*Tf;

	// %powers of the end time:
	float  T2 = Tf * Tf;
	float  T3 = T2 * Tf;
	float  T4 = T3 * Tf;
	float  T5 = T4 * Tf;

	//%solve the trajectories, depending on what's constrained:
	if (posGoalDefined && velGoalDefined && accGoalDefined)
	{
		*a = (60 * T2*delta_a - 360 * Tf*delta_v + 720 * 1 * delta_p) / T5;
		*b = (-24 * T3*delta_a + 168 * T2*delta_v - 360 * Tf*delta_p) / T5;
		*g = (3 * T4*delta_a - 24 * T3*delta_v + 60 * T2*delta_p) / T5;
	}
	else if (posGoalDefined && velGoalDefined)
	{
		*a = (-120 * Tf*delta_v + 320 * delta_p) / T5;
		*b = (72 * T2*delta_v - 200 * Tf*delta_p) / T5;
		*g = (-12 * T3*delta_v + 40 * T2*delta_p) / T5;
	}
	else if (posGoalDefined && accGoalDefined)
	{
		*a = (-15 * T2*delta_a + 90 * delta_p) / (2 * T5);
		*b = (15 * T3*delta_a - 90 * Tf*delta_p) / (2 * T5);
		*g = (-3 * T4*delta_a + 30 * T2*delta_p) / (2 * T5);
	}
	else if (velGoalDefined && accGoalDefined)
	{
		*a = 0;
		*b = (6 * Tf*delta_a - 12 * delta_v) / T3;
		*g = (-2 * T2*delta_a + 6 * Tf*delta_v) / T3;
	}
	else if (posGoalDefined)

	{
		*a = 20 * delta_p / T5;
		*b = -20 * delta_p / T4;
		*g = 10 * delta_p / T3;
	}
	else if (velGoalDefined)
	{
		*a = 0;
		*b = -3 * delta_v / T3;
		*g = 3 * delta_v / T2;
	}
	else if (accGoalDefined)
	{
		*a = 0;
		*b = 0;
		*g = delta_a / Tf;
	}
	else {
		//%Nothing to do!
		*a = 0;
		*b = 0;
		*g = 0;
	}

	//%Calculate the cost:
	*cost = *g* *g + *b* *g*Tf + *b* *b*T2 / 3.0 + *a* *g*T2 / 3.0 + *a* *b*T3 / 4.0 + *a* *a*T4 / 20.0;
}

void get_trajecotry(float p0, float v0, float a0, float a, float b, float g, float t, float *pos, float *spd, float *acc, float *jerk) {
	*pos = p0 + v0 * t + (1 / 2.0)*a0*t*t + (1 / 6.0)*g*t*t*t + (1 / 24.0)*b*t*t*t*t + (1 / 120.0)*a*t*t*t*t*t;
	*jerk = g + b * t + (1 / 2.0)*a*t*t;
	*acc = a0 + g * t + (1 / 2.0)*b*t*t + (1 / 6.0)*a*t*t*t;
	*spd = v0 + a0 * t + (1 / 2.0)*g*t*t + (1 / 6.0)*b*t*t*t + (1 / 24.0)*a*t*t*t*t;
}


void plan_tra(_TRA *tra)
{
	char i, j;
	float cost[3];
	for (i = 0; i < 3; i++)
	{
		GenerateTrajectory(tra->ps[i], tra->vs[i], tra->as[i], tra->pe[i], tra->ve[i], tra->ae[i], tra->Time,
			tra->defined, &tra->param[i * 3 + 0], &tra->param[i * 3 + 1], &tra->param[i * 3 + 2], &cost[i]);
	}
	tra->cost_all = cost[0] + cost[1] + cost[2];
}

void get_tra(_TRA *tra, float t)
{
	char i;
	float acc, jerk;
	for (i = 0; i < 3; i++)
		get_trajecotry(tra->ps[i], tra->vs[i], tra->as[i],
			tra->param[i * 3 + 0], tra->param[i * 3 + 1], tra->param[i * 3 + 2]
			, t,
			&tra->pt[i], &tra->vt[i], &acc, &jerk);
}


//--------------------------------步态规划
int swing_jerk_planner_2d_3point(VMC* in, float lift_spd, float td_spd, float mid_spd_w, float T_sw)
{
	static char init[4];
	char flag[3];
	char i;
	int spd_flag[2] = { 1,1 };
	float cost;
	float spd_av = 0;

	in->param.end_planner.Tsw = T_sw;
	in->param.end_planner.leg_h = in->delta_ht;//抬腿高度
	in->param.end_planner.leg_dis = sqrtf(pow(in->st_pos.x - in->tar_pos.x, 2)
		+ pow(in->st_pos.y - in->tar_pos.y, 2)
		+ pow(in->st_pos.z - in->tar_pos.z, 2));
	if (in->tar_pos.x > in->st_pos.x)
		spd_flag[Xr] = 1;
	else
		spd_flag[Xr] = -1;

	in->param.end_planner.max_spd = in->param.end_planner.leg_dis / in->param.end_planner.Tsw;

	in->param.end_planner.T12 = in->param.end_planner.Tsw*0.5;
	in->param.end_planner.T23 = in->param.end_planner.Tsw - in->param.end_planner.T12;


	//%1 start
	in->param.end_planner.p1[0] = in->st_pos.x;
	in->param.end_planner.p1[1] = in->st_pos.y;
	in->param.end_planner.p1[2] = in->st_pos.z;

	in->param.end_planner.v1[0] = in->spd.x;
	in->param.end_planner.v1[1] = 0;
	in->param.end_planner.v1[2] = lift_spd;

	//%2 ------------------------mid--------------------------
	in->param.end_planner.p2[0] = in->st_pos.x / 2 + in->tar_pos.x / 2;
	in->param.end_planner.p2[1] = in->st_pos.y / 2 + in->tar_pos.y / 2;
	in->param.end_planner.p2[2] = in->st_pos.z / 2 + in->tar_pos.z / 2 + in->param.end_planner.leg_h;//中点高度
	in->param.end_planner.v2[0] = spd_flag[Xr] * in->param.end_planner.max_spd*mid_spd_w;
	in->param.end_planner.v2[1] = 0;
	in->param.end_planner.v2[2] = 0;

	//%3 end
	in->param.end_planner.p3[0] = in->tar_pos.x;
	in->param.end_planner.p3[1] = in->tar_pos.y;
	in->param.end_planner.p3[2] = in->tar_pos.z;
	in->param.end_planner.v3[0] = in->spd.x;
	in->param.end_planner.v3[1] = 0;
	in->param.end_planner.v3[2] = -td_spd;

	flag[0] = 1;
	flag[1] = 1;//SPD  中点速度加速度连续
	flag[2] = 0;
	for (i = 0; i < 3; i++) {
		GenerateTrajectory(
			in->param.end_planner.p1[i],
			in->param.end_planner.v1[i],
			in->param.end_planner.a1[i],
			in->param.end_planner.p2[i],
			in->param.end_planner.v2[i],
			in->param.end_planner.a2[i],
			in->param.end_planner.T12, flag,
			&in->param.end_planner.a12[i],
			&in->param.end_planner.b12[i],
			&in->param.end_planner.g12[i], &cost);
	}

	flag[0] = 1;
	flag[1] = 1;
	flag[2] = 0;
	for (i = 0; i < 3; i++) {
		GenerateTrajectory(
			in->param.end_planner.p2[i],
			in->param.end_planner.v2[i],
			in->param.end_planner.a2[i],
			in->param.end_planner.p3[i],
			in->param.end_planner.v3[i],
			in->param.end_planner.a3[i],
			in->param.end_planner.T23, flag,
			&in->param.end_planner.a23[i],
			&in->param.end_planner.b23[i],
			&in->param.end_planner.g23[i], &cost);
	}
}

int get_swing_jerk_2d_3point(VMC* in, float time_now) {
	float time = time_now;
	if (time_now != 0)
		time = time_now;
	time = LIMIT(time, 0, in->param.end_planner.Tsw);
	float pos, acc, spd, jerk;
	char i = 0;
	for (i = 0; i < 3; i++) {
		if (time <= in->param.end_planner.T12)
			get_trajecotry(
				in->param.end_planner.p1[i],
				in->param.end_planner.v1[i],
				in->param.end_planner.a1[i],
				in->param.end_planner.a12[i],
				in->param.end_planner.b12[i],
				in->param.end_planner.g12[i],
				time,
				&pos, &acc, &spd, &jerk);
		else
			get_trajecotry(
				in->param.end_planner.p2[i],
				in->param.end_planner.v2[i],
				in->param.end_planner.a2[i],
				in->param.end_planner.a23[i],
				in->param.end_planner.b23[i],
				in->param.end_planner.g23[i],
				time - in->param.end_planner.T12,
				&pos, &acc, &spd, &jerk);

		in->param.end_planner.pos_now[i] = pos;
		in->param.end_planner.spd_now[i] = spd;
		in->param.end_planner.acc_now[i] = acc;
	}
	return 1;
}



//---------------------------------------------多项式--------------------------------------
static float c0[4][3], c3[4][3], c4[4][3], c5[4][3], c6[4][3];
float swing_x_pos_weight=0.35;
void cal_curve_from_pos_new(VMC *in, END_POS t_pos, float desire_time)
{
	char id = in->param.id;
	float pos_now[3], pos_tar[3];
	pos_now[Xr] = in->st_pos.x; pos_now[Yr] = in->st_pos.y; pos_now[Zr] = in->st_pos.z;
	pos_tar[Xr] = t_pos.x; pos_tar[Yr] = t_pos.y; pos_tar[Zr] = t_pos.z;
	float t1 = desire_time / 2, t2 = desire_time;
	float p0[3], p1[3], p2[3];
	//----------start
	p0[0] = pos_now[Xr]; p0[1] = pos_now[Yr]; p0[2] = pos_now[Zr];
	//--------------end
	p2[0] = pos_tar[Xr]; p2[1] = pos_tar[Yr]; p2[2] = pos_tar[Zr];//z
	//-------------middle
	float k, b;
	float end_weight = swing_x_pos_weight;
	float att_off = -sind(vmc_all.att_rate_trig[PITr]);

	p1[0] = p0[0] * (1 - end_weight) + p2[0] * end_weight;	//3
	p1[1] = p0[1] * (1 - end_weight) + p2[1] * end_weight;	//3

	p1[2] = p0[2] + in->delta_ht;
	p1[2] = LIMIT(p1[2], MAX_Z, MIN_Z*1.05);
	float p1_p0[3], p0_p2[3];
	int i;
	for (i = 0; i < 3; i++)
	{
		c0[id][i] = p0[i]; p1_p0[i] = p1[i] - p0[i]; p0_p2[i] = p0[i] - p2[i];
	}
	float t1_3 = powf(t1, 3), t1_4 = powf(t1, 4), t1_5 = powf(t1, 5), t1_6 = powf(t1, 6);
	float t2_2 = powf(t2, 2), t2_3 = powf(t2, 3), t2_5 = powf(t2, 5), t2_6 = powf(t2, 6);
	float temp1 = 0; temp1 = 1 / (t1_3*powf((t1 - t2), 3)*t2_3);
	for (i = 0; i < 3; i++) {
		c3[id][i] = -1 * temp1*(t2_6*(p1_p0[i]) + 5 * t1_4*t2_2 * 3 * (p0_p2[i]) + 2 * t1_6 * 5 * (p0_p2[i]) - 3 * t1_5*t2 * 8 * (p0_p2[i]));
		c4[id][i] = temp1 / t2 * (3 * t2_6*(p1_p0[i]) + 15 * t1_3*t2_3*(p0_p2[i]) - 27 * t1_5*t2*(p0_p2[i]) + t1_6 * 15 * (p0_p2[i]));
		c5[id][i] = -temp1 / t2_2 * 3 * (t2_6*(p1_p0[i]) + 2 * t1_6*(p0_p2[i]) + t1_3 * t2_3 * 8 * (p0_p2[i]) - t1_4 * t2_2 * 9 * (p0_p2[i]));
		c6[id][i] = temp1 / t2_2 * (t2_5*(p1_p0[i]) + 6 * t1_5*(p0_p2[i]) + 10 * t1_3*t2_2*(p0_p2[i]) - t1_4 * t2 * 15 * (p0_p2[i]));
	}
}

END_POS cal_pos_tar_from_curve(VMC *in, float desire_time, float dt)
{
	char id = in->param.id;
	END_POS epos;
	float cal_curve[3];
	float time_now = LIMIT(in->param.time_trig, 0, desire_time);
	char i;
	for (i = 0; i < 3; i++)
		cal_curve[i] = c0[id][i] + c3[id][i] * powf(time_now, 3) + c4[id][i] * powf(time_now, 4) + c5[id][i] * powf(time_now, 5) + c6[id][i] * powf(time_now, 6);
	epos.x = cal_curve[Xr];
	epos.y = cal_curve[Yr];
	epos.z = cal_curve[Zr];
	return epos;
}

//--------------------------------------5次多项式----------------------------------
void traj_plan5(float T_traj, float P_st, float V_st, float P_ed, float V_ed, float Acc_st, float Acc_ed, float *param) {
	//%时间
	float t0 = 0;
	float t1 = T_traj;
	//%位置和速度（a）
	float q0 = P_st;
	float q1 = P_ed;
	float v0 = V_st;
	float v1 = V_ed;
	float acc0 = Acc_st;
	float acc1 = Acc_ed;
	//%利用公式（1-25）求系数
	float h = q1 - q0;
	float T = t1 - t0;
	param[1 - 1] = q0;
	param[2 - 1] = v0;
	param[3 - 1] = 1.0 / 2 * acc0;
	param[4 - 1] = 1.0 / (2 * T*T*T)*(20 * h - (8 * v1 + 12 * v0)*T + (acc1 - 3 * acc0) / (T*T));
	param[5 - 1] = 1.0 / (2 * T*T*T*T)*((-30 * h + (14 * v1 + 16 * v0)*T) + (3 * acc0 - 2 * acc1) / (T*T));
	param[6 - 1] = 1.0 / (2 * T*T*T*T*T)*(12 * h - 6 * (v1 + v0)*T + (acc1 - acc0) / (T*T));
	//printf("v0=%f v1=%f\n",v0,v1);
}


void get_traj5(float param[6], float t_now, float *q, float *v, float *acc) {
	//%轨迹生成
	float t0 = 0;
	float t = t_now;

	//%位置
	*q = param[1 - 1] + param[2 - 1] * powf((t - t0), 1) + param[3 - 1] * powf((t - t0), 2) + param[4 - 1] * powf((t - t0), 3) + //...
		param[5 - 1] * powf(t - t0, 4) + param[6 - 1] * pow(t - t0, 5);
	//printf("t=%f q=%f %f %f %f %f %f %f\n", t,*q, param[0], param[1], param[2], param[3], param[4], param[5]);
	//%速度
	*v = param[2 - 1] + 2 * param[3 - 1] * powf((t - t0), 1) + 3 * param[4 - 1] * powf((t - t0), 2) + 4 * param[5 - 1] * powf(t - t0, 3) + //...
		5 * param[6 - 1] * powf(t - t0, 4);
	//%加速度
	*acc = 2 * param[3 - 1] + 6 * param[4 - 1] * powf((t - t0), 1) + 12 * param[5 - 1] * powf(t - t0, 2) + 20 * param[6 - 1] * powf(t - t0, 3);
}


int swing_jerk_planner_5point(VMC* in, float lift_spd, float td_spd, float T_sw)
{
	static char init[4];
	char flag[3];
	char i;
	int spd_flag[2] = { 1,1 };
	float cost;
	float spd_av = 0;

	in->param.end_planner.Tsw = T_sw;
	in->param.end_planner.leg_h = in->delta_ht;//抬腿高度

	//%1 start
	in->param.end_planner.p1[0] = in->st_pos.x;
	in->param.end_planner.p1[1] = in->st_pos.y;
	in->param.end_planner.p1[2] = in->st_pos.z;

	in->param.end_planner.v1[0] = in->st_spd.x * 1;
	in->param.end_planner.v1[1] = 0;
	if (lift_spd == 0)
		in->param.end_planner.v1[2] = in->st_spd.z;
	else
		in->param.end_planner.v1[2] = lift_spd;

	//%2 ------------------------mid--------------------------
	in->param.end_planner.p2[0] = in->st_pos.x / 2 + in->tar_pos.x / 2;
	in->param.end_planner.p2[1] = in->st_pos.y / 2 + in->tar_pos.y / 2;
	//in->param.end_planner.p2[2] = in->st_pos.z / 2 + in->tar_pos.z / 2 + in->param.end_planner.leg_h;//中点高度
	in->param.end_planner.p2[2] = in->st_pos.z + in->param.end_planner.leg_h;//中点高度
	in->param.end_planner.v2[0] = 0;
	in->param.end_planner.v2[1] = 0;
	in->param.end_planner.v2[2] = 0;
	//printf("%f %f\n", in->st_spd.x, vmc_all.spd_n.x);
	//%3 end
	in->param.end_planner.p3[0] = in->tar_pos.x;
	in->param.end_planner.p3[1] = in->tar_pos.y;
	in->param.end_planner.p3[2] = in->tar_pos.z;
	in->param.end_planner.v3[0] =  vmc_all.spd_n.x * 1;
	in->param.end_planner.v3[1] = 0;
	in->param.end_planner.v3[2] = -td_spd;

	//Xr
	traj_plan5(in->param.end_planner.Tsw,
		in->param.end_planner.p1[0],
		in->param.end_planner.v1[0],
		in->param.end_planner.p3[0],
		in->param.end_planner.v3[0],
		0,
		0,
		&in->param.end_planner.param_x[0]);
	//printf("%f %f %f\n", in->param.end_planner.p1[2], in->param.end_planner.p2[2], in->param.end_planner.p3[2]);
	//printf("%f %f\n", in->param.end_planner.param_x[0], in->param.end_planner.param_x[1]);
	//Zr
	traj_plan5(in->param.end_planner.Tsw / 2,//1
		in->param.end_planner.p1[2],
		in->param.end_planner.v1[2],
		in->param.end_planner.p2[2],
		0,
		0,
		0,
		&in->param.end_planner.param_z1[0]);
	//printf("%f %f\n", in->param.end_planner.param_z1[0], in->param.end_planner.param_z2[1]);
	traj_plan5(in->param.end_planner.Tsw / 2,//2
		in->param.end_planner.p2[2],
		0,
		in->param.end_planner.p3[2],
		in->param.end_planner.v3[2],
		0,
		0,
		&in->param.end_planner.param_z2[0]);
	return 1;
}

int get_swing_jerk_5point(VMC* in, float time_now) {
	float time = time_now;
	if (time_now != 0)
		time = time_now;
	time = LIMIT(time, 0, in->param.end_planner.Tsw);

	get_traj5(in->param.end_planner.param_x, time,
		&in->param.end_planner.pos_now[0],
		&in->param.end_planner.spd_now[0],
		&in->param.end_planner.acc_now[0]);

	if (time <= in->param.end_planner.Tsw / 2)
		get_traj5(in->param.end_planner.param_z1, time,
			&in->param.end_planner.pos_now[2],
			&in->param.end_planner.spd_now[2],
			&in->param.end_planner.acc_now[2]);
	else
		get_traj5(in->param.end_planner.param_z2, time - in->param.end_planner.Tsw / 2,
			&in->param.end_planner.pos_now[2],
			&in->param.end_planner.spd_now[2],
			&in->param.end_planner.acc_now[2]);
	//if(in->param.id==0)
	//	printf("%d %f %f %f\n", in->param.id, time,in->param.end_planner.pos_now[0], in->param.end_planner.pos_now[2]);
	return 1;
}
