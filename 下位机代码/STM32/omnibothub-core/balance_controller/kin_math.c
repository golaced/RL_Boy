#include "include.h"
#include "locomotion_header.h"
#include "math.h"
#include "eso.h"
#include "gait_math.h"
#if !RUN_WEBOTS
#include "arm_math.h"
#endif
//------------------------------运动学运算库
float FLT_SPD_END = 30;

void espd_to_neg_dq(int id,float dt)
{	
float flt_espd=1;
float gain_espd=1;
float temp[3]={0,0,0};

temp[0]=(-robotwb.Leg[id].jacobi_inv[0]*robotwb.Leg[id].tar_espd_h.x+(-robotwb.Leg[id].jacobi_inv[1]*robotwb.Leg[id].tar_espd_h.z))*gain_espd;
temp[1]=(-robotwb.Leg[id].jacobi_inv[2]*robotwb.Leg[id].tar_espd_h.x+(-robotwb.Leg[id].jacobi_inv[3]*robotwb.Leg[id].tar_espd_h.z))*gain_espd;

robotwb.Leg[id].tar_sita_d[0]=temp[0]*flt_espd+(1-flt_espd)*robotwb.Leg[id].tar_sita_d[0];
robotwb.Leg[id].tar_sita_d[1]=temp[1]*flt_espd+(1-flt_espd)*robotwb.Leg[id].tar_sita_d[1];
}

void force_to_tao(int id,float dt)
{	
float flt_t=1;//滤波系数
float gain_t=-1;//输出符号
float temp[3]={0,0,0};
temp[0]=(robotwb.Leg[id].jacobi[0]*robotwb.Leg[id].tar_force_h.x+robotwb.Leg[id].jacobi[2]*robotwb.Leg[id].tar_force_h.z)*gain_t;
temp[1]=(robotwb.Leg[id].jacobi[1]*robotwb.Leg[id].tar_force_h.x+robotwb.Leg[id].jacobi[3]*robotwb.Leg[id].tar_force_h.z)*gain_t;
//力梯度？？
robotwb.Leg[id].taod[0]=temp[0]*flt_t+(1-flt_t)*robotwb.Leg[id].taod[0];
robotwb.Leg[id].taod[1]=temp[1]*flt_t+(1-flt_t)*robotwb.Leg[id].taod[1];

robotwb.Leg[id].taod[0]=LIMIT(robotwb.Leg[id].taod[0],-robotwb.Leg[id].limit_tao[0],robotwb.Leg[id].limit_tao[0]);
robotwb.Leg[id].taod[1]=LIMIT(robotwb.Leg[id].taod[1],-robotwb.Leg[id].limit_tao[1],robotwb.Leg[id].limit_tao[1]);
}

void force_to_tao_input(int id,Vect3 force_h, float dt)
{	
float flt_t=1;
float gain_t=-1;
float temp[3]={0,0,0};
temp[0]=(robotwb.Leg[id].jacobi[0]*force_h.x+(robotwb.Leg[id].jacobi[2]*force_h.z))*gain_t;
temp[1]=(robotwb.Leg[id].jacobi[1]*force_h.x+(robotwb.Leg[id].jacobi[3]*force_h.z))*gain_t;

robotwb.Leg[id].taod[0]=temp[0]*flt_t+(1-flt_t)*robotwb.Leg[id].taod[0];
robotwb.Leg[id].taod[1]=temp[1]*flt_t+(1-flt_t)*robotwb.Leg[id].taod[1];

robotwb.Leg[id].taod[0]=LIMIT(robotwb.Leg[id].taod[0],-robotwb.Leg[id].limit_tao[0],robotwb.Leg[id].limit_tao[0]);
robotwb.Leg[id].taod[1]=LIMIT(robotwb.Leg[id].taod[1],-robotwb.Leg[id].limit_tao[1],robotwb.Leg[id].limit_tao[1]);
}

void inv_KI(int id, Vect3 epos_h, float *s0, float *s1)
{
	float temp = 0;
	float x = epos_h.x;
	float z = epos_h.z;
#if KIN_5LINE_IK
	float L1, L2, L3, L4, L5, L6;
	float Alpha10[2];
	float Alpha20[4];
	float d[2], e[2], f[2];
	int i = 0;
	float Xb[2], Yb[2];
	L1 = vmc[id].l1;
	L2 = vmc[id].l2;
	L3 = vmc[id].l2;
	L4 = vmc[id].l1;
	L5 = 0;
	L6 = vmc[id].l3*1;
	float x1, y1, z1;
	x1 = x;
	y1 = -z;
	float a = 2 * x1* L1;
	float b = 2 * y1* L1;
	float c = x1 *x1 + y1*y1 + L1*L1 - (L2 + L6) *(L2 + L6) ;
	float temp1 = sqrtd(a *a + b * b - c * c);
	Alpha10[0] = 2 * atan2f((b + temp1), (a + c + 0.000001));
	Alpha10[1] = 2 * atan2f((b - temp1), (a + c + 0.000001));
	//printf("ac=%f Alpha10[0]=%f %f\n", a + c, Alpha10[0], vmc[id].l3);
	for (i = 0; i < 2; i++) {
		Xb[i] = x1 - L6 * ((x1 - L1 * cos(Alpha10[i])) / (L2 + L6));
		Yb[i] = y1 - L6 * ((y1 - L1 * sin(Alpha10[i])) / (L2 + L6));
	}
	for (i = 0; i < 2; i++) {
		d[i] = 2 * L4*(Xb[i] - L5);
		e[i] = 2 * L4* Yb[i];
	}
	for (i = 0; i < 2; i++) 
		f[i] = powf(Xb[i] - L5,2) + powf(L4,2)  + powf(Yb[i], 2)  - powf(L3, 2);

	//Alpha20[0] = 2 * atanf((e[0] + sqrtd(powf(d[0], 2)  + powf(e[0], 2)   - powf(f[0], 2)  )) / (d[0] + f[0]));
	
	Alpha20[1] = 2 * atan2f((e[0] - sqrtd(powf(d[0], 2)  + powf(e[0], 2)   - powf(f[0], 2))) , (d[0] + f[0] + 0.000001));

	//Alpha20[2] = 2 * atanf((e[1] + sqrtd(powf(d[1], 2)  + powf(e[1], 2)  - powf(f[1],  2)  )) / (d[1] + f[1]));
	//Alpha20[3] = 2 * atanf((e[1] - sqrtd(powf(d[1], 2)  + powf(e[1], 2)  - powf(f[1],  2)  )) / (d[1] + f[1]));

	if ((Alpha10[0]) < 0)
		Alpha10[0] = Alpha10[0] + 2 * PI;
	
	//if( (Alpha10[1]) < 0)
	//	Alpha10[1] = Alpha10[1] + 2 * PI;


	if ((Alpha20[0]) < 0)
		Alpha20[0] = Alpha20[0] + 2 * PI;
	
	if( (Alpha20[1]) < 0)
		Alpha20[1] = Alpha20[1] + 2 * PI;

	//if ((Alpha20[2]) < 0)
	//	Alpha20[2] = Alpha20[2] + 2 * PI;
	
	//if ((Alpha20[3]) < 0)
	//	Alpha20[3] = Alpha20[3] + 2 * PI;
	
	*s0 = To_180_degreesw(Alpha20[1] * 57.3);
	*s1 = (Alpha10[1] * 57.3);
#if RUN_WEBOTS
	float sita = -(atan2f(-z, x + 0.00001)*RAD_TO_DEGw - 90);
	float r = sqrtd(z*z + x * x) + 0.00001;
	temp = LIMIT((r*r + L1w * L1w - L2w * L2w) / (2 * L1w*r), -0.99, 0.99);
	float s1o = 90 - sita - acosf(temp)*RAD_TO_DEGw;
	float s2o = 90 - sita + acosf(temp)*RAD_TO_DEGw;
	float s11 = *s0;
	float s22 = *s1;
	printf("x=%f z=%f 1o=%f 2o=%f 1n=%f 2n=%f\n", x,z,s1o, s2o, s11, s22);
#endif
#else
	float sita=-(atan2f(-z,x+0.00001)*RAD_TO_DEGw-90);
	float r=sqrtd(z*z+x*x)+0.00001;
	temp=LIMIT((r*r+L1w*L1w-L2w*L2w)/(2*L1w*r),-0.99,0.99);
	*s0=90-sita-acosf(temp)*RAD_TO_DEGw;
	*s1=90-sita+acosf(temp)*RAD_TO_DEGw;
#endif
	
}

//正运动学
float k_end_v = 1;
char estimate_end_state_new(VMC *in, float dt)//??????
{
	float alfa = (in->sita2 - in->sita1) / 2;
	float beta = (in->sita2 + in->sita1) / 2;
	float temp;
	float H = 0;
	float L1, L2, L3, L4, L5, L6;
	float A1, B1, C1, lengthAC;
	float Xa, Ya, Xb, Yb, Xc, Yc;
	float theta1, theta2;
	switch (vmc_all.param.leg_dof) {
	case 2:
		if (vmc_all.param.leg_type == LOOP) {
			in->sita3 = 0;
			H = in->l1*cosd(in->sita1) + in->l2*cosd(180 - in->sita2 - in->sita1);
			in->epos.x = -in->l1*sind(in->sita1) + in->l2*sind(180 - in->sita2 - in->sita1);
			in->epos.x *= in->param.invert_knee_epos[Xr];
			in->epos.y = in->l3*cosd(in->sita3) + H * sind(in->sita3);
			in->epos.y *= in->param.invert_knee_epos[Yr];
			in->epos.z = -H * cosd(in->sita3) + in->l3*sind(in->sita3);

			in->r = sqrt(in->epos.x*in->epos.x + in->epos.y*in->epos.y + in->epos.z*in->epos.z);
			in->sita = -fast_atan2(-in->epos.z, in->epos.x + 0.00001)*RAD_TO_DEG - 90;
		}
		else if (vmc_all.param.leg_type == PARALE_LOOP) {
			in->epos.x = -in->l1*sind(in->sita1) - in->l2*sind(in->sita1 + in->sita2);
			temp = (in->l3 + in->l1*cosd(in->sita1) + in->l2*cosd(in->sita1 + in->sita2));
			in->epos.x *= in->param.invert_knee_epos[Xr];
			in->epos.y = sind(in->sita3)*temp;
			in->epos.z = -cosd(in->sita3)*temp;
			in->r = sqrt(in->epos.x*in->epos.x + in->epos.y*in->epos.y + in->epos.z*in->epos.z);
			in->sita = -fast_atan2(-in->epos.z, in->epos.x + 0.00001)*RAD_TO_DEG - 90;
		}
		else {
#if KIN_5LINE_FK
			L1 = in->l1;
			L2 = in->l2;
			L3 = in->l2;
			L4 = in->l1;
			L5 = 0;
			L6 = in->l3;
			Xa = L1*cosd(in->sita2);
			Ya = L1*sind(in->sita2);
			Xc = L5 + L4*cosd(in->sita1);
			Yc = L4*sind(in->sita1);
			lengthAC = sqrtd((Xc - Xa)*(Xc - Xa) + (Yc - Ya)*(Yc - Ya));
			A1 = 2 * L2*(Xc - Xa);
			B1 = 2 * L2*(Yc - Ya);
			C1 = L2*L2 + lengthAC * lengthAC - L3*L3;
			temp = sqrtd(A1*A1 + B1 * B1 - C1 * C1);
			theta1 = 2 * atan2f((B1 + temp), (A1 + C1))*57.3;
			theta2 = 2 * atan2f((B1 - temp), (A1 + C1))*57.3;
			Xb = Xa + (L2)*cosd(theta1);
			Yb = Ya + (L2)*sind(theta1);
			in->epos.x = Xb;
			in->epos.y = 0;
			in->epos.z = -Yb;

			in->r = sqrtd(in->epos.x*in->epos.x+ in->epos.z*in->epos.z);
			//Vect3 temp_pos;
			//temp_pos.x = in->epos.x; temp_pos.y = in->epos.y; temp_pos.z = in->epos.z;

			//inv_KI(in->param.id, temp_pos, &theta1, &theta2);
			//printf("%f %f  %f  %f\n", in->sita1, in->sita2, theta1, theta2);
			//in->sita = beta;
			//printf("%f %f %f   %f\n", in->sita1, in->sita2,  in->epos.x, in->epos.z);
#else
			alfa = (in->sita2 - in->sita1) / 2;
			beta = (in->sita2 + in->sita1) / 2;
			in->r = in->l1*cosd(alfa) + sqrtd(pow(in->l2, 2) - pow(in->l1, 2)*pow(sind(alfa), 2));
			in->sita = beta;
			in->epos.x = cosd(in->sita)*in->r;
			in->epos.y = 0;
			in->epos.z = -sind(in->sita)*in->r;
#endif
		}
		break;
	case 3://MIT????
		/*
		h=l(2)*cosd(s(2))+l(3)*cosd(180-s(3)-s(2));
		x=-l(2)*sind(s(2))+l(3)*sind(180-s(3)-s(2));
		y=l(1)*cosd(s(1))+h*sind(s(1));
		z=-h*cosd(s(1))+l(1)*sind(s(1));
		end_pos_m=[x;y;z]
		*/
		H = in->l1*cosd(in->sita1) + in->l2*cosd(180 - in->sita2 - in->sita1);
		in->epos.x = -in->l1*sind(in->sita1) + in->l2*sind(180 - in->sita2 - in->sita1);
		in->epos.x *= in->param.invert_knee_epos[Xr];
		in->epos.y = in->l3*cosd(in->sita3) + H * sind(in->sita3);
		in->epos.y *= in->param.invert_knee_epos[Yr];
		in->epos.z = -H * cosd(in->sita3) + in->l3*sind(in->sita3);

		in->r = sqrt(in->epos.x*in->epos.x + in->epos.y*in->epos.y + in->epos.z*in->epos.z);
		in->sita = -fast_atan2(-in->epos.z, in->epos.x + 0.00001)*RAD_TO_DEG - 90;
		break;
	}

	in->epos_b.x = in->epos.x + in->flag_fb* vmc_all.H / 2;
	in->epos_b.y = in->epos.y + in->flag_rl* vmc_all.W / 2;
	in->epos_b.z = in->epos.z;

	in->epos_n.x = vmc_all.Rb_n[0][0] * in->epos_b.x + vmc_all.Rb_n[0][1] * in->epos_b.y + vmc_all.Rb_n[0][2] * in->epos_b.z;
	in->epos_n.y = vmc_all.Rb_n[1][0] * in->epos_b.x + vmc_all.Rb_n[1][1] * in->epos_b.y + vmc_all.Rb_n[1][2] * in->epos_b.z;
	in->epos_n.z = vmc_all.Rb_n[2][0] * in->epos_b.x + vmc_all.Rb_n[2][1] * in->epos_b.y + vmc_all.Rb_n[2][2] * in->epos_b.z;

	float d_sita = 0;
	float d_alfa = 0;
	float d_sita1, d_sita2, d_sita3;
	float d_x, d_y, d_z;
	float dr_dt;
	in->param.spd_est_cnt += dt;
	//--------------------------末端速度-------------------------
	if (in->param.spd_est_cnt >= vmc_all.param.end_sample_dt) {
		switch (vmc_all.param.leg_dof) {
		case 2:
			if (vmc_all.param.leg_type == LOOP || vmc_all.param.leg_type == PARALE_LOOP || END_SPD_USE_POS) {
				d_x = (in->epos.x - in->epos_reg.x) / in->param.spd_est_cnt;
				d_y = (in->epos.y - in->epos_reg.y) / in->param.spd_est_cnt;
				d_z = (in->epos.z - in->epos_reg.z) / in->param.spd_est_cnt;
			}
			else {
				d_sita = (beta - in->sita_reg[0]) / in->param.spd_est_cnt / 57.3;
				d_alfa = (alfa - in->sita_reg[1]) / in->param.spd_est_cnt / 57.3;
				temp = fast_sqrt(pow(in->l2, 2) - pow(in->l1, 2)*pow(sind(alfa), 2));
				dr_dt = (-in->l1*sind(alfa)*d_alfa - (pow(in->l1, 2)*sind(alfa)*cosd(alfa)*d_alfa) / (temp + 0.000001));
				d_x = -sind(in->sita)*d_sita*in->r + cosd(in->sita)*dr_dt;
				d_z = -cosd(in->sita)*d_sita*in->r - sind(in->sita)*dr_dt;
				d_y = 0;
				in->sita_reg[0] = beta;
				in->sita_reg[1] = alfa;
			}
			break;
		case 3:
#if END_SPD_USE_POS
			d_x = (in->epos.x - in->epos_reg.x) / in->param.spd_est_cnt;
			d_y = (in->epos.y - in->epos_reg.y) / in->param.spd_est_cnt;
			d_z = (in->epos.z - in->epos_reg.z) / in->param.spd_est_cnt;
#else
			d_sita1 = (in->sita1 - in->sita_reg[0]) / in->param.spd_est_cnt / 57.3*k_end_v;
			d_sita2 = (in->sita2 - in->sita_reg[1]) / in->param.spd_est_cnt / 57.3*k_end_v;
			d_sita3 = (in->sita3 - in->sita_reg[2]) / in->param.spd_est_cnt / 57.3*k_end_v;

			H = -in->l1*sind(in->sita1)*d_sita1 - in->l2*sind(180 - in->sita2 - in->sita1)*(-d_sita2 - d_sita1);
			d_x = -in->l1*cosd(in->sita1)*d_sita1 + in->l2*cosd(180 - in->sita2 - in->sita1)*(-d_sita2 - d_sita1);
			d_x *= in->param.invert_knee_epos[Xr];
			d_y = -in->l3*sind(in->sita3)*d_sita3 + H * cosd(in->sita3)*d_sita3;
			d_y *= in->param.invert_knee_epos[Yr];
			d_z = H * sind(in->sita3)*d_sita3 + in->l3*cosd(in->sita3)*d_sita3;
#endif
			in->sita_reg[0] = in->sita1;
			in->sita_reg[1] = in->sita2;
			in->sita_reg[2] = in->sita3;
			break;
		}
		if (in->ground) {
			in->spd_o.x = -LIMIT(d_x, -2, 2);
			in->spd_o.y = -LIMIT(d_y, -2, 2);
			in->spd_o.z =  LIMIT(d_z, -2, 2);
		}
		in->epos_reg.x = in->epos.x;
		in->epos_reg.y = in->epos.y;
		in->epos_reg.z = in->epos.z;
		in->param.spd_est_cnt = 0;
	}
	DigitalLPF(in->spd_o.x, &in->spd.x, FLT_SPD_END, dt);
	DigitalLPF(in->spd_o.y, &in->spd.y, FLT_SPD_END, dt);
	DigitalLPF(in->spd_o.z, &in->spd.z, FLT_SPD_END, dt);

	in->spd_b.x = in->spd.x;
	in->spd_b.y = in->spd.y;
	in->spd_b.z = in->spd.z;

	in->spd_n.x = vmc_all.Rb_n[0][0] * in->spd_b.x + vmc_all.Rb_n[0][1] * in->spd_b.y + vmc_all.Rb_n[0][2] * in->spd_b.z;
	in->spd_n.y = vmc_all.Rb_n[1][0] * in->spd_b.x + vmc_all.Rb_n[1][1] * in->spd_b.y + vmc_all.Rb_n[1][2] * in->spd_b.z;
	in->spd_n.z = vmc_all.Rb_n[2][0] * in->spd_b.x + vmc_all.Rb_n[2][1] * in->spd_b.y + vmc_all.Rb_n[2][2] * in->spd_b.z;
	return 0;
}


char inv_end_state(VMC *vmc,float x,float y,float z,float *sita1,float *sita2,float *sita3)//unuse
{
	float end_posl[4],end_pos[4],temp[10];
    float sita=-(fast_atan2(-z,x+0.00001)*RAD_TO_DEG-90);
	float r=fast_sqrt(z*z+x*x)+0.00001;
	float Rr,Rr1;
	Vect3 epos_h;
	switch(vmc_all.param.leg_dof)
	{
	case 2:
		if(vmc_all.param.leg_type==LOOP){
		end_pos[Xr]=vmc->param.invert_knee_epos[Xr]*x;
		end_pos[Yr]=0;//vmc->param.invert_knee_epos[Yr]*y;
		end_pos[Zr]=z;
		Rr= end_pos[Xr]*end_pos[Xr] + end_pos[Yr]*end_pos[Yr]+end_pos[Zr]*end_pos[Zr];
		Rr1= end_pos[Yr]*end_pos[Yr] + end_pos[Zr]*end_pos[Zr];
		temp[2]=sqrtd(LIMIT(- vmc->l3*vmc->l3 + Rr1,0,99));
		*sita3=90 - fast_atan2(fabs(end_pos[Zr]), end_pos[Yr]+0.000001)*RAD_TO_DEG - 
		fast_atan2(vmc->l3, temp[2]+0.000001)*RAD_TO_DEG;
		*sita3=0;
		/*
		h1=sqrtd(z*z+y*y-l(1)*l(1));
		sita=atan2(x,h1)*57.3;
		r=(h1^2+x^2)^0.5;
		s21=180+sita-acosf((r*r+l(2)^2-l(3)^2)/(2*l(2)*r))*57.3;
		sita2=180-s21;*/
		temp[0]=sqrtd(LIMIT(Rr1 - vmc->l3* vmc->l3,0,99));//h1
		temp[1]=sqrtd(LIMIT(temp[0]*temp[0]+end_pos[Xr]*end_pos[Xr],0,99));//r
		temp[2]=atan2(end_pos[Xr], temp[0]+0.000001)*RAD_TO_DEG;//sita
		temp[3]=(temp[1]*temp[1]+ vmc->l1*vmc->l1 - vmc->l2*vmc->l2)/(2*vmc->l1*temp[1]+0.000001);
		temp[3]=LIMIT(temp[3],-1,1);
		*sita1=180-(180+temp[2]-acosf(temp[3]) *RAD_TO_DEG);
		
		temp[0]= (vmc->l1*vmc->l1 + vmc->l2*vmc->l2 + vmc->l3*vmc->l3 - Rr)/(2*vmc->l1*vmc->l2);
		temp[0]=LIMIT(temp[0],-1,1);
		*sita2=acosf(temp[0])*RAD_TO_DEG;
		}else if(vmc_all.param.leg_type==PARALE_LOOP){
			*sita3=temp[2]=0;
			end_pos[Xr]=vmc->param.invert_knee_epos[Xr]*x;
			end_pos[Yr]=0;//y*cosd(temp[2])+z*sind(temp[2]);
			end_pos[Zr]=-y*sind(temp[2])+z*cosd(temp[2])+vmc_all.l3;
			sita=-fast_atan2(-end_pos[Zr],end_pos[Xr]+0.00001)*RAD_TO_DEG-90;
			r=sqrtd(end_pos[Zr]*end_pos[Zr]+end_pos[Xr]*end_pos[Xr]);
			//temp[0]=180+sita-acosf((r*r+l(2)^2-l(3)^2)/(2*l(2)*r))*RAD_TO_DEG;
			temp[2]=LIMIT((r*r+vmc_all.l1*vmc_all.l1-vmc_all.l2*vmc_all.l2)/(2*vmc_all.l1*r),-1,1);
			temp[1]=180+sita+acosf(temp[2])*RAD_TO_DEG;
			*sita1=-temp[1];//锟斤拷锟斤拷
			temp[2]=LIMIT((vmc_all.l1*vmc_all.l1+vmc_all.l2*vmc_all.l2-r*r)/(2*vmc_all.l1*vmc_all.l2),-1,1);
			*sita2=180-acosf(temp[2])*RAD_TO_DEG;//小锟斤拷
		}else{
#if 0
			//printf("ss\n");
			temp[0]=LIMIT((r*r+vmc_all.l1*vmc_all.l1-vmc_all.l2*vmc_all.l2)/(2*vmc_all.l1*r),-1,1);
			*sita1=90-sita-acosf(temp[0])*RAD_TO_DEG;
			*sita2=90-sita+acosf(temp[0])*RAD_TO_DEG;
#else
			epos_h.x = x; epos_h.y = 0; epos_h.z = z;
			inv_KI(vmc->param.id, epos_h, sita1, sita2);
#endif
		}
	break;
	case 3:
	/*
	pi/2 - atan2(abs(Z), Y) - atan2(in->l3, (- in->l3^2 + Y^2 + Z^2)^(1/2))
	acosf((in->l1^2 - in->l2^2 - in->l3^2 + X^2 + Y^2 + Z^2)/(2*in->l1*(- in->l3^2 + X^2 + Y^2 + Z^2)^(1/2))) 
		- atan2(X, (- in->l3^2 + Y^2 + Z^2)^(1/2))
	acosf((in->l1^2 + in->l2^2 + in->l3^2 - X^2 - Y^2 - Z^2)/(2*in->l1*in->l2))
		*/
		end_pos[Xr]=vmc->param.invert_knee_epos[Xr]*x;
		end_pos[Yr]=vmc->param.invert_knee_epos[Yr]*y;
		end_pos[Zr]=z;
		Rr= end_pos[Xr]*end_pos[Xr] + end_pos[Yr]*end_pos[Yr]+end_pos[Zr]*end_pos[Zr];
		Rr1= end_pos[Yr]*end_pos[Yr] + end_pos[Zr]*end_pos[Zr];
		temp[2]=sqrtd(LIMIT(- vmc->l3*vmc->l3 + Rr1,0,99));
		*sita3=90 - fast_atan2(fabs(end_pos[Zr]), end_pos[Yr]+0.000001)*RAD_TO_DEG - 
		fast_atan2(vmc->l3, temp[2]+0.000001)*RAD_TO_DEG;
		/*
		h1=sqrtd(z*z+y*y-l(1)*l(1));
		sita=atan2(x,h1)*57.3;
		r=(h1^2+x^2)^0.5;
		s21=180+sita-acosf((r*r+l(2)^2-l(3)^2)/(2*l(2)*r))*57.3;
		sita2=180-s21;*/
		temp[0]=sqrtd(LIMIT(Rr1 - vmc->l3* vmc->l3,0,99));//h1
		temp[1]=sqrtd(LIMIT(temp[0]*temp[0]+end_pos[Xr]*end_pos[Xr],0,99));//r
		temp[2]=atan2(end_pos[Xr], temp[0]+0.000001)*RAD_TO_DEG;//sita
		temp[3]=(temp[1]*temp[1]+ vmc->l1*vmc->l1 - vmc->l2*vmc->l2)/(2*vmc->l1*temp[1]+0.000001);
		temp[3]=LIMIT(temp[3],-1,1);
		*sita1=180-(180+temp[2]-acosf(temp[3]) *RAD_TO_DEG);
		
		temp[0]= (vmc->l1*vmc->l1 + vmc->l2*vmc->l2 + vmc->l3*vmc->l3 - Rr)/(2*vmc->l1*vmc->l2);
		temp[0]=LIMIT(temp[0],-1,1);
		*sita2=acosf(temp[0])*RAD_TO_DEG;
	break;
 }
	return 1;
}

char inv_end_state_new(VMC *vmc,float x,float y,float z,float *sita1,float *sita2,float *sita3)
{
	int i=vmc->param.id;
	robotwb.Leg[i].tar_epos_h.x=x;
	robotwb.Leg[i].tar_epos_h.y=y;
	robotwb.Leg[i].tar_epos_h.z=z;
	inv_KI(i,robotwb.Leg[i].tar_epos_h,sita1,sita2);
	//printf("%f %f\n",robotwb.Leg[i].tar_sita[0],robotwb.Leg[i].tar_sita[1]);
	return 1;
}

char cal_invjacobi(VMC *in)
{
	float det;
  //arm加速
	char i=0;
	#if !RUN_WEBOTS
	arm_status sta;  
	/****浮点数数组******************************************************************/
	float32_t pDataB[4];
	float32_t pDataA[4] = {
							1.0f,   0.0f,   
							0.0f,   1.0f};
				
	arm_matrix_instance_f32 pSrcA;
	arm_matrix_instance_f32 pSrcB; 
	pDataA[0]=in->jacobi22[0];
	pDataA[1]=in->jacobi22[1];
	pDataA[2]=in->jacobi22[2];
	pDataA[3]=in->jacobi22[3];							
	/****浮点数***********************************************************************/
	pSrcA.numCols = 2;
	pSrcA.numRows = 2;
	pSrcA.pData = pDataA;
	
	pSrcB.numCols = 2;
	pSrcB.numRows = 2;
	pSrcB.pData = pDataB;
	
	sta = arm_mat_inverse_f32(&pSrcA, &pSrcB);
	
	in->ijacobi22[0]=pDataB[0];
	in->ijacobi22[1]=pDataB[1];
	in->ijacobi22[2]=pDataB[2];
	in->ijacobi22[3]=pDataB[3];
	#else
	invet22(in->jacobi22,&det,in->ijacobi22);
	if(fabs(det)<0.00001)
		return 0;
	#endif
	return 1;
}

char cal_jacobi_new(VMC *in)
{ 

#if KIN_5LINE_J
	double d;
	double d1;
	double d2;
	double d3;
	double a_tmp_tmp;
	double a_tmp;
	double b_a_tmp_tmp;
	double b_a_tmp;
	double c_a_tmp_tmp;
	double d_a_tmp_tmp;
	double c_a_tmp;
	double d_a_tmp;
	double e_a_tmp_tmp;
	double e_a_tmp;
	double f_a_tmp;
	double a;
	double Jcobi_tmp;
	double b_Jcobi_tmp;
	float Alpha1 = in->sita1 / 57.3;
	float Alpha2 = in->sita2 / 57.3;
	float Param[6] = { in->l1 ,in->l2,in->l2,in->l1 ,0,in->l3 };
	float Jcobi[4];
	float ss1= sinf(Alpha1);
	float cs1= cosf(Alpha1);
	float ss2 = sinf(Alpha2);
	float cs2 = cosf(Alpha2);
	d = ss1;
	d1 = cs1;
	d2 = ss2;
	d3 = cs2;
	a_tmp_tmp = Param[0] * d3;
	a_tmp = Param[0] * d1 - a_tmp_tmp;
	b_a_tmp_tmp = Param[0] * d2;
	b_a_tmp = Param[0] * d - b_a_tmp_tmp;
	c_a_tmp_tmp = a_tmp * a_tmp;
	d_a_tmp_tmp = b_a_tmp * b_a_tmp;
	c_a_tmp = c_a_tmp_tmp + d_a_tmp_tmp;
	d_a_tmp = c_a_tmp + 2.0 * Param[1] * a_tmp;
	e_a_tmp_tmp = Param[1] * Param[1];
	e_a_tmp = 4.0 * e_a_tmp_tmp;
	f_a_tmp = 2.0 * Param[1] * b_a_tmp;
	e_a_tmp = e_a_tmp * c_a_tmp_tmp + e_a_tmp * d_a_tmp_tmp;
	a = sqrt(e_a_tmp - c_a_tmp * c_a_tmp) + f_a_tmp;
	c_a_tmp_tmp = 8.0 * Param[0] * e_a_tmp_tmp;
	d_a_tmp_tmp = 2.0 * Param[0] * Param[1];
	e_a_tmp_tmp = c_a_tmp_tmp * d * a_tmp;
	Jcobi_tmp = c_a_tmp_tmp * d1 * b_a_tmp;
	b_Jcobi_tmp = Param[1] + Param[5];

	Jcobi[0] = -(2.0 * sinf(2.0 * atanf((sqrtd(e_a_tmp - c_a_tmp * c_a_tmp) + f_a_tmp)
		/ d_a_tmp)) * ((((2.0 * (2.0 * Param[0] * d * a_tmp - 2.0 * Param[0] * d1 *
			b_a_tmp) * c_a_tmp - e_a_tmp_tmp) + Jcobi_tmp) / (2.0 * sqrtd(e_a_tmp -
				c_a_tmp * c_a_tmp)) + d_a_tmp_tmp * d1) / d_a_tmp + (sqrtd(e_a_tmp - c_a_tmp *
					c_a_tmp) + f_a_tmp) * ((2.0 * Param[0] * ss1 * (Param[0] * cs1 -Param[0] * cs2) - 2.0 * Param[0] * cs1 * (Param[0]
						* ss1 - Param[0] *ss2)) + d_a_tmp_tmp * d) / (d_a_tmp *
							d_a_tmp)) * b_Jcobi_tmp) / (a * a / (d_a_tmp * d_a_tmp) + 1.0);
	a = sqrtd(e_a_tmp - c_a_tmp * c_a_tmp) + f_a_tmp;
	Jcobi[1] = -(2.0 * cosf(2.0 * atanf((sqrtd(e_a_tmp - c_a_tmp * c_a_tmp) + f_a_tmp)
		/ d_a_tmp)) * ((((2.0 * (2.0 * Param[0] * ss1 * (Param[0] * cs1 -Param[0] * cs2) - 2.0 * Param[0] * cs1 * (Param[0]
			* ss1 - Param[0] * ss2)) * (a_tmp * a_tmp + b_a_tmp *
				b_a_tmp) - e_a_tmp_tmp) + Jcobi_tmp) / (2.0 * sqrtd(e_a_tmp - c_a_tmp *
					c_a_tmp)) + 2.0 * Param[0] * Param[1] * cs1) / d_a_tmp + (sqrtd
					(e_a_tmp - c_a_tmp * c_a_tmp) + f_a_tmp) * ((2.0 * Param[0] * ss1 *
						(Param[0] * cs1 - Param[0] * cs2) - 2.0 * Param[0] * cs1 * (Param[0] * ss1- Param[0] * ss2)) + 2.0 * Param
						[0] * Param[1] * ss1) / (d_a_tmp * d_a_tmp)) * b_Jcobi_tmp) / (a * a
							/ (d_a_tmp * d_a_tmp) + 1.0);
	//a = sqrtd(e_a_tmp - c_a_tmp * c_a_tmp) + f_a_tmp;
	e_a_tmp_tmp = c_a_tmp_tmp * d2 * a_tmp;
	c_a_tmp_tmp = c_a_tmp_tmp * d3 * b_a_tmp;
	Jcobi[2] = 2.0 * sin(2.0 * atanf((sqrtd(e_a_tmp - c_a_tmp * c_a_tmp) + f_a_tmp) /
		d_a_tmp)) * ((((2.0 * (2.0 * Param[0] * d2 * a_tmp - 2.0 * Param[0] * d3 *
			b_a_tmp) * c_a_tmp - e_a_tmp_tmp) + c_a_tmp_tmp) / (2.0 * sqrtd(e_a_tmp -
				c_a_tmp * c_a_tmp)) + d_a_tmp_tmp * d3) / d_a_tmp + (sqrtd(e_a_tmp - c_a_tmp *
					c_a_tmp) + f_a_tmp) * ((2.0 * Param[0] * ss2 * (Param[0] * cs1 -Param[0] * cs2) - 2.0 * Param[0] * cs2* (Param[0]
						* ss1 - Param[0] * ss2)) + d_a_tmp_tmp * d2) / (d_a_tmp *
							d_a_tmp)) * b_Jcobi_tmp / (a * a / (d_a_tmp * d_a_tmp) + 1.0) - b_a_tmp_tmp;
	//a = sqrtd(e_a_tmp - c_a_tmp * c_a_tmp) + f_a_tmp;
	Jcobi[3] = 2.0 * cosf(2.0 * atanf((sqrtd(e_a_tmp - c_a_tmp * c_a_tmp) + f_a_tmp) /
		d_a_tmp)) * ((((2.0 * (2.0 * Param[0] * ss2 * (Param[0] * cs1
			- Param[0] * cs2) - 2.0 * Param[0] * cs2 * (Param[0] * ss1 -Param[0] * ss2)) * (a_tmp * a_tmp + b_a_tmp * b_a_tmp) -
			e_a_tmp_tmp) + c_a_tmp_tmp) / (2.0 * sqrtd(e_a_tmp - c_a_tmp *
				c_a_tmp)) + 2.0 * Param[0] * Param[1] * cs2) / d_a_tmp + (sqrtd
				(e_a_tmp - c_a_tmp * c_a_tmp) + f_a_tmp) * ((2.0 * Param[0] * ss2 *
					(Param[0] * cs1 - Param[0] * cs2) - 2.0 * Param[0] * cs2 * (Param[0] * ss1 - Param[0] * ss2)) + 2.0 * Param
					[0] * Param[1] * ss2) / (d_a_tmp * d_a_tmp)) * b_Jcobi_tmp / (a * a /
					(d_a_tmp * d_a_tmp) + 1.0) - a_tmp_tmp;
	in->jacobi22[0] = Jcobi[0];
	in->jacobi22[2] = Jcobi[1];

	in->jacobi22[1] = Jcobi[2];
	in->jacobi22[3] = Jcobi[3];
#else
	float temp1, temp2, temp3;
	float alfa = (in->sita2 - in->sita1) / 2;
	float beta = (in->sita2 + in->sita1) / 2;
	float det;
	float h, h_s1, h_s2, h_s3;
	float r = sqrtd(in->epos.x*in->epos.x + in->epos.z*in->epos.z);
	float rr = sqrtd(in->l2*in->l2 - in->l1*in->l1*powf(sind(alfa), 2)) + 0.00001;
	float r_s0 = in->l1*sind(alfa)*0.5 + 0.5*in->l1*in->l1*sind(alfa)*cosd(alfa) / rr;
	float r_s1 = -r_s0;
	in->jacobi22[0]=-sind(beta)*r*0.5+cosd(beta)*r_s0;
	in->jacobi22[2]=-cosd(beta)*r*0.5-sind(beta)*r_s0;

	in->jacobi22[1]=-sind(beta)*r*0.5+cosd(beta)*r_s1;
	in->jacobi22[3]=-cosd(beta)*r*0.5-sind(beta)*r_s1;
#endif
	return 1;
}

