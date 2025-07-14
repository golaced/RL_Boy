#include "include.h"
#include "locomotion_header.h"
#include "math.h"
#include "eso.h"
#include "gait_math.h"
#if !RUN_WEBOTS
#include "arm_math.h"
#endif
//----------------------×ø±ê×ª»»¿â
void mat_trans(float src[3][3],float dis[3][3])
{
	char i,j;
	float k;
	for(i = 0;i < 3;i++){
  for(j = 0; j < 3 ; j++){
   dis[i][j] = src[j][i];
  }
 }
}

void converV_n_to_bw(Vect3 vn,Vect3* vb){
	float temp=0;
	vb->x  = robotwb.Rn_b[0][0]*vn.x + robotwb.Rn_b[0][1]*vn.y + robotwb.Rn_b[0][2]*vn.z;
	vb->y  = robotwb.Rn_b[1][0]*vn.x + robotwb.Rn_b[1][1]*vn.y + robotwb.Rn_b[1][2]*vn.z;
	temp   = robotwb.Rn_b[2][0]*vn.x + robotwb.Rn_b[2][1]*vn.y + robotwb.Rn_b[2][2]*vn.z;
	#if F_CONTROL_WITH_ROLL
	vb->z  = temp/cosd(LIMIT(robotwb.now_att.roll,-ROLL_LIMIT_COM,ROLL_LIMIT_COM));
	#else
	vb->z  = temp ;
	#endif
}

void converV_n_to_bw_noroll(Vect3 vn,Vect3* vb){
	float temp=0;
	vb->x  = robotwb.Rn_b_noroll[0][0]*vn.x + robotwb.Rn_b_noroll[0][1]*vn.y + robotwb.Rn_b_noroll[0][2]*vn.z;
	vb->y  = robotwb.Rn_b_noroll[1][0]*vn.x + robotwb.Rn_b_noroll[1][1]*vn.y + robotwb.Rn_b_noroll[1][2]*vn.z;
	temp   = robotwb.Rn_b_noroll[2][0]*vn.x + robotwb.Rn_b_noroll[2][1]*vn.y + robotwb.Rn_b_noroll[2][2]*vn.z;
	vb->z  = temp ;
}

void converV_b_to_legw(int id,Vect3 vb,Vect3* vl){
    vl->x = vb.x- Hw/2*robotwb.Leg[id].flag_fb;
    vl->y = vb.y- Www/2*robotwb.Leg[id].flag_rl;
    vl->z = vb.z;
}

void converV_b_to_leg_ow(char leg,float xb,float yb,float zb,float *xl,float *yl,float *zl)
{
	*xl = xb- Hw/2*robotwb.Leg[leg].flag_fb;
	*yl = yb- Www/2*robotwb.Leg[leg].flag_rl;
	*zl = zb;
}

void converV_leg_to_bw(int id,Vect3 vl,Vect3* vb){
    vb->x = vl.x+ Hw/2*robotwb.Leg[id].flag_fb;
    vb->y = vl.y+ Www/2*robotwb.Leg[id].flag_rl;
    vb->z = vl.z;
}

void converV_b_to_nw(Vect3 vb,Vect3* vn){
    vn->x =  robotwb.Rb_n[0][0]*vb.x +  robotwb.Rb_n[0][1]*vb.y +  robotwb.Rb_n[0][2]*vb.z;
    vn->y =  robotwb.Rb_n[1][0]*vb.x +  robotwb.Rb_n[1][1]*vb.y +  robotwb.Rb_n[1][2]*vb.z;
    vn->z =  robotwb.Rb_n[2][0]*vb.x +  robotwb.Rb_n[2][1]*vb.y +  robotwb.Rb_n[2][2]*vb.z;
}

void force_n_to_bw(Vect3 fn,Vect3* fb){
	float temp=0;
	fb->x =  robotwb.Rn_b[0][0]*fn.x + robotwb.Rn_b[0][1]*fn.y + robotwb.Rn_b[0][2]*fn.z;
	fb->y =  robotwb.Rn_b[1][0]*fn.x + robotwb.Rn_b[1][1]*fn.y + robotwb.Rn_b[1][2]*fn.z;
	temp  =  robotwb.Rn_b[2][0]*fn.x + robotwb.Rn_b[2][1]*fn.y + robotwb.Rn_b[2][2]*fn.z;
	#if F_CONTROL_WITH_ROLL
	fb->z  = temp/cosd(LIMIT(robotwb.now_att.roll,-ROLL_LIMIT_COM,ROLL_LIMIT_COM));
	#else
	fb->z  = temp ;
	#endif
}

void force_n_to_bw_noroll(Vect3 fn,Vect3* fb){
	float temp=0;
	fb->x =  robotwb.Rn_b_noroll[0][0]*fn.x + robotwb.Rn_b_noroll[0][1]*fn.y + robotwb.Rn_b_noroll[0][2]*fn.z;
	fb->y =  robotwb.Rn_b_noroll[1][0]*fn.x + robotwb.Rn_b_noroll[1][1]*fn.y + robotwb.Rn_b_noroll[1][2]*fn.z;
	temp  =  robotwb.Rn_b_noroll[2][0]*fn.x + robotwb.Rn_b_noroll[2][1]*fn.y + robotwb.Rn_b_noroll[2][2]*fn.z;

	fb->z  = temp ;
}

void converV_b_to_n_RTw(float RT[3][3], float yaw, float xb,float yb,float zb,float *xn,float *yn,float *zn)
{
	float x,y,z;
	x = RT[0][0]*xb + RT[0][1]*yb + RT[0][2]*zb;
	y = RT[1][0]*xb + RT[1][1]*yb + RT[1][2]*zb;
	z = RT[2][0]*xb + RT[2][1]*yb + RT[2][2]*zb;
	
	*xn=x*cosdw(yaw)-y*sindw(yaw);
	*yn=x*sindw(yaw)+y*cosdw(yaw);
	*zn=z;
}

void converV_n_to_b_w_yaWww(float yaw,float xn,float yn,float zn,float *xb,float *yb,float *zb)//unuse
{
	float x,y,z;
	x = robotwb.Rn_b[0][0]*xn + robotwb.Rn_b[0][1]*yn + robotwb.Rn_b[0][2]*zn;
	y = robotwb.Rn_b[1][0]*xn + robotwb.Rn_b[1][1]*yn + robotwb.Rn_b[1][2]*zn;
	z = robotwb.Rn_b[2][0]*xn + robotwb.Rn_b[2][1]*yn + robotwb.Rn_b[2][2]*zn;
	
	*xb=x*cosdw(yaw)-y*sindw(yaw);
	*yb=x*sindw(yaw)+y*cosdw(yaw);
	*zb=z;
}

void converV_n_to_b(float xn,float yn,float zn,float *xb,float *yb,float *zb)
{
	*xb = vmc_all.Rn_b[0][0]*xn + vmc_all.Rn_b[0][1]*yn + vmc_all.Rn_b[0][2]*zn;
	*yb = vmc_all.Rn_b[1][0]*xn + vmc_all.Rn_b[1][1]*yn + vmc_all.Rn_b[1][2]*zn;
	*zb = vmc_all.Rn_b[2][0]*xn + vmc_all.Rn_b[2][1]*yn + vmc_all.Rn_b[2][2]*zn;
}

void converV_n_to_b_noroll(float xn,float yn,float zn,float *xb,float *yb,float *zb)
{
	*xb = vmc_all.Rn_b_noroll[0][0]*xn + vmc_all.Rn_b_noroll[0][1]*yn + vmc_all.Rn_b_noroll[0][2]*zn;
	*yb = vmc_all.Rn_b_noroll[1][0]*xn + vmc_all.Rn_b_noroll[1][1]*yn + vmc_all.Rn_b_noroll[1][2]*zn;
	*zb = vmc_all.Rn_b_noroll[2][0]*xn + vmc_all.Rn_b_noroll[2][1]*yn + vmc_all.Rn_b_noroll[2][2]*zn;
}


void converV_n_to_b_RT(float RT[3][3],float yaw,float xn,float yn,float zn,float *xb,float *yb,float *zb)
{
	float x,y,z;
	x = RT[0][0]*xn + RT[0][1]*yn + RT[0][2]*zn;
	y = RT[1][0]*xn + RT[1][1]*yn + RT[1][2]*zn;
	z = RT[2][0]*xn + RT[2][1]*yn + RT[2][2]*zn;
	
	*xb=x*cosd(yaw)-y*sind(yaw);
	*yb=x*sind(yaw)+y*cosd(yaw);
	*zb=z;
}

void converV_n_to_b_w_yaw(float yaw,float xn,float yn,float zn,float *xb,float *yb,float *zb)
{
	float x,y,z;
	x = vmc_all.Rn_b[0][0]*xn + vmc_all.Rn_b[0][1]*yn + vmc_all.Rn_b[0][2]*zn;
	y = vmc_all.Rn_b[1][0]*xn + vmc_all.Rn_b[1][1]*yn + vmc_all.Rn_b[1][2]*zn;
	z = vmc_all.Rn_b[2][0]*xn + vmc_all.Rn_b[2][1]*yn + vmc_all.Rn_b[2][2]*zn;
	
	*xb=x*cosd(yaw)-y*sind(yaw);
	*yb=x*sind(yaw)+y*cosd(yaw);
	*zb=z;
}

void converV_b_to_leg(char leg,float xb,float yb,float zb,float *xl,float *yl,float *zl)
{
	*xl = xb- vmc_all.H/2*vmc[leg].flag_fb;
	*yl = yb- vmc_all.W/2*vmc[leg].flag_rl;
	*zl = zb;
}

void converV_leg_to_b(char leg,float xl,float yl,float zl,float *xb,float *yb,float *zb)
{
	*xb = xl+ vmc_all.H/2*vmc[leg].flag_fb;
	*yb = yl+ vmc_all.W/2*vmc[leg].flag_rl;
	*zb = zl;
}

void converV_b_to_n(float xb,float yb,float zb,float *xn,float *yn,float *zn)
{
	*xn = vmc_all.Rb_n[0][0]*xb + vmc_all.Rb_n[0][1]*yb + vmc_all.Rb_n[0][2]*zb;
	*yn = vmc_all.Rb_n[1][0]*xb + vmc_all.Rb_n[1][1]*yb + vmc_all.Rb_n[1][2]*zb;
	*zn = vmc_all.Rb_n[2][0]*xb + vmc_all.Rb_n[2][1]*yb + vmc_all.Rb_n[2][2]*zb;
}

void converV_b_to_n_RT(float RT[3][3], float yaw, float xb,float yb,float zb,float *xn,float *yn,float *zn)
{
	float x,y,z;
	x = RT[0][0]*xb + RT[0][1]*yb + RT[0][2]*zb;
	y = RT[1][0]*xb + RT[1][1]*yb + RT[1][2]*zb;
	z = RT[2][0]*xb + RT[2][1]*yb + RT[2][2]*zb;
	
	*xn=x*cosd(yaw)-y*sind(yaw);
	*yn=x*sind(yaw)+y*cosd(yaw);
	*zn=z;
}

//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ïµ×ªï¿½ï¿½ï¿½ï¿½ï¿½Ö²ï¿½
char use_cor_trans=1;
void force_n_to_b(VMC *in)
{
	END_POS force_n;
	force_n.x=in->force_n[Xr];
	force_n.y=in->force_n[Yr];
	force_n.z=in->force_n[Zr];
	//×ªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½  XÇ°ï¿½ï¿½  Yï¿½à·½  Zï¿½Ï·ï¿½
	in->force_b[Xr] = vmc_all.Rn_b[0][0]*force_n.x + vmc_all.Rn_b[0][1]*force_n.y + vmc_all.Rn_b[0][2]*force_n.z;
	in->force_b[Yr] = vmc_all.Rn_b[1][0]*force_n.x + vmc_all.Rn_b[1][1]*force_n.y + vmc_all.Rn_b[1][2]*force_n.z;
	in->force_b[Zr] = vmc_all.Rn_b[2][0]*force_n.x + vmc_all.Rn_b[2][1]*force_n.y + vmc_all.Rn_b[2][2]*force_n.z;

	//no trans
	if(!use_cor_trans){
		in->force_b[Xr] = in->force_n[Xr];
		in->force_b[Yr] = in->force_n[Yr];
		in->force_b[Zr] = in->force_n[Zr];
	}
}