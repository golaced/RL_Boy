#include "include.h"
#include "gait_math.h"
#include "imu.h"
#include "nav.h"
#include "usart_fc.h"
#include "gps.h"
_NAV nav;
//-----------------KF  parameter------------------
double P_kf_pose[2][16]; 
double X_kf_pose[2][4];

double Q_pose[4]={0.001,0.001,0.001,0.001};
double R_pose[3]={0.1,0.001,0.05};
double R_GPS[2]={0.006,0.00052125};
float w_spd[2]={0.6,0.2};//外部里程计置信因子
float k_fack_yaw=0.1;
char position_mode=0;

float navCalcBearing(double lat1, double lon1, double lat2, double lon2) {
    float n = (float)((lat2 - lat1) * (double)DEG_TO_RAD * nav.r1);
    float e = (float)((lon2 - lon1) * (double)DEG_TO_RAD * nav.r2);
    float ret = fast_atan2(e, n)*57.3;
    return ret;
}

void CalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    nav.r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    nav.r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

void CalcGlobalDistance(double lat, double lon,float local_Lat,float local_Lon,float *posNorth,float *posEast ) {
    *posNorth = (lat - local_Lat) * nav.r1;
    *posEast =  (lon - local_Lon) * nav.r2;
}

void CalcGlobalLocation(float posNorth,float posEast){ 
    nav.lat=(float)posNorth/(float)(nav.r1+0.1)+nav.local_lat;
    nav.lon=(float)posEast/(float)(nav.r2+0.1)+nav.local_lon;
}


void flow_data_process(float T)
{
	static char init;
	if(!init)
	{
	  optical_flow.k_all=1;
		optical_flow.cut_rate=10;
		optical_flow.k_att_compass[Xr]=optical_flow.k_att_compass[Yr]=600.0f;
	}
  optical_flow.spd[Xr]=optical_flow.spd_o[Xr]*optical_flow.k_all;
  optical_flow.spd[Yr]=optical_flow.spd_o[Yr]*optical_flow.k_all;
	
	optical_flow.spd_flt[Xr] += (optical_flow.spd[Xr] - optical_flow.spd_flt[Xr]) *0.2;
  optical_flow.spd_flt[Yr] += (optical_flow.spd[Yr] - optical_flow.spd_flt[Yr]) *0.2;

	optical_flow.ang[Xr] += (optical_flow.k_att_compass[Xr]*tan(-vmc_all.att[PITr]*0.0173) - optical_flow.ang[Xr]) *0.2;	
	optical_flow.ang[Yr] += (optical_flow.k_att_compass[Yr]*tan(vmc_all.att[ROLr]*0.0173) - optical_flow.ang[Yr]) *0.2;		

	optical_flow.out[Xr] = optical_flow.spd_flt[Xr] - optical_flow.ang[Xr];	
	optical_flow.out[Yr] = optical_flow.spd_flt[Yr] - optical_flow.ang[Yr];

	optical_flow.spdo[Xr] = (optical_flow.out[Xr]- optical_flow.out_reg[Xr])/T;	
	optical_flow.out_reg[Xr]=optical_flow.out[Xr];
	optical_flow.spdo[Yr] = (optical_flow.out[Yr]- optical_flow.out_reg[Yr])/T;	
	optical_flow.out_reg[Yr]=optical_flow.out[Yr];
	
	DigitalLPF(optical_flow.spdo[Xr], &optical_flow.spdo_flt[Xr],optical_flow.cut_rate ,T);
	DigitalLPF(optical_flow.spdo[Yr], &optical_flow.spdo_flt[Yr],optical_flow.cut_rate ,T);
}

void pose_fushion(float T)
{
  double Zm[2][3],M_flag[4]={0,0,1,0};//p v a bv
	float pos_gps[2]={0,0};
	float spd_gps[2]={0,0};
	float yaw_use;
	if(nav.init_cnt[Xr]++>5/T)
		nav.init[Xr]=nav.init[Yr]=1;
		
	if(M_flag[0]){//position :uwb/ laser/ gps / visual odometer
		
	}

	//GPS fusion
	 //GPS init		
	 if(Gps_information.satellite_num>5&&Gps_information.kf_init==0&&
		 Gps_information.latitude>10&&Gps_information.latitude<180)
		{		
		 Gps_information.home_latitude=Gps_information.latitude;
		 Gps_information.home_longitude=Gps_information.longitude;
		 CalcEarthRadius(Gps_information.home_latitude);
		}		
	 //GPS measure
	 if(Gps_information.connect&&Gps_information.update[0]&&Gps_information.latitude>10&&Gps_information.latitude<180){	
		 CalcGlobalDistance(Gps_information.latitude,Gps_information.longitude,
		 Gps_information.home_latitude,Gps_information.home_longitude,
		 &pos_gps[Yr],&pos_gps[Xr]); 
		 
      DigitalLPF(Gps_information.real_N_vel,&Gps_information.local_spd_flt[Yr],3,T);
		  DigitalLPF(Gps_information.real_E_vel,&Gps_information.local_spd_flt[Xr],3,T);
			spd_gps[Xr]=Gps_information.local_spd_flt[Xr];
			spd_gps[Yr]=Gps_information.local_spd_flt[Yr];
		 
			Gps_information.local_pos[Xr]=pos_gps[Xr];
			Gps_information.local_pos[Yr]=pos_gps[Yr];

			if(Gps_information.kf_init==0&&Gps_information.satellite_num>5&&Gps_information.PVT_Vacc<4500){
					Gps_information.off_earth=get_declination(Gps_information.latitude,Gps_information.longitude);
					Gps_information.kf_init=1;
					position_mode=MODE_GPS;//set gps mode
					X_kf_pose[Xr][0]=pos_gps[Xr];
					X_kf_pose[Yr][0]=pos_gps[Yr];
			 }
		}
		//GPS reset
		static int gps_rst_cnt;
		if(Gps_information.kf_init&&(Gps_information.connect==0||Gps_information.PVT_Vacc>10000)) 
			gps_rst_cnt++;
		else
			gps_rst_cnt=0;
		if(gps_rst_cnt>5/T)
			position_mode=gps_rst_cnt=Gps_information.kf_init=0;
			
	//机器人里程驱动			
	nav.spd_b_o[Yr]=(vmc_all.param.encoder_spd[Rs]+vmc_all.param.encoder_spd[Ls])/(2);
	nav.spd_b_o[Zr]=(vmc_all.param.encoder_spd[Rs]-vmc_all.param.encoder_spd[Ls])/(vmc_all.W/2)*RAD_TO_DEG;
	nav.fake_yaw=nav.fake_yaw-k_fack_yaw*nav.spd_b_o[Zr]*T;
	nav.fake_yaw=To_180_degrees(nav.fake_yaw);
	
	//融合机体速度传感器
	if(M_flag[1]){
		nav.spd_b_o[Xr]=nav.spd_b_o[Xr]*w_spd[0]+(1-w_spd[0]);
		nav.spd_b_o[Yr]=nav.spd_b_o[Yr]*w_spd[0]+(1-w_spd[0]);
	}
	
	//机体坐标系数据转换地球坐标系
  yaw_use=vmc_all.att[YAWr];
	nav.acc_b_o[Xr]=vmc_all.acc[Yr];
	nav.acc_b_o[Yr]=vmc_all.acc[Xr];
	nav.acc_n_o[Xr]=nav.acc_b_o[Yr]*sind(yaw_use)+nav.acc_b_o[Xr]*cosd(yaw_use);
  nav.acc_n_o[Yr]=nav.acc_b_o[Yr]*cosd(yaw_use)-nav.acc_b_o[Xr]*sind(yaw_use);
	
	nav.spd_n_o[Yr]=cosd(yaw_use)*nav.spd_b_o[Yr];
	nav.spd_n_o[Xr]=sind(yaw_use)*nav.spd_b_o[Yr];
	
	//GPS 融合全局速度传感器
	if(Gps_information.kf_init&&Gps_information.update[0]){ 
		Gps_information.update[0]=0;	
		M_flag[0]=1;  

		nav.pos_n_o[Xr]=pos_gps[Xr];
		nav.pos_n_o[Yr]=pos_gps[Yr];			
		
		nav.spd_n_o[Xr]=spd_gps[Xr]*w_spd[1]+(1-w_spd[1])*nav.spd_n_o[Xr];
		nav.spd_n_o[Yr]=spd_gps[Yr]*w_spd[1]+(1-w_spd[1])*nav.spd_n_o[Yr];
		
		R_pose[0]=R_GPS[0];
	}	
		
	//发布测量向量
  Zm[Xr][0]=nav.pos_n_o[Xr];
	Zm[Xr][1]=nav.spd_n_o[Xr];
	Zm[Xr][2]=nav.acc_n_o[Xr];
  Zm[Yr][0]=nav.pos_n_o[Yr];
	Zm[Yr][1]=nav.spd_n_o[Yr];
  Zm[Yr][2]=nav.acc_n_o[Yr];	
	
	double Z_f[3]={M_flag[0],M_flag[2],M_flag[3]};
	if(nav.init[Xr]){
	 pose_kf(X_kf_pose[Xr], P_kf_pose[Xr],  Zm[Xr], Z_f, Q_pose, R_pose,  T);
	 pose_kf(X_kf_pose[Yr], P_kf_pose[Yr],  Zm[Yr], Z_f, Q_pose, R_pose,  T);
	}

	//发布状态估计
	nav.pos_n[Xr]=X_kf_pose[Xr][0];
	nav.pos_n[Yr]=X_kf_pose[Yr][0];
	nav.att[YAWr]=yaw_use;
	nav.spd_b[Xr]=-X_kf_pose[Yr][1]*sind(yaw_use)+X_kf_pose[Xr][1]*cosd(yaw_use);
	nav.spd_b[Yr]= X_kf_pose[Yr][1]*cosd(yaw_use)+X_kf_pose[Xr][1]*sind(yaw_use);
	nav.att_rate[Zr]=vmc_all.att_rate[YAWr];

	//计算经纬度估计
	if(Gps_information.kf_init&&position_mode==MODE_GPS)
	{
	nav.local_lat= Gps_information.home_latitude;
	nav.local_lon= Gps_information.home_longitude;
	CalcGlobalLocation(X_kf_pose[Yr][0],X_kf_pose[Xr][0]);
	}
}