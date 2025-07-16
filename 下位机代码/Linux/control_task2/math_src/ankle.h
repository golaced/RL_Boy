/*=========== ***doc description @ yyp*** ===========
并联脚踝解算

本模块电机原点：曲柄水平向后时
本模块电机正方向：电机处于原点时曲柄向上转动
本模块电机次序：左侧曲柄所在电机为0，右侧曲柄所在电机为1

本模块脚踝原点：脚底水平时（由于机械结构，此时电机不在原点）
本模块脚踝正方向：标准俯仰、横滚方向，俯为正、右滚为正
本模块脚踝次序：俯仰pit为0，横滚rol为1

若本模块与外部定义不一致，请在外部处理，不要更改本模块内部代码！
=====================================================*/
#pragma once
#include<Eigen/Dense>

namespace Jnt{
#define vec2f Eigen::Vector2f
#define vec3f Eigen::Vector3f

#define mat2f Eigen::Matrix2f
#define mat3f Eigen::Matrix3f

class ankleClass{
public:
	ankleClass(bool isLeftAnkle);
	//==调用接口，内部状态更新，无线程保护，适合实时调用=====
	//--正解，状态更新--
	void setActMot(	const float &motJL, const float &motJR,
					const float &motWL, const float &motWR,
					const float &motTL, const float &motTR);
	void getActAnk(	float &pitJ, float &rolJ,
					float &pitW, float &rolW,
					float &pitT, float &rolT);
	mat2f &getJcb(){return Jcb;}
	//--逆解--
	void setTgtAnk(	const float &pitJ, const float &rolJ,
					const float &pitW, const float &rolW,
					const float &pitT, const float &rolT);
	void getTgtMot( float &motJL,float &motJR,
					float &motWL,float &motWR,
					float &motTL,float &motTR);
	//==为外部提供的一次性计算器，线程安全，可独立调用，但计算开销大，不要实时调用=====
	mat2f mot2ank(float(&motJ)[2], float(&ankJ)[2]);//计算器，不更新状态，返回雅克比
	void ank2mot(float(&ankJ)[2], float(&motJ)[2]);//计算器，不更新状态
	mat2f calJcb(float(&motJ)[2]);//计算器，不更新状态
private:
	static const float rLen,rLenSq;//跟腱摇杆长度、平方
	static const float cLen,cLenSq;//电机曲柄长
	float lLen[2],lLenSq[2];       //跟腱连杆长度[左+右]，杆长平方，左右腿不同
	static const vec3f ankPr0[2];  //跟腱铰位
	static const vec3f pMLeftLeg[2],pMRightLeg[2];//电机位置，分左右腿的左右电机
	const vec3f *pM;               //电机位置
	static const float motLim,motDif,ankLim;
	float motBias[2];

	float actMotJ[2], oldMotJ[2], tgtMotJ[2], actAnkJ[2];
	vec2f tgtMotW,tgtMotT, actAnkW,actAnkT;
	mat2f Jcb,JcbInvT;

	vec3f tmpAnkPr[2];
	mat3f tmpRxy;
	void mot2ank(float(&motJ)[2], float(&motJRef)[2], mat2f &JcbInvTRes, mat2f &JcbRes, float(&ankJ)[2]);
	void ank2mot(float(&ankJ)[2], vec3f(&ankPr)[2], float(&motJ)[2]);
	void calJcbInvT(const float(&motJ)[2], const float(&ankJ)[2], const vec3f(&ankPr)[2], mat2f &JcbInvTRes);
};

}//namespace
