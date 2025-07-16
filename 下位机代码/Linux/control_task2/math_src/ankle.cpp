/*=========== ***doc description @ yyp*** ===========
踝关节中心为原点，z指向膝，先左后右
=====================================================*/
#include"ankle.h"

// #define AnkTest

#ifdef AnkTest
#include"iopack.h"
#else
#include<iostream>
using namespace std;
#define For2 for(int i(0);i!=2;++i)
#define For(x) for(int i(0);i!=x;++i)

static const float Pi =3.14159265359;
static const float Pi2=6.28318530718;
static const float P2i=1.57079632679;
namespace Alg{
	inline int sign(const float &a){return a<0?-1:1;}
	inline bool clip(float&x,float lim){
		if(lim<0){lim=-lim;}
		if(x<-lim){x=-lim;return 1;}
		else if(x>lim){x=lim;return 1;}
		return 0;
	}
}//namespace
#endif
namespace Ei{
	inline void rp2Rxy(const float &rol,const float &pit, mat3f &Rxy){
		float cx=cos(rol),cy=cos(pit);
		float sx=sin(rol),sy=sin(pit);
		Rxy<<cy, sy*sx, sy*cx,
			  0, cx,   -sx,
			-sy, cy*sx, cy*cx;
	}
	inline mat3f rp2Rxy(const float &rol,const float &pit){
		mat3f Rxy;
		rp2Rxy(rol,pit,Rxy);
		return Rxy;
	}
}//namespace


namespace Jnt{
	/*
	rLen,rLenSq;				//跟腱摇杆长度、平方
	cLen,cLenSq;				//电机曲柄长
	lLen[2],lLenSq[2];			//跟腱连杆长度[左+右]，杆长平方，左右腿不同
	ankPr0[2];					//跟腱铰位
	pMLeftLeg[2],pMRightLeg[2];	//电机位置，分左右腿的左右电机
	pM;							//电机位置*/
	const vec3f ankleClass::ankPr0[2]{  {-0.052, 0.0655/2, 0},
										{-0.052,-0.0655/2, 0}};
	const vec3f ankleClass::pMLeftLeg[2]{{0.01, 0.038, 0.176},
										 {0.01,-0.038, 0.27},};
	const vec3f ankleClass::pMRightLeg[2]{{0.01, 0.038, 0.27},
										  {0.01,-0.038, 0.176},};
	const float ankleClass::rLen{ankPr0[0].norm()}, ankleClass::rLenSq{rLen*rLen};
    const float ankleClass::cLen{0.052}, ankleClass::cLenSq{cLen*cLen};
	const float ankleClass::motLim{1.5}, ankleClass::motDif{1.3}, ankleClass::ankLim{1.5};

	ankleClass::ankleClass(bool isLeftAnkle){
		if(isLeftAnkle){
			pM=pMLeftLeg;
			lLen[0]=0.17628;
			lLen[1]=0.27019;
			motBias[0]=0;
			motBias[1]=0;
		}else{
			pM=pMRightLeg;
			lLen[0]=0.27019;
			lLen[1]=0.17628;
			motBias[0]=0;
			motBias[1]=0;
		}
		For2{
			lLenSq[i]=lLen[i]*lLen[i];
			actMotJ[i]=0;
			oldMotJ[i]=0.01;
			tgtMotJ[i]=0;
			actAnkJ[i]=0;
		}
		tgtMotW.setZero();
		tgtMotT.setZero();
		actAnkW.setZero();
		actAnkT.setZero();
		calJcbInvT(actMotJ,actAnkJ,ankPr0,JcbInvT);//为下面第一次的mot2ank准备初值
		Jcb=JcbInvT.inverse().transpose();
		mot2ank(actMotJ,oldMotJ,JcbInvT,Jcb,actAnkJ);
	}
	void ankleClass::setActMot( const float &motJL, const float &motJR,
								const float &motWL, const float &motWR,
								const float &motTL, const float &motTR)
	{	actMotJ[0]=motJL+motBias[0];
		actMotJ[1]=motJR+motBias[1];
		mot2ank(actMotJ,oldMotJ,JcbInvT,Jcb,actAnkJ);
		actAnkW<<motWL,motWR;
		actAnkT<<motTL,motTR;
		actAnkW=Jcb*actAnkW;
		actAnkT=JcbInvT*actAnkT;
	}
	void ankleClass::getActAnk(	float &pitJ, float &rolJ,
								float &pitW, float &rolW,
								float &pitT, float &rolT)
	{	pitJ=actAnkJ[0];
		rolJ=actAnkJ[1];
		pitW=actAnkW[0];
		rolW=actAnkW[1];
		pitT=actAnkT[0];
		rolT=actAnkT[1];
	}
	void ankleClass::setTgtAnk(	const float &pitJ, const float &rolJ,
								const float &pitW, const float &rolW,
								const float &pitT, const float &rolT)
	{	float tgtAnkJ[2]{pitJ,rolJ};
		// For2{
			// Alg::clip(tgtAnkJ[i],0.5);
		// }
		ank2mot(tgtAnkJ, tgtMotJ);
		tgtMotW<<pitW,rolW;
		tgtMotT<<pitT,rolT;
		tgtMotW=JcbInvT.transpose()*tgtMotW;
		tgtMotT=Jcb.transpose()*tgtMotT;
	}
	void ankleClass::getTgtMot( float &motJL,float &motJR,
								float &motWL,float &motWR,
								float &motTL,float &motTR)
	{	motJL=tgtMotJ[0]-motBias[0];
		motJR=tgtMotJ[1]-motBias[1];
		motWL=tgtMotW[0];
		motWR=tgtMotW[1];
		motTL=tgtMotT[0];
		motTR=tgtMotT[1];
	}
	//外部调用
	mat2f ankleClass::mot2ank(float(&motJ)[2], float(&ankJ)[2]){
		float motRef[2]{oldMotJ[0],oldMotJ[1]};//不能改变old，因此定义临时变量
		mat2f JcbInvTRes(JcbInvT),JcbRes(Jcb);
		ankJ[0]=actAnkJ[0];
		ankJ[1]=actAnkJ[1];
		mot2ank(motJ,motRef,JcbInvTRes,JcbRes,ankJ);
		return JcbRes;
	}
	//外部调用
	void ankleClass::ank2mot(float(&ankJ)[2], float(&motJ)[2]){
		vec3f pR[2];
		ank2mot(ankJ,pR,motJ);
	}
	//外部调用
	mat2f ankleClass::calJcb(float(&motJ)[2]){
		if(Alg::clip(motJ[0],1) || Alg::clip(motJ[1],1)){
			cout<<"calJcb调用电机角度超程，已限幅\n";
		}
		float ankJ[2];
		return mot2ank(motJ,ankJ);
	}
	//传参形式，以便同时满足内部、外部调用数据安全
	void ankleClass::mot2ank(float(&motJ)[2], float(&motJRef)[2], mat2f &JcbInvTRes, mat2f &JcbRes, float(&ankJ)[2]){
		float amp=hypot(motJ[0],motJ[1]);
		if(amp>motLim){
			cout<<"mot2ank调用电机超程，初始motJ=["<<motJ[0]<<", "<<motJ[1]<<"]，";
			motJ[0]*=motLim/amp;
			motJ[1]*=motLim/amp;
			cout<<"限幅为motJ=["<<motJ[0]<<", "<<motJ[1]<<"]\n";
		}
		float dif=motJ[0]-motJ[1];
		if(abs(dif)>motDif){
			cout<<"mot2ank调用电机超程，初始motJ=["<<motJ[0]<<", "<<motJ[1]<<"]，";
			float ave=0.5*(motJ[0]+motJ[1]);
			motJ[0]=ave+Alg::sign(dif)*motDif/2;
			motJ[1]=ave*2-motJ[0];
			cout<<"限幅为motJ=["<<motJ[0]<<", "<<motJ[1]<<"]\n";
		}
		vec2f dAnkJ;
		For(50){//牛顿迭代
			dAnkJ<<motJ[0]-motJRef[0], motJ[1]-motJRef[1];
			dAnkJ=JcbRes*dAnkJ;
			//行列式保证收敛，工作在限幅[-1,1]下。if放开限幅，为保证收敛应添加系数缩小步长。或重新设计步长。
			dAnkJ/=sqrt(abs(JcbRes.determinant()));
			For2{
				// Alg::clip(dAnkJ[i], 0.1);
				ankJ[i]+=dAnkJ[i];
			}
			#ifdef AnkTest
			print(i,ankJ[0],ankJ[1],abs(JcbRes.determinant()));
			#endif
			ank2mot(ankJ, tmpAnkPr, motJRef);
			calJcbInvT(motJRef, ankJ, tmpAnkPr, JcbInvTRes);
			JcbRes=JcbInvTRes.inverse().transpose();
			if(dAnkJ.squaredNorm() <1e-7){
				break;
			}
		}
	}
	//传参形式，以便同时满足内部、外部调用数据安全
	void ankleClass::ank2mot(float(&ankJ)[2], vec3f(&ankPr)[2], float(&motJ)[2]){
		float amp=hypot(ankJ[0],ankJ[1]);
		if(amp>ankLim){
			cout<<"ank2mot调用踝关节超程，初始ankJ=["<<ankJ[0]<<", "<<ankJ[1]<<"]，";
			ankJ[0]*=ankLim/amp;
			ankJ[1]*=ankLim/amp;
			cout<<"限幅为ankJ=["<<ankJ[0]<<", "<<ankJ[1]<<"]\n";
		}
		Ei::rp2Rxy(ankJ[1],ankJ[0],tmpRxy);//结构上先pit后rol
		For2{
			ankPr[i]=tmpRxy*ankPr0[i];
			float dy=ankPr[i][1] -ankPr0[i][1];
			float ll=lLenSq[i]-dy*dy;
			float m2pX=pM[i][0] -ankPr[i][0];
			float m2pZ=pM[i][2] -ankPr[i][2];
			float xz=hypot(m2pX,m2pZ);
			float alpha=acos((cLenSq+xz*xz-ll) /(2*cLen*xz));
			float beta=atan2(m2pX,m2pZ);
			motJ[i]=alpha+beta-P2i;
		}
	}
	//传参形式，以便同时满足内部、外部调用数据安全
	void ankleClass::calJcbInvT(const float(&motJ)[2], const float(&ankJ)[2], const vec3f(&ankPr)[2], mat2f &JcbInvTRes){
		vec3f xx(cos(ankJ[0]), 0, -sin(ankJ[0]));
		vec3f motPc,f,l,m;
		For2{
			float c=cos(motJ[i]), s=sin(motJ[i]);
			motPc=pM[i]+vec3f(-c*cLen, 0, s*cLen);
			f<<s, 0, c;
			l=motPc-ankPr[i];
			l.normalize();
			f=(1/cLen)/(f.dot(l))*l;
			m=ankPr[i].cross(f);
			JcbInvTRes(0,i)=m[1];
			JcbInvTRes(1,i)=m.dot(xx);
		}
	}
}//namespace
