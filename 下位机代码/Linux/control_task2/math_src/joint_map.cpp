/*=========== ***doc description @ yyp*** ===========
踝关节中心为原点，z指向膝，先左后右
=====================================================*/
#include"joint_map.h"
#include"ankle.h"
#include"iopack.h"
#include"ini.h"

namespace JntMap{

// impClass单例保证main前初始化
class impClass{
public:
	static impClass& instance(){
		static impClass singtn;
		return singtn;
	}
	void update(vecXf &pos,vecXf &vel,vecXf &tor);
	void dwdate(vecXf &pos,vecXf &vel,vecXf &tor);
private:
	impClass();
	impClass(const impClass&)=delete;
	impClass & operator=(const impClass&)=delete;

    //Ini::iniClass ini;
	Jnt::ankleClass ankle[2];//左+右
	int drvNums;
};
	impClass::impClass():ankle{0,0}{
        //ini.open("../config/driver.ini");
        //drvNums=ini["drvNums"];
	}
	void impClass::update(vecXf &pos,vecXf &vel,vecXf &tor){
		if(pos.size()!=drvNums || vel.size()!=drvNums || tor.size()!=drvNums){
			throw runtime_error("joint map step(): 输入向量大小不匹配！");
		}
		ankle[0].setActMot(	pos[24], pos[23],
							vel[24], vel[23],
							tor[24], tor[23]);
		ankle[0].getActAnk(	pos[23], pos[24],
							vel[23], vel[24],
							tor[23], tor[24]);
		ankle[1].setActMot(	pos[29], pos[30],
							vel[29], vel[30],
							tor[29], tor[30]);
		ankle[1].getActAnk(	pos[29], pos[30],
							vel[29], vel[30],
							tor[29], tor[30]);
		
		pos[23]*=-1;
		pos[29]*=-1;
		
		vel[23]*=-1;
		vel[29]*=-1;

		tor[23]*=-1;
		tor[29]*=-1;
	}
	void impClass::dwdate(vecXf &pos,vecXf &vel,vecXf &tor){
		if(pos.size()!=drvNums || vel.size()!=drvNums || tor.size()!=drvNums){
			throw runtime_error("joint map step(): 输入向量大小不匹配！");
		}
		ankle[0].setTgtAnk(	-pos[23], pos[24],
							-vel[23], vel[24],
							-tor[23], tor[24]);
		ankle[0].getTgtMot(	pos[24], pos[23],
							vel[24], vel[23],
							tor[24], tor[23]);
		ankle[1].setTgtAnk(	-pos[29], pos[30],
							-vel[29], vel[30],
							-tor[29], tor[30]);
		ankle[1].getTgtMot(	pos[29], pos[30],
							vel[29], vel[30],
							tor[29], tor[30]);
	}
// ==============================================
	void update(vecXf &pos,vecXf &vel,vecXf &tor){
		impClass::instance().update(pos,vel,tor);
	}
	void dwdate(vecXf &pos,vecXf &vel,vecXf &tor){
		impClass::instance().dwdate(pos,vel,tor);
	}
}//namespace
