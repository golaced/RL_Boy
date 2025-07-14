/*=========== ***doc description @ yyp*** ===========

=====================================================*/
#pragma once
#include"eigen.h"

namespace JntMap{

extern "C" void update(vecXf &pos,vecXf &vel,vecXf &tor);
extern "C" void dwdate(vecXf &pos,vecXf &vel,vecXf &tor);

}//namespace
