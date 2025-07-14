#ifndef __LEG_PLANNER_H__
#define __LEG_PLANNER_H__
#include "base_struct.h"

void cal_vir_leg_pos(VMC* in, END_POS *td_vir_n, END_POS *lf_vir_n);
float end_pos_y_correction(VMC* in, float ep_slip_n, float dt);
void trig_curve_global(VMC *in, float *x, float *y, float *z, float dt);
void espd_to_qspd(VMC *in, END_POS spd_end, float* spd_q, float dt);
void reset_sw_torque_as_now(VMC *in, float torque_now[3]);
void trig_plan(char leg_sel_trig, float dt);
char trig_lift(char leg_sel_trig, float dt);
char trig_swing(char leg_sel_trig, float dt);
char trig_td(char leg_sel_trig, float dt);
void swing_spd_control(char leg_sel_trig, float dt);
void reset_tar_pos(char leg_sel_trig);
void trig_plan_online(char leg_sel_trig, float dt);
#endif