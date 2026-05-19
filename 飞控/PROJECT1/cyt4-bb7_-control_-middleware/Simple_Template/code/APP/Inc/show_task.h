#ifndef __SHOW_TASK_H
#define __SHOW_TASK_H

#include <stdint.h>
#include "zf_common_headfile.h"

typedef struct{
    float p;
    float i;
    float d;
}pid;

typedef struct{
	float roll;
	float pitch;
	float yaw;
	float gyrox;
	float gyroy;
	float gyroz;
	float accelx;
	float accely;
	float accelz;
	float m1;
	float m2;
	float m3;
	float m4;
	float balanceTaskFreq;
	float height;
	float spa06Height;
	float spa06NofilterHeight;
	float c_roll;
	float c_pitch;
	float c_yaw;
	float c_height;
	float posx;
	float posy;
	float speedx;
	float speedy;
	float zero_roll;
	float zero_pitch;
	float targetX;
	float targetY;
        
        float motorA_out;
        float motorB_out;
        float motorC_out;
        float motorD_out;
        
        int16 delta_x;
        int16 delta_y;
        
        float flow_ang_x;
        float flow_fix_x_i;
        
        float flow_ang_y;
        float flow_fix_y_i;
        
        float loc_x;
        float loc_y;
        float loc_xs;
        float loc_ys;
        
        float debug_date1;
        float debug_date2;
        float debug_date3;
        float debug_date4;
        float debug_date5;
        float debug_date6;
        float debug_date7;
        float debug_date8;
        float debug_date9;
        float debug_date10;
        
        float setpitch;
        float setroll;
        
        pid pitch_rate;
        pid roll_rate;
        pid pitch_pos;
        pid roll_pos;
        
        
        
}APPShowType_t;

#endif

