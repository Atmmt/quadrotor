#ifndef __BSP_PMW3901_H
#define __BSP_PMW3901_H



#include "zf_common_headfile.h"
#include "bsp_imu.h"
//光流数据结构体
typedef struct 
{       
        float fix_x ,fix_y;
        float fix_x_i,fix_y_i;
        
        float ang_x,ang_y;
        float out_x_i,out_y_i;
        float out_x_i_o,out_y_i_o;
        
        float spend_x,spend_y;
        float loc_x,loc_y;
        float loc_xs,loc_ys;
        
}FLOW_DATE_t;



//IMU操作接口
typedef struct {
    void (*Init)(void);
    void (*UpdateFlow)(ATTITUDE_DATA_t *Imu_Date,float hight,float dT,FLOW_DATE_t* Out_Date); //更新光流数据
	
}FLOWInterface_t,*pFLOWInterface_t;

extern FLOWInterface_t UserPMW3901;



#endif

