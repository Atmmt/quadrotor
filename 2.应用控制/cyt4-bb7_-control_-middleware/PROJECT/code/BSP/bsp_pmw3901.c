#include "bsp_pmw3901.h"
#include "math.h"
#include "string.h"


void Pmw3901_Init(void)
{
    pmw3901_init();
}

void Pmw3901UpDate(ATTITUDE_DATA_t *Imu_Date,float hight,float dT,FLOW_DATE_t* Out_Date)
{
    static FLOW_DATE_t pixel_flow = {0};
    pmw3901_get_motion();
    
    // 低通滤波
    pixel_flow.fix_x_i += (pmw3901_delta_x_i - pixel_flow.fix_x_i) *0.2;
    pixel_flow.fix_y_i += (pmw3901_delta_y_i - pixel_flow.fix_y_i) *0.2;
    
    // 姿态角补偿
    pixel_flow.ang_x += (600.0f*tan(Imu_Date->pitch)  - pixel_flow.ang_x) *0.2;
    pixel_flow.ang_y += (-600.0f*tan(Imu_Date->roll) - pixel_flow.ang_y) *0.2;
    
    
    pixel_flow.out_x_i = (pixel_flow.fix_x_i - pixel_flow.ang_x)*0.01f;
    pixel_flow.out_y_i = (pixel_flow.fix_y_i - pixel_flow.ang_y)*0.01f;
    
    //求微分速度
    pixel_flow.spend_x = (pixel_flow.out_x_i - pixel_flow.out_x_i_o)/dT;	pixel_flow.out_x_i_o = pixel_flow.out_x_i;
    pixel_flow.spend_y = (pixel_flow.out_y_i - pixel_flow.out_y_i_o)/dT;	pixel_flow.out_y_i_o = pixel_flow.out_y_i;
    
    //低通滤波
    pixel_flow.fix_x += ( pixel_flow.spend_x - pixel_flow.fix_x ) * 0.1f;
    pixel_flow.fix_y += ( pixel_flow.spend_y - pixel_flow.fix_y ) * 0.1f;
    
    //式中HIGH为实际高度，单位：米
    float cpi = (hight / 11.914f) *2.54f ;
     
    //积分位移单位转换为：m
    pixel_flow.loc_x = -pixel_flow.out_x_i * cpi;
    pixel_flow.loc_y = pixel_flow.out_y_i * cpi;
     
    //微分速度单位转换为：m/s
    pixel_flow.loc_xs = -pixel_flow.fix_x * cpi;
    pixel_flow.loc_ys = pixel_flow.fix_y * cpi;
    
    memcpy(Out_Date,&pixel_flow,sizeof(FLOW_DATE_t));// 拷贝数据

}
//驱动挂载
FLOWInterface_t UserPMW3901 = {
	.Init = Pmw3901_Init,
	.UpdateFlow = Pmw3901UpDate
};

