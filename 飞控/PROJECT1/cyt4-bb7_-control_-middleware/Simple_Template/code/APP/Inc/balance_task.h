#ifndef __BALANCE_TASK_H
#define __BALANCE_TASK_H

#include <stdint.h>

//Dshot油门范围(48~2047)
#define Dshot_MIN 0
#define Dshot_MAX 1423 //实际最大油门值2047

//四轴机体事件组
enum{
	UNUSE_HeightMode_Event = (1<<0),  //无定高模式事件
	IMU_CalibZeroDone_Event = (1<<1), //IMU标定完成事件
	StartFly_Event = (1<<2),           //启动飞行事件
	LowPower_Event = (1<<3),          //低电量事件
	
	//单路电机、4路电机测试
	TestMotorAup_Event   = ( 1<<4 ),
	TestMotorAdown_Event = ( 1<<5 ),
	TestMotorBup_Event   = ( 1<<6 ),
	TestMotorBdown_Event = ( 1<<7 ),
	TestMotorCup_Event   = ( 1<<8 ),
	TestMotorCdown_Event = ( 1<<9 ),
	TestMotorDup_Event   = ( 1<<10 ),
	TestMotorDdown_Event = ( 1<<11 ),
	TestMotorAllup_Event = ( 1<<12 ),
	TestMotorAlldown_Event=( 1<<13 ),
	
	//电机测试模式
	TestMotorMode_Event =  ( 1<<14 ),
	FlyMode_HeadLessMode_Event = ( 1<<15 ),
        
        //6轴标定完成
        IMU_axilValZeroDone_Event = ( 1<<16 ),
        
        // 开启高度控制模式
        TestSetuphightControl= ( 1<<17 ),
	/* 事件组最大23 */
};

//四轴控制源,控制优先级排序为低到高
enum{
	IDLECmd = 0,//控制空闲
	APPCmd = 1, //APP控制,最低优先级, 1 
	PS2Cmd = 2,
};

//四轴控制结构体
typedef struct{
	uint8_t source; //控制来源
	float pitch;   //前后控制,+前进,-后退
	float roll;    //左右控制,+左移,-右移
	float gyroz;     //方向控制,+逆时针,-顺时针
	float height;  //高度控制
        uint8_t setup;  // 启动开关
        uint8_t height_setup; // 高度模式启动
        short throttle;   // 油门控制
        float set_pitch;
        float set_roll;
        
}FlyControlType_t;

extern float g_readonly_mainTaskFreq;
extern float angle_to_rad(float angle);


#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	        200
#define RATE_250_HZ 	        250
#define RATE_500_HZ 	        500
#define RATE_1000_HZ 	        1000

#define MAIN_LOOP_RATE 	RATE_1000_HZ
#define MAIN_LOOP_DT	(uint32)(1000/MAIN_LOOP_RATE)	/*μ￥??ms*/

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)


#define ATTITUDE_ESTIMAT_RATE	RATE_250_HZ	//×?ì??a???ù?ê
#define ATTITUDE_ESTIMAT_DT		(1.0/RATE_250_HZ)

#define POSITION_ESTIMAT_RATE	RATE_250_HZ	//?????¤1à?ù?ê
#define POSITION_ESTIMAT_DT		(1.0/RATE_250_HZ)

#define RATE_PID_RATE			RATE_500_HZ //???ù?è?·￡¨?ú?·￡?PID?ù?ê
#define RATE_PID_DT				(1.0/RATE_500_HZ)

#define ANGEL_PID_RATE			ATTITUDE_ESTIMAT_RATE //???è?·￡¨ía?·￡?PID?ù?ê
#define ANGEL_PID_DT			(1.0/ATTITUDE_ESTIMAT_RATE)

#define VELOCITY_PID_RATE		POSITION_ESTIMAT_RATE //?ù?è?·￡¨?ú?·￡?PID?ù?ê
#define VELOCITY_PID_DT			(1.0/POSITION_ESTIMAT_RATE)

#define POSITION_PID_RATE		POSITION_ESTIMAT_RATE //?????·￡¨ía?·￡?PID?ù?ê
#define POSITION_PID_DT			(1.0/POSITION_ESTIMAT_RATE)




#endif /* __BALANCE_TASK_H */
