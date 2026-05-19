/*
 * @Author: Jae Frank[thissfk@qq.com]
 * @Date: 2024-02
 * @LastEditors: Jae Frank[thissfk@qq.com]
 * @LastEditTime: 2024-05
 * @FilePath: main_cm7_0.c
 * @Description:
 *            If you need more information,
 * please contact Jae Frank[thissfk@qq.com] to get an access.
 * Copyright (c) 2024 by Jae Frank, All Rights Reserved.
 */
/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 *
 *
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *            佛祖保佑     永不宕机     永无BUG
 */

// #include "cmsis_os.h"
#include "zf_common_headfile.h"


//C库相关
#include <stdio.h>

//RTOS相关
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "event_groups.h" 

//BSP
#include "bsp_led.h"
#include "bsp_imu.h"
#include "bsp_buzzer.h"
#include "bsp_motor.h"
#include "bsp_battery.h"
#include "bsp_key.h"
#include "bsp_uart.h"
#include "bsp_pmw3901.h"
   
//APP
#include "balance_task.h"
#include "show_task.h"

//配置RTOS debug任务,可用于检查系统状态
#define userconfig_OPEN_STACK_CHECK 0     //开启任务栈大小检查打印
#define userconfig_OPEN_CPU_USAGE_CHECK 0 //开启检查CPU占比打印
#define userconfig_OPEN_CHECK_HEAPSIZE 0  //检查剩余的堆大小
#if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
void CpuUsageCheckTask(void *param);
#endif

//显示任务
__weak void ShowTask(void* param)
{
	for(;;) vTaskDelay(1);
}

//平衡主控制任务
__weak void balance_task(void* param)
{
	for(;;) vTaskDelay(1);
}

__weak void stp23L_task(void* param)
{
	for(;;) vTaskDelay(1);
}

//启动任务
static void start_task(void *pvParameters);
static TaskHandle_t StartTask_Handler = NULL; 

//全局变量
//TaskHandle_t stp23LTask_Handle = NULL; //任务句柄
QueueHandle_t           g_xQueueFlyControl;     //四轴飞行器控制队列
EventGroupHandle_t      g_xEventFlyAction; //四轴动作事件组


uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

uint8  get_data = 0;                                                            // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数
fifo_struct uart_data_fifo;

// **************************** 代码区域 ****************************

int main(void) {
      clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
//      debug_init();                  // 调试串口信息初始化
      pSerialInterface_t dubug_uart0 = &UserDev_usart0;
      dubug_uart0->init(115200);
      
      
      float butter_v = 0;
      
      
      pKEYInterface_t key = &UserKEY;      
      key->init();
      
      // LED初始化
      pLedInterface_t led = &UserLed;
      led->init();
      
//    //蜂鸣器
      pBuzzeInterface_t buzzer = &UserBuzzer;
      buzzer->init();
      
////    IMU初始化
      pIMUInterface_t imu = &UserIMU963RA;
      imu->Init();
      
      // 电调输出pwm 100hz duty控制范围 1000 -- 2000
      pMOTORInterface_t motor = &UserMotor;
      motor->init();

//    //显示屏初始化、
      ips200_init(IPS200_TYPE_SPI);
//    
      

      
//    // 光流模块初始化
//      pmw3901_init();
      pFLOWInterface_t pmw3901 = &UserPMW3901;
      pmw3901->Init();
//      
//      
////    // 摄像头初始化    ！！！ 初始化未成功时 调大MT9V03X_COF_IIC_DELAY 给软件IIC留够初始化时间
//      mt9v03x_init();
      
      //    // TOF初始化
      dl1b_init();
      
      pBATTERYInterface_t buttery = &UserBATTERY;
      buttery->init();
//      
////    // 串口485通信
      pSerialInterface_t rs485_uart4 = &UserDev_usart4;
      rs485_uart4->init(115200);
 
      
      
    //创建队列、信号量、事件组等内容
    g_xQueueFlyControl = xQueueCreate(1,sizeof(FlyControlType_t));
    
    //创建四轴动作事件组的实例
     g_xEventFlyAction = xEventGroupCreate();
    
      
      // 初始化完毕 蜂鸣器提示
     buzzer->woring();
    //创建开始任务
     xTaskCreate(start_task,"start_task",128*4,NULL,osPriorityNormal,&StartTask_Handler);
    
    //开启任务调度					
     vTaskStartScheduler();
    
     uint8 k;
     uint32 duty = 1000;
    //不会运行到此处
    while(1)
    {
//          k = key->getValue();
//          if(k == 2)
//          {
//              duty+=25;
//              printf("%d \r\n",duty);
//          }
//          if(k == 3)
//          {
//              duty-=25;
//              printf("%d \r\n",duty);
//          }
//          motor->setduty(duty,duty,duty,duty);
//          ips200_show_int(0,0,duty,5);
          
      
//        system_delay_ms(25);            // PMW3901 光流采集间隔至少需要20ms
//        pmw3901_get_motion();           // 采集光流数据 
//        printf("delta x:%d, delta y:%d\r\n", pmw3901_delta_x, pmw3901_delta_y);
//        printf("distanse x:%d, distanse y:%d\r\n", pmw3901_delta_x_i, pmw3901_delta_y_i);
        
        
//        if(mt9v03x_finish_flag)
//        {
//           ips200_displayimage03x((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);                       // 显示原始图像
////           ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, 240, 180, 1);     // 显示二值化图像
//            mt9v03x_finish_flag = 0;
//        }
          printf("FreeRTOS system error...\r\n");
      
      
//        dl1b_get_distance();
//        if(dl1b_finsh_flag == 1)
//        {
//            dl1b_finsh_flag = 0;
//            printf("dl1b distance: %d mm\r\n", dl1b_distance_mm);
//        }
//        system_delay_ms(20);
      
//      butter_v = USER_ADC_Get_AdcBufValue();
//      printf("%f \r\n",butter_v);
      

//      system_delay_ms(20);
    }
}


//开始任务
void led_task(void *pvParameters)
{
    pLedInterface_t led = &UserLed;
    while(1)
    {
      led->on();

      vTaskDelay(500);
      led->off();
      vTaskDelay(500);
    }

}

void test_task(void* param)
{
    //获取时基,用于辅助任务能指定固定频率运行
    TickType_t preTime = xTaskGetTickCount();
    
    //控制任务的频率,单位HZ
    const uint16_t TaskFreq = 200;
  
    IMU_DATA_t data = {0};
    ATTITUDE_DATA_t data_a = {0};
    pIMUInterface_t imu = &UserIMU963RA;
    
    while (1)
    {
        imu->Update_9axisVal(&data);
        imu->UpdateAttitude(data,&data_a);
        
        extern APPShowType_t appshow;
        appshow.gyrox = data.gyro.x;
        appshow.gyroy = data.gyro.y;
        appshow.gyroz = data.gyro.z;
        
        appshow.accelx = data.accel.x;
        appshow.accely = data.accel.y;
        appshow.accelz = data.accel.z;
        
        appshow.pitch = data_a.pitch;
        appshow.roll  = data_a.roll;
        appshow.yaw   = data_a.yaw;
	

        /* 延迟指定频率 */
        vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
    }

}
//开始任务
void start_task(void *pvParameters)
{
    /* 任务启动前处理的内容 */
    
    //进入临界区
    taskENTER_CRITICAL();
	
    /* 创建其他任务 */
    xTaskCreate(ShowTask,"ShowTask",128*4,NULL,osPriorityBelowNormal7,NULL);
    xTaskCreate(balance_task,"BalanceTask",128*5,NULL,osPriorityNormal,NULL);
    xTaskCreate(stp23L_task,"stp23l_task",128*3,NULL,osPriorityNormal,NULL);
    
    
//    xTaskCreate(led_task,"led_task",128*2,NULL,osPriorityNormal,NULL);
//    xTaskCreate(test_task,"testTask",128*5,NULL,osPriorityNormal,NULL);
    
    #if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
     xTaskCreate(CpuUsageCheckTask,"DebugTask",128*4, NULL ,osPriorityAboveNormal,NULL);
    #endif
     

	//删除开始任务
    vTaskDelete(NULL);
	
	//退出临界区
    taskEXIT_CRITICAL();
}






#if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
void CpuUsageCheckTask(void *param)
{
    static char showbuf[500];
    
    while( 1 )
    {
#if 1 == userconfig_OPEN_CPU_USAGE_CHECK
        //打印CPU占比
        vTaskGetRunTimeStats(showbuf);
        printf("TaskName\tUseTime\tCPU\r\n");
        printf("%s\r\n",showbuf);
        vTaskDelay(5000);
#endif

#if 1 == userconfig_OPEN_STACK_CHECK
        //打印剩余任务栈大小,单位word
        vTaskList(showbuf);
        printf("TaskName\tTaskState\tTaskPrio\tStackSize\tTaskNum\r\n");
        printf("%s\r\n",showbuf);
        vTaskDelay(5000);
#endif
            
#if 1 == userconfig_OPEN_CHECK_HEAPSIZE
        //打印剩余的堆区大小,单位bytes
        printf("free heap size : %d bytes\r\n\r\n",xPortGetFreeHeapSize());
        vTaskDelay(5000);
#endif
    }
}
#endif


