
#include "zf_common_headfile.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "bsp_pmw3901.h"
#include "balance_task.h"

FLOW_DATE_t flow_o={ 0 };

//激光测距传感器任务
void stp23L_task(void *pvParameters)
{
//	pRtosDebugInterface_t rtosdebug = &UserSoft_TimerDebug;
//	RtosDebugPrivateVar privar = { 0 };
//	printf("fre:%d\r\n",rtosdebug->UpdateFreq(&privar));

        pFLOWInterface_t flow = &UserPMW3901;
        extern ATTITUDE_DATA_t* p_attitudeVal;
        extern float use_distance;
        
        
        
	while(1)
	{
//		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);//等待数据就绪处理
              dl1b_get_distance();
//              pmw3901_get_motion();
              flow->UpdateFlow(p_attitudeVal,use_distance,0.03,&flow_o);
              
              vTaskDelay(30);
//                dl1b_finsh_flag = 0;     
	}
}
	


////#include "bsp_N10.h"

////临时数据，用于存放队列内读取的值.union表示以下的数据存放在同一段内容空间.不可同时使用
//typedef union {
//	OriData_STP23L_t stp23L;
//	OriData_N10_t N10;
//}TempqueueHandle_t;

////队列读取内容临时存放
//static TempqueueHandle_t tmpRead;

//void SensorHandleTask(void *param)
//{
//	//任务内使用的队列
//	extern QueueHandle_t g_xQueuestp23L_Ori;
//	extern QueueHandle_t g_xQueueN10_Ori; 
//	extern QueueSetHandle_t g_xQueueSetSensor;
//	
//	QueueSetMemberHandle_t xQueueDataHandle;//队列集状态反馈值
//	
//	while(1)
//	{
//		xQueueDataHandle = xQueueSelectFromSet(g_xQueueSetSensor,portMAX_DELAY);//阻塞等待队列数据
//		
//		//STP23L的数据
//		if( g_xQueuestp23L_Ori == xQueueDataHandle )
//		{
//			/* 调试使用，观察队列中的数据是否被写满 */
//			//printf("stp23L queue free space:%d\r\n",uxQueueSpacesAvailable(g_xQueuestp23L_Ori));
//				
//			xQueueReceive(g_xQueuestp23L_Ori,&tmpRead.stp23L,0);//将队列中的数据赋值
//			
//			if( tmpRead.stp23L.DataLen==195 ) //1帧完整的距离信息数据
//			{
//				for(uint16_t i=0;i<tmpRead.stp23L.DataLen;i++)
//					stp23L_getdistance_callback(tmpRead.stp23L.Buf[i],&g_readonly_distance);
//			}		
//			//每处理1次stp23L数据，占用cpu时间约 0.2ms
//		}
//		
//		//雷达数据
//		else if( g_xQueueN10_Ori == xQueueDataHandle )
//		{		
//			/* 调试使用，观察队列中的数据是否被写满 */
//			//printf("N10 queue free space:%d\r\n",uxQueueSpacesAvailable(g_xQueueN10_Ori));
//			xQueueReceive(g_xQueueN10_Ori,&tmpRead.N10,0);//将队列中的数据赋值
//			
//			//雷达数据处理
//			if( tmpRead.N10.DataLen == 58 )		
//			{
//				N10_DataHandle(tmpRead.N10.Buf);	
//			}	
//		}
//	}
//}

