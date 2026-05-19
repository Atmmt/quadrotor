#include <math.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "balance_task.h"
#include "event_groups.h"
#include "bsp_uart.h"
#include "pid.h"

#include "show_task.h"

enum{
	AppKeyNone = 0,
	AppKeyFront = 1,
	AppKeyFR=2,
	AppKeyRight=3,
	AppKeyBR=4,
	AppKeyBack=5,
	AppKeyBL = 6,
	AppKeyLeft=7,
	AppKeyFL = 8,
	APPKeyUp = 0x18,
	APPKeyDown = 0x19,
};

__weak void SendToAPPFlag(void)
{
	
}

__weak void SaveFlashFlag(void)
{
	
}

//四轴事件
extern void StopFlyAction(void);
extern void StartFlyAction(void);
extern void FlyAction_EnterUNUSEHeightMode(void);
extern void FlyAction_ChangeMotorDir(void);

extern EventGroupHandle_t g_xEventFlyAction;
extern QueueHandle_t g_xQueueFlyControl;

extern float FlyControl_pitch;
extern float FlyControl_roll ;
extern float FlyControl_yaw  ;
extern float FlyControl_height  ;
extern float use_distance;
extern float userset_pitch;
extern float userset_roll;
extern short FlyControl_throttle;

extern QueueHandle_t g_xQueueBlueTooth_Pro;
 


typedef enum {
  STATE_WAIT_HEADER,  // 等待帧头
  STATE_GET_LENGTH,   // 获取长度字段
  STATE_GET_DATA,     // 获取数据字段
  STATE_GET_CHECKSUM, // 获取校验字段
  STATE_WAIT_TAIL     // 等待帧尾
} FrameState;
 







// 定义数据包结构体，包含三个指针成员
typedef struct {
    uint8 pack_head;
    uint8 pack_end;
    uint8 rece_data[200];                // 储存数据包数据
    uint8 recv ;                      //用于接收队列数据
    uint8 recv_cnt;
    uint8 rece_sum;
    uint8 Data_flag;
    uint16 Data_Len; 
    
    float setpitch,setroll,setyaw;
    
    uint8 * _bity_setup;
    uint8 * _bity;
    // 可使用的数据包中的bity
    
    short *_2bity_throttle;
    
    float *_4bity_setroll;
    float *_4bity_setpitch;
    float *_4bity_control_pitch;
    float *_4bity_control_roll;
    float *_4bity_control_yaw;
    float *_4bity_hight;
    
    float *_4bity_pitch_rate_p;
    float *_4bity_pitch_rate_i;
    float *_4bity_pitch_rate_d;
    float *_4bity_pitch_pos_p;
    float *_4bity_pitch_pos_i;
    float *_4bity_pitch_pos_d;
    
    float *_4bity_roll_rate_p;
    float *_4bity_roll_rate_i;
    float *_4bity_roll_rate_d;
    float *_4bity_roll_pos_p;
    float *_4bity_roll_pos_i;
    float *_4bity_roll_pos_d;
    
    float *_4bity_yaw_rate_p;
    float *_4bity_yaw_rate_i;
    float *_4bity_yaw_rate_d;
    float *_4bity_yaw_pos_p;
    float *_4bity_yaw_pos_i;
    float *_4bity_yaw_pos_d;
    
    float *_4bity_high_rate_p;
    float *_4bity_high_rate_i;
    float *_4bity_high_rate_d;
    float *_4bity_high_pos_p;
    float *_4bity_high_pos_i;
    float *_4bity_high_pos_d;
    
} Packet_uart;




extern PIDControllerType_t PitchRatePID;
extern PIDControllerType_t RollRatePID;

extern PIDControllerType_t PitchPID;
extern PIDControllerType_t RollPID;

extern PIDControllerType_t HeightSpeedPID ;
extern PIDControllerType_t HeightPID ;

extern PIDControllerType_t YawRatePID;
extern PIDControllerType_t YawPID;
void BluetoothAPPControl_task(void* param)
{
#if 0
	extern QueueHandle_t g_xQueueBlueTooth_Ori;
	extern void WriteFlyControlQueue(FlyControlType_t val,uint8_t writeEnv,BaseType_t* HigherPriorityTask);
	
	uint8_t recv = 0;     //用于接收队列数据
        uint8_t recv_pro = 0;     //用于接收队列数据
        
	uint8_t app_page = 1; //手机APP控制页的页数. 0:重力页面 1:摇杆页面 2:按键页面
	uint8_t controlKey = 0; //控制值
	
	//APP调参页面辅助参数
	static uint8_t paramFlag=0,param_i=0,param_j=0,paramReceive[50]={0};
	float paramData=0;
        

        
#endif
        
        extern QueueHandle_t g_xQueueBlueTooth_Ori;
        pSerialInterface_t user_uart1 = &UserDev_usart1;
        FrameState frame_state = STATE_WAIT_HEADER; // 初始状态
        
        
        uint8_t i;
        
        Packet_uart pkt = {0};
        
        
        pkt.pack_head = 0xA5;
        pkt.pack_end = 0x5A;
        pkt.Data_Len = 124;
        
        pkt._bity_setup                 =  &pkt.rece_data[0];  // 开关位
        pkt._bity                       = &pkt.rece_data[1];
        
        pkt._2bity_throttle             =  (short*)&pkt.rece_data[2];  // 占用2字节
       
        pkt._4bity_setroll              =  (float*)&pkt.rece_data[4];
        pkt._4bity_setpitch             =  (float*)&pkt.rece_data[8];
        pkt._4bity_control_pitch        =  (float*)(&pkt.rece_data[12]);
        pkt._4bity_control_roll         =  (float*)(&pkt.rece_data[16]);
        pkt._4bity_control_yaw          =  (float*)(&pkt.rece_data[20]);
        pkt._4bity_hight                =  (float*)(&pkt.rece_data[24]);
        
        
        pkt._4bity_pitch_rate_p        =  (float*)(&pkt.rece_data[28]);
        pkt._4bity_pitch_rate_i        =  (float*)(&pkt.rece_data[32]);
        pkt._4bity_pitch_rate_d        =  (float*)(&pkt.rece_data[36]);
        
        pkt._4bity_roll_rate_p         =  (float*)(&pkt.rece_data[40]);
        pkt._4bity_roll_rate_i         =  (float*)(&pkt.rece_data[44]);
        pkt._4bity_roll_rate_d         =  (float*)(&pkt.rece_data[48]);
        
        pkt._4bity_pitch_pos_p         =  (float*)(&pkt.rece_data[52]);
        pkt._4bity_pitch_pos_i         =  (float*)(&pkt.rece_data[56]);
        pkt._4bity_pitch_pos_d         =  (float*)(&pkt.rece_data[60]);
        
        pkt._4bity_roll_pos_p          =  (float*)(&pkt.rece_data[64]);
        pkt._4bity_roll_pos_i          =  (float*)(&pkt.rece_data[68]);
        pkt._4bity_roll_pos_d          =  (float*)(&pkt.rece_data[72]);
        
        pkt._4bity_high_rate_p         =  (float*)(&pkt.rece_data[76]);
        pkt._4bity_high_rate_i         =  (float*)(&pkt.rece_data[80]);
        pkt._4bity_high_rate_d         =  (float*)(&pkt.rece_data[84]);
        
        pkt._4bity_high_pos_p          =  (float*)(&pkt.rece_data[88]);
        pkt._4bity_high_pos_i          =  (float*)(&pkt.rece_data[92]);
        pkt._4bity_high_pos_d          =  (float*)(&pkt.rece_data[96]);
        
        pkt._4bity_yaw_rate_p         =  (float*)(&pkt.rece_data[100]);
        pkt._4bity_yaw_rate_i         =  (float*)(&pkt.rece_data[104]);
        pkt._4bity_yaw_rate_d         =  (float*)(&pkt.rece_data[108]);
        
        pkt._4bity_yaw_pos_p          =  (float*)(&pkt.rece_data[112]);
        pkt._4bity_yaw_pos_i          =  (float*)(&pkt.rece_data[116]);
        pkt._4bity_yaw_pos_d          =  (float*)(&pkt.rece_data[120]);
        
        
        
        FlyControlType_t controlVal= {0};
        
	while( 1 )
	{
//		user_uart1->transmit("abcd",4);
//                vTaskDelay(50);
                
		if( pdPASS == xQueueReceive(g_xQueueBlueTooth_Ori,&pkt.recv,portMAX_DELAY) )   // 解析数据
		{
//                  user_uart1->transmit(&recv,1);
                        switch(frame_state)
                        {
                              case STATE_WAIT_HEADER:    // 包头检验
                                    if(pkt.recv == pkt.pack_head) 
                                    {
                                          frame_state = STATE_GET_DATA;
                                          
                                          pkt.recv_cnt = pkt.Data_Len;
                                          i=pkt.Data_Len;
                                    
                                    }
                                          
                                    
                                break;  
                                
                                
                                case STATE_GET_DATA:    //  获取数据
                                      if(i>0)
                                      {
                                            pkt.rece_data[pkt.recv_cnt-i] = pkt.recv;
                                            pkt.rece_sum+=pkt.recv;
                                            i--;
                                            if(i == 0)
                                            {
                                              frame_state=STATE_GET_CHECKSUM;
                                            }
                                              
                                            
                                            
                                            break;
                                      }
                                      frame_state=STATE_WAIT_HEADER;
                                break;
                                
                                
                                
                                
                                case STATE_GET_CHECKSUM:   // 获取和校验数据
                                    if(pkt.rece_sum == pkt.recv)
                                    {
                                          pkt.rece_sum=0;
                                          frame_state=STATE_WAIT_TAIL;
                                          break;
                                    }
                                    frame_state=STATE_WAIT_HEADER;
                                break;
                                
                                

                                case STATE_WAIT_TAIL:    // 获取帧尾，保存数据
                                    if(pkt.recv == pkt.pack_end)
                                    {
//                                         user_uart1->transmit("5",1);
//                                          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                                          
//                                          xQueueSendFromISR(g_xQueueBlueTooth_Ori,&rece_data,&xHigherPriorityTaskWoken);
                                          
                                          pkt.Data_flag  = 1;
                                          
                                    }
                                    frame_state=STATE_WAIT_HEADER;
                                break;
                                
                                default:
                                  break;
                        }
                  
                }
                
                
                if( pkt.Data_flag  == 1)  // 读取到完整数据包，写入数据
		{
                  
                        pkt.Data_flag  = 0;
                        
//                        if( 1 == *pkt.bity_switch ) 
//                        {
//                            xEventGroupSetBits(g_xEventFlyAction,StartFly_Event);
//                        }
//                        else
//                        {
//                            xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);
//                        }
                        
                        controlVal.setup        = (*pkt._bity_setup)&0x01;
                        controlVal.height_setup = (*pkt._bity_setup)&0x02;
                        controlVal.pitch        = *pkt._4bity_control_pitch;
                        controlVal.roll         = *pkt._4bity_control_roll;
                        controlVal.gyroz        = *pkt._4bity_control_yaw;
                        
                        controlVal.height       = *pkt._4bity_hight;
                        controlVal.throttle     = *pkt._2bity_throttle;

                        controlVal.set_pitch    = *pkt._4bity_setpitch ;
                        controlVal.set_roll     = *pkt._4bity_setroll;
                        
                        
                        
                        
                        PitchRatePID.kp = *pkt._4bity_pitch_rate_p;
                        PitchRatePID.ki = *pkt._4bity_pitch_rate_i;
                        PitchRatePID.kd = *pkt._4bity_pitch_rate_d;
                        
                        
                        RollRatePID.kp = *pkt._4bity_roll_rate_p;
                        RollRatePID.ki = *pkt._4bity_roll_rate_i;
                        RollRatePID.kd = *pkt._4bity_roll_rate_d;
                        
                        PitchPID.kp = *pkt._4bity_pitch_pos_p;
                        PitchPID.ki = *pkt._4bity_pitch_pos_i;
                        PitchPID.kd = *pkt._4bity_pitch_pos_d;
                        
                        RollPID.kp = *pkt._4bity_roll_pos_p;
                        RollPID.ki = *pkt._4bity_roll_pos_i;
                        RollPID.kd = *pkt._4bity_roll_pos_d;
                        
                        HeightSpeedPID.kp = *pkt._4bity_high_rate_p;
                        HeightSpeedPID.ki = *pkt._4bity_high_rate_i;
                        HeightSpeedPID.kd = *pkt._4bity_high_rate_d;
                        
                        HeightPID.kp = *pkt._4bity_high_pos_p;
                        HeightPID.ki = *pkt._4bity_high_pos_i;
                        HeightPID.kd = *pkt._4bity_high_pos_d;
                        
                        
                        YawRatePID.kp = *pkt._4bity_yaw_rate_p;
                        YawRatePID.ki = *pkt._4bity_yaw_rate_i;
                        YawRatePID.kd = *pkt._4bity_yaw_rate_d;
                        
                        YawPID.kp = *pkt._4bity_yaw_pos_p;
                        YawPID.ki = *pkt._4bity_yaw_pos_i;
                        YawPID.kd = *pkt._4bity_yaw_pos_d;
                        
                        
                        
                        extern APPShowType_t appshow;
                        appshow.pitch_rate.p = *pkt._4bity_pitch_rate_p;
                        appshow.pitch_rate.i = *pkt._4bity_pitch_rate_i;
                        appshow.pitch_rate.d = *pkt._4bity_pitch_rate_d;
                        
                        appshow.roll_rate.p =  *pkt._4bity_roll_rate_p;
                        appshow.roll_rate.i =  *pkt._4bity_roll_rate_i;
                        appshow.roll_rate.d =  *pkt._4bity_roll_rate_d;
                        
                        
                        
                      xQueueOverwrite(g_xQueueFlyControl,&controlVal);
                        
                }
        }
}
                  


			#if 0 //WHEELTEC APP
			controlVal.source = APPCmd; //标记控制指令来源
			uint8_t writeFlag = 1;      //标记是否要写入控制队列
			
			/* APP按键页面切换 */
			if( recv == 'K' ) app_page = 2;      //按键页面
			else if( recv == 'J' ) app_page = 1; //摇杆页面
			else if( recv == 'I' ) app_page = 0; //重力球页面
			
			//读取控制信息
			controlKey = recv-0x40;
			
			//不接受重力球页的信息,难以控制
			if( app_page==0 ) controlKey = 0;
			
			//遥控幅度
			const float control_step = 5.0f;
			switch( controlKey )
			{
				case AppKeyFront: controlVal.roll = angle_to_rad(control_step);
					break;
				case AppKeyFR:    controlVal.roll = angle_to_rad(control_step);controlVal.pitch = -angle_to_rad(control_step);
					break;
				case AppKeyRight: 
					if( 2 == app_page )
					{
						controlVal.gyroz = angle_to_rad(60.0f);
					}
					else
						controlVal.pitch = -angle_to_rad(control_step);
					break;
				case AppKeyBR:   controlVal.roll = -angle_to_rad(control_step);controlVal.pitch = -angle_to_rad(control_step);
					break;
				case AppKeyBack: controlVal.roll = -angle_to_rad(control_step);
					break;
				case AppKeyBL:  controlVal.roll = -angle_to_rad(control_step);controlVal.pitch = angle_to_rad(control_step);
					break;
				case AppKeyLeft:
					if( 2==app_page )
					{
						controlVal.gyroz = -angle_to_rad(60.0f);
					}
					else
						controlVal.pitch = angle_to_rad(control_step);
					break;
				case AppKeyFL:  controlVal.roll = angle_to_rad(control_step);controlVal.pitch = angle_to_rad(control_step);
					break;
				case APPKeyUp: controlVal.height = 0.01f;
					break;
				case APPKeyDown: controlVal.height = -0.01f;
					break;
				default : writeFlag = 0;
					break;
			}
			
			//检查是否有控制,有控制则写入队列
			if( 1 == writeFlag ) WriteFlyControlQueue(controlVal,0,NULL);
			
			//APP参数页面 数据格式: {?:?}
			if(recv==0x7B) paramFlag=1;        //The start bit of the APP parameter instruction //APP参数指令起始位
			else if(recv==0x7D) paramFlag=2;   //The APP parameter instruction stops the bit    //APP参数指令停止位
			
			if(paramFlag==1) //Collect data //采集数据
			{
				paramReceive[(param_i%50)]=recv;
				param_i++;
			}
			else if(paramFlag==2) //Analyze the data //分析数据
			{
				if(paramReceive[3]==0x50) 	 SendToAPPFlag();      // {Q:P} 获取设备参数
				else if( paramReceive[3]==0x57 ) SaveFlashFlag(); // {Q:W} 设置掉电保存参数
				
				else if( paramReceive[1]=='M' && paramReceive[3]=='1' ) StartFlyAction(); //自定义协议：{M:1}
				else if( paramReceive[1]=='M' && paramReceive[3]=='2' ) StopFlyAction(); //自定义协议：{M:2}	
				else if( paramReceive[1]=='M' && paramReceive[3]=='3' ) FlyAction_EnterUNUSEHeightMode(); //自定义协议：{M:3}	
				
				else  if(paramReceive[1]!=0x23)                   // {0:xxx} {1:xxx} {2:xxx} 单通道数值设置
				{
					for(param_j=param_i; param_j>=4; param_j--)
					{
						paramData+=(paramReceive[param_j-1]-48)*pow(10,param_i-param_j);
					}
					switch(paramReceive[1])
					{
						case 0x30: break;
						case 0x31: break;
						case 0x32: break;
						case 0x33: break;
						case 0x34: break;
						case 0x35: break;
						case 0x36: break;
						case 0x37: break;
						case 0x38: break;
					}
				}
				else if( paramReceive[1]==0x23 ) //APP上点击“发送所有数据”处理方法  // {#xxx:xxx:xxx...xxx}
				{
					float num=0;
					uint8_t dataIndex=0;
					float dataArray[9]={0};

					if( param_i<=50 ) //数据在可接受范围
					{
						paramReceive[param_i]='}'; //补充帧尾

						for(uint8_t kk=0; paramReceive[kk]!='}'; kk++)
						{
							if( paramReceive[kk]>='0' && paramReceive[kk]<='9' )
							{
								num = num*10 + ( paramReceive[kk] - '0' );
							}
							else if( paramReceive[kk]==':' )
							{
								dataArray[dataIndex++] = num;
								num = 0;
							}

						}
						//处理最后一个数据
						dataArray[dataIndex] = num;
						
						//数据获取案例,例如 xxx = dataArray[0];
						UNUSED(dataArray[0]);
					}
				}

				//Relevant flag position is cleared
				//相关标志位清零
				paramFlag=0;param_i=0;param_j=0;paramData=0;
				memset(paramReceive, 0, sizeof(uint8_t)*50); //Clear the array to zero//数组清零
			}
		
		
		}
        }

}
#endif //WHEELTEC APP