#include "show_task.h"

//C Lib
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "bsp_uart.h"

//RTOS
#include "FreeRTOS.h"
#include "task.h"




APPShowType_t appshow = { 0 }; //显示内容存储变量


void Send_USERData(int32 data);
void ShowTask(void* param)
{
    pSerialInterface_t debug_uart0 = &UserDev_usart0;
    pDebugInterface_t debug_anotc = &UserDev_Debug_ANOTC;


    ips200_show_string(0,16*0,"gyrox:");                ips200_show_string(8*16,16*0,"posX:  ");
    ips200_show_string(0,16*1,"gyroy:");                ips200_show_string(8*16,16*1,"posY:  ");
    ips200_show_string(0,16*2,"gyroz:");
    
    ips200_show_string(0,16*4,"accelx:");
    ips200_show_string(0,16*5,"accely:");
    ips200_show_string(0,16*6,"accelz:");
    
    ips200_show_string(0,16*8,"pitch:");
    ips200_show_string(0,16*9,"roll: ");
    ips200_show_string(0,16*10,"yaw:  ");

    ips200_show_string(0,16*12,"dis:  ");
    ips200_show_string(0,16*13,"A:  ");
    ips200_show_string(0,16*14,"B:  ");
    ips200_show_string(0,16*15,"C:  ");
    ips200_show_string(0,16*16,"D:  ");
    
    

    
    
    
    while (1)
    {
          ips200_show_float(8*6,16*0,appshow.gyrox,3,4);                ips200_show_float(8*22,16*0,appshow.posx,2,4);
          ips200_show_float(8*6,16*1,appshow.gyroy,3,4);                ips200_show_float(8*22,16*1,appshow.posy,2,4);
          ips200_show_float(8*6,16*2,appshow.gyroz,3,4);
          
          ips200_show_float(8*7,16*4,appshow.accelx,5,4);
          ips200_show_float(8*7,16*5,appshow.accely,5,4);
          ips200_show_float(8*7,16*6,appshow.accelz,5,4);
          
          ips200_show_float(8*6,16*8,appshow.pitch,3,4);
          ips200_show_float(8*6,16*9,appshow.roll,3,4);
          ips200_show_float(8*6,16*10,appshow.yaw,3,4);
          
          ips200_show_float(8*6,16*12,appshow.height,3,4);
          ips200_show_float(8*6,16*13,appshow.motorA_out,4,1);
          ips200_show_float(8*6,16*14,appshow.motorB_out,4,1);
          ips200_show_float(8*6,16*15,appshow.motorC_out,4,1);
          ips200_show_float(8*6,16*16,appshow.motorD_out,4,1);
          
//          Send_USERData(appshow.delta_x);
//          debug_uart0->transmit(data_to_send,9);
          debug_anotc->transmit_32biteFloat(0xF1,appshow.debug_date1);
          debug_anotc->transmit_32biteFloat(0xF2,appshow.debug_date2);
          
          debug_anotc->transmit_32biteFloat(0xF3,appshow.debug_date3);
          debug_anotc->transmit_32biteFloat(0xF4,appshow.debug_date4);
          
          debug_anotc->transmit_32biteFloat(0xF5,appshow.debug_date5);
          debug_anotc->transmit_32biteFloat(0xF6,appshow.debug_date6);

          debug_anotc->transmit_32biteFloat(0xF7,appshow.debug_date7);
          debug_anotc->transmit_32biteFloat(0xF8,appshow.debug_date8);
          
          vTaskDelay(50);
    }
  
}



//void Send_USERData(int32 data)
//{
//        
//	uint8 _cnt=0;
//	
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xF1;
//	data_to_send[_cnt++]=0x00;
//        data_to_send[_cnt++]=(char)(data>>24);
//	data_to_send[_cnt++]=(char)(data>>16);
//	data_to_send[_cnt++]=(char)(data>>8);
//	data_to_send[_cnt++]=(char)data;
//	
//        data_to_send[3]=_cnt-4;
//	uint8 sum = 0;
//	for(uint8 i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	
//	data_to_send[_cnt++]=sum;
//}