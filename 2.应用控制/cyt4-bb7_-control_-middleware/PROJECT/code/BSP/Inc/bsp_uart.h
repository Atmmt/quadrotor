#ifndef __BSP_USART_H
#define __BSP_USART_H

#include <stdint.h>
#include "zf_common_headfile.h"

typedef struct {
	void (*init)(uint32_t bound);
	void (*transmit)(const uint8_t *pData, uint16_t Size);
}SerialInterface_t,*pSerialInterface_t;

typedef struct {
	void (*transmit_32biteFloat)(uint8 channel,float data);
        void (*transmit_16biteInit)(uint8 channel,int16 data);
        void (*transmit_8biteUint)(uint8 channel,uint8 data);
}DebugInterface_t,*pDebugInterface_t;

extern SerialInterface_t UserDev_usart0,UserDev_uart1,UserDev_usart2,UserDev_usart4;
extern DebugInterface_t UserDev_Debug_ANOTC;


#endif
