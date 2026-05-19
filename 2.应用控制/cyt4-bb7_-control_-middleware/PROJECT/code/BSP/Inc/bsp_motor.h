#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "zf_common_headfile.h"


#define MOTORA TCPWM_CH09_P05_0
#define MOTORB TCPWM_CH10_P05_1
#define MOTORC TCPWM_CH11_P05_2
#define MOTORD TCPWM_CH12_P05_3

typedef struct {
	void (*init)(void);
        void (*offall)(void);
        void (*setduty)(uint32 A,uint32 B,uint32 C,uint32 D);
}MOTORInterface_t,*pMOTORInterface_t;


extern MOTORInterface_t UserMotor;
#endif
