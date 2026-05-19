#ifndef __BSP_BATTERY_H
#define __BSP_BATTERY_H

#include "zf_driver_adc.h"

typedef struct {
    void (*init)(void);
    float (*getValue)(void);
}BATTERYInterface_t,*pBATTERYInterface_t;

extern BATTERYInterface_t UserBATTERY;
extern float g_robotVOL;

#endif
