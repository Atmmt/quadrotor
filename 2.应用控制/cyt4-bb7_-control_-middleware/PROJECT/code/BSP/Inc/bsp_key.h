#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#define KEY1                    (P20_3)
#define KEY2                    (P20_2)
#define KEY3                    (P20_1)
#define KEY4                    (P20_0)

#include "zf_common_headfile.h"

typedef struct {
    void (*init)(void);
    uint8 (*getValue)(void);
}KEYInterface_t,*pKEYInterface_t;

extern KEYInterface_t UserKEY;

#endif
