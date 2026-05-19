#include "bsp_motor.h"
#include "zf_common_headfile.h"

void MotorOFF(void)
{
    pwm_set_duty(MOTORA,1000);
    pwm_set_duty(MOTORB,1000);
    pwm_set_duty(MOTORC,1000);
    pwm_set_duty(MOTORD,1000);
}
void MotorInit(void)
{
      pwm_init(MOTORA,400,4000);
      pwm_init(MOTORB,400,4000);
      pwm_init(MOTORC,400,4000);
      pwm_init(MOTORD,400,4000);
}
void MotorSetDuty(uint32 A,uint32 B,uint32 C,uint32 D)
{
    pwm_set_duty(MOTORA,A);
    pwm_set_duty(MOTORB,B);
    pwm_set_duty(MOTORC,C);
    pwm_set_duty(MOTORD,D);
}


MOTORInterface_t UserMotor = {
    .init = MotorInit,
    .offall = MotorOFF,
    .setduty = MotorSetDuty
};