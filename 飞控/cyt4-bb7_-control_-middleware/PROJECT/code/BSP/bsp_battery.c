#include "bsp_battery.h"

float g_robotVOL = 12.0f;

void BATTERY_Userconfig_Init(void)
{
    adc_init(ADC0_CH21_P07_5, ADC_12BIT);                                          // 建议电磁传感器用 ADC1 电池检测用 ADC2
}

//获取DMA搬运后的ADC值
float USER_BATTERY_Get_AdcBufValue(void)
{
  return (float)adc_convert(ADC0_CH21_P07_5) /4095.0 * 3.3 * 6;
}

BATTERYInterface_t UserBATTERY = {
	.init = BATTERY_Userconfig_Init,
	.getValue = USER_BATTERY_Get_AdcBufValue
};
