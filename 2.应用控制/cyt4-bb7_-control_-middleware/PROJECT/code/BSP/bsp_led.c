#include "bsp_led.h"
#include "zf_common_headfile.h"

static void Led_Init(void)
{
    gpio_init(LED_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);          // 初始化 LED1 输出 默认高电平 推挽输出模式
}

static void LedOn(void)
{
    gpio_set_level(LED_PIN, 1);
}

static void LedOff(void)
{
    gpio_set_level(LED_PIN, 0);
}

static void LedToggle(void)
{
    gpio_toggle_level(LED_PIN);
}


//驱动挂载
LedInterface_t UserLed = {
    .init = Led_Init,
    .on = LedOn,
    .off = LedOff,
    .toggle = LedToggle
};


