#include "bsp_buzzer.h"
#include "zf_common_headfile.h"


static void BuzzerInit(void)
{
      gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);                        // 初始化 BUZZER_PIN 输出 默认低电平 推挽输出模式
}

static void BuzzerOn(void)
{
    gpio_set_level(BUZZER_PIN, 1);
}

static void BuzzerOff(void)
{
    gpio_set_level(BUZZER_PIN, 0);
}
static void Buzzeroggle(void)
{
    gpio_toggle_level(BUZZER_PIN);
}
static void BuzzerWaring(void)
{
      gpio_toggle_level(BUZZER_PIN);
      system_delay_ms(250);
      gpio_toggle_level(BUZZER_PIN);
      system_delay_ms(250);
      gpio_toggle_level(BUZZER_PIN);
      system_delay_ms(250);
      gpio_toggle_level(BUZZER_PIN);
}
BuzzerInterface_t UserBuzzer = {
    .init = BuzzerInit,
    .on = BuzzerOn,
    .off = BuzzerOff,
    .toggle = Buzzeroggle,
    .woring = BuzzerWaring
};

