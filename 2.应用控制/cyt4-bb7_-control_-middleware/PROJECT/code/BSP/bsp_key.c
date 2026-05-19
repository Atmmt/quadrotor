#include "bsp_key.h"



static void KeyInit(void)
{
          /*按键初始化*/
    gpio_init(P20_3, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(P20_2, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(P20_1, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(P20_0, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入}

}
static uint8 KeyGet(void)
{
    uint8 Key_num = 0;
    if(gpio_get_level(KEY1) == 0)// 读取输入值
    {
        system_delay_ms(20); // 消除抖动
        while(gpio_get_level(KEY1) == 0);
        system_delay_ms(20);
        Key_num = 1;
    }
    if(gpio_get_level(KEY2)==0)
    {
        system_delay_ms(20);
        while(gpio_get_level(KEY2) == 0);
        system_delay_ms(20);
        Key_num = 2;
    }
    if(gpio_get_level(KEY3)==0)
    {
        system_delay_ms(20);
        while(gpio_get_level(KEY3) == 0);
        system_delay_ms(20);
        Key_num = 3;
    }
    if(gpio_get_level(KEY4)==0)
    {
        system_delay_ms(20);
        while(gpio_get_level(KEY4) == 0);
        system_delay_ms(20);
        Key_num = 4;
    }
    return Key_num;
}

KEYInterface_t UserKEY = {
    .init = KeyInit,
    .getValue = KeyGet
};

