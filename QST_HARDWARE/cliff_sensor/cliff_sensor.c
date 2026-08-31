#include "cliff_sensor.h"

/**
 * @brief  初始化车头底板两侧 TCRT5000 光电传感器 (PA11 / PA12)
 */
void Cliff_Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(CLIFF_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = CLIFF_PIN_L | CLIFF_PIN_R;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CLIFF_PORT, &GPIO_InitStructure);
}

/**
 * @brief  检测车头是否悬空 / 处于边缘跌落危险
 * @retval 1: 检测到边缘悬空 (需要避险后退); 0: 正常平地安全
 */
u8 Check_Is_Cliff(void)
{
    // PA11 或 PA12 为高电平说明探头悬空未接收到地面反射
    if (CLIFF_LEFT_VAL == Bit_SET || CLIFF_RIGHT_VAL == Bit_SET)
    {
        return 1;
    }
    return 0;
}

