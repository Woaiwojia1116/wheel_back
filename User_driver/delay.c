#include "delay.h"

static uint32_t s_cycles_per_us = 72u;

void Delay_Init(void)
{
    /* 使能 DWT 周期计数器：72MHz 下 1us = 72 个计数 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    s_cycles_per_us = SystemCoreClock / 1000000u;
    if (s_cycles_per_us == 0u) {
        s_cycles_per_us = 72u;
    }
}

void delay_us(uint32_t us)
{
    if (us == 0u) return;

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * s_cycles_per_us;
    /* 无符号差值天然处理 CYCCNT 32 位回绕 */
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) { }
}

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000u);
    }
}

uint32_t micros(void)
{
    /* CYCCNT 约 59.6s 回绕一次；编码器/滤波只用相邻差值，
     * 无符号减法在回绕处仍然正确。 */
    return DWT->CYCCNT / s_cycles_per_us;
}
