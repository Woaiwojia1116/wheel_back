#ifndef __DELAY_H
#define __DELAY_H
#include "main.h"

/* F103 裸机没有 TIM7 作 HAL 时基，延时与微秒时间戳统一用
 * Cortex-M3 的 DWT->CYCCNT（72MHz 下 72 计数 = 1us）。
 * 使用前必须先调用 Delay_Init()。 */
void     Delay_Init(void);
void     delay_us(uint32_t us);
void     delay_ms(uint32_t ms);
uint32_t micros(void);
#endif
