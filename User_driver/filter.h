#ifndef __FILTER_H
#define __FILTER_H

#include "main.h"

/* 实例化一阶低通：状态放在滤波器对象里，多个通道（如左右电机）
 * 各用各的实例，互不串扰。原 LowPssFilter 用函数内 static，
 * 只适合单实例场景，保留给其它单通道模块使用。 */
typedef struct {
    float    state;      /* 上一拍输出 */
    uint32_t prev_us;    /* 上一拍时间戳（0 = 未初始化） */
    float    Tf;         /* 时间常数 s，越大越平滑 */
} LowPassFilter_t;

void  LowPassFilter_Init(LowPassFilter_t *f, float Tf);
float LowPassFilter_Update(LowPassFilter_t *f, float data);

float LowPssFilter(float data);
// float MedianFilter3(float data);
#endif /* __FILTER_H */
