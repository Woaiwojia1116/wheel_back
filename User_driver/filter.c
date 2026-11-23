#include "filter.h"
#include "delay.h"


/* ========================== 实例化低通 ========================== */
void LowPassFilter_Init(LowPassFilter_t *f, float Tf_s)
{
    if (f == NULL) return;
    f->state   = 0.0f;
    f->prev_us = 0;
    f->Tf      = Tf_s;
}

/* 一阶低通：alpha 随实际 dt 计算，采样周期波动时截止频率仍然恒定 */
float LowPassFilter_Update(LowPassFilter_t *f, float data)
{
    if (f == NULL) return data;

    unsigned long now_us = micros();

    if (f->prev_us == 0) {
        f->state   = data;      /* 首拍直接用实测值填满 */
        f->prev_us = now_us;
        return data;
    }

    float dt = (now_us - f->prev_us) * 1e-6f;
    f->prev_us = now_us;

    float alpha = f->Tf / (f->Tf + dt);
    data = alpha * f->state + (1.0f - alpha) * data;

    f->state = data;
    return data;
}
