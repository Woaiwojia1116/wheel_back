#include "FOC.h"
#include <math.h>
#include <stdio.h>
#include "tim.h"
#include "usart.h"
#include "MagneticSensor.h"
#include "foc_utils.h"
#include "delay.h"
#include "mpu6050.h"
#include "lqr_ctrl.h"

#define ZERO_ANGLE_CAL_V      3.0f     /* 标定施加的 q 轴电压 V */
#define ZERO_ANGLE_CAL_DELAY  3000     /* 转子对齐时间 ms */

float u_ = 0,v_ = 0,w_ = 0;
/* ========================== 电机实例 ========================== */
/* 左轮：硬件 I2C1 读 AS5600（PB6/PB7），TIM1 三相 PWM（PA8/PA9/PA10）
 * 右轮：硬件 I2C2 读 AS5600（PB10/PB11），TIM3 三相 PWM（PA6/PA7/PB0）
 *
 * 两轮机械镜像安装时，编码器正方向与电角度方向必然不同，只改对应
 * 实例的 enc_dir / foc_dir 即可，控制逻辑与其它参数不用动 */

Motor_t motor_left = {
    .name       = "motor_left",
    .htim       = &htim1,
    .pwm_ch     = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3 },
    .enc        = { getAngle, getVelocity },      /* 硬件 I2C1 通道 */
    .enc_dir    = 1,
    .foc_dir    = 1,
    .pole_pairs = 7,
    .wheel_r_mm = 11.0f,
    .vbus       = 12.0f,
    .u_limit    = 6.0f,
    .dt         = 0.002f,                          /* TIM2: 72MHz/72/2000 = 500Hz */
    .kp         = 0.9f,
    .ki         = 0.1f,
    .kd         = 0.0f,
    .speed_lpf  = { .state = 0.0f, .prev_us = 0, .Tf = 0.02f },
    .enabled    = 1,
};


Motor_t motor_right = {
    .name       = "motor_right",
    .htim       = &htim3,
    .pwm_ch     = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3 },
    .enc        = { HI2C_getAngle, HI2C_getVelocity }, /* 硬件 I2C2 通道 */
    .enc_dir    = -1,
    .foc_dir    = -1,
    .pole_pairs = 7,
    .wheel_r_mm = 11.0f,
    .vbus       = 12.0f,
    .u_limit    = 6.0f,                                                                                     
    .dt         = 0.002f,

    .kp         = 0.9f,
    .ki         = 0.1f,
    .kd         = 0.0f,
    .speed_lpf  = { .state = 0.0f, .prev_us = 0, .Tf = 0.02f },
    .enabled    = 1,
};

/* ========================== 逆变输出 ========================== */
/* 输出三相占空比：入参为相对母线中点的相电压 */
static void setPwm(Motor_t *m, float Ua, float Ub, float Uc)
{
    /* 不同定时器的 ARR 可以不同，按实际周期换算比较值 */
    const float period = (float)(m->htim->Init.Period + 1);

    float dc_a = _constrain(Ua / m->vbus, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / m->vbus, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / m->vbus, 0.0f, 1.0f);

    __HAL_TIM_SetCompare(m->htim, m->pwm_ch[0], dc_a * period);
    __HAL_TIM_SetCompare(m->htim, m->pwm_ch[1], dc_b * period);
    __HAL_TIM_SetCompare(m->htim, m->pwm_ch[2], dc_c * period);
}

/* 电角度 = 机械角度 × 极对数 × 方向 - 零电角 */
static float _electricalAngle(const Motor_t *m, float mech_angle)
{   
    return _normalizeAngle((float)(m->foc_dir * m->pole_pairs) * mech_angle
                           - m->zero_electric_angle);
}

/* Park 逆变换 + Clark 逆变换，叠加中点偏移（SVPWM 马鞍波） */
static void setPhaseVoltage(Motor_t *m, float Uq, float Ud, float angle_el)
{
    (void)Ud;                    /* 当前只做 q 轴电压控制，d 轴保留 */

    angle_el = _normalizeAngle(angle_el);

    float Ualpha = -Uq * sin(angle_el);
    float Ubeta  =  Uq * cos(angle_el);

    float Ua = Ualpha;
    float Ub = (sqrt(3) * Ubeta - Ualpha) / 2;
    float Uc = (-Ualpha - sqrt(3) * Ubeta) / 2;

    float Umax = fmaxf(Ua, fmaxf(Ub, Uc));
    float Umin = fminf(Ua, fminf(Ub, Uc));

    float Uoffect = -(Umax + Umin) / 2.0f;
    float vbus_half = m->vbus / 2.0f;

    u_ = Ua + Uoffect + vbus_half;
    v_ = Ub + Uoffect + vbus_half;
    w_ = Uc + Uoffect + vbus_half;
    setPwm(m,
           Ua + Uoffect + vbus_half,
           Ub + Uoffect + vbus_half,
           Uc + Uoffect + vbus_half);

}

/* ========================== 生命周期 ========================== */
void Motor_Calibrate(Motor_t *m)
{
    if (m == NULL) return;

    /* 标定期间速度环不得插手；支持运行时重新标定（任务上下文调用） */
    uint8_t was_ready = m->ready;
    m->ready = 0;

    /* 固定电角度施加电压，把转子拉到已知位置（阻塞，仅开机路径可接受） */
    setPhaseVoltage(m, ZERO_ANGLE_CAL_V, 0, _3PI_2);
    delay_ms(ZERO_ANGLE_CAL_DELAY);

    /* 此刻 zero 仍未知，直接按 zero=0 计算；重复标定不会叠加误差 */
    float mech = m->enc.get_angle();
    m->zero_electric_angle =
        _normalizeAngle((float)(m->foc_dir * m->pole_pairs) * mech);

    setPhaseVoltage(m, 0, 0, _3PI_2);

    /* 标定期间转子被强制拖动，历史控制状态全部失效 */
    m->integral     = 0.0f;
    m->last_err     = 0.0f;
    m->last_dir     = 0;
    m->angle_valid  = 0;
    m->uq           = 0.0f;
    m->speed_lpf.state   = 0.0f;
    m->speed_lpf.prev_us = 0;

    m->ready = was_ready;

    printf("%s: zero_elec=%.4f\r\n", m->name, m->zero_electric_angle);
}

void Motor_Init(Motor_t *m)
{
    if (m == NULL || m->htim == NULL || m->enc.get_angle == NULL || !m->enabled) {
        return;
    }

    HAL_TIM_PWM_Start(m->htim, m->pwm_ch[0]);
    HAL_TIM_PWM_Start(m->htim, m->pwm_ch[1]);
    HAL_TIM_PWM_Start(m->htim, m->pwm_ch[2]);

    Motor_Calibrate(m);

    m->ready = 1;
}

void Motor_Deinit(Motor_t *m)
{
    if (m == NULL) return;

    /* 零矢量（三相 50% 占空比）释放转矩，再关 PWM 输出 */
    setPhaseVoltage(m, 0, 0, 0);
    HAL_TIM_PWM_Stop(m->htim, m->pwm_ch[0]);
    HAL_TIM_PWM_Stop(m->htim, m->pwm_ch[1]);
    HAL_TIM_PWM_Stop(m->htim, m->pwm_ch[2]);

    m->ready = 0;
}

/* ========================== 速度环 ========================== */
void Motor_SetTarget(Motor_t *m, float rad_s)
{
    if (m == NULL) return;
    m->target = rad_s;
}

/* 串级结构：角度环直接下发 q 轴电压，中断里只做换相 */
void Motor_SetUq(Motor_t *m, float uq)
{
    if (m == NULL) return;
    m->uq = uq;
}

/*
    @brief FOC 换相 + 输出电压，控制周期 = m->dt（TIM2 中断里逐个电机调用）
    @param  电压指令由角度环通过 Motor_SetUq 下发（串级结构）
*/
static float eangle = 0;
void Motor_ControlTick(Motor_t *m)
{
    if (m == NULL || !m->enabled || !m->ready) return;

    /* 整个控制周期只读一次编码器：速度和电角度共用同一读数。
       软件 I2C 一次约 0.5ms，读两遍会吞掉整个控制周期，
       饿死 SysTick/TIM7(优先级15) 导致调度器停摆 */
    float shaft_angle = m->enc.get_angle();
    float vel = m->enc.get_velocity(shaft_angle);
    vel = LowPassFilter_Update(&m->speed_lpf, vel);

    /* 串级结构：速度环已移到外环（lqr_ctrl.c），这里只做换相，
       q 轴电压直接用角度环下发的值 */
    float u_cmd = m->uq;
    if (u_cmd >  m->u_limit) u_cmd =  m->u_limit;
    if (u_cmd < -m->u_limit) u_cmd = -m->u_limit;

    /* 位移：角度×半径逐拍累加 弧度单位*/
    m->displacement += m->enc_dir * ((shaft_angle - m->last_angle));

    /* 单拍差分速度：不依赖滤波，调试时与滤波值对照 */
    if (!m->angle_valid) {
        m->angle_valid = 1;
        m->tel_vel = 0.0f;
    } else {
        m->tel_vel = (float)m->enc_dir
                   * (shaft_angle - m->last_angle) / m->dt;
    }

    m->last_angle = shaft_angle;

    eangle = _electricalAngle(m, shaft_angle);
    setPhaseVoltage(m, u_cmd, 0,eangle);

    /* 遥测快照：只用本轮已有的计算结果，不增加 I2C 开销 */
    m->tel_speed = (float)m->enc_dir * vel;
    m->tel_u     = u_cmd;

    /* 原速度内环（已停用，保留备查）：
     * float err = m->target - (float)m->enc_dir * vel;
     * int8_t dir = (int8_t)((m->target > 0.0f) - (m->target < 0.0f));
     * if (dir != 0) {
     *     if (m->last_dir != 0 && dir != m->last_dir) m->integral = 0.0f;
     *     m->last_dir = dir;
     * }
     * float u = m->kp * err + m->ki * m->integral + m->kd * (err - m->last_err);
     * float sat = m->u_limit - 0.1f;
     * if (!((u > sat && err > 0.0f) || (u < -sat && err < 0.0f))) {
     *     m->integral += err * 0.002;
     * }
     * m->tel_err = err;
     * m->last_err = err;
     */
}

/* ========================== 遥测 getter ========================== */
float Motor_GetTarget(const Motor_t *m)     { return (m != NULL) ? m->target : 0.0f; }
float Motor_GetSpeed(const Motor_t *m)      { return (m != NULL) ? m->tel_speed : 0.0f; }
float Motor_GetErr(const Motor_t *m)        { return (m != NULL) ? m->tel_err : 0.0f; }
float Motor_GetU(const Motor_t *m)          { return (m != NULL) ? m->tel_u : 0.0f; }
float Motor_GetVelocity(const Motor_t *m)   { return (m != NULL) ? m->tel_vel : 0.0f; }
float Motor_GetDisplacement(const Motor_t *m)
{
    return (m != NULL) ? (m->displacement * 0.002f) : 0.0f;   /* mm → m */
}

/* ========================== 编码器串口遥测 ========================== */
/* 在 TIM2 中断里按固定分频把两轮编码器角度、速度发到 USART2（PA2，115200）。
 * 帧格式（CSV，一行四列）：
 *   L_angle,L_speed,R_angle,R_speed\r\n     单位：rad, rad/s
 * 角度为编码器原始机械角（含多圈），速度已按 enc_dir 统一为前进正方向。
 *
 * 注意：不能在 1ms 控制中断里用 printf 阻塞发送。115200 下一帧约 3ms，
 * 会直接占满控制周期。这里只用整数定点格式化，然后交给
 * HAL_UART_Transmit_IT 在串口中断里后台搬运；上一帧没发完就丢弃本帧。 */
#define FOC_TELE_DIV       20u    /* 每 20 个控制周期发一帧（TIM2 1kHz → 50Hz） */
#define FOC_TELE_BUF_SIZE  48u

static char     s_tele_buf[FOC_TELE_BUF_SIZE];
static uint16_t s_tele_cnt;

/* 无符号整数转字符串，返回写入长度 */
static int tele_put_uint(char *p, uint32_t v)
{
    char tmp[10];
    int n = 0;
    int len = 0;

    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u);

    while (n > 0) {
        p[len++] = tmp[--n];
    }
    return len;
}

/* 浮点转定点字符串（四舍五入），避免在中断里走 newlib 的浮点 printf */
static int tele_put_fixed(char *p, float v, uint8_t decimals)
{
    static const int32_t scale_tab[4] = { 1, 10, 100, 1000 };
    int32_t scale = scale_tab[decimals];
    int32_t x = (int32_t)(v * (float)scale
                          + ((v >= 0.0f) ? 0.5f : -0.5f));
    int len = 0;

    if (x < 0) {
        p[len++] = '-';
        x = -x;
    }

    len += tele_put_uint(&p[len], (uint32_t)x / (uint32_t)scale);

    if (decimals != 0u) {
        uint32_t frac = (uint32_t)x % (uint32_t)scale;

        p[len++] = '.';
        for (int32_t div = scale / 10; div > 0; div /= 10) {
            p[len++] = (char)('0' + (frac / (uint32_t)div) % 10u);
        }
    }
    return len;
}

static void FOC_Telemetry_Send(void)
{
    /* 串口还在发上一帧：丢弃本帧，绝不阻塞控制中断 */
    if (huart2.gState != HAL_UART_STATE_READY) {
        return;
    }

 
        int len = 0;

        len += tele_put_fixed(&s_tele_buf[len], motor_left.target, 3);
        s_tele_buf[len++] = ',';
        len += tele_put_fixed(&s_tele_buf[len], motor_left.tel_speed, 2);
        s_tele_buf[len++] = ',';
        len += tele_put_fixed(&s_tele_buf[len], motor_left.tel_u, 3);
        s_tele_buf[len++] = ',';
        len += tele_put_fixed(&s_tele_buf[len], motor_left.tel_err, 2);
        s_tele_buf[len++] = '\r';
        s_tele_buf[len++] = '\n';     
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)s_tele_buf, (uint16_t)len);   
}

/* ========================== TIM2 2ms 速度环 ========================== */
/* TIM2 更新中断：两个电机串行跑速度环，并按分频输出编码器遥测 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        Motor_ControlTick(&motor_left);
        Motor_ControlTick(&motor_right);

        if (++s_tele_cnt >= FOC_TELE_DIV) {
            s_tele_cnt = 0;
            // FOC_Telemetry_Send();
            // printf("%f\r\n",lqr_get_u());
            // printf("%f %f\r\n",motor_right.target,motor_right.tel_speed);
        }
    }
}

