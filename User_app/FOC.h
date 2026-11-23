#ifndef FOC_H
#define FOC_H

#include "main.h"
#include "filter.h"

/* ============================================================
 * 电机实例结构体
 *
 * 每个电机一份 Motor_t：原来散落在 FOC.c 里的全局量（极对数、
 * 零电角、积分、上次误差、滤波状态、PWM 定时器…）全部收进
 * 结构体。控制/遥测函数一律用 Motor_t* 指定操作对象，增加电机
 * 只需再定义一个实例并填好硬件资源，不必复制控制逻辑。
 *
 * 约定：target 的正方向 = 轮子前进方向。左右轮机械镜像安装
 * 时，用各自实例的 enc_dir / foc_dir 把方向统一过来。
 * ============================================================ */

/* 编码器接口：不同电机接不同的 AS5600 通道 */
typedef struct {
    float (*get_angle)(void);             /* 机械角度 rad（含多圈累加） */
    float (*get_velocity)(float angle_c); /* rad/s，入参为本次读到的角度 */
} MotorEncoder_t;

typedef struct {
    const char        *name;

    /* ---- 硬件资源 ---- */
    TIM_HandleTypeDef *htim;      /* 三相 PWM 定时器 */
    uint32_t           pwm_ch[3]; /* 三相通道，顺序 A/B/C */

    /* ---- 编码器 ---- */
    MotorEncoder_t enc;
    int8_t  enc_dir;      /* 编码器方向 +1/-1：让速度正方向与目标正方向一致 */
    int8_t  foc_dir;      /* 电角度方向 +1/-1：与相序/接线对应 */
    uint8_t pole_pairs;   /* 极对数 */
    float   wheel_r_mm;   /* 轮半径 mm，位移换算用 */

    /* ---- 电气参数 ---- */
    float vbus;                /* 母线电压 V */
    float u_limit;             /* q 轴电压指令限幅 V */
    float zero_electric_angle; /* 零电角，标定后写入 */
    float dt;                  /* 控制周期 s，与 TIM2 中断周期一致 */

    /* ---- 速度环 ---- */
    float kp, ki, kd;
    LowPassFilter_t speed_lpf; /* 速度低通，每电机独立状态 */

    /* ---- 运行状态（控制环维护） ---- */
    float target;         /* 目标速度 rad/s（旧速度环用，当前串级结构未使用） */
    float integral;       /* 积分累积（未乘 ki） */
    float last_err;
    int8_t last_dir;      /* 上一次非零目标方向，用于换向清积分 */
    float last_angle;     /* 上一拍机械角度 */
    uint8_t angle_valid;  /* 已采到首拍角度 */
    float displacement;   /* 位移累加 mm（带符号） */

    /* ---- 串级结构的内环输出 ---- */
    volatile float uq;    /* q 轴电压指令 V：角度环写、TIM2 中断读 */

    /* ---- 遥测快照（中断写、任务读；32 位对齐 float 单次读写原子） ---- */
    volatile float tel_err;
    volatile float tel_speed;
    volatile float tel_u;
    volatile float tel_vel;   /* 单拍差分速度 rad/s（不滤波，调试用） */

    uint8_t enabled;          /* 0 = 跳过初始化与控制（未接电机/编码器时置 0） */
    uint8_t ready;            /* 1 = PWM 已启动且零电角已标定 */
} Motor_t;

/* ---- 电机实例（定义在 FOC.c） ---- */
extern Motor_t motor_left;    /* TIM1 三相 PWM + I2C1(PB6/PB7) AS5600 */
extern Motor_t motor_right;   /* TIM3 三相 PWM + I2C2(PB10/PB11) AS5600 */

/* ---- 生命周期 ---- */
void Motor_Init(Motor_t *m);       /* 启动 PWM + 零电角标定（阻塞约 3s/电机） */
void Motor_Deinit(Motor_t *m);     /* 零矢量停转 + 关闭 PWM */
void Motor_Calibrate(Motor_t *m);  /* 单独重新标定零电角 */

/* ---- 控制 ---- */
void Motor_SetTarget(Motor_t *m, float rad_s);  /* 速度环目标（当前串级结构未使用） */
void Motor_SetUq(Motor_t *m, float uq);         /* 角度环下发 q 轴电压 V，任意上下文 */
void Motor_ControlTick(Motor_t *m);             /* FOC 换相 + 输出电压，TIM2 中断上下文 */

/* ---- 遥测 getter（任务上下文） ---- */
float Motor_GetTarget(const Motor_t *m);
float Motor_GetSpeed(const Motor_t *m);         /* 滤波后速度 rad/s */
float Motor_GetErr(const Motor_t *m);           /* 速度环误差 rad/s */
float Motor_GetU(const Motor_t *m);             /* 限幅后的电压指令 V */
float Motor_GetVelocity(const Motor_t *m);      /* 单拍差分速度 rad/s */
float Motor_GetDisplacement(const Motor_t *m);  /* 位移 m */

#endif /* FOC_H */
