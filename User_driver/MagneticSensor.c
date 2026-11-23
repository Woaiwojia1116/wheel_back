#include "MagneticSensor.h"
#include "i2c.h"
#include "delay.h"
#include "foc_utils.h"
#include <stdio.h>

/* ========================== AS5600 参数 ========================== */
#define AS5600_ADDR       0x36
#define RAW_ANGLE_HI_REG  0x0C
#define AS5600_CPR        4096
#define HI2C_TIMEOUT_MS   10
#define HI2C_ADDR8        (AS5600_ADDR << 1)   /* HAL 使用 8 位格式地址 */

/* ========================== 全局变量 ========================== */
long          cpr                    = 0;
float         full_rotation_offset   = 0.0f;
long          angle_data_prev        = 0;
unsigned long velocity_calc_timestamp = 0;
float         angle_prev             = 0.0f;

/* ========================== 读原始角度（I2C1：PB6/PB7） ========================== */
/* 失败返回 0xFFFF */
unsigned short I2C_getRawCount(void)
{
    uint8_t buf[2];

    if (HAL_I2C_Mem_Read(&hi2c1, HI2C_ADDR8, RAW_ANGLE_HI_REG,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, HI2C_TIMEOUT_MS) != HAL_OK) {
        // printf("MagneticSensor: I2C read failed!\r\n");
        return 0xFFFF;
    }

    return (unsigned short)(((buf[0] << 8) | buf[1]) & 0x0FFF);
}

/* ========================== 初始化 ========================== */
void MagneticSensor_Init(void)
{
    cpr             = AS5600_CPR;
    angle_data_prev = I2C_getRawCount();
    if (angle_data_prev == 0xFFFF) {
        angle_data_prev = 0;      /* 读取失败，先给个默认值 */
        printf("MagneticSensor: I2C read failed!\r\n");
    }

    full_rotation_offset    = 0.0f;
    velocity_calc_timestamp = 0;
    angle_prev              = 0.0f;

    /* 通道二：硬件 I2C2（PB10/PB11）。设备不应答只告警，不阻塞启动；
     * 右轮未接编码器时需把 motor_right.enabled 置 0，避免无反馈闭环。 */
    if (MagneticSensor_HI2C_Init() != 0) {
        printf("MagneticSensor: hardware I2C2 channel not found!\r\n");
    }
}

/* ========================== 角度（含多圈累加） ========================== */
float getAngle(void)
{
    int32_t angle_data;
    int32_t d_angle;

    unsigned short raw = I2C_getRawCount();
    if (raw == 0xFFFF) {
        /* 读取失败：返回上一次的值，防止跳变 */
        return full_rotation_offset +
               ((float)angle_data_prev / (float)cpr) * _2PI;
    }
    angle_data = (int32_t)raw;

    d_angle = angle_data - (int32_t)angle_data_prev;

    /* 溢出判断：相邻两次差值超过半圈认为是跨圈 */
    if (d_angle >  (cpr / 2)) {
        full_rotation_offset -= _2PI;
    } else if (d_angle < -(cpr / 2)) {
        full_rotation_offset += _2PI;
    }

    angle_data_prev = angle_data;

    return full_rotation_offset +
           ((float)angle_data / (float)cpr) * _2PI;
}

/* ========================== 速度 ========================== */
/* 角度由调用方读好传入（与电角度共用同一次 I2C 读数，避免中断读两遍编码器） */
float getVelocity(float angle_c)
{
    unsigned long now_us;
    float Ts, vel;

    now_us = micros();
    /* 无符号减法天然处理 micros 溢出 */
    Ts = (float)(now_us - velocity_calc_timestamp) / 1000000.0f;
    if(velocity_calc_timestamp==0)
    {
        velocity_calc_timestamp = now_us;  
        angle_prev = angle_c;
        return 0.0f;
    }

    velocity_calc_timestamp = now_us;  
    if (Ts < 1e-6f || Ts > 0.5f) {
        Ts = 1e-3f;         /* 首次调用或异常，给 1ms */
    }

    vel = (angle_c - angle_prev) / Ts;

    angle_prev = angle_c;
    return vel;
}

/* ==================== 硬件 I2C2 读取路径（PB10/PB11，MX_I2C2_Init） ==================== */
/* 与上方 I2C1 路径完全平行：以下状态全部独立，两条路径可同时存在、互不污染。
 * 前置条件：main.c 中 MX_I2C2_Init() 先于本模块 init 执行（当前顺序已满足）。
 * 现阶段轮询读取：100kHz 下读 2 字节约 0.4ms，满足 2ms 控制周期；
 * hdma_i2c2_rx 已就绪，后续需要异步化时再启用。 */

static long          h_angle_data_prev      = 0;
static float         h_full_rotation_offset = 0.0f;
static unsigned long h_velocity_timestamp   = 0;
static float         h_angle_prev           = 0.0f;
static uint32_t      h_fail_cnt             = 0;   /* 连续失败计数，诊断用 */

/* 失败返回 0xFFFF，语义与 I2C_getRawCount 一致 */
unsigned short HI2C_getRawCount(void)
{
    uint8_t buf[2];

    if (HAL_I2C_Mem_Read(&hi2c2, HI2C_ADDR8, RAW_ANGLE_HI_REG,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, HI2C_TIMEOUT_MS) != HAL_OK) {
        h_fail_cnt++;
        return 0xFFFF;
    }
    h_fail_cnt = 0;
    return (unsigned short)(((buf[0] << 8) | buf[1]) & 0x0FFF);
}

/* 返回 0 = 设备应答且状态已复位；-1 = 总线/设备异常，调用方自行决定是否降级 */
int MagneticSensor_HI2C_Init(void)
{
    /* 硬件通道可能先于主通道初始化，cpr 必须有效才能换算角度 */
    if (cpr == 0) {
        cpr = AS5600_CPR;
    }

    unsigned short raw = HI2C_getRawCount();

    if (raw == 0xFFFF) {
        return -1;
    }
    h_angle_data_prev      = (long)raw;
    h_full_rotation_offset = 0.0f;
    h_velocity_timestamp   = 0;
    h_angle_prev           = 0.0f;
    h_fail_cnt             = 0;
    return 0;
}

/* 角度（含多圈累加），逻辑与 getAngle 相同，仅状态独立 */
float HI2C_getAngle(void)
{
    unsigned short raw = HI2C_getRawCount();
    long d_angle;

    if (raw == 0xFFFF) {
        /* 读取失败：返回上一次的值，防止跳变 */
        return h_full_rotation_offset +
               ((float)h_angle_data_prev / (float)cpr) * _2PI;
    }

    d_angle = (long)raw - h_angle_data_prev;

    /* 溢出判断：相邻两次差值超过半圈认为是跨圈 */
    if (d_angle >  (cpr / 2)) {
        h_full_rotation_offset -= _2PI;
    } else if (d_angle < -(cpr / 2)) {
        h_full_rotation_offset += _2PI;
    }

    h_angle_data_prev = (long)raw;

    return h_full_rotation_offset + ((float)raw / (float)cpr) * _2PI;
}

/* 速度，逻辑与 getVelocity 相同，仅状态独立（勿与 I2C1 路径交叉调用） */
float HI2C_getVelocity(float angle_c)
{
    unsigned long now_us = micros();
    float Ts, vel;

    Ts = (float)(now_us - h_velocity_timestamp) / 1000000.0f;
    if (h_velocity_timestamp == 0) {
        h_velocity_timestamp = now_us;
        h_angle_prev = angle_c;
        return 0.0f;
    }

    h_velocity_timestamp = now_us;
    if (Ts < 1e-6f || Ts > 0.5f) {
        Ts = 1e-3f;
    }

    vel = (angle_c - h_angle_prev) / Ts;

    h_angle_prev = angle_c;
    return vel;
}
