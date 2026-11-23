#include "lqr_ctrl.h"
#include "mpu6050.h"
#include "FOC.h"

/* ========================== LQR（已停用，保留备查） ========================== */
/* K1 由 0.09366 下调：44Hz 传感器带宽下原增益会高频自激，先找新的稳定上限 */
// static const float K[4] = {0.414f, 0.05249f, 0.02616f, 0.09509f};

/* 本周期控制量快照：主循环写，调试器/遥测读。
 * 串级结构下它是内环输出 Uq（V） */
static volatile float s_lqr_u = 0.0f;

/* ========================== 串级控制 ==========================
 * 外环（20ms）：速度 PI，把轮速误差换算成角度给定 theta_ref
 * 内环（5ms） ：角度 PD，直接输出 q 轴电压 Uq
 *
 * 符号沿用原已验证的方向（原：target = -(kp*angle + kd*rate)）：
 *   Uq = -(pd_kp*(angle - theta_ref) + pd_kd*rate)
 * ============================================================= */

/* 内环角度 PD（先给 0，调参从 pd_kp 开始） */
static float pd_kp = 0.4f;      /* V/deg */
static float pd_kd = 0.007f;      /* V/(deg/s) */
static float pd_ki = 0.0f;

/* 外环速度 PI（先给 0） */
static float sp_kp = 0.33f;      /* deg/(rad/s) */
static float sp_ki = 0.012f;      /* deg/(rad/s·s) */

static float sp_integral = 0.0f;
static float theta_ref = 0.0f;  /* 角度给定 deg */

#define OUTER_DIV    4          /* 4 × 5ms = 20ms */
#define OUTER_DT     0.02f
#define THETA_LIMIT  10.0f      /* 角度给定限幅 deg */
#define UQ_LIMIT     6.0f       /* 与电机 u_limit 一致 V */

/* 主循环每 5ms 调用一次（函数名沿用 LQR_Update，main.c 不用改） */
void LQR_Update(void)
{
    static float last_angle = 0;
    static float ki_para = 0;
    float angle = mpu6050_get_angle();      /* 倾角 deg */
    float rate  = mpu6050_get_rate();       /* 角速度 deg/s */

    /* ---------- 外环：速度 PI，20ms 一次 ---------- */
    static uint8_t div = 0;
    if (++div >= OUTER_DIV) {
        div = 0;

        /* 目标轮速先固定 0（原地平衡）；接遥控时把 0.0f 换成给定 */
        float v    = 0.5f * (Motor_GetSpeed(&motor_left)
                             + Motor_GetSpeed(&motor_right));
        float verr = v - 0.0f;

        if (sp_ki != 0.0f) {
            sp_integral += verr * OUTER_DT;
        }

        float theta = sp_kp * verr + sp_ki * sp_integral;
        if (theta > THETA_LIMIT) {
            theta = THETA_LIMIT;
            if (verr > 0.0f) sp_integral -= verr * OUTER_DT;
        } else if (theta < -THETA_LIMIT) {
            theta = -THETA_LIMIT;
            if (verr < 0.0f) sp_integral -= verr * OUTER_DT;
        }
        theta_ref = theta;
    }

    ki_para += (angle - last_angle);
    /* ---------- 内环：角度 PD，5ms 一次 ---------- */
    float uq = -(pd_kp * (angle - theta_ref) + pd_ki * ki_para + pd_kd * rate);
    if (uq >  UQ_LIMIT) uq =  UQ_LIMIT;
    if (uq < -UQ_LIMIT) uq = -UQ_LIMIT;

    /* 原结构（速度给定方式，已停用）：
     * omega_ref = pid_kp*angle + pid_kd*rate;
     * Motor_SetTarget(&motor_left, -omega_ref);
     */

    s_lqr_u = uq;
    Motor_SetUq(&motor_left, uq);
    Motor_SetUq(&motor_right, uq);

    last_angle = angle;
}

/* 供遥测读取本周期内环输出 Uq（V） */
float lqr_get_u(void) {return s_lqr_u;} 
