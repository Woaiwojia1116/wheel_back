#ifndef __LQR_CTRL_H
#define __LQR_CTRL_H

#include "main.h"

/* 本周期内环输出 Uq（V），供遥测读取 */
float lqr_get_u(void);

/* 主循环每 5ms 调用一次：外环速度 PI（20ms 分频）修正角度给定，
 * 内环角度 PD 直接输出 q 轴电压（原 LQR 已注释停用，函数名保留） */
void LQR_Update(void);
#endif
