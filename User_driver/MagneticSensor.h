#ifndef MAGNETICSENSOR_H
#define MAGNETICSENSOR_H

#include "main.h"
#define M1_AS5600    1
/******************************************************************************/
extern long  cpr;
extern float full_rotation_offset;
extern long  angle_data_prev;
extern unsigned long velocity_calc_timestamp;
extern float angle_prev;
/******************************************************************************/
void MagneticSensor_Init(void);
float getAngle(void);
float getVelocity(float angle_c);
/******************************************************************************/
/* 硬件 I2C2 路径（PB10/PB11）：与上面 I2C1 路径独立，状态不共享。
 * 依赖 MX_I2C2_Init() 先行；init 返回 -1 表示设备不应答。 */
int            MagneticSensor_HI2C_Init(void);
unsigned short HI2C_getRawCount(void);
float          HI2C_getAngle(void);
float          HI2C_getVelocity(float angle_c);
/******************************************************************************/

#endif
