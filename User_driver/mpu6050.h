#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

/* 7 位器件地址，写地址即 0xD0；IIC 接口内部会自动左移 */
#define MPU6050_ADDR   0x68
#define PI_  3.14159f

typedef struct {
	int16_t Accx;
	int16_t Accy;
	int16_t Accz;
	int16_t Gyrox;
	int16_t Gyroy;
	int16_t Gyroz;
} MPU6050;

void MPU6050_Init(void);
uint8_t MPU6050_Read(uint8_t RegAdress);
void MPU6050_GetData(MPU6050 *MPU6050_Data);
void MPU6050_ReadBytes(uint8_t RegAdress, uint8_t *Buf, uint8_t Len);

/* 姿态解算：由主循环每 5ms 调用一次（互补滤波） */
void MPU6050_Update(void);

/* 姿态输出：倾角（度）与绕 Y 轴角速度（度/秒） */
float mpu6050_get_angle(void);
float mpu6050_get_rate(void);
#endif
