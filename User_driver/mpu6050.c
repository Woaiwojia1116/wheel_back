#include "mpu6050.h"
#include "IIC.h"
#include <math.h>

/* MPU6050 挂在 PA0(SCL) / PA1(SDA)，见 IIC_BUS_MPU6050 */
#define MPU6050_GYRO_LSB_PER_DPS   16.4f   // ±2000dps 量程
/* 文件级姿态输出（供 LQR 等控制模块读取）。
 * 32 位对齐 float 在 CM3 上单次读写是原子的，跨上下文读无需加锁 */
static float s_att_angle = 0.0f;    /* 互补滤波后的倾角 (度) */
static float s_att_rate  = 0.0f;    /* 绕 Y 轴角速度 (度/秒) */

void MPU6050_Write(uint8_t RegAdress, uint8_t Data)
{
	IIC_WriteReg(&IIC_BUS_MPU6050, MPU6050_ADDR, RegAdress, Data);
}

uint8_t MPU6050_Read(uint8_t RegAdress)
{
	uint8_t Data = 0;
	IIC_ReadRegs(&IIC_BUS_MPU6050, MPU6050_ADDR, RegAdress, &Data, 1);
	return Data;
}

/* 从 RegAdress 开始连续读 Len 个字节到 Buf */
void MPU6050_ReadBytes(uint8_t RegAdress, uint8_t *Buf, uint8_t Len)
{
	IIC_ReadRegs(&IIC_BUS_MPU6050, MPU6050_ADDR, RegAdress, Buf, Len);
}

void MPU6050_Init(void)
{
	IIC_Init(&IIC_BUS_MPU6050);
	MPU6050_Write(0x6B, 0x01);
	MPU6050_Write(0x6C, 0x00);
	MPU6050_Write(0x19, 0x00);   /* 采样分频 0：陀螺 ODR = 1kHz，5ms 控制拍每次都能取到新样本 */
	MPU6050_Write(0x1B, 0x18);
	MPU6050_Write(0x1A, 0x04);   /* DLPF 21Hz：群延迟约 8.5ms，在 5Hz/19ms 与 44Hz/4.9ms 之间折中 */
	MPU6050_Write(0x1C, 0x18);
}

void MPU6050_GetData(MPU6050 *MPU6050_Data)
{
	uint8_t buf[14];
	MPU6050_ReadBytes(0x3B, buf, 14); 

	MPU6050_Data->Accx  = (int16_t)((buf[0]  << 8) | buf[1]);   // 0x3B
	MPU6050_Data->Accy  = (int16_t)((buf[2]  << 8) | buf[3]);   // 0x3D
	MPU6050_Data->Accz  = (int16_t)((buf[4]  << 8) | buf[5]);   // 0x3F
	// buf[6],buf[7] 是温度 0x41/0x42，跳过
	MPU6050_Data->Gyrox = (int16_t)((buf[8]  << 8) | buf[9]);   // 0x43
	MPU6050_Data->Gyroy = (int16_t)((buf[10] << 8) | buf[11]);  // 0x45
	MPU6050_Data->Gyroz = (int16_t)((buf[12] << 8) | buf[13]);  // 0x47
}

/* ========================== 姿态解算（原 5ms 慢tick） ========================== */
void MPU6050_Update(void)
{
	static float Angle_xz  = 0;
	static float Angle_gyro = 0;
	static float Angle      = 0;
	static float Alpha      = 0.01;
	static MPU6050 _M;

	static uint32_t last_tick = 0;
	uint32_t now = HAL_GetTick();
	
	float dt = (now - last_tick)*0.001f;//孩子们，注意变量类型，都是泪

	if (last_tick == 0) dt = 5.0f;

	MPU6050_GetData(&_M);
	Angle_xz   = -atan2((float)_M.Accx, (float)_M.Accz) * 180 / PI_;
	Angle_gyro = Angle + _M.Gyroy / MPU6050_GYRO_LSB_PER_DPS * dt;
	Angle      = Alpha * (Angle_xz - Angle_gyro) + Angle_gyro;

	/* 导出给控制模块：倾角 + 绕 Y 轴角速度（±2000dps 量程，灵敏度 16.4 LSB/°/s） */
	s_att_angle = Angle;
	s_att_rate  = (float)_M.Gyroy / MPU6050_GYRO_LSB_PER_DPS;

	last_tick = now;
}

/* 读取互补滤波后的倾角（度），供 LQR 等控制模块调用 */
float mpu6050_get_angle(void) { return s_att_angle; }

/* 读取绕 Y 轴角速度（度/秒），供 LQR 等控制模块调用 */
float mpu6050_get_rate(void)  { return s_att_rate;  }
