#ifndef __IIC_H
#define __IIC_H

#include "main.h"

/* ========================== 总线对象 ========================== */
/* 一个 IIC_Bus 描述一条软件 I2C 总线，驱动函数通过它得知操作哪组引脚 */
typedef struct {
    GPIO_TypeDef *scl_port;
    uint16_t      scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t      sda_pin;
} IIC_Bus;

/* ========================== 引脚定义 ========================== */
/* ---------------- MPU6050：PA0(SCL) / PA1(SDA)，软件 I2C ---------------- */
#define MPU6050_SCL_PORT  GPIOA
#define MPU6050_SCL_PIN   GPIO_PIN_0
#define MPU6050_SDA_PORT  GPIOA
#define MPU6050_SDA_PIN   GPIO_PIN_1

#define IIC_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()

/* 总线实例（定义在 IIC.c） */
extern const IIC_Bus IIC_BUS_MPU6050;

/* 半周期延时(us)：2us 约 250kHz，如需更快可降到 1us */
#define IIC_DELAY_US   2

/* ========================== 返回状态 ========================== */
#define IIC_OK         0
#define IIC_ERR_NACK   1

/* ========================== 底层时序 ========================== */
void    IIC_Init(const IIC_Bus *bus);
void    IIC_Start(const IIC_Bus *bus);
void    IIC_Stop(const IIC_Bus *bus);
uint8_t IIC_SendByte(const IIC_Bus *bus, uint8_t byte);   /* 返回 0 = 收到 ACK，1 = 非应答 */
uint8_t IIC_ReadByte(const IIC_Bus *bus, uint8_t ack);    /* ack = 0 发送应答，ack = 1 发送非应答 */

/* ========================== 寄存器读写 ========================== */
/* dev_addr 为 7 位地址，函数内部左移 */
uint8_t IIC_WriteReg(const IIC_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t data);
uint8_t IIC_ReadRegs(const IIC_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
