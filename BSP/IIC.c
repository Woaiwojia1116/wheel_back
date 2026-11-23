#include "IIC.h"
#include "delay.h"

/* ========================== 总线实例 ========================== */
/* 编码器已改走两路硬件 I2C，软件 I2C 只剩 MPU6050（PA0/PA1） */
const IIC_Bus IIC_BUS_MPU6050 = { MPU6050_SCL_PORT, MPU6050_SCL_PIN,
                                  MPU6050_SDA_PORT, MPU6050_SDA_PIN };

/* ========================== 引脚操作 ========================== */
#define SCL_H(b)     HAL_GPIO_WritePin((b)->scl_port, (b)->scl_pin, GPIO_PIN_SET)
#define SCL_L(b)     HAL_GPIO_WritePin((b)->scl_port, (b)->scl_pin, GPIO_PIN_RESET)
#define SDA_H(b)     HAL_GPIO_WritePin((b)->sda_port, (b)->sda_pin, GPIO_PIN_SET)
#define SDA_L(b)     HAL_GPIO_WritePin((b)->sda_port, (b)->sda_pin, GPIO_PIN_RESET)
#define SDA_READ(b)  HAL_GPIO_ReadPin ((b)->sda_port, (b)->sda_pin)                                                      

/* ========================== GPIO 方向切换 ========================== */
static void SCL_OUT(const IIC_Bus *bus)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = bus->scl_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;   /* 开漏输出，依赖外部上拉 */
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->scl_port, &gpio);
}

static void SDA_OUT(const IIC_Bus *bus)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = bus->sda_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->sda_port, &gpio);
}

static void SDA_IN(const IIC_Bus *bus)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = bus->sda_pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(bus->sda_port, &gpio);
}

/* ========================== 延时 ========================== */
static inline void i2c_delay(void)
{
    delay_us(IIC_DELAY_US);
}

/* ========================== 初始化 ========================== */
void IIC_Init(const IIC_Bus *bus)
{
    /* 确保 GPIO 时钟使能 */
    IIC_GPIO_CLK_ENABLE();

    /* 先把两条线都拉成开漏输出且置高（总线空闲） */
    SCL_OUT(bus);
    SDA_OUT(bus);
    SCL_H(bus);
    SDA_H(bus);
    delay_us(10);           /* 等总线稳定 */
}

/* ========================== 起始 / 停止 ========================== */
void IIC_Start(const IIC_Bus *bus)
{
    SDA_OUT(bus);
    SDA_H(bus); SCL_H(bus); i2c_delay();
    SDA_L(bus); i2c_delay();
    SCL_L(bus); i2c_delay();
}

void IIC_Stop(const IIC_Bus *bus)
{
    SDA_OUT(bus);
    SCL_L(bus); SDA_L(bus); i2c_delay();
    SCL_H(bus); i2c_delay();
    SDA_H(bus); i2c_delay();
}

/* ========================== 收发字节 ========================== */
/* 返回 0 = 收到 ACK，1 = 非应答 */
uint8_t IIC_SendByte(const IIC_Bus *bus, uint8_t byte)
{
    SDA_OUT(bus);
    for (int i = 0; i < 8; i++) {
        if (byte & 0x80) SDA_H(bus); else SDA_L(bus);
        byte <<= 1;
        i2c_delay();
        SCL_H(bus); i2c_delay();
        SCL_L(bus); i2c_delay();
    }
    /* 读 ACK：先确保 SCL 低 → 释放 SDA → 拉高 SCL → 采样 */
    SCL_L(bus); i2c_delay();
    SDA_IN(bus);  i2c_delay();
    SCL_H(bus);   i2c_delay();
    uint8_t ack = SDA_READ(bus);   /* 0 = ACK */
    SCL_L(bus);   i2c_delay();
    SDA_OUT(bus);
    return ack;
}

/* ack = 0 发送应答，ack = 1 发送非应答 */
uint8_t IIC_ReadByte(const IIC_Bus *bus, uint8_t ack)
{
    uint8_t byte = 0;
    SDA_IN(bus);
    for (int i = 0; i < 8; i++) {
        SCL_H(bus); i2c_delay();
        byte <<= 1;
        if (SDA_READ(bus)) byte |= 0x01;
        SCL_L(bus); i2c_delay();
    }
    /* 发送 ACK / NACK */
    SDA_OUT(bus);
    if (ack) SDA_H(bus); else SDA_L(bus);
    i2c_delay();
    SCL_H(bus); i2c_delay();
    SCL_L(bus); i2c_delay();
    return byte;
}

/* ========================== 寄存器读写 ========================== */
/* 写单个寄存器，返回 IIC_OK / IIC_ERR_NACK */
uint8_t IIC_WriteReg(const IIC_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    uint8_t err = IIC_OK;

    IIC_Start(bus);
    if (IIC_SendByte(bus, (uint8_t)(dev_addr << 1)) != 0) err = IIC_ERR_NACK;   /* 写地址 */
    if (IIC_SendByte(bus, reg) != 0)                     err = IIC_ERR_NACK;   /* 寄存器指针 */
    if (IIC_SendByte(bus, data) != 0)                    err = IIC_ERR_NACK;   /* 数据 */
    IIC_Stop(bus);

    return err;
}

/* 从 reg 起连续读 len 字节（重复起始 + 读地址），返回 IIC_OK / IIC_ERR_NACK */
uint8_t IIC_ReadRegs(const IIC_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len == 0) return IIC_OK;

    IIC_Start(bus);
    if (IIC_SendByte(bus, (uint8_t)(dev_addr << 1)) != 0) {       /* 写地址 */
        IIC_Stop(bus);
        return IIC_ERR_NACK;
    }
    if (IIC_SendByte(bus, reg) != 0) {                            /* 寄存器指针 */
        IIC_Stop(bus);
        return IIC_ERR_NACK;
    }

    IIC_Start(bus);                                               /* 重复起始 */
    if (IIC_SendByte(bus, (uint8_t)((dev_addr << 1) | 1)) != 0) { /* 读地址 */
        IIC_Stop(bus);
        return IIC_ERR_NACK;
    }

    for (uint8_t i = 0; i < len; i++) {
        /* 最后一字节发 NACK，其余发 ACK */
        buf[i] = IIC_ReadByte(bus, (i == (uint8_t)(len - 1)) ? 1 : 0);
    }

    IIC_Stop(bus);

    return IIC_OK;
}
