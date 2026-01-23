#include "LSM6DSR_Config.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "Delay.h"
LSM6DSR_DATA_T LSE6DSR_data;
#ifdef USE_SOFTWARE_SPI

// ==================== 软件SPI接口 ====================
static void SoftSPI_Init(void)
{
	GPIO_InitTypeDef gpio;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	
	// CS引脚：推挽输出，默认高电平
	gpio.GPIO_Pin = LSM6DSR_CS_PIN;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LSM6DSR_CS_PORT, &gpio);
	GPIO_SetBits(LSM6DSR_CS_PORT, LSM6DSR_CS_PIN);
	
	// SCK引脚：推挽输出
	gpio.GPIO_Pin = LSM6DSR_SCK_PIN;
	GPIO_Init(LSM6DSR_SCK_PORT, &gpio);
	GPIO_ResetBits(LSM6DSR_SCK_PORT, LSM6DSR_SCK_PIN);
	
	// MOSI引脚：推挽输出
	gpio.GPIO_Pin = LSM6DSR_MOSI_PIN;
	GPIO_Init(LSM6DSR_MOSI_PORT, &gpio);
	GPIO_ResetBits(LSM6DSR_MOSI_PORT, LSM6DSR_MOSI_PIN);
	
	// MISO引脚：浮空输入
	gpio.GPIO_Pin = LSM6DSR_MISO_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(LSM6DSR_MISO_PORT, &gpio);
}

static void SoftSPI_CS_Low(void)
{
	GPIO_ResetBits(LSM6DSR_CS_PORT, LSM6DSR_CS_PIN);
}

static void SoftSPI_CS_High(void)
{
	GPIO_SetBits(LSM6DSR_CS_PORT, LSM6DSR_CS_PIN);
}

static uint8_t SoftSPI_TransmitByte(uint8_t byte)
{
	uint8_t i, recv = 0;
	
	for(i = 0; i < 8; i++)
	{
		// 时钟拉低
		GPIO_ResetBits(LSM6DSR_SCK_PORT, LSM6DSR_SCK_PIN);
		Delay_us(1);
		
		// 发送数据位
		if(byte & 0x80)
			GPIO_SetBits(LSM6DSR_MOSI_PORT, LSM6DSR_MOSI_PIN);
		else
			GPIO_ResetBits(LSM6DSR_MOSI_PORT, LSM6DSR_MOSI_PIN);
		
		byte <<= 1;
		Delay_us(1);
		
		// 时钟拉高，读取MISO
		GPIO_SetBits(LSM6DSR_SCK_PORT, LSM6DSR_SCK_PIN);
		Delay_us(1);
		
		recv <<= 1;
		if(GPIO_ReadInputDataBit(LSM6DSR_MISO_PORT, LSM6DSR_MISO_PIN))
			recv |= 0x01;
		
		Delay_us(1);
	}
	
	return recv;
}

static void SoftSPI_WriteByte(uint8_t reg, uint8_t data)
{
	SoftSPI_CS_Low();
	SoftSPI_TransmitByte(reg & 0x7F);  // 写命令，清除bit7
	SoftSPI_TransmitByte(data);
	SoftSPI_CS_High();
}

static uint8_t SoftSPI_ReadByte(uint8_t reg)
{
	uint8_t data;
	SoftSPI_CS_Low();
	SoftSPI_TransmitByte(reg | 0x80);  // 读命令，设置bit7
	data = SoftSPI_TransmitByte(0xFF);
	SoftSPI_CS_High();
	return data;
}

static void SoftSPI_ReadBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t i;
	SoftSPI_CS_Low();
	SoftSPI_TransmitByte(reg | 0x80);  // 读命令，设置bit7
	for(i = 0; i < len; i++)
	{
		buf[i] = SoftSPI_TransmitByte(0xFF);
	}
	SoftSPI_CS_High();
}

#else
// ==================== 硬件SPI接口 ====================
static void HardwareSPI_Init(void)
{
	GPIO_InitTypeDef gpio;
	SPI_InitTypeDef spi;
	
	// 开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	// CS引脚：推挽输出，软件控制，默认高电平
	gpio.GPIO_Pin = LSM6DSR_CS_PIN;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LSM6DSR_CS_PORT, &gpio);
	GPIO_SetBits(LSM6DSR_CS_PORT, LSM6DSR_CS_PIN);
	
	// SCK, MOSI：复用推挽输出
	gpio.GPIO_Pin = LSM6DSR_SCK_PIN | LSM6DSR_MOSI_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LSM6DSR_SCK_PORT, &gpio);
	
	// MISO：浮空输入
	gpio.GPIO_Pin = LSM6DSR_MISO_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(LSM6DSR_MISO_PORT, &gpio);
	
	// SPI2配置
	SPI_StructInit(&spi);
	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_CPOL = SPI_CPOL_Low;    // CPOL=0
	spi.SPI_CPHA = SPI_CPHA_1Edge;  // CPHA=0
	spi.SPI_NSS = SPI_NSS_Soft;     // 软件NSS管理
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  // SPI2时钟=36MHz/8=4.5MHz (先恢复原速度，确保稳定性)
	spi.SPI_FirstBit = SPI_FirstBit_MSB;
	spi.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI2, &spi);
	SPI_Cmd(SPI2, ENABLE);
}

static void HardwareSPI_CS_Low(void)
{
	GPIO_ResetBits(LSM6DSR_CS_PORT, LSM6DSR_CS_PIN);
}

static void HardwareSPI_CS_High(void)
{
	GPIO_SetBits(LSM6DSR_CS_PORT, LSM6DSR_CS_PIN);
}

// 优化版：使用寄存器直接访问，减少函数调用开销
// 添加超时保护，防止 SPI 异常导致死循环
static uint8_t HardwareSPI_TransmitByte(uint8_t byte)
{
	volatile uint32_t timeout;
	
	// 等待发送缓冲区空（添加超时保护）
	timeout = 1000;
	while((SPI2->SR & SPI_I2S_FLAG_TXE) == RESET)
	{
		if(--timeout == 0) return 0xFF; // 超时返回
	}
	SPI2->DR = byte;  // 直接写入数据寄存器
	
	// 等待接收完成（添加超时保护）
	timeout = 1000;
	while((SPI2->SR & SPI_I2S_FLAG_RXNE) == RESET)
	{
		if(--timeout == 0) return 0xFF; // 超时返回
	}
	return SPI2->DR;  // 直接读取数据寄存器
}

static void HardwareSPI_WriteByte(uint8_t reg, uint8_t data)
{
	HardwareSPI_CS_Low();
	HardwareSPI_TransmitByte(reg & 0x7F);  // 写命令，清除bit7
	HardwareSPI_TransmitByte(data);
	HardwareSPI_CS_High();
}

static uint8_t HardwareSPI_ReadByte(uint8_t reg)
{
	uint8_t data;
	HardwareSPI_CS_Low();
	HardwareSPI_TransmitByte(reg | 0x80);  // 读命令，设置bit7
	data = HardwareSPI_TransmitByte(0xFF);
	HardwareSPI_CS_High();
	return data;
}

// 优化版：使用寄存器直接访问，但保持安全的读取时序
static void HardwareSPI_ReadBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t i;
	HardwareSPI_CS_Low();
	
	// 发送读命令（地址），使用优化的函数
	HardwareSPI_TransmitByte(reg | 0x80);  // 读命令，设置bit7
	// 注意：发送命令时收到的响应需要丢弃，但第一个数据字节需要从第一个0xFF读取
	
	// 读取数据字节：发送dummy字节并接收数据
	for(i = 0; i < len; i++)
	{
		buf[i] = HardwareSPI_TransmitByte(0xFF);
	}
	
	HardwareSPI_CS_High();
}
#endif

// ==================== 统一接口函数 ====================
uint8_t LSM6DSR_ReadReg(uint8_t reg)
{
#ifdef USE_SOFTWARE_SPI
	return SoftSPI_ReadByte(reg);
#else
	return HardwareSPI_ReadByte(reg);
#endif
}

void LSM6DSR_WriteReg(uint8_t reg, uint8_t value)
{
#ifdef USE_SOFTWARE_SPI
	SoftSPI_WriteByte(reg, value);
#else
	HardwareSPI_WriteByte(reg, value);
#endif
}

void LSM6DSR_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
#ifdef USE_SOFTWARE_SPI
	SoftSPI_ReadBytes(reg, buf, len);
#else
	HardwareSPI_ReadBytes(reg, buf, len);
#endif
}

// ==================== LSM6DSR功能函数 ====================
uint8_t LSM6DSR_ReadID(void)
{
	return LSM6DSR_ReadReg(LSM6DSR_WHO_AM_I);
}

uint8_t LSM6DSR_Init(void)
{
#ifdef USE_SOFTWARE_SPI
	SoftSPI_Init();
#else
	HardwareSPI_Init();
#endif
	
	Delay_ms(10);
	
	// 读取设备ID
	uint8_t id = LSM6DSR_ReadID();
	if(id != LSM6DSR_DEVICE_ID)
	{
		return 0;  // 设备ID错误
	}
	
	// 配置加速度计：52Hz，±2g
	LSM6DSR_WriteReg(LSM6DSR_CTRL1_XL, 0x20);
	
	// 使能加速度计x,y,z轴
	LSM6DSR_WriteReg(LSM6DSR_CTRL9_XL, 0x38);
	
	// 陀螺仪电平触发，加速度计高性能使能
	LSM6DSR_WriteReg(LSM6DSR_CTRL6_C, 0x40 | 0x10);
	
	// 陀螺仪高性能使能
	LSM6DSR_WriteReg(LSM6DSR_CTRL7_G, 0x80);
	
	// 加速度计INT2引脚失能,陀螺仪数据INT2使能
	LSM6DSR_WriteReg(LSM6DSR_INT2_CTRL, 0x03);
	
	// 陀螺仪：±2000dps
	LSM6DSR_WriteReg(LSM6DSR_CTRL2_G, 0x58);
	
	// 使能陀螺仪x,y,z轴
	LSM6DSR_WriteReg(LSM6DSR_CTRL10_C, 0x38);
	
	Delay_ms(10);
	
	return 1;  // 初始化成功
}

void LSM6DSR_ReadData(LSM6DSR_DATA_T *physics)
{
	uint8_t buf[12];
	
	// 从陀螺仪X低字节开始连续读取12字节（6轴数据）
	LSM6DSR_ReadRegs(LSM6DSR_OUTX_L_GYRO, buf, 12);
	
	// 组合数据：先低字节后高字节，有符号16位
	physics->gx  = ((int16_t)buf[1] << 8) | buf[0];
	physics->gy  = ((int16_t)buf[3] << 8) | buf[2];
	physics->gz  = ((int16_t)buf[5] << 8) | buf[4];
	physics->ax  = ((int16_t)buf[7] << 8) | buf[6];
	physics->ay  = ((int16_t)buf[9] << 8) | buf[8];
	physics->az = ((int16_t)buf[11] << 8) | buf[10];
}

/**
 * @brief  将LSM6DSR原始数据转换为标准物理单位
 * @param  physics: 输入输出结构体指针（包含原始数据，转换后填充物理量）
 * @note   加速度计配置：±2g，敏感度 = 16384 LSB/g
 *         陀螺仪配置：±2000dps
 *         角速度转换：1度 = π/180 弧度 ≈ 0.017453293 rad
 *         直接转换公式：rad/s = (原始值 / 14.3) * (π/180) = 原始值 / 14.3f
 */
void LSM6DSR_ConvertToPhysics(LSM6DSR_DATA_T *physics)
{
	if(physics == 0) return;
	
	// 加速度转换为g单位：±2g量程，敏感度 = 16384 LSB/g
	physics->ax_g = (float)physics->ax / 16384.0f;
	physics->ay_g = (float)physics->ay / 16384.0f;
	physics->az_g = (float)physics->az / 16384.0f;
	
	// 角速度转换为弧度每秒：±2000dps量程
	// 根据LSM6DSR数据手册，敏感度 = 70 MLSB/(°/s)0.07
	// 转换步骤：
	//   1. 度/秒 = 原始值 / 14.3f
	//   2. 弧度/秒 = 度/秒 × (π/180)
	const float GYRO_LSB_TO_RAD_PER_SEC = 1637.022271802352025f;  
	physics->gx_rads = ((float)physics->gx - 3.519f) / GYRO_LSB_TO_RAD_PER_SEC;
	physics->gy_rads = ((float)physics->gy  + 12.03f)/ GYRO_LSB_TO_RAD_PER_SEC;
	physics->gz_rads = ((float)physics->gz + 3.205f)/ GYRO_LSB_TO_RAD_PER_SEC;
}

