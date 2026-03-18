#include "LSM6DSR_Config.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "Delay.h"
#include <math.h>

LSM6DSR_DATA_T LSE6DSR_data;
LSM6DSR_CALIB_T g_gyro_calib = {0};  // 陀螺仪校准数据

// Yaw角积分相关变量
static float g_yaw_angle   = 0.0f;   // 当前Yaw角（度）
static float g_last_gyro_z = 0.0f;   // 上次Z轴角速度，用于梯形积分
static float g_gyro_z_dps  = 0.0f;   // 死区滤波后Z轴角速度（度/秒）

// 死区阈值（度/秒），小于此值视为噪声
#define GYRO_DEADZONE_DPS   0.8f
#define GYRO_VALID_MAX_DPS   1000.0f
#define YAW_VALID_MAX_DEG    1000.0f

// 非阻塞动态校准内部变量
static uint16_t g_gyro_calib_target = 0;
static uint16_t g_gyro_calib_count  = 0;
static float    g_gyro_calib_sum_x  = 0.0f;
static float    g_gyro_calib_sum_y  = 0.0f;
static float    g_gyro_calib_sum_z  = 0.0f;
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
 * @note   加速度计：±2g，敏感度 = 16384 LSB/g
 *         陀螺仪：±2000dps，灵敏度 = 70 mdps/LSB
 *         零漂：优先使用动态校准值，未校准时回退到硬编码偏置（本板实测值）
 */
void LSM6DSR_ConvertToPhysics(LSM6DSR_DATA_T *physics)
{
	if(physics == 0) return;

	// 加速度计：±2g，敏感度 = 16384 LSB/g
	physics->ax_g = (float)physics->ax / 16384.0f;
	physics->ay_g = (float)physics->ay / 16384.0f;
	physics->az_g = (float)physics->az / 16384.0f;

	// 陀螺仪：CTRL2_G=0x58 → ±2000dps，灵敏度 70 mdps/LSB → 0.07 dps/LSB
	// rad/s = LSB × 0.07 × (π/180) = LSB / 1637.02
	const float GYRO_LSB_TO_RAD_PER_SEC = 1637.022271802352025f;

	/* ---- 非阻塞后台校准采样 ---- */
	if(g_gyro_calib_target > 0)
	{
		g_gyro_calib_sum_x += (float)physics->gx;
		g_gyro_calib_sum_y += (float)physics->gy;
		g_gyro_calib_sum_z += (float)physics->gz;
		g_gyro_calib_count++;

		if(g_gyro_calib_count >= g_gyro_calib_target)
		{
			g_gyro_calib.gyro_offset_x = g_gyro_calib_sum_x / (float)g_gyro_calib_target;
			g_gyro_calib.gyro_offset_y = g_gyro_calib_sum_y / (float)g_gyro_calib_target;
			g_gyro_calib.gyro_offset_z = g_gyro_calib_sum_z / (float)g_gyro_calib_target;
			g_gyro_calib.calibrated = 1;
			g_gyro_calib_target = 0;  // 结束校准状态
			// 校准完成，清零Yaw积分起点
			g_yaw_angle   = 0.0f;
			g_last_gyro_z = 0.0f;
			g_gyro_z_dps  = 0.0f;
		}
	}

	/* ---- 角速度转换（优先动态校准，否则硬编码兜底） ---- */
	if(g_gyro_calib.calibrated)
	{
		physics->gx_rads = ((float)physics->gx - g_gyro_calib.gyro_offset_x) / GYRO_LSB_TO_RAD_PER_SEC;
		physics->gy_rads = ((float)physics->gy - g_gyro_calib.gyro_offset_y) / GYRO_LSB_TO_RAD_PER_SEC;
		physics->gz_rads = ((float)physics->gz - g_gyro_calib.gyro_offset_z) / GYRO_LSB_TO_RAD_PER_SEC;
	}
	else
	{
		// 本板实测硬编码零漂偏置（未完成动态校准时兜底）
		physics->gx_rads = ((float)physics->gx - 13.344f) / GYRO_LSB_TO_RAD_PER_SEC;
		physics->gy_rads = ((float)physics->gy + 16.204f) / GYRO_LSB_TO_RAD_PER_SEC;
		physics->gz_rads = ((float)physics->gz +  5.056f) / GYRO_LSB_TO_RAD_PER_SEC;
	}
}

/* ========================================================================== */
/*                       动态校准 & Yaw角接口                                  */
/* ========================================================================== */

/**
 * @brief  非阻塞启动陀螺仪零漂校准
 * @param  samples  采样次数 (SysTick 2ms 周期，1000次 = 2s)
 * @note   校准期间小车必须静止，完成后 g_gyro_calib.calibrated 置1
 */
void LSM6DSR_StartCalibration(uint16_t samples)
{
	g_gyro_calib_target = samples;
	g_gyro_calib_count  = 0;
	g_gyro_calib_sum_x  = 0.0f;
	g_gyro_calib_sum_y  = 0.0f;
	g_gyro_calib_sum_z  = 0.0f;
	g_gyro_calib.calibrated = 0;
}

/**
 * @brief  检查是否正在校准
 * @return 1=正在校准, 0=空闲
 */
uint8_t LSM6DSR_IsCalibrating(void)
{
	return (g_gyro_calib_target > 0) ? 1 : 0;
}

/**
 * @brief  更新Yaw角积分（在SysTick中每帧调用）
 * @param  dt  采样周期（秒）
 * @note   采用梯形积分；死区 ±0.8 deg/s 内清零，避免噪声漫积
 */
void LSM6DSR_UpdateYaw(float dt)
{
	const float RAD_TO_DEG = 57.2957795131f;
	float gyro_z = LSE6DSR_data.gz_rads * RAD_TO_DEG;

	/* 输入有效性保护：采样周期/角速度异常时不积分，避免Yaw炸值 */
	if((!isfinite(dt)) || dt <= 0.0f || dt > 0.02f)
	{
		g_last_gyro_z = 0.0f;
		g_gyro_z_dps = 0.0f;
		return;
	}

	if((!isfinite(gyro_z)) || (fabsf(gyro_z) > GYRO_VALID_MAX_DPS))
	{
		gyro_z = 0.0f;
	}

	// 死区滤波
	if(fabsf(gyro_z) < GYRO_DEADZONE_DPS)
	{
		gyro_z = 0.0f;
	}
	g_gyro_z_dps = gyro_z;

	// 梯形积分（比矩形积分在角速度变化时误差更小）
	g_yaw_angle += (g_last_gyro_z + gyro_z) * 0.5f * dt;
	g_last_gyro_z = gyro_z;

	/* 积分结果防爆保护 */
	if((!isfinite(g_yaw_angle)) || (fabsf(g_yaw_angle) > YAW_VALID_MAX_DEG))
	{
		g_yaw_angle = 0.0f;
		g_last_gyro_z = 0.0f;
	}
}

/**
 * @brief  获取当前Yaw角（度）
 */
float LSM6DSR_GetYaw(void)
{
	return g_yaw_angle;
}

/**
 * @brief  清零Yaw角
 */
void LSM6DSR_ClearYaw(void)
{
	g_yaw_angle   = 0.0f;
	g_last_gyro_z = 0.0f;
}

/**
 * @brief  获取死区滤波后Z轴角速度（度/秒）
 */
float LSM6DSR_GetGyroZ_DPS(void)
{
	return g_gyro_z_dps;
}

/**
 * @brief  获取偏置校正但未经死区过滤的Z轴角速度（度/秒）
 * @note   用于与死区滤波后的值对比，验证滤波效果
 */
float LSM6DSR_GetRawGyroZ_DPS(void)
{
	const float RAD_TO_DEG = 57.2957795131f;
	return LSE6DSR_data.gz_rads * RAD_TO_DEG;
}

