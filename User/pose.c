#include "pose.h"
#include "LSM6DSR_Config.h"
#include <math.h>
float_ang_struct    att_angle;              //飞机姿态数据
float_xyz_struct    gyr_rad;                //把陀螺仪的各通道读出的数据，转换成弧度制
float_xyz_struct    acc_g;                  //滤波后的加速度数据
float   dcmgb[3][3];                        //方向余弦阵（将 惯性坐标系 转化为 机体坐标系）
#define LIMIT_VAL(a,min,max) ((a)<(min)?(min):((a)>(max)?(max):(a)))
//-------------------------------------------------------------------------------------------------------------------
// 函数简介    快速计算 1/sqrt(x)
// 参数说明     float           输入数据
// 返回参数     float           平方根之一
// 使用示例     y = invsqrt(0.005);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static float invsqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介    反正切
// 参数说明     float           输入数据
// 返回参数     float           反正切值
// 使用示例    y =  arctan1(0.005);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
float arctan1(float tan)
{
    float angle = (fabsf(tan) > 1.0f) ?  90.0f - fabsf(1.0f / tan) * (45.0f - (fabsf(1.0f / tan) - 1.0f) * (14.0f + 3.83f * fabsf(1.0f / tan))):
                                fabsf(tan) * (45.0f - (fabsf(tan) - 1.0f) * (14.0f + 3.83f * fabsf(tan)));
    return (tan > 0) ? angle : -angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介    反正切
// 参数说明     float           输入数据x
// 参数说明     float           输入数据y
// 返回参数     float           反正切值
// 使用示例    y =  arctan2(5, 6);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
float arctan2(float x, float y)
{
    float tan, angle;

    if (x == 0 && y == 0) return 0;

    if (x == 0)
    {
        if (y > 0) return 90;
        else return -90;
    }

    if (y == 0)
    {
        if (x > 0) return 0;
        else return -180.0f;
    }
    tan = y / x;
    angle = arctan1(tan);
    if (x < 0 && angle > 0) angle -= 180.0f;
    else if (x < 0 && angle < 0) angle += 180.0f;
    return angle;
}

float arcsin(float i)
{
    return arctan1(i / sqrt(1 - i * i));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介   数据转换
// 参数说明    void
// 返回参数   void
// 使用示例   prepare_data();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
void prepare_data(void)
{
	//加速度AD值 转换成 米/平方秒 
	acc_g.x = LSE6DSR_data.ax_g;
	acc_g.y =	LSE6DSR_data.ay_g;
	acc_g.z = LSE6DSR_data.az_g;
//	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",acc_g.x,acc_g.y,acc_g.z);

	//陀螺仪AD值 转换成 弧度/秒    
	gyr_rad.x = LSE6DSR_data.gx_rads;
	gyr_rad.y = LSE6DSR_data.gy_rads;
	gyr_rad.z = LSE6DSR_data.gz_rads;
	
}
void one_fiter(float_xyz_struct *acc,float_xyz_struct *gyro, float_xyz_struct *fiter_angle)
{
	static float_xyz_struct gyro_angle = {0};
	static float_xyz_struct last_angle = {0};
	float alpha = 0.98f;  // 互补滤波系数
	
	// 陀螺仪积分
	gyro_angle.x += (gyro->x) * 0.001f;  // 0.001是采样时间
	gyro_angle.y += (gyro->y) * 0.001f;
	gyro_angle.x *= 0.99999999999999f;
	gyro_angle.y *= 0.99999999999999f;
	// 加速度计计算角度
	// 假设Z轴向上为正方向
	float acc_angle_x = -arctan2(acc->y, acc->z);  // Roll角，添加负号
	float acc_angle_y = arctan2(acc->x, acc->z);   // Pitch角
	
	// 互补滤波
	fiter_angle->x = alpha * (last_angle.x + gyro_angle.x) + (1 - alpha) * acc_angle_x;
	fiter_angle->y = alpha * (last_angle.y + gyro_angle.y) + (1 - alpha) * acc_angle_y;
	
	// 保存当前角度用于下一次计算
	last_angle.x = fiter_angle->x;
	last_angle.y = fiter_angle->y;
}

float Kp = 0.05f;                         // proportional gain governs rate of convergence to accelerometer/magnetometer
                                         //比例增益控制加速度计，磁力计的收敛速率

#define halfT 0.0025f                     // half the sample period 采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

void imuupdate(float_xyz_struct *gyr_rad,float_xyz_struct *acc_g,float_ang_struct *att_angle)
{
	uint8_t i;
	float matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };//初始化矩阵
	float ax = acc_g->x,ay = acc_g->y,az = acc_g->z;
	float gx = gyr_rad->x,gy = gyr_rad->y,gz = gyr_rad->z;
	float vx, vy, vz;
	float ex, ey, ez;
	float norm;
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	if(ax*ay*az==0)
	return;

	//加速度计测量的重力向量(机体坐标系)
	norm = invsqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	//	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);

	//陀螺仪积分估计重力向量(机体坐标系)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	// printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz);

	//测量的重力向量与估算的重力向量差积求出向量间的误差
	ex = (ay*vz - az*vy); //+ (my*wz - mz*wy);
	ey = (az*vx - ax*vz); //+ (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx); //+ (mx*wy - my*wx);


	//将误差PI后补偿到陀螺仪
	gx = gx + Kp*ex;
	gy = gy + Kp*ey;
	gz = gz + Kp*ez;//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
	Kp = (10.0f - LIMIT_VAL((fabsf(gyr_rad->x) + fabsf(gyr_rad->y) + fabsf(gyr_rad->z)) / 5.0f,0,10.0f));
	//四元素的微分方程
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	//单位化四元数
	norm = invsqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
//
	//矩阵R 将惯性坐标系(n)转换到机体坐标系(b)
	matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;	// 11(前列后行)
	matrix[1] = 2.f * (q1q2 + q0q3);	    // 12
	matrix[2] = 2.f * (q1q3 - q0q2);	    // 13
	matrix[3] = 2.f * (q1q2 - q0q3);	    // 21
	matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;	// 22
	matrix[5] = 2.f * (q2q3 + q0q1);	    // 23
	matrix[6] = 2.f * (q1q3 + q0q2);	    // 31
	matrix[7] = 2.f * (q2q3 - q0q1);	    // 32
	matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;	// 33

	//四元数转换成欧拉角(Z->y->x)
	att_angle->rol = arctan2(matrix[8], matrix[5]);                          // roll(负号要注意)
	att_angle->pit = -arcsin(matrix[2]); // pitch
    att_angle->yaw = arctan2(matrix[0], matrix[1]);
	for(i=0;i<9;i++)
	{
		*(&(dcmgb[0][0])+i) = matrix[i];
	}

}
