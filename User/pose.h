#ifndef POSE_H_
#define POSE_H_
//三轴浮点型
typedef struct
{
    float x;
    float y;
    float z;
}float_xyz_struct;

//姿态解算后的角度
typedef struct
{
    float rol;
    float pit;
    float yaw;
}float_ang_struct;
extern float q0, q1, q2, q3;
extern float_ang_struct    att_angle;
extern float_xyz_struct    gyr_rad,gyr_radold;               //把陀螺仪的各通道读出的数据，转换成弧度制
extern float_xyz_struct    acc_g,gry_filt,acc_gold;    //滤波后的各通道数据
extern void prepare_data(void);
extern void imuupdate(float_xyz_struct *gyr_rad,float_xyz_struct *acc_g,float_ang_struct *att_angle);
extern void one_fiter(float_xyz_struct *acc,float_xyz_struct *gyro, float_xyz_struct *fiter_angle);
#endif
