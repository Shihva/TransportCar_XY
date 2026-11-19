#ifndef __IMU_H
#define __IMU_H

typedef struct
{
    float q[4]; // 四元数估计值
    float gyro[3]; // 角速度
    float accel[3];// 加速度
    
    float roll;
    float pitch;
    float yaw;
    float YawTotalAngle;
} INS_t;

extern INS_t INS;

#endif
