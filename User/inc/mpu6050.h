
#ifndef __MPU6050_H__
#define __MPU6050_H__

extern short g_fGyro_x,g_fAccel_y,g_fAccel_z;

void MPU6050_Init(void);
void MPU6050_Pose(void);


#endif

