/*
 * MPU6050.h
 *
 *  Created on: Nov 26, 2021
 *      Author: ZY
 */

#ifndef USER_MPU6050_H_
#define USER_MPU6050_H_

#include "i2c.h"
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

//定义输出速度
#define DEFAULT_MPU_HZ  (100)

//DMP初始化
uint8_t DMP_Init(void);
//读取MPU6050内置DMP的姿态信息
uint8_t Read_DMP(float *Pitch,float *Roll,float *Yaw);
//得到陀螺仪值(原始值)
void MPU_Get_Gyro(short *gx,short *gy,short *gz);

#endif /* USER_MPU6050_H_ */
