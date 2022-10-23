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

//��������ٶ�
#define DEFAULT_MPU_HZ  (100)

//DMP��ʼ��
uint8_t DMP_Init(void);
//��ȡMPU6050����DMP����̬��Ϣ
uint8_t Read_DMP(float *Pitch,float *Roll,float *Yaw);
//�õ�������ֵ(ԭʼֵ)
void MPU_Get_Gyro(short *gx,short *gy,short *gz);

#endif /* USER_MPU6050_H_ */
