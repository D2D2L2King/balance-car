/*
 * MPU6050.c
 *
 *  Created on: Nov 26, 2021
 *      Author: ZY
 */

#ifndef USER_MPU6050_C_
#define USER_MPU6050_C_

#include "MPU6050.h"

//�����Ƿ�������
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};
//����ת��
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

//�����Ƿ������
static unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
    XYZ  010_001_000  ��λ����
    XZY  001_010_000
    YXZ  010_000_001
    YZX  000_010_001
    ZXY  001_000_010
    ZYX  000_001_010
    */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

//DMP���Լ죬���Ǳ궨���ڵ�״̬Ϊ����ԭ��
static int run_self_test(void)
{
    uint8_t result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);

    //MPU6050�Բⷵ��ֵΪ0x03
    if (result == 0x03) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        
        accel_sens = 0;//����������У׼
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
//		printf("Reset XYZ\n");

		return 0;
    }
    else
    	return 1;
}

//MPU6050����DMP�ĳ�ʼ��
uint8_t DMP_Init(void)
{
	//��ʼ�����ж��Ƿ�ɹ�
	if(!mpu_init())
	{
		//���������Ǻͼ��ټƴ�������ʱ�Ӻ͹���״̬����
		if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
			return 1;

		//���������Ǻͼ��ټƿ���FIFOͨ������
		if(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
			return 2;

		//����Ĭ�ϵĲ�����
		if(mpu_set_sample_rate(DEFAULT_MPU_HZ))
			return 3;

		//���ز���֤DMPӳ����
		if(dmp_load_motion_driver_firmware())
			return 4;

		//���������Ǻͼ��ٶȼƵķ������DMP�����������Ƿ���
		if(dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
			return 5;

		//DMP�Ĺ���ѡ���־λ���ã���������DMPҪ�����Ĺ���
		if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
			DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
			DMP_FEATURE_GYRO_CAL))
			return 6;

		//����DMP��FIFO�������(��󲻳���200Hz)
		if(dmp_set_fifo_rate(DEFAULT_MPU_HZ))
			return 7;

		//DMP���Լ죬���Ǳ궨���ڵ�״̬Ϊ����ԭ��
		if(run_self_test())
			return 8;

		//ʹ��DMP
		if(mpu_set_dmp_state(1))
			return 9;
	}
	else
		return 10;
	return 0;
}

//q30��ʽ,longתfloatʱ�ĳ���.
#define q30  1073741824.0f

short gyro[3], accel[3], sensors;

float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

//��ȡMPU6050����DMP����̬��Ϣ
//pitch:������ ����:0.1��   ��Χ:-90.0�� <---> +90.0��
//roll:�����  ����:0.1��   ��Χ:-180.0��<---> +180.0��
//yaw:�����   ����:0.1��   ��Χ:-180.0��<---> +180.0��
uint8_t Read_DMP(float *Pitch, float *Roll, float *Yaw)
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];

	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
		return 1;

	if (sensors & INV_WXYZ_QUAT )
	{
		//q30��ʽת��Ϊ������
		q0=quat[0] / q30;
		q1=quat[1] / q30;
		q2=quat[2] / q30;
		q3=quat[3] / q30;

		//����õ�������/�����/�����
		*Pitch = asin((-2) * q1 * q3 + 2 * q0 * q2)* 57.3;
		*Roll  = atan2(2*q2*q3 + 2*q0 *q1, (-2)*q1*q1 - 2*q2*q2 + 1) * 57.3;
		*Yaw   = atan2(2*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3;
	}
	return 0;
}

//�õ�������ֵ(ԭʼֵ)
void MPU_Get_Gyro(short *gx, short *gy, short *gz)
{
	short Data[3];

	mpu_get_gyro_reg(Data, NULL);

	*gx = Data[0];
	*gy = Data[1];
	*gz = Data[2];
}


void I2C_Write(unsigned char dev_addr, unsigned char mem_addr, 
								unsigned char data,int16_t size)
{
	HAL_I2C_Mem_Write(&hi2c1, dev_addr, mem_addr,
						I2C_MEMADD_SIZE_8BIT, &data, size, 2);
}

void I2C_Read(unsigned char dev_addr, unsigned char mem_addr, 
	unsigned char *buf, unsigned char len)
{
	HAL_I2C_Mem_Read(&hi2c1, dev_addr, mem_addr, 
					I2C_MEMADD_SIZE_8BIT, buf, len, 2);
}

//#define i2c_write(a,b,c,d)   \ HAL_I2C_Mem_Write(&hi2c1, a, b, I2C_MEMADD_SIZE_8BIT, d, c, 2)
//#define i2c_read(a,b,c,d)    \ HAL_I2C_Mem_Read(&hi2c1, a, b, I2C_MEMADD_SIZE_8BIT, d, c, 2)
#endif /* USER_MPU6050_C_ */
