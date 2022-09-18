# balance-car
平衡小车
主控用的arduino uno，电机控制用的tb6612fng，imu是mpu6050;
编码器已进行2倍频处理，因arduino uno外部中断接口不够，所以只能做到2倍频，电机编码器分辨率不够问题将会在stm32版本解决;
