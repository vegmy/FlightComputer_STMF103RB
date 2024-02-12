#ifndef __LSM6DSO_IMU_H
#define __LSM6DSO_IMU_H

#include "stm32f1xx_hal_i2c.h"

/* LSM6DSO gyroscope function prototypes */
uint8_t lsm6dso_imu_init(I2C_HandleTypeDef *ptr_i2c1);
void lsm6dso_calibrate_sensor(I2C_HandleTypeDef *ptr_i2c1, uint8_t read_start_register, float *vec_offset);
void lsm6dso_read_imu(I2C_HandleTypeDef *ptr_i2c1, float *vec_imu);
void lsm6dso_read_vector(I2C_HandleTypeDef *ptr_i2c1, uint8_t read_start_register, float *offset_vector, float *return_vector);
float lsm6dso_read_axis(I2C_HandleTypeDef *ptr_i2c1, uint8_t read_start_register, float offset_axis);

#endif