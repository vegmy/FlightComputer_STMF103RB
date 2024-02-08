#ifndef __LSM6DSO_GYROSCOPE_H
#define __LSM6DSO_GYROSCOPE_H

#include "stm32f1xx_hal_i2c.h"

/* LSM6DSO gyroscope function prototypes */
HAL_StatusTypeDef lsm6dso_gyroscope_init(I2C_HandleTypeDef *ptr_i2c1);
void lsm6dso_read_rot_vector(I2C_HandleTypeDef *ptr_i2c1, float *gyro_vector);
void lsm6dso_calibrate_gyro(I2C_HandleTypeDef *ptr_i2c1);

#endif