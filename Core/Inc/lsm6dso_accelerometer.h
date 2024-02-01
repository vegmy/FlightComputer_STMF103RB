#ifndef __LSM6DSO_ACCELEROMETER_H
#define __LSM6DSO_ACCELEROMETER_H

#include "stm32f1xx_hal_i2c.h"

/* LSM6DSO accelerometer function prototypes */
HAL_StatusTypeDef lsm6dso_acc_init(I2C_HandleTypeDef *ptr_i2c2);
float lsm6dso_read_linear_acc(I2C_HandleTypeDef *ptr_i2c2);
float lsm6dso_read_acc_axis(I2C_HandleTypeDef *ptr_i2c2, uint8_t high_byte_reg, uint8_t low_byte_reg, float acc_axis_offset);
void lsm6dso_calibrate_acc(I2C_HandleTypeDef *ptr_i2c1);

#endif