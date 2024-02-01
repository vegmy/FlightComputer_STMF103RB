#ifndef __LSM6DSO_GYROSCOPE_H
#define __LSM6DSO_GYROSCOPE_H

#include "stm32f1xx_hal_i2c.h"

/* LSM6DSO gyroscope function prototypes */
HAL_StatusTypeDef lsm6dso_gyroscope_init(I2C_HandleTypeDef *ptr_i2c2);




#endif