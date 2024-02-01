/*******************/

#ifndef __STTS751_TEMP_SENSOR_H
#define __STTS751_TEMP_SENSOR_H

#include "stm32f1xx_hal_i2c.h"

/* Funtion to initialize the sensor STTS751*/
int stts751_init(I2C_HandleTypeDef *ptr_i2c1);

/* Function to read temperature from STTS751 registers */
float stts751_read_temperature();

#endif