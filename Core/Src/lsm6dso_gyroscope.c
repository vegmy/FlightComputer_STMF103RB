/**
 * @file lsm6dso_gyroscope.c
 * @brief Contains functionality to read LSM6DSO gyroscope sensor data and calculate rotation rate.
 *
 * @author Vegard Myhre
 * @date 31-01-2024
 * @version 1.0
 *
 * @see Datasheets/lsm6dso_acc_gyro_datasheet.pdf
 */

#include "stm32f1xx_hal.h"
#include "lsm6dso_gyroscope.h"
#include "math.h"

/* LSM6DSO Registers */
const uint8_t LSM6DSO_ADD_REG2 = 0xD6;

// Gyroscope control registers
const uint8_t LSM6DSO_CTRL2_G  = 0x11;

/* LSM6DSO gyroscope registers */
const uint8_t LSM6DSO_OUTX_L_G = 0x22;
const uint8_t LSM6DSO_OUTX_H_G = 0x23;
const uint8_t LSM6DSO_OUTY_L_G = 0x24;
const uint8_t LSM6DSO_OUTY_H_G = 0x25;
const uint8_t LSM6DSO_OUTZ_L_G = 0x26;
const uint8_t LSM6DSO_OUTZ_H_G = 0x27;

/* LSM6DSO register data */
const uint16_t ONEBYTE             = 1;
const uint8_t LSM6DSO_CTRL2_G_DATA = 0x60;

HAL_StatusTypeDef lsm6dso_gyroscope_init(I2C_HandleTypeDef *ptr_i2c1)
{
    HAL_StatusTypeDef write_memory_ok = HAL_I2C_Mem_Write(ptr_i2c1, LSM6DSO_ADD_REG2, LSM6DSO_CTRL2_G,
                     ONEBYTE, &LSM6DSO_CTRL2_G_DATA, ONEBYTE, HAL_MAX_DELAY);

    return write_memory_ok;
}

