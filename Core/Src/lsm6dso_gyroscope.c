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

/* LSM6DSO device register on I2C bus */
const uint8_t LSM6DSO_ADD_REG2 = 0xD6;

/// Gyroscope control registers
const uint8_t LSM6DSO_CTRL2_G  = 0x11;

/* LSM6DSO gyroscope registers */
const uint8_t LSM6DSO_OUTX_L_G = 0x22;
const uint8_t LSM6DSO_OUTX_H_G = 0x23;
const uint8_t LSM6DSO_OUTY_L_G = 0x24;
const uint8_t LSM6DSO_OUTY_H_G = 0x25;
const uint8_t LSM6DSO_OUTZ_L_G = 0x26;
const uint8_t LSM6DSO_OUTZ_H_G = 0x27;

 /* LSM6DSO register data                                
 *  Gyro control register is set to high performance mode 
 *  Data: 0x44 = 0100 0100 => 104Hz, 2000dps               
 *                                                      */
const uint16_t ONEBYTE               = 1;
const uint8_t LSM6DSO_CTRL2_G_DATA   = 0x4C;
const float LSM6DSO_GYRO_SENSITIVITY = 0.07f;
int16_t LSM6DSO_GYRO_RAW_DATA[6];

 /* Initialise gyoroscope, write to control register *
 * 
 * */
HAL_StatusTypeDef lsm6dso_gyroscope_init(I2C_HandleTypeDef *ptr_i2c1)
{
    HAL_StatusTypeDef write_memory_ok = HAL_I2C_Mem_Write(ptr_i2c1, LSM6DSO_ADD_REG2, LSM6DSO_CTRL2_G,
                     ONEBYTE, &LSM6DSO_CTRL2_G_DATA, ONEBYTE, 1000);

    return write_memory_ok;
}


void lsm6dso_read_rot_vector(I2C_HandleTypeDef *ptr_i2c1, float *gyro_vector)
{
    HAL_StatusTypeDef read_memory_ok = HAL_I2C_Mem_Read(ptr_i2c1, LSM6DSO_ADD_REG2, LSM6DSO_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, LSM6DSO_GYRO_RAW_DATA, 6, 50);
    if (HAL_OK != read_memory_ok)
    {
        gyro_vector[0] = -999.0f;
        gyro_vector[1] = -999.0f;
        gyro_vector[2] = -999.0f;
    }
    else
    {
        int16_t gyroX = (int16_t)((LSM6DSO_GYRO_RAW_DATA[1] << 8) | LSM6DSO_GYRO_RAW_DATA[0]);
        int16_t gyroY = (int16_t)((LSM6DSO_GYRO_RAW_DATA[3] << 8) | LSM6DSO_GYRO_RAW_DATA[2]);
        int16_t gyroZ = (int16_t)((LSM6DSO_GYRO_RAW_DATA[5] << 8) | LSM6DSO_GYRO_RAW_DATA[4]);
        
        gyro_vector[0] = (float)gyroX * LSM6DSO_GYRO_SENSITIVITY; // Convert to dps
        gyro_vector[1] = (float)gyroY * LSM6DSO_GYRO_SENSITIVITY; // Convert to dps
        gyro_vector[2] = (float)gyroZ * LSM6DSO_GYRO_SENSITIVITY;
    }
}

void lsm6dso_calibrate_gyro(I2C_HandleTypeDef *ptr_i2c1, float *gyro_offset) 
{
    float sumX = 0, sumY = 0, sumZ = 0;
    float gyro_vector[3];
    
    for (int i = 0; i < 100; i++) {
        lsm6dso_read_gyro(ptr_i2c1, gyro_vector);
        sumX += gyro_vector[0];
        sumY += gyro_vector[1];
        sumZ += gyro_vector[2];
        HAL_Delay(10); // Short delay between readings
    }
    
    gyro_offset[0] = sumX / 100.0f;
    gyro_offset[1] = sumY / 100.0f;
    gyro_offset[2] = sumZ / 100.0f;
}


