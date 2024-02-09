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

/* LSM6DSO device address */
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

 /*
 *  LSM6DSO register data                                 
 *  Gyro control register is set to high performance mode 
 *  Data: 0x44 = 0100 0100 => 104Hz, 2000dps               
 */

const uint8_t LSM6DSO_CTRL2_G_DATA   = 0x4C;
const float LSM6DSO_GYRO_SENSITIVITY = 0.07f;
float gyro_offset[3];
uint8_t LSM6DSO_GYRO_RAW_DATA[6];

 /* Initialise gyoroscope, write to control register *
 * 
 * */
HAL_StatusTypeDef lsm6dso_gyroscope_init(I2C_HandleTypeDef *ptr_i2c1)
{
    HAL_StatusTypeDef write_memory_ok = HAL_I2C_Mem_Write(ptr_i2c1, LSM6DSO_ADD_REG2, LSM6DSO_CTRL2_G,
                     I2C_MEMADD_SIZE_8BIT, &LSM6DSO_CTRL2_G_DATA, 1, 1000);

    lsm6dso_calibrate_gyro(ptr_i2c1);

    return write_memory_ok;
}

void lsm6dso_read_rot_vector(I2C_HandleTypeDef *ptr_i2c1, float *gyro_vector)
{
    HAL_StatusTypeDef read_memory_ok = HAL_I2C_Mem_Read(ptr_i2c1, LSM6DSO_ADD_REG2, LSM6DSO_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, LSM6DSO_GYRO_RAW_DATA, 6, HAL_MAX_DELAY);
    if (HAL_OK != read_memory_ok)
    {
        gyro_vector[0] = -999.0f;
        gyro_vector[1] = -999.0f;
        gyro_vector[2] = -999.0f;
    }
    else
    {
        /* This should be split up into variables to improve readability */
        gyro_vector[0] = ((float)((int16_t)((LSM6DSO_GYRO_RAW_DATA[1] << 8) | LSM6DSO_GYRO_RAW_DATA[0])) - gyro_offset[0]) * LSM6DSO_GYRO_SENSITIVITY;
        gyro_vector[1] = ((float)((int16_t)((LSM6DSO_GYRO_RAW_DATA[3] << 8) | LSM6DSO_GYRO_RAW_DATA[2])) - gyro_offset[1]) * LSM6DSO_GYRO_SENSITIVITY;
        gyro_vector[2] = ((float)((int16_t)((LSM6DSO_GYRO_RAW_DATA[5] << 8) | LSM6DSO_GYRO_RAW_DATA[4])) - gyro_offset[2]) * LSM6DSO_GYRO_SENSITIVITY;
    }
}

/*  Calibrates gyroscope  
    @TODO Should be more readable */
void lsm6dso_calibrate_gyro(I2C_HandleTypeDef *ptr_i2c1) 
{
    const uint8_t NUM_SAMPLES = 100;
    float vec_sample_sum[3] = {0.0f, 0.0f, 0.0f};
    uint8_t vec_raw_data[6];

    for (int sample = 0; sample < NUM_SAMPLES; sample++) 
    {
        HAL_StatusTypeDef mem_read_status = HAL_I2C_Mem_Read(ptr_i2c1, LSM6DSO_ADD_REG2, LSM6DSO_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, vec_raw_data, 6, HAL_MAX_DELAY);
        if (mem_read_status == HAL_OK) 
        {
            vec_sample_sum[0] += (float) ((int16_t) ((vec_raw_data[1] << 8) | vec_raw_data[0]));
            vec_sample_sum[1] += (float) ((int16_t) ((vec_raw_data[3] << 8) | vec_raw_data[2]));
            vec_sample_sum[2] += (float) ((int16_t) ((vec_raw_data[5] << 8) | vec_raw_data[4]));
        }

        HAL_Delay(10);
    }

    gyro_offset[0] = ( (vec_sample_sum[0] / (float)NUM_SAMPLES));
    gyro_offset[1] = ( (vec_sample_sum[1] / (float)NUM_SAMPLES));
    gyro_offset[2] = ( (vec_sample_sum[2] / (float)NUM_SAMPLES));
}