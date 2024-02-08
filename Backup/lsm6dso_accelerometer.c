/**
 * @file lsm6dso_accelerometer.c
 * @brief Contains functionality to read LSM6DSO accelerometer sensor data and calculate linear acceleration.
 *
 * @author Vegard Myhre
 * @date 31-01-2024
 * @version 1.0
 *
 * @see Datasheets/lsm6dso_gyroscope_datasheet.pdf
 */

#include "main.h"
#include "lsm6dso_accelerometer.h"
#include "math.h"

/* LSM6DSO Registers */
const uint8_t LSM6DSO_ADD_REG = 0xD6;

/* LSM6DSO acceleration registers */
const uint8_t LSM6DSO_OUTX_L_A = 0x28;
const uint8_t LSM6DSO_OUTX_H_A = 0x29;
const uint8_t LSM6DSO_OUTY_L_A = 0x2A;
const uint8_t LSM6DSO_OUTY_H_A = 0x2B;
const uint8_t LSM6DSO_OUTZ_L_A = 0x2C;
const uint8_t LSM6DSO_OUTZ_H_A = 0x2D;

/* LSM6DSO control registers */
const uint8_t LSM6DSO_CTRL1_XL = 0x10;
const uint8_t LSM6DSO_CTRL6_C = 0x15;

/* LSM6DSO data */
uint8_t LSM6DSO_CTRL1XL_DATA = 0x44;
const float LSM6DSO_ACC_SENSITIVITY = 0.000488f; //LSB/G 
uint8_t LSM6DSO_ACC_RAW_DATA[6];

/* Calibration data */
float vec_acc_offset[3];

float accX_offset = 0.0f;
float accY_offset = 0.0f;
float accZ_offset = 0.0f;

/* Initialise LSM6DSO accelerometer */
HAL_StatusTypeDef lsm6dso_acc_init(I2C_HandleTypeDef *ptr_i2c1)
{
    /* Set accelerometer to 2g full scale range and 104 Hz output data rate */
    HAL_StatusTypeDef write_mem_ok = HAL_I2C_Mem_Write(ptr_i2c1, LSM6DSO_ADD_REG, LSM6DSO_CTRL1_XL,
                    I2C_MEMADD_SIZE_8BIT, &LSM6DSO_CTRL1XL_DATA, 1, 1000);

    if (HAL_OK != write_mem_ok)
    {
        return write_mem_ok;
    }

    /* Calibrate accelerometer */
    lsm6dso_calibrate_acc(ptr_i2c1);

    return write_mem_ok;
}

/* Calibrates accelerometer */
void lsm6dso_calibrate_acc(I2C_HandleTypeDef *ptr_i2c1)
{
    /* Read and average 100 samples of accelerometer data */
    const uint8_t NUM_SAMPLES = 100;
    uint8_t vec_raw_data[6];
    float accX_sum = 0.0f;
    float accY_sum = 0.0f;
    float accZ_sum = 0.0f;

    for (uint8_t sample = 0; sample < NUM_SAMPLES; sample++)
    {
        HAL_StatusTypeDef read_i2c_status = HAL_I2C_Mem_Read(ptr_i2c1, LSM6DSO_ADD_REG, LSM6DSO_OUTX_L_A, I2C_MEMADD_SIZE_8BIT, vec_raw_data, 6, HAL_MAX_DELAY);

        if (HAL_OK == read_i2c_status)
        {

        accX_sum += (float) ((int16_t) (vec_raw_data[1] << 8) | vec_raw_data[0]);
        accY_sum += (float) ((int16_t) (vec_raw_data[3] << 8) | vec_raw_data[2]);
        accZ_sum += (float) ((int16_t) (vec_raw_data[5] << 8) | vec_raw_data[4]);

        HAL_Delay(10);
        }
        
    }

    /* Calculate accelerometer offsets */
    accX_offset = accX_sum / (float)NUM_SAMPLES;
    accY_offset = accY_sum / (float)NUM_SAMPLES;
    accZ_offset = accZ_sum / (float)NUM_SAMPLES;
}

/* Returns acceleration vector */
void lsm6dso_read_acc_vector(I2C_HandleTypeDef *ptr_i2c1, float *acceleration_vector)
{
    HAL_StatusTypeDef read_memory_ok = HAL_I2C_Mem_Read(ptr_i2c1, LSM6DSO_ADD_REG, LSM6DSO_OUTX_L_A, I2C_MEMADD_SIZE_8BIT, LSM6DSO_ACC_RAW_DATA, 6, HAL_MAX_DELAY);
    if (HAL_OK != read_memory_ok)
    {
        acceleration_vector[0] = -999.0f;
        acceleration_vector[1] = -999.0f;
        acceleration_vector[2] = -999.0f;
    }
    else
    {
        /* This should be split up into variables to improve readability */
        acceleration_vector[0] = ((float)((int16_t)((LSM6DSO_ACC_RAW_DATA[1] << 8) | LSM6DSO_ACC_RAW_DATA[0])) - vec_acc_offset[0]) * LSM6DSO_ACC_SENSITIVITY;
        acceleration_vector[1] = ((float)((int16_t)((LSM6DSO_ACC_RAW_DATA[3] << 8) | LSM6DSO_ACC_RAW_DATA[2])) - vec_acc_offset[1]) * LSM6DSO_ACC_SENSITIVITY;
        acceleration_vector[2] = ((float)((int16_t)((LSM6DSO_ACC_RAW_DATA[5] << 8) | LSM6DSO_ACC_RAW_DATA[4])) - vec_acc_offset[2]) * LSM6DSO_ACC_SENSITIVITY;
    }
}

/* Returns acceleration in one axis in mg's */
float lsm6dso_read_acc_axis(I2C_HandleTypeDef *ptr_i2c1, uint8_t high_byte_reg, uint8_t low_byte_reg, float acc_axis_offset)
{
    uint8_t high_byte_data = 0;
    uint8_t low_byte_data = 0;

    HAL_StatusTypeDef read_high_byte = HAL_I2C_Mem_Read(ptr_i2c1, LSM6DSO_ADD_REG, high_byte_reg,
                     I2C_MEMADD_SIZE_8BIT, &high_byte_data, 1, HAL_MAX_DELAY);
    HAL_StatusTypeDef read_low_byte = HAL_I2C_Mem_Read(ptr_i2c1, LSM6DSO_ADD_REG, low_byte_reg,
                     I2C_MEMADD_SIZE_8BIT, &low_byte_data, 1, HAL_MAX_DELAY);
    
    if ( (HAL_OK != read_high_byte) || (HAL_OK != read_low_byte) )
    {
        return -999.0f;
    }
    else
    {
        int16_t combined_data = (high_byte_data << 8) | low_byte_data;
        float acceleration = (((float)combined_data * LSM6DSO_ACC_SENSITIVITY) - acc_axis_offset);
        return acceleration;
    }
}



