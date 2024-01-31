#include "main.h"
#include "lsm6dso_gyroscope.h"
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
const uint8_t LSM6DSO_CTRL1XL_DATA = 0x44;
const float LSM6DSO_ACC_SENSITIVITY = 0.122f;

/* Calibration data */
float accX_offset = 0.0f;
float accY_offset = 0.0f;
float accZ_offset = 0.0f;



HAL_StatusTypeDef lsm6dso_init(I2C_HandleTypeDef *ptr_i2c1)
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

float lsm6dso_read_linear_acc(I2C_HandleTypeDef *ptr_i2c1)
{
    float accX = lsm6dso_read_acc_axis(ptr_i2c1, LSM6DSO_OUTX_H_A, LSM6DSO_OUTX_L_A, accX_offset);
    float accY = lsm6dso_read_acc_axis(ptr_i2c1, LSM6DSO_OUTY_H_A, LSM6DSO_OUTY_L_A, accY_offset);
    float accZ = lsm6dso_read_acc_axis(ptr_i2c1, LSM6DSO_OUTZ_H_A, LSM6DSO_OUTZ_L_A, accZ_offset);
    
    float acceleration = sqrt(accX * accX + accY * accY + accZ * accZ);

    return acceleration;
}

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
        return 0;
    }
    else
    {
        int16_t combined_data = (high_byte_data << 8) | low_byte_data;

        float acceleration = ((float)combined_data * LSM6DSO_ACC_SENSITIVITY) + acc_axis_offset;
        return acceleration;
    }
}

void lsm6dso_calibrate_acc(I2C_HandleTypeDef *ptr_i2c1)
{
    /* Read and average 100 samples of accelerometer data */
    const uint8_t NUM_SAMPLES = 100;
    float accX_sum = 0.0f;
    float accY_sum = 0.0f;
    float accZ_sum = 0.0f;

    for (uint8_t i = 0; i < NUM_SAMPLES; i++)
    {
        float accX_raw = lsm6dso_read_acc_axis(ptr_i2c1, LSM6DSO_OUTX_H_A, LSM6DSO_OUTX_L_A, accX_offset);
        float accY_raw = lsm6dso_read_acc_axis(ptr_i2c1, LSM6DSO_OUTY_H_A, LSM6DSO_OUTY_L_A, accY_offset);
        float accZ_raw = lsm6dso_read_acc_axis(ptr_i2c1, LSM6DSO_OUTZ_H_A, LSM6DSO_OUTZ_L_A, accZ_offset);

        accX_sum += accX_raw;
        accY_sum += accY_raw;
        accZ_sum += accZ_raw;

        HAL_Delay(10);
    }

    float accX_avg = accX_sum / (float)NUM_SAMPLES;
    float accY_avg = accY_sum / (float)NUM_SAMPLES;
    float accZ_avg = accZ_sum / (float)NUM_SAMPLES;

    /* Calculate accelerometer offsets */
    accX_offset = -accX_avg * LSM6DSO_ACC_SENSITIVITY;
    accY_offset = -accY_avg * LSM6DSO_ACC_SENSITIVITY;
    accZ_offset = (1.0f - accZ_avg) * LSM6DSO_ACC_SENSITIVITY;
}

