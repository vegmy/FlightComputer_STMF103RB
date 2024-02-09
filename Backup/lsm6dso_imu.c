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
// #include "lsm6dso_imu.h"
#include "math.h"

/* LSM6DSO Address Register */
const uint8_t LSM6DSO_ADD_REG  = 0xD6;

/* LSM6DSO Control Register */
const uint8_t LSM6DSO_CTRL1_XL = 0x10;
const uint8_t LSM6DSO_CTRL2_G  = 0x11;
const uint8_t LSM6DSO_CTRL6_C  = 0x15;

/* LSM6DSO Data Registers */
const uint8_t LSM6DSO_OUTX_L_G = 0x22;
const uint8_t LSM6DSO_OUTX_H_G = 0x23;
const uint8_t LSM6DSO_OUTY_L_G = 0x24;
const uint8_t LSM6DSO_OUTY_H_G = 0x25;
const uint8_t LSM6DSO_OUTZ_L_G = 0x26;
const uint8_t LSM6DSO_OUTZ_H_G = 0x27;
const uint8_t LSM6DSO_OUTX_L_A = 0x28;
const uint8_t LSM6DSO_OUTX_H_A = 0x29;
const uint8_t LSM6DSO_OUTY_L_A = 0x2A;
const uint8_t LSM6DSO_OUTY_H_A = 0x2B;
const uint8_t LSM6DSO_OUTZ_L_A = 0x2C;
const uint8_t LSM6DSO_OUTZ_H_A = 0x2D;

 /* 
 * LSM6DSO register data                                
 * Gyro control register is set to high performance mode: 104Hz
 * Gyro control register data:          0x4C = 0100 0110 => 2000dps
 * Accelerometer control register Data: 0x44 = 0100 0100 => 104Hz,                
 * Accelerometer sensitivity:           0.00488 [LSB/g]
 * Gyroscope sensitivity:               0.07    [LSB/g]                                                     
 */
const uint8_t LSM6DSO_CTRL1XL_DATA   = 0x44;
const uint8_t LSM6DSO_CTRL2_G_DATA   = 0x4C;
const float LSM6DSO_ACC_SENSITIVITY  = 0.000488f; //LSB/G 
const float LSM6DSO_GYRO_SENSITIVITY = 0.07f;

/* LSM6DSO IMU offsets */
float vec_acc_offset[3]  = {0.0f, 0.0f, 0.0f};
float vec_gyro_offset[3] = {0.0f, 0.0f, 0.0f};

/* Writes to LSM6DSO registers */
uint8_t lsm6dso_imu_init(I2C_TypeDef *ptr_i2c1)
{
    HAL_StatusTypeDef gyroscope_status = HAL_I2C_Mem_Write(ptr_i2c1, LSM6DSO_ADD_REG, LSM6DSO_CTRL2_G,
                     I2C_MEMADD_SIZE_8BIT, &LSM6DSO_CTRL2_G_DATA, 1, 1000);
    HAL_StatusTypeDef accelerometer_status = HAL_I2C_Mem_Write(ptr_i2c1, LSM6DSO_ADD_REG, LSM6DSO_CTRL1_XL, 
                     I2C_MEMADD_SIZE_8BIT, &LSM6DSO_CTRL1XL_DATA, 1, 1000); 
    uint8_t lsm6dso_status = 1;

    if ((HAL_OK != accelerometer_status) && (HAL_OK != gyroscope_status))
    {
        lsm6dso_status = 0;
    }

    lsm6dso_calibrate(ptr_i2c1, LSM6DSO_OUTX_L_A, vec_acc_offset);
    lsm6dso_calibrate(ptr_i2c1, LSM6DSO_OUTX_L_G, vec_gyro_offset);

    return lsm6dso_status;
}

void lsm6dso_calibrate(I2C_TypeDef *ptr_i2c1, uint8_t read_start_register, float *vec_offset)
{
    const uint8_t NUM_SAMPLES = 100;
    float vec_sample_sum[3] = {0.0f, 0.0f, 0.0f};
    uint8_t vec_raw_data[6];

    for (int sample = 0; sample < NUM_SAMPLES; sample++) 
    {
        HAL_StatusTypeDef mem_read_status = HAL_I2C_Mem_Read(ptr_i2c1, LSM6DSO_ADD_REG, read_start_register,
                     I2C_MEMADD_SIZE_8BIT, vec_raw_data, 6, HAL_MAX_DELAY);

        if (mem_read_status == HAL_OK) 
        {
            vec_sample_sum[0] += (float) ((int16_t) ((vec_raw_data[1] << 8) | vec_raw_data[0]));
            vec_sample_sum[1] += (float) ((int16_t) ((vec_raw_data[3] << 8) | vec_raw_data[2]));
            vec_sample_sum[2] += (float) ((int16_t) ((vec_raw_data[5] << 8) | vec_raw_data[4]));
        }

        HAL_Delay(10);
    }

    vec_offset[0] = ( (vec_sample_sum[0] / (float)NUM_SAMPLES));
    vec_offset[1] = ( (vec_sample_sum[1] / (float)NUM_SAMPLES));
    vec_offset[2] = ( (vec_sample_sum[2] / (float)NUM_SAMPLES));
}

/* 
* Reads the 3 axis with 6 degrees of freedom from the IMU.
* returns a vector with length 6.
*/
void lsm6dso_read_imu(I2C_TypeDef *ptr_i2c1, float *vec_imu)
{
    /* IMU Rotation speed values */
    vec_imu[0] = lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTX_L_G, vec_gyro_offset[0]) * LSM6DSO_GYRO_SENSITIVITY;
    vec_imu[1] = lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTY_L_G, vec_gyro_offset[1]) * LSM6DSO_GYRO_SENSITIVITY;
    vec_imu[2] = lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTZ_L_G, vec_gyro_offset[2]) * LSM6DSO_GYRO_SENSITIVITY;
    
    /* IMU Acceleration values */
    vec_imu[3] = lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTX_L_A, vec_acc_offset[0]) * LSM6DSO_ACC_SENSITIVITY;
    vec_imu[4] = lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTY_L_A, vec_acc_offset[1]) * LSM6DSO_ACC_SENSITIVITY;
    vec_imu[5] = lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTZ_L_A, vec_acc_offset[2]) * LSM6DSO_ACC_SENSITIVITY;
}

void lsm6dso_read_vector(I2C_HandleTypeDef *ptr_i2c1, uint8_t read_start_register, float *offset_vector, float *return_vector)
{
    const uint8_t NUM_AXIS = 3;

    for(int axis = 0; axis <= NUM_AXIS; axis++)
    {
        return_vector[axis] = lsm6dso_read_axis(ptr_i2c1, read_start_register, offset_vector[axis]);
        read_start_register += 2;
    }
}

float lsm6dso_read_axis(I2C_HandleTypeDef *ptr_i2c1, uint8_t read_start_register, float offset_axis)
{
    uint8_t vec_axis_raw[2];
    int16_t raw_data_combined;
    HAL_StatusTypeDef read_axis_status = HAL_I2C_Mem_Read(ptr_i2c1, LSM6DSO_ADD_REG, read_start_register, I2C_MEMADD_SIZE_8BIT, vec_axis_raw, 2, HAL_MAX_DELAY);

    if (HAL_OK != read_axis_status)
    {
        vec_axis_raw[0] = -999.0f;
        vec_axis_raw[1] = -999.0f;
    }
    else
    { 
     raw_data_combined = (int16_t) ((vec_axis_raw[1] << 8) | vec_axis_raw[0]);   
    }

    float return_data = ((float) raw_data_combined) - offset_axis;

    return return_data;
}