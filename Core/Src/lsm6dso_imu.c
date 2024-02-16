/**
 * @file lsm6dso_gyroscope.c
 * @brief Contains functionality to read LSM6DSO gyroscope sensor data and calculate rotation rate.
 *
 * @author Vegard Myhre
 * @date 31-01-2024
 * @version 1.0
 *
 * @see Datasheets/lsm6dso_acc_gyro_datasheet.pdf
 * @TODO: Better error value handling and sanitization.
 */

#include "stm32f1xx_hal.h"
#include "lsm6dso_imu.h"
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
const float LSM6DSO_ACC_SENSITIVITY  = 0.000488f; // [LSB/g]
const float LSM6DSO_GYRO_SENSITIVITY = 0.07f;     // [LSB/g]

/* LSM6DSO IMU offsets */
const uint8_t NUM_SAMPLES = 100;
float vec_acc_offset[3]  = {0.0f, 0.0f, 0.0f};
float vec_gyro_offset[3] = {0.0f, 0.0f, 0.0f};

/**
 * @brief Writes to LSM6DSO control register for configuration and calibrates the accelermoeter and gyroscope. \n.
 *
 * Sets both accelerometer and gyroscope into high performance mode (104Hz) 
 * by writing the value 0x4C into gyro control register CTRL2_G and
 * 0x44 into accelerometer control register CTRL1_XL.
 * These values sets gyroscope measurement at 2000 [dps] max and accelerometer
 * measurements at +- 16g.
 * 
 * After writing to the registers the function calibrates the accelerometer and gyroscope by calling lsm6dso_calibrate()
 * for both sensors.
 * 
 * @param ptr_i2c1 Pointer to I2C1 bus.
 * @return Returns 0 if there is an error and 1 if the memory write was succsesful.
 */
uint8_t lsm6dso_imu_init(I2C_HandleTypeDef *ptr_i2c1)
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

    lsm6dso_calibrate_sensor(ptr_i2c1, LSM6DSO_OUTX_L_A, vec_acc_offset, NUM_SAMPLES);
    lsm6dso_calibrate_sensor(ptr_i2c1, LSM6DSO_OUTX_L_G, vec_gyro_offset, NUM_SAMPLES);
    
    return lsm6dso_status;
}


/**
 * @brief Calibrates a sensor of the IMU LSM6DSO.
 *
 * @param ptr_i2c1 Pointer to I2C1 bus.
 * @param read_start_register Start register for the sensor to calibrate, this will read memory from the I2C bus of size 6.
 * @param vec_offset Vector to put the offset into, needs to be of size 3.
 */
void lsm6dso_calibrate_sensor(I2C_HandleTypeDef *ptr_i2c1, uint8_t read_start_register, float *vec_offset, const uint8_t NUM_SAMPLES)
{
    // const uint8_t NUM_SAMPLES = 100;
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

/**
 * @brief Reads 6 axis of the LSM6DSO IMU by first reading 3 axis of the gyroscope, then 3 axis of the accelerometer 
 * and puts them into a vector.
 *
 * @param ptr_i2c1 Pointer to I2C1 bus.
 * @param vec_imu Vector to put the IMU values into. This needs to be of size 6.
 */
void lsm6dso_read_imu(I2C_HandleTypeDef *ptr_i2c1, float *vec_imu)
{
    /* IMU Rotation speed values */
    vec_imu[0] = lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTX_L_G, vec_gyro_offset[0]) * LSM6DSO_GYRO_SENSITIVITY;
    vec_imu[1] = lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTY_L_G, vec_gyro_offset[1]) * LSM6DSO_GYRO_SENSITIVITY;
    vec_imu[2] = lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTZ_L_G, vec_gyro_offset[2]) * LSM6DSO_GYRO_SENSITIVITY;
    
    /* IMU Acceleration values */
    vec_imu[3] = (lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTX_L_A, vec_acc_offset[0])  ) * LSM6DSO_ACC_SENSITIVITY;
    vec_imu[4] = (lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTY_L_A, vec_acc_offset[1])  ) * LSM6DSO_ACC_SENSITIVITY;
    vec_imu[5] = (lsm6dso_read_axis(ptr_i2c1, LSM6DSO_OUTZ_L_A, vec_acc_offset[2]) ) * LSM6DSO_ACC_SENSITIVITY;
}

/**
 * @brief Reads 3 axis from one sensor of the LSM6DSO IMU and puts the values into a vector.
 * @TODO fix this.
 * @param ptr_i2c1 Pointer to I2C1 bus.
 * @param read_start_register Register to start reading from, this is incremented by 2 to read both high and low byte from each XYZ axis.
 * @param offset_vector The offset vector for the sensor.
 * @param vec_imu Vector to put the IMU values into. This needs to be of size 3.
 */
// void lsm6dso_read_vector(I2C_HandleTypeDef *ptr_i2c1, uint8_t read_start_register, float *offset_vector, float *vec_imu)
// {
//     const uint8_t NUM_AXIS = 3;

//     for(int axis = 0; axis <= NUM_AXIS; axis++)
//     {
//         vec_imu[axis] = lsm6dso_read_axis(ptr_i2c1, read_start_register, offset_vector[axis]);
//         read_start_register += 2;
//     }
// }

/**
 * @brief Reads 1 axis from a sensor the LSM6DSO IMU and returns it as a float value.
 *
 * @param ptr_i2c1 Pointer to I2C1 bus.
 * @param read_start_register Register to start reading from, this is the low byte register.
 * @param offset_axis The offset axis of the same axis to read from.
 */
float lsm6dso_read_axis(I2C_HandleTypeDef *ptr_i2c1, uint8_t read_start_register, float offset_axis)
{
    uint8_t vec_axis_raw[2];
    int16_t low_byte = 0;
    int16_t high_byte = 0;
    int16_t raw_data_combined;
    HAL_StatusTypeDef read_axis_status = HAL_I2C_Mem_Read(ptr_i2c1, LSM6DSO_ADD_REG, read_start_register, I2C_MEMADD_SIZE_8BIT, vec_axis_raw, 2, 50);

    if (HAL_OK != read_axis_status)
    {
        // vec_axis_raw[0] = 999.0f;
        // vec_axis_raw[1] = 999.0f;
        return -999.0f;
    }
    else
    {
        low_byte = (int16_t) vec_axis_raw[0];
        high_byte = (int16_t) vec_axis_raw[1];

        // raw_data_combined = (int32_t) ( (high_byte << 16) | low_byte );   
        raw_data_combined = (int16_t) ((vec_axis_raw[1] << 8) | vec_axis_raw[0]);   
    }

    float return_data = ((float) raw_data_combined) - offset_axis;

    return return_data;
}

void lsm6dso_update_acc_offset(I2C_HandleTypeDef *ptr_i2c1)
{
    lsm6dso_calibrate_sensor(ptr_i2c1, LSM6DSO_OUTX_L_A, vec_acc_offset, 10);
}