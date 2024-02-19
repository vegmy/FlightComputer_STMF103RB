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

#include "math_filter.h"

float IIR_filter(float raw_data, float iir_data, float alpha)
{
    return alpha * raw_data + (1 - alpha) * iir_data;
}