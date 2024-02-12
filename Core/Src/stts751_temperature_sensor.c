#include "main.h"
#include "stts751_temperature_sensor.h"

/* STTS751 Registers */
const uint8_t STTS751_HIGH_BYTE_REG = 0x00;
const uint8_t STTS751_STATUS_REG    = 0x01;
const uint8_t STTS751_LOW_BYTE_REG  = 0x02;
const uint8_t STTS751_CONF_REG      = 0x03;
const uint8_t STTS751_CONV_REG      = 0x04;
const uint8_t STTS751_ADDR_REG      = 0x94;

/* STTS751 Data */
const uint8_t STTS751_CONF_DATA     = 0x8C;
const uint8_t STTS751_CONV_DATA     = 0x07;
const uint8_t STTS751_REG_SIZE      = 0x01;
const uint8_t STTS751_DATA_SIZE     = 0x01;

int stts751_init(I2C_HandleTypeDef *ptr_i2c1)
{
    uint8_t write_memory_ok = 1;
    HAL_StatusTypeDef write_conf_reg;
    
    write_conf_reg = HAL_I2C_Mem_Write(ptr_i2c1, STTS751_ADDR_REG, STTS751_CONF_REG,
                     STTS751_DATA_SIZE, STTS751_CONF_DATA, STTS751_DATA_SIZE, 50);

    switch (write_conf_reg)
    {
    case !HAL_OK:
        write_memory_ok = 0;
        break;
    
    default:
        break;
    }

    /* TODO Skriv til conv reg */

    return write_memory_ok;
}

void stts751_read_temperature(I2C_HandleTypeDef *ptr_i2c1, float *temperature)
{
    uint8_t temperature_high;
    uint8_t temperature_low;
    int16_t temperature_combined;
    HAL_StatusTypeDef read_high_byte_status;
    HAL_StatusTypeDef read_low_byte_status;
    // float temperature = 0.0f;

    read_high_byte_status = HAL_I2C_Mem_Read(ptr_i2c1, STTS751_ADDR_REG, STTS751_HIGH_BYTE_REG,
                            STTS751_REG_SIZE, &temperature_high, STTS751_DATA_SIZE, HAL_MAX_DELAY);

    read_low_byte_status = HAL_I2C_Mem_Read(ptr_i2c1, STTS751_ADDR_REG, STTS751_LOW_BYTE_REG,
                            STTS751_REG_SIZE, &temperature_low, STTS751_DATA_SIZE, HAL_MAX_DELAY);

    if (read_high_byte_status || read_high_byte_status != HAL_OK)
    {
        *temperature = -999.0f; /* Error value */
    }
    else
    {  
        temperature_combined = (temperature_combined | temperature_high) << 8;
        temperature_combined |= temperature_low;
        temperature_combined >>= 4;

        /* If the temperature is a negative value */
        if (temperature_combined & 0x800) 
        { 
            temperature_combined |= 0xF000;
        }
        
        *temperature = (float)temperature_combined * 0.0625f; // Convert to temperature with 0.0625 resolution
    }
}