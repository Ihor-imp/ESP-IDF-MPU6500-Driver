#include <MPU6500.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

esp_err_t mpu_init(mpu *sensor)
{
    if (sensor == NULL || sensor->device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data[] = {0x6B, 0x01};
    esp_err_t ret = i2c_master_transmit(sensor->device, data, sizeof(data), 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

esp_err_t mpu_who_i_am(mpu *sensor, uint8_t *indef)
{
    uint8_t reg_addr = 0x75;
    if (sensor == NULL || indef == NULL || sensor->device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_transmit_receive(sensor->device, &reg_addr, sizeof(reg_addr), indef, sizeof(uint8_t), 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}

esp_err_t mpu_set_accel_range(mpu *sensor, uint8_t range)
{
    if (sensor == NULL || sensor->device == NULL || range > 3)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data[] = {0x1C, (uint8_t)(range << 3)};
    uint8_t reg_addr = 0x1C;
    uint8_t value = 0;

    esp_err_t ret = i2c_master_transmit(sensor->device, data, sizeof(data), 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = i2c_master_transmit_receive(sensor->device, &reg_addr, sizeof(reg_addr), &value, sizeof(value), 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }
    if ((value & 0x18) != (range << 3))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t mpu_set_gyro_range(mpu *sensor, uint8_t range)
{
    if (sensor == NULL || sensor->device == NULL || range > 3)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data[] = {0x1B, (uint8_t)(range << 3)};
    uint8_t value = 0;

    esp_err_t ret = i2c_master_transmit(sensor->device, data, sizeof(data), 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = i2c_master_transmit_receive(sensor->device, &data[0], sizeof(data[0]), &value, sizeof(value), 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }
    if ((value & 0x18) != (range << 3))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}