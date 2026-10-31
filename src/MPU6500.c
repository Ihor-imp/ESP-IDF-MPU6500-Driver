#include <MPU6500.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <math.h>
#include "esp_log.h"

esp_err_t mpu_init(mpu *sensor)
{
    if (sensor == NULL || sensor->device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    sensor->accel_offset_x = 0.0f;
    sensor->accel_offset_y = 0.0f;
    sensor->accel_offset_z = 0.0f;

    sensor->accel_scale_x = 1.0f;
    sensor->accel_scale_y = 1.0f;
    sensor->accel_scale_z = 1.0f;

    sensor->gyro_offset_x = 0.0f;
    sensor->gyro_offset_y = 0.0f;
    sensor->gyro_offset_z = 0.0f;
    
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

    sensor->accel_range = range;

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

    sensor->gyro_range = range;

    return ESP_OK;
}

esp_err_t mpu_read_accel(mpu *sensor, float *x, float *y, float *z)
{
    if (sensor == NULL || sensor->device == NULL || x == NULL || y == NULL || z == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr = 0x3B;
    uint8_t data[6] = {0};
    const float sensitivity[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};

    esp_err_t ret = i2c_master_transmit_receive(sensor->device, &reg_addr, sizeof(reg_addr), data, sizeof(data), 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }

    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_y = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

    *x = (raw_x / sensitivity[sensor->accel_range] - sensor->accel_offset_x) * sensor->accel_scale_x;
    *y = (raw_y / sensitivity[sensor->accel_range] - sensor->accel_offset_y) * sensor->accel_scale_y;
    *z = (raw_z / sensitivity[sensor->accel_range] - sensor->accel_offset_z) * sensor->accel_scale_z;

    return ESP_OK;
}

esp_err_t mpu_read_gyro(mpu *sensor, float *x, float *y, float *z)
{
    if (sensor == NULL || sensor->device == NULL || x == NULL || y == NULL || z == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr = 0x43;
    uint8_t data[6] = {0};
    float sensitivity[] = {131.f, 65.5f, 32.8f, 16.4f};

    esp_err_t ret = i2c_master_transmit_receive(sensor->device, &reg_addr, sizeof(reg_addr), data, sizeof(data), 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }

    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_y = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

    *x = raw_x / sensitivity[sensor->gyro_range] - sensor->gyro_offset_x;
    *y = raw_y / sensitivity[sensor->gyro_range] - sensor->gyro_offset_y;
    *z = raw_z / sensitivity[sensor->gyro_range] - sensor->gyro_offset_z;

    return ESP_OK;
}

esp_err_t mpu_calibrate_accel(mpu *sensor)
{
    if (sensor == NULL || sensor->device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const char *positions[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};

    const int sample_count = 500;

    float samples[6] = {0};

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float old_offset_x = sensor->accel_offset_x;
    float old_offset_y = sensor->accel_offset_y;
    float old_offset_z = sensor->accel_offset_z;

    float old_scale_x = sensor->accel_scale_x;
    float old_scale_y = sensor->accel_scale_y;
    float old_scale_z = sensor->accel_scale_z;

    sensor->accel_offset_x = 0.0f;
    sensor->accel_offset_y = 0.0f;
    sensor->accel_offset_z = 0.0f;

    sensor->accel_scale_x = 1.0f;
    sensor->accel_scale_y = 1.0f;
    sensor->accel_scale_z = 1.0f;

    esp_err_t ret = ESP_OK;

    for (int position = 0; position < 6; position++)
    {
        ESP_LOGI("MPU6500", "Place sensor: %s", positions[position]);
        ESP_LOGI("MPU6500", "You have 5 seconds");

        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI("MPU6500", "Measuring %s...", positions[position]);

        float sum = 0.0f;

        for (int i = 0; i < sample_count; i++)
        {
            ret = mpu_read_accel(sensor, &x, &y, &z);

            if (ret != ESP_OK)
            {
                goto calibration_failed;
            }

            if (position == 0 || position == 1)
            {
                sum += x;
            }
            else if (position == 2 || position == 3)
            {
                sum += y;
            }
            else
            {
                sum += z;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        samples[position] = sum / (float)sample_count;

        ESP_LOGI("MPU6500", "%s = %.4f g",
                 positions[position], samples[position]);
    }

    float dx = samples[0] - samples[1];
    float dy = samples[2] - samples[3];
    float dz = samples[4] - samples[5];

    if (dx < 1.5f || dx > 2.5f ||
        dy < 1.5f || dy > 2.5f ||
        dz < 1.5f || dz > 2.5f)
    {
        ret = ESP_ERR_INVALID_STATE;
        goto calibration_failed;
    }

    sensor->accel_offset_x = (samples[0] + samples[1]) / 2.0f;
    sensor->accel_offset_y = (samples[2] + samples[3]) / 2.0f;
    sensor->accel_offset_z = (samples[4] + samples[5]) / 2.0f;

    sensor->accel_scale_x = 2.0f / dx;
    sensor->accel_scale_y = 2.0f / dy;
    sensor->accel_scale_z = 2.0f / dz;

    ESP_LOGI("MPU6500", "Accelerometer calibration completed");

    ESP_LOGI("MPU6500", "Offset: X=%.4f Y=%.4f Z=%.4f",
             sensor->accel_offset_x,
             sensor->accel_offset_y,
             sensor->accel_offset_z);

    ESP_LOGI("MPU6500", "Scale: X=%.4f Y=%.4f Z=%.4f",
             sensor->accel_scale_x,
             sensor->accel_scale_y,
             sensor->accel_scale_z);

    return ESP_OK;

calibration_failed:

    sensor->accel_offset_x = old_offset_x;
    sensor->accel_offset_y = old_offset_y;
    sensor->accel_offset_z = old_offset_z;

    sensor->accel_scale_x = old_scale_x;
    sensor->accel_scale_y = old_scale_y;
    sensor->accel_scale_z = old_scale_z;

    ESP_LOGE("MPU6500", "Accelerometer calibration failed");

    return ret;
}

esp_err_t mpu_calibrate_gyro(mpu *sensor)
{
    if (sensor == NULL || sensor->device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    sensor->gyro_offset_x = 0;
    sensor->gyro_offset_y = 0;
    sensor->gyro_offset_z = 0;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    for (int i = 0; i < 500; i++)
    {
        esp_err_t ret = mpu_read_gyro(sensor, &x, &y, &z);
        if (ret != ESP_OK)
        {
            return ret;
        }

        sum_x += x;
        sum_y += y;
        sum_z += z;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    sensor->gyro_offset_x = sum_x / 500.0f;
    sensor->gyro_offset_y = sum_y / 500.0f;
    sensor->gyro_offset_z = sum_z / 500.0f;

    return ESP_OK;
}