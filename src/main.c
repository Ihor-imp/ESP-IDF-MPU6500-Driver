#include <MPU6500.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define I2C_PORT I2C_NUM_0
#define I2C_CLK_SOURCE I2C_CLK_SRC_DEFAULT
#define SCL_IO_PIN 9
#define SDA_IO_PIN 8

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

esp_err_t i2c_init()
{
    i2c_master_bus_config_t bus_config =
        {
            .i2c_port = I2C_PORT,
            .clk_source = I2C_CLK_SOURCE,
            .scl_io_num = SCL_IO_PIN,
            .sda_io_num = SDA_IO_PIN,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true};

    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK)
    {
        return ret;
    }

    i2c_device_config_t dev_config =
        {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x68,
            .scl_speed_hz = 100000};

    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}

void app_main()
{
    float accel_x = 0;
    float accel_y = 0;
    float accel_z = 0;

    float gyro_x = 0;
    float gyro_y = 0;
    float gyro_z = 0;

    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI("TEST", "Program started");

    ESP_ERROR_CHECK(i2c_init());

    ESP_LOGI("TEST", "I2C initialized");

    mpu sensor =
        {
            .device = dev_handle};

    uint8_t id = 0;

    esp_err_t ret = mpu_who_i_am(&sensor, &id);
    if (ret == ESP_OK)
    {
        ESP_LOGI("MPU6500", "Device ID: 0x%02X", id);
    }
    else
    {
        ESP_LOGE("SENSOR", "MPU6500 read error: %s", esp_err_to_name(ret));
        return;
    }

    ret = mpu_init(&sensor);
    if (ret == ESP_OK)
    {
        ESP_LOGI("MPU6500", "Device is init");
    }
    else
    {
        ESP_LOGE("MPU6500", "Device init fail: %s", esp_err_to_name(ret));
        return;
    }

    ret = mpu_set_accel_range(&sensor, 2);
    if (ret == ESP_OK)
    {
        ESP_LOGI("MPU6500 ACCEL", "Device range is success");
    }
    else
    {
        ESP_LOGE("MPU6500 ACCEL", "Device range is fail %s", esp_err_to_name(ret));
        return;
    }

    ret = mpu_set_gyro_range(&sensor, 1);
    if (ret == ESP_OK)
    {
        ESP_LOGI("MPU6500 GYRO", "Device range is success");
    }
    else
    {
        ESP_LOGE("MPU6500 GYRO", "Device range is fail %s", esp_err_to_name(ret));
        return;
    }

    ret = mpu_calibrate_gyro(&sensor);

    if (ret == ESP_OK)
    {
        ESP_LOGI("MPU6500 GYRO", "Device calibrate is success");
    }
    else
    {
        ESP_LOGE("MPU6500 GYRO", "Device calibrate is fail %s", esp_err_to_name(ret));
        return;
    }

    ret = mpu_calibrate_accel(&sensor);

    if (ret == ESP_OK)
    {
        ESP_LOGI("MPU6500 ACCEL", "Device calibrate is success");
    }
    else
    {
        ESP_LOGE("MPU6500 ACCEL", "Device calibrate is fail %s", esp_err_to_name(ret));
        return;
    }

    while (1)
    {
        ret = mpu_read_accel(&sensor, &accel_x, &accel_y, &accel_z);
        if (ret == ESP_OK)
        {
            ESP_LOGI("ACCEL", "X: %.3f g | Y: %.3f g | Z: %.3f g", accel_x, accel_y, accel_z);
        }
        else
        {
            ESP_LOGE("ACCEL", "Device value is fail %s", esp_err_to_name(ret));
        }

        ret = mpu_read_gyro(&sensor, &gyro_x, &gyro_y, &gyro_z);
        if (ret == ESP_OK)
        {
            ESP_LOGI("GYRO", "X: %.3f °/s | Y: %.3f °/s | Z: %.3f °/s", gyro_x, gyro_y, gyro_z);
        }
        else
        {
            ESP_LOGE("GYRO", "Device value is fail %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}