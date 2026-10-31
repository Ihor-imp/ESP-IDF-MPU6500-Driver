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
        ESP_LOGI("MPU6500", "Device range is success");
    }
    else{
        ESP_LOGE("MPU6500", "Device range is fail %s", esp_err_to_name(ret));
    }
}