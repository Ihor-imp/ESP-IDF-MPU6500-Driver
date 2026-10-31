#ifndef MPU6500_H
#define MPU6500_H
#include "driver/i2c_master.h"

typedef struct
{
    i2c_master_dev_handle_t device;
    uint8_t accel_range;
    uint8_t gyro_range;

    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;

    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;

    float accel_scale_x;
    float accel_scale_y;
    float accel_scale_z;
} mpu;

esp_err_t mpu_init(mpu *sensor);
esp_err_t mpu_who_i_am(mpu *sensor, uint8_t *indef);

esp_err_t mpu_read_accel(mpu *sensor, float *x, float *y, float *z);
esp_err_t mpu_read_gyro(mpu *sensor, float *x, float *y, float *z);

esp_err_t mpu_set_accel_range(mpu *sensor, uint8_t range);
esp_err_t mpu_set_gyro_range(mpu *sensor, uint8_t range);

esp_err_t mpu_calibrate_accel(mpu *sensor);
esp_err_t mpu_calibrate_gyro(mpu *sensor);

#endif