#ifndef MPU6500_H
#define MPU6500_H
#include "driver/i2c_master.h"

typedef struct 
{
    i2c_master_dev_handle_t device;
}mpu;


esp_err_t mpu_init(mpu *sensor);
esp_err_t mpu_who_i_am(mpu *sensor, uint8_t *indef);
esp_err_t mpu_read_accel(mpu *sensor, float *x, float *y, float *z);
esp_err_t mpu_read_gyro(mpu *sensor, float *x, float *y, float *z);
esp_err_t mpu_set_accel_range(mpu *sensor, uint8_t range);
esp_err_t mpu_set_gyro_range(mpu *sensor, uint32_t range);

#endif