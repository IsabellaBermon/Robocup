
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#define MPU6050_i2c i2c1
#define MPU6050_addr 0x68
#define WINDOW_SIZE 10
#define SDA_MPU 18
#define SCL_MPU 19
extern int gyro_z_readings[WINDOW_SIZE];
extern int index_media;
extern int sum;
void mpu_init();
void mpu6050_reset();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]);
int filter_median_moving(int new_reading);
void resetFilter();