#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "MPU6050_i2c.h"


// By default these devices  are on bus address 0x68

void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(MPU6050_i2c, MPU6050_addr, buf, sizeof(buf), true);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(MPU6050_i2c, MPU6050_addr, &val, sizeof(val), true); // true to keep master control of bus
    i2c_read_blocking(MPU6050_i2c, MPU6050_addr, buffer, sizeof(buffer), false);

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];

    val = 0x43;
    i2c_write_blocking(MPU6050_i2c, MPU6050_addr, &val, sizeof(val), true);
    i2c_read_blocking(MPU6050_i2c, MPU6050_addr, buffer, sizeof(buffer), false);  // False - finished with bus

    gyro[0] = (buffer[0] << 8) | buffer[1];
    gyro[1] = (buffer[2] << 8) | buffer[3];
    gyro[2] = (buffer[4] << 8) | buffer[5];


    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(MPU6050_i2c, MPU6050_addr, &val, sizeof(val), true);
    i2c_read_blocking(MPU6050_i2c, MPU6050_addr, buffer, sizeof(buffer), false);  // False - finished with bus

    *temp = (buffer[0] << 8) | buffer[1];
}


