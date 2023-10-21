#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "encoder.h"

void readAngle(int16_t *buffer) {
    uint16_t addr = 0x0E;
    i2c_write_blocking(AS560_i2c0, AS560_ADDR, &addr, sizeof(addr), true);
    i2c_read_blocking(AS560_i2c0, AS560_ADDR, buffer, sizeof(buffer), false);
}

void readAngleRaw0(int16_t *buffer) {
    uint16_t addr = 0x0C;
    //i2c_write_blocking(AS560_i2c0, AS560_ADDR, &addr, sizeof(addr), true);
    //i2c_read_blocking(AS560_i2c0, AS560_ADDR, buffer, sizeof(buffer), false);
    i2c_write_timeout_us(AS560_i2c0, AS560_ADDR, &addr, sizeof(addr), false, 2000);
    i2c_read_timeout_us(AS560_i2c0, AS560_ADDR, buffer, sizeof(buffer), false, 2000);
}

void readAngleRaw1(int16_t *buffer) {
    uint16_t addr = 0x0C;
    //i2c_write_blocking(AS560_i2c1, AS560_ADDR, &addr, sizeof(addr), true);
    //i2c_read_blocking(AS560_i2c1, AS560_ADDR, buffer, sizeof(buffer), false);
    i2c_write_timeout_us(AS560_i2c0, AS560_ADDR, &addr, sizeof(addr), false, 2000); //us
    i2c_read_timeout_us(AS560_i2c0, AS560_ADDR, buffer, sizeof(buffer), false, 2000);
}

void getStatus(int16_t *buffer){
    uint16_t addr = 0x0B;
    i2c_write_blocking(AS560_i2c0, AS560_ADDR, &addr, sizeof(addr), true);
    i2c_read_blocking(AS560_i2c0, AS560_ADDR, buffer, sizeof(buffer), false);
}
