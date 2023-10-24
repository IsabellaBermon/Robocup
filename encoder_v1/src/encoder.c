#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "encoder.h"
// Dirección del registro de reinicio (Reset)
#define AS5600_RESET_REG 0x0E

// Valor para restablecer el encoder AS5600
#define AS5600_RESET_VALUE 0x0010

void resetAS5600() {
    // Escribe el valor de reinicio en el registro de reinicio
    uint8_t resetData[2] = {AS5600_RESET_REG, AS5600_RESET_VALUE};
    i2c_write_blocking(AS560_i2c, AS560_ADDR, resetData, sizeof(resetData), false);
}
void readAngle(int16_t *buffer) {
    uint16_t addr = 0x0E;
    i2c_write_blocking(AS560_i2c, AS560_ADDR, &addr, sizeof(addr), true);
    i2c_read_blocking(AS560_i2c, AS560_ADDR, buffer, sizeof(buffer), false);
}
void readAngleRaw(int16_t *buffer) {
    uint16_t addr = 0x0C; 
    i2c_write_timeout_us(AS560_i2c, AS560_ADDR, &addr, sizeof(addr),false,1000);
    i2c_read_timeout_us(AS560_i2c, AS560_ADDR, buffer, 2, false,1000);
}


void getStatus(int16_t *buffer){
    uint16_t addr = 0x0B;
    i2c_write_blocking(AS560_i2c, AS560_ADDR, &addr, sizeof(addr), true);
    i2c_read_blocking(AS560_i2c, AS560_ADDR, buffer, sizeof(buffer), false);
}