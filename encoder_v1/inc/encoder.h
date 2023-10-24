#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define AS560_ADDR 0x36
#define AS560_i2c i2c0

void resetAS5600();
void readAngle(int16_t *buffer);
void getStatus(int16_t *buffer);
void readAngleRaw(int16_t *buffer);