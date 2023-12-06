#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define AS560_ADDR 0x36
#define AS560_i2c i2c0
#define TCA_ADDR 0x70 


#define SIG_SDA 0
#define SCL 1


extern uint16_t offsetAngleMotor1;
extern uint16_t offsetAngleMotor2;
extern uint16_t offsetAngleMotor3;
extern uint16_t offsetAngleMotor4;

void initI2C();
void readAngleRaw(int16_t *buffer);
void tca_select_channel(uint8_t channel);
int16_t getAngle();
int16_t angleSubtraction(int16_t angle, int16_t angleOffset);
void getOffsets();
