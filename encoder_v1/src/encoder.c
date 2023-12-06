#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "encoder.h"
// Dirección del registro de reinicio (Reset)
#define AS5600_RESET_REG 0x0E

// Valor para restablecer el encoder AS5600
#define AS5600_RESET_VALUE 0x0010


uint16_t offsetAngleMotor1 = 0;
uint16_t offsetAngleMotor2 = 0;
uint16_t offsetAngleMotor3 = 0;
uint16_t offsetAngleMotor4 = 0;

void initI2C(){
  i2c_init(AS560_i2c,400000);
  gpio_set_function(SIG_SDA,GPIO_FUNC_I2C);
  gpio_set_function(SCL,GPIO_FUNC_I2C);
  gpio_pull_up(SIG_SDA);
  gpio_pull_up(SCL);
}

void readAngleRaw(int16_t *buffer) {
    uint16_t addr = 0x0C; 
    i2c_write_timeout_us(AS560_i2c, AS560_ADDR, &addr, sizeof(addr),false,1000);
    i2c_read_timeout_us(AS560_i2c, AS560_ADDR, buffer, 2, false,1000);
}



void tca_select_channel(uint8_t channel){
    uint8_t data = 1 << channel;
    i2c_write_blocking(i2c0, TCA_ADDR, &data, 1, false);
}


int16_t getAngle()
{
  int16_t raw;
  readAngleRaw(&raw);
  return raw * 360 / 0xFFF;
}

int16_t angleSubtraction(int16_t angle, int16_t angleOffset)
{
  int16_t angleSub = angle - angleOffset;
  if (angleSub < 0)
  {
    return 360 + angleSub;
  }
  else
  {
    return angleSub;
  }
}

void getOffsets(){
  tca_select_channel(0);
  offsetAngleMotor1 = getAngle();
  tca_select_channel(1);
  offsetAngleMotor2 = getAngle();
  tca_select_channel(2);
  offsetAngleMotor3 = getAngle(); 
  tca_select_channel(3);
  offsetAngleMotor4 = getAngle();
}
