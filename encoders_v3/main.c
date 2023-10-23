#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include "encoder/encoder.h"

#define SIG_SDA 0
#define SCL 1
#define circunference 0.037698
#define TCA_ADDR 0x70 

uint16_t angleMotor1 = 0;
uint16_t angleMotor2 = 0;
uint16_t angleMotor3 = 0;

float distanceMotor1=0;
float distanceMotor2=0;
float distanceMotor3=0;

uint16_t offsetAngleMotor1 = 0;
uint16_t offsetAngleMotor2 = 0;
uint16_t offsetAngleMotor3 = 0;

uint16_t turnMotor1 = 0;
uint16_t turnMotor2 = 0;
uint16_t turnMotor3 = 0;
bool banTurnsMotor1 = 0;
bool banTurnsMotor2 = 0;
bool banTurnsMotor3 = 0;

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
bool timer_callback(repeating_timer_t *t){
  tca_select_channel(0);
  angleMotor1 = angleSubtraction(getAngle(),offsetAngleMotor1);
  tca_select_channel(1);
  angleMotor2 = angleSubtraction(getAngle(),offsetAngleMotor2);
  tca_select_channel(2);
  angleMotor3 = angleSubtraction(getAngle(),offsetAngleMotor3);
  return true;
}
float turnsToDistance(uint16_t *turnMotor)
{
  return *turnMotor * circunference;
}
void distanceRobotForward(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, float *distanceMotor){

  if (angleMotor >= 350 && *banTurnsMotor)
  {
    (*turnMotor)++;
    *banTurnsMotor = false;
  }
  else if (angleMotor <= 20)
  {
    *banTurnsMotor = true;
  }
  *distanceMotor = (*turnMotor)*circunference;

}
void getOffsets(){
  tca_select_channel(0);
  offsetAngleMotor1 = getAngle();
  tca_select_channel(1);
  offsetAngleMotor2 = getAngle();
  tca_select_channel(2);
  offsetAngleMotor3 = getAngle();  
}

int main(){
  stdio_init_all();

  static repeating_timer_t timer;

  i2c_init(AS560_i2c,400000);
  gpio_set_function(SIG_SDA,GPIO_FUNC_I2C);
  gpio_set_function(SCL,GPIO_FUNC_I2C);
  gpio_pull_up(SIG_SDA);
  gpio_pull_up(SCL);

  getOffsets();

  add_repeating_timer_us(2000,&timer_callback,NULL,&timer);



  while (1){
    distanceRobotForward(angleMotor1,&turnMotor1,&banTurnsMotor1,&distanceMotor1);
    distanceRobotForward(angleMotor2,&turnMotor2,&banTurnsMotor2,&distanceMotor2);
    distanceRobotForward(angleMotor3,&turnMotor3,&banTurnsMotor3,&distanceMotor3);
    printf("Motor 1 %f ",distanceMotor1);
    printf("Motor 2 %f ",distanceMotor2);
    printf("Motor 3 %f\n",distanceMotor3);

  }
  return 0;
}                                                         