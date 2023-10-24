#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include "encoder/encoder.h"
#include <math.h>
#define SIG_SDA 0
#define SCL 1
#define circunference 0.037698
#define radio 0.0925
#define PI 3.14159265358979323846
#define TCA_ADDR 0x70 
#define PWM_PIN1 10 // pin 14   SLICE 5 CH A
#define PWM_PIN2 11 // pin 15   SLICE 5 CH b
#define PWM_PIN3 12 // pin 16   SLICE 5 CH b
#define PWM_PIN4 13 // pin 17   SLICE 5 CH b

uint16_t basePWM = 780;

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
uint slice_num_5;
uint slice_num_6;


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
void distanceRobotCounterClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, float *distanceMotor){

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
void distanceRobotClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, float *distanceMotor){

  if (angleMotor >= 350)
  {
    *banTurnsMotor = true;
  }
  else if (angleMotor <= 20  && *banTurnsMotor )
  {
    (*turnMotor)++;

    *banTurnsMotor = false;
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

void initMotor(){
  gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
  gpio_set_function(PWM_PIN2, GPIO_FUNC_PWM);
  gpio_set_function(PWM_PIN3, GPIO_FUNC_PWM);
  gpio_set_function(PWM_PIN4, GPIO_FUNC_PWM);


  slice_num_5 = pwm_gpio_to_slice_num(PWM_PIN1); // Find out which PWM slice is connected to GPIO
  pwm_set_clkdiv(slice_num_5, 250.0f); // Set clock freq at 500kHz
  pwm_set_wrap(slice_num_5, 10000);    // Set period of 50Hz (20 ms)
  pwm_set_enabled(slice_num_5, true); // Set the PWM running

  slice_num_6 = pwm_gpio_to_slice_num (PWM_PIN3); // Find out which PWM slice is connected to GPIO 
  pwm_set_clkdiv (slice_num_6, 250.0f); // Set clock freq at 500kHz
  pwm_set_wrap(slice_num_6, 10000); // Set period of 50Hz (20 ms)
  pwm_set_enabled(slice_num_6, true); // Set the PWM running  


  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 650); 
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 650); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 650); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 650); 
  sleep_ms(3000);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 750);  
  sleep_ms(3000);

}



void motorCounterClockWise1(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 780);
}
void motorClockWise1(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 720);
}
void motorCounterClockWise2(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 780);

}
void motorClockWise2(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 660);
}

void motorCounterClockWise3(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 700);

}
void motorClockWise3(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 840);

}

void motorCounterClockWise4(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 780);

}
void motorClockWise4(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 720);

}

void motorStop(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 750);
}
void rotation(float rotationAngle){
  if(rotationAngle > 0){
    motorClockWise2();
    motorClockWise3();
    distanceRobotClockWise(angleMotor2,&turnMotor2,&banTurnsMotor2,&distanceMotor2);
    distanceRobotClockWise(angleMotor3,&turnMotor3,&banTurnsMotor3,&distanceMotor3);
    
    float angleMotors2 = distanceMotor2/radio;
    float angleMotors3 = distanceMotor3/radio;
    if(angleMotors2*180/PI >=rotationAngle){
      motorStop();
      sleep_ms(10000);
    }
    if(angleMotors3*180/PI >=rotationAngle){
      motorStop();
      sleep_ms(10000);

    }

  }
  
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
  initMotor();
  add_repeating_timer_us(2000,&timer_callback,NULL,&timer);

  while (1){
    //rotation(90);
    pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 840);

  }
  return 0;
}                                                         