#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "encoder.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include <math.h>
#include "pico/stdlib.h"

#define SIG_SDA 0
#define SCL 1
#define circunference 0.037698 // en metros
#define radio 0.0925
#define PI 3.14159265358979323846
#define TCA_ADDR 0x70 
#define PWM_PIN1 10 // pin 14   SLICE 5 CH A
#define PWM_PIN2 11 // pin 15   SLICE 5 CH B
#define PWM_PIN3 12 // pin 16   SLICE 6 CH A
#define PWM_PIN4 13 // pin 17   SLICE 6 CH B

uint16_t basePWM = 780;

uint16_t angleMotor1 = 0;
uint16_t angleMotor2 = 0;
uint16_t angleMotor3 = 0;
uint16_t angleMotor4 = 0;

double distanceMotor1=0;
double distanceMotor2=0;
double distanceMotor3=0;
double distanceMotor4=0;

uint16_t offsetAngleMotor1 = 0;
uint16_t offsetAngleMotor2 = 0;
uint16_t offsetAngleMotor3 = 0;
uint16_t offsetAngleMotor4 = 0;

uint16_t turnMotor1 = 0;
uint16_t turnMotor2 = 0;
uint16_t turnMotor3 = 0;
uint16_t turnMotor4 = 0;

bool banTurnsMotor1 = 0;
bool banTurnsMotor2 = 0;
bool banTurnsMotor3 = 0;
bool banTurnsMotor4 = 0;

uint slice_num_5;
uint slice_num_6;

uint16_t offCC1 = 777;
uint16_t offCW1 = 723;
uint16_t offCC2 = 780;
uint16_t offCW2 = 720;
uint16_t offCC3 = 700;
uint16_t offCW3 = 800;
uint16_t offCC4 = 800;
uint16_t offCW4 = 700;
int16_t offset = -5;  // positivo si quiero que incremente velocidad

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
  tca_select_channel(3);
  angleMotor4 = angleSubtraction(getAngle(),offsetAngleMotor4);
  return true;
}
double turnsToDistance(uint16_t *turnMotor){
  return *turnMotor * circunference;
}

void distanceRobotCounterClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, double *distanceMotor){

  if (angleMotor >= 350 && *banTurnsMotor){
    (*turnMotor)++;
    *banTurnsMotor = false;
  }
  else if (angleMotor <= 20){
    *banTurnsMotor = true;
  }
  *distanceMotor = (*turnMotor)*circunference;
}

void distanceRobotClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, double *distanceMotor){

  if (angleMotor >= 350){
    *banTurnsMotor = true;
  }
  else if (angleMotor <= 20  && *banTurnsMotor ){
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
  tca_select_channel(3);
  offsetAngleMotor4 = getAngle();
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
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, offCC1 + offset); // 777
}
void motorClockWise1(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, offCW1 - offset); // 723
}
void motorCounterClockWise2(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, offCC2 + offset); // 780
}
void motorClockWise2(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, offCW2 - offset); // 720
}

void motorCounterClockWise3(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, offCC3 - offset); //700

}
void motorClockWise3(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, offCW3 + offset); // 800
}

void motorCounterClockWise4(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, offCC4 + offset); // 800
}

void motorClockWise4(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, offCW4 - offset); // 700
}

void motorStop(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 750);
}
void rotation(double rotationAngle){
  if(rotationAngle > 0){
    motorClockWise1();
    motorClockWise2();
    motorClockWise3();
    motorClockWise4();
    distanceRobotClockWise(angleMotor1,&turnMotor1,&banTurnsMotor1,&distanceMotor1);
    distanceRobotClockWise(angleMotor2,&turnMotor2,&banTurnsMotor2,&distanceMotor2);
    distanceRobotClockWise(angleMotor3,&turnMotor3,&banTurnsMotor3,&distanceMotor3);
    distanceRobotClockWise(angleMotor4,&turnMotor4,&banTurnsMotor4,&distanceMotor4);
    
    double angleMotors1 = distanceMotor1/radio;
    double angleMotors2 = distanceMotor2/radio;
    double angleMotors3 = distanceMotor3/radio;
    double angleMotors4 = distanceMotor4/radio;
    printf("%f\n",angleMotors2*180/PI);
    if(angleMotors2*180/PI >=rotationAngle || angleMotors3*180/PI >=rotationAngle ){
      motorStop();
      sleep_ms(10000);
    }
    // if(angleMotors2*180/PI >=rotationAngle){
    //   motorStop();
    //   sleep_ms(10000);
    // }
    // if(angleMotors3*180/PI >=rotationAngle){
    //   motorStop();
    //   sleep_ms(10000);
    // }
    // if(angleMotors4*180/PI >=rotationAngle){
    //   motorStop();
    //   sleep_ms(10000);
    // }

  }
}  

void moveForward(double distance){
  if (distance > 0){
    motorClockWise1();
    motorCounterClockWise2();
    motorClockWise3();
    motorCounterClockWise4();
    distanceRobotClockWise(angleMotor1,&turnMotor1,&banTurnsMotor1,&distanceMotor1);
    distanceRobotCounterClockWise(angleMotor2,&turnMotor2,&banTurnsMotor2,&distanceMotor2);
    distanceRobotClockWise(angleMotor3,&turnMotor3,&banTurnsMotor3,&distanceMotor3);
    distanceRobotCounterClockWise(angleMotor4,&turnMotor4,&banTurnsMotor4,&distanceMotor4);

    double posx1 = distanceMotor1*cos(45*PI/180);
    double posx2 = distanceMotor2*cos(45*PI/180);

    double finalPos = (posx1 + posx2)/2;
    printf("Final pos %f\n",finalPos);
    if (finalPos >= distance){
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
  //add_repeating_timer_us(2000,&timer_callback,NULL,&timer);

  while (1){
    tca_select_channel(0);
    angleMotor1 = angleSubtraction(getAngle(),offsetAngleMotor1);
    tca_select_channel(1);
    angleMotor2 = angleSubtraction(getAngle(),offsetAngleMotor2);
    tca_select_channel(2);
    angleMotor3 = angleSubtraction(getAngle(),offsetAngleMotor3);
    tca_select_channel(3);
    angleMotor4 = angleSubtraction(getAngle(),offsetAngleMotor4);
    moveForward(1);
    //pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 800);
    //moveForward(0.3); // distancia en m
    //motorClockWise3();
  }

  return 0;
}                       