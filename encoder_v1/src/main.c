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

uint16_t offCC1 = 770;
uint16_t offCW1 = 730;
uint16_t offCC2 = 770;
uint16_t offCW2 = 730;
uint16_t offCC3 = 730;
uint16_t offCW3 = 770;
uint16_t offCC4 = 770;
uint16_t offCW4 = 730;
int16_t offset1 = 0;  // positivo si quiero que incremente velocidad
int16_t offset2 = 0;  // positivo si quiero que incremente velocidad
int16_t offset3 = 0;  // positivo si quiero que incremente velocidad
int16_t offset4 = 0;  // positivo si quiero que incremente velocidad

// Coeficientes del controlador PID
double Kp = 0.1; // Coeficiente proporcional
double Ki = 0.05; // Coeficiente integral
double Kd = 0.01; // Coeficiente derivativo

// Variables para almacenar errores acumulados y error anterior
double integralError1_4 = 0;
double integralError2_3 = 0;
double previousError1_4 = 0;
double previousError2_3 = 0;


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
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, offCC1 + offset1); // 777
}
void motorClockWise1(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, offCW1 + offset1); // 723
}
void motorCounterClockWise2(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, offCC2 + offset2); // 780
}
void motorClockWise2(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, offCW2 + offset2); // 720
}

void motorCounterClockWise3(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, offCC3 + offset3); //700

}
void motorClockWise3(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, offCW3 + offset3); // 800
}

void motorCounterClockWise4(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, offCC4 + offset4); // 800
}

void motorClockWise4(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, offCW4 + offset4); // 700
}

void motorStop(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 750);
}

void adjustMotorSpeed(uint motorNumber, double adjustment) {
 switch (motorNumber){
 case 1:
  offset1+=adjustment;
  break;
 case 2:
  offset2+=adjustment;
  break;
 case 3:
  offset3+=adjustment;
  break;
 case 4:
  offset4+=adjustment;
  break; 
 default:
  break;
 }
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
    double error1_4 = angleMotors1 - angleMotors4;
    double error2_3 = angleMotors2 - angleMotors3;

    double pidAdjustment1_4 = Kp * error1_4 + Kd * (error1_4 - previousError1_4);
    double pidAdjustment2_3 = Kp * error2_3 + Kd * (error2_3 - previousError2_3);
    
    previousError1_4 = error1_4;
    previousError2_3 = error2_3;
    if(error1_4 > 0){
      adjustMotorSpeed(1, pidAdjustment1_4);  // Reducir la velocidad del motor 1 si el error es positivo
    }else{

      adjustMotorSpeed(4, pidAdjustment1_4);   // Aumentar la velocidad del motor 4 si el error es positivo
    }
    if(error2_3 > 0){

    adjustMotorSpeed(2, pidAdjustment2_3);  // Reducir la velocidad del motor 2 si el error es positivo
    }else{

    adjustMotorSpeed(3, -pidAdjustment2_3);   // Aumentar la velocidad del motor 3 si el error es positivo
    }


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

    double error1_4 = distanceMotor1 - distanceMotor4;
    double error2_3 = distanceMotor2 - distanceMotor3;

    double pidAdjustment1_4 = Kp * error1_4 + Kd * (error1_4 - previousError1_4);
    double pidAdjustment2_3 = Kp * error2_3 + Kd * (error2_3 - previousError2_3);
    printf("adj1 %lf ",pidAdjustment1_4);
    printf("error1_4 %lf ",error1_4);
    printf("error2_3 %lf ",error2_3);
    printf("adj2 %lf\n",pidAdjustment2_3);


    previousError1_4 = error1_4;
    previousError2_3 = error2_3;
    if(error1_4 > 0){
      adjustMotorSpeed(1, -pidAdjustment1_4);  // Reducir la velocidad del motor 1 si el error es positivo
    }else{

      adjustMotorSpeed(4, -pidAdjustment1_4);   // Aumentar la velocidad del motor 4 si el error es positivo
    }
    if(error2_3 > 0){

    adjustMotorSpeed(2, -pidAdjustment2_3);  // Reducir la velocidad del motor 2 si el error es positivo
    }else{

    adjustMotorSpeed(3, pidAdjustment2_3);   // Aumentar la velocidad del motor 3 si el error es positivo
    }

    


    double posx1 = (distanceMotor1+distanceMotor4)*cos(45*PI/180)/2;
    double posx2 = (distanceMotor2+distanceMotor3)*cos(45*PI/180)/2;

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
  //add_repeating_timer_us(200,&timer_callback,NULL,&timer);

  while (1){
    tca_select_channel(0);
    angleMotor1 = angleSubtraction(getAngle(),offsetAngleMotor1);
    tca_select_channel(1);
    angleMotor2 = angleSubtraction(getAngle(),offsetAngleMotor2);
    tca_select_channel(2);
    angleMotor3 = angleSubtraction(getAngle(),offsetAngleMotor3);
    tca_select_channel(3);
    angleMotor4 = angleSubtraction(getAngle(),offsetAngleMotor4);
    rotation(360);
    //moveForward(1);
    //pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 800);
    //moveForward(0.3); // distancia en m
    //motorClockWise3();
  }

  return 0;
}                       