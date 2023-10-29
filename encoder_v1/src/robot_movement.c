#include "robot_movement.h"

uint16_t offCC1 = 780;
uint16_t offCW1 = 720;
uint16_t offCC2 = 780;
uint16_t offCW2 = 720;
uint16_t offCC3 = 720;
uint16_t offCW3 = 780;
uint16_t offCC4 = 780;
uint16_t offCW4 = 720;

uint16_t angleMotor1 = 0;
uint16_t angleMotor2 = 0;
uint16_t angleMotor3 = 0;
uint16_t angleMotor4 = 0;


uint16_t turnMotor1 = 0;
uint16_t turnMotor2 = 0;
uint16_t turnMotor3 = 0;
uint16_t turnMotor4 = 0;

bool banTurnsMotor1 = 0;
bool banTurnsMotor2 = 0;
bool banTurnsMotor3 = 0;
bool banTurnsMotor4 = 0;


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
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, offCW3 + offset3); // 780
}
void motorCounterClockWise4(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, offCC4 + offset4+10); // 780
}
void motorClockWise4(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, offCW4 + offset4); // 720
}
void motorStop(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 750);
}
void motorsForward(){
  motorClockWise1();
  motorCounterClockWise2();
  motorClockWise3();
  motorCounterClockWise4();
}
void motorsClockWise(){
    motorClockWise1();
    motorClockWise2();
    motorClockWise3();
    motorClockWise4();
}
void distanceMotorsForward(){
  distanceRobotClockWise(angleMotor1,&turnMotor1,&banTurnsMotor1,&distanceMotor1);
  distanceRobotCounterClockWise(angleMotor2,&turnMotor2,&banTurnsMotor2,&distanceMotor2);
  distanceRobotClockWise(angleMotor3,&turnMotor3,&banTurnsMotor3,&distanceMotor3);
  distanceRobotCounterClockWise(angleMotor4,&turnMotor4,&banTurnsMotor4,&distanceMotor4);
}
void distanceMotorsClockWise(){
    distanceRobotClockWise(angleMotor1,&turnMotor1,&banTurnsMotor1,&distanceMotor1);
    distanceRobotClockWise(angleMotor2,&turnMotor2,&banTurnsMotor2,&distanceMotor2);
    distanceRobotClockWise(angleMotor3,&turnMotor3,&banTurnsMotor3,&distanceMotor3);
    distanceRobotClockWise(angleMotor4,&turnMotor4,&banTurnsMotor4,&distanceMotor4);
}
void rotation(double rotationAngle){
  if(rotationAngle > 0){
    motorsClockWise();
    distanceMotorsClockWise();
    
    dualMotorPDControlRotation();
    motorAngle1 = distanceMotor1/radio;
    motorAngle2 = distanceMotor2/radio;
    motorAngle3 = distanceMotor3/radio;
    motorAngle4 = distanceMotor4/radio;
    double motorAngle1_4 =(motorAngle1 + motorAngle4)/2;
    double motorAngle2_3 = (motorAngle2 + motorAngle3)/2;
    double angleError = motorAngle1_4 - motorAngle2_3;
    anglesPControlRotation(angleError);

    double angleFinal = (motorAngle1_4+motorAngle2_3)/2;

    printf("angle %lf \n",angleFinal);
    if(angleFinal*180/PI >=rotationAngle){
      motorStop();
      sleep_ms(10000);
    }
  

  }
}
void moveForward(double distance){
  if (distance > 0){
    motorsForward();
    distanceMotorsForward();
    dualMotorPIDControl();
    double posx1 = (distanceMotor1+distanceMotor4)*cos(45*PI/180)/2;
    double posx2 = (distanceMotor2+distanceMotor3)*cos(45*PI/180)/2;
    double errorX1_X2 = posx1-posx2;
    // Controlador PID para ajustar la velocidad entre los pares de motores
    motorsPIControlPosition(errorX1_X2);
    double finalPos = (posx1 + posx2)/2;
    // printf("Final pos %f\n",finalPos);
    if (finalPos >= distance){
      motorStop();
      sleep_ms(10000);
    }
  }
}

