#include "robot_movement.h"

uint16_t offCW1 = 720;
uint16_t offCW2 = 720;
uint16_t offCW3 = 780;
uint16_t offCW4 = 720;
uint16_t offCC1 = 780;
uint16_t offCC2 = 780;
uint16_t offCC3 = 720;
uint16_t offCC4 = 780;

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
bool banStop = false;



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
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, offCC4 + offset4+10); 
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
  offCW1 = 728;
  offCW2 = 728;
  offCW3 = 772;
  offCW4 = 728;
  if(rotationAngle > 0){
    motorsClockWise();
    distanceMotorsClockWise();
    dualMotorPDControlRotation();
    motorsClockWise();
    distanceMotorsClockWise();
    motorAngle1 = distanceMotor1/radio;
    motorAngle2 = distanceMotor2/radio;
    motorAngle3 = distanceMotor3/radio;
    motorAngle4 = distanceMotor4/radio;
    double motorAngle1_4 =(motorAngle1 + motorAngle4)/2;
    double motorAngle2_3 = (motorAngle2 + motorAngle3)/2;
    double angleError = motorAngle1_4 - motorAngle2_3;
    anglesPIDControlRotation(angleError);

    double angleFinal = (motorAngle1_4+motorAngle2_3)/2;

    printf(" angle %lf \n",angleFinal);
    if(angleFinal*180/PI >=rotationAngle-3){
      motorStop();
      restartControl();
      restartMovement();
      getOffsets();
      banStop=true;
    }
  

  }
}
void moveForward(double distance){
  offCW1 = 725;
  offCW2 = 705;
  offCW3 = 795;
  offCW4 = 725;
  if (distance > 0){    
    motorsForward();
    distanceMotorsForward();
    dualMotorPIDControl();
    motorsForward();
    distanceMotorsForward();

    double posx1 = (distanceMotor1+distanceMotor4)*cos(52*PI/180)/2;
    double posx2 = (distanceMotor2+distanceMotor3)*cos(52*PI/180)/2;
    double errorX1_X2 = posx1-posx2;
    // Controlador PID para ajustar la velocidad entre los pares de motores
    motorsPIControlPosition(errorX1_X2);
    
    double finalPos = (posx1 + posx2)/2;
    // printf("Final pos %f\n",finalPos);
    if (finalPos >= distance){
      motorStop();
      restartControl();
      restartMovement();
      getOffsets();
      banStop=true;

    }
  }
}

void restartMovement(){
  angleMotor1 = 0;
  angleMotor2 = 0;
  angleMotor3 = 0;
  angleMotor4 = 0;
  turnMotor1 = 0;
  turnMotor2 = 0;
  turnMotor3 = 0;
  turnMotor4 = 0;
  banTurnsMotor1 = 0;
  banTurnsMotor2 = 0;
  banTurnsMotor3 = 0;
  banTurnsMotor4 = 0;

}
