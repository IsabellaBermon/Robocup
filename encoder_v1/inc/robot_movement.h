#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include "control_functions.h"
#include "motor_config.h"
#include "encoder.h"
#include <math.h>

#define PI 3.14159265358979323846

extern uint16_t offCC1;
extern uint16_t offCW1;
extern uint16_t offCC2;
extern uint16_t offCW2;
extern uint16_t offCC3;
extern uint16_t offCW3;
extern uint16_t offCC4;
extern uint16_t offCW4;

extern uint16_t angleMotor1;
extern uint16_t angleMotor2;
extern uint16_t angleMotor3;
extern uint16_t angleMotor4;


extern uint16_t turnMotor1;
extern uint16_t turnMotor2;
extern uint16_t turnMotor3;
extern uint16_t turnMotor4;

extern bool banTurnsMotor1 ;
extern bool banTurnsMotor2 ;
extern bool banTurnsMotor3 ;
extern bool banTurnsMotor4 ;

// Coeficientes del controlador PID



extern double Kp_pair;
extern double Ki_pair;
extern double Kd_pair;
extern double integralError_pair;
extern double previousError_pair;
void motorCounterClockWise1();
void motorCounterClockWise2();
void motorCounterClockWise3();
void motorCounterClockWise4();
void motorClockWise1();
void motorClockWise2();
void motorClockWise3();
void motorClockWise4();
void motorStop();
void motorsForward();
void motorsClockWise();
void distanceMotorsForward();
void distanceMotorsClockWise();
void rotation(double rotationAngle);
void moveForward(double distance);
void restartMovement();
#endif
