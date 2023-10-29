#ifndef CONTROL_FUNCTIONS_H
#define CONTROL_FUNCTIONS_H

#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#define circunference 0.037698 // en metros
#define radio 0.0925

extern double distanceMotor1;
extern double distanceMotor2;
extern double distanceMotor3;
extern double distanceMotor4;

extern double motorAngle1;
extern double motorAngle2;
extern double motorAngle3;
extern double motorAngle4;

extern int16_t offset1;  
extern int16_t offset2;  
extern int16_t offset3;  
extern int16_t offset4;  

extern double Kp_d;
extern double Ki_d;
extern double Kd_d;
extern double Kp; // Coeficiente proporcional
extern double Ki; // Coeficiente integral
extern double Kd;// Coeficiente derivativo

extern double integralError1_4;
extern double integralError2_3;
extern double previousError1_4;
extern double previousError2_3;

extern double integralError_pair ;
extern double previousError_pair ;
extern double Kp_pair;
extern double Ki_pair;
extern double Kd_pair;

void distanceRobotCounterClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, double *distanceMotor);
void distanceRobotClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, double *distanceMotor);
void adjustMotorSpeed(uint motorNumber, double adjustment);
void dualMotorPIDControl();
void dualMotorPDControlRotation();
void motorsPIControlPosition(double error);
void anglesPIDControlRotation(double errorAngle);
void restartControl();
#endif