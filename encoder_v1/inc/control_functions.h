/**
 * @file control_functions.h
 * @brief Cabecera para funciones de control.
 *
 * Este archivo de cabecera declara funciones y variables externas para el control de motores,
 * incluyendo ajustes de velocidad, cálculos de distancia y funciones de control de velocidad basadas en PID.
 * Las variables y funciones definidas aquí son utilizadas para gestionar el movimiento y la orientación
 * de un robot en diferentes modos de operación.
 */
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>

#ifndef CONTROL_FUNCTIONS_H
#define CONTROL_FUNCTIONS_H

/// Constantes para cálculos de movimiento.
#define circunference 0.037698 ///< Circunferencia en metros de 1/5 de giro de la rueda.
#define radio 0.094 ///< Radio en metros del centro del robot hacia el extremo de la rueda.

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

extern double integralError1_4;
extern double integralError2_3;
extern double previousError1_4;
extern double previousError2_3;

extern uint64_t prevTimeUsMotor1;
extern uint64_t prevTimeUsMotor2;
extern uint64_t prevTimeUsMotor3;
extern uint64_t prevTimeUsMotor4;

extern double windowTimeMotor1;
extern double windowTimeMotor2;
extern double windowTimeMotor3;
extern double windowTimeMotor4;

extern int velMotor1;
extern int velMotor2;
extern int velMotor3;
extern int velMotor4;

/// Declaraciones de funciones para el control de motores.
void distanceRobotCounterClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, double *distanceMotor,int *velMotor,double *windowTimeMotor,uint64_t *prevTimeUsMotor);
void distanceRobotClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, double *distanceMotor,int *velMotor,double *windowTimeMotor,uint64_t *prevTimeUsMotor);
void adjustMotorSpeed(uint motorNumber, double adjustment);
void m1ControlSpeed(int velRef, int turn);
void m2ControlSpeed(int velRef, int turn);
void m3ControlSpeed(int velRef, int turn);
void m4ControlSpeed(int velRef, int turn);
void restartControl();
#endif