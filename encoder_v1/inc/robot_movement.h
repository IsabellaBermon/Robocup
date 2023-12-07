/**
 * @file robot_movement.h
 * @brief Funciones y variable para el movimiento del robot.
 *
 * Este archivo de cabecera declara funciones y variables externas para la gestión del movimiento del robot.
 * Incluye funciones para controlar los movimientos básicos de los motores, realizar movimientos complejos como
 * rotación sobre le mismo eje, desplazamiento circular y avances en linea recta
 */
#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include "control_functions.h"
#include "motor_config.h"
#include "encoder.h"
#include "MPU6050_i2c.h"
#include <math.h>

#define PI 3.14159265358979323846

// Variables externas para modificar velocidad de motores
extern uint16_t offCC1;
extern uint16_t offCW1;
extern uint16_t offCC2;
extern uint16_t offCW2;
extern uint16_t offCC3;
extern uint16_t offCW3;
extern uint16_t offCC4;
extern uint16_t offCW4;

// Ángulos de los motores
extern uint16_t angleMotor1;
extern uint16_t angleMotor2;
extern uint16_t angleMotor3;
extern uint16_t angleMotor4;

// Ángulos de giro de los motores
extern uint16_t turnMotor1;
extern uint16_t turnMotor2;
extern uint16_t turnMotor3;
extern uint16_t turnMotor4;

// Banderas para el control de giros de los motores
extern bool banTurnsMotor1;
extern bool banTurnsMotor2;
extern bool banTurnsMotor3;
extern bool banTurnsMotor4;

// Bandera de parada y variables relacionadas con la posición angular y velocidad angular
extern bool banStop;
extern double angularPosition;
extern double prevAngularPosition;
extern double angularVelocity;
extern double robotAngle;
extern bool angularFlag;

// Offset del giroscopio y acelerómetro
extern int offsetZ;
extern int16_t acceleration[3], gyro[3], temp;
extern float offsetXa;
extern float offsetYa;

// Coeficientes del controlador PID
extern double Kp_pair;
extern double Ki_pair;
extern double Kd_pair;
extern double integralError_pair;
extern double previousError_pair;

/**
 * @brief Gira el motor 1 en sentido antihorario.
 */
void motorCounterClockWise1();
/**
 * @brief Gira el motor 2 en sentido antihorario.
 */
void motorCounterClockWise2();
/**
 * @brief Gira el motor 3 en sentido antihorario.
 */
void motorCounterClockWise3();
/**
 * @brief Gira el motor 4 en sentido antihorario.
 */
void motorCounterClockWise4();

/**
 * @brief Hace que el motor 1 gire en sentido horario.
 */
void motorClockWise1();
/**
 * @brief Hace que el motor 2 gire en sentido horario.
 */
void motorClockWise2();
/**
 * @brief Hace que el motor 3 gire en sentido horario.
 */
void motorClockWise3();
/**
 * @brief Hace que el motor 4 gire en sentido horario.
 */
void motorClockWise4();

/**
 * @brief Detiene todos los motores.
 */
void motorStop();

/**
 * @brief Hace que todos los motores avancen hacia adelante.
 */
void motorsForward();

/**
 * @brief Hace que todos los motores giren en sentido horario.
 */
void motorsClockWise();

/**
 * @brief Avanza los motores una distancia específica.
 * @param distance Distancia a avanzar.
 */
void distanceMotorsForward();

/**
 * @brief Gira los motores una distancia específica en sentido horario.
 * @param distance Distancia a girar.
 */
void distanceMotorsClockWise();

/**
 * @brief Rota el robot en un ángulo específico.
 * @param rotationAngle Ángulo de rotación en grados.
 */
void rotation(double rotationAngle);

/**
 * @brief Mueve el robot hacia adelante una distancia específica.
 * @param distance Distancia a avanzar.
 */
void moveForward(double distance);

/**
 * @brief Reinicia los parámetros de movimiento del robot.
 */
void restartMovement();

/**
 * @brief Realiza un movimiento circular con el robot.
 * @param r Radio del movimiento circular.
 * @param angle Ángulo del movimiento circular en grados.
 */
void circularMovement(double r, int angle);

/**
 * @brief Calibra los sensores y compensa los motores.
 */
void calibrate();

/**
 * @brief Actualiza la posición angular del robot.
 */
void updateAngle();

#endif