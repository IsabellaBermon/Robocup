/**
 * @file motor_config.h
 * @brief Configuración y funciones para el control de motores.
 *
 * Este archivo de cabecera proporciona la configuración de pines y funciones necesarias
 * para el control de motores utilizando la Raspberry Pi Pico.
 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define PWM_PIN1 10 // pin 14   SLICE 5 CH A
#define PWM_PIN2 11 // pin 15   SLICE 5 CH B
#define PWM_PIN3 12 // pin 16   SLICE 6 CH A
#define PWM_PIN4 13 // pin 17   SLICE 6 CH B

extern uint slice_num_5; ///< Número de la slice para PWM relacionada con los pines 10 y 11.
extern uint slice_num_6; ///< Número de la slice para PWM relacionada con los pines 12 y 13.

void initMotor();

#endif