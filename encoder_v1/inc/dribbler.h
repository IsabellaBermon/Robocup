#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#define PWMKICK 200
#define PWMDRIBBLE 200
void initMotorControl();
void kick();
void dribble();
void stop();

#endif // MOTOR_CONTROL_H