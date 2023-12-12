#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

class MotorControl {
public:
    void kick(uint duty_cycle);
    void dribble(uint duty_cycle);
    void stop();
};

#endif // MOTOR_CONTROL_H
