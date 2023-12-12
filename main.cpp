#include "dribbler.h"

const uint MOTOR_PIN1 = 0; // GPIO 0 para control de dirección
const uint MOTOR_PIN2 = 1; // GPIO 1 para control de dirección
const uint PWM_PIN = 2;    // GPIO 2 para control de velocidad con PWM

void MotorControl::kick(uint duty_cycle) {
    gpio_put(MOTOR_PIN1, 1);
    gpio_put(MOTOR_PIN2, 0);
    pwm_set_gpio_level(PWM_PIN, duty_cycle);
}

void MotorControl::dribble(uint duty_cycle) {
    gpio_put(MOTOR_PIN1, 0);
    gpio_put(MOTOR_PIN2, 1);
    pwm_set_gpio_level(PWM_PIN, duty_cycle);
}

void MotorControl::stop() {
    gpio_put(MOTOR_PIN1, 0);
    gpio_put(MOTOR_PIN2, 0);
    pwm_set_gpio_level(PWM_PIN, 0);
}


int main() {
    stdio_init_all();

    gpio_init(MOTOR_PIN1);
    gpio_set_dir(MOTOR_PIN1, GPIO_OUT);

    gpio_init(MOTOR_PIN2);
    gpio_set_dir(MOTOR_PIN2, GPIO_OUT);

    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_wrap(slice_num, 255); // Configurar el rango de PWM
    pwm_set_enabled(slice_num, true);

    MotorControl motor;

    while (true) {
        motor.kick(200); // 50% de ciclo de trabajo
        sleep_ms(2000);
        motor.stop();
        sleep_ms(2000);
        motor.dribble(200); // 50% de ciclo de trabajo
        sleep_ms(2000);
        //motor.stop();
        //sleep_ms(1000);
    }

    return 0;
}
