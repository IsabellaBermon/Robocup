/**
 * @file motor_config.c
 * @brief Configuración y control de los motores del robot mediante PWM.
 *
 * Este archivo proporciona funciones para inicializar y configurar la señal PWM
 * utilizada para controlar los motores del robot. Configura los pines GPIO para PWM,
 * establece la frecuencia y el período de la señal PWM, y activa los canales PWM para los motores.
 * Además, establece niveles iniciales y finales para los canales PWM.
 *
 * @note La configuración establece la frecuencia del reloj en 500kHz y un período de 50Hz.
 *       Los niveles iniciales y finales para los canales se establecen en valores predefinidos. Toda esta configuración
 *       en debido al funcionamiento que deben tene rlos motores brushless
 */
#include "motor_config.h"

uint slice_num_5;
uint slice_num_6;
/**
 * @brief Inicializa los motores del robot estableciendo la configuración PWM.
 *
 * Esta función configura los pines GPIO para la señal PWM, establece la frecuencia y el período
 * de la señal PWM y activa los canales PWM para los motores. Además, establece un nivel inicial
 * para los canales PWM y espera un breve período antes de ajustarlos a un nuevo nivel.
 *
 * @note La función configura dos rebanadas PWM (slice_num_5 y slice_num_6) para controlar
 *       cuatro canales de motor. Establece la frecuencia del reloj en 500kHz y un período de 50Hz.
 *       Los niveles iniciales y finales para los canales se establecen en valores predefinidos.
 */
void initMotor(){
    /// Configuración de los pines GPIO para PWM.
    gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN2, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN3, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN4, GPIO_FUNC_PWM);

  /// Configuración de los slice del PWM y activación de los canales para PWM_PIN1 y PWM_PIN3.
  slice_num_5 = pwm_gpio_to_slice_num(PWM_PIN1);  ///< Encuentra el slice PWM para GPIO.
  pwm_set_clkdiv(slice_num_5, 250.0f);            ///< Establece la frecuencia del reloj a 500kHz.
  pwm_set_wrap(slice_num_5, 10000);               ///< Establece el período a 50Hz (20 ms).
  pwm_set_enabled(slice_num_5, true);             ///< Activa el slice del PWM.

  slice_num_6 = pwm_gpio_to_slice_num(PWM_PIN3); ///< Encuentra la slice PWM para GPIO.
  pwm_set_clkdiv(slice_num_6, 250.0f); ///< Establece la frecuencia del reloj a 500kHz.
  pwm_set_wrap(slice_num_6, 10000);    ///< Establece el período a 50Hz (20 ms).
  pwm_set_enabled(slice_num_6, true);  ///< Activa la slice PWM.

  /// Configuración inicial de los niveles PWM para los motores.
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 650); 
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 650); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 650); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 650); 
  sleep_ms(3000); ///< Espera antes de ajustar los niveles PWM.
  /// Ajuste de los niveles PWM después de la espera.
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 750);  
  sleep_ms(3000); ///< Espera adicional después del ajuste.
}