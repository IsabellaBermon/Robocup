/**
 * @file encoder.c
 * @brief Control de encoders AS5600 a través de I2C para un robot.
 *
 * Este archivo contiene funciones para inicializar la comunicación I2C con los encoders AS5600,
 * leer el ángulo crudo de los encoders, seleccionar canales en un multiplexor TCA9548A,
 * obtener y convertir el ángulo crudo a grados, restar un offset de un ángulo y obtener offsets
 * de ángulo para cada motor.
 */

#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "encoder.h"
// Dirección del registro de reinicio (Reset)
#define AS5600_RESET_REG 0x0E

// Valor para restablecer el encoder AS5600
#define AS5600_RESET_VALUE 0x0010

/// Offsets de ángulo para cada motor.
uint16_t offsetAngleMotor1 = 0;
uint16_t offsetAngleMotor2 = 0;
uint16_t offsetAngleMotor3 = 0;
uint16_t offsetAngleMotor4 = 0;
/**
 * @brief Inicializa la interfaz I2C para la comunicación con los encoders AS5600.
 *
 * Configura los pines para la comunicación I2C y establece las resistencias de pull-up necesarias.
 */
void initI2C(){
  i2c_init(AS560_i2c,400000);
  gpio_set_function(SIG_SDA,GPIO_FUNC_I2C);
  gpio_set_function(SCL,GPIO_FUNC_I2C);
  gpio_pull_up(SIG_SDA);
  gpio_pull_up(SCL);
}
/**
 * @brief Lee el ángulo crudo del encoder AS5600.
 *
 * Solicita y lee el ángulo crudo del encoder AS5600 a través de la interfaz I2C.
 *
 * @param buffer Puntero donde se almacenará el ángulo leído.
 */
void readAngleRaw(int16_t *buffer) {
    uint16_t addr = 0x0C; 
    i2c_write_timeout_us(AS560_i2c, AS560_ADDR, &addr, sizeof(addr),false,1000);
    i2c_read_timeout_us(AS560_i2c, AS560_ADDR, buffer, 2, false,1000);
}


/**
 * @brief Selecciona un canal específico en el multiplexor TCA.
 *
 * Cambia el canal activo en un multiplexor TCA para permitir la lectura de múltiples encoders AS5600.
 *
 * @param channel Número de canal a seleccionar en el multiplexor.
 */
void tca_select_channel(uint8_t channel){
    uint8_t data = 1 << channel;
    i2c_write_blocking(i2c0, TCA_ADDR, &data, 1, false);
}

/**
 * @brief Obtiene y convierte el ángulo crudo del encoder AS5600 a grados.
 *
 * Lee el ángulo crudo del encoder AS5600 y lo convierte a grados.
 *
 * @return El ángulo leído del encoder en grados.
 */
int16_t getAngle()
{
  int16_t raw;
  readAngleRaw(&raw);
  return raw * 360 / 0xFFF;
}
/**
 * @brief Calcula la diferencia entre un ángulo y un offset.
 *
 * Resta un offset a un ángulo dado, ajustando el resultado para mantenerlo dentro del rango de 0 a 360 grados.
 *
 * @param angle Ángulo actual.
 * @param angleOffset Offset a restar del ángulo.
 *
 * @return Diferencia de ángulos ajustada al rango de 0 a 360 grados.
 */
int16_t angleSubtraction(int16_t angle, int16_t angleOffset)
{
  int16_t angleSub = angle - angleOffset;
  if (angleSub < 0)
  {
    return 360 + angleSub;
  }
  else
  {
    return angleSub;
  }
}
/**
 * @brief Obtiene y establece los offsets de ángulo para cada motor.
 *
 * Utiliza la función 'getAngle' para leer y almacenar los ángulos iniciales de los encoders
 * como offsets para cada motor.
 */
void getOffsets(){
  tca_select_channel(0);
  offsetAngleMotor1 = getAngle();
  tca_select_channel(1);
  offsetAngleMotor2 = getAngle();
  tca_select_channel(2);
  offsetAngleMotor3 = getAngle(); 
  tca_select_channel(3);
  offsetAngleMotor4 = getAngle();
}
