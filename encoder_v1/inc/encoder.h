/**
 * @file sensor_interface.h
 * @brief Interfaz para la comunicación con sensores AS5600 y TCA9548A.
 *
 * Este archivo proporciona funciones y definiciones necesarias para la comunicación
 * con sensores AS5600 y un multiplexor TCA9548A a través del bus I2C.
 */
#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define AS560_ADDR 0x36 ///< Dirección I2C del sensor AS5600.
#define AS560_i2c i2c0 ///< Instancia del controlador I2C utilizada para el sensor AS5600.
#define TCA_ADDR 0x70 ///< Dirección I2C del multiplexor TCA9548A.
#define SIG_SDA 0 ///< Pin de datos (SDA) en el multiplexor TCA9548A.
#define SCL 1 ///< Pin de reloj (SCL) en el multiplexor TCA9548A.

// Variables externas para compensación de ángulos de motores
extern uint16_t offsetAngleMotor1;
extern uint16_t offsetAngleMotor2;
extern uint16_t offsetAngleMotor3;
extern uint16_t offsetAngleMotor4;


/**
 * @brief Inicializa la configuración del bus I2C y los sensores.
 *
 * Esta función debe llamarse antes de utilizar otras funciones de lectura de sensores.
 */
void initI2C();

/**
 * @brief Lee el ángulo sin procesar del sensor AS5600.
 * @param[out] buffer Puntero al buffer donde se almacenarán los datos leídos.
 */
void readAngleRaw(int16_t *buffer);

/**
 * @brief Selecciona un canal específico en el multiplexor TCA9548A.
 * @param channel Número de canal a seleccionar en el multiplexor.
 */
void tca_select_channel(uint8_t channel);

/**
 * @brief Obtiene el ángulo procesado del sensor AS5600.
 * @return Ángulo procesado en grados.
 */
int16_t getAngle();

/**
 * @brief Realiza la resta de dos ángulos teniendo en cuenta la compensación de ángulos.
 * @param angle Ángulo al que se le restará el offset.
 * @param angleOffset Offset a restar al ángulo principal.
 * @return Resultado de la resta de ángulos.
 */
int16_t angleSubtraction(int16_t angle, int16_t angleOffset);

/**
 * @brief Obtiene los offsets de ángulos de los motores.
 * 
 * Esta función debe llamarse inicialmente para obtener los offsets necesarios
 * para compensar los ángulos de los motores.
 */
void getOffsets();
