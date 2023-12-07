/**
 * @file MPU6050_i2c.h
 * @brief Interfaz para la comunicación con el sensor MPU6050.
 *
 * Este archivo de cabecera proporciona funciones y definiciones necesarias para la comunicación con el sensor MPU6050.
 * Incluye la inicialización del sensor, lectura de datos raw, y un filtro de media móvil para procesar las lecturas.
 *
 * @note Se conecta el pin SDA a GPIO 18 y el pin SCL a GPIO 19 para la comunicación I2C.
 *
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#define MPU6050_i2c i2c1
#define MPU6050_addr 0x68
#define WINDOW_SIZE 10
#define SDA_MPU 18
#define SCL_MPU 19

extern int gyro_z_readings[WINDOW_SIZE]; ///< Arreglo para almacenar lecturas del eje Z del giroscopio.
extern int index_media; ///< Índice actual en el filtro de mediana móvil.
extern int sum; ///< Suma acumulada de las lecturas para el filtro de mediana móvil.

/**
 * @brief Inicializa la comunicación con el sensor MPU6050.
 */
void mpu_init();

/**
 * @brief Resetea el sensor MPU6050.
 */
void mpu6050_reset();

/**
 * @brief Lee datos raw del acelerómetro y giroscopio del MPU6050.
 * @param[out] accel Arreglo para almacenar lecturas del acelerómetro (X, Y, Z).
 * @param[out] gyro Arreglo para almacenar lecturas del giroscopio (X, Y, Z).
 */
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]);

/**
 * @brief Aplica un filtro de median móvil a una nueva lectura del giroscopio.
 * @param new_reading Nueva lectura del giroscopio a filtrar.
 * @return Valor filtrado después de aplicar el filtro de median móvil.
 */
int filter_median_moving(int new_reading);