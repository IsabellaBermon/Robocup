/**
 * @file MPU6050_i2c.h
 * @brief Control del sensor MPU6050 a través de I2C para un robot.
 *
 * Este archivo contiene funciones para inicializar la comunicación I2C con el MPU6050,
 * leer datos crudos del acelerómetro y el giroscopio, y filtrar los datos del giroscopio.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "MPU6050_i2c.h"


/// Buffer y variables para el filtro de mediana móvil.
int gyro_z_readings[WINDOW_SIZE] = {0};
int index_media = 0;
int sum = 0;


/**
 * @brief Inicializa la interfaz I2C para la comunicación con el MPU6050.
 *
 * Configura los pines para la comunicación I2C y establece las resistencias de pull-up necesarias.
 */
void mpu_init(){
    i2c_init(MPU6050_i2c,100000);
    gpio_set_function(SDA_MPU,GPIO_FUNC_I2C);
    gpio_set_function(SCL_MPU,GPIO_FUNC_I2C);
    gpio_pull_up(SDA_MPU);
    gpio_pull_up(SCL_MPU);
}
/**
 * @brief Restablece el MPU6050 a través de la interfaz I2C.
 *
 * Envía un comando de restablecimiento al MPU6050 para inicializar o reiniciar el sensor.
 */
void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_timeout_us(MPU6050_i2c, MPU6050_addr, buf, sizeof(buf), false, 1000);
}
/**
 * @brief Lee los datos crudos del acelerómetro y el giroscopio del MPU6050.
 *
 * Solicita y lee los datos crudos de los sensores de aceleración y giroscopio del MPU6050.
 *
 * @param accel Buffer para almacenar los datos del acelerómetro.
 * @param gyro Buffer para almacenar los datos del giroscopio.
 */
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[6];

    uint8_t val = 0x3B;
    i2c_write_timeout_us(MPU6050_i2c, MPU6050_addr, &val, sizeof(val), false, 1000); 
    i2c_read_timeout_us(MPU6050_i2c, MPU6050_addr, buffer, sizeof(buffer), false, 1000);

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];

    val = 0x43;
    i2c_write_timeout_us(MPU6050_i2c, MPU6050_addr, &val, sizeof(val), false, 1000);
    i2c_read_timeout_us(MPU6050_i2c, MPU6050_addr, buffer, sizeof(buffer), false, 1000); 

    gyro[0] = (buffer[0] << 8) | buffer[1];
    gyro[1] = (buffer[2] << 8) | buffer[3];
    gyro[2] = (buffer[4] << 8) | buffer[5];
}


/**
 * @brief Aplica un filtro de mediana móvil a una nueva lectura de giroscopio.
 *
 * Actualiza un buffer de lecturas de giroscopio y calcula la mediana móvil.
 *
 * @param new_reading Nueva lectura del giroscopio para agregar al filtro.
 * @return Valor filtrado (promedio) de las lecturas del giroscopio.
 */
int filter_median_moving(int new_reading) {
    // Subtract the oldest reading from sum
    sum -= gyro_z_readings[index_media];
    // Add the new reading to sum
    sum += new_reading;
    // Store the new reading in the buffer
    gyro_z_readings[index_media] = new_reading;
    // Increment the index
    index_media = (index_media + 1) % WINDOW_SIZE;
    // Return the average
    return sum / WINDOW_SIZE;
}

