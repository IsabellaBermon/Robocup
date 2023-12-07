/**
 * @file main.c
 * 
 * @mainpage Robot Documentation
 *
 * @section intro_sec Introducción
 *
 *  POner aqui introducción
 *
 * @section comp_sec Componentes del Sistema
 *
 * El sistema se basa en un microcontrolador Raspberry Pi Pico y utiliza varios componentes como:
 * - MPU6050: Un sensor de movimiento para medir la aceleración y la orientación.
 * - Encoders: Para el seguimiento preciso de la posición y ángulo de los motores.
 * - Controladores de motor y algoritmos de control PID para la manipulación precisa de los motores.
 * - Comunicación Bluetooth para recibir comandos de movimiento y control.
 *
 * @section func_sec Funcionalidades Principales
 *
 * El programa principal (`main`) coordina las siguientes tareas:
 * - Inicialización de sensores y comunicaciones.
 * - Lectura y procesamiento continuo de datos de sensores.
 * - Ejecución de comandos de movimiento como rotación, movimiento hacia adelante y movimientos circulares.
 * - Actualización del estado y posición del robot.
 
 *
 * @section author_sec Autores
 *
 * - Isabella Bermón Rojas
 * - Jonathan Valencia Sanchez
 * - Daniela Curtas Marulanda
 * - Alejandro Ocampo
 * - Jose Jaramillo
 *
 *
 */
#include <stdio.h>
#include "robot_movement.h"
#include "bt_functions.h"

/**
 * @brief Obtiene los ángulos actuales de los motores ajustados por sus respectivos offsets.
 *
 * Esta función lee los ángulos de cada motor a través de un multiplexor y ajusta estos valores
 * con los offsets previamente calculados.
 */
void getAnglesMotors(){
  tca_select_channel(0);
  angleMotor1 = angleSubtraction(getAngle(),offsetAngleMotor1);
  tca_select_channel(1);
  angleMotor2 = angleSubtraction(getAngle(),offsetAngleMotor2);
  tca_select_channel(2);
  angleMotor3 = angleSubtraction(getAngle(),offsetAngleMotor3);
  tca_select_channel(3);
  angleMotor4 = angleSubtraction(getAngle(),offsetAngleMotor4);
  tca_select_channel(4);
}



/**
 * @brief Función principal que inicializa el sistema y ejecuta el bucle de control del robot.
 *
 * Inicializa los componentes del sistema, como comunicación I2C, MPU6050, Bluetooth y motores.
 * Dentro del bucle principal, realiza lecturas de ángulos, actualiza el ángulo del robot y procesa
 * comandos de movimiento.
 */
int main(){
  stdio_init_all();
  mpu_init();
  mpu6050_reset();
  initI2C();
  getOffsets();
  initBluetooth();    
  initMotor();
  calibrate();

  while (1){
     /// Lectura continua cuando el robot esta en movimiento de los angulos
    if(!banStop){
      getAnglesMotors();
      updateAngle();
    }

    /// Se espera hasta que haya llegado datos del Bluetooth
    if(btAvailable){
      continue;
    }
    ///Ejecución de movimientos basado en los comandos recibidos
    if(banAngle){
      rotation(angleBt);
    }else if(banDistance){
      moveForward(distanceBt);
    }else if(banCircularMovement){
      circularMovement(radioBt,angleTurnBt);
    }
    /// Reinicio de estados y banderas después de completar un movimiento.
    if(banStop){
      banAngle=false;
      banDistance=false;
      banCircularMovement=false;
      btAvailable = true;
      banStop = false;
      
    }
  }

  return 0;
}                       