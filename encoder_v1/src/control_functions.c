/**
 * @file control_functions.c
 * @brief Funciones de control para el movimiento y velocidad de los motores del robot.
 *
 * Este archivo contiene funciones que calculan la distancia recorrida, velocidad y ajustes finos
 * de velocidad para los motores del robot. También incluye funciones para reiniciar los parámetros
 * de control y variables de estado de los motores.
 */
#include "control_functions.h"

double distanceMotor1 = 0;
double distanceMotor2 = 0;
double distanceMotor3 = 0;
double distanceMotor4 = 0;

double motorAngle1=0;
double motorAngle2=0;
double motorAngle3=0;
double motorAngle4=0;

int16_t offset1=0;  
int16_t offset2=0;  
int16_t offset3=0;  
int16_t offset4=0; 

double integralError1_4 = 0;
double integralError2_3 = 0;
double previousError1_4 = 0;
double previousError2_3 = 0;

double filtroD = 0.0;
double alpha = 0.1;  // Factor de suavizado, ajustar según sea necesario
double previousErrorAngle = 0;
double integralErrorAngle = 0;

uint64_t prevTimeUsMotor1=0;
uint64_t prevTimeUsMotor2=0;
uint64_t prevTimeUsMotor3=0;
uint64_t prevTimeUsMotor4=0;

double windowTimeMotor1=0;
double windowTimeMotor2=0;
double windowTimeMotor3=0;
double windowTimeMotor4=0;

int velMotor1 = 0;
int velMotor2 = 0;
int velMotor3 = 0;
int velMotor4 = 0;

double prevErrorVel1 = 0;
double prevErrorVel2 = 0;
double prevErrorVel3 = 0;
double prevErrorVel4 = 0;

/**
 * @brief Calcula la distancia recorrida y la velocidad del robot en sentido contrario a las agujas del reloj.
 *
 * Esta función actualiza la distancia recorrida, la velocidad del motor y los giros
 * completos cuando el motor gira en sentido contrario a las agujas del reloj. Se basa en
 * el ángulo del motor y en el conteo de giros completos para calcular la distancia total recorrida.
 *
 * @param angleMotor Ángulo actual del motor.
 * @param turnMotor Puntero al contador de giros completos del motor.
 * @param banTurnsMotor Puntero a la bandera que indica si el motor ha completado un giro.
 * @param distanceMotor Puntero a la variable que almacena la distancia total recorrida.
 * @param velMotor Puntero a la variable que almacena la velocidad del motor.
 * @param windowTimeMotor Puntero al intervalo de tiempo utilizado para el cálculo de la velocidad.
 * @param prevTimeUsMotor Puntero al registro del tiempo anterior en microsegundos.
 *
 * @note La función utiliza un umbral de ángulo (330 grados) para detectar un giro completo.
 *       Cada vez que se completa un giro, se actualiza el contador de giros y, periódicamente,
 *       la velocidad del motor. La distancia total se calcula multiplicando el número de giros
 *       por la circunferencia de la trayectoria del motor.
 */
void distanceRobotCounterClockWise(uint16_t angleMotor, uint16_t *turnMotor, bool *banTurnsMotor, double *distanceMotor, int *velMotor, double *windowTimeMotor, uint64_t *prevTimeUsMotor){
    /// Condición para actualizar el contador de giros y la distancia.
    if (angleMotor >= 330 && *banTurnsMotor){
        (*turnMotor)++;
        /// Actualiza la velocidad cada 5 giros.
        if((*turnMotor) % 5 == 0){
            uint64_t currentTimeUs = time_us_64();
            *windowTimeMotor = (currentTimeUs - (*prevTimeUsMotor)) / 1000;
            *velMotor = (circunference * 100000) / (*windowTimeMotor); ///< se calcula la velocidad el motor
            *prevTimeUsMotor = currentTimeUs;
        }
        *distanceMotor = (*turnMotor) * circunference; ///< Se calcula la distancia

        *banTurnsMotor = false;
    }
    else if (angleMotor <= 30){
        /// Restablece la bandera para permitir la detección de nuevos giros.
        *banTurnsMotor = true;
    }
}
/**
 * @brief Calcula la distancia recorrida y la velocidad del robot en sentido de las agujas del reloj.
 *
 * Esta función actualiza la distancia recorrida, la velocidad del motor y los giros
 * completos cuando el motor gira en sentido de las agujas del reloj. Se basa en
 * el ángulo del motor y en el conteo de giros completos para calcular la distancia total recorrida.
 *
 * @param angleMotor Ángulo actual del motor.
 * @param turnMotor Puntero al contador de giros completos del motor.
 * @param banTurnsMotor Puntero a la bandera que indica si el motor ha completado un giro.
 * @param distanceMotor Puntero a la variable que almacena la distancia total recorrida.
 * @param velMotor Puntero a la variable que almacena la velocidad del motor.
 * @param windowTimeMotor Puntero al intervalo de tiempo utilizado para el cálculo de la velocidad.
 * @param prevTimeUsMotor Puntero al registro del tiempo anterior en microsegundos.
 *
 * @note La función utiliza un umbral de ángulo (30 grados) para detectar un giro completo en sentido
 *       de las agujas del reloj. Cada vez que se completa un giro, se actualiza el contador de giros y,
 *       periódicamente, la velocidad del motor. La distancia total se calcula multiplicando el número
 *       de giros por la circunferencia de la trayectoria del motor.
 */
void distanceRobotClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, double *distanceMotor,int *velMotor,double *windowTimeMotor,uint64_t *prevTimeUsMotor){
  if (angleMotor >= 330){
    *banTurnsMotor = true;
  }
  else if (angleMotor <= 30  && *banTurnsMotor ){
    (*turnMotor)++;
    if((*turnMotor)%5 == 0){
      uint64_t currentTimeUs = time_us_64();
      *windowTimeMotor = (currentTimeUs - (*prevTimeUsMotor))/1000;
      *velMotor= (circunference*100000)/(*windowTimeMotor);
      *prevTimeUsMotor = currentTimeUs;
    }
    *distanceMotor = (*turnMotor)*circunference;
    *banTurnsMotor = false;
  } 
}

/**
 * @brief Ajusta la velocidad de un motor específico.
 *
 * Esta función incrementa el offset de velocidad de un motor seleccionado. El offset se utiliza
 * para ajustar finamente la velocidad del motor en respuesta a condiciones de control dinámico.
 *
 * @param motorNumber Número del motor a ajustar. Los valores válidos son 1, 2, 3, y 4.
 * @param adjustment Valor de ajuste que se añadirá al offset actual del motor.
 *
 * @note Esta función modifica los offsets globales 'offset1', 'offset2', 'offset3', y 'offset4',
 *       dependiendo del número de motor especificado. Un ajuste positivo incrementa el offset,
 *       mientras que un ajuste negativo lo disminuye.
 */
void adjustMotorSpeed(uint motorNumber, double adjustment) {
  /// Selecciona el motor y ajusta su offset de velocidad.
  switch (motorNumber){
    case 1:
      offset1 += adjustment;
      break;
    case 2:
      offset2 += adjustment;
      break;
    case 3:
      offset3 += adjustment;
      break;
    case 4:
      offset4 += adjustment;
      break; 
    default:
      break;
}
}

/**
 * @brief Controla la velocidad del motor 1.
 *
 * Esta función ajusta la velocidad del motor 1 utilizando un controlador PID.
 * Calcula el error entre la velocidad de referencia y la velocidad actual del motor,
 * y aplica un ajuste basado en este error.
 *
 * @param velRef Velocidad de referencia para el motor.
 * @param turn Dirección del ajuste de velocidad (positivo o negativo).
 *
 * @note Utiliza 'adjustMotorSpeed' para realizar el ajuste de velocidad. La dirección
 *       del ajuste depende del signo de 'turn' y del cálculo PID.
 */
void m1ControlSpeed(int velRef, int turn){
  double error = velRef - velMotor1;
  double pid = 0.25*error + 0.05*(error-prevErrorVel1);

  if(error != prevErrorVel1){
    if(error > 0){
      adjustMotorSpeed(1, pid > 0 ? -turn*pid : turn*pid);  

    }else if (error < 0){
      adjustMotorSpeed(1, pid > 0 ? turn*pid : -turn*pid);  
    }
  }
  prevErrorVel1 = error;
}
/**
 * @brief Controla la velocidad del motor 2.
 *
 * Análogo a 'm1ControlSpeed', pero para el motor 2.
 */
void m2ControlSpeed(int velRef,int turn){
  double error = velRef - velMotor2;
  double pid = 0.25*error + 0.05*(error-prevErrorVel2);

  if(error != prevErrorVel2){
    if(error > 0){
      adjustMotorSpeed(2, pid > 0 ? -turn*pid : turn*pid);  

    }else if (error < 0){
      adjustMotorSpeed(2, pid > 0 ? turn*pid : -turn*pid);  
    }
  }
  prevErrorVel2 = error;
}
/**
 * @brief Controla la velocidad del motor 3.
 *
 * Análogo a 'm1ControlSpeed', pero para el motor 3.
 */
void m3ControlSpeed(int velRef,int turn){
  double error = velRef - velMotor3;
  double pid = 0.3*error + 0.05*(error-prevErrorVel3);

  if(error != prevErrorVel3){
    if(error > 0){
      adjustMotorSpeed(3, pid > 0 ? turn*pid : -turn*pid);  

    }else if (error < 0){
      adjustMotorSpeed(3, pid > 0 ? -turn*pid : turn*pid);  
    }
  }
  prevErrorVel3 = error;
}

/**
 * @brief Controla la velocidad del motor 4.
 *
 * Análogo a 'm1ControlSpeed', pero para el motor 4.
 */
void m4ControlSpeed(int velRef,int turn){
  double error = velRef - velMotor4;
  double pid = 0.3*error + 0.05*(error-prevErrorVel4);

  if(error != prevErrorVel4){
    if(error > 0){
      adjustMotorSpeed(4, pid > 0 ? -turn*pid : turn*pid);  

    }else if (error < 0){
      adjustMotorSpeed(4, pid > 0 ? turn*pid : -turn*pid);  
    }
  }
  prevErrorVel4 = error;
}

/**
 * @brief Reinicia todos los parámetros de control y variables de estado de los motores del robot.
 *
 * Esta función restablece las distancias recorridas, velocidades, ángulos de los motores,
 * offsets de velocidad, errores integrales y errores previos de los controladores PID a sus valores iniciales.
 * Se utiliza para reiniciar el estado de control del robot, típicamente después de completar una tarea
 * o en situaciones donde se necesita un reinicio del estado de control.
 */
void restartControl(){
  distanceMotor1 = 0;
  distanceMotor2 = 0;
  distanceMotor3 = 0;
  distanceMotor4 = 0;
  velMotor1=0;
  velMotor2=0;
  velMotor3=0;
  velMotor4=0;
  motorAngle1=0;
  motorAngle2=0;
  motorAngle3=0;
  motorAngle4=0;
  offset1=0;  
  offset2=0;  
  offset3=0;  
  offset4=0; 
  integralError1_4 = 0;
  integralError2_3 = 0;
  previousError1_4 = 0;
  previousError2_3 = 0;
  previousErrorAngle = 0;
  integralErrorAngle = 0; 
  prevErrorVel1 = 0;
  prevErrorVel2 = 0;
  prevErrorVel3 = 0;
  prevErrorVel4 = 0;  
  prevTimeUsMotor1=0;
  prevTimeUsMotor2=0;
  prevTimeUsMotor3=0;
  prevTimeUsMotor4=0;

}
