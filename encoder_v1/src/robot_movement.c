/**
 * @file robot_movement.c
 * @brief Implementación de funciones para el control de movimiento del robot.
 *
 * Este archivo contiene la implementación de funciones para el control del movimiento del robot,
 * incluyendo funciones para girar, avanzar, realizar movimientos circulares, y gestionar el giroscopio
 * y acelerómetro para la actualización del ángulo de orientación del robot.
 * Además, se definen variables globales y constantes utilizadas en el control de movimiento.
 */
#include "robot_movement.h"

#define N_SAMPLES 100            ///< Número de muestras para el filtro del acelerometro.
#define ACCEL_SCALE_FACTOR 16384.0 ///< Factor de escala para datos del acelerómetro.
// Variables para almacenar los offsets

double wTimeUpdateAngle = 0.0022; ///< Intervalo de tiempo para la actualización del ángulo.

float offsetXa = 0, offsetYa = 0;///< Offsets para el acelerómetro en los ejes X e Y.
uint16_t offCW1 = 726; ///< Offset para el motor 1 en sentido horario.
uint16_t offCW2 = 720; ///< Offset para el motor 2 en sentido horario.
uint16_t offCW3 = 780; ///< Offset para el motor 3 en sentido horario.
uint16_t offCW4 = 720; ///< Offset para el motor 4 en sentido horario.
uint16_t offCC1 = 780; ///< Offset para el motor 1 en sentido antihorario.
uint16_t offCC2 = 780; ///< Offset para el motor 2 en sentido antihorario. 
uint16_t offCC3 = 720; ///< Offset para el motor 3 en sentido antihorario.
uint16_t offCC4 = 774; ///< Offset para el motor 4 en sentido antihorario.

int refVelMotor1=12;///< Velocidad de referencia para el motor 1
int refVelMotor2=12;///< Velocidad de referencia para el motor 2
int refVelMotor3=12;///< Velocidad de referencia para el motor 3
int refVelMotor4=12;///< Velocidad de referencia para el motor 4
double prevErrorAngle =0; ///< Error angular previo para control PID o similar.

double prevMpuOffset = 0; ///< Offset previo del MPU
uint16_t angleMotor1 = 0;///< Ángulo acumulado del motor 1.
uint16_t angleMotor2 = 0;///< Ángulo acumulado del motor 2.
uint16_t angleMotor3 = 0;///< Ángulo acumulado del motor 3.
uint16_t angleMotor4 = 0;///< Ángulo acumulado del motor 4.

uint16_t turnMotor1 = 0; ///< Contador de giros del motor 1.
uint16_t turnMotor2 = 0;  ///< Contador de giros del motor 2.
uint16_t turnMotor3 = 0;  ///< Contador de giros del motor 3.
uint16_t turnMotor4 = 0;  ///< Contador de giros del motor 4.

bool banTurnsMotor1 = 0; ///< Indicador de control de giros para el motor 1.
bool banTurnsMotor2 = 0; ///< Indicador de control de giros para el motor 2.
bool banTurnsMotor3 = 0;///< Indicador de control de giros para el motor 3.
bool banTurnsMotor4 = 0;///< Indicador de control de giros para el motor 4.

bool banStop = false; ///< Indicador de detención del movimiento.

int ax, ay;///< Valores de aceleración en ejes X e Y.
int prevAy=10000000;///< Valor previo de aceleración en eje Y para control.


double angularPosition=0, prevAngularPosition=0; ///< Posición angular actual y previa.
double angularVelocity; ///< Velocidad angular calculada.
double robotAngle = 0; ///< Ángulo total acumulado del robot.
bool angularFlag = true; ///< Flag para control del calculo del offset.
int offsetZ;///< Offset para el giroscopio en eje Z.
int16_t acceleration[3], gyro[3];

uint64_t prevTimeW = 0;
double ventana = 0; 

/**
 * @brief Gira el motor 1 en sentido contrario a las agujas del reloj.
 *
 * Establece el nivel PWM para el motor 1 para girarlo en sentido contrario a las agujas del reloj.
 * Utiliza el offset predefinido 'offCC1' y un offset ajustable 'offset1'.
 */
void motorCounterClockWise1(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, offCC1 + offset1); // 777
}
/**
 * @brief Gira el motor 1 en sentido de las agujas del reloj.
 *
 * Establece el nivel PWM para el motor 1 para girarlo en sentido de las agujas del reloj.
 * Utiliza el offset predefinido 'offCW1' y un offset ajustable 'offset1'.
 */
void motorClockWise1(){
  if((offCW1 + offset1) >= 718){
    if((offCW1+offset1) >= 738){
      offset1 = 738-offCW1;
    }
    pwm_set_chan_level(slice_num_5, PWM_CHAN_A, offCW1 + offset1); // 723
  }else{
    offset1=-15;
  }
}
/**
 * @brief Gira el motor 2 en sentido contrario a las agujas del reloj.
 *
 * Establece el nivel PWM para el motor 2 para girarlo en sentido contrario a las agujas del reloj.
 * Utiliza el offset predefinido 'offCC2' y un offset ajustable 'offset2'.
 */
void motorCounterClockWise2(){
  if((offCC2 + offset2)<=790){
     if((offCC2 + offset2) <= 770){
      offset2 = 770-offCC2;
    }
    pwm_set_chan_level(slice_num_5, PWM_CHAN_B, offCC2 + offset2); // 780
  }else{
    offset2=12;
  }
}
/**
 * @brief Gira el motor 2 en sentido de las agujas del reloj.
 *
 * Establece el nivel PWM para el motor 1 para girarlo en sentido de las agujas del reloj.
 * Utiliza el offset predefinido 'offCW2' y un offset ajustable 'offset2'.
 */
void motorClockWise2(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, offCW2 + offset2); // 720
}
/**
 * @brief Gira el motor 3 en sentido contrario a las agujas del reloj.
 *
 * Establece el nivel PWM para el motor 3 para girarlo en sentido contrario a las agujas del reloj.
 * Utiliza el offset predefinido 'offCC3' y un offset ajustable 'offset3'.
 */
void motorCounterClockWise3(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, offCC3 + offset3); //700

}
/**
 * @brief Gira el motor 3 en sentido de las agujas del reloj.
 *
 * Establece el nivel PWM para el motor 1 para girarlo en sentido de las agujas del reloj.
 * Utiliza el offset predefinido 'offCW3' y un offset ajustable 'offset3'.
 */
void motorClockWise3(){
  if((offCW3+offset3) <= 785){
    if((offCW3 + offset3) <= 766){
      offset3 = 766-offCW3;
    }
    pwm_set_chan_level(slice_num_6, PWM_CHAN_A, offCW3 + offset3); // 780
  }else{
    offset3=10;
  }
}
/**
 * @brief Gira el motor 4 en sentido contrario a las agujas del reloj.
 *
 * Establece el nivel PWM para el motor 4 para girarlo en sentido contrario a las agujas del reloj.
 * Utiliza el offset predefinido 'offCC4' y un offset ajustable 'offset4'.
 */
void motorCounterClockWise4(){
  if((offCC4 + offset4) <= 785){
    if((offCC4 + offset4) <= 766){
      offset4 = 766-offCC4;
    }
    pwm_set_chan_level(slice_num_6, PWM_CHAN_B, offCC4 + offset4); 
  }else{
    offset4 = 10;
  }
}
/**
 * @brief Gira el motor 4 en sentido de las agujas del reloj.
 *
 * Establece el nivel PWM para el motor 4 para girarlo en sentido de las agujas del reloj.
 * Utiliza el offset predefinido 'offCW4' y un offset ajustable 'offset4'.
 */
void motorClockWise4(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, offCW4 + offset4); // 720
}
void motorStop(){
  // taskENTER_CRITICAL();
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 750);
  // taskEXIT_CRITICAL();
}

/**
 * @brief Mueve todos los motores para avanzar hacia adelante.
 *
 * Activa los motores en una configuración que hace que el robot se mueva hacia adelante.
 * Esto se logra girando los motores 1 y 3 en el sentido de las agujas del reloj y los motores 2 y 4 en sentido contrario.
 */
void motorsForward(){
  // taskENTER_CRITICAL();
  motorClockWise1();
  motorCounterClockWise2();
  motorClockWise3();
  motorCounterClockWise4();
  // taskEXIT_CRITICAL();
}
/**
 * @brief Gira todos los motores en el sentido de las agujas del reloj.
 *
 * Activa los cuatro motores para girar en el sentido de las agujas del reloj.
 * Esta configuración hace que el robot gire sobre su eje central en el sentido de las agujas del reloj.
 */
void motorsClockWise(){
  // taskENTER_CRITICAL();
  motorClockWise1();
  motorClockWise2();
  motorClockWise3();
  motorClockWise4();
  // taskEXIT_CRITICAL();
}
/**
 * @brief Controla los motores para avanzar una distancia específica.
 *
 * Esta función utiliza la función 'distanceRobotClockWise' o 'distanceRobotCounterClockWise'
 * para cada motor con el fin de avanzar una distancia determinada, controlando
 * la rotación de cada motor en su respectiva dirección.
 */
void distanceMotorsForward(){
  // taskENTER_CRITICAL();
  distanceRobotClockWise(angleMotor1,&turnMotor1,&banTurnsMotor1,&distanceMotor1,&velMotor1,&windowTimeMotor1,&prevTimeUsMotor1);
  distanceRobotCounterClockWise(angleMotor2,&turnMotor2,&banTurnsMotor2,&distanceMotor2,&velMotor2,&windowTimeMotor2,&prevTimeUsMotor2);
  distanceRobotClockWise(angleMotor3,&turnMotor3,&banTurnsMotor3,&distanceMotor3,&velMotor3,&windowTimeMotor3,&prevTimeUsMotor3);
  distanceRobotCounterClockWise(angleMotor4,&turnMotor4,&banTurnsMotor4,&distanceMotor4,&velMotor4,&windowTimeMotor4,&prevTimeUsMotor4);
  // taskEXIT_CRITICAL();
}

/**
 * @brief Controla los motores para girar el robot en el sentido de las agujas del reloj una distancia específica.
 *
 * Activa cada motor en el sentido de las agujas del reloj para lograr un giro coordinado
 * del robot. Utiliza la función 'distanceRobotClockWise' para cada motor, permitiendo
 * un control preciso de la distancia de giro.
 */
void distanceMotorsClockWise(){
  // taskENTER_CRITICAL();
  distanceRobotClockWise(angleMotor1,&turnMotor1,&banTurnsMotor1,&distanceMotor1,&velMotor1,&windowTimeMotor1,&prevTimeUsMotor1);
  distanceRobotClockWise(angleMotor2,&turnMotor2,&banTurnsMotor2,&distanceMotor2,&velMotor2,&windowTimeMotor2,&prevTimeUsMotor2);
  distanceRobotClockWise(angleMotor3,&turnMotor3,&banTurnsMotor3,&distanceMotor3,&velMotor3,&windowTimeMotor3,&prevTimeUsMotor3);
  distanceRobotClockWise(angleMotor4,&turnMotor4,&banTurnsMotor4,&distanceMotor4,&velMotor4,&windowTimeMotor4,&prevTimeUsMotor4);
  // taskEXIT_CRITICAL();  
}

void calibrate() {
  float sum_x = 0, sum_y = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    sum_x += acceleration[0];
    sum_y += acceleration[1];

    sleep_ms(10); // Pequeña pausa entre lecturas
  }
  offsetXa = sum_x / N_SAMPLES;
  offsetYa = sum_y / N_SAMPLES;
}

// Función para procesar los datos del acelerómetro
void readAndProcessAccelerometer(int *ax, int *ay) {
    // Restar los offsets y convertir a m/s²
  *ax = (acceleration[0] - offsetXa) / ACCEL_SCALE_FACTOR * 9.81;
  *ay = (acceleration[1] - offsetYa) / ACCEL_SCALE_FACTOR * 9.81;

}

/**
 * @brief Realiza la rotación del robot en base a un ángulo especificado.
 *
 * Esta función configura y controla los motores para rotar el robot un ángulo determinado.
 * Modifica los parámetros de control de los motores y utiliza la función 'motorsClockWise'
 * para iniciar la rotación. La rotación se detiene cuando el robot alcanza el ángulo deseado.
 *
 * @param rotationAngle Ángulo de rotación deseado en grados.
 *                      Un valor positivo indica una rotación en el sentido de las agujas del reloj.
 *
 * @note La función monitorea el ángulo de rotación actual del robot ('robotAngle') y compara
 *       este valor con el ángulo objetivo ('rotationAngle') para detener los motores una vez alcanzado
 *       el objetivo. Utiliza 'motorStop' para detener los motores, 'restartControl' y 'restartMovement'
 *       para restablecer los controles del robot, y 'getOffsets' para actualizar los parámetros necesarios.
 */
void rotation(int rotationAngle){
  wTimeUpdateAngle=0.0028;
  /// Ajusta los offsets para los motores en la rotación.
  offCW1 = 736;
  offCW2 = 736;
  offCW3 = 767; 
  offCW4 = 736;
  /// Inicia la rotación en el sentido de las agujas del reloj si el ángulo es positivo.
  if(rotationAngle > 0){
    motorsClockWise();
    /// Verifica si el ángulo objetivo está cerca de ser alcanzado.    
    if(robotAngle+20>=rotationAngle){      
      motorStop();       ///< Detiene los motores
      restartControl();  ///< Reinicia los controles del robot
      restartMovement(); ///< Reinicia el movimiento
      getOffsets();      ///< Actualiza los offsets
      banStop = true;    ///< Indica que se ha completado la rotación  
    }
  }else if(rotationAngle < 0){
    motorCounterClockWise1();
    motorCounterClockWise2();
    motorCounterClockWise3();
    motorCounterClockWise4();
    if(robotAngle-20 <= rotationAngle){
      motorStop();       ///< Detiene los motores
      restartControl();  ///< Reinicia los controles del robot
      restartMovement(); ///< Reinicia el movimiento
      getOffsets();      ///< Actualiza los offsets
      banStop = true;    ///< Indica qu
    }
  }else{
    banStop=true;
  }
}

/**
 * @brief Mueve el robot hacia adelante una distancia específica.
 *
 * Esta función controla los motores para mover el robot hacia adelante la distancia especificada.
 * Configura los offsets para los motores y utiliza 'motorsForward' y 'distanceMotorsForward'
 * para avanzar. El movimiento se ajusta en base al ángulo actual del robot para mantener la dirección.
 *
 * @param distance Distancia a avanzar en unidades específicas.
 *                 Solo se toman acciones si la distancia es mayor a cero.
 *
 * @note La función calcula la posición final basándose en la distancia recorrida por los motores
 *       y ajusta la velocidad de los motores individualmente si es necesario para mantener la dirección.
 *       Utiliza 'motorStop' para detener los motores, 'restartControl' y 'restartMovement'
 *       para restablecer los controles del robot, y 'getOffsets' para actualizar los parámetros necesarios.
 */
void moveForward(double distance){
  /// Ajusta los offsets para los motores en movimiento hacia adelante.
  offCW1 = 733;
  offCC2 = 779;
  offCW3 = 775;
  offCC4 = 775;
  wTimeUpdateAngle=0.0022;
  if (distance > 0){  
    int mpuOffset = 0.5*robotAngle;///< Offset basado en el ángulo actual del robot.
    motorsForward(); ///< Inicia el movimiento hacia adelante.
    distanceMotorsForward(); ///< Controla la distancia a avanzar.
    /// Calcula la posición final basada en la distancia recorrida.
    double posx1 = (distanceMotor1+distanceMotor4)*cos(52*PI/180)/2;
    double posx2 = (distanceMotor2+distanceMotor3)*cos(52*PI/180)/2;
    double finalPos = (posx1 + posx2)/2;
    printf("Distancia calculada %d \n",finalPos);
    if(finalPos >= distance){
      motorStop();      ///< Detiene los motores
      restartControl(); ///< Reinicia los controles del robot
      restartMovement();///< Reinicia el movimiento
      getOffsets();     ///< Actualiza los offsets
      banStop = true;   ///< Indica que se ha completado el movimiento
    }else{
      /// Ajusta la velocidad de los motores en función del ángulo actual.
      if(mpuOffset == 0){
        m2ControlSpeed(refVelMotor2,-1);
        m4ControlSpeed(refVelMotor4,-1);
        m1ControlSpeed(refVelMotor1,1);
        m3ControlSpeed(refVelMotor3,1);
      }else{
        if(mpuOffset>0){
          m2ControlSpeed(refVelMotor2+mpuOffset,-1);
          m1ControlSpeed(refVelMotor1-mpuOffset/2,1);
        }else{
          m2ControlSpeed(refVelMotor2+mpuOffset/2,-1);
          m1ControlSpeed(refVelMotor1-mpuOffset,1);
        }
        m4ControlSpeed(refVelMotor4,-1);
        m3ControlSpeed(refVelMotor3,1);
      }
      motorsForward();         //< Continúa el movimiento hacia adelante.
      distanceMotorsForward(); //< Continúa controlando la distancia a avanzar.
    }
    //dualMotorPIDControl();
  }
}

/**
 * @brief Realiza un movimiento circular del robot con un radio y ángulo específicos.
 *
 * Esta función controla los motores para mover el robot en un trayecto circular.
 * Configura los offsets de los motores y ajusta sus velocidades para crear un movimiento circular.
 * El movimiento se detiene una vez que el robot alcanza el ángulo de giro deseado.
 *
 * @param r Radio del círculo en metros.
 * @param angle Ángulo de giro deseado en grados.
 *
 * @note La función calcula las velocidades para los motores de manera que los motores
 *       en los lados opuestos del robot se mueven a velocidades diferentes, creando así un movimiento circular.
 *       Utiliza 'motorStop' para detener los motores, 'restartControl' y 'restartMovement'
 *       para restablecer los controles del robot, y 'getOffsets' para actualizar los parámetros necesarios.
 */
void circularMovement(double r, int angle){
  wTimeUpdateAngle=0.0022;
  /// Ajusta los offsets para los motores en movimiento circular.
  offCW1 = 733;
  offCC2 = 777;
  offCW3 = 775; 
  offCC4 = 775;
  motorsForward();
  distanceMotorsForward();
  /// Calcula las velocidades para los motores basándose en el radio.
  int16_t velM24 =ceil((15/r)*(r - 0.135/2));
  int16_t velM13 = (15/r)*(r + 0.135/2);
  /// Ajusta la velocidad de los motores para el movimiento circular.
  m2ControlSpeed(velM24,-1);
  m4ControlSpeed(velM24,-1);
  m1ControlSpeed(velM13,1);
  m3ControlSpeed(velM13,1);
  ///Calcula la posición actual basada en la distancia recorrida.
  double posx1 = (distanceMotor1+distanceMotor3)/2;

  if(posx1>= angle*PI*r/180){
    motorStop();
    restartControl();
    restartMovement();
    getOffsets();
    banStop=true;
  }

}

/**
 * @brief Reinicia todos los parámetros de movimiento y ángulos del robot.
 *
 * Esta función restablece todos los contadores de ángulos y variables de control de movimiento
 * a sus valores iniciales. Se utiliza para preparar el robot para un nuevo movimiento después
 * de completar una acción o para corregir cualquier desviación acumulada en los controles.
 */
void restartMovement(){
  angleMotor1 = 0;
  angleMotor2 = 0;
  angleMotor3 = 0;
  angleMotor4 = 0;
  turnMotor1 = 0;
  turnMotor2 = 0;
  turnMotor3 = 0;
  turnMotor4 = 0;
  banTurnsMotor1 = 0;
  banTurnsMotor2 = 0;
  banTurnsMotor3 = 0;
  banTurnsMotor4 = 0;
  angularPosition=0;
  prevAngularPosition=0;
  angularVelocity=0;
  robotAngle = 0;
}
/**
 * @brief Actualiza el ángulo de orientación del robot basándose en los datos del giroscopio.
 *
 * Esta función lee los datos crudos del giroscopio y los procesa para actualizar el ángulo de orientación
 * del robot. Utiliza un filtro de mediana móvil para obtener un valor de compensación inicial, y luego
 * calcula la velocidad angular y la posición angular acumulada para ajustar el ángulo del robot.
 *
 * @note La función se basa en un flag inicial 'angularFlag' para realizar la compensación inicial
 *       y luego pasa al cálculo continuo de la posición angular. La velocidad angular se calcula
 *       ajustándola con el valor de compensación y se usa para actualizar la posición angular y, por
 *       lo tanto, el ángulo total del robot.
 */

void updateAngle(){
  
  /// Lee los datos crudos del acelerómetro y giroscopio.
  mpu6050_read_raw(acceleration,gyro);
  /// Fase inicial: calcula el offset del giroscopio
  if(angularFlag==true){
    offsetZ = filter_median_moving(gyro[2]);
    if (index_media==9){
      angularFlag = false;
    }
  }
  else {
    uint64_t currentTime = time_us_64();
    // printf("%ld\n",currentTime-prevTimeW);
    ventana = ((double)(currentTime - prevTimeW)) / 1000000.0;
    prevTimeW=currentTime;
    /// Fase de cálculo continuo: actualiza la velocidad y posición angular.
    angularVelocity = (gyro[2] > 0 ? gyro[2]+offsetZ : gyro[2]-offsetZ)/131;
    double angle = (prevAngularPosition + (angularVelocity*ventana));
    prevAngularPosition = angle;
    angularPosition = angle > 0 ? angle*1: angle*1;
    robotAngle = angle;
    /// Ajusta el ángulo total del robot.
    // if (angularPosition>=2){
    //   prevAngularPosition = 0;
    //   robotAngle += 2;
    // }
    // else if (angularPosition<=-2)
    // {
    //   prevAngularPosition = 0;
    //   robotAngle -= 2;
    // }
  }
}