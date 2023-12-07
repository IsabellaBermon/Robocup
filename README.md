# Control de Movimiento y Funciones de Control para Robot Robocup

## Descripción

Este código implementa el control de movimiento para un robot de Robocup con cuatro motores. El enfoque se centra en el control de velocidad y posición de cada motor del robot, así como en el ajuste de velocidad angular. Incluye varias funciones para el control preciso, medición de distancia y control de velocidad basado en PID. Además gestiona el movimiento del robot, incorporando controles de motor para diferentes direcciones, seguimiento de distancia, calibración y movimientos circulares. A continuación, se presentan descripciones detalladas de cada función:

## Variables Principales

### Distancia y Ángulo de los Motores

- `distanceMotor1`, `distanceMotor2`, `distanceMotor3`, `distanceMotor4`: Almacenan la distancia recorrida por cada motor.
- `motorAngle1`, `motorAngle2`, `motorAngle3`, `motorAngle4`: Almacenan el ángulo de cada motor.

### Offset y Ajuste de Velocidad

- `offset1`, `offset2`, `offset3`, `offset4`: Almacenan los offsets para ajustar la velocidad de cada motor.
- `velMotor1`, `velMotor2`, `velMotor3`, `velMotor4`: Almacenan las velocidades actuales de cada motor.

## Funciones de movimiento
A continuación, se presentan descripciones detalladas de cada función:

1. **distanceRobotCounterClockWise**
   - **Argumentos:** 
      - `angleMotor`: Ángulo del motor.
      - `turnMotor`: Contador de giros del motor.
      - `banTurnsMotor`: Bandera de giros del motor.
      - `distanceMotor`: Distancia recorrida por el motor.
      - `velMotor`: Velocidad del motor.
      - `windowTimeMotor`: Ventana de tiempo para el cálculo de velocidad.
      - `prevTimeUsMotor`: Tiempo previo en microsegundos.
   - **Propósito:** Controla la medición de distancia para el movimiento del motor en sentido contrario a las agujas del reloj. Calcula la distancia, velocidad y ventana de tiempo para cada giro.

2. **distanceRobotClockWise**
   - **Argumentos:** 
      - `angleMotor`: Ángulo del motor.
      - `turnMotor`: Contador de giros del motor.
      - `banTurnsMotor`: Bandera de giros del motor.
      - `distanceMotor`: Distancia recorrida por el motor.
      - `velMotor`: Velocidad del motor.
      - `windowTimeMotor`: Ventana de tiempo para el cálculo de velocidad.
      - `prevTimeUsMotor`: Tiempo previo en microsegundos.
   - **Propósito:** Similar a la función anterior, pero para el movimiento del motor en sentido de las agujas del reloj.

3. **adjustMotorSpeed**
   - **Argumentos:** 
      - `motorNumber`: Número de motor.
      - `adjustment`: Ajuste de velocidad.
   - **Propósito:** Ajusta la velocidad de un motor específico según el ajuste proporcionado.

4. **m1ControlSpeed, m2ControlSpeed, m3ControlSpeed, m4ControlSpeed**
   - **Argumentos:** 
      - `velRef`: Velocidad de referencia.
      - `turn`: Parámetro de giro.
   - **Propósito:** Implementa el control proporcional (P) de velocidad para los motores individuales (Motor 1 a Motor 4). 

5. **dualMotorPIDControl**
   - **Propósito:** Implementa el control PID dual del motor, centrándose principalmente en los errores relacionados con la distancia entre el Motor 1-4 y el Motor 2-3.
6. **dualMotorPDControlRotation**
   - **Propósito:** Implementa el control PD dual del motor para movimientos de rotación, ajustando las velocidades de los motores según el error de rotación.

7. **restartControl**
   - **Propósito:** Reinicia varias variables relacionadas con el control, permitiendo un reinicio limpio.

# Funciones de Movimiento del Robot

1. **motorCounterClockWise1, motorClockWise1, motorCounterClockWise2, motorClockWise2, motorCounterClockWise3, motorClockWise3, motorCounterClockWise4, motorClockWise4**
   - **Propósito:** Controla los movimientos del motor en diferentes direcciones (en sentido contrario a las agujas del reloj y en sentido de las agujas del reloj) para los Motores 1 a 4.

2. **motorStop**
   - **Propósito:** Detiene todos los motores, llevando al robot a un alto completo.

3. **motorsForward, motorsClockWise, distanceMotorsForward, distanceMotorsClockWise**
   - **Propósito:** Funciones para controlar el movimiento del robot en direcciones hacia adelante, en sentido de las agujas del reloj y seguimiento de distancia.

4. **calibrate**
   - **Propósito:** Inicia un proceso de calibración para el acelerómetro, calculando compensaciones basadas en lecturas del acelerómetro.

5. **readAndProcessAccelerometer**
   - **Argumentos:** 
      - `ax`: Aceleración en el eje x.
      - `ay`: Aceleración en el eje y.
   - **Propósito:** Lee y procesa los datos del acelerómetro, ajustándolos por compensaciones y convirtiéndolos a m/s².

6. **updateAngle**
   - **Propósito:** Actualiza la posición angular del robot según los datos del giroscopio, teniendo en cuenta la velocidad angular y actualizando periódicamente la posición angular en intervalos de 10 grados.
