# Robot Robocup - Sistema de Control 

Esta contiene la implementación de un sistema de control de robot diseñado con 4 ruedas omnidireccionales, una MPU, encoders as5600 y la Raspberry Pi Pico W. El código está estructurado utilizando FreeRTOS para la programación de tareas e incluye módulos para la comunicación Bluetooth, lecturas de sensores, control de motores y funcionalidad de dribbler.

### Características

1. **Comunicación Bluetooth:** El sistema establece una conexión Bluetooth para recibir comandos que controlan el robot.

2. **Lectura de Sensores:** Utiliza sensores para recopilar datos, incluidos los ángulos de los motores e información de un sensor MPU6050.

3. **Control de Motores:** Implementa tareas para varios movimientos del robot, como rotación, movimiento hacia adelante y movimiento circular.

4. **Funcionalidad de Dribbler:** Controla el mecanismo de dribbler y patea la pelota según los comandos recibidos.

### Tareas y Planificación

La implementación utiliza FreeRTOS para crear tareas concurrentes que operan de manera independiente y se programan para ejecutarse en intervalos específicos. Las tareas principales son:

1. **Tarea de Lectura de Sensores (`sensorReadingTask`):**
   - Lee los ángulos de los motores y actualiza el estado interno del robot.
   - Utiliza un semáforo para garantizar la exclusión mutua durante la lectura del sensor.

2. **Tarea de Lectura de MPU (`mpuReadingTask`):**
   - Lee datos del sensor MPU6050 para calcular la orientación del robot.
   - También utiliza un semáforo para gestionar el acceso exclusivo al sensor.

3. **Tarea de Comunicación (`communicationTask`):**
   - Maneja la comunicación Bluetooth utilizando BTstack.
   - Programada para ejecutarse a intervalos regulares.

4. **Tarea de Control de Robot (`robotControlTask`):**
   - Gestiona la ejecución de movimientos específicos del robot basados en comandos Bluetooth.
   - Crea tareas adicionales, como la tarea de rotación, movimiento hacia adelante y movimiento circular.

5. **Tarea de Dribbler (`dribbleTask`):**
   - Controla el mecanismo de dribbler o realiza una acción de patada según los comandos.
   - Ejecuta a intervalos regulares.

### Exclusión Mutua y Semáforos

La implementación incorpora exclusión mutua para garantizar un acceso seguro y sincronizado a recursos compartidos. En particular, se utilizan semáforos para lograr esto:

- **Semáforo para Lectura de Sensores (`sensorSemaphore`):**
  - La tarea `sensorReadingTask` utiliza este semáforo para controlar el acceso al proceso de lectura de sensores. Se adquiere antes de realizar la lectura y se libera después de completarla, evitando conflictos en el acceso simultáneo.

- **Semáforo para Lectura de MPU (`mpuSemaphore`):**
  - Similar al caso de lectura de sensores, la tarea `mpuReadingTask` utiliza este semáforo para gestionar el acceso exclusivo al sensor MPU6050.

### Configuración de FreeRTOS

El código del sistema de control de robot se ve afectado por diversas configuraciones en el archivo `FreeRTOSConfig.h`. Aquí se describen algunas de las configuraciones más relevantes y su impacto en el código:

#### Planificación y Tiempo

- **`configUSE_PREEMPTION` (1):**
  - Habilita el uso de un planificador preemptivo. En esta configuración, las tareas pueden ser interrumpidas en cualquier momento por tareas de mayor prioridad. Esto es esencial en sistemas donde la respuesta rápida a eventos es crítica y se necesita una gestión eficiente del tiempo de CPU. Si se deseara un planificador cooperativo, se configuraría en 0.

- **`configUSE_TICKLESS_IDLE` (0):**
  - Configurado en 1, permite el modo de bajo consumo cuando el sistema está inactivo.

- **`configTICK_RATE_HZ` (1000):**
  - Frecuencia del tick del sistema en Hz, se establece el valor máximo de funcionamiento.

- **`configNUM_CORES` (2):**
  - Especifica el número de núcleos del procesador, 2 para la Raspberry Pi Pico W.

### Dependencias

- [FreeRTOS](https://www.freertos.org/)
- [Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [BTstack](https://github.com/bluekitchen/btstack)

