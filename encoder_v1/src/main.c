// IMPLEMENTACIÓN REALIZADA EN EL MONTAJE DEL ROBOT
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "robot_movement.h"
#include "bt_functions.h"
#include "dribbler.h"

SemaphoreHandle_t mutex; ///< Maneja el mutex para operaciones de impresión seguras en un entorno multitarea.
SemaphoreHandle_t sensorSemaphore; ///< Maneja el semáforo para sincronizar el acceso a los sensores del robot.
SemaphoreHandle_t mpuSemaphore; ///< Maneja el semáforo para sincronizar el acceso al MPU (Unidad de Medición Inercial).
TaskHandle_t xTaskToStop = NULL; ///< Maneja la tarea que debe detenerse; inicialmente no hay ninguna.



volatile BaseType_t flagToStopTask = pdFALSE;

// Function declarations
static void HardwareInit()
{
    initBluetooth();
    stdio_init_all();
    mpu_init();
    mpu6050_reset();
    initI2C();
    getOffsets();
    initMotor();
    initMotorControl();
    dribble();
}

void sensorReadingTask(void *pvParameters);
void mpuReadingTask(void *pvParameters);
void robotControlTask(void *pvParameters);
void dribbleTask(void *pvParameters);

const TickType_t xFrequency_sensor = pdMS_TO_TICKS(2.8);     
const TickType_t xFrequency_mpu = pdMS_TO_TICKS(2.2);       
const TickType_t xFrequency_movement = pdMS_TO_TICKS(2.8); 
const TickType_t xFrequency_robotControl = pdMS_TO_TICKS(2.8); 
const TickType_t xFrequency_dribble = pdMS_TO_TICKS(2.8); 

int main(){

  BaseType_t xReturnedA, xReturnedB,xReturnedC,xReturnD,xReturnE;
  HardwareInit();

  mutex = xSemaphoreCreateMutex();
  sensorSemaphore = xSemaphoreCreateBinary();
  mpuSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sensorSemaphore);  // Inicializar el semáforo en estado "libre"
  xSemaphoreGive(mpuSemaphore);  // Inicializar el semáforo en estado "libre"

  // Create tasks on the first core
  xReturnedA = xTaskCreate(sensorReadingTask, "SensorReadingTask", 1000, NULL, 5, NULL);
  xReturnedB = xTaskCreate(mpuReadingTask, "mpuReadingTask", 1000, NULL, 5, NULL);
  xReturnD = xTaskCreate(robotControlTask, "robotControlTask", 1000, NULL, 3, NULL);
  xReturnE = xTaskCreate(dribbleTask, "dribbleControl", 1000, NULL, 3, NULL);

  // Check return values for errors
  if (xReturnedA != pdPASS || xReturnedB != pdPASS)
  {
      printf("Error creating tasks\n");
      panic_unsupported();
  }

  // Start FreeRTOS scheduler
  vTaskStartScheduler();
  panic_unsupported();

}                       

/**
 * @brief Imprime un mensaje de forma segura en un entorno multitarea.
 *
 * Esta función toma un mutex antes de imprimir un mensaje en la consola para evitar
 * conflictos en un entorno multitarea, luego libera el mutex.
 *
 * @param out El mensaje a imprimir.
 */
void vSafePrint(char *out)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    puts(out);
    xSemaphoreGive(mutex);
}

/**
 * @brief Imprime información sobre una tarea de FreeRTOS.
 *
 * Genera y muestra información sobre la tarea actual, incluyendo su nombre, el núcleo en el que
 * se ejecuta y el tiempo actual en milisegundos.
 *
 * @param taskName Nombre de la tarea a imprimir.
 */
void printTaskInfo(const char *taskName)
{
    char out[128];
    sprintf(out, "%s running on Core %d, Time: %lu ms", taskName, get_core_num(), xTaskGetTickCount());
    vSafePrint(out);
}

/**
 * @brief Lee y actualiza los ángulos de los motores del robot.
 *
 * Selecciona cada canal de motor y actualiza el ángulo del motor correspondiente
 * restando el ángulo offset inicial.
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
 * @brief Tarea que se ejecuta periódicamente para leer los ángulos de los motores.
 *
 * Esta tarea se bloquea en un semáforo hasta que es seguro leer los sensores, luego
 * actualiza los ángulos de los motores y libera el semáforo.
 *
 * @param pvParameters Parámetros pasados a la tarea; no se utiliza en esta función.
 */
void sensorReadingTask(void *pvParameters)
{
    while (1)
    {
        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();
        // printTaskInfo("Encoders Reading Task executed");
        if(!banStop){
            // Esperar hasta que sea seguro leer el sensor
            if (xSemaphoreTake(sensorSemaphore, portMAX_DELAY) == pdTRUE)
            {            
                // printTaskInfo("Sensor Reading Task");
                // Realizar la lectura del sensor                
                getAnglesMotors();           
                // Notificar que la lectura del sensor está completa
                xSemaphoreGive(sensorSemaphore);
            }        
        }
        vTaskDelay(pdMS_TO_TICKS(2.8)); 
    }
}

/**
 * @brief Tarea para leer datos del MPU.
 *
 * Esta tarea espera a que sea seguro leer el MPU, luego actualiza la posición angular
 * y libera el semáforo una vez completada la lectura.
 *
 * @param pvParameters Parámetros pasados a la tarea; no se utiliza en esta función.
 */

void mpuReadingTask(void *pvParameters)
{
    while (1)
    {   
        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();
        if(!banStop){  
            // Esperar hasta que sea seguro leer el sensor
            if (xSemaphoreTake(mpuSemaphore, portMAX_DELAY) == pdTRUE)
            {            
                // printTaskInfo("Sensor Reading Task");
                updateAngle();                
                // printf("Ventana de tiempo %.6f \n",ventana);
                // printf("Angulo calculado %f \n",angularPosition);
                // Notificar que la lectura del sensor está completa
                xSemaphoreGive(mpuSemaphore);
            }      
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency_mpu);       
    }
}

/**
 * @brief Tarea para manejar la rotación del robot.
 *
 * Esta tarea controla la rotación del robot basándose en un ángulo recibido. La tarea se detiene
 * si se activa la bandera 'banStop'.
 *
 * @param pvParameters Parámetros pasados a la tarea; no se utiliza en esta función.
 */
void rotationTask(void *pvParameters){
    while (1){
        // Comprobar la bandera para detener la tarea
        if (banStop)
        {
            // printf("flagToStopTask is true. Stopping the task.\n");
            btAvailable = true;
            banStop = false;
            vTaskDelete(NULL);  // Eliminar la tarea
        }
        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();
        // Intentar tomar ambos semáforos
        BaseType_t mpuTaken = xSemaphoreTake(mpuSemaphore, portMAX_DELAY);
        if (mpuTaken == pdTRUE)
        {
            // Realizar rotación del robot
            // printf("Angulo calculado %f \n",robotAngle);
            rotation(angleBt);
        }
        // Liberar el semáforo del MPU si se tomó correctamente
        if (mpuTaken == pdTRUE) { xSemaphoreGive(mpuSemaphore); }

        vTaskDelayUntil(&xLastWakeTime, xFrequency_movement);        
    }
}

/**
 * @brief Tarea para mover el robot hacia adelante.
 *
 * Controla el movimiento hacia adelante del robot hasta una distancia especificada.
 * La tarea se detiene si se activa la bandera 'banStop'.
 *
 * @param pvParameters Parámetros pasados a la tarea; no se utiliza en esta función.
 */
void moveForwardTask(void *pvParameters){
    while (1){
        // Comprobar la bandera para detener la tarea
        if (banStop) {
            btAvailable = true;
            banStop = false;
            vTaskDelete(NULL);  // Eliminar la tarea
        }
        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();
        // Intentar tomar ambos semáforos
        BaseType_t sensorTaken = xSemaphoreTake(sensorSemaphore, portMAX_DELAY);
        BaseType_t mpuTaken = xSemaphoreTake(mpuSemaphore, portMAX_DELAY);
        if (sensorTaken == pdTRUE && mpuTaken == pdTRUE)
        {
            // printTaskInfo("Motor Control Task executed");
            // Realizar el control del motor
            moveForward(distanceBt);
        }
        // Liberar el semáforo del sensor si se tomó correctamente
        if (sensorTaken == pdTRUE) { xSemaphoreGive(sensorSemaphore); }
        // Liberar el semáforo del MPU si se tomó correctamente
        if (mpuTaken == pdTRUE) { xSemaphoreGive(mpuSemaphore); }

        vTaskDelayUntil(&xLastWakeTime, xFrequency_movement);
    }
}


/**
 * @brief Tarea para el movimiento circular del robot.
 *
 * Realiza un movimiento circular basado en un radio y ángulo especificados.
 * La tarea se detiene si se activa la bandera 'banStop'.
 *
 * @param pvParameters Parámetros pasados a la tarea; no se utiliza en esta función.
 */
void circularMovementTask(void *pvParameters){
    while (1){
        // Comprobar la bandera para detener la tarea
        if (banStop) {
            btAvailable = true;
            banStop = false;
            vTaskDelete(NULL);  // Eliminar la tarea
        }
        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();
        // Intentar tomar ambos semáforos
        BaseType_t sensorTaken = xSemaphoreTake(sensorSemaphore, portMAX_DELAY);
        if (sensorTaken == pdTRUE)
        {
            // printTaskInfo("Motor Control Task executed");
            // Realizar el control del motor
            circularMovement(radioBt,angleTurnBt);
        }
        // Liberar el semáforo del sensor si se tomó correctamente
        if (sensorTaken == pdTRUE) { xSemaphoreGive(sensorSemaphore);}

        vTaskDelayUntil(&xLastWakeTime, xFrequency_movement);        
    }
}

/**
 * @brief Controla el robot basado en comandos recibidos.
 *
 * Esta tarea verifica la disponibilidad de comandos Bluetooth y ejecuta movimientos
 * del robot como rotación, movimiento hacia adelante y movimiento circular.
 *
 * @param pvParameters Parámetros pasados a la tarea; no se utiliza en esta función.
 */
void robotControlTask(void *pvParameters) {
    while(1) {
        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();
        /// Se espera hasta que haya llegado datos del Bluetooth
        if(btAvailable){
            continue;
        }
        ///Ejecución de movimientos basado en los comandos recibidos
        if(banAngle){
            // Crear de rotación
            printf("Crea la tarea \n");
            xTaskCreate(rotationTask, "rotationTask", 1000, NULL, 4, NULL);
            banAngle = false;
        }else if(banDistance){
            // Crear de moverse adelante
            xTaskCreate(moveForwardTask, "moveForwardTask", 1000, NULL, 4, NULL);
            banDistance = false;
        }else if(banCircularMovement){
            // Crear de movimiento circular
            xTaskCreate(circularMovementTask, "circularMovementTask", 1000, NULL, 4, NULL);
            banCircularMovement=false;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency_robotControl);
    }
}

/**
 * @brief Tarea para controlar el driblador del robot.
 *
 * Activa el mecanismo de driblado o disparo en el robot, dependiendo del estado de
 * la variable 'shootBt'.
 *
 * @param pvParameters Parámetros pasados a la tarea; no se utiliza en esta función.
 */
void dribbleTask(void *pvParameters){
     while (1){
        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();       
        if(!shootBt){
            dribble();
        }else{
            kick();
        }       
        vTaskDelayUntil(&xLastWakeTime, xFrequency_dribble);        
    }
}
