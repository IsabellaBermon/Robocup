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

SemaphoreHandle_t mutex;
SemaphoreHandle_t sensorSemaphore;
SemaphoreHandle_t mpuSemaphore;
TaskHandle_t xTaskToStop = NULL;

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
}

void sensorReadingTask(void *pvParameters);
void mpuReadingTask(void *pvParameters);
void communicationTask(void *pvParameters);
void robotControlTask(void *pvParameters);
void dribbleTask(void *pvParameters);

const TickType_t xFrequency_sensor = pdMS_TO_TICKS(2.8);     
const TickType_t xFrequency_mpu = pdMS_TO_TICKS(2.2);      
const TickType_t xFrequency_communication = pdMS_TO_TICKS(5); 
const TickType_t xFrequency_movement = pdMS_TO_TICKS(2.8); 
const TickType_t xFrequency_robotControl = pdMS_TO_TICKS(2.8); 

int main(){

  BaseType_t xReturnedA, xReturnedB,xReturnedC,xReturnD,xReturnE;
  HardwareInit();

  mutex = xSemaphoreCreateMutex();
  sensorSemaphore = xSemaphoreCreateBinary();
  mpuSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sensorSemaphore);  // Inicializar el semáforo en estado "libre"
  xSemaphoreGive(mpuSemaphore);  // Inicializar el semáforo en estado "libre"

  // Create tasks on the first core
  xReturnedA = xTaskCreate(sensorReadingTask, "SensorReadingTask", 1000, NULL, 4, NULL);
  xReturnedB = xTaskCreate(mpuReadingTask, "mpuReadingTask", 1000, NULL, 4, NULL);
  xReturnedC = xTaskCreate(communicationTask, "CommunicationTask", 1000, NULL, 3, NULL);
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

void vSafePrint(char *out)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    puts(out);
    xSemaphoreGive(mutex);
}

void printTaskInfo(const char *taskName)
{
    char out[128];
    sprintf(out, "%s running on Core %d, Time: %lu ms", taskName, get_core_num(), xTaskGetTickCount());
    vSafePrint(out);
}

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
                xSemaphoreTake(mutex, portMAX_DELAY);
                getAnglesMotors();
                xSemaphoreGive(mutex); 
                // Notificar que la lectura del sensor está completa
                xSemaphoreGive(sensorSemaphore);
            }        
        }
        vTaskDelay(pdMS_TO_TICKS(2.8)); 
    }
}

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
                xSemaphoreTake(mutex, portMAX_DELAY);
                updateAngle();
                // printf("Angulo calculado %f \n",robotAngle);
                // Liberar el semáforo binario después de actualizar los ángulos
                xSemaphoreGive(mutex); 
                // Notificar que la lectura del sensor está completa
                xSemaphoreGive(mpuSemaphore);
            }      
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency_mpu);       
    }
}

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
            printf("Angulo calculado %f \n",robotAngle);
            // Realizar rotación del robot
            rotation(angleBt);
        }
        // Liberar el semáforo del MPU si se tomó correctamente
        if (mpuTaken == pdTRUE) { xSemaphoreGive(mpuSemaphore); }

        vTaskDelayUntil(&xLastWakeTime, xFrequency_movement);
        
    }
}

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
void dribbleTask(void *pvParameters){

     while (1){
        // Comprobar la bandera para detener la tarea
        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();
     
        // printTaskInfo("Motor Control Task executed");
        // Realizar el control del motor
        if(!shootBt){
            dribble();
        }else{
            kick();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency_movement);        
    }
}
void communicationTask(void *pvParameters)
{
    while (1)
    {
        TickType_t xLastWakeTime;
        xLastWakeTime = xTaskGetTickCount();
        btstack_run_loop_execute();
        vTaskDelayUntil(&xLastWakeTime, xFrequency_communication);

    }
}
