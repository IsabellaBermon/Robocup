// IMPLEMENTACIÓN SIMULADA DE LAS 5 TAREAS 
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "robot_movement.h"
#include "bt_functions.h"

const int taskDelay = 100;
int randomNumber_encoder;
int randomNumber_mpu;
SemaphoreHandle_t mutex;
SemaphoreHandle_t sensorSemaphore;
SemaphoreHandle_t mpuSemaphore;
SemaphoreHandle_t resource;

const TickType_t xFrequency_sensor = pdMS_TO_TICKS(2);    
const TickType_t xFrequency_motor = pdMS_TO_TICKS(3);      
const TickType_t xFrequency_mpu = pdMS_TO_TICKS(2);      
const TickType_t xFrequency_communication = pdMS_TO_TICKS(5); 
const TickType_t xFrequency_strategy = pdMS_TO_TICKS(5);   

// Function declarations
static void HardwareInit();
void sensorReadingTask(void *pvParameters);
void motorControlTask(void *pvParameters);
void mpuReadingTask(void *pvParameters);
void communicationTask(void *pvParameters);
void strategyTask(void *pvParameters);

int main(void)
{
    BaseType_t xReturnedA, xReturnedB, xReturnedC, xReturnedD,xReturnedE;
    HardwareInit();
    while (!stdio_usb_connected())
        ;
    sleep_ms(1000);
    TaskHandle_t handleA_sensor, handleA_motor, handleA_mpu;
    TaskHandle_t handleB_communication, handleB_strategy;

    resource = xSemaphoreCreateMutex();
    mutex = xSemaphoreCreateMutex();
    sensorSemaphore = xSemaphoreCreateBinary();
    mpuSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(sensorSemaphore);  // Inicializar el semáforo en estado "libre"
    xSemaphoreGive(mpuSemaphore);  // Inicializar el semáforo en estado "libre"

    // Create tasks on the first core
    xReturnedA = xTaskCreate(sensorReadingTask, "SensorReadingTask", 1000, NULL, 4, &handleA_sensor);
    xReturnedB = xTaskCreate(motorControlTask, "MotorControlTask", 1000, NULL, 5, &handleA_motor);
    xReturnedC = xTaskCreate(mpuReadingTask, "mpuReadingTask", 1000, NULL, 4, &handleA_mpu);
    xReturnedD = xTaskCreate(communicationTask, "CommunicationTask", 1000, NULL, 3, &handleB_communication);
    xReturnedE = xTaskCreate(strategyTask, "StrategyTask", 1000, NULL, 3, &handleB_strategy);
    printf("Task A_sensor on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_sensor));
    printf("Task A_motor on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_motor));
    printf("Task A_mpu on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_mpu));
    printf("Task B_communication on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleB_communication));
    printf("Task B_strategy on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleB_strategy));

    // Start FreeRTOS scheduler
    vTaskStartScheduler();
    panic_unsupported();

    // // Check return values for errors
    // if (xReturnedA != pdPASS || xReturnedB != pdPASS)
    // {
    //     printf("Error creating tasks\n");
    //     panic_unsupported();
    // }
}

static void HardwareInit()
{
    // set_sys_clock_48mhz();
    set_sys_clock_khz(100000, true);  // Establecer el reloj a 100 MHz, por ejemplo
    stdio_init_all();
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
    TickType_t currentTime = xTaskGetTickCount();
    sprintf(out, "%s running on Core %d, Time: %lu ms", taskName, get_core_num(), currentTime * portTICK_PERIOD_MS);
    vSafePrint(out);
}

void sensorReadingTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {        
        // Esperar hasta que sea seguro leer el sensor
        if (xSemaphoreTake(sensorSemaphore, portMAX_DELAY) == pdTRUE)
        {            
            printTaskInfo("Sensor Reading Task");
            // Realizar la lectura del sensor
            randomNumber_encoder = rand() % 101;

            // Notificar que la lectura del sensor está completa
            xSemaphoreGive(sensorSemaphore);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency_sensor);
    }
}

void mpuReadingTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {        
        // Esperar hasta que sea seguro leer el MPU
        if (xSemaphoreTake(mpuSemaphore, portMAX_DELAY) == pdTRUE)
        {            
            printTaskInfo("MPU Reading Task executed");
            // Realizar la lectura del MPU
            randomNumber_mpu = rand() % 101;
            // Notificar que la lectura del MPU está completa
            xSemaphoreGive(mpuSemaphore);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency_mpu);
    }
}

void motorControlTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // Intentar tomar ambos semáforos
        BaseType_t sensorTaken = xSemaphoreTake(sensorSemaphore, portMAX_DELAY);
        BaseType_t mpuTaken = xSemaphoreTake(mpuSemaphore, portMAX_DELAY);
        if (sensorTaken == pdTRUE && mpuTaken == pdTRUE)
        {
            printTaskInfo("Motor Control Task executed");
            // Realizar el control del motor
            int error_PID = randomNumber_encoder*randomNumber_mpu;
        }
        // Liberar el semáforo del sensor si se tomó correctamente
        if (sensorTaken == pdTRUE) { xSemaphoreGive(sensorSemaphore); }
        // Liberar el semáforo del MPU si se tomó correctamente
        if (mpuTaken == pdTRUE) { xSemaphoreGive(mpuSemaphore); }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency_motor);
    }
}


void communicationTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {   
        // xSemaphoreTake(resource, portMAX_DELAY);
        printTaskInfo("Communication Task executed");
        // xSemaphoreGive(resource);
        vTaskDelayUntil(&xLastWakeTime, xFrequency_communication );
    }
}

void strategyTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // xSemaphoreTake(resource, portMAX_DELAY);
        printTaskInfo("Strategy Task executed");
        // printf("Strategy Task: Priority %d\n", uxTaskPriorityGet(NULL));
        // xSemaphoreGive(resource);
        vTaskDelayUntil(&xLastWakeTime, xFrequency_strategy );
    }
}

// IMPLEMENTACIÓN REALIZADA EN EL MONTAJE DEL ROBOT
/* #include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "robot_movement.h"
#include "bt_functions.h"


SemaphoreHandle_t mutex;
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
  printf("Wheel angle 1 %d\n ",angleMotor1);
  printf("Robot Angle: %f \n",robotAngle);
//   printf("angle 2 %d\n",angleMotor2);

}

// Function declarations
static void HardwareInit()
{
    initBluetooth();
    stdio_init_all();
    gpio_init(4);
    gpio_set_dir(4, GPIO_IN);
    gpio_is_pulled_down(4);    
    mpu_init();
    mpu6050_reset();
    initI2C();
    getOffsets();
    initMotor();
}

void sensorReadingTask(void *pvParameters);
// void motorControlTask(void *pvParameters);
void mpuReadingTask(void *pvParameters);
void communicationTask(void *pvParameters);
// void strategyTask(void *pvParameters);

int main(){

  BaseType_t xReturnedA, xReturnedB,xReturnedC;
  HardwareInit();

  while (!stdio_usb_connected())
      ;
  sleep_ms(1000);
  mutex = xSemaphoreCreateMutex();
  TaskHandle_t handleA_sensor, handleA_motor, handleA_mpu;
  TaskHandle_t handleB_communication, handleB_strategy;

  // Create tasks on the first core
  xReturnedA = xTaskCreate(sensorReadingTask, "SensorReadingTask", 1000, NULL, 3, &handleA_sensor);
  vTaskCoreAffinitySet(handleA_sensor, 1 << 0);
  xReturnedB = xTaskCreate(mpuReadingTask, "mpuReadingTask", 1000, NULL, 3, &handleA_mpu);
  vTaskCoreAffinitySet(handleA_mpu, 1 << 1);
  xReturnedC = xTaskCreate(communicationTask, "CommunicationTask", 1000, NULL, 3, &handleB_communication);
  vTaskCoreAffinitySet(handleB_communication, 1 << 1);
  // tast2 = xTaskCreate(motorControlTask, "MotorControlTask", 1000, NULL, 3, &handleA_motor);
  // vTaskCoreAffinitySet(handleA_motor, 1 << 0);  
  // tast5 = xTaskCreate(strategyTask, "StrategyTask", 1000, NULL, 2, &handleB_strategy);    
  // vTaskCoreAffinitySet(handleB_strategy, 1 << 1);

  // Check return values for errors
  if (xReturnedA != pdPASS || xReturnedB != pdPASS)
  {
      printf("Error creating tasks\n");
      panic_unsupported();
  }

  printf("Task A_sensor on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_sensor));
  printf("Task A_mpu on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_mpu));
  printf("Task B_communication on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleB_communication));
  // printf("Task A_motor on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_motor));  
  // printf("Task B_strategy on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleB_strategy));

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

void sensorReadingTask(void *pvParameters)
{
    while (1)
    {
        // printTaskInfo("Encoders Reading Task executed");
        getAnglesMotors();
        vTaskDelay(pdMS_TO_TICKS(1)); // Execute every 5 ms
    }
}

// void motorControlTask(void *pvParameters)
// {
//     while (1)
//     {
//         printTaskInfo("Motor Control Task executed");
//         vTaskDelay(pdMS_TO_TICKS(50)); // Execute every 50 ms
//     }
// }

void mpuReadingTask(void *pvParameters)
{
    while (1)
    {
        // printTaskInfo("MPU Reading Task executed");
        updateAngle();
        vTaskDelay(pdMS_TO_TICKS(2.8)); // Execute every 5 ms
    }
}

void communicationTask(void *pvParameters)
{
    while (1)
    {
        
        if (btAvailable)
        {
            continue;
        }
        if (banAngle)
        {
            printTaskInfo("Communication Task executed");
            rotation(angleBt);
        }
        else if (banDistance)
        {
            xSemaphoreTake(mutex, portMAX_DELAY);
            printf("Moverse una distancia %f \n",distanceBt);
            xSemaphoreGive(mutex);            
            moveForward(1.5);
        }

        if (banStop)
        {
            banAngle = false;
            banDistance = false;
            btAvailable = true;
            banStop = false;
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Execute every 500 ms
    }
}

void strategyTask(void *pvParameters)
{
    while (1)
    {
        printTaskInfo("Strategy Task executed");
        vTaskDelay(pdMS_TO_TICKS(100)); // Execute every 1000 ms
    }
} */
