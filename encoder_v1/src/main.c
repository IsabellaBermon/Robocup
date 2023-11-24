#include "FreeRTOS.h"
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
  // tca_select_channel(4);
  // updateAngle();
  // // printf("angle 1 %d ",angleMotor1);
  // printf("angle 4 %d\n",angleMotor4);
}


// Function declarations
static void HardwareInit();
void sensorReadingTask(void *pvParameters);
void motorControlTask(void *pvParameters);
void mpuReadingTask(void *pvParameters);
void communicationTask(void *pvParameters);
void strategyTask(void *pvParameters);

int main(void)
{
    BaseType_t xReturnedA, xReturnedB;
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
    xReturnedA = xTaskCreate(motorControlTask, "MotorControlTask", 1000, NULL, 3, &handleA_motor);
    vTaskCoreAffinitySet(handleA_motor, 1 << 0);
    xReturnedA = xTaskCreate(mpuReadingTask, "mpuReadingTask", 1000, NULL, 3, &handleA_mpu);
    vTaskCoreAffinitySet(handleA_mpu, 1 << 0);
    // Create tasks on the second core
    xReturnedB = xTaskCreate(communicationTask, "CommunicationTask", 1000, NULL, 1, &handleB_communication);
    vTaskCoreAffinitySet(handleB_communication, 1 << 1);
    xReturnedB = xTaskCreate(strategyTask, "StrategyTask", 1000, NULL, 2, &handleB_strategy);
     // Set core affinity
   
    
    
    
    vTaskCoreAffinitySet(handleB_strategy, 1 << 1);
    // Check return values for errors
    if (xReturnedA != pdPASS || xReturnedB != pdPASS)
    {
        printf("Error creating tasks\n");
        panic_unsupported();
    }

    printf("Task A_sensor on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_sensor));
    printf("Task A_motor on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_motor));
    printf("Task A_mpu on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_mpu));
    printf("Task B_communication on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleB_communication));
    printf("Task B_strategy on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleB_strategy));

   

    // Start FreeRTOS scheduler
    vTaskStartScheduler();
    panic_unsupported();
}

static void HardwareInit()
{
    // set_sys_clock_48mhz();
    // stdio_init_all();
    // gpio_init(PICO_DEFAULT_LED_PIN);
    // gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    // gpio_put(PICO_DEFAULT_LED_PIN, true);
    stdio_init_all();
    gpio_init(4);
    gpio_set_dir(4, GPIO_IN);
    gpio_is_pulled_down(4);
    mpu_init();
    mpu6050_reset();

    initI2C();

    getOffsets();
    // initBluetooth();
    initMotor();
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
    // while (1) {
    //     printTaskInfo("Sensor Reading Task");
    //     vTaskDelay(pdMS_TO_TICKS(1)); // Execute every 1 ms
    // }
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // Desired interval: 1 ms
    // Initialize the xLastWakeTime variable with the current time
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        printTaskInfo("Sensor Reading Task");
        // Wait until it's time to run again
        getAnglesMotors();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void motorControlTask(void *pvParameters)
{
    while (1)
    {
        printTaskInfo("Motor Control Task executed");
        vTaskDelay(pdMS_TO_TICKS(50)); // Execute every 50 ms
    }
}

void mpuReadingTask(void *pvParameters)
{
    while (1)
    {
        printTaskInfo("MPU Reading Task executed");
        updateAngle();
        vTaskDelay(pdMS_TO_TICKS(5)); // Execute every 5 ms
    }
}

void communicationTask(void *pvParameters)
{
    while (1)
    {
        printTaskInfo("Communication Task executed");
        if (btAvailable)
        {
            continue;
        }
        if (banAngle)
        {
            rotation(angleBt);
        }
        else if (banDistance)
        {
            moveForward(distanceBt);
        }

        if (banStop)
        {
            banAngle = false;
            banDistance = false;
            btAvailable = true;
            banStop = false;
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Execute every 500 ms
    }
}

void strategyTask(void *pvParameters)
{
    while (1)
    {
        printTaskInfo("Strategy Task executed");
        vTaskDelay(pdMS_TO_TICKS(100)); // Execute every 1000 ms
    }
}
