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
  tca_select_channel(4);

}

// Function declarations
static void HardwareInit()
{
    //initBluetooth();
    stdio_init_all();
    //gpio_init(4);
    //gpio_set_dir(4, GPIO_IN);
    //gpio_is_pulled_down(4);    
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
  // stdio_init_all();
  // mpu_init();
  // mpu6050_reset();
  // initI2C();
  // getOffsets();
  // //initBluetooth();    
  // initMotor();

  while (!stdio_usb_connected())
      ;
  sleep_ms(1000);
  mutex = xSemaphoreCreateMutex();
  TaskHandle_t handleA_sensor, handleA_motor, handleA_mpu;
  TaskHandle_t handleB_communication, handleB_strategy;

  // Create tasks on the first core
  xReturnedA = xTaskCreate(sensorReadingTask, "SensorReadingTask", 1000, NULL, 3, &handleA_sensor);
  vTaskCoreAffinitySet(handleA_sensor, 1 << 0);
  // tast2 = xTaskCreate(motorControlTask, "MotorControlTask", 1000, NULL, 3, &handleA_motor);
  // vTaskCoreAffinitySet(handleA_motor, 1 << 0);
  xReturnedB = xTaskCreate(mpuReadingTask, "mpuReadingTask", 1000, NULL, 3, &handleA_mpu);
  vTaskCoreAffinitySet(handleA_mpu, 1 << 0);
  // Create tasks on the second core
  xReturnedC = xTaskCreate(communicationTask, "CommunicationTask", 1000, NULL, 3, &handleB_communication);
  vTaskCoreAffinitySet(handleB_communication, 1 << 1);
  // tast5 = xTaskCreate(strategyTask, "StrategyTask", 1000, NULL, 2, &handleB_strategy);    
  // vTaskCoreAffinitySet(handleB_strategy, 1 << 1);

  // Check return values for errors
  if (xReturnedA != pdPASS || xReturnedB != pdPASS)
  {
      printf("Error creating tasks\n");
      panic_unsupported();
  }

  printf("Task A_sensor on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_sensor));
  // printf("Task A_motor on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_motor));
  printf("Task A_mpu on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_mpu));
  // printf("Task B_communication on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleB_communication));
  // printf("Task B_strategy on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleB_strategy));

  // Start FreeRTOS scheduler
  vTaskStartScheduler();
  panic_unsupported();

  // while (1){

  //   getAnglesMotors();
  //   updateAngle();
  //   //pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 780); // 780
  //   moveForward(1.5);
  //   // printf("vel1 %f ",velMotor1);
  //   printf("vel1 %d\n",velMotor1);
  //   // printf("ang %f\n",robotAngle);
  //   // if(btAvailable){
  //   //   continue;
  //   // }

  //   // if(banAngle){
  //   //   rotation(angleBt);
  //   // }else if(banDistance){
  //   //   moveForward(distanceBt);
  //   // }

  //   // if(banStop){
  //   //   banAngle=false;
  //   //   banDistance=false;
  //   //   btAvailable = true;
  //   //   banStop = false;
      
  //   //   }
  // }

  // return 0;
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
        // printTaskInfo("MPU Reading Task executed");
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
        // printTaskInfo("Communication Task executed");
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
        vTaskDelay(pdMS_TO_TICKS(5)); // Execute every 500 ms
    }
}

// void strategyTask(void *pvParameters)
// {
//     while (1)
//     {
//         printTaskInfo("Strategy Task executed");
//         vTaskDelay(pdMS_TO_TICKS(100)); // Execute every 1000 ms
//     }
// }