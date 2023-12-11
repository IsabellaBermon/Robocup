// IMPLEMENTACIÓN REALIZADA EN EL MONTAJE DEL ROBOT
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <stdint.h>
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "robot_movement.h"
#include "lwipopts.h"
#include "lwip/apps/mqtt.h"
//#include "bt_functions.h"

struct mqtt_connect_client_info_t mqtt_client_info=
{
  "robocupPicoW",
  NULL, /* user */
  NULL, /* pass */
  0,  /* keep alive */
  NULL, /* will_topic */
  NULL, /* will_msg */
  0,    /* will_qos */
  0     /* will_retain */
#if LWIP_ALTCP && LWIP_ALTCP_TLS
  , NULL
#endif
};
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

//   printf("angle 2 %d\n",angleMotor2);
}

void wifiConnect(){

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    
    while(cyw43_arch_wifi_connect_timeout_ms("PocoX3", "jonathan10", CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
        printf("Attempting to connect...\n");
    } 
    printf("Connected.\n");
    

}
void mqtt_sub_request_cb(void *arg, err_t result) {

    if(result == ERR_OK) {
        printf("Suscripción exitosa\n");
    } else {
        printf("Error en suscripción\n");
    }
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
  if(status == MQTT_CONNECT_ACCEPTED) {
      printf("Conexión MQTT establecida\n");

      const char* topic = "picow/posiciones";
      err_t err = mqtt_subscribe(client, topic, 0, mqtt_sub_request_cb, NULL);

      if(err == ERR_OK) {
          printf("Solicitud de suscripción enviada\n");
      } else {
          printf("Error al intentar suscribirse\n");
      }
      
  } else {
      printf("Conexión MQTT fallida\n");
  
  }
}

void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    printf("Mensaje recibido en el tópico: %s, longitud del mensaje: %u\n", topic, tot_len);
    // Aquí puedes preparar para recibir los datos del mensaje
}


void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    printf("Datos del mensaje recibidos: ");
    for(int i = 0; i < len; i++) {
        printf("%c", data[i]);
    }
    printf("\n");
}

void connect_mqtt(){
   
    mqtt_client_t *client = mqtt_client_new();
    if(client != NULL) {

      ip_addr_t ip_addr;
      IP_ADDR4(&ip_addr, 91, 121, 93, 94);


      mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
      err_t err= mqtt_client_connect(client, &ip_addr, MQTT_PORT, &mqtt_connection_cb, 0, &mqtt_client_info);
      if(err != ERR_OK){
        printf("Connect error\n");

      }
      
    }
}

// Function declarations
static void HardwareInit()
{
    //initBluetooth();
    stdio_init_all();
    gpio_init(4);
    gpio_set_dir(4, GPIO_IN);
    gpio_is_pulled_down(4);    
    mpu_init();
    mpu6050_reset();
    initI2C();
    getOffsets();
    initMotor();

    wifiConnect();
    
    connect_mqtt();
}

void sensorReadingTask(void *pvParameters);
// void motorControlTask(void *pvParameters);
void mpuReadingTask(void *pvParameters);
void communicationTask(void *pvParameters);
// void strategyTask(void *pvParameters);

int main(){

  BaseType_t xReturnedA, xReturnedB,xReturnedC;
  HardwareInit();

//   while (!stdio_usb_connected())
//       ;
  sleep_ms(1000);
  mutex = xSemaphoreCreateMutex();
  TaskHandle_t handleA_sensor, handleA_motor, handleA_mpu;
  TaskHandle_t handleB_communication, handleB_strategy;

  // Create tasks on the first core
  xReturnedA = xTaskCreate(sensorReadingTask, "SensorReadingTask", 1000, NULL, 3, &handleA_sensor);
//   vTaskCoreAffinitySet(handleA_sensor, 1 << 0);
  xReturnedB = xTaskCreate(mpuReadingTask, "mpuReadingTask", 1000, NULL, 3, &handleA_mpu);
//   vTaskCoreAffinitySet(handleA_mpu, 1 << 1);
  //xReturnedC = xTaskCreate(communicationTask, "CommunicationTask", 1000, NULL, 3, &handleB_communication);
//   vTaskCoreAffinitySet(handleB_communication, 1 << 1);

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

//   printf("Task A_sensor on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_sensor));
//   printf("Task A_mpu on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleA_mpu));
//   printf("Task B_communication on core %lu\n", (unsigned long)vTaskCoreAffinityGet(handleB_communication));
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
        if(!banStop){
        getAnglesMotors();
        }
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
        if(!banStop){
        updateAngle();
        }
        // if(banAngle=true){
        //     vTaskDelay(pdMS_TO_TICKS(2.8)); // Execute every 2.8 ms
        // }
        // else{
        //     vTaskDelay(pdMS_TO_TICKS(2.2)); // Execute every 2.2 ms
        // }
        vTaskDelay(pdMS_TO_TICKS(2.2)); // Execute every 2.2 ms

        
    }
}

// void communicationTask(void *pvParameters)
// {
//     while (1)
//     {
//         if (btAvailable)
//         {
//             continue;
//         }
//         if (banAngle)
//         {
//             printTaskInfo("Communication Task executed");
//             rotation(angleBt);
//         }
//         else if (banDistance)
//         {
//             xSemaphoreTake(mutex, portMAX_DELAY);
//             printf("Moverse una distancia %f \n",distanceBt);
//             xSemaphoreGive(mutex);            
//             moveForward(distanceBt);
//         }
//         else if(banCircularMovement){
//             circularMovement(radioBt,angleTurnBt);
//         }
//         if (banStop)
//         {
//             banAngle=false;
//             banDistance=false;
//             banCircularMovement=false;
//             btAvailable = true;
//             banStop = false;  
//         }
//         vTaskDelay(pdMS_TO_TICKS(5)); // Execute every 500 ms
//     }
// }

// void strategyTask(void *pvParameters)
// {
//     while (1)
//     {
//         printTaskInfo("Strategy Task executed");
//         vTaskDelay(pdMS_TO_TICKS(100)); // Execute every 1000 ms
//     }
// } 