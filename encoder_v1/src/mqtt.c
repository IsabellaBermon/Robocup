
#include "mqtt.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int positions[9];
bool banPositions =false;
void mqtt_sub_request_cb(void *arg, err_t result) {

    if(result == ERR_OK) {
        printf("Suscripción exitosa\n");
    } else {
        printf("Error en suscripción\n");
    }
}

void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
  if(status == MQTT_CONNECT_ACCEPTED) {
      printf("Conexión MQTT establecida\n");

      //const char* topic = "picow/posiciones";
      err_t err = mqtt_subscribe(client, TOPIC, 0, mqtt_sub_request_cb, NULL);

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
    
}


void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {

     // Copia de la cadena de datos para no modificar la original
    char buffer[256]; // Asegúrate de que este buffer sea lo suficientemente grande
    strncpy(buffer, (const char *)data, sizeof(buffer));
    buffer[sizeof(buffer) - 1] = '\0'; // Asegurando que la cadena esté terminada en null

    // Dividir la cadena en tokens usando la coma como delimitador
    char *token = strtok(buffer, ",");
    int i = 0;
    while (token != NULL) {
        // Convertir el token a un número entero
        positions[i] = atoi(token);
        i++;
        // Obtener el siguiente token
        token = strtok(NULL, ",");
    }
    banPositions = true;
}

void connect_mqtt(){
    struct mqtt_connect_client_info_t ci;
    mqtt_client_t *client = mqtt_client_new();
    if(client != NULL) {

      ip_addr_t ip_addr;
      IP_ADDR4(&ip_addr, 91, 121, 93, 94);
      memset(&ci, 0, sizeof(ci));
      ci.client_id ="robocupPicoW";


      mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
      err_t err= mqtt_client_connect(client, &ip_addr, MQTT_PORT, &mqtt_connection_cb, 0, &ci);
      if(err != ERR_OK){
        printf("Connect error\n");

      }
      
    }
}