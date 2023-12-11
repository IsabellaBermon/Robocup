#include "lwip/apps/mqtt.h"

#include <string.h>
#define SSID "PocoX3"
#define PASSWORD "jonathan10"
#define TOPIC "robot/posiciones"


void mqtt_sub_request_cb(void *arg, err_t result);
void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
void connect_mqtt();