#ifndef BT_FUNCTIONS_H
#define BT_FUNCTIONS_H
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "btstack_run_loop.h"
#include "btstack_event.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "ble/gatt-service/nordic_spp_service_server.h"
#include "btstack.h"
#include "robot_movement.h"

extern hci_con_handle_t con_handle;
extern btstack_context_callback_registration_t send_request;
extern btstack_packet_callback_registration_t  hci_event_callback_registration;
extern const uint8_t adv_data_len;
extern const uint8_t adv_data[];

extern double angleBt;
extern double distanceBt;
extern double angleTurnBt;
extern double radioBt;
extern bool banAngle;
extern bool banDistance;
extern bool banCircularMovement;
extern bool btAvailable;

void initBluetooth();
void hci_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
void nordic_spp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
#endif