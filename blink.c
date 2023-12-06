// Implementación BLE.
// Barcelona FC
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "btstack_run_loop.h"
#include "pico/stdlib.h"
#include "btstack_event.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "hal_led.h"
#include "btstack.h"
#include "ble/gatt-service/nordic_spp_service_server.h"
#include "mygatt.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"

// Inclusión de bibliotecas estándar de C y bibliotecas relacionadas con Bluetooth y PWM.

// Variables globales para manejar el identificador de conexión y registros de Bluetooth.
static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static btstack_context_callback_registration_t send_request;
static btstack_packet_callback_registration_t  hci_event_callback_registration;

// Variables para identificar el slice (bloque de PWM) y el canal de salida.
uint slice, channel;

// Función para detener el carro
void stop(){
    pwm_set_chan_level(slice, channel, 750);
    gpio_put(12, 0);
    gpio_put(14, 0);
    gpio_put(15, 0);
    printf("Stop");
}

// Función para mover el carro hacia adelante
void forward(){
    gpio_put(12, 1);
    pwm_set_chan_level(slice, channel, 1000);
    printf("Forward");
}

// Función para mover el carro hacia atrás
void reverse(){
    gpio_put(12, 0);
    pwm_set_chan_level(slice, channel, 500);
    printf("Reverse");
}

// Función para mover el carro hacia adelante y a la izquierda
void forward_left(){
    gpio_put(14, 1);
    gpio_put(15, 0);
    forward();
    printf("Forward_left");
}

// Función para mover el coche hacia adelante y a la derecha
void forward_right(){
    gpio_put(14, 0);
    gpio_put(15, 1);
    forward();
    printf("Forward_right");
}

// Datos de anuncio Bluetooth (nombre del dispositivo y UUID del servicio)
const uint8_t adv_data[] = {
    // Flags general discoverable, BR/EDR not supported
    2, BLUETOOTH_DATA_TYPE_FLAGS, 0x06, 
    // Name
    8, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'n', 'R', 'F',' ', 'S', 'P', 'P',
    // UUID ...
    17, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, 0x9e, 0xca, 0xdc, 0x24, 0xe, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x1, 0x0, 0x40, 0x6e,
};
const uint8_t adv_data_len = sizeof(adv_data);

// Manejo de paquetes Bluetooth HCI
static void hci_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            con_handle = HCI_CON_HANDLE_INVALID;
            break;
        default:
            break;
    }
}

// Manejo de paquetes para el servicio GATT (Generic Attribute Profile) SPP (Serial Port Profile)
static void nordic_spp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    switch (packet_type){
        case HCI_EVENT_PACKET:
            if (hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META) break;
            switch (hci_event_gattservice_meta_get_subevent_code(packet)){
                case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED:
                    con_handle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
                    printf("Connected with handle 0x%04x\n", con_handle);
                    break;
                case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED:
                    con_handle = HCI_CON_HANDLE_INVALID;
                    break;
                default:
                    break;
            }
            break;
        case RFCOMM_DATA_PACKET:
            // Manejo de datos recibidos a través del servicio SPP
            // Aquí se procesan y se imprimen los datos según el formato esperado
            // (puede requerir ajustes según la estructura real de los datos)
            printf("%.*s\n", packet);
            
            // Procesamiento adicional de los datos recibidos, para reconocimiento de giro 
            char outputBuffer[256];
            char* lista[10];
            snprintf(outputBuffer, sizeof(outputBuffer), "%.*s\n", (int)size, (char*)packet);
            char* token = strtok(outputBuffer, ",");
            int i = 0;
            while (token != NULL) {
                lista[i] = token;
                token = strtok(NULL, ",");
                i++;
            }
            for (int j = 0; j < i; j++) {
                printf("Elemento %d: %s\n", j, lista[j]);
            }
            if (lista[0][0] == 'G') {
                float angulo = atof(lista[1]);
                uint8_t valorStr[20];
                sprintf(valorStr, "%s", lista[0]);
                printf("Estoy girando y este es mi giro: %f, %s\n", angulo, valorStr);
            }
            else {
                float valor = atof(lista[1]);
                uint8_t mono[20];
                sprintf(mono, "%s", lista[0]);
                printf("Estoy: %f, %s\n", valor, mono);
            }
            break;
        default:
            break;
    }
}

int main() {
    // Configuración inicial del hardware y variables
    stdio_init_all();
    gpio_init(12);
    gpio_init(14);
    gpio_init(15);
    gpio_set_dir(12, GPIO_OUT);
    gpio_set_dir(14, GPIO_OUT);
    gpio_set_dir(15, GPIO_OUT);
    gpio_set_function(12, GPIO_FUNC_PWM);

    // Obtener el número de slice y canal PWM a partir del pin GPIO 12
    slice = pwm_gpio_to_slice_num(12);
    channel = pwm_gpio_to_channel(12);

    // Configuración del bloque PWM
    pwm_set_clkdiv(slice, 250.0f);  // Ralentizar el reloj
    pwm_set_wrap(slice, 10000);      // Tiempo de envoltura    
    pwm_set_enabled(slice, true);

    // Detener el carro por un breve período al inicio
    sleep_ms(100);
    stop();

    // Inicialización del controlador CYW43 (Bluetooth)
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    // Registro de devolución de llamada para manejar eventos HCI (Host Controller Interface)
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Inicialización de servicios Bluetooth
    l2cap_init();
    sm_init();
    att_server_init(profile_data, NULL, NULL);
    nordic_spp_service_server_init(&nordic_spp_packet_handler);

    // Configuración de parámetros de anuncio Bluetooth
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
    gap_advertisements_enable(1);

    // Encender el controlador Bluetooth
    hci_power_control(HCI_POWER_ON);

    // Ejecutar el bucle de eventos de Bluetooth
    btstack_run_loop_execute();

    return 0;
}

