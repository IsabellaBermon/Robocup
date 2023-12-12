/**
 * @file bt_functions.c
 * @brief Funciones relacionadas con la configuración y manejo de Bluetooth.
 *
 * Este archivo contiene funciones para la inicialización y manejo de las comunicaciones Bluetooth,
 * así como la interpretación de paquetes específicos del servicio Nordic SPP.
 */
#include "bt_functions.h"
#include "mygatt.h"
hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
btstack_context_callback_registration_t send_request;
btstack_packet_callback_registration_t  hci_event_callback_registration;

double angleBt = 0;
double distanceBt = 0;
double angleTurnBt = 0;
double radioBt = 0;
bool btAvailable = true;
bool banAngle = false;
bool banCircularMovement=false;
bool banDistance = false;

const uint8_t adv_data[] = {
    // Flags general discoverable, BR/EDR not supported
    2, BLUETOOTH_DATA_TYPE_FLAGS, 0x06, 
    // Name
    8, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'n', 'R', 'F',' ', 'S', 'P', 'P',
    // UUID ...
    17, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, 0x9e, 0xca, 0xdc, 0x24, 0xe, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x1, 0x0, 0x40, 0x6e,
};
const uint8_t adv_data_len = sizeof(adv_data);

/**
 * @brief Inicializa y configura el módulo Bluetooth del dispositivo.
 *
 * Esta función se encarga de iniciar el hardware Bluetooth específico del chip CYW43xx,
 * establecer manejadores de eventos HCI, inicializar los protocolos L2CAP y de Seguridad,
 * configurar y activar la publicidad Bluetooth, y finalmente, encender el controlador HCI.
 * Además, inicia un servicio específico del fabricante (Nordic SPP).
 *
 * @section Funciones Principales
 * - Inicialización del hardware Bluetooth.
 * - Configuración de manejadores de eventos HCI.
 * - Inicialización de protocolos L2CAP y de Seguridad.
 * - Configuración y activación de la publicidad Bluetooth.
 * - Encendido del controlador HCI.
 *
 * @section Uso Previsto
 * Debe ser llamada al inicio del programa para preparar el dispositivo para operaciones Bluetooth,
 * incluyendo la comunicación con otros dispositivos Bluetooth.
 *
 * @return No devuelve nada.
 */
void initBluetooth(){
  if (cyw43_arch_init()) {
    printf("failed to initialise cyw43_arch\n");
  }

  hci_event_callback_registration.callback = &hci_packet_handler;
  hci_add_event_handler(&hci_event_callback_registration);
  l2cap_init();
  sm_init();
  att_server_init(profile_data, NULL, NULL);
  nordic_spp_service_server_init(&nordic_spp_packet_handler);
  uint16_t adv_int_min = 0x0030;
  uint16_t adv_int_max = 0x0030;
  uint8_t adv_type = 0;
  bd_addr_t null_addr;
  memset(null_addr, 0, 6);
  gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
  gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
  gap_advertisements_enable(1);
  hci_power_control(HCI_POWER_ON);
  //btstack_run_loop_execute();
}

/**
 * @brief Manejador de paquetes HCI (Interfaz de Controlador de Host).
 *
 * Esta función se llama cuando se recibe un paquete HCI. Se encarga de procesar
 * los paquetes HCI, específicamente para eventos como la desconexión.
 * 
 * @param packet_type Tipo del paquete recibido.
 * @param channel Canal por el cual se recibió el paquete. No se utiliza en esta función.
 * @param packet Puntero al paquete recibido.
 * @param size Tamaño del paquete recibido. No se utiliza en esta función.
 *
 * @note Esta función ignora todos los paquetes que no son del tipo HCI_EVENT_PACKET.
 *
 * @section Procesamiento
 * - Si el paquete es de tipo HCI_EVENT_PACKET, se procesa según el tipo de evento HCI.
 * - En caso de un evento de desconexión (HCI_EVENT_DISCONNECTION_COMPLETE), se actualiza
 *   el controlador de conexión a un estado inválido.
 */
void hci_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);///< Marca los parámetros no utilizados para evitar advertencias de compilador.
    UNUSED(size);

    ///Ignora paquetes que no son eventos HCI.
    if (packet_type != HCI_EVENT_PACKET) return;
    /// Procesa eventos HCI específicos.
    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            /// Maneja el evento de desconexión completada
            con_handle = HCI_CON_HANDLE_INVALID;
            break;
        default:
            break;
    }
}

/**
 * @brief Maneja paquetes específicos del servicio Nordic SPP (Serial Port Profile).
 *
 * Esta función procesa los paquetes recibidos a través del servicio Nordic SPP,
 * manejando tanto eventos como datos RFCOMM. Se encarga de reconocer y responder a
 * la conexión y desconexión de servicios, así como de procesar y actuar sobre los datos recibidos.
 *
 * @param packet_type Tipo del paquete recibido.
 * @param channel Canal por el cual se recibió el paquete. No se utiliza en esta función.
 * @param packet Puntero al paquete recibido.
 * @param size Tamaño del paquete recibido.
 *
 * @section Procesamiento
 * - Procesa eventos HCI (como la conexión y desconexión del servicio).
 * - Procesa datos RFCOMM, interpretando comandos específicos y ejecutando acciones basadas en ellos.
 *
 * @note Los comandos específicos están indicados por caracteres iniciales ('G', 'D', 'C') y se procesan
 *       para realizar acciones correspondientes, como ajustar ángulos o distancias.
 */
void nordic_spp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
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
      printf("%.*s\n", packet);
      char outputBuffer[256];  // Búfer para almacenar la salida
      char* lista[10];
      // Utiliza snprintf para formatear la cadena y guardarla en outputBuffer
      snprintf(outputBuffer, sizeof(outputBuffer), "%.*s\n", (int)size, (char*)packet);
      // Divide la cadena en tokens separados por comas
      char* token = strtok(outputBuffer, ",");
      int i = 0;
      while (token != NULL){
        lista[i] = token;
        token = strtok(NULL, ",");
        i++;
      }
      // Antes de usar las variables compartidas de Bluetooth
      if(btAvailable){
        if (*lista[0] == 'G') {
          angleBt = atof(lista[1]);
          banAngle = true;
          robotAngle = 0;
          prevAngularPosition = 0;
          angularVelocity= 0;
          angularPosition =0;
          printf("Entra G\n");
        }else if(*lista[0] == 'D'){
          distanceBt = atof(lista[1]);
          banDistance = true;          
          printf("Entra D\n");          
        }else if(*lista[0] == 'C'){
            radioBt = atof(lista[1]);
            angleTurnBt = atof(lista[2]);
            banCircularMovement=true;
            printf("Entra C: %f, %f\n",radioBt,angleTurnBt);
        }    
        btAvailable = false; 
      }
      
    default:
        break;
  }

}