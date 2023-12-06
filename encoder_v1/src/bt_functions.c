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


void hci_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return
    ;

    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            con_handle = HCI_CON_HANDLE_INVALID;
            break;
        default:
            break;
    }
}
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
      if(btAvailable){
        if (*lista[0] == 'G') {
          angleBt = atof(lista[1]);
          banAngle = true;
          robotAngle = 0;
          prevAngularPosition = 0;
          angularVelocity= 0;
          angularPosition =0;
          printf("Entra G\n");
          // uint8_t valorStr[20];  // Búfer para almacenar el valor en formato de cadena
          // sprintf(valorStr, "%s", lista[0]);  // Convierte el valor en la posición 0 en una cadena
          // printf("Estoy girando y este es mi giro: %f, %s\n", angulo, valorStr);
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