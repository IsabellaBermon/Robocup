#include <stdio.h>
#include "robot_movement.h"
#include "mygatt.h"
#include "bt_functions.h"



bool timer_callback(repeating_timer_t *t){
  tca_select_channel(0);
  angleMotor1 = angleSubtraction(getAngle(),offsetAngleMotor1);
  tca_select_channel(1);
  angleMotor2 = angleSubtraction(getAngle(),offsetAngleMotor2);
  tca_select_channel(2);
  angleMotor3 = angleSubtraction(getAngle(),offsetAngleMotor3);
  tca_select_channel(3);
  angleMotor4 = angleSubtraction(getAngle(),offsetAngleMotor4);
  return true;
}

void restartRobot(){
  restartControl();
  restartMovement();
  getOffsets();
}


void getAnglesMotors(){
  tca_select_channel(0);
  angleMotor1 = angleSubtraction(getAngle(),offsetAngleMotor1);
  tca_select_channel(1);
  angleMotor2 = angleSubtraction(getAngle(),offsetAngleMotor2);
  tca_select_channel(2);
  angleMotor3 = angleSubtraction(getAngle(),offsetAngleMotor3);
  tca_select_channel(3);
  angleMotor4 = angleSubtraction(getAngle(),offsetAngleMotor4);
}




int main(){
  stdio_init_all();

  static repeating_timer_t timer;
  initI2C();
  getOffsets();
  initMotor();
  //add_repeating_timer_us(200,&timer_callback,NULL,&timer);
      
  if (cyw43_arch_init()) {
    printf("failed to initialise cyw43_arch\n");
    return -1;
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
  btstack_run_loop_execute();

  while (1){
    getAnglesMotors();
    rotation(45);
    //moveForward(1);
  }

  return 0;
}                       