#include <stdio.h>
#include "robot_movement.h"
#include "bt_functions.h"



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
  // printf("angle 1 %d ",angleMotor1);
  // printf("angle 4 %d ",angleMotor4);
}

bool interruptFlag = false;
void prueba(){
  interruptFlag = true;
}




int main(){
  stdio_init_all();
  gpio_init(4);
  gpio_set_dir(4, GPIO_IN);
  gpio_is_pulled_down(4);
  mpu_init();
  mpu6050_reset();
  // uint8_t interrupt_enable_data[] = {0x38, 0x01}; // Registro INT_ENABLE (dirección 0x38) para habilitar la interrupción de detección de movimiento
  // i2c_write_blocking(MPU6050_i2c,MPU6050_addr, interrupt_enable_data, sizeof(interrupt_enable_data), false);
  // gpio_set_irq_enabled_with_callback(4,GPIO_IRQ_EDGE_RISE,true,&prueba);
  // Configurar la función de interrupció
  static repeating_timer_t timer;
  initI2C();

  getOffsets();
  //initBluetooth();    
  initMotor();
  //add_repeating_timer_us(125,&updateAngle,NULL,&timer);
  offCW1 = 730;
  offCC2 = 0;
  offCW3 = 0;
  offCC4 = 770;
  while (1){

    getAnglesMotors();
    updateAngle();
    //motorsForward();
    distanceMotorsForward();
    pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 770); 

    printf("Vel4 %f\n",velMotor4);
    // if(btAvailable){
    //   continue;
    // }

    // if(banAngle){
    //   rotation(angleBt);
    // }else if(banDistance){
    //   moveForward(distanceBt);
    // }

    // if(banStop){
    //   banAngle=false;
    //   banDistance=false;
    //   btAvailable = true;
    //   banStop = false;
      
    //   }
  }

  return 0;
}                       