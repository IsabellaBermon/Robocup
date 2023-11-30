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
  tca_select_channel(4);

}

int main(){
  stdio_init_all();
  mpu_init();
  mpu6050_reset();
  initI2C();
  getOffsets();
  //initBluetooth();    
  initMotor();


  while (1){

    getAnglesMotors();
    updateAngle();
   
    moveForward(5);
    printf("vel1 %f ",velMotor1);
    printf("vel3 %f ",velMotor3);
    printf("ang %f\n",robotAngle);
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