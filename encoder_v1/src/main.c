#include <stdio.h>
#include "robot_movement.h"
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
  initBluetooth();    

  while (1){
    getAnglesMotors();
    //distanceMotorsClockWise();
    
    if(btAvailable){
      continue;
    }

    if(banAngle){
      rotation(angleBt);
    }else if(banDistance){
      moveForward(distanceBt);
    }

    if(banStop){
      banAngle=false;
      banDistance=false;
      btAvailable = true;
      banStop = false;
    }
    //moveForward(1);
  }

  return 0;
}                       