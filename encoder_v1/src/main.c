#include <stdio.h>
#include "robot_movement.h"
#include "control_functions.h"
#include "motor_config.h"
#include "encoder.h"
#include "hardware/i2c.h"


#define SIG_SDA 0
#define SCL 1
#define TCA_ADDR 0x70 


uint16_t basePWM = 780;



uint16_t offsetAngleMotor1 = 0;
uint16_t offsetAngleMotor2 = 0;
uint16_t offsetAngleMotor3 = 0;
uint16_t offsetAngleMotor4 = 0;




// Variables para almacenar errores acumulados y error anterior




void tca_select_channel(uint8_t channel){
    uint8_t data = 1 << channel;
    i2c_write_blocking(i2c0, TCA_ADDR, &data, 1, false);
}

int16_t getAngle()
{
  int16_t raw;
  readAngleRaw(&raw);
  return raw * 360 / 0xFFF;
}

int16_t angleSubtraction(int16_t angle, int16_t angleOffset)
{
  int16_t angleSub = angle - angleOffset;
  if (angleSub < 0)
  {
    return 360 + angleSub;
  }
  else
  {
    return angleSub;
  }
}
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




void getOffsets(){
  tca_select_channel(0);
  offsetAngleMotor1 = getAngle();
  tca_select_channel(1);
  offsetAngleMotor2 = getAngle();
  tca_select_channel(2);
  offsetAngleMotor3 = getAngle(); 
  tca_select_channel(3);
  offsetAngleMotor4 = getAngle();
}






int main(){
  stdio_init_all();

  static repeating_timer_t timer;

  i2c_init(AS560_i2c,400000);
  gpio_set_function(SIG_SDA,GPIO_FUNC_I2C);
  gpio_set_function(SCL,GPIO_FUNC_I2C);
  gpio_pull_up(SIG_SDA);
  gpio_pull_up(SCL);

  getOffsets();
  initMotor();
  //add_repeating_timer_us(200,&timer_callback,NULL,&timer);

  while (1){
    tca_select_channel(0);
    angleMotor1 = angleSubtraction(getAngle(),offsetAngleMotor1);
    tca_select_channel(1);
    angleMotor2 = angleSubtraction(getAngle(),offsetAngleMotor2);
    tca_select_channel(2);
    angleMotor3 = angleSubtraction(getAngle(),offsetAngleMotor3);
    tca_select_channel(3);
    angleMotor4 = angleSubtraction(getAngle(),offsetAngleMotor4);
    rotation(180);
    //moveForward(1);

  }

  return 0;
}                       