#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include "encoder/encoder.h"




#define S0 2
#define S1 3
#define S2 4



#define SIG_SDA 0
#define SCL 1
#define TCA_ADDR 0x70 

uint16_t angleMotor1 = 0;
uint16_t angleMotor2 = 0;
uint16_t angleMotor3 = 0;


void tca_select_channel(uint8_t channel){
    uint8_t data = 1 << channel;
    i2c_write_blocking(i2c0, TCA_ADDR, &data, 1, false);
}



void writeBinaryToPins(uint8_t number){
  gpio_put(S0, (number & 0x1));
  gpio_put(S1, (number & 0x2) >> 1);
  gpio_put(S2, (number & 0x4) >> 2);

}

int16_t getAngle()
{
  int16_t raw;
  readAngleRaw(&raw);
  return raw * 360 / 0xFFF;
}
bool timer_callback(repeating_timer_t *t){
  // currentSDA = (currentSDA + 1)%3;
  tca_select_channel(0);
  angleMotor1 = getAngle();
  tca_select_channel(1);

  angleMotor2 = getAngle();
  tca_select_channel(2);
  angleMotor3 = getAngle();  
  return true;
}


int main(){
  stdio_init_all();

  static repeating_timer_t timer;

  i2c_init(AS560_i2c,400000);
  gpio_set_function(SIG_SDA,GPIO_FUNC_I2C);
  gpio_set_function(SCL,GPIO_FUNC_I2C);
  gpio_pull_up(SIG_SDA);
  gpio_pull_up(SCL);

  add_repeating_timer_us(2000,&timer_callback,NULL,&timer);



  while (1){  
    printf("Motor 1 %d ",angleMotor1);
    printf("Motor 2 %d ",angleMotor2);
    printf("Motor 3 %d\n",angleMotor3);
  }
  return 0;
}                                                         