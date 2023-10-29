#include "motor_config.h"


uint slice_num_5;
uint slice_num_6;

void initMotor(){
  gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
  gpio_set_function(PWM_PIN2, GPIO_FUNC_PWM);
  gpio_set_function(PWM_PIN3, GPIO_FUNC_PWM);
  gpio_set_function(PWM_PIN4, GPIO_FUNC_PWM);


  slice_num_5 = pwm_gpio_to_slice_num(PWM_PIN1); // Find out which PWM slice is connected to GPIO
  pwm_set_clkdiv(slice_num_5, 250.0f); // Set clock freq at 500kHz
  pwm_set_wrap(slice_num_5, 10000);    // Set period of 50Hz (20 ms)
  pwm_set_enabled(slice_num_5, true); // Set the PWM running

  slice_num_6 = pwm_gpio_to_slice_num (PWM_PIN3); // Find out which PWM slice is connected to GPIO 
  pwm_set_clkdiv (slice_num_6, 250.0f); // Set clock freq at 500kHz
  pwm_set_wrap(slice_num_6, 10000); // Set period of 50Hz (20 ms)
  pwm_set_enabled(slice_num_6, true); // Set the PWM running  

  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 650); 
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 650); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 650); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 650); 
  sleep_ms(3000);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 750);  
  sleep_ms(3000);
}