#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "encoder.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include <math.h>
#include "pico/stdlib.h"

// Definición de constantes
#define SDA 0
#define SCL 1
#define SDA1 2
#define SCL1 3
#define S0 24
#define S1 25
#define S2 26
#define S3 27
#define Sig_out 6
#define PWM_PIN1 10        // pin 14   SLICE 5 CH A
#define PWM_PIN2 11        // pin 15   SLICE 5 CH B
#define finalDistance 1    // en metros
#define perimeter 0.037698 // perimetro en metros

float distance0 = 0;
float distance1 = 0;
bool banTurns0 = true;
bool banTurns1 = true;

uint16_t turns0 = 0;
uint16_t turns1 = 0;
int16_t angleOffset0 = 0;
int16_t angleOffset1 = 0;
int16_t sel = 0;

int16_t getAngle0()
{
  int16_t raw;
  readAngleRaw0(&raw);
  return raw * 360 / 0xFFF;
}

int16_t getAngle1()
{
  int16_t raw;
  readAngleRaw1(&raw);
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

void turnsToDistance0()
{
  distance0 = turns0 * perimeter;
}

void turnsToDistance1()
{
  distance1 = turns1 * perimeter;
}

void distanceRobotForward0()
{
  int16_t angle = getAngle0();
  // printf("Angulo0 %d\n", angle);
  if (angleSubtraction(angle, angleOffset0) >= 350 && banTurns0)
  {
    turns0++;
    banTurns0 = false;
  }
  else if (angleSubtraction(angle, angleOffset0) <= 20)
  {
    banTurns0 = true;
  }
  turnsToDistance0();
  printf("Vueltas0: %d, Angulo0: %d\n", turns0, angleSubtraction(angle, angleOffset0));
}

void distanceRobotForward1()
{
  int16_t angle = getAngle1();
  // printf("Angulo1 %d ", angle);
  if (angleSubtraction(angle, angleOffset1) >= 350 && banTurns1)
  {
    turns1++;
    banTurns1 = false;
  }
  else if (angleSubtraction(angle, angleOffset1) <= 20)
  {
    banTurns0 = true;
  }
  turnsToDistance1();
  printf("Vueltas1: %d, Angulo1: %d\n", turns1, angleSubtraction(angle, angleOffset1));
}

void timer_callback(repeating_timer_t *rt)
{
  gpio_put(S0, sel);
  if(sel == 0) sel = 1;
  Sig_out


}

int main()
{

  stdio_init_all();
  // Inicializar el bus I2C en Raspberry Pi Pico
  i2c_init(AS560_i2c0, 200000); // Velocidad de 200 kHz
  // i2c_init(AS560_i2c1, 200000); // Velocidad de 200 kHz
  //  Configura un temporizador para 1 segundo
  uint32_t timer_interval_us = 2050; // microsegundos
  repeating_timer_t my_timer;
  add_repeating_timer_us(timer_interval_us, timer_callback, NULL, &my_timer);

  gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
  gpio_set_function(PWM_PIN2, GPIO_FUNC_PWM);
  gpio_set_function(SDA, GPIO_FUNC_I2C);  // GPIO0 como SDA
  gpio_set_function(SCL, GPIO_FUNC_I2C);  // GPIO1 como SCL
  gpio_set_function(SDA1, GPIO_FUNC_I2C); // GPIO0 como SDA
  gpio_set_function(SCL1, GPIO_FUNC_I2C); // GPIO1 como SCL

  gpio_set_function(S0, GPIO_FUNC_SIO); // GPIO como
  gpio_set_function(S1, GPIO_FUNC_SIO); // GPIO como SCL
  gpio_set_function(S2, GPIO_FUNC_SIO); // GPIO como SDA
  gpio_set_function(S3, GPIO_FUNC_SIO); // GPIO como SCL

  gpio_set_dir(S0, GPIO_IN);
  gpio_set_dir(S1, GPIO_IN);
  gpio_set_dir(S2, GPIO_IN);
  gpio_set_dir(S3, GPIO_IN);

  gpio_pull_up(SDA);
  gpio_pull_up(SCL);
  gpio_pull_up(SDA1);
  gpio_pull_up(SCL1);

  uint slice_num_5 = pwm_gpio_to_slice_num(PWM_PIN1); // Find out which PWM slice is connected to GPIO
  pwm_set_clkdiv(slice_num_5, 250.0f);                // Set clock freq at 500kHz
  pwm_set_wrap(slice_num_5, 10000);                   // Set period of 50Hz (20 ms)
  // pwm_set_phase_correct (slice_num, false); IT SEEMS THAT IS NOT NECESSARY
  pwm_set_enabled(slice_num_5, true); // Set the PWM running
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 650);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 650);
  sleep_ms(3000);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750);
  sleep_ms(3000);

  angleOffset0 = getAngle0();
  angleOffset1 = getAngle1();

  gpio_put(S0, 0);
  gpio_put(S1, 0);
  gpio_put(S2, 0);
  gpio_put(S3, 0);

  while (1)
  {
    pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 780);
    pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 780);

    // printf("Distancia1 %f, Distancia2 %f\n",distance0,distance1);

    // if (distance0 >= finalDistance){
    //   pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
    //   pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750);
    //   angleOffset0 = angleSubtraction(getAngle0(), angleOffset0);
    //   angleOffset1 = angleSubtraction(getAngle0(), angleOffset1);
    //   turns0 = 0;
    //   distance0 = 0;
    //   turns1 = 0;
    //   distance1 = 0;
    //   sleep_ms(10000);
    // }
  }
  return 0;
}