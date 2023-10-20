#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include "encoder/encoder.h"
#include <math.h>

#define testDistance 1

// Definición de constantes

#define SDA 0
#define SCL 1
#define SDA1 2
#define SCL1 3
#define PWM_PIN1 10 // pin 14   SLICE 5 CH A
#define PWM_PIN2 11 // pin 15   SLICE 5 CH b

#define circunference 0.037698 // Circunferencia en metros
float distance0 = 0;
float distance1 = 0;
uint16_t turns0 = 0;
uint16_t turns1 = 0;
bool banTurns0 = true;
bool banTurns1 = true;

int16_t angleOffset0 = 0;
int16_t angleOffset1 = 0;
int prueba = 0;
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
  distance0 = turns0 * circunference;
}
void turnsToDistance1()
{
  distance1 = turns1 * circunference;
}
void distanceRobotForward0()
{
  int16_t angle =getAngle0();
  //printf("Angulo0 %d ", angle);
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
  //printf("Vueltas: %d, Angulo: %d\n", turns0, angleSubtraction(angle, angleOffset0));
}
void distanceRobotForward1()
{
  int16_t angle = getAngle1();
  //printf("Angulo1 %d\n",angle);
  if (angleSubtraction(angle, angleOffset1) >= 350 && banTurns1)
  {
    turns1++;
    banTurns1 = false;
  }
  else if (angleSubtraction(angle, angleOffset1) <= 20)
  {
    banTurns1 = true;
  }
  turnsToDistance1();
  // printf("Vueltas: %d, Angulo: %d\n",turns,angleSubtraction(angle));
}

// void distanceRobotBackward()
// {
//   int16_t angle = getAngle();

//   if(angleSubtraction(angle)<=20 && banTurns){
//     turns++;
//     banTurns = false;
//   }else if(angleSubtraction(angle)>=350){
//     banTurns = true;
//   }
//   turnsToDistance();
//   //printf("Vueltas: %d, Angulo: %d\n",turns,angleSubtraction(angle));
// }
int main()
{
  stdio_init_all();

  // Inicializar el bus I2C en Raspberry Pi Pico
  i2c_init(AS560_i2c, 200000);           // Velocidad de Hz
  i2c_init(AS560_i2c1, 200000);          // Velocidad de Hz
  gpio_set_function(SDA, GPIO_FUNC_I2C);  // GPIO0 como SDA
  gpio_set_function(SDA1, GPIO_FUNC_I2C); // GPIO0 como SDA1
  gpio_set_function(SCL1, GPIO_FUNC_I2C); // GPIO0 como SDA1
  gpio_set_function(SCL, GPIO_FUNC_I2C);  // GPIO1 como SCL

  gpio_pull_up(SDA);
  gpio_pull_up(SDA1);
  gpio_pull_up(SCL1);
  gpio_pull_up(SCL);

  gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
  gpio_set_function(PWM_PIN2, GPIO_FUNC_PWM);
  uint slice_num_5 = pwm_gpio_to_slice_num(PWM_PIN1); // Find out which PWM slice is connected to GPIO
  pwm_set_clkdiv(slice_num_5, 250.0f); // Set clock freq at 500kHz
  pwm_set_wrap(slice_num_5, 10000);    // Set period of 50Hz (20 ms)
  // pwm_set_phase_correct (slice_num, false); IT SEEMS THAT IS NOT NECESSARY
  pwm_set_enabled(slice_num_5, true); // Set the PWM running
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 650);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 650);
  // pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 650);
  // gpio_set_irq_enabled_with_callback(SCL, GPIO_IRQ_EDGE_RISE, true, &i2c_interrupt_handler);
  sleep_ms(3000);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750);

  // pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750);

  sleep_ms(3000);
  angleOffset0 = getAngle0();
  //angleOffset1 = getAngle1();
  while (1)
  {

    pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 780);
    pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 780);
    // uint64_t currentTime = time_us_64();
    distanceRobotForward0();
    //distanceRobotForward1();
    printf("Distancia %f, Vueltas %d\n",distance1,turns1);
    if (distance0 >= testDistance)
    {
      pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
      pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750);
      angleOffset0 = angleSubtraction(getAngle0(), angleOffset0);
      angleOffset1 = angleSubtraction(getAngle0(), angleOffset1);
      turns0 = 0;
      distance0 = 0;
      turns1 = 0;
      distance1 = 0;
      sleep_ms(10000);
    }
    // printf("Lectura. = %d\n", buffer);
    // printf("Distancia = %f \n",distance);
    // printf("Time = %llu \n", currentTime);
    // sleep_ms(1000);
  }

  return 0;
}