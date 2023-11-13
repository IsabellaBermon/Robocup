#include <stdio.h>
#include "robot_movement.h"
#include "bt_functions.h"
#include "MPU6050_i2c.h"

#define WINDOW_SIZE 10
#define SDA_MPU 18
#define SCL_MPU 19
int offsetZ = 0;
bool angularFlag = true;
double angularPosition=0, prevAngularPosition=0;
double angularVelocity;
int16_t acceleration[3], gyro[3],temp;
int gyro_z_readings[WINDOW_SIZE] = {0};
int index_media = 0;
int sum = 0;
double sumTime = 0;
double robotAngle = 0;

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
  // printf("angle 1 %d ",angleMotor1);
  // printf("angle 4 %d\n",angleMotor4);
}

int filter_median_moving(int new_reading) {
    // Subtract the oldest reading from sum
    sum -= gyro_z_readings[index_media];
    // Add the new reading to sum
    sum += new_reading;
    // Store the new reading in the buffer
    gyro_z_readings[index_media] = new_reading;
    // Increment the index
    index_media = (index_media + 1) % WINDOW_SIZE;
    // Return the average
    return sum / WINDOW_SIZE;
}

bool timer_callback(repeating_timer_t *t){
  mpu6050_read_raw(acceleration,gyro);
  if(angularFlag==true){
    offsetZ = filter_median_moving(gyro[2]);
    if (index_media==9){
      angularFlag = false;
    }
  }
  else {
    angularVelocity = (gyro[2] > 0 ? gyro[2]+offsetZ : gyro[2]-offsetZ)/131; 
    double angle = (prevAngularPosition + (angularVelocity*0.0022));
    prevAngularPosition = angle;
    angularPosition = angle > 0 ? angle*2 : angle*1.9;
    // Actualiza ángulo cada 10°
    if (angularPosition>=10){
      prevAngularPosition = 0;
      robotAngle += 10;
    }
    else if (angularPosition<=-10)
    {
      prevAngularPosition = 0;
      robotAngle -= 10;
    }
  }
  return true;
}

int main(){
  stdio_init_all();

  static repeating_timer_t timer;
  initI2C();
  i2c_init(MPU6050_i2c,100000);
  gpio_set_function(SDA_MPU,GPIO_FUNC_I2C);
  gpio_set_function(SCL_MPU,GPIO_FUNC_I2C);
  gpio_pull_up(SDA_MPU);
  gpio_pull_up(SCL_MPU);
  mpu6050_reset();

  getOffsets();
  initMotor();
  add_repeating_timer_us(2200,&timer_callback,NULL,&timer);

  //initBluetooth();    

  while (1){
    printf("angle: %lf, angularP: %lf\n",robotAngle,angularPosition);
    // getAnglesMotors();
    // moveForward(1.5);

    //   if(btAvailable){
    //     continue;
    //   }

    //   if(banAngle){
    //     rotation(angleBt);
    //   }else if(banDistance){
    //     moveForward(distanceBt);
    //   }

    //   if(banStop){
    //     banAngle=false;
    //     banDistance=false;
    //     btAvailable = true;
    //     banStop = false;
    //   }
  }

  return 0;
}                       