#include "robot_movement.h"

#define N_SAMPLES 100
#define ACCEL_SCALE_FACTOR 16384.0
// Variables para almacenar los offsets

double wTimeUpdateAngle = 0.0022;

float offsetXa = 0, offsetYa = 0;
uint16_t offCW1 = 726;
uint16_t offCW2 = 720;
uint16_t offCW3 = 780;
uint16_t offCW4 = 720;
uint16_t offCC1 = 780;
uint16_t offCC2 = 780;
uint16_t offCC3 = 720;
uint16_t offCC4 = 774;

int refVelMotor1=9;
int refVelMotor2=9;
int refVelMotor3=9;
int refVelMotor4=9;
double prevErrorAngle =0;

double prevMpuOffset = 0;
uint16_t angleMotor1 = 0;
uint16_t angleMotor2 = 0;
uint16_t angleMotor3 = 0;
uint16_t angleMotor4 = 0;

uint16_t turnMotor1 = 0;
uint16_t turnMotor2 = 0;
uint16_t turnMotor3 = 0;
uint16_t turnMotor4 = 0;

bool banTurnsMotor1 = 0;
bool banTurnsMotor2 = 0;
bool banTurnsMotor3 = 0;
bool banTurnsMotor4 = 0;

bool banStop = false;

int ax, ay;
int prevAy=10000000;

int64_t prevTime = 0;
double angularPosition=0, prevAngularPosition=0;
double angularVelocity;
double robotAngle = 0;
bool angularFlag = true;
int offsetZ;
int16_t acceleration[3], gyro[3],temp;

void motorCounterClockWise1(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, offCC1 + offset1); // 777
}
void motorClockWise1(){
  if((offCW1 + offset1) >= 718){
    if((offCW1+offset1) >= 742){
      offset1 = 742-offCW1;
    }
    pwm_set_chan_level(slice_num_5, PWM_CHAN_A, offCW1 + offset1); // 723
  }else{
    offset1=-15;
  }
}
void motorCounterClockWise2(){
  if((offCC2 + offset2)<=785){
     if((offCC2 + offset2) <= 768){
      offset2 = 768-offCC2;
    }
    pwm_set_chan_level(slice_num_5, PWM_CHAN_B, offCC2 + offset2); // 780
  }else{
    offset2=12;
  }
}
void motorClockWise2(){

  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, offCW2 + offset2); // 720
}
void motorCounterClockWise3(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, offCC3 + offset3); //700

}
void motorClockWise3(){
  if((offCW3+offset3) <= 785){
    if((offCW3 + offset3) <= 766){
      offset3 = 766-offCW3;
    }
    pwm_set_chan_level(slice_num_6, PWM_CHAN_A, offCW3 + offset3); // 780
  }else{
    offset3=10;
  }
}
void motorCounterClockWise4(){
  if((offCC4 + offset4) <= 785){
    if((offCC4 + offset4) <= 766){
      offset4 = 766-offCC4;
    }
    pwm_set_chan_level(slice_num_6, PWM_CHAN_B, offCC4 + offset4); 
  }else{
    offset4 = 10;
  }
}
void motorClockWise4(){
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, offCW4 + offset4); // 720
}
void motorStop(){
  pwm_set_chan_level(slice_num_5, PWM_CHAN_A, 750);
  pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_A, 750); 
  pwm_set_chan_level(slice_num_6, PWM_CHAN_B, 750);
}
void motorsForward(){
  motorClockWise1();
  motorCounterClockWise2();
  motorClockWise3();
  motorCounterClockWise4();
}
void motorsClockWise(){
    motorClockWise1();
    motorClockWise2();
    motorClockWise3();
    motorClockWise4();
}
void distanceMotorsForward(){
  distanceRobotClockWise(angleMotor1,&turnMotor1,&banTurnsMotor1,&distanceMotor1,&velMotor1,&windowTimeMotor1,&prevTimeUsMotor1);
  distanceRobotCounterClockWise(angleMotor2,&turnMotor2,&banTurnsMotor2,&distanceMotor2,&velMotor2,&windowTimeMotor2,&prevTimeUsMotor2);
  distanceRobotClockWise(angleMotor3,&turnMotor3,&banTurnsMotor3,&distanceMotor3,&velMotor3,&windowTimeMotor3,&prevTimeUsMotor3);
  distanceRobotCounterClockWise(angleMotor4,&turnMotor4,&banTurnsMotor4,&distanceMotor4,&velMotor4,&windowTimeMotor4,&prevTimeUsMotor4);
}
void distanceMotorsClockWise(){
    distanceRobotClockWise(angleMotor1,&turnMotor1,&banTurnsMotor1,&distanceMotor1,&velMotor1,&windowTimeMotor1,&prevTimeUsMotor1);
    distanceRobotClockWise(angleMotor2,&turnMotor2,&banTurnsMotor2,&distanceMotor2,&velMotor2,&windowTimeMotor2,&prevTimeUsMotor2);
    distanceRobotClockWise(angleMotor3,&turnMotor3,&banTurnsMotor3,&distanceMotor3,&velMotor3,&windowTimeMotor3,&prevTimeUsMotor3);
    distanceRobotClockWise(angleMotor4,&turnMotor4,&banTurnsMotor4,&distanceMotor4,&velMotor4,&windowTimeMotor4,&prevTimeUsMotor4);
  }
void calibrate() {
  float sum_x = 0, sum_y = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    sum_x += acceleration[0];
    sum_y += acceleration[1];

    sleep_ms(10); // Pequeña pausa entre lecturas
  }
  offsetXa = sum_x / N_SAMPLES;
  offsetYa = sum_y / N_SAMPLES;
}

// Función para procesar los datos del acelerómetro
void readAndProcessAccelerometer(int *ax, int *ay) {
    // Restar los offsets y convertir a m/s²
  *ax = (acceleration[0] - offsetXa) / ACCEL_SCALE_FACTOR * 9.81;
  *ay = (acceleration[1] - offsetYa) / ACCEL_SCALE_FACTOR * 9.81;

}

void rotation(double rotationAngle){
  wTimeUpdateAngle=0.0028;

  offCW1 = 738;
  offCW2 = 738;
  offCW3 = 765; 
  offCW4 = 738;
  if(rotationAngle > 0){
    motorsClockWise();
    //distanceMotorsClockWise();
    // motorAngle1 = distanceMotor1/radio;
    // motorAngle2 = distanceMotor2/radio;
    // motorAngle3 = distanceMotor3/radio;
    // motorAngle4 = distanceMotor4/radio;
    // double motorAngle1_4 =(motorAngle1 + motorAngle4)/2;
    // double motorAngle2_3 = (motorAngle2 + motorAngle3)/2;
    // double angleFinal = (motorAngle1_4+motorAngle2_3)/2;

    // printf(" angle %f \n",motorAngle1_4);
    if(robotAngle+20 >=rotationAngle){
      motorStop(); 
      //sleep_ms(5000);
      restartControl();
      restartMovement(); 
      //resetFilter();
      getOffsets();
      banStop=true;
    }

    //dualMotorPDControlRotation();

   
    // double angleError = motorAngle1_4 - motorAngle2_3;
    // anglesPIDControlRotation(angleError);

  
  }
}

void moveForward(double distance){
  offCW1 = 733;
  offCC2 = 777;
  offCW3 = 775;
  offCC4 = 775;
  wTimeUpdateAngle=0.0022;
  if (distance > 0){  
    int mpuOffset = robotAngle;
    
    motorsForward();
    distanceMotorsForward();
    double posx1 = (distanceMotor1+distanceMotor4)*cos(52*PI/180)/2;
    double posx2 = (distanceMotor2+distanceMotor3)*cos(52*PI/180)/2;
    double finalPos = (posx1 + posx2)/2;
    if(finalPos >= distance){
      motorStop();
      restartControl();
      restartMovement(); 
      getOffsets();
      banStop=true;
    }else{
      if(mpuOffset == 0){
        m2ControlSpeed(refVelMotor2,-1);
        m4ControlSpeed(refVelMotor4,-1);
        m1ControlSpeed(refVelMotor1,1);
        m3ControlSpeed(refVelMotor3,1);
      }else{
        m2ControlSpeed(refVelMotor2+mpuOffset/2,-1);
        m1ControlSpeed(refVelMotor1-mpuOffset/2,1);
        m4ControlSpeed(refVelMotor4,-1);
        m3ControlSpeed(refVelMotor3,1);
      }

      motorsForward();
      distanceMotorsForward();
    }
    //dualMotorPIDControl();
 
   
  }
}
int errorAcc =0;
void circularMovement(double r, int angle){
  wTimeUpdateAngle=0.0022;

  offCW1 = 733;
  offCC2 = 777;
  offCW3 = 775; 
  offCC4 = 775;
  motorsForward();
  distanceMotorsForward();
  //readAndProcessAccelerometer(&ax, &ay);
  int16_t velM24 =ceil((15/r)*(r - 0.135/2));
  int16_t velM13 = (15/r)*(r + 0.135/2);
  // if(ay != prevAy){
  //   if(errorAcc>= (velM13-velM24)){
  //     errorAcc += ay;
  //   }else{
  //     errorAcc = velM13-velM24;
  //   } 
  // }

  // if(ay == 0){
  //   errorAcc=0;
    m2ControlSpeed(velM24,-1);
    m4ControlSpeed(velM24,-1);
    m1ControlSpeed(velM13,1);
    m3ControlSpeed(velM13,1);

  // }else{
  //   m2ControlSpeed(velM24,-1);
  //   m4ControlSpeed(velM24,-1);
  //   m1ControlSpeed(velM13+errorAcc,1);
  //   m3ControlSpeed(velM13+errorAcc,1);
  // }
  // prevAy = ay;

  double posx1 = (distanceMotor1+distanceMotor3)/2;

  if(posx1>= angle*PI*r/90){
    motorStop();
    // sleep_ms(10000);
    restartControl();
    restartMovement();
   // resetFilter();
    getOffsets();
    banStop=true;
  }




}


void restartMovement(){
  angleMotor1 = 0;
  angleMotor2 = 0;
  angleMotor3 = 0;
  angleMotor4 = 0;
  turnMotor1 = 0;
  turnMotor2 = 0;
  turnMotor3 = 0;
  turnMotor4 = 0;
  banTurnsMotor1 = 0;
  banTurnsMotor2 = 0;
  banTurnsMotor3 = 0;
  banTurnsMotor4 = 0;
  angularPosition=0;
  prevAngularPosition=0;
  angularVelocity=0;
  robotAngle = 0;


  //offsetZ=0;
}
void updateAngle(){
  //uint64_t currentTime = time_us_64();

  mpu6050_read_raw(acceleration,gyro);
  if(angularFlag==true){
    offsetZ = filter_median_moving(gyro[2]);
    if (index_media==9){
      angularFlag = false;
    }
  }
  else {
    // double windowTime = (currentTime - prevTime);
    //  printf("window %lf\n ",windowTime);
    // prevTime=currentTime;
    angularVelocity = (gyro[2] > 0 ? gyro[2]+offsetZ : gyro[2]-offsetZ)/131;
    double angle = (prevAngularPosition + (angularVelocity*wTimeUpdateAngle));
    prevAngularPosition = angle;
    angularPosition = angle > 0 ? angle*1.125: angle*1;
    // Actualiza ángulo cada 10°
    if (angularPosition>=2){
      prevAngularPosition = 0;
      robotAngle += 2;
    }
    else if (angularPosition<=-2)
    {
      prevAngularPosition = 0;
      robotAngle -= 2;
    }
  }
  

}