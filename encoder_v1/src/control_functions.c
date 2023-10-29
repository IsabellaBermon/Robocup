#include "control_functions.h"

double distanceMotor1 = 0;
double distanceMotor2 = 0;
double distanceMotor3 = 0;
double distanceMotor4 = 0;
double motorAngle1=0;
double motorAngle2=0;
double motorAngle3=0;
double motorAngle4=0;
int16_t offset1=0;  
int16_t offset2=0;  
int16_t offset3=0;  
int16_t offset4=0; 
double integralError1_4 = 0;
double integralError2_3 = 0;
double previousError1_4 = 0;
double previousError2_3 = 0;
double Kp_d = 0.3; // Coeficiente proporcional
double Ki_d = 0.000001; // Coeficiente integral
double Kd_d = 0.02; // Coeficiente derivativo
// Coeficientes del controlador PID
double Kp = 0.15; // Coeficiente proporcional
double Ki = 0.05; // Coeficiente integral
double Kd = 0.1; // Coeficiente derivativo
double Kp_r= 0.6; // Coeficiente proporcional
double Ki_r = 0.00022; // Coeficiente integral
double Kd_r = 0.1; // Coeficiente derivativo
double integralError_pair = 0;
double previousError_pair = 0;
double Kp_pair = 1; // Coeficiente proporcional
double Ki_pair = 0.000001; // Coeficiente integral
double Kd_pair = 0; // Coeficiente derivativo

double previousErrorAngle = 0;
double integralErrorAngle = 0;

void distanceRobotCounterClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, double *distanceMotor){

  if (angleMotor >= 350 && *banTurnsMotor){
    (*turnMotor)++;
    *banTurnsMotor = false;
  }
  else if (angleMotor <= 20){
    *banTurnsMotor = true;
  }
  *distanceMotor = (*turnMotor)*circunference;
}

void distanceRobotClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, double *distanceMotor){

  if (angleMotor >= 350){
    *banTurnsMotor = true;
  }
  else if (angleMotor <= 20  && *banTurnsMotor ){
    (*turnMotor)++;

    *banTurnsMotor = false;
  }
  *distanceMotor = (*turnMotor)*circunference;

}
void adjustMotorSpeed(uint motorNumber, double adjustment) {
 switch (motorNumber){
 case 1:
  offset1+=adjustment;
  break;
 case 2:
  offset2+=adjustment;
  break;
 case 3:
  offset3+=adjustment;
  break;
 case 4:
  offset4+=adjustment;
  break; 
 default:
  break;
 }
}
void dualMotorPIDControl(){
  double error1_4 = (distanceMotor1 - distanceMotor4);
  double error2_3 = (distanceMotor2 - distanceMotor3);
  double pidAdjustment1_4 = Kp_d * error1_4 + Ki_d*integralError1_4 + Kd_d * (error1_4 - previousError1_4);
  double pidAdjustment2_3 = Kp_d * error2_3 + Ki_d*integralError2_3 + Kd_d* (error2_3 - previousError2_3);
  
  integralError1_4 += error1_4;
  integralError2_3 += error2_3;
  
  // printf("adj1 %lf ,",pidAdjustment1_4);
  // printf("error1_4 %lf ,",error1_4);
  // printf("error2_3 %lf ,",error2_3);
  // printf("adj2 %lf\n",pidAdjustment2_3);
  previousError1_4 = error1_4;
  previousError2_3 = error2_3;
  if(error1_4 > 0){
    adjustMotorSpeed(1, pidAdjustment1_4);  
  }else{

    adjustMotorSpeed(4, -pidAdjustment1_4);  
  }
  if(error2_3 > 0){

  adjustMotorSpeed(2, -pidAdjustment2_3);  
  }else{

  adjustMotorSpeed(3, -pidAdjustment2_3); 
  }
}

void dualMotorPDControlRotation(){

  double error1_4 = distanceMotor1 - distanceMotor4;
  double error2_3 = distanceMotor2 - distanceMotor3;

  double pidAdjustment1_4 = (Kp_r) * error1_4 + Ki_r*integralError1_4 + Kd_r * (error1_4 - previousError1_4);
  double pidAdjustment2_3 = Kp_r * error2_3 + Ki_r*integralError2_3 + Kd_r* (error2_3 - previousError2_3);
  integralError1_4 += error1_4;
  integralError2_3 += error2_3;
  previousError1_4 = error1_4;
  previousError2_3 = error2_3;
  printf("adj1 %lf ,",pidAdjustment1_4);
  printf("error1_4 %lf ,",error1_4);
  printf("error2_3 %lf ,",error2_3);
  printf("adj2 %lf\n",pidAdjustment2_3);
  if(error1_4 > 0){
    adjustMotorSpeed(1, pidAdjustment1_4);  // Reducir la velocidad del motor 1 si el error es positivo
  }else{

    adjustMotorSpeed(4, pidAdjustment1_4);   // Aumentar la velocidad del motor 4 si el error es positivo
  }
  if(error2_3 > 0){

  adjustMotorSpeed(2, pidAdjustment2_3);  // Reducir la velocidad del motor 2 si el error es positivo
  }else{

  adjustMotorSpeed(3, -pidAdjustment2_3);   // Aumentar la velocidad del motor 3 si el error es positivo
  }
}

void motorsPIControlPosition(double error){
  double pidAdjustment_pair = Kp_pair * error + Ki_pair * integralError_pair + Kd_pair * (error - previousError_pair);
  integralError_pair += error;
  previousError_pair = error;
  //   printf("adj1 %lf ,",pidAdjustment_pair);
  //   printf("error %lf ,",errorX1_X2);

  // Ajuste de la velocidad de los motores basado en el error
  if (error > 0) {
      // Si el error es positivo, reducimos la velocidad de los motores 1 y 4
      adjustMotorSpeed(1, pidAdjustment_pair);  
      adjustMotorSpeed(4, -pidAdjustment_pair);  
  } else {
      // Si el error es negativo, reducimos la velocidad de los motores 2 y 3
      adjustMotorSpeed(2, -pidAdjustment_pair);  
      adjustMotorSpeed(3, -pidAdjustment_pair);  
  }
}

void anglesPIDControlRotation(double errorAngle){
  double adjustmentAngle = 0.4*errorAngle +0.000005*integralErrorAngle+ 0.005*(errorAngle-previousErrorAngle);
  integralErrorAngle += errorAngle;
  previousErrorAngle = errorAngle;
  // printf("adj1 %lf ,",adjustmentAngle);
  // printf("error %lf ,",errorAngle);
  if (errorAngle > 0) {
      // Si el error es positivo, reducimos la velocidad de los motores 1 y 4
      adjustMotorSpeed(1, adjustmentAngle);  
      adjustMotorSpeed(4, adjustmentAngle);  
  } else {
      // Si el error es negativo, reducimos la velocidad de los motores 2 y 3
      adjustMotorSpeed(2, adjustmentAngle);  
      adjustMotorSpeed(3, -adjustmentAngle);  
  }
}

void restartControl(){
  distanceMotor1 = 0;
  distanceMotor2 = 0;
  distanceMotor3 = 0;
  distanceMotor4 = 0;
  motorAngle1=0;
  motorAngle2=0;
  motorAngle3=0;
  motorAngle4=0;
  offset1=0;  
  offset2=0;  
  offset3=0;  
  offset4=0; 
  integralError1_4 = 0;
  integralError2_3 = 0;
  previousError1_4 = 0;
  previousError2_3 = 0;
  previousErrorAngle = 0;
  integralErrorAngle = 0; 
  previousErrorAngle = 0;
  integralErrorAngle = 0;
}
