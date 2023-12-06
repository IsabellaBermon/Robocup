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

double filtroD = 0.0;
double alpha = 0.1;  // Factor de suavizado, ajustar según sea necesario
double previousErrorAngle = 0;
double integralErrorAngle = 0;

uint64_t prevTimeUsMotor1=0;
uint64_t prevTimeUsMotor2=0;
uint64_t prevTimeUsMotor3=0;
uint64_t prevTimeUsMotor4=0;

double windowTimeMotor1=0;
double windowTimeMotor2=0;
double windowTimeMotor3=0;
double windowTimeMotor4=0;

int velMotor1 = 0;
int velMotor2 = 0;
int velMotor3 = 0;
int velMotor4 = 0;

double prevErrorVel1 = 0;
double prevErrorVel2 = 0;
double prevErrorVel3 = 0;
double prevErrorVel4 = 0;

void distanceRobotCounterClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, double *distanceMotor,int *velMotor,double *windowTimeMotor,uint64_t *prevTimeUsMotor){
  
  if (angleMotor >= 330 && *banTurnsMotor){
    (*turnMotor)++;
    if((*turnMotor)%5 == 0){
      uint64_t currentTimeUs = time_us_64();
      *windowTimeMotor = (currentTimeUs - (*prevTimeUsMotor))/1000;
      *velMotor= (circunference*100000)/(*windowTimeMotor);
      *prevTimeUsMotor = currentTimeUs;

    }
    *distanceMotor = (*turnMotor)*circunference;

    *banTurnsMotor = false;
  }
  else if (angleMotor <= 30){
    *banTurnsMotor = true;
  }
}

void distanceRobotClockWise(uint16_t angleMotor,uint16_t *turnMotor,bool *banTurnsMotor, double *distanceMotor,int *velMotor,double *windowTimeMotor,uint64_t *prevTimeUsMotor){

  if (angleMotor >= 330){
    *banTurnsMotor = true;
  }
  else if (angleMotor <= 30  && *banTurnsMotor ){
    (*turnMotor)++;
    if((*turnMotor)%5 == 0){
      uint64_t currentTimeUs = time_us_64();
      *windowTimeMotor = (currentTimeUs - (*prevTimeUsMotor))/1000;
      *velMotor= (circunference*100000)/(*windowTimeMotor);
      *prevTimeUsMotor = currentTimeUs;

    }
    *distanceMotor = (*turnMotor)*circunference;
    *banTurnsMotor = false;
  }
 
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

void m1ControlSpeed(int velRef, int turn){
  double error = velRef - velMotor1;
  double pid = 0.25*error + 0.05*(error-prevErrorVel1);

  if(error != prevErrorVel1){
    if(error > 0){
      adjustMotorSpeed(1, pid > 0 ? -turn*pid : turn*pid);  

    }else if (error < 0){
      adjustMotorSpeed(1, pid > 0 ? turn*pid : -turn*pid);  
    }
  }
  prevErrorVel1 = error;
}

void m2ControlSpeed(int velRef,int turn){
  double error = velRef - velMotor2;
  double pid = 0.25*error + 0.05*(error-prevErrorVel2);

  if(error != prevErrorVel2){
    if(error > 0){
      adjustMotorSpeed(2, pid > 0 ? -turn*pid : turn*pid);  

    }else if (error < 0){
      adjustMotorSpeed(2, pid > 0 ? turn*pid : -turn*pid);  
    }
  }
  prevErrorVel2 = error;
}

void m3ControlSpeed(int velRef,int turn){
  double error = velRef - velMotor3;
  double pid = 0.3*error + 0.05*(error-prevErrorVel3);

  if(error != prevErrorVel3){
    if(error > 0){
      adjustMotorSpeed(3, pid > 0 ? turn*pid : -turn*pid);  

    }else if (error < 0){
      adjustMotorSpeed(3, pid > 0 ? -turn*pid : turn*pid);  
    }
  }
  prevErrorVel3 = error;
}

void m4ControlSpeed(int velRef,int turn){
  double error = velRef - velMotor4;
  double pid = 0.3*error + 0.05*(error-prevErrorVel4);

  if(error != prevErrorVel4){
    if(error > 0){
      adjustMotorSpeed(4, pid > 0 ? -turn*pid : turn*pid);  

    }else if (error < 0){
      adjustMotorSpeed(4, pid > 0 ? turn*pid : -turn*pid);  
    }
  }
  prevErrorVel4 = error;
}

void dualMotorPIDControl(){
 
  double error1_4 = (distanceMotor1 - distanceMotor4);
  //double error2_3 = (velMotor2 - velMotor3);a
  printf("error %f ",error1_4);
  printf("dis1 %f ",distanceMotor1);
  printf("dis4 %f ",distanceMotor4);

  // double derivativoSinFiltrar = (error1_4 - previousError1_4);
  // filtroD = alpha * derivativoSinFiltrar + (1 - alpha) * filtroD;
  integralError1_4 += error1_4; 
  //integralError2_3 += error2_3;

  double pidAdjustment1_4 = 0.3*error1_4 + 0*integralError1_4 + 0*(error1_4 - previousError1_4);
  //double pidAdjustment1_4 = 1.4* error1_4 + 0.001*integralError1_4 + 0.6*(error1_4 - previousError1_4);
  //double pidAdjustment2_3 = 2.2* error2_3 + 0.0002*integralError2_3 + 1* (error2_3 - previousError2_3);
  
  if(pidAdjustment1_4 > 0){
    pidAdjustment1_4 = ceil(pidAdjustment1_4); 
  }else{
    pidAdjustment1_4 = floor(pidAdjustment1_4);
  }
  printf("pid %f",pidAdjustment1_4);
  printf(" offset1 %d",offset1);
  printf(" offset4 %d\n",offset4);
  
  // if(pidAdjustment2_3 > 0){
  //   pidAdjustment2_3 = ceil(pidAdjustment2_3);
  // }else{
  // //   pidAdjustment2_3 = floor(pidAdjustment2_3);
  // // }
   //printf("adj1 %lf \n",pidAdjustment1_4);
  // printf("off1 %d ",offset1);
  // printf("off4 %d ",offset4);
  // printf("error1_4 %lf\n",error1_4);
  
  if(error1_4!=previousError1_4){
    if(error1_4 > 0){
      adjustMotorSpeed(1, pidAdjustment1_4 > 0 ? pidAdjustment1_4 : -pidAdjustment1_4);  
      adjustMotorSpeed(4, pidAdjustment1_4 > 0 ? pidAdjustment1_4 : -pidAdjustment1_4);        
    }else if(error1_4<0){
      adjustMotorSpeed(1, pidAdjustment1_4 > 0 ? -pidAdjustment1_4 : pidAdjustment1_4);  
      adjustMotorSpeed(4, pidAdjustment1_4 > 0 ? -pidAdjustment1_4 : pidAdjustment1_4);  
    }
  }
    previousError1_4 = error1_4;
  // if(error2_3 != previousError2_3){
  //   if(error2_3 > 0){
  //     adjustMotorSpeed(2, pidAdjustment2_3 > 0 ? -pidAdjustment2_3 : pidAdjustment2_3);  
  //     adjustMotorSpeed(3, pidAdjustment2_3 > 0 ? pidAdjustment2_3 : -pidAdjustment2_3);        
  //   }else if(error2_3<0){
  //     adjustMotorSpeed(2, pidAdjustment2_3 > 0 ? pidAdjustment2_3 : -pidAdjustment2_3);  
  //     adjustMotorSpeed(3, pidAdjustment2_3 > 0 ? -pidAdjustment2_3 : pidAdjustment2_3);  
  //   }
  // }
  // previousError2_3 = error2_3;
}

void dualMotorPDControlRotation(){


  double error1_4 = motorAngle1 - motorAngle4;
  double error2_3 = motorAngle2 - motorAngle3;

  double pidAdjustment1_4 = (0.15) * error1_4  + 0.1 * (error1_4 - previousError1_4);
  double pidAdjustment2_3 = 0.15 * error2_3 +  0.1* (error2_3 - previousError2_3);

  // printf("adj1 %lf ,",pidAdjustment1_4);
  // printf("error1_4 %lf ,",error1_4);
  // printf("error2_3 %lf ,",error2_3);
  // printf("adj2 %lf\n",pidAdjustment2_3);
 if(error1_4!=previousError1_4){
    if(error1_4 > 0){
      adjustMotorSpeed(1, pidAdjustment1_4 > 0 ? pidAdjustment1_4 : -pidAdjustment1_4);  
      adjustMotorSpeed(4, pidAdjustment1_4 > 0 ? -pidAdjustment1_4 : pidAdjustment1_4);        
    }else if(error1_4<0){
      adjustMotorSpeed(1, pidAdjustment1_4 > 0 ? -pidAdjustment1_4 : pidAdjustment1_4);  
      adjustMotorSpeed(4, pidAdjustment1_4 > 0 ? pidAdjustment1_4 : -pidAdjustment1_4);  
    }
  }
  if(error2_3 != previousError2_3){
    if(error2_3 > 0){
      adjustMotorSpeed(2, pidAdjustment2_3 > 0 ? pidAdjustment2_3 : -pidAdjustment2_3);  
      adjustMotorSpeed(3, pidAdjustment2_3 > 0 ? pidAdjustment2_3 : -pidAdjustment2_3);        
    }else if(error2_3<0){
      adjustMotorSpeed(2, pidAdjustment2_3 > 0 ? -pidAdjustment2_3 : pidAdjustment2_3);  
      adjustMotorSpeed(3, pidAdjustment2_3 > 0 ? -pidAdjustment2_3 : pidAdjustment2_3);  
    }
  }

  previousError1_4 = error1_4;
  previousError2_3 = error2_3;
  
}

void restartControl(){
  distanceMotor1 = 0;
  distanceMotor2 = 0;
  distanceMotor3 = 0;
  distanceMotor4 = 0;
  velMotor1=0;
  velMotor2=0;
  velMotor3=0;
  velMotor4=0;
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
  prevErrorVel1 = 0;
  prevErrorVel2 = 0;
  prevErrorVel3 = 0;
  prevErrorVel4 = 0;
  
  prevTimeUsMotor1=0;
  prevTimeUsMotor2=0;
  prevTimeUsMotor3=0;
  prevTimeUsMotor4=0;

}
