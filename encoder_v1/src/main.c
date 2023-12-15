#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "robot_movement.h"
#include "pathfinder.h"
#include "dribbler.h"
#include "mqtt.h"
Movimiento movimientosConsolidados[MAX_NODES];


void wifiConnect(){

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    
    while(cyw43_arch_wifi_connect_timeout_ms(SSID, PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
        printf("Attempting to connect...\n");
    } 
    printf("Connected.\n");
    

}

void pathFinder(){
    printf("Calculando ruta\n");
    Robot robot = {{positions[0], positions[1]}, positions[2]};
    Ball ball = {{positions[3], positions[4]}};        // Example ball position
    Obstacle obstacles[NUM_OBSTACLES] = { {{positions[5], positions[6]}}, {{positions[7], positions[8]}}};  // Example obstacle positions

    Node nodes[GRID_WIDTH][GRID_HEIGHT];
    initializeNodes(nodes, &robot, &ball, obstacles, NUM_OBSTACLES);

    Point start = {robot.position.x, robot.position.y};
    Point goal = {ball.position.x, ball.position.y};  
    Point arco = {GRID_WIDTH/2,GRID_HEIGHT-3};

    NodeList pathList;
    sleep_ms(1000);
    NodeList_init(&pathList);
    AStar(nodes, start, goal, &pathList);
    markPathOnGrid(nodes, &pathList, 'O');
    printGrid(nodes);
    
    consolidateMovements(&pathList, movimientosConsolidados);
    for (int i = 0; i < numMovimientosConsolidados; i++) {
        printf("Girar: %d grados, Mover: %d\n", movimientosConsolidados[i].giro, movimientosConsolidados[i].movimiento);
    }


}

// Function declarations
void HardwareInit()
{
    stdio_init_all();
    mpu_init();
    mpu6050_reset();
    initI2C();
    getOffsets();
    initMotor();
    initMotorControl();
    // dribble();
    wifiConnect();
    connect_mqtt();
}
/**
 * @brief Lee y actualiza los ángulos de los motores del robot.
 *
 * Selecciona cada canal de motor y actualiza el ángulo del motor correspondiente
 * restando el ángulo offset inicial.
 */
void getAnglesMotors(){
    tca_select_channel(0);
    angleMotor1 = angleSubtraction(getAngle(),offsetAngleMotor1);
    tca_select_channel(1);
    angleMotor2 = angleSubtraction(getAngle(),offsetAngleMotor2);
    tca_select_channel(2);
    angleMotor3 = angleSubtraction(getAngle(),offsetAngleMotor3);
    tca_select_channel(3);
    angleMotor4 = angleSubtraction(getAngle(),offsetAngleMotor4);
    tca_select_channel(4);
}


int main(){

  HardwareInit();
  while(1){
    // getAnglesMotors();
    // updateAngle();
    // if (!banStop){
    //     rotation(-45);
    //     printf("angulo %f \n",robotAngle);
    // }
    // else{
    //     sleep_ms(1000);
    //     rotation(-45);
    //     banStop = false;
    // }
    
    if(banPositions){        
        pathFinder();
        for (int i = 0; i < numMovimientosConsolidados; i++) {
            while(1){
                // printf("rotacion\n");
                updateAngle();
                rotation(movimientosConsolidados[i].giro);
                if(banStop){
                    banStop=false;
                    break;
                }
            }
            sleep_ms(500);
            while(1){
                // printf("adelante\n");

                getAnglesMotors();
                updateAngle();
                moveForward(movimientosConsolidados[i].movimiento*0.04);
                if(banStop){
                    banStop=false;
                    break;
                }
            }
            sleep_ms(500);        
        }
        banPositions=false;
        numMovimientos = 0;
        numMovimientosConsolidados = 0;
    }
  }
  return 0;
}                       


