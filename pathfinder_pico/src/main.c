#include "pico/stdlib.h"
#include <stdio.h>
#include "pathfinder.h"

int main() {
    
    stdio_init_all();

    Robot robot = {{1, 1}, 0};  // Example initial position for the robot
    Ball ball = {{25, 25}};        // Example ball position
    Obstacle obstacles[NUM_OBSTACLES] = { {{5, 5}}, {{15, 15}}, {{24, 20}}};  // Example obstacle positions

    Node nodes[GRID_WIDTH][GRID_HEIGHT];
    initializeNodes(nodes, &robot, &ball, obstacles, NUM_OBSTACLES);

    Point start = {round(robot.position.x), round(robot.position.y)};
    Point goal = {round(ball.position.x), round(ball.position.y)};
    
    NodeList pathList;

   
    //int numMovimientos;
    // convertPathToMovements(&pathList, movimientos, &numMovimientos);

    Movimiento movimientosConsolidados[MAX_NODES];

    // consolidateMovements(movimientos, numMovimientos, movimientosConsolidados, &numMovimientosConsolidados);
     sleep_ms(2000);
    NodeList_init(&pathList);
    AStar(nodes, start, goal, &pathList);
    while (1)
    {
           
            // markPathOnGrid(nodes, &pathList, 'O');
            // printGrid(nodes);
         
            consolidateMovements(&pathList, movimientosConsolidados);
            printf("num %d \n",numMovimientos);
            // for (int i = 0; i < numMovimientos; i++) {
            //     printf("Girar: %d grados, Mover %d hacia adelante\n", movimientos[i].giro,movimientos[i].movimiento);
            // }
            // printf("\n");
            // // Imprimir los movimientos consolidados
            for (int i = 0; i < numMovimientosConsolidados; i++) {
                printf("Girar: %d grados, Mover: %d\n", movimientosConsolidados[i].giro, movimientosConsolidados[i].movimiento);
            }
            sleep_ms(100000);
    
    }

    return 0;
}