#include "pico/stdlib.h"
#include <stdio.h>
#include "pathfinder.h"

int main() {
    
    stdio_init_all();

    Robot robot = {{2, 2}, 0};
    Ball ball = {{20, 20}};        // Example ball position
    Obstacle obstacles[NUM_OBSTACLES] = { {{8, 8}}, {{15, 15}}};  // Example obstacle positions

    Node nodes[GRID_WIDTH][GRID_HEIGHT];
    initializeNodes(nodes, &robot, &ball, obstacles, NUM_OBSTACLES);

    Point start = {robot.position.x, robot.position.y};
    Point goal = {ball.position.x, ball.position.y};  
    Point arco = {GRID_WIDTH/2,GRID_HEIGHT-3};

    NodeList pathList;

   
    //int numMovimientos;
    // convertPathToMovements(&pathList, movimientos, &numMovimientos);

    Movimiento movimientosConsolidados[MAX_NODES];

    // consolidateMovements(movimientos, numMovimientos, movimientosConsolidados, &numMovimientosConsolidados);
    sleep_ms(3000);
    NodeList_init(&pathList);
    AStar(nodes, start, goal, &pathList);
    while (1)
    {
           
            markPathOnGrid(nodes, &pathList, 'O');
            printGrid(nodes);
         
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