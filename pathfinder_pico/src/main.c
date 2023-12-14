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
    NodeList_init(&pathList);

    Movimiento movimientos[MAX_NODES]; // Asegúrate de que este tamaño sea suficiente
    int numMovimientos;
    // convertPathToMovements(&pathList, movimientos, &numMovimientos);

    Movimiento movimientosConsolidados[MAX_NODES];
    int numMovimientosConsolidados;
    // consolidateMovements(movimientos, numMovimientos, movimientosConsolidados, &numMovimientosConsolidados);

    while (1)
    {
        if(getchar()) {
            AStar(nodes, start, goal, &pathList);
            // markPathOnGrid(nodes, &pathList, 'O');
            // printGrid(nodes);
            convertPathToMovements(&pathList, movimientos, &numMovimientos);
            consolidateMovements(movimientos, numMovimientos, movimientosConsolidados, &numMovimientosConsolidados);
            for (int i = 0; i < numMovimientos; i++) {
                printf("Girar: %d grados, Mover %d hacia adelante\n", movimientos[i].giro,movimientos[i].movimiento);
            }
            printf("\n");
            // Imprimir los movimientos consolidados
            for (int i = 0; i < numMovimientosConsolidados; i++) {
                printf("Girar: %d grados, Mover: %d\n", movimientosConsolidados[i].giro, movimientosConsolidados[i].movimiento);
            }
        }
    }

    return 0;
}