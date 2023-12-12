#include "pico/stdlib.h"
#include <stdio.h>
#include "pathfinder.h"
#include <tusb.h> // TinyUSB tud_cdc_connected()

int main() {
    stdio_init_all();

    // wait until the CDC ACM (serial port emulation) is connected
    while (!tud_cdc_connected()) 
    {
        sleep_ms(10);
    }

    Robot robot = {{0, 0}, 0};  // Example initial position for the robot
    Ball ball = {{7.5, 7.5}};        // Example ball position
    Obstacle obstacles[NUM_OBSTACLES] = { {{2, 2}}, {{4, 4}}, {{6, 6}}, {{9, 9}}};  // Example obstacle positions

    Cell grid[GRID_HEIGHT][GRID_WIDTH];
    Node nodes[GRID_HEIGHT][GRID_WIDTH];
    initializeGrid(grid, &robot, &ball, obstacles, NUM_OBSTACLES);
    initializeNodes(nodes, grid);

    printf("Initial Environment:\n");
    printGrid(grid);

    Point start = {round(robot.position.x), round(robot.position.y)};
    Point goal = {round(ball.position.x), round(ball.position.y)};
    AStar(nodes, start, goal);

    while (1)
    {
        if(getchar()) {
            printf("Initial Environment:\n");
            printGrid(grid);
            AStar(nodes, start, goal);
        }
    }

    return 0;
}