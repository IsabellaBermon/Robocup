#include <stdio.h>
#include <stdlib.h>
#include "pathfinder.h"

//==========================================================================================================================================

int main() {
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

    return 0;
}