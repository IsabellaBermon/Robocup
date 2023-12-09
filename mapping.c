#include <stdio.h>
#include <math.h>

#define GRID_SIZE 10
#define NUM_OBSTACLES 4

typedef struct {
    float x;
    float y;
} Point;

typedef struct {
    Point position;
    float angle;
} Robot;

typedef struct {
    Point position;
} Obstacle;

typedef struct {
    Point position;
} Ball;

typedef struct {
    char type;  // 'O' for obstacle, 'R' for robot, 'B' for ball, ' ' for open
} Cell;

void initializeGrid(Cell grid[][GRID_SIZE], Robot *robot, Ball *ball, Obstacle obstacles[], int numObstacles) {
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            grid[i][j].type = ' ';
        }
    }

    int robotX = (int)round(robot->position.x);
    int robotY = (int)round(robot->position.y);
    grid[robotY][robotX].type = 'R';

    int ballX = (int)round(ball->position.x);
    int ballY = (int)round(ball->position.y);
    grid[ballY][ballX].type = 'B';
    printf("%d,%d\n",ballX,ballY);

    for (int k = 0; k < numObstacles; ++k) {
        int obstacleX = (int)round(obstacles[k].position.x);
        int obstacleY = (int)round(obstacles[k].position.y);
        grid[obstacleY][obstacleX].type = 'O';
    }
}

void printGrid(Cell grid[][GRID_SIZE]) {
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            printf("%c ", grid[i][j].type);
        }
        printf("\n");
    }
}

int main() {
    Robot robot = {0, 0, 0};  // Example initial position for the robot
    Ball ball = {7.5, 7.5};        // Example ball position
    Obstacle obstacles[NUM_OBSTACLES] = { {{2, 2}}, {{4, 4}}, {{6, 6}}, {{9, 9}}};  // Example obstacle positions

    Cell grid[GRID_SIZE][GRID_SIZE];
    initializeGrid(grid, &robot, &ball, obstacles, NUM_OBSTACLES);

    printf("Initial Environment:\n");
    printGrid(grid);

    // Implement your A* algorithm here to find the fastest route in the grid

    return 0;
}