#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define GRID_SIZE 10
#define NUM_OBSTACLES 4
#define INFINITE_COST 99999
#define MAX_NODES 100  // Un número arbitrario que sea suficientemente grande

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
            grid[i][j].type = '.';
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

//==========================================================================================================================================

typedef struct Node {
    int x, y;           // Coordinates of the node
    int G, H;           // G and H costs for A*
    struct Node* parent; // Parent node for path tracking
    char type;          // 'O' for obstacle, 'R' for robot, 'B' for ball, ' ' for open
} Node;

void initializeNodes(Node nodes[][GRID_SIZE], Cell grid[][GRID_SIZE]) {
    for (int x = 0; x < GRID_SIZE; x++) {
        for (int y = 0; y < GRID_SIZE; y++) {
            nodes[x][y].x = x;
            nodes[x][y].y = y;
            nodes[x][y].G = INFINITE_COST;
            nodes[x][y].H = INFINITE_COST;
            nodes[x][y].parent = NULL;
            nodes[x][y].type = grid[x][y].type;
        }
    }
}

int heuristic(int x, int y, int goalX, int goalY) {
    return abs(goalX - x) + abs(goalY - y);  // Distancia de Manhattan
    // return (int)sqrt((goalX - x)*(goalX - x) + (goalY - y)*(goalY - y)); // Distancia Euclidiana
}

void findNeighbors(Node nodes[][GRID_SIZE], int x, int y, Node* neighbors[], int* neighborCount) {
    *neighborCount = 0;
    int dx[4] = {0, 1, 0, -1};
    int dy[4] = {1, 0, -1, 0};

    for (int i = 0; i < 4; i++) {
        int newX = x + dx[i];
        int newY = y + dy[i];
        if (newX >= 0 && newX < GRID_SIZE && newY >= 0 && newY < GRID_SIZE) {
            if (nodes[newX][newY].type != 'O')
            {
                neighbors[(*neighborCount)++] = &nodes[newX][newY];
            }      
        }
    }
}

typedef struct {
    Node* items[MAX_NODES];
    int size;
} NodeList;

void NodeList_init(NodeList* list) {
    list->size = 0;
}

void NodeList_add(NodeList* list, Node* node) {
    if (list->size < MAX_NODES) {
        list->items[list->size++] = node;
    }
}

int NodeList_contains(NodeList* list, Node* node) {
    for (int i = 0; i < list->size; i++) {
        if (list->items[i] == node) {
            return 1;
        }
    }
    return 0;
}

void NodeList_remove(NodeList* list, Node* node) {
    for (int i = 0; i < list->size; i++) {
        if (list->items[i] == node) {
            list->items[i] = list->items[--list->size];
            return;
        }
    }
}

Node* NodeList_getLowestF(NodeList* list) {
    Node* lowestFNode = NULL;
    int lowestF = INFINITE_COST;

    for (int i = 0; i < list->size; i++) {
        int F = list->items[i]->G + list->items[i]->H;
        if (F < lowestF) {
            lowestF = F;
            lowestFNode = list->items[i];
        }
    }
    return lowestFNode;
}

void reconstructPath(Node* goalNode) {
    Node* current = goalNode;
    while (current != NULL) {
        printf("(%d, %d) <- ", current->x, current->y);
        current = current->parent;
    }
    printf("START\n");
}

void AStar(Node nodes[][GRID_SIZE], Point start, Point goal) {
    Node* startNode = &nodes[(int)start.x][(int)start.y];
    Node* goalNode = &nodes[(int)goal.x][(int)goal.y];

    NodeList openList, closedList;
    NodeList_init(&openList);
    NodeList_init(&closedList);

    startNode->G = 0;
    startNode->H = heuristic(startNode->x, startNode->y, goalNode->x, goalNode->y);

    NodeList_add(&openList, startNode);

    while (openList.size > 0) {
        Node* currentNode = NodeList_getLowestF(&openList);

        if (currentNode == goalNode) {
            reconstructPath(goalNode);
            return;
        }

        NodeList_remove(&openList, currentNode);
        NodeList_add(&closedList, currentNode);

        Node* neighbors[4];
        int neighborCount;
        findNeighbors(nodes, currentNode->x, currentNode->y, neighbors, &neighborCount);

        for (int i = 0; i < neighborCount; i++) {
            Node* neighbor = neighbors[i];

            if (NodeList_contains(&closedList, neighbor)) continue;

            int tentativeGScore = currentNode->G + 1;

            if (!NodeList_contains(&openList, neighbor)) {
                NodeList_add(&openList, neighbor);
            } else if (tentativeGScore >= neighbor->G) {
                continue;
            }

            neighbor->parent = currentNode;
            neighbor->G = tentativeGScore;
            neighbor->H = heuristic(neighbor->x, neighbor->y, goalNode->x, goalNode->y);
        }
    }
}



//==========================================================================================================================================

int main() {
    Robot robot = {0, 0, 0};  // Example initial position for the robot
    Ball ball = {7.5, 7.5};        // Example ball position
    Obstacle obstacles[NUM_OBSTACLES] = { {{2, 2}}, {{4, 4}}, {{6, 6}}, {{9, 9}}};  // Example obstacle positions

    Cell grid[GRID_SIZE][GRID_SIZE];
    Node nodes[GRID_SIZE][GRID_SIZE];
    initializeGrid(grid, &robot, &ball, obstacles, NUM_OBSTACLES);
    initializeNodes(nodes, grid);

    printf("Initial Environment:\n");
    printGrid(grid);

    Point start = {round(robot.position.x), round(robot.position.y)};
    Point goal = {round(ball.position.x), round(ball.position.y)};
    AStar(nodes, start, goal);

    // Implement your A* algorithm here to find the fastest route in the grid

    return 0;
}