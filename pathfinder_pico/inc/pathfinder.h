#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define GRID_WIDTH 40
#define GRID_HEIGHT 50
#define NUM_OBSTACLES 3
#define INFINITE_COST 99999
#define MAX_NODES 200 // Un número arbitrario que sea suficientemente grande
#define OBJECT_SIZE 3


extern int numMovimientos;
extern int numMovimientosConsolidados;
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

typedef struct Node {
    int x, y;           // Coordinates of the node
    int G, H;           // G and H costs for A*
    struct Node* parent; // Parent node for path tracking
    char type;          // 'O' for obstacle, 'R' for robot, 'B' for ball, ' ' for open
} Node;

typedef struct {
    Node* items[MAX_NODES];
    int size;
} NodeList;

typedef struct {
    int giro;
    int movimiento; // siempre será 1 en este caso

} Movimiento;


// void markObjectInGrid(Cell grid[GRID_WIDTH][GRID_HEIGHT], int centerX, int centerY, char type);

void markObjectInGrid(Node nodes[GRID_WIDTH][GRID_HEIGHT], int centerX, int centerY, char type);

// void initializeGrid(Cell grid[GRID_WIDTH][GRID_HEIGHT], Robot *robot, Ball *ball, Obstacle obstacles[], int numObstacles);

// void printGrid(Cell grid[GRID_WIDTH][GRID_HEIGHT]);

void printGrid(Node nodes[GRID_WIDTH][GRID_HEIGHT]);

void initializeNodes(Node nodes[GRID_WIDTH][GRID_HEIGHT], Robot *robot, Ball *ball, Obstacle obstacles[], int numObstacles);

int heuristic(int x, int y, int goalX, int goalY);

void findNeighbors(Node nodes[GRID_WIDTH][GRID_HEIGHT], int x, int y, Node *neighbors[], int *neighborCount);

void NodeList_init(NodeList *list);

void NodeList_add(NodeList *list, Node *node);

int NodeList_contains(NodeList *list, Node *node);

void NodeList_remove(NodeList *list, Node *node);

Node *NodeList_getLowestF(NodeList *list);

void reconstructPath(Node* goalNode, NodeList* pathList);

void NodeList_reverse(NodeList *list);

void markPathOnGrid(Node nodes[GRID_WIDTH][GRID_HEIGHT], NodeList *pathList, char pathMarker);

// void markPathOnGrid(Cell grid[GRID_WIDTH][GRID_HEIGHT], NodeList *pathList, char pathMarker);

void convertPathToMovements(NodeList *pathList, Movimiento* movimientos);

void consolidateMovements(NodeList* pathList, Movimiento *movimientosConsolidados);

void AStar(Node nodes[GRID_WIDTH][GRID_HEIGHT], Point start, Point goal, NodeList* pathList);
