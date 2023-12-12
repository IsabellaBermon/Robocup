#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define GRID_HEIGHT 10
#define GRID_WIDTH 10
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

void initializeGrid(Cell grid[GRID_HEIGHT][GRID_WIDTH], Robot *robot, Ball *ball, Obstacle obstacles[], int numObstacles);

void printGrid(Cell grid[GRID_HEIGHT][GRID_WIDTH]);

void initializeNodes(Node nodes[GRID_HEIGHT][GRID_WIDTH], Cell grid[GRID_HEIGHT][GRID_WIDTH]);

int heuristic(int x, int y, int goalX, int goalY);

void findNeighbors(Node nodes[GRID_HEIGHT][GRID_WIDTH], int x, int y, Node *neighbors[], int *neighborCount);

void NodeList_init(NodeList *list);

void NodeList_add(NodeList *list, Node *node);

int NodeList_contains(NodeList *list, Node *node);

void NodeList_remove(NodeList *list, Node *node);

Node *NodeList_getLowestF(NodeList *list);

void reconstructPath(Node *goalNode);

void AStar(Node nodes[GRID_HEIGHT][GRID_WIDTH], Point start, Point goal);
