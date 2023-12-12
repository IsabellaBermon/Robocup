#include "pathfinder.h"

void initializeGrid(Cell grid[GRID_HEIGHT][GRID_WIDTH], Robot *robot, Ball *ball, Obstacle obstacles[], int numObstacles) {
    for (int i = 0; i < GRID_HEIGHT; ++i) {
        for (int j = 0; j < GRID_WIDTH; ++j) {
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
        grid[obstacleY][obstacleX].type = '#';
    }
}

void printGrid(Cell grid[GRID_HEIGHT][GRID_WIDTH]) {
    for (int i = 0; i < GRID_HEIGHT; ++i) {
        for (int j = 0; j < GRID_WIDTH; ++j) {
            printf("%c ", grid[i][j].type);
        }
        printf("\n");
    }
}

//==========================================================================================================================================



void initializeNodes(Node nodes[GRID_HEIGHT][GRID_WIDTH], Cell grid[GRID_HEIGHT][GRID_WIDTH]) {
    for (int x = 0; x < GRID_HEIGHT; x++) {
        for (int y = 0; y < GRID_WIDTH; y++) {
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
    // return abs(goalX - x) + abs(goalY - y);  // Distancia de Manhattan
    return (int)sqrt((goalX - x)*(goalX - x) + (goalY - y)*(goalY - y)); // Distancia Euclidiana
}

void findNeighbors(Node nodes[GRID_HEIGHT][GRID_WIDTH], int x, int y, Node* neighbors[], int* neighborCount) {
    *neighborCount = 0;
    int dx[8] = {0, 1, 0, -1, -1, 1, -1, 1}; // Todos los nodos alrededor del nodo actual
    int dy[8] = {1, 0, -1, 0, -1, -1, 1, 1};

    for (int i = 0; i < 8; i++) {
        int newX = x + dx[i];
        int newY = y + dy[i];
        if (newX >= 0 && newX < GRID_WIDTH && newY >= 0 && newY < GRID_HEIGHT) { // Se verifica que este dentro de la cuadricula
            if (nodes[newX][newY].type != '#') // Se verifica que no sea obstaculo
            {
                neighbors[(*neighborCount)++] = &nodes[newX][newY]; // Se agregan los vecinos al arreglo
            }      
        }
    }
}

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

void AStar(Node nodes[GRID_HEIGHT][GRID_WIDTH], Point start, Point goal) {
    // Asignacion de nodo de INICIO y nodo OBJETIVO
    Node* startNode = &nodes[(int)start.x][(int)start.y];
    Node* goalNode = &nodes[(int)goal.x][(int)goal.y];

    // Inicializacion de lista ABIERTA (nodos por explorar) y VACIA (nodos ya explorados)
    NodeList openList, closedList;
    NodeList_init(&openList);
    NodeList_init(&closedList);

    // Inicializacion del nodo de INICIO
    startNode->G = 0;
    startNode->H = heuristic(startNode->x, startNode->y, goalNode->x, goalNode->y);

    // Se agrega nodo de INICIO a lista ABIERTA
    NodeList_add(&openList, startNode);

    while (openList.size > 0) { // Mientras la lista ABIERTA no este vacia
        Node* currentNode = NodeList_getLowestF(&openList);

        if (currentNode == goalNode) {
            reconstructPath(goalNode);
            return;
        }

        NodeList_remove(&openList, currentNode);
        NodeList_add(&closedList, currentNode);

        Node* neighbors[8];
        int neighborCount;
        findNeighbors(nodes, currentNode->x, currentNode->y, neighbors, &neighborCount);

        for (int i = 0; i < neighborCount; i++) {
            Node* neighbor = neighbors[i];

            if (NodeList_contains(&closedList, neighbor)) continue;

            int isDiagonal = (abs(currentNode->x - neighbor->x) == 1 && abs(currentNode->y - neighbor->y) == 1);
            int tentativeGScore = currentNode->G + (isDiagonal ? sqrt(2) : 1);
            // int tentativeGScore = currentNode->G + 1;

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