#include "pathfinder.h"

// void markObjectInGrid(Cell grid[GRID_WIDTH][GRID_HEIGHT], int centerX, int centerY, char type) { // Crea la huella del objeto
//     int halfSize = (OBJECT_SIZE-1)/2; // Para una huella de 7x7, el tamaño de la mitad es 3

//     for (int dx = -halfSize; dx <= halfSize; dx++) {
//         for (int dy = -halfSize; dy <= halfSize; dy++) {
//             int x = centerX + dx;
//             int y = centerY + dy;

//             if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
//                 grid[x][y].type = type;  // Marcar como ocupado por el objeto
//             }
//         }
//     }
// }

// void initializeGrid(Cell grid[GRID_WIDTH][GRID_HEIGHT], Robot *robot, Ball *ball, Obstacle obstacles[], int numObstacles) { // Inicializa la cuadricula de forma visual
//     for (int y = 0; y < GRID_HEIGHT; ++y) {
//         for (int x = 0; x < GRID_WIDTH; ++x) {
//             grid[x][y].type = '.';
//         }
//     }

//     int robotX = (int)round(robot->position.x);
//     int robotY = (int)round(robot->position.y);
//     markObjectInGrid(grid, robotX, robotY, 'R'); // Crea la huella del robot

//     int ballX = (int)round(ball->position.x);
//     int ballY = (int)round(ball->position.y);
//     grid[ballX][ballY].type = 'B';

//     for (int k = 0; k < numObstacles; ++k) {
//         int obstacleX = (int)round(obstacles[k].position.x);
//         int obstacleY = (int)round(obstacles[k].position.y);
//         markObjectInGrid(grid, obstacleX, obstacleY, '#'); // Crea la huella de un obstaculo
//     }
// }

// void printGrid(Cell nodes[GRID_WIDTH][GRID_HEIGHT]) {
//     for (int y = 0; y < GRID_HEIGHT; ++y) {
//         for (int x = 0; x < GRID_WIDTH; ++x) {
//             printf("%c ", nodes[x][y].type);
//         }
//         printf("\n");
//     }
// }
int numMovimientos=0;
int numMovimientosConsolidados =0;
void markObjectInGrid(Node nodes[GRID_WIDTH][GRID_HEIGHT], int centerX, int centerY, char type) { // Crea la huella del objeto
    int halfSize = (OBJECT_SIZE-1)/2; // Para una huella de 7x7, el tamaño de la mitad es 3

    for (int dx = -halfSize; dx <= halfSize; dx++) {
        for (int dy = -halfSize; dy <= halfSize; dy++) {
            int x = centerX + dx;
            int y = centerY + dy;

            if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
                nodes[x][y].type = type;  // Marcar como ocupado por el objeto
            }
        }
    }
}

void printGrid(Node nodes[GRID_WIDTH][GRID_HEIGHT]) {
    for (int y = 0; y < GRID_HEIGHT; ++y) {
        for (int x = 0; x < GRID_WIDTH; ++x) {
            printf("%c ", nodes[x][y].type);
        }
        printf("\n");
    }
}

//==========================================================================================================================================

void initializeNodes(Node nodes[GRID_WIDTH][GRID_HEIGHT], Robot *robot, Ball *ball, Obstacle obstacles[], int numObstacles) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            nodes[x][y].x = x;
            nodes[x][y].y = y;
            nodes[x][y].G = INFINITE_COST;
            nodes[x][y].H = INFINITE_COST;
            nodes[x][y].parent = NULL;
            nodes[x][y].type = '.';
        }
    }

    int robotX = (int)round(robot->position.x);
    int robotY = (int)round(robot->position.y);
    markObjectInGrid(nodes, robotX, robotY, 'R'); // Crea la huella del robot

    int ballX = (int)round(ball->position.x);
    int ballY = (int)round(ball->position.y);
    nodes[ballX][ballY].type = 'B';

    for (int k = 0; k < numObstacles; ++k) {
        int obstacleX = (int)round(obstacles[k].position.x);
        int obstacleY = (int)round(obstacles[k].position.y);
        markObjectInGrid(nodes, obstacleX, obstacleY, '#'); // Crea la huella de un obstaculo
    }
}

int heuristic(int x, int y, int goalX, int goalY) {
    // return abs(goalX - x) + abs(goalY - y);  // Distancia de Manhattan
    return (int)sqrt((goalX - x)*(goalX - x) + (goalY - y)*(goalY - y)); // Distancia Euclidiana
}

void findNeighbors(Node nodes[GRID_WIDTH][GRID_HEIGHT], int x, int y, Node* neighbors[], int* neighborCount) {
    *neighborCount = 0;
    int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1}; // Todos los nodos alrededor del nodo actual
    int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int halfSize = (OBJECT_SIZE-1)/2;

    for (int i = 0; i < 8; i++) {
        int newX = x + dx[i];
        int newY = y + dy[i];

        if (newX >= halfSize && newX < GRID_WIDTH - halfSize && newY >= halfSize && newY < GRID_HEIGHT - halfSize) { // Se verifica que este dentro de la cuadricula
            int canMove = 1;
            // Verificar si alguna parte del robot colisionaría con un obstáculo
            for (int dy = -halfSize; dy <= halfSize && canMove; dy++) {
                for (int dx = -halfSize; dx <= halfSize && canMove; dx++) {
                    if (nodes[newX + dx][newY + dy].type == '#') {
                        canMove = 0;
                    }
                }
            }
            if (canMove) {
                neighbors[(*neighborCount)++] = &nodes[newX][newY];
                // printf("vecinos: %d,%d\n",newX,newY);
            }

            // if (!(nodes[newX + dx[i]*halfSize][newY + dy[i]*halfSize].type == '#'))
            // {
            //     neighbors[(*neighborCount)++] = &nodes[newY][newX];
            // }
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

void reconstructPath(Node* goalNode, NodeList* pathList) {
    Node* current = goalNode;
    while (current != NULL) {
        // printf("(%d, %d) <- ", current->x, current->y);
        NodeList_add(pathList, current);
        current = current->parent;
    }
    // printf("START\n");
}

void NodeList_reverse(NodeList* list) {
    Node* reversedItems[MAX_NODES];
    int index = 0;
    for (int i = list->size - 1; i >= 0; i--) {
        reversedItems[index++] = list->items[i];
    }
    for (int i = 0; i < list->size; i++) {
        list->items[i] = reversedItems[i];
    }
}

void markPathOnGrid(Node nodes[GRID_WIDTH][GRID_HEIGHT], NodeList* pathList, char pathMarker) { // Actualiza la grid con la ruta encontrada
    for (int i = 0; i < pathList->size; i++) {
        Node* node = pathList->items[i];
        if (nodes[node->x][node->y].type != 'B')
        {
            nodes[node->x][node->y].type = pathMarker;
        }
    }
}

void convertPathToMovements(NodeList* pathList, Movimiento* movimientos) {
    int angle = 0; // Angulo inicial del robot

    int setpoint, giro;
    

    for (int i = 1; i < pathList->size; i++) {
        Node* current = pathList->items[i];
        Node* previous = pathList->items[i - 1];

        int deltaX = current->x - previous->x;
        int deltaY = current->y - previous->y;

        // Asignar setpoint basado en la resta de coordenadas
        if (deltaX == -1 && deltaY == -1) setpoint = -135;
        else if (deltaX == 0 && deltaY == -1) setpoint = 180;
        else if (deltaX == 1 && deltaY == -1) setpoint = 135;
        else if (deltaX == -1 && deltaY == 0) setpoint = -90;
        else if (deltaX == 1 && deltaY == 0) setpoint = 90;
        else if (deltaX == -1 && deltaY == 1) setpoint = -45;
        else if (deltaX == 0 && deltaY == 1) setpoint = 0;
        else if (deltaX == 1 && deltaY == 1) setpoint = 45;

        giro = setpoint - angle;
        angle += giro;

        // Guardar el movimiento en el arreglo
        movimientos[numMovimientos].giro = giro;
        movimientos[numMovimientos].movimiento = 1;
        (numMovimientos)++;

        // // Imprimir el movimiento
        // printf("Girar: %d grados, Mover hacia adelante\n", giro);
        
        // if (giro == 0) {
        //     // Si no hay giro, acumular movimiento hacia adelante
        //     forwardCount++;
        // } else {
        //     // Si hay un giro, imprimir movimientos acumulados y reiniciar contador
        //     if (forwardCount > 0) {
        //         printf("Mover hacia adelante %d veces\n", forwardCount);
        //         forwardCount = 0;
        //     }
        //     angle += giro;
        //     printf("Girar: %d grados, Mover hacia adelante\n", giro);
        // }
        
    }
}

void consolidateMovements(NodeList* pathList, Movimiento* movimientosConsolidados) {
    Movimiento movimientos[MAX_NODES];
    convertPathToMovements(pathList,movimientos);
    
    int i = 0;

    while (i < numMovimientos) {
        Movimiento movimientoActual = movimientos[i];
        int movimientoSumado = movimientoActual.movimiento;

        // Sumar movimientos mientras el giro sea cero y no sea el último elemento
        while (i + 1 < numMovimientos && movimientos[i + 1].giro == 0) {
            movimientoSumado += movimientos[i + 1].movimiento;
            i++;
        }

        // Guardar el movimiento consolidado
        movimientosConsolidados[numMovimientosConsolidados].giro = movimientoActual.giro;
        movimientosConsolidados[numMovimientosConsolidados].movimiento = movimientoSumado;
        (numMovimientosConsolidados)++;

        i++;
    }
}


//==========================================================================================================================================

void  AStar(Node nodes[GRID_WIDTH][GRID_HEIGHT], Point start, Point goal, NodeList* pathList) {
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

        // printf("current node: %d,%d\n",currentNode->x,currentNode->y);
        if (currentNode == goalNode) {
            reconstructPath(goalNode, pathList);
            NodeList_reverse(pathList);
            return;
        }

        NodeList_remove(&openList, currentNode);
        NodeList_add(&closedList, currentNode);

        Node* neighbors[8];
        int neighborCount = 0;
        findNeighbors(nodes, currentNode->x, currentNode->y, neighbors, &neighborCount);

        for (int i = 0; i < neighborCount; i++) {
            Node* neighbor = neighbors[i];
            // printf("vecinos: %d,%d\n",neighbors[i]->x,neighbors[i]->y);

            if (NodeList_contains(&closedList, neighbor)) continue;
            
            int isDiagonal = (abs(currentNode->x - neighbor->x) == 1 && abs(currentNode->y - neighbor->y) == 1); 
            int tentativeGScore = currentNode->G + (isDiagonal ? sqrt(2) : 1);

            if (!NodeList_contains(&openList, neighbor)) {
                NodeList_add(&openList, neighbor);
            } else if (tentativeGScore >= neighbor->G) {
                continue;
            }

            neighbor->parent = currentNode;
            neighbor->G = tentativeGScore;
            neighbor->H = heuristic(neighbor->x, neighbor->y, goalNode->x, goalNode->y);
        }
        //printf("\n");
    }
}