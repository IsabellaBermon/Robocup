#include "pathfinder.h"

int numMovimientos=0; ///< Contador global de movimientos.
int numMovimientosConsolidados =0; ///< Contador global de movimientos consolidados.

/**
 * @brief Marca la huella de un objeto en la cuadrícula.
 * 
 * @param nodes Matriz de nodos que representa la cuadrícula.
 * @param centerX Coordenada X del centro del objeto.
 * @param centerY Coordenada Y del centro del objeto.
 * @param type Carácter que representa el tipo de objeto.
 */
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

/**
 * @brief Imprime el estado actual de la cuadrícula.
 * 
 * @param nodes Matriz de nodos que representa la cuadrícula.
 */
void printGrid(Node nodes[GRID_WIDTH][GRID_HEIGHT]) {
    for (int y = 0; y < GRID_HEIGHT; ++y) {
        for (int x = 0; x < GRID_WIDTH; ++x) {
            printf("%c ", nodes[x][y].type);
        }
        printf("\n");
    }
}

/**
 * @brief Inicializa los nodos de la cuadrícula, configurando obstáculos, robot y pelota.
 * 
 * @param nodes Matriz de nodos que representa la cuadrícula.
 * @param robot Puntero al robot en la cuadrícula.
 * @param ball Puntero a la pelota en la cuadrícula.
 * @param obstacles Array de obstáculos en la cuadrícula.
 * @param numObstacles Número de obstáculos en el array.
 */
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

/**
 * @brief Calcula la heurística (distancia euclidiana) entre dos puntos.
 * 
 * @param x Coordenada X del punto inicial.
 * @param y Coordenada Y del punto inicial.
 * @param goalX Coordenada X del punto objetivo.
 * @param goalY Coordenada Y del punto objetivo.
 * @return int Valor de la heurística entre los dos puntos.
 */
int heuristic(int x, int y, int goalX, int goalY) {
    // return abs(goalX - x) + abs(goalY - y);  // Distancia de Manhattan
    return (int)sqrt((goalX - x)*(goalX - x) + (goalY - y)*(goalY - y)); // Distancia Euclidiana
}

/**
 * @brief Encuentra los vecinos accesibles de un nodo dado.
 * 
 * @param nodes Matriz de nodos que representa la cuadrícula.
 * @param x Coordenada X del nodo actual.
 * @param y Coordenada Y del nodo actual.
 * @param neighbors Array para almacenar los nodos vecinos encontrados.
 * @param neighborCount Puntero para almacenar el conteo de vecinos encontrados.
 */
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

/**
 * @brief Inicializa una lista de nodos.
 * 
 * @param list Puntero a la lista de nodos a inicializar.
 */
void NodeList_init(NodeList* list) {
    list->size = 0;
}

/**
 * @brief Agrega un nodo a la lista de nodos.
 * 
 * @param list Puntero a la lista de nodos.
 * @param node Nodo a agregar a la lista.
 */
void NodeList_add(NodeList* list, Node* node) {
    if (list->size < MAX_NODES) {
        list->items[list->size++] = node;
    }


/**
 * @brief Verifica si un nodo específico está contenido en la lista de nodos.
 * 
 * @param list Puntero a la lista de nodos.
 * @param node Nodo a verificar en la lista.
 * @return int Retorna 1 si el nodo está en la lista, de lo contrario 0.
 */
int NodeList_contains(NodeList* list, Node* node) {
    for (int i = 0; i < list->size; i++) {
        if (list->items[i] == node) {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Elimina un nodo de la lista de nodos.
 * 
 * @param list Puntero a la lista de nodos.
 * @param node Nodo a eliminar de la lista.
 */
void NodeList_remove(NodeList* list, Node* node) {
    for (int i = 0; i < list->size; i++) {
        if (list->items[i] == node) {
            list->items[i] = list->items[--list->size];
            return;
        }
    }
}

/**
 * @brief Obtiene el nodo con el valor F más bajo en la lista de nodos.
 * 
 * @param list Puntero a la lista de nodos.
 * @return Node* Puntero al nodo con el valor F más bajo.
 */
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

/**
 * @brief Reconstruye el camino desde el nodo objetivo hasta el inicio.
 * 
 * @param goalNode Nodo objetivo desde el cual empezar la reconstrucción.
 * @param pathList Lista para almacenar el camino reconstruido.
 */
void reconstructPath(Node* goalNode, NodeList* pathList) {
    Node* current = goalNode;
    while (current != NULL) {
        // printf("(%d, %d) <- ", current->x, current->y);
        NodeList_add(pathList, current);
        current = current->parent;
    }
    // printf("START\n");
}

/**
 * @brief Invierte el orden de los nodos en una lista de nodos.
 * 
 * @param list Puntero a la lista de nodos a invertir.
 */
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

/**
 * @brief Marca el camino encontrado en la cuadrícula.
 * 
 * @param nodes Matriz de nodos que representa la cuadrícula.
 * @param pathList Lista que contiene los nodos del camino.
 * @param pathMarker Carácter para marcar el camino en la cuadrícula.
 */
void markPathOnGrid(Node nodes[GRID_WIDTH][GRID_HEIGHT], NodeList* pathList, char pathMarker) { // Actualiza la grid con la ruta encontrada
    for (int i = 0; i < pathList->size; i++) {
        Node* node = pathList->items[i];
        if (nodes[node->x][node->y].type != 'B')
        {
            nodes[node->x][node->y].type = pathMarker;
        }
    }
}

/**
 * @brief Convierte el camino en una secuencia de movimientos.
 * 
 * @param pathList Lista que contiene los nodos del camino.
 * @param movimientos Array para almacenar los movimientos calculados.
 */
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
        
    }
}

/**
 * @brief Consolida movimientos secuenciales en una dirección en un único movimiento.
 * 
 * @param pathList Lista que contiene los nodos del camino.
 * @param movimientosConsolidados Array para almacenar los movimientos consolidados.
 */
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

/**
 * @brief Implementa el algoritmo A* para encontrar el camino más corto en una cuadrícula.
 * 
 * @param nodes Matriz de nodos que representa la cuadrícula.
 * @param start Punto de inicio en la cuadrícula.
 * @param goal Punto objetivo en la cuadrícula.
 * @param pathList Lista para almacenar el camino encontrado.
 */
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
