#include <stdio.h>
#include <math.h>

// Dimensiones del robot
#define L 0.1  // Ancho entre las ruedas en metros
#define R 0.05 // Radio de las ruedas en metros

// Estructura para almacenar la posición del robot
struct Robot_stats {
    uint16_t x;      // Posición en el eje X
    uint16_t y;      // Posición en el eje Y
    uint16_t theta;  // Ángulo en radianes
};

// Estructura para almacenar las lecturas de encoder de cada rueda
struct stuct_Encoder {
    int counts;    // Cuentas del encoder
    uint16_t last_time; // Tiempo de la última lectura
};

// Función para inicializar un encoder
void init_Encoder(struct stuct_Encoder *encoder) {
    encoder->counts = 0;
    encoder->last_time = 0.0;
}

// Función para leer del encoder y calcular la velocidad angular
uint16_t update_angular_velocity(struct stuct_Encoder *encoder, int new_counts, uint16_t current_time) {
    
    // Calcula el tiempo transcurrido desde la última lectura
    uint16_t dt = current_time - encoder->last_time;
    
    // Calcula la velocidad angular en radianes por segundo
    uint16_t angular_velocity = ((new_counts - encoder->counts) / 360.0) / dt;

    // Actualiza las cuentas y el tiempo de la última lectura
    encoder->counts = new_counts;
    encoder->last_time = current_time;

    return angular_velocity;
}

// Función para actualizar la posición del robot en función de las velocidades de las ruedas
void update_Odometry(struct Robot_stats *robot, uint16_t v1, uint16_t v2, uint16_t v3, uint16_t v4, uint16_t dt) {
    
    // Calcula las velocidades lineales de cada rueda
    uint16_t vl1 = v1 / R;
    uint16_t vl2 = v2 / R;
    uint16_t vl3 = v3 / R;
    uint16_t vl4 = v4 / R;

    // Calcula la velocidad lineal promedio
    uint16_t v = (vl1 + vl2 + vl3 + vl4) / 4.0;

    // Calcula la velocidad angular promedio
    uint16_t w = (vl1 - vl2 - vl3 + vl4) / (4.0 * L);

    // Actualiza la posición y la orientación del robot
    robot->x += v * cos(robot->theta) * dt;
    robot->y += v * sin(robot->theta) * dt;
    robot->theta += w * dt;

    // Comprobar que la orientación esté en el rango [0, 2*pi]
    if (robot->theta < 0) {
        robot->theta += 2.0 * M_PI;
    } else if (robot->theta >= 2.0 * M_PI) {
        robot->theta -= 2.0 * M_PI;
    }
}

int main() {
    // Inicializa la posición y la orientación del robot
    struct Robot_stats robot;
    robot.x = 0.0;
    robot.y = 0.0;
    robot.theta = 0.0;

    // Inicializa los encoders para cada rueda
    struct stuct_Encoder encoder1, encoder2, encoder3, encoder4;
    init_Encoder(&encoder1);
    init_Encoder(&encoder2);
    init_Encoder(&encoder3);
    init_Encoder(&encoder4);

    // Supuesta lectura de encoder de las ruedas
    int new_counts1 = 1000;
    int new_counts2 = 1200;
    int new_counts3 = 1050;
    int new_counts4 = 1250;

    // Este es el tiempo actual en segundos
    // NOTA : hay que hacer un timer para leer cada x tiempo
    uint16_t current_time = 1.0;

    // Calcula las velocidades angulares de las ruedas a partir de las lecturas de encoder
    uint16_t angular_velocity1 = update_angular_velocity(&encoder1, new_counts1, current_time);
    uint16_t angular_velocity2 = update_angular_velocity(&encoder2, new_counts2, current_time);
    uint16_t angular_velocity3 = update_angular_velocity(&encoder3, new_counts3, current_time);
    uint16_t angular_velocity4 = update_angular_velocity(&encoder4, new_counts4, current_time);

    // Tiempo transcurrido (puedes ajustarlo según tus necesidades)
    uint16_t dt = current_time - encoder1.last_time;

    // Actualiza la odometría del robot
    update_Odometry(&robot, angular_velocity1, angular_velocity2, angular_velocity3, angular_velocity4, dt);

    // Imprime la posición y la orientación
    printf("Posición (X, Y): (%lf, %lf)\n", robot.x, robot.y);
    printf("Orientación (Theta): %lf radianes\n", robot.theta);

    return 0;
}
