# Telemetría

## Descripción

En el contexto de las evaluaciones de Telemetría, se procedió a implementar la tecnología Bluetooth Low Energy (BLE) en la plataforma Raspberry Pi Pico W. En una fase inicial, se lleva a cabo la inicialización y configuración del controlador CYW43, encargado de la gestión de la conectividad Bluetooth del dispositivo. Posteriormente, se procede a la inicialización de servicios Bluetooth, específicamente el Servicio de Perfil de Puerto Serie (SPP), permitiendo así la instauración de una comunicación serial entre dispositivos. En este proceso, se realizan ajustes en los parámetros de anuncio Bluetooth, los cuales abarcan la designación del dispositivo y el identificador UUID del servicio. Se hace uso de funciones de modulación por ancho de pulsos (PWM) con el propósito de regular la velocidad y dirección de los motores. En el ámbito del procesamiento de datos, se implementa un protocolo de manipulación de paquetes Bluetooth, facilitando la interpretación de datos recibidos a través del servicio SPP mencionado con anterioridad. Con el fin de discernir la acción de giro, se lleva a cabo un procesamiento específico, consistente en la identificación de una cadena 'G' acompañada del correspondiente valor de giro.

## Archivos Principales

### `bt_functions.h`

Este archivo contiene las declaraciones de funciones y definiciones necesarias para el manejo de Bluetooth.

### `mygatt.h`

Contiene las definiciones y estructuras relacionadas con el servicio GATT utilizado en la comunicación Bluetooth.

### `main.c`

El archivo principal del proyecto, `main.c`, contiene la lógica central para la inicialización de Bluetooth, manejo de paquetes HCI y el control del robot en base a los comandos recibidos.

## Variables y Funciones Principales

### Variables de Estado

- `angleBt`: Almacena el ángulo recibido desde la aplicación móvil.
- `distanceBt`: Almacena la distancia recibida desde la aplicación móvil.
- `angleTurnBt`: Almacena el ángulo de giro recibido para movimientos circulares.
- `radioBt`: Almacena el radio de movimiento recibido para movimientos circulares.
- `btAvailable`: Indica si el Bluetooth está disponible para recibir nuevos comandos.
- `banAngle`, `banDistance`, `banCircularMovement`: Banderas que indican la recepción de comandos específicos.

### Funciones Principales

#### `initBluetooth()`

- Inicializa la pila Bluetooth y configura parámetros de anuncio.
- Inicia el servicio SPP (Serial Port Profile) para la comunicación.
- Establece parámetros de anuncio y habilita el anuncio Bluetooth.

#### `hci_packet_handler()`

- Maneja los eventos HCI, especialmente el evento de desconexión completa.

#### `nordic_spp_packet_handler()`

- Maneja los eventos relacionados con el servicio SPP.
- Interpreta y procesa los datos recibidos, estableciendo variables de estado según el tipo de comando.

## Procedimiento

1. **Inicialización de Bluetooth**: Llamada a `initBluetooth()` en la inicialización del sistema.
2. **Manejo de Eventos HCI**: La función `hci_packet_handler()` gestiona eventos HCI, como la desconexión del dispositivo Bluetooth.
3. **Manejo de Eventos SPP**: La función `nordic_spp_packet_handler()` maneja eventos del servicio SPP, interpretando comandos y actualizando las variables de estado del robot.

## Integración y Uso

Las características de BLE, que emplea perfiles y servicios Generic Attribute Profile (GATT) para estructurar y describir la información compartida entre dispositivos, se procede a la compilación del archivo respectivo junto con el proyecto en cuestión. Es importante resaltar que la aplicación desarrollada mediante App Inventor ha sido concebida con valores UUID idénticos, garantizando así una conexión coherente con la Raspberry Pi Pico W. Asimismo, se destaca que la elección de la tecnología BLE se fundamenta en su capacidad para reducir el consumo energético, aspecto crucial para dispositivos alimentados por batería.
