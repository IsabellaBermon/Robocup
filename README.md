# Robot Soccer Controller

## Descripción
Este proyecto es un controlador para un robot de fútbol, diseñado para participar en competiciones de robótica como RoboCup. Utiliza un Raspberry Pi Pico y varios sensores para controlar los movimientos del robot, navegar por el campo, evitar obstáculos y seguir la pelota.

## Características
- Control de movimientos de robots mediante algoritmos de ruta.
- Conexión WiFi para comunicación en tiempo real.
- Integración con MQTT para envío de datos.
- Detección y evasión de obstáculos.

## Requisitos previos
Para utilizar este código, necesitarás:
- Raspberry Pi Pico.
- Sensores y motores compatibles.
- Acceso a una red WiFi.
- Un broker MQTT configurado.

## Configuración del entorno
1. Instala el SDK de Raspberry Pi Pico siguiendo las [instrucciones oficiales](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html).
2. Clona este repositorio en tu máquina local.
3. Configura los parámetros de conexión WiFi y MQTT en el código (`SSID`, `PASSWORD`, dirección del broker, etc.).

## Compilación y ejecución
Para compilar y cargar el programa en tu Raspberry Pi Pico:
1. Navega a la carpeta del proyecto.
2. Ejecuta `cmake .` para preparar el build.
3. Ejecuta `make` para compilar el código.
4. Carga el binario resultante en tu Raspberry Pi Pico.

## Uso
Una vez que el firmware está cargado y el robot está encendido:
1. El robot intentará conectarse a la red WiFi configurada.
2. Una vez conectado, establecerá una conexión MQTT con el broker especificado.
3. El robot empezará a ejecutar los algoritmos de navegación y seguimiento de pelota.

## Contribuciones
Las contribuciones son bienvenidas. Si deseas contribuir al proyecto, por favor, envía un pull request o abre un issue para discutir lo que te gustaría cambiar.

## Licencia
[MIT](https://choosealicense.com/licenses/mit/)
