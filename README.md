# Proyecto Robocup Soccer  - Vision Software

Primera versión del software para detección de robots según su ID y la pelota.
<img src= "output_image.jpg" width="500" height="370">

Una vez el modelo identifica el robot, se realizar un procesamiento de imagen en el espacio HSV para extraer los puntos y detectar el frente del robot. Esto con el fin de encontrar el ángulo de inclinación que tiene. 
<img src= "output_front.jpg" width="100" height="100">

## Retorna
 Además de la predicción se extraen los datos de cada objeto en el siguiente formato : 

```json
{
  "robot": [x, y, ángulo],
  "pelota": [x, y],
  "objetos": [ [x, y], [x, y], ...]
}

Se realiza la detección de la cancha de juego

![foto_cancha](https://github.com/IsabellaBermon/Robocup/assets/42179443/d2373f5e-9c71-4d22-b780-be831954317f)

