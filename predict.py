from ultralytics import YOLO
import numpy as np
import cv2
import os
import shutil
import json
from mqtt import client,flagPayload

angle_deg = 0

# Load a model
model = YOLO('best.pt')


bin_window_name2 = 'recorte'
cv2.namedWindow(bin_window_name2, cv2.WINDOW_NORMAL)
cv2.resizeWindow(bin_window_name2, 305, 305)

bin_window_name = 'recorte'
cv2.namedWindow(bin_window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(bin_window_name, 305, 305)

bin_window_name3 = 'cancha'
cv2.namedWindow(bin_window_name3, cv2.WINDOW_NORMAL)
cv2.resizeWindow(bin_window_name3, 305, 305)

# Crear una ventana para mostrar el valor de angle_deg
# angle_window_name = 'angulo'
# cv2.namedWindow(angle_window_name, cv2.WINDOW_NORMAL)
# cv2.resizeWindow(angle_window_name, 200, 100)

# Configurar la captura de video desde la cámara
cap = cv2.VideoCapture(1)  # 0 representa la cámara predeterminada

def map_coordinates(x, a, b, c, d):
    """
    Mapea el valor x de un rango [a, b] a un rango [c, d].
    """
    return (x - a) * (d - c) / (b - a) + c
flag=False 
cont=0
while(True):
    cont+=1
    if(cont==30):
        flag=True

    # Capturar frame desde la cámara
    ret, frame = cap.read()
    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Umbralizar para detectar el color negro
    _, mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

    # Invertir la máscara
    mask = cv2.bitwise_not(mask)

    # Encontrar contornos en la máscara
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Encontrar el contorno más grande (el cuadrado)
    max_contour = max(contours, key=cv2.contourArea)

    # Crear una máscara en negro del mismo tamaño que la imagen original
    final_mask = np.zeros_like(frame)

    # Dibujar el contorno en la máscara
    cv2.drawContours(final_mask, [max_contour], -1, (255, 255, 255), thickness=cv2.FILLED)

    # Bitwise-AND entre la imagen original y la nueva máscara
    result_image = cv2.bitwise_and(frame, final_mask)

    # Obtener las coordenadas del rectángulo que rodea al contorno
    x, y, w, h = cv2.boundingRect(max_contour)

    # Recortar la imagen final para que tenga el tamaño del contorno
    result_image_cropped = result_image[y:y + h, x:x + w]

    # Mostrar la imagen final con el contenido dentro del contorno rojo
    #cv2.imshow(bin_window_name3, result_image_cropped)


    # Perform object detection using the model
    results = model.predict(result_image_cropped, conf=0.5, save=False)  # No es necesario guardar los resultados en este caso
    cv2.imshow(bin_window_name3, result_image_cropped)
    # Dictionary to store extracted information
    extracted_data = {
        "robot": [],
        "pelota": [],
        "objetos": []
    }
            
    # Extracct coordinates and class for each detected object
    for r in results:
        boxes = r.boxes
        for box in boxes:
            b = box.xyxy[0]
            coord = np.array(b)
            c = box.cls.item()
            c = int(np.array(c))
            x_center = int((coord[0] + coord[2]) / 2)
            y_center = int((coord[1] + coord[3]) / 2)
            
            # Extraer las coordenadas del bounding box
            #width, height, _ = frame.shape
            x1, y1, x2, y2 = map(int, coord)
            x1 += 1
            x2 += 10
            y1 += 10
            y2 += 10

            # Dibujar el bounding box en la imagen
            color = (0, 255, 0)  # Puedes ajustar el color según tus preferencias
            thickness = 2  # Puedes ajustar el grosor según tus preferencias
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

            # cv2.putText(frame, f'cosa: {coord[0]}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            if c == 0:  # Assuming class 0 corresponds to "my robot"
                    #extracted_data["robot"] = [x_center, y_center, 1]
                    # Definir las coordenadas para el recorte
                    
                    # Recortar la región de interés (ROI)
                    roi = frame[y1:y2, x1:x2]
                    #roi = frame[top:bottom, left:right]
                    # Mostrar el recorte en una ventana separada
                    cv2.imshow(bin_window_name2, roi)

                    # Definir los valores HSV mínimo y máximo para el color azul claro
                    lower_blue = np.array([90, 100, 100])
                    upper_blue = np.array([130, 255, 255])

                    # Convertir la imagen a formato HSV
                    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                    # Aplicar la máscara de color para el color azul claro
                    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
                    result_blue = cv2.bitwise_and(roi, roi, mask=mask_blue)

                    # Convertir la imagen con la máscara de azul a blanco y negro
                    gray = cv2.cvtColor(result_blue, cv2.COLOR_BGR2GRAY)

                    # Aplicar umbral para resaltar objetos
                    _, threshold = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

                    # Encontrar contornos en la imagen umbral
                    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    # Dibujar contornos y encontrar centroides del color azul
                    image_with_contours = roi.copy()
                    centroids_blue = []

                    for contour in contours:
                        # Calcular el centroide del contorno
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])

                            # Dibujar un punto en el centro
                            cv2.circle(image_with_contours, (cx, cy), 3, (0, 255, 255), -1)

                            # Almacenar el centro en la lista de centroides del color azul
                            centroids_blue.append((cx, cy))

                    # Dibujar los centroides rojos y trazar una línea recta entre ellos
                    # Establecer los valores HSV mínimo y máximo para el color rojo
                    lower_red = np.array([166, 66, 149])
                    upper_red = np.array([179, 255, 255])

                    # Aplicar la máscara de color para el color rojo
                    mask_red = cv2.inRange(hsv, lower_red, upper_red)
                    result_red = cv2.bitwise_and(roi, roi, mask=mask_red)

                    # Convertir la imagen con la máscara de rojo a blanco y negro
                    gray_red = cv2.cvtColor(result_red, cv2.COLOR_BGR2GRAY)

                    # Aplicar umbral para resaltar objetos
                    _, threshold_red = cv2.threshold(gray_red, 1, 255, cv2.THRESH_BINARY)

                    # Encontrar contornos en la imagen umbral
                    contours_red, _ = cv2.findContours(threshold_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    # Dibujar contornos y encontrar centroides del color rojo
                    centroids_red = []

                    for contour in contours_red:
                        # Calcular el centroide del contorno
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])

                            # Dibujar un punto en el centro
                            cv2.circle(image_with_contours, (cx, cy), 3, (0, 0, 255), -1)

                            # Almacenar el centro en la lista de centroides del color rojo
                            centroids_red.append((cx, cy))

                    # Trazar una línea recta entre los centroides rojos más cercanos
                    if len(centroids_red) >= 2:
                        min_distance = float('inf')
                        closest_pair = None

                        for i in range(len(centroids_red) - 1):
                            for j in range(i + 1, len(centroids_red)):
                                distance = np.linalg.norm(np.array(centroids_red[i]) - np.array(centroids_red[j]))
                                if distance < min_distance:
                                    min_distance = distance
                                    closest_pair = (centroids_red[i], centroids_red[j])

                        if closest_pair:
                            # Dibujar línea roja entre centroides rojos más cercanos
                            cv2.line(image_with_contours, closest_pair[0], closest_pair[1], (0, 0, 255), 2)

                            # Calcular el punto medio de la línea roja
                            midpoint_red = ((closest_pair[0][0] + closest_pair[1][0]) // 2, (closest_pair[0][1] + closest_pair[1][1]) // 2)

                            # Dibujar el punto medio de la línea roja
                            cv2.circle(image_with_contours, midpoint_red, 3, (0, 255, 0), -1)

                            # Dibujar línea verde entre centroide azul y punto medio de la línea roja
                            if centroids_blue:
                                # Calcular el ángulo entre la línea verde y la horizontal de la cruz
                                delta_x = midpoint_red[0] - centroids_blue[0][0]
                                delta_y = midpoint_red[1] - centroids_blue[0][1]
                                angle_rad = np.arctan2(delta_y, delta_x)
                                angle_deg = (angle_rad * 180) / np.pi

                               
                                # Verificar que angle_deg sea un número entero y está en el rango [0, 360]
                                if isinstance(angle_deg, int) and 0 <= angle_deg <= 360:
                                    # Asignar los valores al diccionario
                                    extracted_data["robot"] = [int(y_center), int(x_center), int(angle_deg)]
                                    print(y_center,x_center)

                                    # Mostrar el valor de angle_deg en la ventana
                                    cv2.putText(frame, f'angulo: {int(angle_deg)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                                else:
                                    # Manejar el caso donde angle_deg no es un entero o no está en el rango deseado
                                    print("El ángulo no es un número entero o no está en el rango [0, 360].")
                                    extracted_data["robot"] = [int(y_center * 0.086), int(x_center * 0.083), 0]
                                # Mostrar la imagen con el valor de angle_deg
                                #cv2.imshow(angle_window_name, frame)
                                
                                # Calcular los puntos de inicio y fin de la línea verde
                                line_length = 500  # Longitud deseada de la línea verde
                                green_line_start = centroids_blue[0]
                                green_line_end = (
                                    int(centroids_blue[0][0] + line_length * np.cos(angle_rad)),
                                    int(centroids_blue[0][1] + line_length * np.sin(angle_rad))
                                )

                                # Dibujar la línea verde
                                cv2.line(image_with_contours, green_line_start, green_line_end, (0, 255, 0), 2)

                            # Dibujar una cruz en el centro del centroide azul
                            if centroids_blue:
                                cross_size = 500
                                cv2.line(image_with_contours, (centroids_blue[0][0] - cross_size, centroids_blue[0][1]),
                                            (centroids_blue[0][0] + cross_size, centroids_blue[0][1]), (100, 0, 100), 2)
                                cv2.line(image_with_contours, (centroids_blue[0][0], centroids_blue[0][1] - cross_size),
                                            (centroids_blue[0][0], centroids_blue[0][1] + cross_size), (100, 0, 100), 2)

                            # Dibujar un círculo alrededor del centroide azul para representar un transportador
                            if centroids_blue:
                                radius = 500
                                cv2.circle(image_with_contours, centroids_blue[0], radius, (255, 0, 0), 2)

                    # Dibujar contornos en la imagen original
                    cv2.drawContours(image_with_contours, contours, -1, (0, 0, 0), 2)

                    # Mostrar la imagen con contornos, centroides y líneas
                    cv2.imshow(bin_window_name, image_with_contours)
                    #cv2.imwrite("output_with_cross.jpg", image_with_contours)

        
            elif c == 3:  # Assuming class 1 corresponds to "pelota"
                extracted_data["pelota"] = [int(y_center*0.086), int(x_center*0.083)]
            elif c==2 or c==1:  # Assuming other classes correspond to "objetos"
                extracted_data["objetos"].append([int(y_center*0.086),int(x_center*0.083)])
                # extracted_data["objetos"].append(int(x_center*0.083))

            # Dibujar un círculo en el centro del objeto detectado
            #cv2.circle(frame, (x_center, y_center), radius=10, color=(0, 255, 0), thickness=2)
            #cv2.rectangle(frame, (x1, y1), (x2, y2), color=(0, 255, 0), thickness=2)

            # Agregar etiqueta
            #cv2.putText(frame, str(c), (x_center + 10, y_center - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Mostrar el frame con los resultados en tiempo real
    cv2.imshow('Object Detection', frame)

    
    if(flag==True):
        if(len(extracted_data["objetos"])>2):
            extracted_data["objetos"].pop()
        # if(len(extracted_data["objetos"])>2):
        #     extracted_data["objetos"].pop()
        print(extracted_data)
        extracted_data = ",".join([",".join(map(str, values)) for values in extracted_data.values()])
        extracted_data=extracted_data.replace("[","").replace("]","")
        extracted_data=extracted_data.replace(" ","")
        print(extracted_data)
        client.publish("robot/posiciones",extracted_data)
        flag=False
        break

    # Salir del bucle si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la captura de video y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()


# # Convertir los valores de cada clave a cadenas y unirlos con comas
# result_string = ",".join([",".join(map(str, values)) for values in extracted_data.values()])
# extracted_data = "2,2,30,10,20,8,8,15,15"
# client.publish("robot/posiciones",extracted_data)
# flagPayload = False
# if(flagPayload):

# extracted_data = ",".join([",".join(map(str, values)) for values in extracted_data.values()])
# client.publish("robot/posiciones",extracted_data)

# flagPayload=False

# # Salir del bucle si se presiona la tecla 'q'
# if cv2.waitKey(1) & 0xFF == ord('q'):
#     break

