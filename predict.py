from ultralytics import YOLO
import numpy as np
import cv2
import os
import shutil
import json
#from detect_dots import calculateAngle

# Path to robot crop
#folder = "runs/detect/predict/crops/0"

# Load a model
model = YOLO('C:/Users/Daniela Cuartas/Documents/UdeA/Semestre 9/Embebidos/best.pt')

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
angle_window_name = 'Ángulo'
cv2.namedWindow(angle_window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(angle_window_name, 200, 100)

# Configurar la captura de video desde la cámara
cap = cv2.VideoCapture("video3.mp4")  # 0 representa la cámara predeterminada

while True:
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

            if c == 0:  # Assuming class 0 corresponds to "my robot"
                    extracted_data["robot"] = [x_center, y_center, 1]
                    # Definir las coordenadas para el recorte
                    left = int(coord[0])
                    top = int(coord[1])
                    right = int(coord[2])
                    bottom = int(coord[3])
                    
                    # Recortar la región de interés (ROI)
                    roi = frame[top:bottom, left:right]
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

                                # Ajustar el ángulo para que esté en el rango [0, 360] en sentido antihorario
                                angle_deg = 360 - angle_deg if angle_deg > 0 else -angle_deg
                                angle_deg = angle_deg - 180
                                #print(int(angle_deg))
                                # Mostrar el valor de angle_deg en la ventana
                                cv2.putText(frame, f'Ángulo: {int(angle_deg)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                                
                                # Mostrar la imagen con el valor de angle_deg
                                cv2.imshow(angle_window_name, frame)
                                
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
                extracted_data["pelota"] = [x_center, y_center]
            else:  # Assuming other classes correspond to "objetos"
                extracted_data["objetos"].append((x_center, y_center))

            # Dibujar un círculo en el centro del objeto detectado
            cv2.circle(frame, (x_center, y_center), radius=10, color=(0, 255, 0), thickness=2)

            # Agregar etiqueta
            cv2.putText(frame, str(c), (x_center + 10, y_center - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Mostrar el frame con los resultados en tiempo real
    cv2.imshow('Object Detection', frame)

    # Convertir el diccionario a formato JSON
    json_data = json.dumps(extracted_data, indent=2)

    # Imprimir el JSON resultante
    print(json_data)

    # Salir del bucle si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la captura de video y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()
