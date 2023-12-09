import cv2
import numpy as np

# Read image
image = cv2.imread('aaa1.jpg')

# Establecer los valores HSV mínimo y máximo que se mostrarán
lower = np.array([166, 66, 149])
upper = np.array([179, 255, 255])

# Convertir la imagen a formato HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Aplicar la máscara de color
mask = cv2.inRange(hsv, lower, upper)
result = cv2.bitwise_and(image, image, mask=mask)
# cv2.imshow(result)

# Convertir la imagen con máscara a blanco y negro
gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

# Aplicar umbral para resaltar objetos
_, threshold = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

# Encontrar contornos en la imagen umbral
contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Dibujar contornos y encontrar centros
image_with_contours = image.copy()
centroids = []
for contour in contours:
    # Calcular el centroide del contorno
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Dibujar un punto en el centro
        cv2.circle(image_with_contours, (cx, cy), 3, (0, 255, 255), -1)

        # Almacenar el centro en la lista de centroides
        centroids.append((cx, cy))

# Calcular la distancia entre cada par de centroides
min_distance = float('inf')
closest_pair = None

for i in range(len(centroids) - 1):
    for j in range(i + 1, len(centroids)):
        distance = np.linalg.norm(np.array(centroids[i]) - np.array(centroids[j]))
        if distance < min_distance:
            min_distance = distance
            closest_pair = (centroids[i], centroids[j])

# Dibujar una línea que conecta los centros más cercanos
if closest_pair:
    cv2.line(image_with_contours, closest_pair[0], closest_pair[1], (0, 0, 255), 2)
    # # Calcular la inclinación de la línea respecto al eje y
    # dy = closest_pair[1][1] - closest_pair[0][1]
    # dx = closest_pair[1][0] - closest_pair[0][0]
    # angle_y = np.degrees(np.arctan2(dy, dx))

    # # Calcular el ángulo de la línea respecto al eje x
    # angle_x = 90 - angle_y

    # print(f'Inclinación respecto al eje y: {angle_y:.2f} grados')
    # print(f'Ángulo respecto al eje x: {angle_x:.2f} grados')

# Dibujar contornos en la imagen original
cv2.drawContours(image_with_contours, contours, -1, (0, 0, 0), 2)

# Mostrar la imagen con contornos, puntos en el centro y línea entre centros más cercanos
cv2.imshow('Objects Detected with Line', image_with_contours)
cv2.imwrite("output_front.jpg", image_with_contours)
# Imprimir la cantidad de objetos detectados y la distancia mínima
print(f'Cantidad de objetos detectados: {len(contours)}')
print(f'Distancia mínima entre centros: {min_distance}')

# Esperar por un evento de teclado y cerrar la ventana al presionar una tecla
cv2.waitKey(0)
cv2.destroyAllWindows()

# Filtro de HSV para máscara de color
# https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv/48367205#48367205
""" def nothing(x):
    pass

# Load image
image = cv2.imread('aaa.jpg')

# Create a window
cv2.namedWindow('image')

# Create trackbars for color change
# Hue is from 0-179 for Opencv
cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

# Initialize HSV min/max values
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

while(1):
    # Get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin', 'image')
    sMin = cv2.getTrackbarPos('SMin', 'image')
    vMin = cv2.getTrackbarPos('VMin', 'image')
    hMax = cv2.getTrackbarPos('HMax', 'image')
    sMax = cv2.getTrackbarPos('SMax', 'image')
    vMax = cv2.getTrackbarPos('VMax', 'image')

    # Set minimum and maximum HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Convert to HSV format and color threshold
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(image, image, mask=mask)

    # Print if there is a change in HSV value
    if((phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
        print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
        phMin = hMin
        psMin = sMin
        pvMin = vMin
        phMax = hMax
        psMax = sMax
        pvMax = vMax

    # Display result image
    cv2.imshow('image', result)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows() """
