"""
Link documentación
https://github.com/ultralytics/ultralytics
https://docs.ultralytics.com/usage/cfg/#predict
"""
from ultralytics import YOLO
import numpy as np
import cv2
from detect_dots import calculateAngle
import os
import shutil

# Path to robot crop
folder = "runs/detect/predict/crops/0"

# Load a model
model = YOLO('best.pt')  # load a custom model

# Eliminar la carpeta "predict"
ruta_predict = os.path.join("runs/detect/predict")
if os.path.exists(ruta_predict):    
    shutil.rmtree(ruta_predict)

# Load an image
image_path = './Robocup-1/valid/images/WhatsApp-Image-2023-12-07-at-5-17-07-PM_jpeg.rf.4f7d25ad9dc004142e91fe61cd1ab967.jpg'
# Draw a circle at the center of each object and add label
image = cv2.imread(image_path)
# Perform object detection using the model
results = model.predict(image_path,conf=0.5,save=True, save_txt=True, save_crop =True)

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
        b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
        coord = np.array(b)
        c = box.cls.item()  # convert class index to Python scalar
        c = int(np.array(c))
        x_center = int((coord[0] + coord[2]) / 2)
        y_center = int((coord[1] + coord[3]) / 2)

        if c == 0:  # Assuming class 0 corresponds to "my robot"
            # Lista todos los archivos en la carpeta
            archivos = os.listdir(folder)
            # Encuentra la imagen (suponiendo que sea una imagen con extensión jpg)
            imagen = next((archivo for archivo in archivos if archivo.endswith(".jpg")), None)
            if imagen:
                ruta_completa = os.path.join(folder, imagen)
                print(f"La imagen se encuentra en: {ruta_completa}")
                # angle = calculateAngle() # Run model 2 to segment front of the robot
                extracted_data["robot"] = [x_center, y_center, calculateAngle(ruta_completa)]
        elif c == 3:  # Assuming class 1 corresponds to "pelota"
            extracted_data["pelota"] = [x_center, y_center]
        else:  # Assuming other classes correspond to "objetos"
            extracted_data["objetos"].append((x_center, y_center))

# Verificar IDs con datos obtenidos
"""         # Draw a circle at the center
        cv2.circle(image, (x_center, y_center), radius=10, color=(0, 255, 0), thickness=2)
        # Add label
        cv2.putText(image, str(c), (x_center + 10, y_center - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Save the image with circles and labels
cv2.imwrite("output_image.jpg", image) """

# Print the extracted data
print(extracted_data)