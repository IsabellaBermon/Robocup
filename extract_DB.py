# Script para descargar localmente la base de datos
from roboflow import Roboflow
from IPython import display
from IPython.display import Image, display

# Instancia de Roboflow
rf = Roboflow(api_key="pxBbr78gPQ3Bv0NKTL7i")
project = rf.workspace("robocup-hdybz").project("robocup-lyidt")
dataset = project.version(1).download("yolov8")