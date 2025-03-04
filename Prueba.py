from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import cv2
import numpy as np

# Conectar al dron
vehicle = connect('127.0.0.1:14550', wait_ready=True)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Envía comandos de velocidad en el sistema de coordenadas NED (Norte, Este, Abajo).
    - velocity_x: Adelante (+) / Atrás (-)
    - velocity_y: Derecha (+) / Izquierda (-)
    - velocity_z: Subir (-) / Bajar (+)
    - duration: Tiempo en segundos
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (no usado)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Sistema de coordenadas NED
        0b0000111111000111,  # type_mask (solo velocidad)
        0, 0, 0,  # Posición (no usada)
        velocity_x, velocity_y, velocity_z,  # Velocidades en m/s
        0, 0, 0,  # Aceleraciones (no usadas)
        0, 0  # yaw, yaw_rate (no usadas)
    )

    for _ in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

    # Detener el movimiento después de la duración
    send_ned_velocity(0, 0, 0, 1)

def simular_despegue():
    """
    Simula el despegue del dron sin que realmente vuele.
    """
    print("Simulando despegue...")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")  # Modo sin GPS
    vehicle.armed = True  # Armar el dron
    time.sleep(2)  # Esperar a que el dron esté listo
    print("Dron listo para moverse.")

def detectar_persona():
    """
    Detecta la posición de una persona en la imagen utilizando YOLO.
    Retorna:
    - (x, y): Coordenadas del centroide de la persona.
    - None: Si no se detecta ninguna persona.
    """
    # Cargar el modelo YOLO
    net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # Cargar las clases (persona es la clase 0 en YOLO)
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]

    # Capturar imagen desde la cámara
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if not ret:
        return None

    # Preprocesar la imagen para YOLO
    height, width, channels = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Procesar las detecciones
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5 and class_id == 0:  # Solo personas (class_id = 0)
                # Obtener las coordenadas de la caja delimitadora
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Calcular las esquinas de la caja delimitadora
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Aplicar Non-Max Suppression para eliminar detecciones redundantes
    indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    if len(indices) > 0:
        for i in indices.flatten():
            x, y, w, h = boxes[i]
            center_x = x + w // 2
            center_y = y + h // 2
            return (center_x, center_y)

    return None

def simular_seguimiento():
    """
    Simula el seguimiento de una persona utilizando YOLO.
    """
    print("Simulando seguimiento...")
    while True:
        posicion = detectar_persona()
        if posicion is None:
            print("No se detecta ninguna persona.")
            send_ned_velocity(0, 0, 0, 1)  # Detener el dron
            continue

        x, y = posicion
        frame_center_x = 320  # Asumiendo una resolución de 640x480
        frame_center_y = 240

        if x < frame_center_x - 50:  # Persona a la izquierda
            print("Moviendo a la izquierda...")
            send_ned_velocity(0, -1, 0, 1)
        elif x > frame_center_x + 50:  # Persona a la derecha
            print("Moviendo a la derecha...")
            send_ned_velocity(0, 1, 0, 1)
        else:  # Persona en el centro
            print("Moviendo hacia adelante...")
            send_ned_velocity(1, 0, 0, 1)

# **Ejecutar la simulación**
try:
    simular_despegue()  # Simular el despegue
    simular_seguimiento()  # Simular el seguimiento
except KeyboardInterrupt:
    print("Simulación detenida por el usuario.")
finally:
    # Desarmar el dron y cerrar la conexión
    vehicle.armed = False
    vehicle.close()
    print("Simulación completada.")
