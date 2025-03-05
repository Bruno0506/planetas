from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import cv2
import numpy as np

# Conectar al dron a través de la Navio2
try:
    print("Conectando al dron...")
    vehicle = connect('udpin:0.0.0.0:14550', wait_ready=True)
    print("Conexión exitosa!")
except Exception as e:
    print(f"Error al conectar: {e}")
    exit()

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Envía comandos de velocidad en el sistema de coordenadas NED (Norte, Este, Abajo).
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,  # time_boot_ms, target_system, target_component
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

def detectar_persona(cap, net, output_layers, classes):
    """
    Detecta la posición de una persona en la imagen utilizando YOLO.
    """
    ret, frame = cap.read()
    if not ret:
        print("Error: No se pudo capturar la imagen.")
        return None

    height, width, channels = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5 and classes[class_id] == "person":  # Solo personas (class_id = 0)
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    if len(indices) > 0:
        for i in indices.flatten():
            x, y, w, h = boxes[i]
            center_x = x + w // 2
            center_y = y + h // 2
            return (center_x, center_y)

    return None

def simular_seguimiento(cap, net, output_layers, classes):
    """
    Simula el seguimiento de una persona.
    """
    print("Simulando seguimiento...")
    while True:
        posicion = detectar_persona(cap, net, output_layers, classes)
        if posicion is None:
            print("No se detecta ninguna persona.")
            send_ned_velocity(0, 0, 0, 1)  # Detener el dron
            time.sleep(1)  # Esperar antes de la siguiente detección
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

        time.sleep(1)  # Esperar antes de la siguiente detección

# **Cargar el modelo YOLO y las clases**
print("Cargando modelo YOLO...")
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# **Cargar las clases desde coco.names**
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# **Abrir la cámara**
print("Abriendo cámara...")
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: No se pudo abrir la cámara.")
    vehicle.close()
    exit()

# **Ejecutar la simulación**
try:
    print("Cambiando a modo GUIDED_NOGPS...")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    while not vehicle.mode.name == "GUIDED_NOGPS":
        print("Esperando cambio de modo...")
        time.sleep(1)

    print("Armando motores...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Esperando armado...")
        time.sleep(1)

    print("Motores armados. Iniciando seguimiento...")
    simular_seguimiento(cap, net, output_layers, classes)
except KeyboardInterrupt:
    print("Simulación detenida por el usuario.")
finally:
    print("Desarmando motores...")
    vehicle.armed = False
    while vehicle.armed:
        print("Esperando desarmado...")
        time.sleep(1)

    print("Cerrando cámara...")
    cap.release()
    print("Cerrando conexión...
