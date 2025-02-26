import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Conectar al dron
print("Conectando al dron...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Función para enviar comandos de velocidad
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # Enviar solo velocidad
        0, 0, 0,  # Posición (no usada)
        velocity_x, velocity_y, velocity_z,  # Velocidad en NED
        0, 0, 0,  # Aceleración (no usada)
        0, 0  # Yaw (no usado)
    )
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

# Funciones de movimiento
def avanzar(): send_ned_velocity(1, 0, 0, 2)  # Adelante 2s
def retroceder(): send_ned_velocity(-1, 0, 0, 2)  # Atrás 2s
def mover_derecha(): send_ned_velocity(0, 1, 0, 2)  # Derecha 2s
def mover_izquierda(): send_ned_velocity(0, -1, 0, 2)  # Izquierda 2s
def girar_derecha(): vehicle.channels.overrides['yaw'] = 1600; time.sleep(2); vehicle.channels.overrides['yaw'] = 1500
def girar_izquierda(): vehicle.channels.overrides['yaw'] = 1400; time.sleep(2); vehicle.channels.overrides['yaw'] = 1500

# Cargar el modelo YOLO
net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Iniciar cámara
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret: break

    height, width, _ = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    detected = False
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5 and classes[class_id] == "person":
                detected = True
                center_x = int(detection[0] * width)
                
                if center_x < width // 3:
                    print("Persona detectada a la izquierda. Girando a la derecha.")
                    girar_derecha()
                elif center_x > 2 * width // 3:
                    print("Persona detectada a la derecha. Girando a la izquierda.")
                    girar_izquierda()
                else:
                    print("Persona detectada al frente. Avanzando.")
                    avanzar()
                break

    if not detected:
        print("No se detectó nada. Retrocediendo...")
        retroceder()

    if cv2.waitKey(1) == 27: break  # Salir con ESC

cap.release()
vehicle.close()
