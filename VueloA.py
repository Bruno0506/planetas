import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Conexión al dron (simulación o hardware real)
print("Conectando al dron...")
vehicle = connect('127.0.0.1:14550', baud=57600, wait_ready=True)

# Cargar YOLOv3-tiny
net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
classes = open("coco.names").read().strip().split("\n")

# Iniciar cámara
cap = cv2.VideoCapture(0)  # Cambia a 2 si usas otra cámara

def send_velocity(velocity_x, velocity_y, velocity_z, yaw_rate=0):
    """ Enviar comandos de velocidad al dron en modo GUIDED_NOGPS """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # Control solo de velocidades
        0, 0, 0,  # Posición X, Y, Z
        velocity_x, velocity_y, velocity_z,  # Velocidades
        0, 0, yaw_rate  # Yaw rate
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def arm_and_start():
    """ Arma motores y activa el modo GUIDED_NOGPS """
    print("Chequeando estado del dron...")
    while not vehicle.is_armable:
        print("Esperando inicialización del dron...")
        time.sleep(1)

    print("Activando modo GUIDED_NOGPS...")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")

    print("Armando motores...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Esperando armado...")
        time.sleep(1)

    print("Motores armados. Listo para volar.")

# Iniciar la simulación
arm_and_start()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    detections = net.forward(output_layers)

    person_detected = False
    center_x, center_y = 0, 0

    for detection in detections:
        for obj in detection:
            scores = obj[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5 and classes[class_id] == "person":
                person_detected = True
                center_x = int(obj[0] * width)
                center_y = int(obj[1] * height)

                # Dibujar rectángulo
                w = int(obj[2] * width)
                h = int(obj[3] * height)
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Ajuste de movimiento basado en detección
    if person_detected:
        move_x = 0.3 if center_x > width * 0.6 else (-0.3 if center_x < width * 0.4 else 0)
        move_y = 0.3 if center_y > height * 0.6 else (-0.3 if center_y < height * 0.4 else 0)
        send_velocity(move_x, move_y, 0)
    else:
        send_velocity(0, 0, 0)

    # Mostrar frame
    cv2.imshow("YOLO Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Finalizar
cap.release()
cv2.destroyAllWindows()
vehicle.mode = VehicleMode("LAND")
vehicle.close()
