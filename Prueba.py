from dronekit import connect, VehicleMode
import cv2
import numpy as np
import time

# Conexión al dron
print("Conectando al dron...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Configuración de parámetros para desactivar verificaciones y failsafes
print("Configurando parámetros...")
vehicle.parameters['ARMING_CHECK'] = 0  # Desactiva las verificaciones de armado
vehicle.parameters['THR_FAILSAFE'] = 0  # Desactiva el failsafe del throttle

# Cargar el modelo YOLOv3-tiny
print("Cargando modelo YOLOv3-tiny...")
net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Cargar las clases
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Iniciar la cámara
cap = cv2.VideoCapture(0)

def arm_motors():
    """
    Arma los motores del dron.
    """
    print("Armando motores...")
    vehicle.mode = VehicleMode("GUIDED")

    # Esperar a que el dron esté listo para armarse
    while not vehicle.is_armable:
        print("Esperando que el dron esté listo para armarse...")
        print(f"GPS: {vehicle.gps_0}")
        print(f"Batería: {vehicle.battery}")
        print(f"Estado de los sensores: {vehicle.system_status}")
        time.sleep(1)

    vehicle.armed = True

    # Esperar hasta que los motores estén armados
    while not vehicle.armed:
        print("Esperando que los motores se armen...")
        time.sleep(1)

    print("Motores armados.")

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Envía comandos de velocidad al dron en las direcciones Norte, Este y Abajo.
    :param velocity_x: Velocidad en dirección Norte (m/s)
    :param velocity_y: Velocidad en dirección Este (m/s)
    :param velocity_z: Velocidad en dirección Abajo (m/s)
    :param duration: Duración del movimiento (segundos)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (no usado)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (solo velocidad)
        0, 0, 0, # x, y, z positions (no usado)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (no usado)
        0, 0)    # yaw, yaw_rate (no usado)
    
    # Enviar el comando repetidamente durante la duración especificada
    for _ in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

    # Detener el dron después del movimiento
    send_ned_velocity(0, 0, 0, 1)  # Envía velocidad cero para detener el dron

def avanzar(duration):
    print("Avanzando...")
    send_ned_velocity(1, 0, 0, duration)  # Adelante (Norte)

def retroceder(duration):
    print("Retrocediendo...")
    send_ned_velocity(-1, 0, 0, duration)  # Atrás (Sur)

def derecha(duration):
    print("Girando a la derecha...")
    send_ned_velocity(0, 1, 0, duration)  # Derecha (Este)

def izquierda(duration):
    print("Girando a la izquierda...")
    send_ned_velocity(0, -1, 0, duration)  # Izquierda (Oeste)

def detect_person_and_follow():
    """
    Detecta a una persona y ajusta la posición del dron según la distancia.
    """
    while True:
        # Capturar frame de la cámara
        ret, frame = cap.read()
        if not ret:
            break

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
                if confidence > 0.5 and classes[class_id] == "person":  # Solo detectar personas
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Dibujar las detecciones
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Calcular la distancia aproximada (basada en el tamaño de la caja)
                distance = (1 / (w * h)) * 10000  # Fórmula simplificada para pruebas

                # Tomar decisiones basadas en la distancia
                if distance > 2:  # Si está a más de 2 metros, acercarse
                    print("Persona detectada. Acercándose...")
                    avanzar(2)
                elif distance < 1:  # Si está a menos de 1 metro, alejarse
                    print("Persona detectada. Alejándose...")
                    retroceder(2)
                else:  # Si está entre 1 y 2 metros, girar
                    print("Persona detectada. Girando...")
                    if x < width / 2:  # Si la persona está a la izquierda, girar a la izquierda
                        izquierda(2)
                    else:  # Si la persona está a la derecha, girar a la derecha
                        derecha(2)

        # Mostrar el frame
        cv2.imshow("YOLOv3-tiny", frame)
        key = cv2.waitKey(1)
        if key == 27:  # Presionar ESC para salir
            break

# Ejecución principal
try:
    # Armar los motores (sin despegar)
    arm_motors()

    # Iniciar la detección de personas y seguimiento
    detect_person_and_follow()
finally:
    # Liberar recursos
    cap.release()
    cv2.destroyAllWindows()
    vehicle.close()
    print("Conexión cerrada.")
