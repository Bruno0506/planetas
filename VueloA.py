import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Conectar al dron
print("Conectando al dron...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Desactivar chequeos de prearmado y GPS
print("Desactivando verificaciones de prearm...")
vehicle.parameters['ARMING_CHECK'] = 0
vehicle.parameters['GPS_TYPE'] = 0  # Desactiva GPS si no se usa
time.sleep(2)

# Configurar EKF para permitir vuelo sin GPS
print("Configurando EKF...")
vehicle.parameters['AHRS_EKF_TYPE'] = 3
vehicle.parameters['EK2_ENABLE'] = 1
vehicle.parameters['EK3_ENABLE'] = 0
time.sleep(2)

# Función para enviar comandos de velocidad
def send_velocity(velocity_x, velocity_y, velocity_z, yaw_rate=0):
    """Envia comandos de velocidad al dron en modo GUIDED_NOGPS"""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # Control solo de velocidades
        0, 0, 0,  # Posición X, Y, Z
        velocity_x, velocity_y, velocity_z,  # Velocidades
        0, 0, yaw_rate  # Yaw rate
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Función para esperar que el dron esté listo
def esperar_listo():
    print("Chequeando estado del dron...")
    while not vehicle.is_armable:
        print(f"GPS: fix={vehicle.gps_0.fix_type}, Satélites={vehicle.gps_0.satellites_visible}")
        print(f"EKF OK: {vehicle.ekf_ok}, Batería: {vehicle.battery.level}%, Modo: {vehicle.mode.name}")
        time.sleep(1)
    print("✅ Dron listo para armar!")

# Esperar que el dron esté listo
esperar_listo()

# Cambiar a modo de vuelo sin GPS
print("Cambiando a modo GUIDED_NOGPS...")
vehicle.mode = VehicleMode("GUIDED_NOGPS")
time.sleep(2)
print(f"Modo actual: {vehicle.mode.name}")

# Intentar armar el dron
print("Armando motores...")
vehicle.armed = True

# Esperar confirmación de armado
while not vehicle.armed:
    print("Esperando armado...")
    time.sleep(1)

print("✅ Motores armados, iniciando misión!")

# Configurar YOLO
config_path = "yolov3-tiny.cfg"
weights_path = "yolov3-tiny.weights"
labels_path = "coco.names"

with open(labels_path, "r") as f:
    labels = f.read().strip().split("\n")

net = cv2.dnn.readNetFromDarknet(config_path, weights_path)
layer_names = net.getLayerNames()
output_layers = [layer_names[int(i) - 1] for i in net.getUnconnectedOutLayers()]

# Inicializar cámara
cap = cv2.VideoCapture(0)

print("Iniciando visión y seguimiento...")
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error al capturar la imagen. Saliendo...")
            break

        # Preprocesamiento de la imagen para YOLO
        height, width, _ = frame.shape
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        detections = net.forward(output_layers)

        person_detected = False
        center_x, center_y = 0, 0
        bbox_width = 0  # Ancho de la caja de la persona detectada

        # Procesar detecciones
        for detection in detections:
            for obj in detection:
                scores = obj[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5 and labels[class_id] == "person":
                    person_detected = True
                    center_x = int(obj[0] * width)
                    center_y = int(obj[1] * height)
                    bbox_width = int(obj[2] * width)

                    # Dibujar rectángulo
                    w = int(obj[2] * width)
                    h = int(obj[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Ajuste de movimiento basado en detección
        if person_detected:
            # Movimiento lateral
            move_x = 0.3 if center_x > width * 0.6 else (-0.3 if center_x < width * 0.4 else 0)
            
            # Movimiento adelante/atrás basado en tamaño del bounding box (distancia)
            if bbox_width < width * 0.2:  # Persona lejos
                move_y = 0.3
                print("🔹 Persona lejos, avanzando...")
            elif bbox_width > width * 0.4:  # Persona muy cerca
                move_y = -0.3
                print("🔸 Persona muy cerca, retrocediendo...")
            else:
                move_y = 0  # Mantener posición

            # Enviar comandos al dron
            send_velocity(move_x, move_y, 0)

        else:
            print("❌ No se detectó persona, deteniendo dron...")
            send_velocity(0, 0, 0)

        # Mostrar frame con detección
        cv2.imshow("YOLO Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Salida por el usuario.")
            break

except KeyboardInterrupt:
    print("Simulación detenida por el usuario.")
finally:
    print("Simulando aterrizaje...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("Esperando aterrizaje...")
        time.sleep(1)

    print("✅ Dron aterrizado y apagado.")

    # Cerrar todo correctamente
    cap.release()
    cv2.destroyAllWindows()
    vehicle.close()
    print("Dron desconectado correctamente.")
