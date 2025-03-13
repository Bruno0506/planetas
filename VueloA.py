import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Conexión al dron
print("Conectando al dron...")
vehicle = connect('udpin:0.0.0.0:14550', wait_ready=True)

# Desactivar verificaciones de prearm
print("Desactivando verificaciones de prearm...")
vehicle.parameters['ARMING_CHECK'] = 0
time.sleep(2)
print("Estado de ARMING_CHECK:", vehicle.parameters['ARMING_CHECK'])

# Cargar YOLOv3-tiny
net = cv2.dnn.readNet("modelo/YOLO/yolov3-tiny.weights", "modelo/YOLO/yolov3-tiny.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
classes = open("modelo/YOLO/my_classes.name").read().strip().split("\n")

# Iniciar cámara
cap = cv2.VideoCapture(0)  # Cambia a 2 si usas otra cámara

def wait_until_armable(timeout=60):
    """ Espera hasta que el dron esté listo para armarse o se alcance el tiempo de espera. """
    start_time = time.time()
    while not vehicle.is_armable:
        print(f"Estado del dron: {vehicle.system_status.state}")
        print(f"GPS: {vehicle.gps_0}")
        print(f"EKF OK: {vehicle.ekf_ok}")
        print(f"Batería: {vehicle.battery.level}%")
        print(f"Modo actual: {vehicle.mode.name}")
        print("Esperando inicialización del dron...")
        
        # Verificar si se ha alcanzado el tiempo de espera
        if time.time() - start_time > timeout:
            print("Tiempo de espera agotado. El dron no está listo para armarse.")
            return False
        
        time.sleep(1)
    
    print("El dron está listo para armarse.")
    return True

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

try:
    # Esperar a que el dron esté listo para armarse
    if not wait_until_armable(timeout=60):
        print("No se pudo armar el dron. Saliendo...")
        vehicle.close()
        exit()

    # Código principal (despegue, detección, etc.)
    print("Armando motores...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Esperando armado...")
        time.sleep(1)

    print("Motores armados. Listo para volar.")

    # Simular despegue y vuelo
    print("Simulando despegue...")
    time.sleep(5)  # Simular ascenso
    print("Simulando vuelo...")
    time.sleep(10)  # Simular vuelo

except KeyboardInterrupt:
    print("\nInterrupción detectada. Aterrizando el dron...")

finally:
    # Volver a un estado seguro
    print("Aterrizando el dron...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("Esperando aterrizaje...")
        time.sleep(1)

    print("Desarmando motores...")
    vehicle.armed = False
    while vehicle.armed:
        print("Esperando desarme...")
        time.sleep(1)

    print("Cerrando conexión con el dron...")
    vehicle.close()
    print("Dron desconectado correctamente.")
