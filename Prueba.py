from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

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

def simular_seguimiento():
    """
    Simula el seguimiento de una persona.
    """
    print("Simulando seguimiento...")
    # Simular movimiento hacia adelante
    print("Moviendo adelante...")
    send_ned_velocity(1, 0, 0, 3)  # 1 m/s hacia adelante por 3 segundos

    # Simular movimiento hacia la derecha
    print("Moviendo derecha...")
    send_ned_velocity(0, 1, 0, 3)

    # Simular movimiento hacia la izquierda
    print("Moviendo izquierda...")
    send_ned_velocity(0, -1, 0, 3)

    # Simular movimiento hacia atrás
    print("Moviendo atrás...")
    send_ned_velocity(-1, 0, 0, 3)

    # Simular subida
    print("Subiendo...")
    send_ned_velocity(0, 0, -1, 3)

    # Simular bajada
    print("Bajando...")
    send_ned_velocity(0, 0, 1, 3)

# **Ejecutar la simulación**
try:
    simular_despegue()  # Simular el despegue
    simular_seguimiento()  # Simular el seguimiento
finally:
    # Desarmar el dron y cerrar la conexión
    vehicle.armed = False
    vehicle.close()
    print("Simulación completada.")
