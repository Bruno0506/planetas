from dronekit import connect, VehicleMode
import time

# Conectar al dron
try:
    print("Conectando al dron...")
    vehicle = connect('udpin:0.0.0.0:14550', wait_ready=True)
    print("Conexión exitosa!")
except Exception as e:
    print(f"Error al conectar: {e}")
    exit()

# Función para despegar a una altitud específica
def despegar(altura):
    print(f"Despegando a {altura} metros...")
    vehicle.simple_takeoff(altura)  # Comando oficial de DroneKit para despegar

    # Esperar a que el dron alcance la altitud deseada
    while True:
        print(f"Altura actual: {vehicle.location.global_relative_frame.alt} metros")
        if vehicle.location.global_relative_frame.alt >= altura * 0.95:  # 95% de la altitud deseada
            print(f"Altura de {altura} metros alcanzada.")
            break
        time.sleep(1)

# Ejecutar la simulación
try:
    print("Cambiando a modo GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Esperando cambio de modo...")
        time.sleep(1)

    print("Armando motores...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Esperando armado...")
        time.sleep(1)

    print("Motores armados. Despegando...")
    despegar(altura=2)  # Despegar a 2 metros de altura

    print("Manteniendo la altitud durante 10 segundos...")
    time.sleep(10)  # Mantener la altitud durante 10 segundos

except KeyboardInterrupt:
    print("Simulación detenida por el usuario.")
finally:
    print("Aterrizando...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("Esperando aterrizaje...")
        time.sleep(1)

    print("Cerrando conexión...")
    vehicle.close()
    print("Simulación completada.")
