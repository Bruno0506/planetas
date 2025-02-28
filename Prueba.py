from dronekit import connect, VehicleMode
import time

# Conectar con el dron (simulación o hardware)
print("Conectando al dron...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(target_altitude):
    """
    Arma los motores y despega a la altitud indicada.
    """
    print("Iniciando despegue...")

    # Poner en modo GUIDED (necesario para control autónomo)
    vehicle.mode = VehicleMode("GUIDED")
    
    # Esperar hasta que el dron pueda armarse
    while not vehicle.is_armable:
        print("Esperando que el dron sea armable...")
        time.sleep(1)

    # Armar motores
    vehicle.armed = True
    while not vehicle.armed:
        print("Esperando que los motores se armen...")
        time.sleep(1)

    print("Motores armados. Despegando...")

    # Orden de despegue
    vehicle.simple_takeoff(target_altitude)

    # Esperar hasta alcanzar la altitud deseada
    while True:
        altura_actual = vehicle.location.global_relative_frame.alt
        print(f"Altura actual: {altura_actual:.2f} m")
        
        if altura_actual >= target_altitude * 0.95:  # Si está cerca de la altitud deseada
            print("Altitud alcanzada.")
            break
        
        time.sleep(1)

def land_and_shutdown():
    """
    Aterriza el dron y apaga los motores.
    """
    print("Iniciando aterrizaje...")
    vehicle.mode = VehicleMode("LAND")

    # Esperar hasta que aterrice
    while vehicle.armed:
        print(f"Altura actual: {vehicle.location.global_relative_frame.alt:.2f} m")
        time.sleep(1)

    print("Dron en tierra. Apagando motores...")
    vehicle.armed = False

    print("Motores apagados. Cerrando conexión.")
    vehicle.close()

# 📌 Ejecución principal
try:
    arm_and_takeoff(1)  # Despegar a 1 metro
    time.sleep(5)  # Esperar 5 segundos en el aire
    land_and_shutdown()  # Aterrizar y apagar motores

except Exception as e:
    print(f"Error detectado: {e}")
    vehicle.close()
