import time
from dronekit import connect, VehicleMode

# Conectar al dron
print("Conectando al dron...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Desactivar verificaciones de prearmado
print("Desactivando verificaciones de prearm...")
vehicle.parameters['ARMING_CHECK'] = 0
vehicle.parameters['GPS_TYPE'] = 0  # Desactiva GPS si no se usa
time.sleep(2)

# Forzar EKF si es necesario
print("Configurando EKF...")
vehicle.parameters['AHRS_EKF_TYPE'] = 3
vehicle.parameters['EK2_ENABLE'] = 1
vehicle.parameters['EK3_ENABLE'] = 0
time.sleep(2)

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

# Simulación de vuelo
print("Simulando despegue...")
time.sleep(5)  # Simula
