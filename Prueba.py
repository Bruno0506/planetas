from navio2 import pwm
import time

# Configuración de PWM para los motores
PWM_OUTPUT = 0
pwm.initialize()
pwm.set_period(PWM_OUTPUT, 50)  # Frecuencia de 50Hz para ESCs

def set_throttle(channel, value):
    pwm.set_duty_cycle(channel, value)

try:
    print("Encendiendo motores...")
    set_throttle(PWM_OUTPUT, 10)  # Encender motores (ajusta el valor según sea necesario)
    time.sleep(20)  # Esperar 20 segundos
    print("Apagando motores...")
    set_throttle(PWM_OUTPUT, 0)  # Apagar motores
finally:
    set_throttle(PWM_OUTPUT, 0)  # Asegurarse de que los motores estén apagados al finalizar
