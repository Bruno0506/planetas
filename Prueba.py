import time
import navio.pwm
import navio.util

# Inicializar PWM para los motores
MOTOR_CHANNEL = 0  # Puedes cambiarlo si usas otro canal PWM
PWM_FREQUENCY = 50  # Hz (Frecuencia para ESCs de motores)
MIN_THROTTLE = 1.0  # PWM mínimo (motores apagados)
ARM_THROTTLE = 1.3  # PWM para armar motores
HOVER_THROTTLE = 1.5  # PWM para mantener "vuelo" (simulado)

# Habilitar Navio2
navio.util.check_apm()  # Verifica si ArduPilot está corriendo
pwm = navio.pwm.PWM(MOTOR_CHANNEL)
pwm.set_period(PWM_FREQUENCY)  # Configurar la frecuencia de PWM
pwm.enable()  # Habilitar PWM

print("🚀 Iniciando secuencia de armado...")
time.sleep(2)

# 1️⃣ **Armar motores**
print("⚙ Armando motores...")
pwm.set_duty_cycle(ARM_THROTTLE)  # Enviar señal de armado
time.sleep(3)

# 2️⃣ **Simular despegue**
print("🛫 Simulando despegue (sin levantar el dron realmente)...")
pwm.set_duty_cycle(HOVER_THROTTLE)  # Mantener un throttle estable
time.sleep(5)

# 3️⃣ **Aterrizar**
print("🛬 Iniciando aterrizaje...")
pwm.set_duty_cycle(ARM_THROTTLE)  # Reducir potencia
time.sleep(3)

# 4️⃣ **Apagar motores**
print("⛔ Apagando motores...")
pwm.set_duty_cycle(MIN_THROTTLE)  # Cortar potencia
time.sleep(2)

print("✅ Secuencia completa.")
