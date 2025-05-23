import Jetson.GPIO as GPIO
import time

# === Configuración del GPIO para el servo ===
GPIO.setmode(GPIO.BOARD)
pin_pwm = 33
GPIO.setup(pin_pwm, GPIO.OUT)
pwm_servo = GPIO.PWM(pin_pwm, 300)  # 300Hz para mayor precisión
pwm_servo.start(0)


current_angle = 90  # Inicialmente centrado

def set_neck_angle(angle):
    global current_angle
    # Limita el ángulo entre 50 y 130 por construcción del QuBot
    angle = max(50, min(130, angle))
    duty_cycle = angle * (66.5 - 24.5) / 180.0 + 24.5  #
    pwm_servo.ChangeDutyCycle(duty_cycle)
    current_angle = angle

def rotate_neck_relative(delta):
    """Rota el cuello delta grados respecto al ángulo actual."""
    set_neck_angle(current_angle + delta)
