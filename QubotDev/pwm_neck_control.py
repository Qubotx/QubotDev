import Jetson.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

pin_pwm = 33
GPIO.setup(pin_pwm, GPIO.OUT)

pwm_servo = GPIO.PWM(pin_pwm, 50)  # 50Hz para servo estándar
pwm_servo.start(0)


def set_neck_angle(angle):
    duty_cycle = angle / 18.0 + 2.5
    pwm_servo.ChangeDutyCycle(duty_cycle)
