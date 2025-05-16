import QubotDev.ZLAC8015D_workinglib as ZLAC8015D

import time
from pynput import keyboard

# Inicializar el controlador de motores
motors = ZLAC8015D.Controller(port="/dev/ttyUSB0")

# Configuración inicial
motors.disable_motor()

motors.set_accel_time(100, 100)  # Reducir tiempo de aceleración para mayor sensibilidad
motors.set_decel_time(
    100, 100
)  # Reducir tiempo de desaceleración para frenados más rápidos
motors.set_maxRPM_pos(22.5, 22.5)
motors.enable_motor()


# Funciones de movimiento
def medio_giro_positivo():
    motors.set_mode(1)
    motors.set_position_async_control()
    motors.set_relative_angle(-90, -90)  # Gira a la derecha
    motors.move_left_wheel()
    motors.move_right_wheel()
    time.sleep(2.0)
    motors.disable_motor()
    time.sleep(0.00001)
    motors.enable_motor()


def medio_giro_negativo():
    motors.set_mode(1)
    motors.set_position_async_control()
    motors.set_relative_angle(90, 90)  # Gira a la izquierda
    motors.move_left_wheel()
    motors.move_right_wheel()
    time.sleep(2.0)
    motors.disable_motor()
    time.sleep(0.00001)
    motors.enable_motor()


def giro_completo_positivo():
    motors.set_mode(1)
    motors.set_position_async_control()
    motors.set_relative_angle(-180, -180)  # Gira completo a la derecha
    motors.move_left_wheel()
    motors.move_right_wheel()
    time.sleep(4.0)
    motors.disable_motor()
    time.sleep(0.00001)
    motors.enable_motor()


def giro_completo_negativo():
    motors.set_mode(1)
    motors.set_position_async_control()
    motors.set_relative_angle(180, 180)  # Gira completo a la izq
    motors.move_left_wheel()
    motors.move_right_wheel()
    time.sleep(4.0)
    motors.disable_motor()
    time.sleep(0.00001)
    motors.enable_motor()


def avanzar():
    motors.set_mode(3)
    motors.set_rpm(-20, 20)  # Avanza recto
    time.sleep(2.0)
    motors.disable_motor()
    time.sleep(0.00001)
    motors.enable_motor()


def avanzar2():
    motors.set_mode(3)
    motors.set_rpm(-20, 20)  # Avanza recto
    time.sleep(2.0)


def retroceder2():
    motors.set_mode(3)
    motors.set_rpm(20, -20)  # Avanza recto
    time.sleep(2.0)


def circulo_positivo():
    motors.set_mode(3)
    motors.set_rpm(-18, 3)  # Avanza en circulo hacia la derecha
    time.sleep(5.0)
    motors.disable_motor()
    time.sleep(0.00001)
    motors.enable_motor()


def circulo_negativo():
    motors.set_mode(3)
    motors.set_rpm(-3, 18)  # Avanza en circulo hacia la izquierda
    time.sleep(5.0)
    motors.disable_motor()
    time.sleep(0.00001)
    motors.enable_motor()


def detener():
    motors.disable_motor()


def rutina_paseo():
    avanzar()
    avanzar()
    avanzar()
    time.sleep(0.5)
    medio_giro_negativo()
    time.sleep(0.5)
    avanzar()
    avanzar()
    avanzar()
    avanzar()
    time.sleep(0.5)
    medio_giro_negativo()
    time.sleep(0.5)
    avanzar()
    avanzar()
    time.sleep(1.0)
    medio_giro_negativo()
    time.sleep(1.0)
    giro_completo_positivo()
    time.sleep(0.5)
    avanzar()
    time.sleep(1.5)
    giro_completo_negativo()
    time.sleep(0.5)
    avanzar()
    avanzar()
    time.sleep(1.0)
    medio_giro_negativo()
    time.sleep(0.5)
    circulo_positivo()
    time.sleep(1.5)
    giro_completo_negativo()
    time.sleep(1.0)
    circulo_negativo()
    time.sleep(0.5)
    medio_giro_negativo()
    time.sleep(0.5)
    avanzar()
    avanzar()
    avanzar()
    time.sleep(1.0)
    medio_giro_positivo()
    time.sleep(0.5)
    avanzar()
    time.sleep(1.0)
    giro_completo_positivo()


# Función para manejar eventos de teclado
def on_press(key):
    try:
        if key.char == "a":
            runita_paseo()

    except AttributeError:
        pass  # Para ignorar teclas especiales


def on_release(key):
    if key == keyboard.Key.esc:
        print("Saliendo del programa...")
        return False  # Detener el listener


def main(args=None):
    print("Presiona las teclas para ejecutar rutinas:")
    print("  A: Avanzar")
    print("Presiona ESC para salir.")

    try:
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()  # Mantener la escucha activa

    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")

    finally:
        motors.disable_motor()  # Apagar motores al final
        print("Motores desactivados.")


# Ejecutar el programa
if __name__ == "__main__":
    main()
