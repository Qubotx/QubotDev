import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# import pymodbus.client.sync as _sync

# # patch write_register
# _orig_wr = _sync.ModbusSerialClient.write_register


# def _write_register(self, address, value, device_id=None, **kw):
#     return _orig_wr(self, address, value, unit=device_id, **kw)


# _sync.ModbusSerialClient.write_register = _write_register

# # patch write_registers
# _orig_wrs = _sync.ModbusSerialClient.write_registers


# def _write_registers(self, address, values, device_id=None, **kw):
#     return _orig_wrs(self, address, values, unit=device_id, **kw)


# _sync.ModbusSerialClient.write_registers = _write_registers


# from pymodbus.client import ModbusSerialClient
import QubotDev.ZLAC8015D_workinglib as ZLAC8015D
import time

import QubotDev.modo_espera as pre_movs

from QubotDev.pwm_neck_control import set_neck_angle

# Modbus addresses
MOTOR_1 = 1
MOTOR_2 = 2
SENSORS_UNIT = 10
HEAD_CONTROLLER = 11

# Modbus registers
VOLT_REG = 8224
TEMP_REG = 8230
STATUS_REG = 8241
VMODE_REG = 8242
TARGET_RPM_REG = 8250
ACELERATION_REG = 8247
DESACELERATION_REG = 8248

HEAD_MOTOR = 40001
LED_STRIP = 40002
CODE_REGISTER = 40003
LED_BUILTIN = 10001
BUZZER = 10002


def scale(value):
    return (128 + int(value * 128.0 / 100.0)) * 128


class zlac8015dBridge(Node):
    def __init__(self):
        super().__init__("zlac8015d_bridge")
        # Inicializar el controlador de motores
        self.motors = ZLAC8015D.Controller(port="/dev/ttyUSB0")

        # inicializa variables
        self.polling_period = 0.1  # sec
        self.time_last = self.get_clock().now()  # time in nanoseconds
        self.ctrl_vel = {"vx": 0, "vy": 0, "vr": 0}
        self.left_vel = 0
        self.right_vel = 0
        self.new_data = False

        # inicializa los motores
        self.motors.disable_motor()
        self.motors.set_accel_time(
            100, 100
        )  # Reducir tiempo de aceleración para mayor sensibilidad
        self.motors.set_decel_time(
            100, 100
        )  # Reducir tiempo de desaceleración para frenados más rápidos
        # self.motors.set_maxRPM_pos(22.5, 22.5)
        self.motors.enable_motor()

        self.timer = self.create_timer(self.polling_period, self.timer_callback)

        # inicializa publicadores
        self.cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 1
        )

        self.get_logger().info(f"{self.get_name()} started")

        # Add subscriber for /control_cmd topic
        self.control_cmd_sub = self.create_subscription(
            String, "/control_cmd", self.control_cmd_callback, 1
        )

    def timer_callback(self):

        # mueve los motores
        if self.new_data:
            self.get_logger().info(f"{self.get_name()} moving...")
            self.motors.set_mode(3)
            self.motors.set_rpm(self.left_vel, self.right_vel)
            time.sleep(2.0)

            # pre_movs.avanzar2()
            # pre_movs.avanzar2()
            # pre_movs.avanzar2()
            # pre_movs.avanzar2()

            # self.motors.set_mode(3)
            # self.motors.set_rpm(-20, 20)  # Avanza recto
            # time.sleep(2.0)
            # self.motors.disable_motor()
            # time.sleep(0.00001)
            # self.motors.enable_motor()
            self.get_logger().info(f"{self.get_name()} waiting cmd_vel...")
        else:
            # print("waiting cmd_vel...")
            self.motors.set_rpm(0, 0)
        self.new_data = False

        time.sleep(0.01)

    def cmd_vel_callback(self, msg):
        # lee el mensaje
        time_now = self.get_clock().now()

        if time_now.nanoseconds - self.time_last.nanoseconds > 100000:
            scale = 1.0
            r = 0.0625
            b = 0.38
            vel = msg.linear.x * 5
            omega = msg.angular.z
            self.left_vel = int(scale * 1 / r * (omega + b * vel))
            self.right_vel = int(scale * 1 / r * (omega - b * vel))
            print(self.left_vel, self.right_vel)
            self.new_data = True
            self.time_last = time_now
            # print(msg)

    def control_cmd_callback(self, msg):
        # Parse the command string (expects commands like 'forward', 'backward', 'left', 'right')
        command = msg.data.strip().lower()
        self.get_logger().info(f"{self.get_name()} received control command: {command}")
        if command == "forward":
            self.get_logger().info(f"{self.get_name()} going forward...")
            pre_movs.avanzar2()
            pass
        elif command == "backward":
            self.get_logger().info(f"{self.get_name()} going backward...")
            pre_movs.retroceder2()
            pass
        elif command == "left":
            self.get_logger().info(f"{self.get_name()} turning left...")
            pre_movs.medio_giro_negativo()
            pass
        elif command == "right":
            self.get_logger().info(f"{self.get_name()} turning right...")
            pre_movs.medio_giro_positivo()
            pass
        elif command == "stop":
            self.get_logger().info(f"{self.get_name()} stopping...")
            pre_movs.detener()
            pass
        elif command == "rotate-neck-left":
            self.get_logger().info(f"{self.get_name()} rotating neck left...")
            set_neck_angle(90 + 20)
            pass
        elif command == "center-neck":
            self.get_logger().info(f"{self.get_name()} centering neck...")
            set_neck_angle(90)
            pass
        elif command == "rotate-neck-right":
            self.get_logger().info(f"{self.get_name()} rotating neck right...")
            set_neck_angle(90 - 26)
            pass
        else:
            self.get_logger().error(
                f"{self.get_name()} unknown control command: {command}"
            )


def main(args=None):
    try:
        rclpy.init(args=args)
        zlac8015d_bridge = zlac8015dBridge()
        rclpy.spin(zlac8015d_bridge)

    except KeyboardInterrupt:
        print(" ... exit node")
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
