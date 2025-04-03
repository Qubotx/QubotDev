import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from pymodbus.client import ModbusSerialClient
import time

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
    return (128 + int(value*128.0/100.0)) * 128


class ModbusBridge(Node):
    def __init__(self):
        super().__init__('modbus_bridge')
        self.client = ModbusSerialClient(
            port="/dev/ttyUSB0", baudrate=115200, timeout=0.5)
        self.client.connect()

        # inicializa variables
        self.polling_period = 0.1  # sec
        self.time_last = self.get_clock().now()   # time in nanoseconds
        self.ctrl_vel = {'vx': 0, 'vy': 0, 'vr': 0}
        self.left_vel = 0
        self.right_vel = 0
        self.new_data = False
        self.sensor_data = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        self.led_strip_value = 0
        self.move_code = ""
        self.latch = False

        # inicializa los motores
        self.client.write_register(VMODE_REG, 3, MOTOR_1)  # set velocity mode
        self.client.write_register(VMODE_REG, 3, MOTOR_2)  # set velocity mode
        self.client.write_register(
            ACELERATION_REG, 1000, MOTOR_1)  # aceleration
        self.client.write_register(
            ACELERATION_REG, 1000, MOTOR_2)  # aceleration
        self.client.write_register(
            DESACELERATION_REG, 800, MOTOR_1)  # desaceleration
        self.client.write_register(
            DESACELERATION_REG, 800, MOTOR_2)  # desaceleration
        self.client.write_register(STATUS_REG, 8, MOTOR_1)  # enable motor
        self.client.write_register(STATUS_REG, 8, MOTOR_2)  # enable motor

        self.timer = self.create_timer(
            self.polling_period, self.timer_callback)

        # publicador y suscriptor
        self.publisher = self.create_publisher(String, '/ir_sensors', 1)
        self.cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.led_strip_value = self.create_subscription(
            String, '/led_strip_value', self.led_strip_value_callback, 1)
        self.move_code = self.create_subscription(
            String, '/move_code', self.move_code_callback, 1)

        self.get_logger().info(f"{self.get_name()} started")

    def timer_callback(self):
        try:
            data = self.client.read_input_registers(
                address=0, count=6, slave=SENSORS_UNIT)
            # self.get_logger().info(f"IR sensors values: {' '.join(str(d) for d in data.registers)}")
            outrange = 200
            inrange = 1200
            self.sensor_data = []
            for d in data.registers:
                if d < outrange:
                    d = outrange
                if d > inrange:
                    d = inrange
                self.sensor_data.append(
                    round(1.0 - (d - outrange)/(inrange-outrange), 1))

            msg = String()
            # msg.data = " ".join(str(d) for d in data.registers)
            msg.data = " ".join(str(s) for s in self.sensor_data)
            self.publisher.publish(msg)
        except:
            self.get_logger().warning('no communication with sensors')

        # now send velocities
        if self.new_data:
            # data=self.client.read_holding_registers(VOLT_REG, slave = 1)
            # print(data.registers[0])
            self.client.write_register(
                TARGET_RPM_REG, self.left_vel, MOTOR_1)  # set
            self.client.write_register(
                TARGET_RPM_REG, self.right_vel, MOTOR_2)  # set
        else:
            print('waiting instruction...')
            self.client.write_register(TARGET_RPM_REG, 0, MOTOR_1)  # set
            self.client.write_register(TARGET_RPM_REG, 0, MOTOR_2)  # set
        self.new_data = False

        time.sleep(0.01)

        try:
            # self.client.write_register(LED_STRIP, scale(self.led_strip_value), HEAD_CONTROLLER)
            self.client.write_register(LED_STRIP, scale(100), HEAD_CONTROLLER)
            pass
        except:
            self.get_logger().warning('no communication with head controller')

        try:
            if self.move_code == "move_left":
                self.client.write_register(CODE_REGISTER, 10, HEAD_CONTROLLER)
            if self.move_code == "move_right":
                self.client.write_register(CODE_REGISTER, 11, HEAD_CONTROLLER)
            if self.move_code == "led_on":
                if self.latch == False:
                    self.latch = True
                else:
                    self.latch = False
            if self.latch:
                self.client.write_register(CODE_REGISTER, 5, HEAD_CONTROLLER)

            self.move_code = ""
        except:
            self.get_logger().warning('problems moving head...')

    def cmd_vel_callback(self, msg):
        # lee el mensaje
        time_now = self.get_clock().now()
        # discard collision velocities

        if time_now.nanoseconds - self.time_last.nanoseconds > 100000:
            scale = 10
            r = 0.085
            b = 0.32
            vel = msg.linear.x
            # discard collision velocities
            if vel < 0:
                vel *= self.sensor_data[0]*self.sensor_data[5]
            if vel > 0:
                vel *= self.sensor_data[2]*self.sensor_data[3]
            omega = msg.angular.z
            self.left_vel = int(scale * 1/r * (omega + b*vel))
            self.right_vel = int(scale * 1/r * (omega - b*vel))
            print(self.left_vel, self.right_vel)
            self.left_vel = self.left_vel if self.left_vel >= 0 else (
                65535 - abs(self.left_vel))
            self.right_vel = self.right_vel if self.right_vel >= 0 else (
                65535 - abs(self.right_vel))
            print(self.left_vel, self.right_vel)
            self.new_data = True
            self.time_last = time_now

    def led_strip_value_callback(self, msg):
        self.led_strip_value = int(msg.data)

    def move_code_callback(self, msg):
        self.move_code = msg.data


def main(args=None):
    try:
        rclpy.init(args=args)
        modbus_bridge = ModbusBridge()
        rclpy.spin(modbus_bridge)

    except KeyboardInterrupt:
        print(' ... exit node')
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()