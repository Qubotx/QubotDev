import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.time import Duration

import QubotDev.ZLAC8015D_workinglib as ZLAC8015D
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


class zlac8015dBridge(Node):
    def __init__(self):
        super().__init__("zlac8015d_bridge")

        # Robot physical parameters
        self.wheel_radius = 0.065  # meters
        self.wheel_base = 0.38  # meters (distance between wheels)

        # Control parameters
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 1.6  # rad/s
        self.max_rpm = 73  # Maximum motor RPM
        self.cmd_timeout = 0.15  # sec - se detiene si no se recibe cmd_vel
        self.min_cmd_interval = 0.05  # sec - minimum time between processing commands

        # Initialize motor controller
        self.motors = ZLAC8015D.Controller(port="/dev/ttyFT232")

        # Control state variables
        self.last_cmd_time = self.get_clock().now() - Duration(seconds=1.0)
        self.last_process_time = self.last_cmd_time
        self.current_left_vel = 0
        self.current_right_vel = 0
        self.cmd_received = False

        # Initialize motors
        self.motors.disable_motor()
        self.motors.set_accel_time(100, 100)
        self.motors.set_decel_time(100, 100)
        self.motors.set_maxRPM_pos(self.max_rpm, self.max_rpm)
        # self.motors.enable_motor()
        self.moving = False

        self.count = 0

        # Tiempo para el control de motor.
        self.polling_period = 0.01  # 100Hz
        self.timer = self.create_timer(self.polling_period, self.timer_callback)

        # Subscriber for cmd_vel messages
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.get_logger().info(
            f"{self.get_name()} started with improved cmd_vel handling"
        )

    def cmd_vel_callback(self, msg):
        """
        Improved cmd_vel callback with proper throttling and safety checks
        """
        current_time = self.get_clock().now()

        # Check if enough time has passed since last processing (anti-spam)
        time_since_last_process = (
            current_time - self.last_process_time
        ).nanoseconds / 1e9
        if time_since_last_process < self.min_cmd_interval:
            return

        # Update timing
        self.last_cmd_time = current_time
        self.last_process_time = current_time

        # Extract and limit velocities
        linear_vel = self.clamp(msg.linear.x, -self.max_linear_vel, self.max_linear_vel)
        angular_vel = self.clamp(
            msg.angular.z, -self.max_angular_vel, self.max_angular_vel
        )

        # Convert to wheel velocities using differential drive kinematics
        # v_left = (linear_vel - angular_vel * wheel_base/2) / wheel_radius
        # v_right = (linear_vel + angular_vel * wheel_base/2) / wheel_radius
        left_wheel_vel = (
            linear_vel - angular_vel * self.wheel_base / 2
        ) / self.wheel_radius
        right_wheel_vel = (
            linear_vel + angular_vel * self.wheel_base / 2
        ) / self.wheel_radius

        # Convert from rad/s to RPM
        left_rpm = -left_wheel_vel * 60 / (2 * 3.14159)
        right_rpm = right_wheel_vel * 60 / (2 * 3.14159)

        # Limit RPM to motor capabilities
        self.current_left_vel = int(self.clamp(left_rpm, -self.max_rpm, self.max_rpm))
        self.current_right_vel = int(self.clamp(right_rpm, -self.max_rpm, self.max_rpm))

        self.cmd_received = True

        # Log significant velocity changes (optional, for debugging)
        if abs(linear_vel) > 0.1 or abs(angular_vel) > 0.1 or True:
            self.get_logger().info(
                f"cmd_vel: linear={msg.linear.x} angular={msg.angular.z} "
                f"=> used: linear={linear_vel:.2f} angular={angular_vel:.2f} "
                f"=> RPM: L={self.current_left_vel} R={self.current_right_vel}"
            )

    def timer_callback(self):
        """
        Improved timer callback with timeout handling and smooth control
        """
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9

        # print("Corrientes: ", self.motors.get_motor_current())

        # Check for command timeout
        if time_since_last_cmd > self.cmd_timeout:
            if self.moving:
                self.get_logger().info("cmd_vel timeout - stopping motors")
                self.cmd_received = False
                self.moving = False
                # Si disable_motor() es llamada muy seguido muy rapido (<50ms) no se logra desactivar
                self.motors.disable_motor()
            self.current_left_vel = 0
            self.current_right_vel = 0

        else:
            try:
                if self.cmd_received:
                    self.motors.enable_motor()
                    self.motors.set_mode(3)
                    self.get_logger().info(
                        f"RPM: L={self.current_left_vel} R={self.current_right_vel}"
                    )

                    self.motors.set_rpm(self.current_left_vel, self.current_right_vel)
                    self.moving = True
                    self.cmd_received = False
            except Exception as e:
                self.get_logger().error(f"Motor control error: {e}")

        # # Small delay to prevent overwhelming the motor controller
        # time.sleep(0.01)

    def clamp(self, value, min_val, max_val):
        """
        Utility function to clamp a value between min and max
        """
        return max(min_val, min(value, max_val))

    # Control command handling has been moved to the qubot_walk node
    # which publishes Twist messages to /cmd_vel


def main(args=None):
    try:
        rclpy.init(args=args)
        zlac8015d_bridge = zlac8015dBridge()
        rclpy.spin(zlac8015d_bridge)
    except KeyboardInterrupt:
        print("Shutting down zlac8015d_bridge node...")
    except Exception as e:
        print(f"Error in zlac8015d_bridge: {e}")
    finally:
        if "zlac8015d_bridge" in locals():
            zlac8015d_bridge.destroy_node()
        rclpy.shutdown()
        pre_movs.detener()


if __name__ == "__main__":
    main()
