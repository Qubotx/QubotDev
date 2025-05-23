import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from QubotDev.pwm_neck_control import set_neck_angle, rotate_neck_relative


class NeckServoBridge(Node):
    def __init__(self):
        super().__init__("neck_servo_bridge")
        self.get_logger().info("Neck Servo Bridge node started")
        # Subscribe to the desired angle topic (in degrees)
        self.subscription = self.create_subscription(
            Float32, "/neck_servo/angle", self.angle_callback, 1
        )
        # Subscribe to control commands for neck
        self.control_cmd_sub = self.create_subscription(
            String, "/control_cmd", self.control_cmd_callback, 1
        )

    def angle_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f"Received neck angle command: {angle}")
        set_neck_angle(angle)

    def control_cmd_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f"Received control command: {command}")
        if command == "rotate-neck-left":
            self.get_logger().info("Rotating neck left by 5 degrees...")
            rotate_neck_relative(5)
        elif command == "center-neck":
            self.get_logger().info("Centering neck...")
            set_neck_angle(90)
        elif command == "rotate-neck-right":
            self.get_logger().info("Rotating neck right by 5 degrees...")
            rotate_neck_relative(-5)


def main(args=None):
    rclpy.init(args=args)
    node = NeckServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
