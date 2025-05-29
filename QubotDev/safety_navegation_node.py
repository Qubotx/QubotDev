#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String
import math
import time
from collections import deque


class SafetyNavigationNode(Node):
    def __init__(self):
        super().__init__("safety_navigation_node")

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.control_cmd_publisher = self.create_publisher(String, "/control_cmd", 10)

        # Subscribers
        self.cmd_vel_safe_subscriber = self.create_subscription(
            Twist, "/cmd_vel_safe", self.cmd_vel_safe_callback, 10
        )

        # Sensor subscribers
        self.sensor_subscribers = {
            "izq_back": self.create_subscription(
                Range,
                "/sensor/izq_back",
                lambda msg: self.sensor_callback(msg, "izq_back"),
                10,
            ),
            "izq_front": self.create_subscription(
                Range,
                "/sensor/izq_front",
                lambda msg: self.sensor_callback(msg, "izq_front"),
                10,
            ),
            "der_front": self.create_subscription(
                Range,
                "/sensor/der_front",
                lambda msg: self.sensor_callback(msg, "der_front"),
                10,
            ),
            "der_back": self.create_subscription(
                Range,
                "/sensor/der_back",
                lambda msg: self.sensor_callback(msg, "der_back"),
                10,
            ),
            "lidar": self.create_subscription(
                Range,
                "/sensor/lidar",
                lambda msg: self.sensor_callback(msg, "lidar"),
                10,
            ),
        }

        # Sensor configuration with angles in radians
        self.sensor_config = {
            "izq_back": {
                "angle": math.radians(225),
                "data": deque(),
                "last_update": None,
            },  # back-left
            "izq_front": {
                "angle": math.radians(315),
                "data": deque(),
                "last_update": None,
            },  # front-left
            "der_front": {
                "angle": math.radians(45),
                "data": deque(),
                "last_update": None,
            },  # front-right
            "der_back": {
                "angle": math.radians(135),
                "data": deque(),
                "last_update": None,
            },  # back-right
            "lidar": {
                "angle": math.radians(0),
                "data": deque(),
                "last_update": None,
            },  # front
        }

        # Timing parameters
        self.data_timeout = 2.5  # 2.5s - sensor data timeout
        self.averaging_window = 1.5  # 1s - averaging window

        self.dt = 0.100  # 100ms - time step
        self.prediction_time = 0.300  # 300ms - collision prediction time

        # Safety parameters
        self.min_safe_distance = 0.3  # meters - minimum safe distance
        self.robot_radius = 0.46  # meters - robot radius for collision detection

        # State management
        self.last_cmd_vel = None
        self.is_paused = False

        # Timer for sensor data cleanup
        self.cleanup_timer = self.create_timer(
            0.05, self.cleanup_old_sensor_data
        )  # 50ms cleanup interval

        # Timer for re-evaluating paused commands
        self.reevaluation_timer = self.create_timer(
            0.1, self.reevaluate_paused_command
        )  # 100ms re-evaluation

        self.get_logger().info("Enhanced Safety Navigation Node initialized")

    def sensor_callback(self, msg, sensor_name):
        """Update sensor data with timestamp"""
        current_time = time.time()
        sensor_info = self.sensor_config[sensor_name]

        # Add new data point with timestamp
        sensor_info["data"].append((current_time, msg.range))
        sensor_info["last_update"] = current_time

        # Remove data older than averaging window
        cutoff_time = current_time - self.averaging_window
        while sensor_info["data"] and sensor_info["data"][0][0] < cutoff_time:
            sensor_info["data"].popleft()

    def cleanup_old_sensor_data(self):
        """Mark sensor data as None if no recent updates"""
        current_time = time.time()
        for sensor_name, sensor_info in self.sensor_config.items():
            if (
                sensor_info["last_update"] is None
                or current_time - sensor_info["last_update"] > self.data_timeout
            ):
                # Clear old data and mark as unavailable
                sensor_info["data"].clear()
                sensor_info["last_update"] = None

    def get_averaged_sensor_data(self, sensor_name):
        """Get averaged sensor data for the last 100ms or None if unavailable"""
        sensor_info = self.sensor_config[sensor_name]

        if not sensor_info["data"] or sensor_info["last_update"] is None:
            return None

        # Check if data is recent enough
        current_time = time.time()
        if current_time - sensor_info["last_update"] > self.data_timeout:
            return None

        # Calculate average of recent data
        ranges = [data_point[1] for data_point in sensor_info["data"]]
        return sum(ranges) / len(ranges)

    def cmd_vel_safe_callback(self, msg):
        """Process incoming safe command velocity messages"""
        self.last_cmd_vel = msg

        # Check if movement is safe using geometric prediction
        if self.is_movement_safe_geometric(msg):
            # Safe to move - publish command and resume if paused
            self.cmd_vel_publisher.publish(msg)

            if self.is_paused:
                self.publish_control_command("resume")
                self.is_paused = False
                self.get_logger().info("Movement resumed - predicted path clear")
        else:
            # Collision predicted - pause movement
            halt_msg = Twist()
            self.cmd_vel_publisher.publish(halt_msg)

            if not self.is_paused:
                self.publish_control_command("pause")
                self.is_paused = True
                self.get_logger().warn("Movement paused - collision predicted")

    def reevaluate_paused_command(self):
        """
        Periodically re-evaluate the last command when paused to check if path is clear
        """
        # Only re-evaluate if we're currently paused and have a command to re-evaluate
        if self.is_paused and self.last_cmd_vel is not None:
            # Check if the last command is now safe to execute
            if self.is_movement_safe_geometric(self.last_cmd_vel):
                # Path is now clear - resume movement
                self.cmd_vel_publisher.publish(self.last_cmd_vel)
                self.publish_control_command("resume")
                self.is_paused = False
                self.get_logger().info(
                    "Movement automatically resumed - obstacle cleared"
                )

    def is_movement_safe_geometric(self, cmd_vel):
        """
        Use geometric calculations to predict collisions based on robot trajectory
        """
        # If no movement requested, it's safe
        if (
            abs(cmd_vel.linear.x) < 0.01
            and abs(cmd_vel.linear.y) < 0.01
            and abs(cmd_vel.angular.z) < 0.01
        ):
            return True

        # Calculate robot's predicted trajectory
        trajectory_points = self.calculate_trajectory(cmd_vel, self.prediction_time)

        # Check each trajectory point against all sensors
        for point in trajectory_points:
            if not self.is_point_safe(point):
                return False

        return True

    def calculate_trajectory(self, cmd_vel, duration):
        """
        Calculate robot trajectory points for the given duration
        Returns list of (x, y, theta) positions
        """
        trajectory_points = []
        dt = self.dt
        steps = int(duration / dt)

        # Current position (robot starts at origin with 0 orientation)
        x, y, theta = 0.0, 0.0, 0.0

        for step in range(steps + 1):
            t = step * dt

            if abs(cmd_vel.angular.z) < 0.001:
                # Pure linear motion
                x = cmd_vel.linear.x * t
                y = cmd_vel.linear.y * t
                theta = 0.0
            else:
                # Motion with rotation (circular arc)
                omega = cmd_vel.angular.z

                # Calculate position on circular arc
                if abs(cmd_vel.linear.x) > 0.001 or abs(cmd_vel.linear.y) > 0.001:
                    # Combined linear and angular motion
                    radius = math.sqrt(
                        cmd_vel.linear.x**2 + cmd_vel.linear.y**2
                    ) / abs(omega)
                    arc_angle = omega * t

                    x = radius * math.sin(arc_angle)
                    y = (
                        radius * (1 - math.cos(arc_angle))
                        if omega > 0
                        else radius * (math.cos(arc_angle) - 1)
                    )
                    theta = arc_angle
                else:
                    # Pure rotation
                    x, y = 0.0, 0.0
                    theta = omega * t

            trajectory_points.append((x, y, theta))

        return trajectory_points

    def is_point_safe(self, robot_position):
        """
        Check if a robot position (x, y, theta) is safe given current sensor readings
        """
        x, y, theta = robot_position

        # Check each sensor
        for sensor_name, sensor_info in self.sensor_config.items():
            sensor_range = self.get_averaged_sensor_data(sensor_name)

            # Skip if no sensor data available
            if sensor_range is None:
                self.get_logger().info(
                    f"No data from sensor {sensor_name}, assuming unsafe"
                )
                # return False
                continue
            else:
                self.get_logger().info(
                    f"Data from sensor {sensor_name}: {sensor_range}m"
                )

            # Calculate sensor position in robot frame
            # Sensor angle relative to world
            sensor_angle = sensor_info["angle"] + theta
            # Assume sensors are at the borders of the robot
            sensor_x = x + self.robot_radius * math.cos(sensor_angle)
            sensor_y = y + self.robot_radius * math.sin(sensor_angle)

            # Calculate obstacle position based on sensor reading
            obstacle_x = sensor_x + sensor_range * math.cos(sensor_angle)
            obstacle_y = sensor_y + sensor_range * math.sin(sensor_angle)

            # Check if obstacle intersects with robot's predicted position
            distance_to_robot_center = math.sqrt(
                (obstacle_x - x) ** 2 + (obstacle_y - y) ** 2
            )

            if distance_to_robot_center < (self.robot_radius + self.min_safe_distance):
                self.get_logger().info(
                    f"Collision predicted with obstacle detected by {sensor_name}"
                )
                self.get_logger().info(
                    f"Robot pos: ({x:.2f}, {y:.2f}), Obstacle pos: ({obstacle_x:.2f}, {obstacle_y:.2f})"
                )
                self.get_logger().info(
                    f"Distance: {distance_to_robot_center:.2f}m, Required: {self.robot_radius + self.min_safe_distance:.2f}m"
                )
                return False

        return True

    def publish_control_command(self, command):
        """Publish control command (pause/resume)"""
        msg = String()
        msg.data = command
        self.control_cmd_publisher.publish(msg)
        self.get_logger().info(f"Published control command: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
